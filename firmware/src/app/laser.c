/*
 * laser.c - laser line finder: LASR mailbox, profile block, calibration
 *
 * Part of net_stack.c until 24.09.2026.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "memory_map.h"
#include "mailbox.h"
#include "laser.h"
#include "work_mode.h"
#include "cache.h"
#include "platform.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"



// ===========================================================================
// Laser line finder: a frame for the algorithm
// ===========================================================================
//
// Design: docs/ARCHITECTURE.md § 6. Everything except the algorithm itself is
// gathered here: the profile block layout, the mailbox, telemetry, calibration
// and the mode state machine. The algorithm enters in two places marked
// "HOOK POINT", and nowhere else.

// The block layout must match tools/laser/laser_block.py and the RTL, when it
// appears. Compile-time check: otherwise a frame geometry change
// (VIDEO_FRAME_HEIGHT, row stride) would silently run the profile block over
// the frame or push it out of the slot, and that would be found only on the
// board.
typedef char laser_layout_check[
    (sizeof(laser_rec_t) == LASER_REC_BYTES &&
     LASER_FOOT_OFF == 0x5100U && LASER_PTS_OFF == 0x5200U && LASER_MM_OFF == 0x6600U &&
     LASER_BLOCK_BYTES == 0x7A00U &&
     (VIDEO_PIX_BASE_OFF + (uint32_t)VIDEO_FRAME_HEIGHT * VIDEO_ROW_STRIDE) <= LASER_BLOCK_OFF &&
     (LASER_BLOCK_OFF + LASER_BLOCK_BYTES) <= VIDEO_SLOT_BYTES &&
     // the dense ring: the block fits its 32 KB slot, 8192 slots = cam_channel.v DRY_SLOT_BITS 13,
     // 256 MB that do not reach the other camera's ring
     (LASER_DRY_BLOCK_OFF + LASER_BLOCK_BYTES) <= LASER_DRY_SLOT_BYTES &&
     LASER_DRY_SLOTS == 8192U &&
     DENSE_RING0_ADDR + LASER_DRY_SLOTS * LASER_DRY_SLOT_BYTES <= DENSE_RING1_ADDR) ? 1 : -1];

// ---- Finder settings: mailbox shadows, blocks 32..40 and 41..49 --------------

// The shadow default is LASER_CFG_DEFAULT_INIT in laser.h, the same for both cameras.
static laser_cfg_t s_lcfg[LASER_CAM_COUNT] = { LASER_CFG_DEFAULT_INIT, LASER_CFG_DEFAULT_INIT };
static uint32_t s_lasr_apply_seq[LASER_CAM_COUNT] = { 0, 0 };
static bool     s_lasr_init = false;
// Frame without rows (LASR_CTRL_NO_FRAME): only for the duration of the profile
// stream, not part of the settings and not saved; after a reboot the frame is
// always written.
static bool     s_no_frame[LASER_CAM_COUNT] = { false, false };

static uint8_t laser_cam(uint8_t cam)
{
    return (cam < LASER_CAM_COUNT) ? cam : 0U;
}

void laser_cfg_get(uint8_t cam, laser_cfg_t *cfg)
{
    if (cfg) *cfg = s_lcfg[laser_cam(cam)];
}

// Shadow only, no mailbox write: the settings storage puts it here, and it goes
// into the mailbox with one laser_cfg_write() (work_mode.c).
void laser_cfg_shadow_set(uint8_t cam, const laser_cfg_t *cfg)
{
    if (cfg) s_lcfg[laser_cam(cam)] = *cfg;
}

// Continue the command counters from what is in memory: DDR3 keeps the mailbox
// across a CPU reboot, and the FPGA has already seen it (the same reason as in
// sync_init()).
static void laser_cfg_init(void)
{
    uint8_t c;

    if (s_lasr_init) return;
    mailbox_load(256);
    for (c = 0; c < LASER_CAM_COUNT; ++c)
        if (mailbox_get(LASR_W_BASE(c) + LASR_O_MAGIC) == LASR_MAGIC)
            s_lasr_apply_seq[c] = mailbox_get(LASR_W_BASE(c) + LASR_O_APPLY_SEQ);
    s_lasr_init = true;
}

// System mode for the LASR control word. Set by work_mode_set() at the same
// point where the mode itself changes; the initial value is the same as the mode's.
static uint8_t s_lasr_mode = WORK_MODE_STANDBY;     /* since 02.10.2026, work_mode.c */

void laser_set_mode(uint8_t mode)
{
    s_lasr_mode = mode;
}

static uint32_t laser_ctrl_word(uint8_t c)
{
    const laser_cfg_t *k = &s_lcfg[c];
    uint32_t ctrl = (uint32_t)s_lasr_mode & LASR_CTRL_MODE_MASK;

    ctrl |= ((uint32_t)k->axis << LASR_CTRL_AXIS_SHIFT) & LASR_CTRL_AXIS_MASK;
    if (k->flags & LASER_CFG_F_KEEP_FRAME) ctrl |= LASR_CTRL_KEEP_FRAME;
    if (k->flags & LASER_CFG_F_WRITE_PROF) ctrl |= LASR_CTRL_WRITE_PROF;
    if (k->flags & LASER_CFG_F_SOFT)       ctrl |= LASR_CTRL_SOFT;
    if (s_no_frame[c])                     ctrl |= LASR_CTRL_NO_FRAME;
    return ctrl;
}

// One camera's block: magic and parameters, then the APPLY_SEQ counter, the
// mailbox.h order. The FPGA applies the block when it reads APPLY_SEQ.
static void laser_cfg_write_cam(uint8_t c)
{
    const laser_cfg_t *k = &s_lcfg[c];
    uint32_t b = LASR_W_BASE(c);

    mailbox_put(b + LASR_O_MAGIC,     LASR_MAGIC);
    mailbox_put(b + LASR_O_CTRL,      laser_ctrl_word(c));
    mailbox_put(b + LASR_O_THRESH,    (uint32_t)k->thresh | ((uint32_t)k->offset << 8));
    mailbox_put(b + LASR_O_GAP,       k->gap);
    mailbox_put(b + LASR_O_MIN_SUM,   k->min_sum);
    mailbox_put(b + LASR_O_WIN_LO,    k->win_lo);
    mailbox_put(b + LASR_O_WIN_HI,    k->win_hi);
    mailbox_put(b + LASR_O_WIDTH_MAX, k->width_max);
    mailbox_publish(b + LASR_O_APPLY_SEQ, ++s_lasr_apply_seq[c]);
}

void laser_cfg_write(void)
{
    uint8_t c;

    laser_cfg_init();
    for (c = 0; c < LASER_CAM_COUNT; ++c) laser_cfg_write_cam(c);
}

// Value check: the same for 0x0150 and for an image from the store.
int laser_cfg_check(const laser_cfg_t *cfg)
{
    if (cfg == NULL) return 6;                       /* VCP_ST_BAD_LENGTH */
    if (cfg->axis > LASER_AXIS_COL) return 9;        /* VCP_ST_DENIED     */
    if (cfg->thresh == 0U) return 9;
    // The floor is not above the threshold: otherwise a spot pixel's contribution
    // is negative (tools/laser/model.py extract_raw rejects the same).
    if (cfg->offset > cfg->thresh) return 9;
    if (cfg->gap == 0U || cfg->gap > 15U) return 9;
    if (cfg->win_hi != 0U && cfg->win_hi <= cfg->win_lo) return 9;
    // The window is set along the search axis: with LASER_AXIS_ROW it is a pixel in
    // the row (0..1079), with LASER_AXIS_COL a row number (0..1279).
    if (cfg->win_hi >= ((cfg->axis == LASER_AXIS_ROW) ? VIDEO_FRAME_WIDTH
                                                      : VIDEO_FRAME_HEIGHT)) return 9;
    return 0;
}

int laser_cfg_set(uint8_t cam, const laser_cfg_t *cfg)
{
    int st = laser_cfg_check(cfg);
    uint8_t c = laser_cam(cam);
    const laser_cfg_t *k = &s_lcfg[c];

    if (st != 0) return st;
    if (cam >= LASER_CAM_COUNT) return 9;            /* VCP_ST_DENIED */
    s_lcfg[c] = *cfg;
    laser_cfg_init();
    laser_cfg_write_cam(c);
    printf("[LASER] cam %u cfg: axis %u thresh %u offset %u gap %u win %u..%u min_sum %u"
           " flags 0x%02X (seq %u)\r\n", (unsigned int)c,
           (unsigned int)k->axis, (unsigned int)k->thresh, (unsigned int)k->offset,
           (unsigned int)k->gap, (unsigned int)k->win_lo, (unsigned int)k->win_hi,
           (unsigned int)k->min_sum, (unsigned int)k->flags,
           (unsigned int)s_lasr_apply_seq[c]);
    return 0;
}

bool laser_read_telemetry(uint8_t cam, laser_telemetry_t *t)
{
    uint32_t base = VIDEO_DIAG_ADDR(laser_cam(cam));
    volatile uint32_t *d = (volatile uint32_t *)base;
    uint32_t e, b;

    if (t == NULL) return false;
    memset(t, 0, sizeof(*t));
    ae350_dcache_invalidate_range(base, 256);
    // Builds before L2a have no finder words (word 36 is the filler 0xA5A55A5A).
    // That is not an error: the host must see the difference between "no block" and
    // "the block does not answer".
    e = d[LASR_T_ECHO];
    b = d[LASR_T_BLOCKS];
    t->valid = (d[0] == VIDEO_FRAME_MAGIC) && ((e >> 24) == LASR_T_ID_BYTE);
    if (!t->valid) return false;
    t->apply_seq = (uint8_t)(e >> 16);
    t->loaded    = (e & (1U << 15)) != 0U;
    t->enable    = (e & (1U << 14)) != 0U;
    t->no_frame  = (e & (1U << 13)) != 0U;   /* bitstreams before stage 2: always 0 */
    // Bit 12 means the 8192-slot ring of 20.10.2026 (LASER_DRY_*): a bitstream of
    // 19.10 with the 2048-slot ring gives this firmware no blocks, never wrong ones.
    t->dense     = (e & (1U << 12)) != 0U;
    t->gap       = (uint8_t)((e >> 8) & 0x0FU);
    t->thresh    = (uint8_t)e;
    t->blocks    = (uint16_t)(b >> 16);
    t->no_line   = (uint16_t)b;
    return true;
}

// FPGA confirmation: each camera in the mask reflects the last written
// APPLY_SEQ (low byte) in telemetry. Telemetry is written every 0.5 s, so the
// wait is up to ~0.5 s, not the mailbox poll period.
// -1 = the FPGA has no finder (build before L2a), 1 = confirmed, 0 = not in time.
int laser_fpga_ack(uint8_t cam_mask, uint32_t timeout_ms)
{
    laser_telemetry_t t;
    uint32_t waited = 0;
    uint8_t c;

    for (;;)
    {
        bool all = true;

        for (c = 0; c < LASER_CAM_COUNT; ++c)
        {
            if (!(cam_mask & (1U << c))) continue;
            if (!laser_read_telemetry(c, &t))
            {
                if (c == 0U) return -1;
                all = false;
                continue;
            }
            if (t.apply_seq != (uint8_t)s_lasr_apply_seq[c]) all = false;
        }
        if (all) return 1;
        if (waited >= timeout_ms) return 0;
        vTaskDelay(pdMS_TO_TICKS(20));
        waited += 20U;
    }
}

// ---- Profile block in the slot -----------------------------------------------
//
// The cache operation is flush, not invalidate: the block is written either by
// the FPGA or by the CPU itself (laser_profile_build), and invalidate on memory
// the CPU wrote throws away its not yet written-back lines (CLAUDE.md
// invariant 4, ERRORS former 31.37). The whole block at once, in 8 KB pieces:
// 256 CCTL operations per piece is the limit beyond which a ranged flush becomes
// dangerous (it runs with interrupts disabled, ERRORS 5.2, former 30.2). The
// 21 KB block is 656 operations, and they must not be done in one call. It is
// flushed here, not in laser_block_rec(): reading a record must not touch the
// cache at all, otherwise 1280 records would mean 1280 calls.
static void laser_block_sync_cache(uint32_t slot)
{
    uint32_t base = LASER_BLOCK_ADDR(slot);
    uint32_t off;

    for (off = 0; off < LASER_BLOCK_BYTES; off += 8192U)
    {
        uint32_t chunk = (LASER_BLOCK_BYTES - off > 8192U) ? 8192U : (LASER_BLOCK_BYTES - off);
        ae350_dcache_flush_range(base + off, chunk);
    }
}

// Version 3 header words 13..18 (R-38). The layout word is checked like the points
// one: a foreign format gives "no mm", not misread mm.
static void laser_block_mm_hdr(volatile uint32_t *h, laser_block_hdr_t *hdr)
{
    uint32_t st;
    bool crc = (hdr->kind_flags & LASER_KIND_F_CRC) != 0U;

    hdr->crc_pts = crc ? h[LASER_H_CRC_PTS] : 0U;
    hdr->crc_mmw = crc ? h[LASER_H_CRC_MMW] : 0U;
    hdr->has_mm = (hdr->version >= 3U) && hdr->has_points &&
                  (h[LASER_H_MM] == ((LASER_MM_OFF << 16) | LASER_PT_BYTES));
    hdr->mm_on = hdr->mm_bad = false;
    hdr->mm_id = hdr->mm_crc = 0U;
    hdr->mm_points = 0U;
    if (!hdr->has_mm) return;
    st = h[LASER_H_MM_STATE];
    hdr->mm_on     = (st >> 31) != 0U;
    hdr->mm_bad    = ((st >> 30) & 1U) != 0U;
    hdr->mm_points = (uint16_t)st;
    hdr->mm_id     = h[LASER_H_MM_ID];
    hdr->mm_crc    = h[LASER_H_MM_CRC];
}

bool laser_block_valid(uint32_t slot, uint32_t frame_id, laser_block_hdr_t *hdr)
{
    volatile uint32_t *h = (volatile uint32_t *)LASER_BLOCK_ADDR(slot);
    volatile uint32_t *f = (volatile uint32_t *)(LASER_BLOCK_ADDR(slot) + LASER_FOOT_OFF);

    laser_block_sync_cache(slot);

    if (h[LASER_H_MAGIC] != LASER_BLOCK_MAGIC) return false;
    if (h[LASER_H_FRAME_ID] != frame_id) return false;
    // The final burst is written last: it is the "block finished" marker.
    if (f[0] != frame_id || f[63] != VIDEO_HW_HDR_INTEGRITY) return false;

    if (hdr != NULL)
    {
        uint32_t kind = h[LASER_H_KIND];
        uint32_t geom = h[LASER_H_GEOM];
        uint32_t rows = h[LASER_H_ROWS];

        hdr->frame_id     = h[LASER_H_FRAME_ID];
        hdr->timestamp    = h[LASER_H_TIMESTAMP];
        hdr->strobe_ts    = h[LASER_H_STROBE_TS];
        hdr->axis         = (uint8_t)(kind & 0xFFU);
        hdr->version      = (uint8_t)((kind >> 8) & 0xFFU);
        hdr->cam_id       = (uint8_t)((kind >> 16) & 0xFFU);
        hdr->kind_flags   = (uint8_t)((kind >> 24) & 0xFFU);
        hdr->points       = (uint16_t)(geom >> 16);
        hdr->rows_used    = (uint16_t)(rows >> 16);
        hdr->rows_no_line = (uint16_t)(rows & 0xFFFFU);
        hdr->rows_dropped = h[LASER_H_DROPPED];
        if (hdr->points > LASER_POINTS_MAX) hdr->points = LASER_POINTS_MAX;
        // Version 2 from the FPGA: the points lie after the final burst, the CPU does
        // not need to compute them. The offset and size are checked against the layout:
        // a foreign format is better recomputed than parsed wrongly.
        hdr->has_points   = (hdr->version >= 2U) &&
                            ((hdr->kind_flags & LASER_KIND_F_FPGA) != 0U) &&
                            (h[LASER_H_PTS] == ((LASER_PTS_OFF << 16) | LASER_PT_BYTES));
        laser_block_mm_hdr(h, hdr);
    }
    return true;
}

// The cache is not touched here: laser_block_valid() flushed it, and it is
// called before any read of records. Records must not be read apart from it:
// they could come from cache left over from the previous frame in this slot.
bool laser_block_rec(uint32_t slot, uint32_t idx, laser_rec_t *rec)
{
    if (rec == NULL || idx >= LASER_POINTS_MAX) return false;
    memcpy(rec, (const void *)LASER_REC_ADDR(slot, idx), sizeof(*rec));
    return true;
}

bool laser_block_point(uint32_t slot, uint32_t idx, laser_point_t *pt)
{
    uint32_t w;

    if (pt == NULL || idx >= LASER_POINTS_MAX) return false;
    w = *(volatile const uint32_t *)LASER_PT_ADDR(slot, idx);
    pt->x_sub = (uint16_t)(w & 0xFFFFU);
    pt->z     = 0U;
    pt->i     = (uint8_t)((w >> 16) & 0xFFU);
    pt->q     = (uint8_t)((w >> 24) & 0xFFU);
    return true;
}

// ---- Profile stream (stage 2, 01.10.2026) -------------------------------------
//
// Only the FPGA writes the block in the slot (laser_profile_build is a stub and
// writes nothing), but the cache operation is still flush, not invalidate, for
// the same reason as in laser_block_sync_cache(): laser_block_erase()
// writes here from the CPU itself.

bool laser_block_peek(uint32_t blk, laser_block_hdr_t *hdr)
{
    uint32_t base = blk;
    volatile uint32_t *h = (volatile uint32_t *)base;
    volatile uint32_t *f = (volatile uint32_t *)(base + LASER_FOOT_OFF);
    uint32_t fid, kind;

    ae350_dcache_flush_range(base, 128);                 /* words 0..18 (version 3) */
    ae350_dcache_flush_range(base + LASER_FOOT_OFF, 256);
    if (h[LASER_H_MAGIC] != LASER_BLOCK_MAGIC) return false;
    fid = h[LASER_H_FRAME_ID];
    if (f[0] != fid || f[63] != VIDEO_HW_HDR_INTEGRITY) return false;
    kind = h[LASER_H_KIND];
    // Only an FPGA version 2 block with the points in place: the stream does not compute.
    if (((kind >> 8) & 0xFFU) < 2U || !((kind >> 24) & LASER_KIND_F_FPGA) ||
        h[LASER_H_PTS] != ((LASER_PTS_OFF << 16) | LASER_PT_BYTES)) return false;
    if (hdr != NULL)
    {
        uint32_t geom = h[LASER_H_GEOM];
        uint32_t rows = h[LASER_H_ROWS];

        hdr->frame_id     = fid;
        hdr->timestamp    = h[LASER_H_TIMESTAMP];
        hdr->strobe_ts    = h[LASER_H_STROBE_TS];
        hdr->axis         = (uint8_t)(kind & 0xFFU);
        hdr->version      = (uint8_t)((kind >> 8) & 0xFFU);
        hdr->cam_id       = (uint8_t)((kind >> 16) & 0xFFU);
        hdr->kind_flags   = (uint8_t)((kind >> 24) & 0xFFU);
        hdr->points       = (uint16_t)(geom >> 16);
        hdr->rows_used    = (uint16_t)(rows >> 16);
        hdr->rows_no_line = (uint16_t)(rows & 0xFFFFU);
        hdr->rows_dropped = h[LASER_H_DROPPED];
        hdr->has_points   = true;
        if (hdr->points > LASER_POINTS_MAX) hdr->points = LASER_POINTS_MAX;
        laser_block_mm_hdr(h, hdr);
    }
    return true;
}

// Measurement (01.10.2026, ERRORS 1.15): reading the points takes 571 us per
// profile, the main cost of the stream. Split into the cache flush and the copy,
// MTIME clocks (10 ns).
volatile uint64_t g_laser_pt_t_flush = 0, g_laser_pt_t_copy = 0;

void laser_block_points_read(uint32_t blk, uint32_t n, uint32_t *dst)
{
    uint32_t addr = blk + LASER_PTS_OFF;
    uint32_t t0, t1;

    if (n > LASER_POINTS_MAX) n = LASER_POINTS_MAX;
    // 1080 points = 4320 B = 135 CCTL operations: under the limit of 256
    // (ERRORS 5.2, former 30.2).
    t0 = DEV_PLMT->MTIME[0];
    ae350_dcache_flush_range(addr, n * LASER_PT_BYTES);
    t1 = DEV_PLMT->MTIME[0];
    memcpy(dst, (const void *)addr, n * LASER_PT_BYTES);
    g_laser_pt_t_flush += t1 - t0;
    g_laser_pt_t_copy  += DEV_PLMT->MTIME[0] - t1;
}

void laser_block_mm_read(uint32_t blk, uint32_t n, uint32_t *dst)
{
    uint32_t addr = blk + LASER_MM_OFF;
    uint32_t t0, t1;

    if (n > LASER_POINTS_MAX) n = LASER_POINTS_MAX;
    t0 = DEV_PLMT->MTIME[0];
    ae350_dcache_flush_range(addr, n * 4U);
    t1 = DEV_PLMT->MTIME[0];
    memcpy(dst, (const void *)addr, n * 4U);
    g_laser_pt_t_flush += t1 - t0;
    g_laser_pt_t_copy  += DEV_PLMT->MTIME[0] - t1;
}

bool laser_block_still(uint32_t blk, uint32_t frame_id)
{
    volatile uint32_t *h = (volatile uint32_t *)blk;

    ae350_dcache_flush_range(blk, 32);
    return h[LASER_H_MAGIC] == LASER_BLOCK_MAGIC && h[LASER_H_FRAME_ID] == frame_id;
}

void laser_block_erase(uint32_t blk)
{
    volatile uint32_t *h = (volatile uint32_t *)blk;
    volatile uint32_t *f = (volatile uint32_t *)(blk + LASER_FOOT_OFF);

    h[LASER_H_MAGIC] = 0U;
    f[0]  = 0U;
    f[63] = 0U;
    ae350_dcache_flush_range(blk, 32);
    ae350_dcache_flush_range(blk + LASER_FOOT_OFF, 256);
}

void laser_no_frame_post(uint8_t cam_mask, bool on)
{
    uint8_t c;

    laser_cfg_init();
    for (c = 0; c < LASER_CAM_COUNT; ++c)
    {
        if (!(cam_mask & (1U << c)) || s_no_frame[c] == on) continue;
        s_no_frame[c] = on;
        laser_cfg_write_cam(c);
    }
}

int laser_no_frame_set(uint8_t cam_mask, bool on, uint32_t timeout_ms)
{
    laser_telemetry_t t;
    uint32_t waited = 0;
    uint8_t c;

    laser_cfg_init();
    for (c = 0; c < LASER_CAM_COUNT; ++c)
    {
        if (!(cam_mask & (1U << c))) continue;
        s_no_frame[c] = on;
        laser_cfg_write_cam(c);
    }
    printf("[LASER] no-frame %s for mask 0x%X\r\n", on ? "ON" : "off", (unsigned int)cam_mask);
    // Like laser_fpga_ack(), but also the no_frame bit in the echo: a bitstream
    // before stage 2 reflects APPLY_SEQ but still writes the frame; that must be
    // visible.
    for (;;)
    {
        bool all = true, dense = true;

        for (c = 0; c < LASER_CAM_COUNT; ++c)
        {
            if (!(cam_mask & (1U << c))) continue;
            if (!laser_read_telemetry(c, &t))
            {
                if (c == 0U) return -1;
                all = false;
                continue;
            }
            if (t.apply_seq != (uint8_t)s_lasr_apply_seq[c] || t.no_frame != on) all = false;
            dense = dense && t.dense;
        }
        if (all) return dense ? 2 : 1;
        if (waited >= timeout_ms) return 0;
        vTaskDelay(pdMS_TO_TICKS(20));
        waited += 20U;
    }
}


// ===========================================================================
// HOOK POINT 1: computing the profile block from a frame (reference for RTL checks)
// ===========================================================================
//
// What should appear here (docs/ARCHITECTURE.md § 6):
//   for each row r of the frame in the slot:
//       walk the 1080 pixels of the row (VIDEO_ROW_ADDR(slot, r)) by words,
//       not bytes: the row is 4-aligned, and reading from DDR3 costs ~36 ms per
//       megabyte (ERRORS former 31.44) and is the main expense here;
//       threshold s_lcfg.thresh, spots with gap s_lcfg.gap, pick the spot with
//       the largest sum_i, write laser_rec_t at LASER_REC_ADDR(slot, r);
//   fill the block header, then the final burst LAST;
//   ae350_dcache_flush_all() before reading the frame and after writing the block.
//
// Call under video_ring_lock() with the sensor stopped (the old invariant 5;
// on the AXI4 back end see CLAUDE.md invariant 5).
// Expected cost: 0.1-0.2 s per frame at -Og; measure first.
int laser_profile_build(uint32_t slot, uint32_t frame_id)
{
    (void)slot;
    (void)frame_id;
    // Stub: the block is not written. That is exactly why 0x0210 answers
    // UNSUPPORTED rather than staying silent or returning garbage.
    return -1;
}

// ===========================================================================
// HOOK POINT 2: raw record -> STP point (subpixel and height)
// ===========================================================================
//
// What should appear here (docs/ARCHITECTURE.md § 6, docs/DECISIONS.md R-16):
//   v_cog = sum_iv / sum_i;
//   v_g   = argmax + (ln i_minus - ln i_plus) / (2*(ln i_minus - 2*ln max_i + ln i_plus)),
//           the logarithm from a 256-entry fixed-point table: there is no
//           floating-point hardware (-mcpu=a25);
//   v_g is published if the denominator is not degenerate and max_i < 255,
//   otherwise v_cog;
//   |v_cog - v_g| > 0.5 px -> LASER_Q_DISAGREE;
//   z from the calibration: precomputed a,b,c,d per column, z = (a*v+b)/(c*v+d),
//   in fixed point; without calibration z = 0 and LASER_Q_UNCALIB.
//
// idx is the point number along the profile (with LASER_AXIS_ROW it is the row number).
void laser_refine(const laser_rec_t *rec, uint32_t idx, laser_point_t *pt)
{
    (void)idx;

    if (pt == NULL) return;
    pt->x_sub = 0U;
    pt->z     = 0U;
    pt->i     = (rec != NULL) ? rec->max_i : 0U;
    // The stub is honest both ways: no coordinate, no height. The host sees exactly
    // that and will not take a zero for a measurement.
    pt->q     = LASER_Q_NO_LINE | LASER_Q_UNCALIB;
}


// ---- Calibration (0x0130) -----------------------------------------------------

static uint8_t s_calib[LASER_CALIB_BYTES];
static bool    s_calib_valid = false;

bool laser_calib_valid(void)
{
    return s_calib_valid;
}

int laser_calib_set(const uint8_t *body, uint32_t len)
{
    if (body == NULL || len != LASER_CALIB_BYTES) return 6;   /* VCP_ST_BAD_LENGTH */
    memcpy(s_calib, body, LASER_CALIB_BYTES);
    s_calib_valid = true;
    // Here too, when the algorithm appears, the table of "coordinate -> Z"
    // coefficients per profile point is computed (docs/DECISIONS.md R-22): the work
    // loop must be left with two multiplications and a division, not a
    // floating-point matrix.
    printf("[LASER] calibration stored (%u bytes), coefficient table pending\r\n",
           (unsigned int)len);
    return 0;
}

uint32_t laser_calib_get(uint8_t *out, uint32_t max)
{
    uint32_t n = (max < LASER_CALIB_BYTES) ? max : LASER_CALIB_BYTES;

    if (out == NULL || !s_calib_valid) return 0;
    memcpy(out, s_calib, n);
    return n;
}
