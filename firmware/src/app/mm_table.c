/*
 * mm_table.c - "pixel -> mm" table: upload into DDR3, the band for the ROI, the
 * MMTB mailbox block (16.10.2026, docs/DECISIONS.md R-38). See mm_table.h.
 */

#include "mm_table.h"
#include "spi_flash.h"
#include "mailbox.h"
#include "memory_map.h"
#include "vmsp_protocol.h"     // vmsp_crc32_update(): zlib CRC-32, as the FPGA and the PC
#include "cache.h"
#include "frame_udp_map.h"     // FU_APB_BASE
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// VCP_ST_* (vmsp_service.c): the codes are the protocol's.
#define ST_OK           0x00
#define ST_BUSY         0x02
#define ST_BAD_VERSION  0x04
#define ST_BAD_LENGTH   0x06
#define ST_BAD_CRC      0x07
#define ST_BAD_TARGET   0x08
#define ST_DENIED       0x09

#define MM_CAMS         2U

static SemaphoreHandle_t s_mx;
static mm_info_t         s_st[MM_CAMS];
static uint32_t          s_seq;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// apb_ctl.v (mailbox_on_apb()): the band goes to the FPGA word by word through 0x420, and
// 0x424 says what the loader (lane5_reader.v) wants: [31:29] state, 2 = wants words,
// [28] camera, [23:16] words dropped (written while none was wanted), [10:0] words still
// wanted.
#define MM_APB_BAND     (FU_APB_BASE + 0x420U)
#define MM_APB_LDST     (FU_APB_BASE + 0x424U)
#define MM_LD_FETCH     2U
// Reads of 0x424 before giving up (~0.1-0.2 us each, so 40-80 ms): the loader starts on
// its next mailbox poll, 2^20 FPGA clocks (~10.5 ms) after APPLY_SEQ at most.
#define MM_PUSH_POLLS   400000U

static bool lock(void)
{
    return s_mx != NULL && xSemaphoreTake(s_mx, pdMS_TO_TICKS(500)) == pdTRUE;
}

static void unlock(void)
{
    xSemaphoreGive(s_mx);
}

// With the control blocks on the APB the FPGA does not fetch the band from DDR3: after
// APPLY_SEQ its loader takes the band of each enabled camera, camera 0 first, as words
// the CPU writes, and 2^20 FPGA clocks (~10 ms) without a word end the job with a load
// error (telemetry word 38 [13]). So the CPU waits here for the loader to ask (up to
// one mailbox poll, ~10 ms; the ROI listener too waits that long now) and writes each
// band in one go (~2000 words, ~0.2 ms) with interrupts off. Nothing to do when both
// bands are off - at mm_init() before the scheduler, for one.
static void push_bands_locked(void)
{
    volatile uint32_t *const band = (volatile uint32_t *)MM_APB_BAND;
    volatile uint32_t *const ldst = (volatile uint32_t *)MM_APB_LDST;
    uint32_t need[MM_CAMS], polls = 0, st = 0;
    bool done[MM_CAMS];
    uint8_t c;

    if (!mailbox_on_apb()) return;
    for (c = 0; c < MM_CAMS; ++c) {
        need[c] = (s_st[c].off_why == MM_OFF_NONE) ? (uint32_t)s_st[c].rows * MM_NODE_COLS : 0U;
        done[c] = (need[c] == 0U);
    }
    while (!(done[0] && done[1]) && polls++ < MM_PUSH_POLLS) {
        uint32_t left;
        const uint32_t *src;

        st   = *ldst;
        left = st & 0x7FFU;
        c    = (uint8_t)((st >> 28) & 1U);
        if ((st >> 29) != MM_LD_FETCH || left == 0U || done[c]) continue;
        done[c] = true;
        if (left > need[c]) continue;       // not this band: the job fails on its own
        src = (const uint32_t *)(MM_TABLE_ADDR(c) + (uint32_t)s_st[c].r0 * MM_NODE_COLS * 4U) +
              (need[c] - left);
        taskENTER_CRITICAL();
        while (left-- != 0U) *band = *src++;
        taskEXIT_CRITICAL();
    }
    if (!(done[0] && done[1]))
        printf("[MM] band push: the FPGA did not ask for the band (0x424 = %08X)\r\n", (unsigned int)st);
}

// The MMTB block from the state of both cameras, then the apply counter. Both
// cameras share one counter: the FPGA reloads both bands on every job (a few
// blocks of the other camera go without mm; ROI changes are rare).
static void publish_locked(void)
{
    uint8_t c;

    mailbox_put(MMTB_W_MAGIC, MMTB_MAGIC);
    for (c = 0; c < MM_CAMS; ++c) {
        const mm_info_t *s = &s_st[c];
        bool on = (s->off_why == MM_OFF_NONE);
        mailbox_put(MMTB_W_CAM(c) + 0U, MM_TABLE_ADDR(c) + (uint32_t)s->r0 * MM_NODE_COLS * 4U);
        mailbox_put(MMTB_W_CAM(c) + 1U, s->band_crc);
        mailbox_put(MMTB_W_CAM(c) + 2U, on ? s->hdr.id : 0U);
        mailbox_put(MMTB_W_CAM(c) + 3U, ((uint32_t)(s->kofs & 0xFFFU) << 16) | (uint16_t)s->vofs);
        mailbox_put(MMTB_W_CAM(c) + 4U, ((uint32_t)(s->rows & 0x1FU) << 8) | (on ? 1U : 0U));
    }
    ++s_seq;
    for (c = 0; c < MM_CAMS; ++c) s_st[c].seq = s_seq;
    mailbox_publish(MMTB_W_SEQ, s_seq);
    push_bands_locked();
}

// CRC-32 of table words [first, first + n) as they lie in DDR3. The caller made
// sure the cache holds nothing newer (ae350_dcache_flush_all after the upload).
static uint32_t table_crc(uint8_t cam, uint32_t first, uint32_t n)
{
    return vmsp_crc32_final(vmsp_crc32_update(VMSP_CRC32_INIT,
                                              (const uint8_t *)(MM_TABLE_ADDR(cam) + first * 4U),
                                              n * 4U));
}

// The band of a camera for an ROI; the state is "off" with the reason when there
// is none. Returns true if anything the FPGA uses changed.
static bool band_locked(uint8_t cam, const camera_params_t *roi)
{
    mm_info_t *s = &s_st[cam];
    mm_info_t old = *s;

    if (s->state != MM_ST_READY) {
        s->off_why = MM_OFF_NO_TABLE;
    } else if (roi == NULL) {
        s->off_why = MM_OFF_ROI_BAD;
    } else {
        uint32_t oy = roi->offset_y, h = roi->roi_h, ox = roi->offset_x, w = roi->roi_w;
        uint32_t r0 = oy >> MM_GRID_V_LOG2;
        uint32_t rows = ((oy + h - 1U) >> MM_GRID_V_LOG2) - r0 + 2U;
        // mm_table.py band(): the same rule.
        if (h < 1U || w < 1U || oy + h > VIDEO_FRAME_HEIGHT || ox + w > VIDEO_FRAME_WIDTH) {
            s->off_why = MM_OFF_ROI_BAD;
        } else if (rows > MM_BAND_ROWS_MAX || r0 + rows > MM_NODE_ROWS) {
            s->off_why = MM_OFF_ROI_TALL;
        } else {
            s->off_why  = MM_OFF_NONE;
            s->r0       = (uint8_t)r0;
            s->rows     = (uint8_t)rows;
            s->vofs     = (int16_t)((oy - (r0 << MM_GRID_V_LOG2)) << 4);
            s->kofs     = (uint16_t)ox;
            s->roi_h    = (uint16_t)h;
            s->off_y    = (uint16_t)oy;
            s->band_crc = table_crc(cam, r0 * MM_NODE_COLS, rows * MM_NODE_COLS);
        }
    }
    if (s->off_why != MM_OFF_NONE) {
        s->rows = 0; s->band_crc = 0; s->vofs = 0; s->kofs = 0;
    }
    return s->off_why != old.off_why || s->r0 != old.r0 || s->rows != old.rows ||
           s->vofs != old.vofs || s->kofs != old.kofs || s->band_crc != old.band_crc ||
           s->hdr.id != old.hdr.id;
}

// The last ROI camera.c reported as settled (read back): what a table committed
// later is applied to.
static camera_params_t s_roi[MM_CAMS];
static uint8_t         s_roi_state[MM_CAMS];   /* 0 not reported, 1 known, 2 not what was asked */

// camera.c's ROI listener: under the I2C bus, no waiting.
static void on_roi(uint8_t cam, bool settled, const camera_params_t *roi)
{
    mm_info_t *s;

    if (cam >= MM_CAMS || !lock()) return;
    s = &s_st[cam];
    if (!settled) {
        // Keep the band only if the ROI about to be written is the one it is for.
        if (s->off_why == MM_OFF_NONE && roi != NULL && roi->offset_y == s->off_y &&
            roi->roi_h == s->roi_h && roi->offset_x == s->kofs) {
            unlock();
            return;
        }
        if (s->off_why == MM_OFF_NONE) {
            s->off_why = MM_OFF_CHANGING;
            s->rows = 0; s->band_crc = 0; s->vofs = 0; s->kofs = 0;
            publish_locked();
        } else if (s->state == MM_ST_READY) {
            s->off_why = MM_OFF_CHANGING;
        }
    } else {
        s_roi_state[cam] = (roi != NULL) ? 1U : 2U;
        if (roi != NULL) s_roi[cam] = *roi;
        if (band_locked(cam, roi)) publish_locked();
    }
    unlock();
}

static bool parse_hdr(const uint8_t *body, mm_hdr_t *h)
{
    h->cam        = body[0];
    h->version    = body[1];
    h->gk_log2    = body[2];
    h->gv_log2    = body[3];
    h->cols       = rd16(body + 4);
    h->rows       = rd16(body + 6);
    h->id         = rd32(body + 8);
    h->crc        = rd32(body + 12);
    h->unit_shift = body[16];
    h->frame      = body[17];
    return h->cam < MM_CAMS && h->version == MM_HDR_VERSION && h->gk_log2 == MM_GRID_K_LOG2 &&
           h->gv_log2 == MM_GRID_V_LOG2 && h->cols == MM_NODE_COLS && h->rows == MM_NODE_ROWS;
}

// ---- the table in SPI flash (16.10.2026, docs/DECISIONS.md R-39) ---------------
//
// Two copies per camera, each in its own 64 KB of the flash data area: a 256 B
// header page, the table from +256. A save goes to the copy that does NOT hold the
// newest whole table, the header page is written last, so a power failure in the
// middle leaves the previous table; at start-up the newest copy whose header and
// table CRCs both check out is taken (the same scheme as the store settings).
#define MM_FL_SLOT(c, k)    (SPI_FLASH_DATA_BASE + (2U * (uint32_t)(c) + (uint32_t)(k)) * 0x10000U)
#define MM_FL_DATA_OFF      256U
#define MM_FL_MAGIC         0x4D4D464CU   /* "MMFL" */
#define MM_FL_VERSION       1U

typedef struct {
    uint32_t magic, version, seq;
    uint8_t  hdr[MM_HDR_BYTES];     /* the 0x0134 body, big-endian */
    uint32_t hdr_crc;               /* CRC-32 of the fields above  */
} mm_fl_hdr_t;

static uint8_t  s_fl_mask[MM_CAMS];     /* copies with a whole header (bit k) */
static uint32_t s_fl_seq[MM_CAMS][2];
static uint32_t s_fl_id[MM_CAMS][2];
static int8_t   s_fl_used[MM_CAMS];     /* copy the RAM table came from, -1 none */
static volatile bool s_saving;

static uint32_t fl_hdr_crc(const mm_fl_hdr_t *f)
{
    return vmsp_crc32_final(vmsp_crc32_update(VMSP_CRC32_INIT, (const uint8_t *)f,
                                              offsetof(mm_fl_hdr_t, hdr_crc)));
}

// Headers of both copies of a camera -> s_fl_*; a copy counts only with a whole header.
static void fl_scan(uint8_t cam, mm_fl_hdr_t f[2])
{
    uint8_t k;
    mm_hdr_t h;

    uint8_t try;

    s_fl_mask[cam] = 0;
    for (k = 0; k < 2; ++k) {
        s_fl_seq[cam][k] = 0;
        s_fl_id[cam][k]  = 0;
        // Twice before a copy that carries the magic is taken for damaged: a misread is not
        // a damaged copy (ERRORS 5.15), and a damaged one is an alarm, not the normal case.
        for (try = 0; try < 2; ++try) {
            if (spi_flash_read(MM_FL_SLOT(cam, k), (uint8_t *)&f[k], sizeof(f[k])) != SPI_FLASH_OK) continue;
            if (f[k].magic == MM_FL_MAGIC && f[k].version == MM_FL_VERSION &&
                f[k].hdr_crc == fl_hdr_crc(&f[k]) && parse_hdr(f[k].hdr, &h) && h.cam == cam) break;
        }
        if (try == 2) {
            if (f[k].magic == MM_FL_MAGIC) {
                spi_flash_alarm_set(SPI_FLASH_ALARM_MM(cam));
                printf("[MM] ALARM cam %u: flash copy %u header damaged on two reads\r\n", (unsigned int)cam,
                       (unsigned int)k);
            }
            continue;
        }
        s_fl_mask[cam] |= (uint8_t)(1U << k);
        s_fl_seq[cam][k] = f[k].seq;
        s_fl_id[cam][k]  = h.id;
    }
}

// At start-up: the newest whole copy into DDR3; its table CRC is checked on the
// bytes in DDR3, as for an upload. Before the scheduler and the cameras.
static void fl_load(uint8_t cam)
{
    mm_fl_hdr_t f[2];
    mm_hdr_t h;
    uint8_t order[2], i, k;

    s_fl_used[cam] = -1;
    fl_scan(cam, f);
    order[0] = (s_fl_mask[cam] == 2U || (s_fl_mask[cam] == 3U && s_fl_seq[cam][1] > s_fl_seq[cam][0])) ? 1U : 0U;
    order[1] = order[0] ^ 1U;
    for (i = 0; i < 2; ++i) {
        k = order[i];
        if (!(s_fl_mask[cam] & (1U << k))) continue;
        uint8_t try, ok = 0;
        (void)parse_hdr(f[k].hdr, &h);
        for (try = 0; try < 2 && !ok; ++try) {          // a misread is retried once (ERRORS 5.15)
            if (spi_flash_read(MM_FL_SLOT(cam, k) + MM_FL_DATA_OFF, (uint8_t *)MM_TABLE_ADDR(cam),
                               MM_TABLE_BYTES) != SPI_FLASH_OK)
                continue;
            __asm__ volatile ("fence" ::: "memory");
            ae350_dcache_flush_all();
            ok = (table_crc(cam, 0, MM_TABLE_WORDS) == h.crc);
        }
        if (!ok) {
            spi_flash_alarm_set(SPI_FLASH_ALARM_MM(cam));
            printf("[MM] ALARM cam %u: flash copy %u (seq %u) - table CRC mismatch on two reads, not used\r\n",
                   (unsigned int)cam, (unsigned int)k, (unsigned int)f[k].seq);
            continue;
        }
        if (i != 0U) {
            // the newest copy failed: the table in use is an OLDER calibration
            spi_flash_alarm_set(SPI_FLASH_ALARM_MM(cam));
            printf("[MM] ALARM cam %u: using the OLDER table id %08X (copy %u, seq %u) - the newest did not read back\r\n",
                   (unsigned int)cam, (unsigned int)h.id, (unsigned int)k, (unsigned int)f[k].seq);
        }
        s_st[cam].state    = MM_ST_READY;
        s_st[cam].hdr      = h;
        s_st[cam].received = MM_TABLE_BYTES;
        s_fl_used[cam]     = (int8_t)k;
        printf("[MM] cam %u: table id %08X from flash copy %u (seq %u)\r\n",
               (unsigned int)cam, (unsigned int)h.id, (unsigned int)k, (unsigned int)f[k].seq);
        return;
    }
    printf("[MM] cam %u: no table in flash (copies 0x%X)\r\n", (unsigned int)cam,
           (unsigned int)s_fl_mask[cam]);
}

// Save the camera's RAM table (READY) into the copy that does not hold the newest
// one. Not under the module lock: it takes seconds, and the ROI listener must not
// be kept waiting; an upload is refused meanwhile (s_saving).
static int fl_save(uint8_t cam, uint8_t *copy)
{
    mm_fl_hdr_t f;
    mm_hdr_t h;
    uint32_t seq;
    uint8_t k;
    int rc;

    if (copy) *copy = 0xFFU;
    if (!spi_flash_present()) return SPI_FLASH_ERR_ABSENT;
    if (!lock()) return SPI_FLASH_ERR_BUSY;
    if (s_st[cam].state != MM_ST_READY || s_saving) {
        unlock();
        return SPI_FLASH_ERR_BUSY;
    }
    s_saving = true;
    h = s_st[cam].hdr;
    unlock();

    {
        mm_fl_hdr_t old[2];
        fl_scan(cam, old);
    }
    // Never the copy the RAM table came from: a copy with a good header but a bad
    // table can be the newest by seq and still be the one NOT in use.
    if (s_fl_used[cam] == 0 || s_fl_used[cam] == 1) k = (uint8_t)(1 - s_fl_used[cam]);
    else if (s_fl_mask[cam] == 3U)  k = (s_fl_seq[cam][0] >= s_fl_seq[cam][1]) ? 1U : 0U;
    else if (s_fl_mask[cam] == 1U)  k = 1U;
    else                            k = 0U;
    seq = ((s_fl_seq[cam][0] > s_fl_seq[cam][1]) ? s_fl_seq[cam][0] : s_fl_seq[cam][1]) + 1U;

    memset(&f, 0xFF, sizeof(f));
    f.magic   = MM_FL_MAGIC;
    f.version = MM_FL_VERSION;
    f.seq     = seq;
    f.hdr[0] = h.cam; f.hdr[1] = h.version; f.hdr[2] = h.gk_log2; f.hdr[3] = h.gv_log2;
    f.hdr[4] = (uint8_t)(h.cols >> 8); f.hdr[5] = (uint8_t)h.cols;
    f.hdr[6] = (uint8_t)(h.rows >> 8); f.hdr[7] = (uint8_t)h.rows;
    f.hdr[8]  = (uint8_t)(h.id >> 24);  f.hdr[9]  = (uint8_t)(h.id >> 16);
    f.hdr[10] = (uint8_t)(h.id >> 8);   f.hdr[11] = (uint8_t)h.id;
    f.hdr[12] = (uint8_t)(h.crc >> 24); f.hdr[13] = (uint8_t)(h.crc >> 16);
    f.hdr[14] = (uint8_t)(h.crc >> 8);  f.hdr[15] = (uint8_t)h.crc;
    f.hdr[16] = h.unit_shift; f.hdr[17] = h.frame; f.hdr[18] = 0; f.hdr[19] = 0;
    f.hdr_crc = fl_hdr_crc(&f);

    rc = spi_flash_erase(MM_FL_SLOT(cam, k), MM_FL_DATA_OFF + MM_TABLE_BYTES);
    if (rc == SPI_FLASH_OK)
        rc = spi_flash_write(MM_FL_SLOT(cam, k) + MM_FL_DATA_OFF,
                             (const uint8_t *)MM_TABLE_ADDR(cam), MM_TABLE_BYTES);
    if (rc == SPI_FLASH_OK)                       /* header last: the copy becomes whole */
        rc = spi_flash_write(MM_FL_SLOT(cam, k), (const uint8_t *)&f, sizeof(f));
    {
        mm_fl_hdr_t now[2];
        fl_scan(cam, now);
    }
    if (rc == SPI_FLASH_OK) s_fl_used[cam] = (int8_t)k;
    if (copy && rc == SPI_FLASH_OK) *copy = k;
    printf("[MM] cam %u: table id %08X saved to flash copy %u (seq %u): rc %d\r\n",
           (unsigned int)cam, (unsigned int)h.id, (unsigned int)k, (unsigned int)seq, rc);
    s_saving = false;
    return rc;
}

void mm_init(void)
{
    uint8_t c;

    if (s_mx == NULL) s_mx = xSemaphoreCreateMutex();
    memset(s_st, 0, sizeof(s_st));
    for (c = 0; c < MM_CAMS; ++c) s_st[c].off_why = MM_OFF_NO_TABLE;
    // The counter goes on from what the FPGA has seen: the mailbox survives a CPU reboot.
    mailbox_load(256U);
    s_seq = mailbox_get(MMTB_W_SEQ);
    publish_locked();
    // The saved tables (R-39); their bands come when the cameras report their ROI.
    spi_flash_init();
    for (c = 0; c < MM_CAMS; ++c) s_fl_used[c] = -1;
    if (spi_flash_present())
        printf("[MM] SPI flash JEDEC id %06X\r\n", (unsigned int)spi_flash_id());
    // Reads need no register port (the boot window): the saved tables load on any bitstream.
    for (c = 0; c < MM_CAMS; ++c) fl_load(c);
    camera_set_roi_listener(on_roi);
}

int mm_save(uint8_t cam, uint8_t *copy)
{
    if (cam >= MM_CAMS) return SPI_FLASH_ERR_RANGE;
    return fl_save(cam, copy);
}

void mm_flash_info(uint8_t cam, mm_flash_info_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (cam >= MM_CAMS) return;
    out->mask   = s_fl_mask[cam];
    out->used   = s_fl_used[cam];
    out->seq[0] = s_fl_seq[cam][0];
    out->seq[1] = s_fl_seq[cam][1];
    out->id[0]  = s_fl_id[cam][0];
    out->id[1]  = s_fl_id[cam][1];
}

int mm_write(uint8_t cam, uint32_t off, const uint8_t *data, uint32_t len)
{
    mm_info_t *s;

    if (cam >= MM_CAMS) return ST_BAD_TARGET;
    if (data == NULL || len == 0U || (off & 3U) || (len & 3U) || off + len > MM_TABLE_BYTES)
        return ST_BAD_LENGTH;
    if (s_saving) return ST_BUSY;           /* the table is being copied into flash */
    if (!lock()) return ST_BUSY;
    s = &s_st[cam];
    if (off == 0U) {
        // A new table: the FPGA must stop using this camera's band before its source
        // changes under it.
        bool was_on = (s->off_why == MM_OFF_NONE);
        s->state    = MM_ST_UPLOADING;
        s->received = 0;
        memset(&s->hdr, 0, sizeof(s->hdr));
        s->off_why  = MM_OFF_NO_TABLE;
        s->rows = 0; s->band_crc = 0;
        if (was_on) publish_locked();
    }
    if (s->state != MM_ST_UPLOADING || off != s->received) {
        unlock();
        return ST_DENIED;               /* out of order: start again from offset 0 */
    }
    memcpy((void *)(MM_TABLE_ADDR(cam) + off), data, len);
    s->received += len;
    unlock();
    return ST_OK;
}

int mm_commit(const uint8_t *body, uint32_t len, uint8_t *cam_out)
{
    mm_hdr_t h;
    mm_info_t *s;
    camera_params_t p;
    uint32_t crc;
    int st = ST_OK;

    if (body == NULL || len != MM_HDR_BYTES) return ST_BAD_LENGTH;
    if (cam_out) *cam_out = body[0];
    if (body[0] >= MM_CAMS) return ST_BAD_TARGET;
    if (!parse_hdr(body, &h)) return ST_BAD_VERSION;
    if (!lock()) return ST_BUSY;
    s = &s_st[h.cam];
    if (s->state != MM_ST_UPLOADING || s->received != MM_TABLE_BYTES) {
        unlock();
        return ST_DENIED;
    }
    // Out of the cache into DDR3 (a frame-sized range: flush_all, invariant 4), and
    // the CRC from DDR3 itself - the bytes the FPGA will read.
    __asm__ volatile ("fence" ::: "memory");
    ae350_dcache_flush_all();
    crc = table_crc(h.cam, 0, MM_TABLE_WORDS);
    if (crc != h.crc) {
        s->state = MM_ST_NONE;
        st = ST_BAD_CRC;
        printf("[MM] cam %u: table CRC %08X, header says %08X - rejected\r\n",
               (unsigned int)h.cam, (unsigned int)crc, (unsigned int)h.crc);
    } else {
        s->state = MM_ST_READY;
        s->hdr   = h;
        // The band for the ROI the camera has now: as last read back, or before any
        // reconfiguration since boot the ROI the firmware asked for.
        if (s_roi_state[h.cam] == 1U)                       (void)band_locked(h.cam, &s_roi[h.cam]);
        else if (s_roi_state[h.cam] == 0U &&
                 camera_cached_params(h.cam, &p) == 0)      (void)band_locked(h.cam, &p);
        else                                                (void)band_locked(h.cam, NULL);
        publish_locked();
        printf("[MM] cam %u: table id %08X ready, band %s (rows %u from %u, crc %08X)\r\n",
               (unsigned int)h.cam, (unsigned int)h.id,
               (s->off_why == MM_OFF_NONE) ? "on" : "off", (unsigned int)s->rows,
               (unsigned int)s->r0, (unsigned int)s->band_crc);
    }
    unlock();
    return st;
}

bool mm_read_echo(uint8_t cam, mm_echo_t *e)
{
    uint32_t base = VIDEO_DIAG_ADDR(cam);
    volatile uint32_t *d = (volatile uint32_t *)base;
    uint32_t w;

    if (e == NULL) return false;
    memset(e, 0, sizeof(*e));
    // Telemetry is written by the FPGA alone: invalidate is enough (invariant 4).
    ae350_dcache_invalidate_range(base, 256);
    w = d[MM_T_ECHO];
    e->valid = (d[0] == VIDEO_FRAME_MAGIC) && ((w >> 24) == MM_T_ID_BYTE);
    if (!e->valid) return false;
    e->seq        = (uint8_t)(w >> 16);
    e->band_valid = (w & (1U << 15)) != 0U;
    e->crc_fail   = (w & (1U << 14)) != 0U;
    e->load_err   = (w & (1U << 13)) != 0U;
    e->loading    = (w & (1U << 12)) != 0U;
    e->rows       = (uint8_t)(w & 0x1FU);
    e->crc        = d[MM_T_CRC];
    return true;
}

void mm_info(uint8_t cam, mm_info_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (cam >= MM_CAMS) return;
    if (lock()) {
        *out = s_st[cam];
        unlock();
    }
    (void)mm_read_echo(cam, &out->echo);
}

int mm_wait_fpga(uint8_t cam, uint32_t timeout_ms)
{
    mm_echo_t e;
    uint32_t waited = 0;
    uint32_t seq;
    bool on;

    if (cam >= MM_CAMS || !lock()) return 0;
    seq = s_st[cam].seq;
    on  = (s_st[cam].off_why == MM_OFF_NONE);
    unlock();
    for (;;) {
        if (!mm_read_echo(cam, &e)) {
            if (waited >= 1200U) return -1;     /* two telemetry periods without 'M' */
        } else if (e.seq == (uint8_t)seq && !e.loading) {
            return (e.band_valid == on) ? 1 : 0;
        }
        if (waited >= timeout_ms) return 0;
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50U;
    }
}

bool mm_expected(uint8_t cam, uint32_t *table_id, uint32_t *band_crc)
{
    bool on = false;

    if (cam >= MM_CAMS || !lock()) return false;
    if (s_st[cam].off_why == MM_OFF_NONE) {
        on = true;
        if (table_id) *table_id = s_st[cam].hdr.id;
        if (band_crc) *band_crc = s_st[cam].band_crc;
    }
    unlock();
    return on;
}
