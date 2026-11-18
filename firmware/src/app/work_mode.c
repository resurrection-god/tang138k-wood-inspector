/*
 * work_mode.c - operating modes STANDBY / WORK / SERVICE and their settings
 * sets (what is applied on entering a mode, what is saved to the store)
 *
 * Part of net_stack.c until 24.09.2026.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cam_trig.h"
#include "sync.h"
#include "video.h"
#include "laser.h"
#include "work_mode.h"
#include "settings.h"
#include "camera.h"
#include "i2c_bus.h"
#include "cache.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"



// ---- Operating modes (0x0110) ----------------------------------------------
//
// docs/ARCHITECTURE.md § 7. The order of steps is not cosmetic: the laser is
// turned off FIRST, so that a failure at any later step leaves it off.

// Since 02.10.2026 the board starts in STANDBY (before that, in SERVICE): only
// there does it answer UDP discovery (vmsp_service.c), and the client finds it
// without knowing the address (docs/DECISIONS.md R-27, R-31). At boot the
// cameras are still set up from the SERVICE set (work_mode_boot_cam), the
// synchroniser gets the same words but disabled (ws_boot), so entering SERVICE
// reconfigures nothing needlessly.
static volatile uint8_t s_work_mode = WORK_MODE_STANDBY;

// The mode settings sets are below, section "mode sets in RAM".
static void    mode_set_capture(uint8_t mode);
static uint8_t mode_set_apply(uint8_t mode);

uint8_t work_mode_get(void)
{
    return s_work_mode;
}

// The laser enable is per mode, for this session only (not saved,
// docs/ARCHITECTURE.md § 7). Until 23.09.2026 there was one state here,
// remembered on entering STANDBY; now WORK and SERVICE have different sets, and
// an enable given for alignment in SERVICE must not move to WORK along with the
// transition.
static bool s_mode_laser_en[2] = { false, false };   /* [0] WORK, [1] SERVICE */

static void work_mode_laser_off(void)
{
    uint32_t ctrl = sync_get(SYNC_W_CTRL);

    sync_set(SYNC_W_LASER_MODE, 1U);                  /* "always 0" */
    sync_set(SYNC_W_CTRL, ctrl & ~SYNC_CTRL_LASER_EN);
}

// Stop sensor capture. Not fatal: the camera does not write the ring anyway
// while the sync block gives no pulses, so a busy I2C bus is a reason for a
// flag in the response, not for refusing the transition.
static bool work_mode_stop_sensor(void)
{
    bool stopped = false;

    if (i2c_bus_lock(1000))
    {
        camera_select_channel(0);
        stopped = (camera_set_acquisition(0, 0) == 0);
        i2c_bus_unlock();
    }
    return stopped;
}

int work_mode_set(uint8_t mode, uint8_t *mode_actual, uint8_t *flags)
{
    uint32_t ctrl;
    int      ack;
    uint8_t  f = 0;
    uint8_t  old;

    if (mode > WORK_MODE_SERVICE)
    {
        if (mode_actual) *mode_actual = s_work_mode;
        if (flags) *flags = 0;
        return 12;                                   /* VCP_ST_UNSUPPORTED */
    }
    // The transition changes the pulse source and the laser: not allowed during
    // frame delivery.
    if (!video_ring_lock(2000U))
    {
        if (mode_actual) *mode_actual = s_work_mode;
        if (flags) *flags = 0;
        return 2;                                    /* VCP_ST_BUSY */
    }

    // 0. The state of the mode being left goes into its set. Before turning the
    // laser off: otherwise the set would get the "always 0" mode set by step 1.
    old = s_work_mode;
    if (old != WORK_MODE_STANDBY)
    {
        mode_set_capture(old);
        s_mode_laser_en[MODE_SET_IDX(old)] =
            (sync_get(SYNC_W_CTRL) & SYNC_CTRL_LASER_EN) != 0U;
    }

    // 1. The laser first.
    work_mode_laser_off();
    sync_apply();

    // 2. Pulses: in STANDBY the burst generator is stopped and the block disabled.
    if (mode == WORK_MODE_STANDBY)
    {
        (void)cam_trig_stop();
        ctrl = sync_get(SYNC_W_CTRL) & ~SYNC_CTRL_ENABLE;
        sync_set(SYNC_W_CTRL, ctrl);
        sync_apply();
        if (work_mode_stop_sensor()) f |= WORK_MODE_F_CAM_STOP;
        // The sensor may have been armed and frames may be left in the ring: the next
        // capture must arm the camera again (see the function's comment).
        video_invalidate_trigger_arming();
    }
    else
    {
        // The new mode's set: the synchroniser (with enable and this mode's session
        // laser enable) and the cameras. The finder is not part of the sets: it is
        // shared by the system (since 24.09.2026).
        f |= mode_set_apply(mode);
    }

    s_work_mode = mode;
    laser_set_mode(mode);

    // 3. The mode goes into the mailbox, together with the finder setting of both
    // cameras (the CTRL word carries it in bits 1:0).
    laser_cfg_write();

    // 4. Echo: both cameras reflected their APPLY_SEQ. The FPGA does not reflect
    // the mode itself (the echo has two words, ERRORS former 31.53); what is
    // confirmed is the write of the blocks that hold it. Builds before L2a have no
    // finder words; that means "the FPGA cannot", not "did not confirm"
    // (docs/ARCHITECTURE.md § 7).
    ack = laser_fpga_ack(0x03U, LASER_ACK_TIMEOUT_MS);
    if (ack >= 0)
    {
        f |= WORK_MODE_F_FPGA_LASR;
        if (ack > 0) f |= WORK_MODE_F_FPGA_ACK;
    }

    video_ring_unlock();

    printf("[MODE] -> %u (%s), flags 0x%02X\r\n", (unsigned int)mode,
           (mode == WORK_MODE_STANDBY) ? "STANDBY" :
           (mode == WORK_MODE_WORK) ? "WORK" : "SERVICE", (unsigned int)f);

    if (mode_actual) *mode_actual = s_work_mode;
    if (flags) *flags = f;
    // FPGA silence with a live LASR block is an error, and it must be reported:
    // silently working in the wrong mode is worse than refusing.
    if ((f & WORK_MODE_F_FPGA_LASR) && !(f & WORK_MODE_F_FPGA_ACK))
        return 10;                                   /* VCP_ST_HW_ERROR */
    // The mode changed but the camera kept the previous settings: the same thing.
    if (f & WORK_MODE_F_CAM_FAIL)
        return 10;                                   /* VCP_ST_HW_ERROR */
    return 0;
}


// Synchroniser mailbox words that are part of a set: 7 (CTRL) .. 18 (EXT_FILTER).
#define SET_SYNC_FIRST      SYNC_W_CTRL
#define SET_SYNC_WORDS      (SYNC_W_EXT_FILTER - SYNC_W_CTRL + 1U)
// From CTRL only the source and the EXT edge go into the set: enable is set by
// the mode, and the laser enable is never saved (docs/DECISIONS.md R-12).
#define SET_CTRL_SAVED      (SYNC_CTRL_SRC_MASK | SYNC_CTRL_EXT_FALL)

// Section items; tag = (mode << 4) | item. Mode SET_MODE_SYSTEM holds settings
// outside the mode sets: the line finder, a record of its own per camera
// (since image version 2, 24.09.2026; before that there was one finder, in the
// mode set). The version 1 layout is not read at all (settings.c, min_version).
#define SET_ITEM_CAM0       1U
#define SET_ITEM_CAM1       2U
#define SET_ITEM_SYNC       3U
#define SET_ITEM_LASER0     4U      /* SET_MODE_SYSTEM: finder of camera 0 */
#define SET_ITEM_LASER1     5U      /* ... of camera 1                     */
#define SET_MODE_SYSTEM     0x0FU
#define SET_CAM_BYTES       20U
#define SET_SYNC_BYTES      (SET_SYNC_WORDS * 4U)
#define SET_LASER_BYTES     14U
#define SET_CFG_PAYLOAD     (2U * (VIDEO_CAM_COUNT * (2U + SET_CAM_BYTES) + \
                                   (2U + SET_SYNC_BYTES)) +                   \
                             VIDEO_CAM_COUNT * (2U + SET_LASER_BYTES))

typedef struct {
    uint32_t exp_us;      // us
    uint32_t fps;         // 0.01 Hz
    uint16_t gain;        // 0.1 dB
    uint16_t roi_w, roi_h, off_x, off_y;
    uint8_t  testpat;
} set_cam_t;

typedef struct {
    set_cam_t   cam[VIDEO_CAM_COUNT];
    uint32_t    sync[SET_SYNC_WORDS];     /* mailbox words 7..18 */
} mode_set_t;

static mode_set_t s_set[2];                 /* [0] WORK, [1] SERVICE — MODE_SET_IDX */
static bool       s_set_init  = false;

// ---- mode sets in RAM ------------------------------------------------------------------

static void set_defaults(mode_set_t *s)
{
    uint32_t w[SYNC_W_COUNT];
    uint8_t c;

    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        s->cam[c].exp_us  = CAM_BOOT_EXP_US;
        s->cam[c].fps     = CAM_BOOT_FPS;
        s->cam[c].gain    = CAM_BOOT_GAIN;
        s->cam[c].roi_w   = VIDEO_FRAME_WIDTH;
        s->cam[c].roi_h   = VIDEO_FRAME_HEIGHT;
        s->cam[c].off_x   = 0U;
        s->cam[c].off_y   = 0U;
        s->cam[c].testpat = 0U;
    }
    sync_default_words(w);
    memcpy(s->sync, &w[SET_SYNC_FIRST], sizeof(s->sync));
}

static void set_laser_defaults(laser_cfg_t *l)
{
    static const laser_cfg_t def = LASER_CFG_DEFAULT_INIT;
    uint8_t c;

    for (c = 0; c < VIDEO_CAM_COUNT; ++c) l[c] = def;
}

static void set_init(void)
{
    if (s_set_init) return;
    set_defaults(&s_set[0]);
    set_defaults(&s_set[1]);
    s_set_init = true;
}

// The same limits as 0x0112 and camera_configure(): a value the camera would
// not accept over the network must not reach it from the store either.
static bool set_cam_valid(const set_cam_t *c)
{
    if (c->exp_us < 16U || c->exp_us > 1000000U) return false;
    if (c->gain > 240U) return false;
    if (c->fps < 100U || c->fps > 400000U) return false;
    if (c->roi_w < 128U || c->roi_w > VIDEO_FRAME_WIDTH) return false;
    if (c->roi_h < 64U || c->roi_h > VIDEO_FRAME_HEIGHT) return false;
    if ((uint32_t)c->off_x + c->roi_w > VIDEO_FRAME_WIDTH) return false;
    if ((uint32_t)c->off_y + c->roi_h > VIDEO_FRAME_HEIGHT) return false;
    return c->testpat <= 1U;
}

static bool set_sync_valid(const uint32_t *w)
{
    if (((w[SYNC_W_CTRL - SET_SYNC_FIRST] & SYNC_CTRL_SRC_MASK) >> SYNC_CTRL_SRC_SHIFT) > 2U)
        return false;
    if (w[SYNC_W_LASER_MODE - SET_SYNC_FIRST] > 3U) return false;
    return w[SYNC_W_POLARITY - SET_SYNC_FIRST] <= 7U;
}

static bool set_cam_matches(const set_cam_t *s, const camera_params_t *p)
{
    return p->exp == s->exp_us && p->gain == s->gain && p->fps == s->fps &&
           p->roi_w == s->roi_w && p->roi_h == s->roi_h &&
           p->offset_x == s->off_x && p->offset_y == s->off_y &&
           p->testpat == s->testpat;
}

// Current state -> mode set. Under video_ring_lock: capture temporarily changes
// the mailbox (source, the neighbouring camera blanked, laser), and a snapshot
// in the middle of it would carry foreign values into the set.
static void mode_set_capture(uint8_t mode)
{
    mode_set_t *s;
    camera_params_t p;
    uint32_t w;
    uint8_t c;

    if (mode != WORK_MODE_WORK && mode != WORK_MODE_SERVICE) return;
    set_init();
    s = &s_set[MODE_SET_IDX(mode)];
    // A camera that is absent is not touched: its cache holds the driver defaults,
    // not an operator setting, and there is no point overwriting the saved one.
    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        if (!camera_connected(c)) continue;
        if (camera_cached_params(c, &p) != 0) continue;
        s->cam[c].exp_us  = p.exp;
        s->cam[c].fps     = p.fps;
        s->cam[c].gain    = (uint16_t)p.gain;
        s->cam[c].roi_w   = (uint16_t)p.roi_w;
        s->cam[c].roi_h   = (uint16_t)p.roi_h;
        s->cam[c].off_x   = (uint16_t)p.offset_x;
        s->cam[c].off_y   = (uint16_t)p.offset_y;
        s->cam[c].testpat = p.testpat;
    }
    for (w = 0; w < SET_SYNC_WORDS; ++w) s->sync[w] = sync_get(SET_SYNC_FIRST + w);
}

// Synchroniser from the set. The enable bit follows the mode (only WORK and
// SERVICE have a set, both with the block enabled; enable = false means boot
// into STANDBY), the laser enable is this mode's session one. The finder is not
// in the set (it is shared by the system); it goes into the mailbox through
// laser_cfg_write(): work_mode_set() calls it itself, other paths right after
// this function.
static uint8_t set_apply_sync(uint8_t mode, bool enable)
{
    const mode_set_t *s = &s_set[MODE_SET_IDX(mode)];
    uint32_t ctrl;
    uint32_t w;
    uint8_t f = 0;

    for (w = 0; w < SET_SYNC_WORDS; ++w) sync_set(SET_SYNC_FIRST + w, s->sync[w]);
    ctrl = s->sync[0] & SET_CTRL_SAVED;
    if (enable) ctrl |= SYNC_CTRL_ENABLE;
    if (enable && s_mode_laser_en[MODE_SET_IDX(mode)])
    {
        ctrl |= SYNC_CTRL_LASER_EN;
        f |= WORK_MODE_F_LASER_EN;
    }
    sync_set(SYNC_W_CTRL, ctrl);
    sync_apply();
    return f;
}

// Cameras from the set: only connected ones and only if the set differs from
// what the camera already has. Reconfiguration stops the sensor, and the first
// pulses after it give no frame (ERRORS former 31.37), so there is no point
// doing it needlessly.
static uint8_t set_apply_cams(uint8_t mode)
{
    const mode_set_t *s = &s_set[MODE_SET_IDX(mode)];
    camera_params_t p;
    uint8_t f = 0;
    uint8_t c;

    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        const set_cam_t *k = &s->cam[c];
        int rc;

        if (!camera_connected(c)) continue;
        if (camera_cached_params(c, &p) != 0 || set_cam_matches(k, &p)) continue;
        if (!i2c_bus_lock(1000))
        {
            f |= WORK_MODE_F_CAM_FAIL;
            continue;
        }
        rc = camera_configure(c, 0, g_trigger_source, 0, k->exp_us, k->gain, k->fps,
                              k->testpat, k->roi_w, k->roi_h, k->off_x, k->off_y);
        video_invalidate_trigger_arming_cam(c);   /* the next capture will arm again */
        i2c_bus_unlock();
        if (rc != 0)
        {
            f |= WORK_MODE_F_CAM_FAIL;
            continue;
        }
        f |= WORK_MODE_F_CAM_CFG;
        // As in 0x0112: camera 0's receiver must know the same geometry.
        if (c == 0) video_geom_set(k->roi_w, k->roi_h);
    }
    return f;
}

static uint8_t mode_set_apply(uint8_t mode)
{
    uint8_t f;

    if (mode != WORK_MODE_WORK && mode != WORK_MODE_SERVICE) return 0;
    set_init();
    f = set_apply_sync(mode, true);
    f |= set_apply_cams(mode);
    printf("[SETTINGS] %s set applied, flags 0x%02X\r\n",
           (mode == WORK_MODE_WORK) ? "WORK" : "SERVICE", (unsigned int)f);
    return f;
}

// Apply the current mode's set outside a transition (load, reset, import).
// The finder in any mode: it is outside the sets.
static int mode_set_apply_current(void)
{
    uint8_t f;

    if (s_work_mode == WORK_MODE_STANDBY)
    {
        laser_cfg_write();
        return SETTINGS_ST_OK;                     /* the set will be applied on entry */
    }
    f = mode_set_apply(s_work_mode);
    laser_cfg_write();
    return (f & WORK_MODE_F_CAM_FAIL) ? SETTINGS_ST_HW_ERROR : SETTINGS_ST_OK;
}

void work_mode_boot_cam(uint8_t cam_index, camera_params_t *p)
{
    const set_cam_t *k;

    if (p == NULL) return;
    memset(p, 0, sizeof(*p));
    set_init();
    k = &s_set[MODE_SET_IDX(WORK_MODE_SERVICE)].cam[(cam_index < VIDEO_CAM_COUNT) ? cam_index : 0U];
    p->exp      = k->exp_us;
    p->gain     = k->gain;
    p->fps      = k->fps;
    p->testpat  = k->testpat;
    p->roi_w    = k->roi_w;
    p->roi_h    = k->roi_h;
    p->offset_x = k->off_x;
    p->offset_y = k->off_y;
}

static uint16_t set_serialize(const mode_set_t *sets, const laser_cfg_t *lz, uint8_t *out)
{
    static const uint8_t modes[2] = { WORK_MODE_WORK, WORK_MODE_SERVICE };
    uint32_t n = 0;
    uint32_t i, w;
    uint8_t c;

    for (i = 0; i < 2U; ++i)
    {
        const mode_set_t *s = &sets[MODE_SET_IDX(modes[i])];
        uint8_t base = (uint8_t)(modes[i] << 4);

        for (c = 0; c < VIDEO_CAM_COUNT; ++c)
        {
            const set_cam_t *k = &s->cam[c];
            out[n++] = (uint8_t)(base | (SET_ITEM_CAM0 + c));
            out[n++] = SET_CAM_BYTES;
            settings_wr32(out + n + 0,  k->exp_us);
            settings_wr32(out + n + 4,  k->fps);
            settings_wr16(out + n + 8,  k->gain);
            settings_wr16(out + n + 10, k->roi_w);
            settings_wr16(out + n + 12, k->roi_h);
            settings_wr16(out + n + 14, k->off_x);
            settings_wr16(out + n + 16, k->off_y);
            out[n + 18] = k->testpat;
            out[n + 19] = 0U;
            n += SET_CAM_BYTES;
        }

        out[n++] = (uint8_t)(base | SET_ITEM_SYNC);
        out[n++] = SET_SYNC_BYTES;
        for (w = 0; w < SET_SYNC_WORDS; ++w)
        {
            uint32_t v = s->sync[w];
            if (SET_SYNC_FIRST + w == SYNC_W_CTRL) v &= SET_CTRL_SAVED;
            // "Always 1" is the alignment mode; after power-up the laser must be off
            // (docs/DECISIONS.md R-12): it is saved as "always 0".
            if (SET_SYNC_FIRST + w == SYNC_W_LASER_MODE && v == 2U) v = 1U;
            settings_wr32(out + n, v);
            n += 4U;
        }

    }

    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        const laser_cfg_t *k = &lz[c];
        out[n++] = (uint8_t)((SET_MODE_SYSTEM << 4) | (SET_ITEM_LASER0 + c));
        out[n++] = SET_LASER_BYTES;
        out[n + 0] = k->axis;
        out[n + 1] = k->thresh;
        out[n + 2] = k->gap;
        out[n + 3] = k->flags;
        settings_wr16(out + n + 4,  k->win_lo);
        settings_wr16(out + n + 6,  k->win_hi);
        settings_wr16(out + n + 8,  k->min_sum);
        settings_wr16(out + n + 10, k->width_max);
        out[n + 12] = k->offset;
        out[n + 13] = 0U;
        n += SET_LASER_BYTES;
    }
    return (uint16_t)n;
}

// Parse the settings partition data into sets (prefilled with defaults).
// Returns the number of rejected sections: wrong length or a value out of range.
// A rejected section keeps the default, an unknown one is skipped silently.
static uint32_t set_parse(const uint8_t *p, uint32_t len, mode_set_t *sets, laser_cfg_t *lz)
{
    uint32_t pos = 0, bad = 0;

    while (pos + 2U <= len)
    {
        uint8_t tag = p[pos], l = p[pos + 1];
        const uint8_t *d = p + pos + 2U;
        uint8_t mode = (uint8_t)(tag >> 4), item = (uint8_t)(tag & 0x0FU);
        mode_set_t *s;

        if (pos + 2U + l > len) { ++bad; break; }
        pos += 2U + l;
        if (mode == SET_MODE_SYSTEM &&
            (item == SET_ITEM_LASER0 || item == SET_ITEM_LASER1))
        {
            laser_cfg_t k;
            if (l != SET_LASER_BYTES) { ++bad; continue; }
            k.axis      = d[0];
            k.thresh    = d[1];
            k.gap       = d[2];
            k.flags     = d[3];
            k.win_lo    = settings_rd16(d + 4);
            k.win_hi    = settings_rd16(d + 6);
            k.min_sum   = settings_rd16(d + 8);
            k.width_max = settings_rd16(d + 10);
            k.offset    = d[12];
            if (laser_cfg_check(&k) != 0) { ++bad; continue; }
            lz[item - SET_ITEM_LASER0] = k;
            continue;
        }
        if (mode != WORK_MODE_WORK && mode != WORK_MODE_SERVICE) continue;
        s = &sets[MODE_SET_IDX(mode)];

        if (item == SET_ITEM_CAM0 || item == SET_ITEM_CAM1)
        {
            set_cam_t k;
            if (l != SET_CAM_BYTES) { ++bad; continue; }
            k.exp_us  = settings_rd32(d + 0);
            k.fps     = settings_rd32(d + 4);
            k.gain    = settings_rd16(d + 8);
            k.roi_w   = settings_rd16(d + 10);
            k.roi_h   = settings_rd16(d + 12);
            k.off_x   = settings_rd16(d + 14);
            k.off_y   = settings_rd16(d + 16);
            k.testpat = d[18];
            if (!set_cam_valid(&k)) { ++bad; continue; }
            s->cam[item - SET_ITEM_CAM0] = k;
        }
        else if (item == SET_ITEM_SYNC)
        {
            uint32_t w[SET_SYNC_WORDS];
            uint32_t i;
            if (l != SET_SYNC_BYTES) { ++bad; continue; }
            for (i = 0; i < SET_SYNC_WORDS; ++i) w[i] = settings_rd32(d + 4U * i);
            w[0] &= SET_CTRL_SAVED;
            if (!set_sync_valid(w)) { ++bad; continue; }
            memcpy(s->sync, w, sizeof(w));
        }
    }
    return bad;
}

// ---- owner of the settings partition in the storage (settings.c) --------------
//
// The partition data is the WORK and SERVICE sets in the TLV layout of
// set_serialize(). Until 24.09.2026 this was in settings.c; the image bytes did
// not change with the move.

static mode_set_t  s_tmp_set[2];                   /* parse before the check */
static laser_cfg_t s_tmp_laser[VIDEO_CAM_COUNT];

typedef char mode_set_payload_check[
    (SETTINGS_HDR_BYTES + SET_CFG_PAYLOAD <= SETTINGS_SLOT_BYTES) ? 1 : -1];

// The current mode is first captured from the live state; the finder from the
// laser.c shadow.
static uint16_t ws_payload(uint8_t *out)
{
    laser_cfg_t lz[VIDEO_CAM_COUNT];
    uint8_t c;

    set_init();
    mode_set_capture(s_work_mode);
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) laser_cfg_get(c, &lz[c]);
    return set_serialize(s_set, lz, out);
}

static int ws_take(const uint8_t *p, uint16_t len, bool strict, uint32_t *bad)
{
    uint32_t b;

    uint8_t c;

    set_defaults(&s_tmp_set[0]);
    set_defaults(&s_tmp_set[1]);
    set_laser_defaults(s_tmp_laser);
    b = set_parse(p, len, s_tmp_set, s_tmp_laser);
    if (bad) *bad = b;
    if (strict && b != 0U) return SETTINGS_ST_DENIED;
    memcpy(s_set, s_tmp_set, sizeof(s_set));
    s_set_init = true;
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) laser_cfg_shadow_set(c, &s_tmp_laser[c]);
    return SETTINGS_ST_OK;
}

static void ws_defaults(void)
{
    uint8_t c;

    set_defaults(&s_set[0]);
    set_defaults(&s_set[1]);
    s_set_init = true;
    set_laser_defaults(s_tmp_laser);
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) laser_cfg_shadow_set(c, &s_tmp_laser[c]);
}

// The SERVICE set goes into the synchroniser and the finder; camera_boot_reset()
// sets the cameras from the same set (work_mode_boot_cam). The laser will not be
// enabled: the session enable is off in all modes after boot. The block is
// disabled: since 02.10.2026 the board starts in STANDBY, and enable comes with
// entering a working mode.
static void ws_boot(void)
{
    set_init();
    (void)set_apply_sync(WORK_MODE_SERVICE, s_work_mode != WORK_MODE_STANDBY);
    laser_cfg_write();
}

static void ws_print(void)
{
    laser_cfg_t lz;
    uint8_t i;

    printf("  mode %u\r\n", (unsigned int)s_work_mode);
    set_init();
    for (i = 0; i < 2U; ++i)
    {
        const mode_set_t *s = &s_set[i];
        printf("  %s set: cam0 %u us / %u.%u dB / %u.%02u fps / %ux%u, cam1 %u us / %u.%u dB,"
               " sync src %u laser mode %u\r\n",
               (i == 0U) ? "WORK   " : "SERVICE",
               (unsigned int)s->cam[0].exp_us, (unsigned int)(s->cam[0].gain / 10U),
               (unsigned int)(s->cam[0].gain % 10U), (unsigned int)(s->cam[0].fps / 100U),
               (unsigned int)(s->cam[0].fps % 100U), (unsigned int)s->cam[0].roi_w,
               (unsigned int)s->cam[0].roi_h, (unsigned int)s->cam[1].exp_us,
               (unsigned int)(s->cam[1].gain / 10U), (unsigned int)(s->cam[1].gain % 10U),
               (unsigned int)((s->sync[0] & SYNC_CTRL_SRC_MASK) >> SYNC_CTRL_SRC_SHIFT),
               (unsigned int)s->sync[SYNC_W_LASER_MODE - SET_SYNC_FIRST]);
    }
    for (i = 0; i < VIDEO_CAM_COUNT; ++i)
    {
        laser_cfg_get(i, &lz);
        printf("  finder cam%u: axis %u thr %u offset %u gap %u win %u..%u flags 0x%02X\r\n",
               (unsigned int)i, (unsigned int)lz.axis, (unsigned int)lz.thresh,
               (unsigned int)lz.offset, (unsigned int)lz.gap, (unsigned int)lz.win_lo,
               (unsigned int)lz.win_hi, (unsigned int)lz.flags);
    }
}

// Version 2 (24.09.2026): the finder is outside the sets, one record per camera.
// A version 1 image is rejected whole on load; the board comes up on defaults
// with flag SETTINGS_F_REJECTED (owner's decision).
const settings_section_t work_mode_settings_section = {
    ws_payload, ws_take, mode_set_apply_current, ws_defaults, ws_boot, ws_print, 0U, 2U
};

// In WORK the storage is read-only: ~0.1 s of I2C writes in the middle of the
// pipeline (docs/ARCHITECTURE.md § 7).
bool work_mode_settings_writable(void)
{
    return s_work_mode != WORK_MODE_WORK;
}
