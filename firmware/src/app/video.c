/*
 * video.c - receive geometry (GEOM), capture by trigger, frame detection in the ring
 *
 * Part of net_stack.c until 24.09.2026.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "platform.h"
#include "memory_map.h"
#include "mailbox.h"
#include "sync.h"
#include "video.h"
#include "camera.h"
#include "i2c_bus.h"
#include "cache.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"



// ---- MIPI receive geometry ---------------------------------------------------
//
// Shadow of the apply counter, for the same reason as the sync block's: the
// mailbox lives in DDR3 and survives a CPU reboot, so the count must continue
// from what is already in memory, otherwise the first command after a restart
// matches the old value and the FPGA does not notice it.
static uint32_t s_geom_seq;
static bool     s_geom_init = false;
static uint32_t s_geom_applied = 0xFFFFFFFFU;   // last value sent

static void geom_init(void)
{
    if (s_geom_init) return;
    mailbox_load(128);
    s_geom_seq = (mailbox_get(GEOM_W_MAGIC) == GEOM_MAGIC) ? mailbox_get(GEOM_W_SEQ) : 0U;
    s_geom_init = true;
}

void video_geom_set(uint16_t line_bytes, uint16_t frame_lines)
{
    uint32_t want = ((uint32_t)frame_lines << 16) | (uint32_t)line_bytes;

    geom_init();
    // Do not touch the FPGA if the geometry is the same.
    //
    // Applying holds the D-PHY receiver in reset, and it must not be released while
    // the sensor is silent: the aligner sticks on the preamble and does not lock on
    // the sync byte (the watchdog comment in cam_channel.v, formerly
    // fpga_camera_ddr3_streamer.v; ERRORS 2.5). `0x0112` is called often and usually
    // with the same geometry; every such call would pulse the reset needlessly and
    // break reception that already works.
    if (want == s_geom_applied) return;
    s_geom_applied = want;
    // Magic and size, then the counter: the mailbox.h order.
    mailbox_put(GEOM_W_MAGIC, GEOM_MAGIC);
    mailbox_put(GEOM_W_SIZE,  want);
    mailbox_publish(GEOM_W_SEQ, ++s_geom_seq);
    printf("[VIDEO] rx geometry -> %u x %u (seq %u)\r\n",
           (unsigned int)line_bytes, (unsigned int)frame_lines,
           (unsigned int)s_geom_seq);
}


// ---- Capture by trigger (protocol stage 3, docs/DECISIONS.md R-04) -----------
//
// A frame is one sync block pulse. The camera is armed for the hardware trigger
// (0x0404 = 1, 0x0408 = 1, 0x0400 = 1), the block's source is CORE, the
// firmware gives a software trigger (sync_fire) and waits until the FPGA closes
// the frame.
//
// There is no free-run window any more. It was needed while one pulse could not
// give a frame: the writer did not close the last frame without words of the
// next one (ERRORS 2.5, cause 1; fixed in the RTL, former 31.37), and the D-PHY
// receiver was released into the silence of the armed sensor (cause 2; release
// on the camera strobe, 14.09).
//
// While the frame is being written the CPU does not read the ring (the old
// invariant 5, Shared DDR3): the close marker is the frame number in the
// telemetry block 0x06000000, outside the ring. The block is updated every
// 0.5 s, hence most of the response delay.

// Whether this firmware armed the camera. The flag was one per board while there
// was one write path and only one camera could capture. With two writers
// (18.09.2026) the cameras arm and capture independently, so they have their
// own flags.
static volatile bool s_armed[VIDEO_CAM_COUNT] = { false, false };

// The frames of the last capture are in this camera's ring, and it has written
// nothing since.
static volatile bool s_frames_ready[VIDEO_CAM_COUNT] = { false, false };
// Whether these frames were taken with illumination (see video_frames_had_laser).
static volatile bool s_frames_laser[VIDEO_CAM_COUNT] = { false, false };
static volatile bool s_capture_busy = false;

void video_invalidate_trigger_arming(void)
{
    uint8_t c;
    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        s_armed[c] = false;
        s_frames_ready[c] = false;
    }
}

void video_invalidate_trigger_arming_cam(uint8_t cam_id)
{
    if (cam_id >= VIDEO_CAM_COUNT) return;
    s_armed[cam_id] = false;
    s_frames_ready[cam_id] = false;
}

void video_invalidate_frames(void)
{
    uint8_t c;
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) s_frames_ready[c] = false;
}

// Make sure the sensor's ROI is the geometry the bitstream expects, leaving the
// sensor stopped. Caller must hold i2c_bus_lock() and have stopped acquisition.
//
// Measured on 15.09: after a boot the camera read back 0x0818 = 1080 and
// 0x081C = 1280 - its full-sensor default - so every line arrived 1080 bytes
// long, the CSI-2 bridge rejected all of them on word count, and FETCH ran both
// bursts to FRAME_DONE: SENT=0 with the FPGA frame counter stuck at 1. Nothing
// checked the ROI after camera_load_default_config(), so the board stayed in
// that state until a CFG rewrote it. The FPGA accepts only
// VIDEO_FRAME_WIDTH x VIDEO_FRAME_HEIGHT lines, so a different ROI can never
// yield a frame and is always rewritten here (height last, per the VEYE map).
static bool video_ensure_roi_locked(uint8_t cam_id)
{
    uint32_t w = 0, h = 0;
    int rc_w = camera_read_reg_status(0x0818, &w);
    int rc_h = camera_read_reg_status(0x081C, &h);

    if (rc_w == 0 && rc_h == 0 &&
        w == VIDEO_FRAME_WIDTH && (h >= 64 && h <= VIDEO_FRAME_HEIGHT))
    {
        camera_cache_set_roi_h(cam_id, h, -1);   /* the frame is checked against it */
        video_geom_set(VIDEO_FRAME_WIDTH, (uint16_t)h);
        return true;
    }

    printf("[VIDEO] capture: cam %u ROI %ux%u (rc %d/%d), restoring %ux%u\r\n",
           (unsigned int)cam_id, (unsigned int)w, (unsigned int)h, rc_w, rc_h,
           (unsigned int)VIDEO_FRAME_WIDTH, (unsigned int)VIDEO_FRAME_HEIGHT);

    // The camera MCU drops back-to-back I2C transactions; space the writes.
    vTaskDelay(pdMS_TO_TICKS(50));
    camera_write_reg(0x0818, VIDEO_FRAME_WIDTH);
    vTaskDelay(pdMS_TO_TICKS(20));
    camera_write_reg(0x0820, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    camera_write_reg(0x0824, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    camera_write_reg(0x081C, VIDEO_FRAME_HEIGHT);
    vTaskDelay(pdMS_TO_TICKS(20));

    w = 0; h = 0;
    rc_w = camera_read_reg_status(0x0818, &w);
    rc_h = camera_read_reg_status(0x081C, &h);
    if (rc_w != 0 || rc_h != 0 ||
        w != VIDEO_FRAME_WIDTH || (h < 64 || h > VIDEO_FRAME_HEIGHT))
    {
        printf("[VIDEO] capture: cam %u ROI still %ux%u after rewrite\r\n",
               (unsigned int)cam_id, (unsigned int)w, (unsigned int)h);
        return false;
    }
    camera_cache_set_roi_h(cam_id, h, 0);        /* 0x0824 written as 0 above */
    video_geom_set(VIDEO_FRAME_WIDTH, (uint16_t)h);
    printf("[VIDEO] capture: cam %u ROI restored (%ux%u)\r\n",
           (unsigned int)cam_id, (unsigned int)w, (unsigned int)h);
    return true;
}

// Arm the camera for the hardware trigger without emitting a single frame.
// Under i2c_bus_lock().
//
// camera_set_trigger_mode() stops the sensor, keeps the pauses between writes
// (the camera MCU loses back-to-back transactions, ERRORS former 31.30,
// failure (a); ERRORS 2.6), checks 0x0404/0x0408 by reading back and only then
// writes 0x0400 = 1.
static bool video_arm_trigger_locked(uint8_t cam_id)
{
    camera_select_channel(cam_id);
    camera_set_acquisition(cam_id, 0);
    vTaskDelay(pdMS_TO_TICKS(VIDEO_QUIESCE_MS));
    if (!video_ensure_roi_locked(cam_id)) return false;

    g_trigger_source = 1;                  /* the Trigger In input, driven by the FPGA */
    if (camera_set_trigger_mode(cam_id, 1, 1) != 0)
    {
        printf("[VIDEO] capture: cam %u refused hardware-trigger mode\r\n",
               (unsigned int)cam_id);
        return false;
    }
    // After a mode change the camera MCU does not execute a start immediately.
    vTaskDelay(pdMS_TO_TICKS(VIDEO_MODE_SETTLE_MS));
    return true;
}

// Re-arm because of the frame height (ERRORS 2.13, 2.14). The height from the
// GEOM mailbox is applied only by camera 0's writer, on a reset of its own
// D-PHY watchdog, and camera 1's writer takes it from there over a wire
// (cam_channel.v, geom_lines_in). So camera 1 gets the height from re-arming
// camera 0, not from its own re-arm: on 05.10.2026 five re-arms of camera 1 in
// a row left its writer at 1280 with ROI 128. Camera 0 is then armed without
// video_ensure_roi_locked: that would rewrite the mailbox with camera 0's
// height, while the capturing camera's height is needed (its own arming wrote
// it). Under i2c_bus_lock().
static bool video_rearm_for_height_locked(uint8_t cam_id)
{
    if (cam_id == 0U) return video_arm_trigger_locked(0);
    camera_select_channel(0);
    camera_set_acquisition(0, 0);
    vTaskDelay(pdMS_TO_TICKS(VIDEO_QUIESCE_MS));
    g_trigger_source = 1;
    if (camera_set_trigger_mode(0, 1, 1) != 0)
    {
        printf("[VIDEO] capture: cam 0 refused hardware-trigger mode\r\n");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(VIDEO_MODE_SETTLE_MS));
    return true;
}

// A sync block pulse goes to ALL channels at once, so when one camera captures,
// the other is exposed too, and its writer starts using Lane 4, which the two
// share. Measured on the board 18.09.2026: both cameras lose ~17 000 rows each
// and drop 31 frames each, capture answers TIMEOUT every other time. So for the
// capture the neighbouring camera's pulse is blanked by a zero width: sync_block
// drops the output before raising it, and "with zero width the output does not
// rise at all" (its own comment).
//
// This does not hinder stereo capture: the width is restored right after the
// capture, and simultaneous exposure of both cameras was done by spreading them
// in time with the cam1 channel delay (docs/ARCHITECTURE.md § 3.4).
static uint32_t s_cam_width_saved = 0;
static bool     s_cam_muted       = false;

// Laser state saved for the duration of a capture with illumination.
static uint32_t s_laser_mode_saved  = 0;
static uint32_t s_laser_width_saved = 0;
static uint32_t s_laser_ctrl_saved  = 0;
static bool     s_laser_forced      = false;

static uint32_t video_other_width_word(uint8_t cam_id)
{
    return (cam_id == 0) ? (uint32_t)SYNC_W_CAM1_WIDTH : (uint32_t)SYNC_W_CAM0_WIDTH;
}

// Laser pulse width when there is no measured strobe: from the exposure
// REQUESTED from this camera, with the same 25 % margin as the measured path.
//
// It used to take SYNC_W_LASER_WIDTH from the mailbox, and that was a silent
// failure (ERRORS 4.2, former 31.52). The mailbox holds what the operator set
// with command 0x0140 and has nothing to do with the exposure: `sync_check.py`
// leaves 13 clocks (100 ns) there and restores nothing. Camera 1 always takes
// this path (its strobe saturates between captures and reads 0xFFFFFF), so in
// the GUI "Both in turn, with laser" the illumination was visible only on
// camera 0. Measured 22.09.2026: frame mean increase +1.76 against +0.10.
//
// The firmware knows the exposure itself: it is what it wrote into 0x0C10. The
// mailbox remains the last resort for a bad camera number.
static uint32_t video_laser_width_from_exposure(uint8_t cam_id)
{
    uint32_t exp_us = camera_cached_exposure_us(cam_id);

    if (exp_us == 0U) return sync_get(SYNC_W_LASER_WIDTH);

    return (uint32_t)(LANE_US_TO_TICKS(exp_us) / VIDEO_LASER_MARGIN_DEN)
           * VIDEO_LASER_MARGIN_NUM;
}

// Laser pulse width in clocks: the exposure measured by the FPGA plus margin.
//
// Telemetry block word 15 is {strobe width, strobe count}; the width counter is
// 24 bits and saturates at 125 ms, which normally happens in free-run
// (ERRORS former 31.47). A saturated or zero value is unusable; then we compute
// from the requested exposure.
static uint32_t video_laser_width_ticks(uint8_t cam_id)
{
    uint32_t diag = VIDEO_DIAG_ADDR(cam_id);
    volatile uint32_t *d = (volatile uint32_t *)diag;
    uint32_t strobe;

    ae350_dcache_invalidate_range(diag, 128);
    if (d[0] != VIDEO_FRAME_MAGIC) return video_laser_width_from_exposure(cam_id);

    strobe = d[15] >> 8;
    if (strobe == 0U || strobe >= 0x00FFFFFFU)
        return video_laser_width_from_exposure(cam_id);

    return (strobe / VIDEO_LASER_MARGIN_DEN) * VIDEO_LASER_MARGIN_NUM;
}

// Sync block source CORE, block enabled, the neighbouring camera's pulse
// blanked. One apply for everything: the FPGA reads the mailbox every ~10 ms.
//
// Illumination is set ONLY by the request: laser_on raises SYNC_CTRL_LASER_EN,
// and !laser_on clears it even if the operator turned the laser on with command
// 0x0142. Otherwise a frame would be taken with illumination but marked in the
// ring as taken without it, and the next request without laser would get a lit
// frame from memory.
static void video_sync_prepare(uint8_t cam_id, bool laser_on)
{
    uint32_t ctrl = sync_get(SYNC_W_CTRL);
    uint32_t want = (ctrl & ~(SYNC_CTRL_SRC_MASK | SYNC_CTRL_LASER_EN)) |
                    SYNC_CTRL_ENABLE;                                  /* CORE = 0 */
    uint32_t w    = video_other_width_word(cam_id);

    s_cam_width_saved = sync_get(w);
    s_cam_muted       = (s_cam_width_saved != 0U);
    // The enable bit is touched both ways, so it must be restored after a capture
    // without illumination too, but only if it was raised.
    s_laser_forced    = laser_on || ((ctrl & SYNC_CTRL_LASER_EN) != 0U);

    if (s_laser_forced)
    {
        // Remember everything we touch, to put it back exactly as it was: mode, width
        // and the enable bit. The channel delay is not touched: the operator may have
        // tuned it for their optics.
        s_laser_mode_saved  = sync_get(SYNC_W_LASER_MODE);
        s_laser_width_saved = sync_get(SYNC_W_LASER_WIDTH);
        s_laser_ctrl_saved  = ctrl;
    }

    if (laser_on)
    {
        uint32_t width = video_laser_width_ticks(cam_id);

        sync_set(SYNC_W_LASER_MODE, 0U);          /* 0 = pulse on trigger */
        sync_set(SYNC_W_LASER_WIDTH, width);
        want |= SYNC_CTRL_LASER_EN;
        printf("[VIDEO] capture: laser ON for this capture, width %u ticks (%u us)\r\n",
               (unsigned int)width, (unsigned int)LANE_TICKS_TO_US(width));
    }

    if ((ctrl == want) && !s_cam_muted && !s_laser_forced) return;

    if (ctrl != want)
        printf("[VIDEO] capture: sync source -> CORE (CTRL 0x%03X -> 0x%03X)\r\n",
               (unsigned int)ctrl, (unsigned int)want);
    sync_set(SYNC_W_CTRL, want);
    if (s_cam_muted) sync_set(w, 0U);
    sync_apply();
    vTaskDelay(pdMS_TO_TICKS(VIDEO_SYNC_APPLY_MS));
}

// Height for camera 1 when it captures alone (after
// video_rearm_for_height_locked): camera 0's D-PHY watchdog applies the height
// from the mailbox on its strobe, and its pulse is blanked for the capture, so
// the re-arm by itself changed nothing (05.10.2026: 5 re-arms of 5 wasted,
// camera 1's writer at 1280 with ROI 128). Camera 0's channel gets its width
// back for a while, with pulses until camera 1's writer settles on its sensor's
// ROI (telemetry every 0.5 s), at most VIDEO_GEOM_PULSES; then the channel is
// blanked again. Camera 0's frames from these pulses overwrite its ring, and
// its "frames in memory" flag is cleared.
#define VIDEO_GEOM_PULSES       4U

static bool video_writer_height_wrong(uint8_t cam_id);
static uint32_t video_writer_lines(uint8_t cam_id);

static void video_geom_pulse_cam0(void)
{
    uint32_t w = video_other_width_word(1);
    uint32_t width = s_cam_muted ? s_cam_width_saved
                                 : (uint32_t)LANE_US_TO_TICKS(VIDEO_STEREO_CAM_WIDTH_US);
    unsigned k;

    sync_set(w, width);
    sync_apply();
    vTaskDelay(pdMS_TO_TICKS(VIDEO_SYNC_APPLY_MS));
    for (k = 0; k < VIDEO_GEOM_PULSES; ++k)
    {
        sync_fire();
        vTaskDelay(pdMS_TO_TICKS(VIDEO_REFIRE_MS));
        if (!video_writer_height_wrong(1)) break;
    }
    sync_set(w, 0U);       /* blanked again (an unblanked one had zero width) */
    sync_apply();
    vTaskDelay(pdMS_TO_TICKS(VIDEO_SYNC_APPLY_MS));
    s_frames_ready[0] = false;
    printf("[VIDEO] capture: cam 0 pulsed %u time(s) for the cam 1 writer height -> %u rows\r\n",
           (unsigned int)(k < VIDEO_GEOM_PULSES ? k + 1U : k),
           (unsigned int)video_writer_lines(1));
}

// Give the neighbouring camera its pulse back, and the laser its previous state.
//
// The enable bit gets back exactly the value it had before the capture: if the
// operator kept the laser on (alignment), the capture does not turn it off, and
// if not, it does not leave it on after a frame with illumination.
static void video_sync_restore(uint8_t cam_id)
{
    if (s_laser_forced)
    {
        sync_set(SYNC_W_LASER_MODE, s_laser_mode_saved);
        sync_set(SYNC_W_LASER_WIDTH, s_laser_width_saved);
        sync_set(SYNC_W_CTRL, (sync_get(SYNC_W_CTRL) & ~SYNC_CTRL_LASER_EN) |
                              (s_laser_ctrl_saved & SYNC_CTRL_LASER_EN));
        s_laser_forced = false;
        if (s_cam_muted) sync_set(video_other_width_word(cam_id), s_cam_width_saved);
        s_cam_muted = false;
        sync_apply();
        return;
    }
    if (!s_cam_muted) return;
    sync_set(video_other_width_word(cam_id), s_cam_width_saved);
    sync_apply();
    s_cam_muted = false;
}

// ---- Frame detection in the ring -----------------------------------------------
//
// Before capture the headers of all slots are erased, so any VID0 header with a
// number greater than last_id is a frame of this capture. The slot is read
// sparingly: 24 bytes of header and 8 bytes of the last row's trailer, with a
// ranged cache invalidate; only the FPGA writes this memory, and the header
// erase has already been flushed by flush_all (CLAUDE.md invariant 4).

uint32_t video_slot_height(uint32_t slot)
{
    volatile uint32_t *hdr = (volatile uint32_t *)slot;
    uint32_t h = hdr[4] >> 16;
    if (h < 64 || h > VIDEO_FRAME_HEIGHT) return VIDEO_FRAME_HEIGHT;
    return h;
}

// The last row of the frame is finished: a quick check before the full one.
static bool video_frame_closed(uint32_t slot, uint32_t id)
{
    uint32_t h = video_slot_height(slot);
    uint32_t addr = VIDEO_ROW_ADDR(slot, h - 1) + VIDEO_ROW_TAG_OFFSET;
    volatile uint32_t *tag = (volatile uint32_t *)addr;

    ae350_dcache_invalidate_range(addr, 8);
    return tag[0] == (((uint32_t)VIDEO_ROW_TAG_MAGIC << 16) | (h - 1)) &&
           tag[1] == id;
}

bool video_slot_closed(uint32_t slot, uint32_t frame_id)
{
    return video_frame_closed(slot, frame_id);
}

// ---- frame streams (GVSP): the freshest closed frame of a camera --------
// Moved from vmsp_service.c on 23.10.2026 so both streams share them.

uint32_t video_slot_frame_id(uint8_t cam, uint32_t s)
{
    uint32_t slot = VIDEO_RING_BASE(cam) + s * VIDEO_SLOT_BYTES;
    volatile uint32_t *hdr = (volatile uint32_t *)slot;

    ae350_dcache_invalidate_range(slot, 32U);           /* only the FPGA writes the ring */
    if (hdr[0] != VIDEO_FRAME_MAGIC || hdr[5] != VIDEO_HW_HDR_INTEGRITY) return 0U;
    return hdr[1];
}

void video_stream_baseline(uint8_t cam, uint32_t base[VIDEO_RING_SLOTS])
{
    for (uint32_t s = 0; s < VIDEO_RING_SLOTS; ++s) base[s] = video_slot_frame_id(cam, s);
}

// Cheap: the headers of 32 slots (one cache line each), then the tail of the last
// row of the candidates in descending number order, usually one or two.
// video_collect_frames() is too heavy for this: it checks the tail of EVERY row of
// every slot (~41 thousand reads). The frame rows are not checked here: the FPGA
// marks bad rows in what it sends, the host sees them.
bool video_stream_newest(uint8_t cam, const uint32_t base[VIDEO_RING_SLOTS], uint32_t *slot_out,
                         uint32_t *id_out)
{
    uint32_t id[VIDEO_RING_SLOTS], sl[VIDEO_RING_SLOTS];
    uint32_t n = 0, s, k;

    for (s = 0; s < VIDEO_RING_SLOTS; ++s) {
        uint32_t f = video_slot_frame_id(cam, s);
        if (f == 0U || f == base[s]) continue;           /* untouched since the stream start */
        id[n] = f;
        sl[n] = VIDEO_RING_BASE(cam) + s * VIDEO_SLOT_BYTES;
        ++n;
    }
    while (n > 0U) {
        uint32_t best = 0;
        for (k = 1; k < n; ++k)
            if (id[k] > id[best]) best = k;
        if (video_slot_closed(sl[best], id[best])) {
            *slot_out = sl[best];
            *id_out = id[best];
            return true;
        }
        id[best] = id[n - 1U];                           /* still being written: the next by number */
        sl[best] = sl[n - 1U];
        --n;
    }
    return false;
}

// The frame is whole: the trailer of EVERY row holds its own index and this
// frame's number.
//
// The last row alone is not enough (17.09.2026, ERRORS former 31.45). The first
// frame after arming the camera comes without its first rows (the D-PHY is
// released already inside the readout, ERRORS 2.5), yet it has the last row.
// The writer does not close such a frame and abandons it after ~0.33 s, and
// rows 0..57 in the slot stay from the previous pass over the ring. After a
// reboot the FPGA frame numbers start from 1 again, and the old rows carry the
// same number: a splice of two shots passed this check and the row trailers on
// delivery too.
// Call after ae350_dcache_flush_all() (only the FPGA writes this memory).
static bool video_frame_complete(uint32_t slot, uint32_t id)
{
    uint32_t h = video_slot_height(slot);
    uint32_t r;

    for (r = 0; r < h; ++r)
    {
        volatile uint32_t *tag =
            (volatile uint32_t *)(VIDEO_ROW_ADDR(slot, r) + VIDEO_ROW_TAG_OFFSET);
        if (tag[0] != (((uint32_t)VIDEO_ROW_TAG_MAGIC << 16) | r) || tag[1] != id)
            return false;
    }
    return true;
}

// The height of the frame in the slot is what the sensor outputs. A new ROI
// height reaches the FPGA writer with a delay (ERRORS 2.11): after an ROI change
// 128 -> 1280 the writer still closes the frame after 128 rows, all rows have
// their own trailers, and the checks below let such a stub through as a whole
// 1080 x 128 frame. The sensor height comes from the camera cache (maintained by
// camera_configure() and video_ensure_roi_locked(); I2C is not allowed here,
// polling runs under the ring lock). A mismatch goes to the UART once per pair
// of values, otherwise ring polling would flood the console.
// Whether there was a frame of the wrong height since the start of the capture
// attempt: a reason to re-arm the camera (video_capture_frames_impl); the height
// will not reach the writer by itself.
static volatile bool s_height_bad[VIDEO_CAM_COUNT] = { false, false };

// The frame height this camera's FPGA writer applied: telemetry word 35,
// [23:12] (cam_channel.v header_ext_hi; on the host vmsp.fpga_frame_lines). The
// block is outside the ring and is written every 0.5 s. If the writer's height
// is larger than the sensor ROI, there is no frame header at all (the frame does
// not start), and the ring cannot show it: on 02.10.2026 2 captures of 16 ended
// this way on an ROI change 1280 -> 128, 12 pulses and not a single header.
static uint32_t video_writer_lines(uint8_t cam_id)
{
    uint32_t diag = VIDEO_DIAG_ADDR(cam_id);
    volatile uint32_t *d = (volatile uint32_t *)diag;

    ae350_dcache_invalidate_range(diag + 32U * 4U, 32);
    return (d[35] >> 12) & 0xFFFU;
}

// The writer is on a different height than the sensor ROI (by the camera
// cache). An unknown sensor height is not a reason.
static bool video_writer_height_wrong(uint8_t cam_id)
{
    camera_params_t p;

    if (camera_cached_params(cam_id, &p) != 0 || p.roi_h < 64U || p.roi_h > VIDEO_FRAME_HEIGHT)
        return false;
    return video_writer_lines(cam_id) != p.roi_h;
}

static bool video_height_matches_sensor(uint8_t cam_id, uint32_t slot, uint32_t id)
{
    static uint32_t s_warned[VIDEO_CAM_COUNT] = { 0, 0 };
    camera_params_t p;
    uint32_t h = video_slot_height(slot);
    uint32_t key;

    if (camera_cached_params(cam_id, &p) != 0 || p.roi_h < 64U || p.roi_h > VIDEO_FRAME_HEIGHT)
        return true;                                 /* sensor height unknown */
    if (h == p.roi_h) return true;
    s_height_bad[cam_id] = true;
    key = (h << 16) | p.roi_h;
    if (s_warned[cam_id] != key)
    {
        s_warned[cam_id] = key;
        printf("[VIDEO] cam %u frame %u: %u rows in the slot, sensor ROI %u - not a whole frame"
               " (writer geometry not applied yet)\r\n", (unsigned int)cam_id, (unsigned int)id,
               (unsigned int)h, (unsigned int)p.roi_h);
    }
    return false;
}

// After a re-arm because of the height: erase the frame headers of the camera
// with a number greater than last_id, i.e. everything taken since the start of
// the attempt (frames of the wrong height and abandoned ones). Without this,
// every next poll saw them again and spent a re-arm at once (02.10.2026: three
// re-arms in 2.3 s, TIMEOUT).
//
// Erasing rather than raising the count to the newest number (as it was until
// 05.10.2026): only a CLOSED frame uses up a number, and the writer rewrites an
// abandoned one under the same number into the same slot
// (video_fire_and_wait). With ROI 1280 -> 128 the writer at 1280 rows does not
// close the frame, and the very first frame after the re-arm came under an
// already "counted" number: 12 pulses, TIMEOUT, while the next capture (ring
// erased, count from 0) got a frame in 66 ms. Writing to the ring is allowed:
// after a re-arm the camera is stopped, there are no pulses.
static void video_ring_drop_newer(uint8_t cam_id, uint32_t last_id)
{
    uint32_t s;

    ae350_dcache_flush_all();
    for (s = 0; s < VIDEO_RING_SLOTS; ++s)
    {
        volatile uint32_t *hdr =
            (volatile uint32_t *)(VIDEO_RING_BASE(cam_id) + (s * VIDEO_SLOT_BYTES));
        if (hdr[0] == VIDEO_FRAME_MAGIC && hdr[1] > last_id) hdr[0] = 0U;
    }
    ae350_dcache_flush_all();
}

// When a slot's frame was first seen whole (MTIME, 10 ns): the anchor for
// extending the 32-bit strobe (docs/OPEN_ISSUES.md 1.4). The strobe in the FPGA
// wraps every 42.9 s, and delivery used to extend its upper bits by the time of
// DELIVERY, assuming the strobe was within +-21.5 s of it. A frame taken from
// the ring later got a timestamp 42.9 s larger than the real one. From the
// moment of detection the strobe is no more than a frame length away.
static uint32_t s_seen_id[VIDEO_CAM_COUNT][VIDEO_RING_SLOTS];
static uint64_t s_seen_mtime[VIDEO_CAM_COUNT][VIDEO_RING_SLOTS];

static uint64_t video_mtime(void)
{
    uint32_t hi, lo;
    do {
        hi = DEV_PLMT->MTIME[1];
        lo = DEV_PLMT->MTIME[0];
    } while (hi != DEV_PLMT->MTIME[1]);
    return ((uint64_t)hi << 32) | lo;
}

bool video_frame_seen_time(uint32_t slot, uint32_t frame_id, uint64_t *mtime)
{
    uint8_t cam = (slot >= (uint32_t)VIDEO_RING_BASE(1)) ? 1U : 0U;
    uint32_t s = (slot - (uint32_t)VIDEO_RING_BASE(cam)) / VIDEO_SLOT_BYTES;

    if (s >= VIDEO_RING_SLOTS || s_seen_id[cam][s] != frame_id || frame_id == 0U)
        return false;
    *mtime = s_seen_mtime[cam][s];
    return true;
}

// The earliest WHOLE frame with a number greater than last_id; *seen tells
// whether there is a header with such a number at all (the frame is being
// written or was abandoned unfinished). For every whole frame remembers the
// time of first detection (video_frame_seen_time).
static bool video_find_complete(uint8_t cam_id, uint32_t last_id,
                                uint32_t *id_out, bool *seen)
{
    bool found = false;
    uint32_t s;

    ae350_dcache_flush_all();
    for (s = 0; s < VIDEO_RING_SLOTS; ++s)
    {
        uint32_t slot = VIDEO_RING_BASE(cam_id) + (s * VIDEO_SLOT_BYTES);
        volatile uint32_t *hdr = (volatile uint32_t *)slot;
        uint32_t id;

        if (hdr[0] != VIDEO_FRAME_MAGIC || hdr[5] != VIDEO_HW_HDR_INTEGRITY) continue;
        id = hdr[1];
        if (id <= last_id) continue;
        *seen = true;
        if (found && id >= *id_out) continue;
        if (!video_height_matches_sensor(cam_id, slot, id)) continue;
        if (video_frame_closed(slot, id) && video_frame_complete(slot, id))
        {
            if (s_seen_id[cam_id][s] != id)
            {
                s_seen_id[cam_id][s] = id;
                s_seen_mtime[cam_id][s] = video_mtime();
            }
            *id_out = id;
            found = true;
        }
    }
    return found;
}

// Software trigger and wait until a WHOLE frame with a number greater than
// *last_id appears in the ring. On success *last_id is its number, *refired
// tells whether there were repeated pulses.
//
// The pulse repeats every VIDEO_REFIRE_MS, up to VIDEO_TRIGGER_FIRES_MAX times.
// If a new header is already there but the frame is not whole, the repeat waits
// another VIDEO_CLOSE_TIMEOUT_MS: the writer may still be finishing. An
// abandoned frame does NOT raise last_id: the writer rewrites the same slot
// with the next exposure under the same number; the FPGA frame number grows
// only on a closed frame. The former version raised last_id and then looked for
// a number that would never come, hence 6-12 pulses after a camera mode change
// (ERRORS former 31.44).
// After a camera mode change (0x0112 before every frame_viewer shot, then
// re-arming) the first pulses give no frame: the camera counts them, but the
// MIPI receiver was knocked out by the sensor stop. The first pulse goes
// through the "receiver locked" branch and is lost, the second through the
// D-PHY hold path, which by design sacrifices a frame, and only the third gives
// a frame (ERRORS former 31.37). The former "pulse, 2 s wait, one repeat" never
// reached the third: 4 failures of 5 on the frame_viewer button path against
// 5 of 5 without 0x0112.
static bool video_fire_and_wait(uint8_t cam_id, uint32_t *last_id, bool *refired)
{
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_fire = t0;
    TickType_t seen_since = t0;
    bool seen_any = false;
    unsigned fires = 0;

    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(VIDEO_TRIGGER_TIMEOUT_MS))
    {
        TickType_t now = xTaskGetTickCount();
        uint32_t id = 0;
        bool seen = false;

        if (fires == 0 ||
            (fires < VIDEO_TRIGGER_FIRES_MAX &&
             (now - last_fire) >= pdMS_TO_TICKS(VIDEO_REFIRE_MS) &&
             (!seen_any || (now - seen_since) >= pdMS_TO_TICKS(VIDEO_CLOSE_TIMEOUT_MS))))
        {
            if (seen_any)
                printf("[VIDEO] capture: partial frame in the ring, trigger again\r\n");
            sync_fire();
            last_fire = now;
            ++fires;
            seen_any = false;
        }
        vTaskDelay(pdMS_TO_TICKS(VIDEO_DETECT_POLL_MS));

        if (video_find_complete(cam_id, *last_id, &id, &seen))
        {
            if (fires > 1)
            {
                *refired = true;
                printf("[VIDEO] capture: frame %u complete after %u trigger(s)\r\n",
                       (unsigned int)id, (unsigned int)fires);
            }
            *last_id = id;
            return true;
        }
        // A frame of the wrong height, or the FPGA writer still on the old ROI height
        // after the pulses (telemetry, with margin for its 0.5 s period): new pulses
        // will give the same; the camera has to be re-armed.
        if (!s_height_bad[cam_id] &&
            (xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(VIDEO_GEOM_CHECK_MS) &&
            video_writer_height_wrong(cam_id))
        {
            s_height_bad[cam_id] = true;
            printf("[VIDEO] cam %u: writer at %u rows after %u ms of triggers, sensor ROI differs\r\n",
                   (unsigned int)cam_id, (unsigned int)video_writer_lines(cam_id),
                   (unsigned int)VIDEO_GEOM_CHECK_MS);
        }
        if (s_height_bad[cam_id])
        {
            *refired = *refired || (fires > 1);
            return false;
        }
        if (seen && !seen_any)
        {
            seen_any = true;
            seen_since = xTaskGetTickCount();
        }
    }
    printf("[VIDEO] capture: no frame after %u trigger(s) in %u ms\r\n",
           (unsigned int)fires, (unsigned int)VIDEO_TRIGGER_TIMEOUT_MS);
    *refired = *refired || (fires > 1);
    return false;
}

// A repeated pulse may have gone out a moment before the frame close became
// visible, and then the camera writes one more frame, and right after it the
// firmware reads the whole ring. We wait VIDEO_REFIRE_MS (the same exposure
// assumption as the repeat): if a new header appeared in that time, we wait for
// that frame to close too.
static void video_wait_quiet(uint8_t cam_id, uint32_t *last_id)
{
    unsigned k;

    for (k = 0; k < VIDEO_TRIGGER_FIRES_MAX; ++k)
    {
        TickType_t t0 = xTaskGetTickCount();
        bool seen = false, done = false;
        uint32_t id = 0;

        // A new header within VIDEO_REFIRE_MS?
        while (!seen && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(VIDEO_REFIRE_MS))
        {
            vTaskDelay(pdMS_TO_TICKS(VIDEO_DETECT_POLL_MS));
            done = video_find_complete(cam_id, *last_id, &id, &seen);
        }
        if (!seen) return;

        // Yes: wait until the writer finishes or abandons it.
        t0 = xTaskGetTickCount();
        while (!done && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(VIDEO_CLOSE_TIMEOUT_MS))
        {
            vTaskDelay(pdMS_TO_TICKS(VIDEO_DETECT_POLL_MS));
            seen = false;
            done = video_find_complete(cam_id, *last_id, &id, &seen);
        }
        if (!done) return;                /* abandoned: the number is not used, the ring is idle */
        printf("[VIDEO] capture: extra frame %u from a repeated trigger\r\n", (unsigned int)id);
        *last_id = id;
    }
}

// The row trailers of the previous session are erased (see
// video_capture_frames_impl). One per camera ring: the erase happens once for each.
static bool s_ring_scrubbed[VIDEO_CAM_COUNT] = { false, false };

// Prepare the camera's ring for capture: erase the slot headers (and once per
// session the row trailers), flush the cache and make sure this camera's writer
// is alive. Call when the camera is armed and there are no pulses yet.
// false: no FPGA telemetry, nothing to capture with.
static bool video_ring_prepare(uint8_t cam_id)
{
    unsigned s;

    // The slot headers are erased so that only a frame taken now counts as whole in
    // the ring: the FPGA frame number starts from 1 after each of its boots, while
    // DDR3 keeps slots of previous sessions with larger numbers. Writing to the ring
    // is allowed: the camera is armed, there are no pulses, the writer is idle.
    for (s = 0; s < VIDEO_RING_SLOTS; ++s)
    {
        volatile uint32_t *hdr =
            (volatile uint32_t *)(VIDEO_RING_BASE(cam_id) + (s * VIDEO_SLOT_BYTES));
        hdr[0] = 0U;
    }
    // Once after the firmware starts, the trailers of all rows are erased as well:
    // after an FPGA reboot its frame numbers start from 1 again, and the previous
    // session's rows in DDR3 would otherwise be indistinguishable from rows of the
    // current frame with the same number (ERRORS former 31.45). Within one session
    // the numbers only grow, and old rows show by their number.
    if (!s_ring_scrubbed[cam_id])
    {
        TickType_t ts = xTaskGetTickCount();
        uint32_t r;
        for (s = 0; s < VIDEO_RING_SLOTS; ++s)
        {
            uint32_t slot = VIDEO_RING_BASE(cam_id) + (s * VIDEO_SLOT_BYTES);
            for (r = 0; r < VIDEO_FRAME_HEIGHT; ++r)
            {
                volatile uint32_t *tag =
                    (volatile uint32_t *)(VIDEO_ROW_ADDR(slot, r) + VIDEO_ROW_TAG_OFFSET);
                tag[0] = 0U;
            }
        }
        s_ring_scrubbed[cam_id] = true;
        ae350_dcache_flush_all();
        printf("[VIDEO] capture: cam %u row trailers of the whole ring cleared in %u ms\r\n",
               (unsigned int)cam_id,
               (unsigned int)((xTaskGetTickCount() - ts) * portTICK_PERIOD_MS));
    }
    ae350_dcache_flush_all();

    // The telemetry block is needed only as a sign that this camera's writer is
    // alive; there is no point waiting for it to be fresh (up to 0.5 s) any more:
    // the frame is detected in the ring. Each camera has its own block: 0x06000000
    // and 0x06000200.
    {
        uint32_t diag = VIDEO_DIAG_ADDR(cam_id);
        volatile uint32_t *d = (volatile uint32_t *)diag;
        ae350_dcache_invalidate_range(diag, 32);
        if (d[0] != VIDEO_FRAME_MAGIC)
        {
            printf("[VIDEO] capture: no FPGA telemetry for cam %u at 0x%08X\r\n",
                   (unsigned int)cam_id, (unsigned int)diag);
            return false;
        }
    }
    return true;
}

static uint32_t video_capture_frames_impl(uint8_t cam_id, uint32_t n, bool laser_on)
{
    uint32_t last_id = 0;
    uint32_t i;
    unsigned rearm;
    bool armed;
    bool refired = false;
    TickType_t t0 = xTaskGetTickCount();

    if (cam_id >= VIDEO_CAM_COUNT) return 0;
    if (n == 0) n = 1;
    if (n > VIDEO_RING_SLOTS - 1U) n = VIDEO_RING_SLOTS - 1U;

    if (!i2c_bus_lock(2000))
    {
        printf("[VIDEO] capture: I2C bus busy, capture skipped\r\n");
        return 0;
    }
    armed = s_armed[cam_id] || video_arm_trigger_locked(cam_id);
    i2c_bus_unlock();
    if (!armed)
    {
        s_armed[cam_id] = false;
        return 0;
    }
    s_armed[cam_id] = true;

    video_sync_prepare(cam_id, laser_on);
    if (!video_ring_prepare(cam_id))
    {
        // Until 30.09.2026 this was an exit without restoring: the neighbouring
        // camera's pulse stayed blanked until the next successful capture.
        video_sync_restore(cam_id);
        return 0;
    }

    // The next pulse only after the previous frame has closed. The count goes from
    // the number of the last closed frame: an extra frame from a repeated pulse must
    // not count as the next one. The ring is erased, so 0 means "any".
    //
    // A frame of the wrong height (video_height_matches_sensor): re-arm the camera;
    // the height reaches the writer only on a D-PHY watchdog reset, and that
    // happens when the camera loses its first exposure after a re-arm (ERRORS 2.11;
    // vmsp.profile_rig_wait_geometry does the same from the host). On 02.10.2026
    // without this the writer stood at 128 rows with the sensor at 1280: TIMEOUT on
    // every capture.
    s_height_bad[cam_id] = false;
    for (i = 0; i < n; ++i)
    {
        if (!video_fire_and_wait(cam_id, &last_id, &refired)) break;
    }
    for (rearm = 1; i < n && s_height_bad[cam_id] && rearm <= VIDEO_GEOM_REARMS; ++rearm)
    {
        printf("[VIDEO] capture: cam %u frame height is not the sensor ROI, re-arming cam 0 (%u of %u)\r\n",
               (unsigned int)cam_id, rearm, (unsigned int)VIDEO_GEOM_REARMS);
        if (!i2c_bus_lock(2000)) break;
        armed = video_rearm_for_height_locked(cam_id);
        i2c_bus_unlock();
        s_armed[0] = armed;                           /* camera 0 is always the one re-armed */
        if (!armed) break;
        if (cam_id != 0U) video_geom_pulse_cam0();    /* its pulse is blanked: give strobes */
        video_ring_drop_newer(cam_id, last_id);       /* stubs from before the re-arm do not count */
        s_height_bad[cam_id] = false;
        for (; i < n; ++i)
        {
            if (!video_fire_and_wait(cam_id, &last_id, &refired)) break;
        }
    }
    if (refired) video_wait_quiet(cam_id, &last_id);
    video_sync_restore(cam_id);
    printf("[VIDEO] capture: %u of %u frame(s) by trigger in %u ms\r\n",
           (unsigned int)i, (unsigned int)n,
           (unsigned int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
    return i;
}

// Capture taking the flags for STATUS and delivery from memory into account.
// Call under video_ring_lock().
uint32_t video_capture_frames(uint8_t cam_id, uint32_t n, bool laser_on)
{
    uint32_t got;

    if (cam_id >= VIDEO_CAM_COUNT) return 0;
    s_frames_ready[cam_id] = false;
    s_capture_busy = true;
    got = video_capture_frames_impl(cam_id, n, laser_on);
    s_capture_busy = false;
    s_frames_ready[cam_id] = (got > 0);
    s_frames_laser[cam_id] = laser_on;
    return got;
}


// ---- Stereo capture: both cameras from one pulse (30.09.2026) -----------------
//
// One sync block pulse goes to both camera channels, camera 1 through the delay
// SYNC_W_CAM1_DELAY set by the operator (default 0: both cameras expose at the
// same time). The firmware does not touch it.
//
// Until 09.10.2026 the delay was raised for the capture to 12 ms plus the
// exposure difference (if camera 0 is longer): on Shared DDR3 both writers
// shared one 100 MHz Lane 4 channel, and overlapping readouts of two frames did
// not fit in it; 7 ms and less gave 50 % whole frames, different exposures
// without the correction gave TIMEOUT (ERRORS 1.12, 1.17). With the AXI4 back
// end (FPGA D7A87A77) each writer has its own port: with delay 0, 603/603 and
// 601/603 frames (ERRORS 1.21, docs/DECISIONS.md R-28), and the raise was removed.
static uint32_t s_st_w0, s_st_w1, s_st_d1;
// How long after a pulse not to touch the rings: both frames are exposed and
// read out (see video_fire_and_wait_stereo).
static uint32_t s_st_settle_ms;

static void video_sync_prepare_stereo(bool laser_on)
{
    uint32_t ctrl  = sync_get(SYNC_W_CTRL);
    uint32_t want  = (ctrl & ~(SYNC_CTRL_SRC_MASK | SYNC_CTRL_LASER_EN)) |
                     SYNC_CTRL_ENABLE;                                 /* CORE = 0 */
    uint32_t e0 = camera_cached_exposure_us(0);
    uint32_t e1 = camera_cached_exposure_us(1);
    uint32_t e  = (e0 > e1) ? e0 : e1;
    uint32_t d1;

    s_st_w0 = sync_get(SYNC_W_CAM0_WIDTH);
    s_st_w1 = sync_get(SYNC_W_CAM1_WIDTH);
    s_st_d1 = sync_get(SYNC_W_CAM1_DELAY);
    d1 = s_st_d1;                                   /* the operator's, no raise */

    if (e == 0U) e = VIDEO_STEREO_EXPOSURE_UNKNOWN_US;
    s_st_settle_ms = (uint32_t)((LANE_TICKS_TO_US(d1) + e) / 1000U) + VIDEO_STEREO_READOUT_MS;

    // Zero width: the channel does not rise at all, and the camera will not capture.
    if (s_st_w0 == 0U) sync_set(SYNC_W_CAM0_WIDTH, (uint32_t)LANE_US_TO_TICKS(VIDEO_STEREO_CAM_WIDTH_US));
    if (s_st_w1 == 0U) sync_set(SYNC_W_CAM1_WIDTH, (uint32_t)LANE_US_TO_TICKS(VIDEO_STEREO_CAM_WIDTH_US));

    // Laser as in video_sync_prepare(), but the pulse covers both exposures: from
    // the start of camera 0's exposure to the end of camera 1's.
    s_laser_forced = laser_on || ((ctrl & SYNC_CTRL_LASER_EN) != 0U);
    if (s_laser_forced)
    {
        s_laser_mode_saved  = sync_get(SYNC_W_LASER_MODE);
        s_laser_width_saved = sync_get(SYNC_W_LASER_WIDTH);
        s_laser_ctrl_saved  = ctrl;
    }
    if (laser_on)
    {
        uint32_t w0 = video_laser_width_ticks(0);
        uint32_t w1 = d1 + video_laser_width_ticks(1);
        uint32_t width = (w1 > w0) ? w1 : w0;

        sync_set(SYNC_W_LASER_MODE, 0U);
        sync_set(SYNC_W_LASER_WIDTH, width);
        want |= SYNC_CTRL_LASER_EN;
        printf("[VIDEO] stereo: laser ON over both exposures, width %u ticks (%u us)\r\n",
               (unsigned int)width, (unsigned int)LANE_TICKS_TO_US(width));
    }

    sync_set(SYNC_W_CTRL, want);
    sync_apply();
    printf("[VIDEO] stereo: exposure %u / %u us, cam1 delay %u us, widths %u / %u us, rings untouched %u ms after a pulse\r\n",
           (unsigned int)e0, (unsigned int)e1,
           (unsigned int)LANE_TICKS_TO_US(d1),
           (unsigned int)LANE_TICKS_TO_US(sync_get(SYNC_W_CAM0_WIDTH)),
           (unsigned int)LANE_TICKS_TO_US(sync_get(SYNC_W_CAM1_WIDTH)),
           (unsigned int)s_st_settle_ms);
    vTaskDelay(pdMS_TO_TICKS(VIDEO_SYNC_APPLY_MS));
}

static void video_sync_restore_stereo(void)
{
    if (s_laser_forced)
    {
        sync_set(SYNC_W_LASER_MODE, s_laser_mode_saved);
        sync_set(SYNC_W_LASER_WIDTH, s_laser_width_saved);
        sync_set(SYNC_W_CTRL, (sync_get(SYNC_W_CTRL) & ~SYNC_CTRL_LASER_EN) |
                              (s_laser_ctrl_saved & SYNC_CTRL_LASER_EN));
        s_laser_forced = false;
    }
    sync_set(SYNC_W_CAM0_WIDTH, s_st_w0);
    sync_set(SYNC_W_CAM1_WIDTH, s_st_w1);
    sync_set(SYNC_W_CAM1_DELAY, s_st_d1);
    sync_apply();
}

// Pulse and wait for a pair: a whole frame with a number greater than last[c]
// in BOTH rings. Repeats follow the same rules as video_fire_and_wait(). A
// repeated pulse gives a new frame to both cameras, so a frame closed before the
// repeat is not paired with a frame after it: the number base is raised to it,
// and the pair is searched anew, from frames of one pulse.
//
// After a pulse the rings are not polled for s_st_settle_ms: while one camera is
// read out, checking the other (a full cache flush and the trailers of 1280
// rows) is a CPU read of DDR3 during writes (the old invariant 5), and on Shared
// DDR3 its burst overflowed the neighbour's FIFO (ERRORS 1.13). The first
// version polled at once, every 5 ms: pairs came together after 5-9 pulses, the
// cameras closed frames every other time and in antiphase (30.09.2026). On the
// AXI4 back end reads during frame writes cost nothing (ERRORS 1.21, CLAUDE.md
// invariant 5); the pause stayed, since the frame cannot close earlier.
static bool video_fire_and_wait_stereo(uint32_t last[VIDEO_CAM_COUNT], bool *refired)
{
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_fire = t0;
    TickType_t seen_since = t0;
    bool seen_any = false;
    bool got[VIDEO_CAM_COUNT] = { false, false };
    uint32_t id[VIDEO_CAM_COUNT] = { 0, 0 };
    uint32_t base[VIDEO_CAM_COUNT];
    unsigned fires = 0;
    uint8_t c;

    for (c = 0; c < VIDEO_CAM_COUNT; ++c) base[c] = last[c];

    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(VIDEO_STEREO_TIMEOUT_MS))
    {
        TickType_t now = xTaskGetTickCount();

        if (fires == 0 ||
            (fires < VIDEO_STEREO_FIRES_MAX &&
             (now - last_fire) >= pdMS_TO_TICKS(VIDEO_REFIRE_MS) &&
             (!seen_any || (now - seen_since) >= pdMS_TO_TICKS(VIDEO_CLOSE_TIMEOUT_MS))))
        {
            if (fires > 0)
                printf("[VIDEO] stereo: pair incomplete (cam0 %s, cam1 %s), trigger again\r\n",
                       got[0] ? "ok" : "no", got[1] ? "ok" : "no");
            for (c = 0; c < VIDEO_CAM_COUNT; ++c)
            {
                if (got[c]) base[c] = id[c];
                got[c] = false;
            }
            sync_fire();
            last_fire = now;
            ++fires;
            seen_any = false;
            vTaskDelay(pdMS_TO_TICKS(s_st_settle_ms));
        }
        vTaskDelay(pdMS_TO_TICKS(VIDEO_DETECT_POLL_MS));

        for (c = 0; c < VIDEO_CAM_COUNT; ++c)
        {
            bool seen = false;
            uint32_t x = 0;

            if (got[c]) continue;
            if (video_find_complete(c, base[c], &x, &seen))
            {
                got[c] = true;
                id[c] = x;
            }
            else if (seen && !seen_any)
            {
                seen_any = true;
                seen_since = xTaskGetTickCount();
            }
        }
        if (got[0] && got[1])
        {
            if (fires > 1)
            {
                *refired = true;
                printf("[VIDEO] stereo: pair %u / %u complete after %u trigger(s)\r\n",
                       (unsigned int)id[0], (unsigned int)id[1], (unsigned int)fires);
            }
            last[0] = id[0];
            last[1] = id[1];
            return true;
        }
        // As in video_fire_and_wait(): a frame of the wrong height or the writer on the
        // old ROI height after the pulses means pulses will not help, the camera has to
        // be re-armed (video_capture_stereo_impl). Until 05.10.2026 stereo did not do
        // this: after ROI 128 -> 1280, 20 wasted pulses and TIMEOUT on every capture
        // until a single-camera capture re-armed it.
        if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(VIDEO_GEOM_CHECK_MS))
        {
            for (c = 0; c < VIDEO_CAM_COUNT; ++c)
            {
                if (got[c] || s_height_bad[c] || !video_writer_height_wrong(c)) continue;
                s_height_bad[c] = true;
                printf("[VIDEO] stereo: cam %u writer at %u rows after %u ms of triggers, sensor ROI differs\r\n",
                       (unsigned int)c, (unsigned int)video_writer_lines(c),
                       (unsigned int)VIDEO_GEOM_CHECK_MS);
            }
        }
        if (s_height_bad[0] || s_height_bad[1])
        {
            *refired = *refired || (fires > 1);
            return false;
        }
    }
    printf("[VIDEO] stereo: no pair after %u trigger(s) in %u ms (cam0 %s, cam1 %s)\r\n",
           (unsigned int)fires, (unsigned int)VIDEO_STEREO_TIMEOUT_MS,
           got[0] ? "ok" : "no", got[1] ? "ok" : "no");
    *refired = *refired || (fires > 1);
    return false;
}

// Erase the headers of whole frames that did not make it into pairs. A repeated
// pulse leaves an extra frame in one camera's ring, and then the "N last whole"
// of the two rings (0x0220) shifted by one: the first version returned a pair
// with camera 1's strobe 30 ms EARLIER than camera 0's (30.09.2026). Writing to
// the ring is allowed: there are no more pulses, the writers are idle (like the
// erase before capture).
static void video_drop_unpaired(uint8_t cam_id, const uint32_t *keep, uint32_t n_keep)
{
    uint32_t s, k, dropped = 0;

    ae350_dcache_flush_all();
    for (s = 0; s < VIDEO_RING_SLOTS; ++s)
    {
        volatile uint32_t *hdr =
            (volatile uint32_t *)(VIDEO_RING_BASE(cam_id) + (s * VIDEO_SLOT_BYTES));
        bool paired = false;

        if (hdr[0] != VIDEO_FRAME_MAGIC) continue;
        for (k = 0; k < n_keep && !paired; ++k) paired = (hdr[1] == keep[k]);
        if (paired) continue;
        hdr[0] = 0U;
        ++dropped;
    }
    ae350_dcache_flush_all();
    if (dropped)
        printf("[VIDEO] stereo: cam %u - %u frame(s) outside the pairs dropped from the ring\r\n",
               (unsigned int)cam_id, (unsigned int)dropped);
}

static uint32_t video_capture_stereo_impl(uint32_t n, bool laser_on)
{
    uint32_t last[VIDEO_CAM_COUNT] = { 0, 0 };
    // static: 256 B are not for the network task's stack; always called under video_ring_lock().
    static uint32_t keep[VIDEO_CAM_COUNT][VIDEO_RING_SLOTS];
    uint32_t i;
    uint8_t c;
    unsigned rearm;
    bool armed = true;
    bool refired = false;
    TickType_t t0 = xTaskGetTickCount();

    if (n == 0) n = 1;
    if (n > VIDEO_RING_SLOTS - 1U) n = VIDEO_RING_SLOTS - 1U;

    if (!i2c_bus_lock(2000))
    {
        printf("[VIDEO] stereo: I2C bus busy, capture skipped\r\n");
        return 0;
    }
    for (c = 0; c < VIDEO_CAM_COUNT && armed; ++c)
    {
        armed = s_armed[c] || video_arm_trigger_locked(c);
        s_armed[c] = armed;
    }
    i2c_bus_unlock();
    if (!armed) return 0;

    video_sync_prepare_stereo(laser_on);
    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        if (!video_ring_prepare(c))
        {
            video_sync_restore_stereo();
            return 0;
        }
    }

    for (c = 0; c < VIDEO_CAM_COUNT; ++c) s_height_bad[c] = false;
    for (i = 0; i < n; ++i)
    {
        if (!video_fire_and_wait_stereo(last, &refired)) break;
        keep[0][i] = last[0];
        keep[1][i] = last[1];
    }
    // The height did not reach the writer: re-arm camera 0 (like
    // video_capture_frames_impl; camera 1 takes the height from it), with normal
    // arming: in stereo both cameras have one ROI, and the mailbox gets camera 0's
    // height. Frames of both cameras from before the re-arm are erased; they do
    // not go into pairs.
    for (rearm = 1; i < n && (s_height_bad[0] || s_height_bad[1]) && rearm <= VIDEO_GEOM_REARMS; ++rearm)
    {
        bool ok;

        printf("[VIDEO] stereo: frame height is not the sensor ROI (cam0 %s, cam1 %s), re-arming cam 0 (%u of %u)\r\n",
               s_height_bad[0] ? "bad" : "ok", s_height_bad[1] ? "bad" : "ok",
               rearm, (unsigned int)VIDEO_GEOM_REARMS);
        if (!i2c_bus_lock(2000)) break;
        ok = video_rearm_for_height_locked(0);
        i2c_bus_unlock();
        s_armed[0] = ok;
        if (!ok) break;
        for (c = 0; c < VIDEO_CAM_COUNT; ++c)
        {
            video_ring_drop_newer(c, last[c]);
            s_height_bad[c] = false;
        }
        for (; i < n; ++i)
        {
            if (!video_fire_and_wait_stereo(last, &refired)) break;
            keep[0][i] = last[0];
            keep[1][i] = last[1];
        }
    }
    // Extra frames from a repeated pulse, in both rings.
    if (refired)
    {
        for (c = 0; c < VIDEO_CAM_COUNT; ++c) video_wait_quiet(c, &last[c]);
    }
    video_sync_restore_stereo();
    // Only paired frames remain in the rings: 0x0220 returns them per camera in the
    // same order, the k-th frame of camera 0 and the k-th of camera 1 come from one
    // pulse.
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) video_drop_unpaired(c, keep[c], i);
    printf("[VIDEO] stereo: %u of %u pair(s) by trigger in %u ms\r\n",
           (unsigned int)i, (unsigned int)n,
           (unsigned int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
    return i;
}

uint32_t video_capture_stereo(uint32_t n, bool laser_on)
{
    uint32_t got;
    uint8_t c;

    for (c = 0; c < VIDEO_CAM_COUNT; ++c) s_frames_ready[c] = false;
    s_capture_busy = true;
    got = video_capture_stereo_impl(n, laser_on);
    s_capture_busy = false;
    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
    {
        s_frames_ready[c] = (got > 0);
        s_frames_laser[c] = laser_on;
    }
    return got;
}

static SemaphoreHandle_t s_ring_mutex = NULL;

bool video_ring_lock(uint32_t timeout_ms)
{
    if (s_ring_mutex == NULL)
    {
        // Lazy creation: the network tasks come here first, seconds after the scheduler
        // starts, but two at once still can.
        vTaskSuspendAll();
        if (s_ring_mutex == NULL) s_ring_mutex = xSemaphoreCreateMutex();
        (void)xTaskResumeAll();
        if (s_ring_mutex == NULL) return false;
    }
    return xSemaphoreTake(s_ring_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void video_ring_unlock(void)
{
    if (s_ring_mutex != NULL) (void)xSemaphoreGive(s_ring_mutex);
}

bool video_frames_ready(uint8_t cam_id)
{
    return (cam_id < VIDEO_CAM_COUNT) && s_frames_ready[cam_id];
}

bool video_frames_had_laser(uint8_t cam_id)
{
    return (cam_id < VIDEO_CAM_COUNT) && s_frames_laser[cam_id];
}

uint32_t video_status_bits(void)
{
    // The "frames in memory" bit for either camera: the status word is one per
    // board, and which camera was captured the host learns from the 0x0220/0x0222
    // response.
    return ((s_frames_ready[0] || s_frames_ready[1]) ? VIDEO_STATUS_FRAME_READY : 0U) |
           (s_capture_busy ? VIDEO_STATUS_CAPTURE_BUSY : 0U);
}

uint32_t video_collect_frames(uint8_t cam_id, uint32_t max_n,
                              uint32_t *slot_addrs, uint32_t *frame_ids)
{
    uint32_t addr[VIDEO_RING_SLOTS];
    uint32_t id[VIDEO_RING_SLOTS];
    uint32_t count = 0;
    uint32_t first, i;
    int s;

    if (max_n == 0 || slot_addrs == NULL || frame_ids == NULL) return 0;
    if (cam_id >= VIDEO_CAM_COUNT) return 0;
    ae350_dcache_flush_all();

    for (s = 0; s < VIDEO_RING_SLOTS; ++s)
    {
        uint32_t slot = VIDEO_RING_BASE(cam_id) + ((uint32_t)s * VIDEO_SLOT_BYTES);
        volatile uint32_t *hdr = (volatile uint32_t *)slot;
        uint32_t frame;
        uint32_t k;
        uint32_t h;
        volatile uint32_t *tag;

        if (hdr[0] != VIDEO_FRAME_MAGIC || hdr[5] != VIDEO_HW_HDR_INTEGRITY) continue;
        frame = hdr[1];
        if (!video_height_matches_sensor(cam_id, slot, frame)) continue;
        h = video_slot_height(slot);
        tag = (volatile uint32_t *)(VIDEO_ROW_ADDR(slot, h - 1) + VIDEO_ROW_TAG_OFFSET);
        // The last row was finished by this very frame; otherwise it is an abandoned
        // frame whose bottom is left from the previous pass over the ring.
        if (tag[0] != (((uint32_t)VIDEO_ROW_TAG_MAGIC << 16) | (h - 1)) ||
            tag[1] != frame)
            continue;
        // And all the other rows too: an abandoned frame without its first rows has the
        // last one (ERRORS former 31.45). The cache was already flushed by flush_all at
        // the start.
        if (!video_frame_complete(slot, frame))
            continue;

        // Insertion in ascending number order.
        k = count;
        while (k > 0 && id[k - 1] > frame)
        {
            id[k] = id[k - 1];
            addr[k] = addr[k - 1];
            --k;
        }
        id[k] = frame;
        addr[k] = slot;
        ++count;
    }

    first = (count > max_n) ? (count - max_n) : 0U;
    for (i = first; i < count; ++i)
    {
        slot_addrs[i - first] = addr[i];
        frame_ids[i - first] = id[i];
    }
    return count - first;
}
