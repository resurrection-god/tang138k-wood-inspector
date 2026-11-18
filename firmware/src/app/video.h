/*
 * video.h - capture by trigger, the frame ring as a shared resource, receive geometry
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_VIDEO_H_
#define APP_VIDEO_H_

#include <stdint.h>
#include <stdbool.h>
#include "memory_map.h"

// Capture by trigger (stage 3): N frames = N software triggers of the sync
// block, each next one after the previous frame has closed. The camera is armed
// for the hardware trigger on the first capture and stays armed; the block's
// source is set to CORE. Returns the number of frames captured (0 = none).
// Call under video_ring_lock(). There is no free-run window any more
// (ERRORS former 31.37).
//
// Waiting for a frame uses the ring itself (17.09.2026): a new VID0 header in
// the ring erased before capture, then the trailer of the last row of that
// slot. Before, it used the frame number in the telemetry block 0x06000000,
// which the FPGA updates every 0.5 s: that was the ~0.5-1 s of the 2 s per
// frame. Small reads of the ring during writes are safe and almost free
// (ERRORS 1.6: ~2 frames per megabyte, and here it is kilobytes per frame).
#define VIDEO_QUIESCE_MS            60U
// Settle time the camera MCU needs after a mode change before it will act on
// the acquisition-start write.
#define VIDEO_MODE_SETTLE_MS        250U
// The FPGA reads the mailbox every ~10 ms; with margin.
#define VIDEO_SYNC_APPLY_MS         50U
// Ring poll step while waiting for a frame.
#define VIDEO_DETECT_POLL_MS        5U
// The frame header appeared but the trailer of the last row did not: the frame
// was abandoned (partial after a camera mode change). The FPGA abandons a frame
// itself after ~0.33 s (FRAME_ABANDON_CYCLES); a normal frame closes within a
// few milliseconds after the header, so 400 ms has margin, and after it a retry
// is allowed.
#define VIDEO_CLOSE_TIMEOUT_MS      400U
// How long to wait for a frame from the first pulse: up to
// VIDEO_TRIGGER_FIRES_MAX pulses every VIDEO_REFIRE_MS plus abandoned frames
// (VIDEO_CLOSE_TIMEOUT_MS) and the exposure.
#define VIDEO_TRIGGER_TIMEOUT_MS    9000U
// Repeat the pulse until the frame closes. Longer than exposure and readout at
// normal settings (10 ms + ~0.1 s); with a longer exposure the camera simply
// does not accept pulses inside it (loss counter 0x0418).
#define VIDEO_REFIRE_MS             600U
// How many pulses are needed after a camera mode change for the MIPI receiver
// to catch the stream again. Measured 16.09.2026 (ERRORS former 31.37),
// 22 attempts on two bitstreams: 4 on average, a tail up to 11. The former limit
// of 5 gave exactly those timeouts "the camera counted all pulses, the FPGA
// frame number stands still": 1-3 failures out of 8.
#define VIDEO_TRIGGER_FIRES_MAX     12U
// The frame in the ring has a height different from the sensor ROI, or the FPGA
// writer is still on the old height after VIDEO_GEOM_CHECK_MS pulses
// (ERRORS 2.11): re-arm the camera this many times before giving up. Like the
// host (vmsp.profile_rig_wait_geometry: 5 attempts of 1 s; 8 changes of 8 done
// in 1..3).
#define VIDEO_GEOM_REARMS           5U
#define VIDEO_GEOM_CHECK_MS         1000U
// laser_on: capture with illumination: for the capture the laser channel is
// switched to pulse (mode 0) and enabled, the width comes from the measured
// exposure, and after the capture the previous laser state is restored. This is
// the `laser_pulse_en` from the body of 0x0220 (STP scenario 4.1).
//
// Width: the FPGA measures the camera strobe window to ~0.2 %
// (ERRORS former 31.47), and the firmware takes the measured value plus margin.
// While there is no measurement (the first capture after reset) or the counter
// is saturated, the width from the mailbox is used. The camera integrates
// light over the whole exposure, so a short pulse is not visible in the frame
// at all: 200 us of light in a 5 ms exposure is indistinguishable from the
// background (ERRORS 4.2, docs/DECISIONS.md R-30).
#define VIDEO_LASER_MARGIN_NUM  5      /* width = strobe * 5/4, 25 % margin */
#define VIDEO_LASER_MARGIN_DEN  4
uint32_t video_capture_frames(uint8_t cam_id, uint32_t n, bool laser_on);

// Stereo capture (30.09.2026): N pulses, each captures both cameras; the next
// one when a whole frame of this pulse has closed in BOTH rings. Returns the
// number of whole pairs. The neighbouring camera is not blanked. The cam1
// channel delay is what the operator set (0x0140; default 0, both cameras
// expose at the same time), and the frames of a pair differ by exactly that in
// strobe (timestamp_us in 0x0221).
// Until 09.10.2026 the delay was raised for the capture to 12 ms plus the
// exposure difference (ERRORS 1.12, 1.17): on Lane 4 (100 MHz) overlapping
// readouts of two cameras did not fit. With the AXI4 back end each camera has
// its own port, and with delay 0 603/603 and 601/603 frames close
// (ERRORS 1.21, docs/DECISIONS.md R-28). Under video_ring_lock().
// Camera pulse width if the channel has 0 (the channel would not rise at all).
#define VIDEO_STEREO_CAM_WIDTH_US   1000U
// After a pulse the rings are not polled for the cam1 delay + the largest
// exposure + this time: frame readout (~5..10 ms) with margin; the frame cannot
// close earlier. On Shared DDR3 polling during the neighbouring camera's
// readout also disturbed it (the old invariant 5, ERRORS 1.13); on the AXI4
// back end it does not, and the pause stayed to save polls.
#define VIDEO_STEREO_READOUT_MS     20U
// Exposure if the firmware does not know it (the camera cache is empty).
#define VIDEO_STEREO_EXPOSURE_UNKNOWN_US  50000U
// The warm-up after arming (VIDEO_TRIGGER_FIRES_MAX: 4 pulses on average, a tail
// up to 11) is needed in stereo by BOTH cameras on the same pulse: on
// 30.09.2026 right after boot camera 1 did not answer any of 12 pulses (no
// strobe, no rows), on another boot the pair came together from the 3rd. Hence
// the larger limit; warmed-up cameras give a pair from the first pulse
// (~145 ms for two pairs).
#define VIDEO_STEREO_FIRES_MAX      20U
#define VIDEO_STEREO_TIMEOUT_MS     15000U
uint32_t video_capture_stereo(uint32_t n, bool laser_on);

// Forget which camera is armed, so the next capture re-arms it. Call after any
// code path that reconfigures the sensor mode behind the capture helper's back.
//
// Also clears the "the ring holds frames of the last capture" flag: a sensor
// started bypassing capture writes the ring again, and on Shared DDR3 the CPU
// was not allowed to read it (the old invariant 5, ERRORS 1.6). Call it
// EVERYWHERE the sensor is started or changes mode outside
// video_capture_frames().
void video_invalidate_trigger_arming(void);
// The same, but for one camera: the other stays armed.
void video_invalidate_trigger_arming_cam(uint8_t cam_id);

// Only the "frames in memory" flag: the camera stayed armed but may have taken
// a frame outside capture (software trigger, sync block source change).
void video_invalidate_frames(void);

// ---- The ring as a shared resource (protocol stage 1, docs/DECISIONS.md R-04) --
//
// Capture and any reading of frames from the ring go under one lock: otherwise
// capture 0x0222 and delivery 0x0220 (or the UART menu) could erase slot headers
// under someone else's send, or read the ring while another task keeps the
// sensor running.
bool video_ring_lock(uint32_t timeout_ms);
void video_ring_unlock(void);

// Whether the ring holds whole frames of the last capture, with the sensor
// stopped since.
bool video_frames_ready(uint8_t cam_id);
// Whether the frames now in this camera's ring were taken with illumination.
// Needed so that a request "frame with laser" does not get a frame without
// laser lying in memory (and vice versa); from outside such a swap is
// indistinguishable.
bool video_frames_had_laser(uint8_t cam_id);

// Up to max_n last whole frames in ascending number order. Whole = a VID0
// header with the integrity constant and a row 1023 trailer with this frame's
// number (the last frame before a stop is abandoned and not finished,
// ERRORS 2.5). Under video_ring_lock(), sensor stopped.
uint32_t video_collect_frames(uint8_t cam_id, uint32_t max_n,
                              uint32_t *slot_addrs, uint32_t *frame_ids);

// The last row of frame frame_id in the slot is finished (the tail of row h-1
// with this number): a quick check without the other rows, for the frame
// stream (GVSP).
bool video_slot_closed(uint32_t slot, uint32_t frame_id);

// Frame streams (GVSP). The frame number in the header of camera slot s (0 = no
// header). The baseline: every slot's number at the stream start - only a slot whose number
// changed since counts as new (after an FPGA reboot numbers restart from 1 while DDR3 keeps the
// old headers, ERRORS 3.14). Newest: the freshest closed frame not in the baseline.
uint32_t video_slot_frame_id(uint8_t cam, uint32_t s);
void video_stream_baseline(uint8_t cam, uint32_t base[VIDEO_RING_SLOTS]);
bool video_stream_newest(uint8_t cam, const uint32_t base[VIDEO_RING_SLOTS], uint32_t *slot_out,
                         uint32_t *id_out);

// MTIME (10 ns) when capture first saw frame frame_id in slot `slot` whole.
// The anchor for extending the 32-bit strobe: the strobe is no further than a
// frame length from it, but any distance from the moment of delivery
// (docs/OPEN_ISSUES.md 1.4). false: this capture did not detect the frame.
bool video_frame_seen_time(uint32_t slot, uint32_t frame_id, uint64_t *mtime);

// STATUS response bits (STP 3.3.2). Bits 0..2 (PLL) are 0: there are no FPGA
// registers on the CPU bus (docs/DECISIONS.md R-13); 3 and 4 are software.
#define VIDEO_STATUS_FRAME_READY    (1U << 3)
#define VIDEO_STATUS_CAPTURE_BUSY   (1U << 4)
uint32_t video_status_bits(void);


// ---- MIPI receive geometry: mailbox words 29..31 -----------------------------
//
// MIPI receive geometry in the same mailbox, words 29..31. The top of the free
// range 21..31 is taken: the sync block grows from the bottom, the LASR block
// starts at 32. Until 22.09.2026 the row length was an FPGA build parameter,
// and the ROI could not be changed without rebuilding the bitstream (ERRORS
// former 31.53; docs/ARCHITECTURE.md § 3.3).
#define GEOM_W_MAGIC            29
#define GEOM_W_SIZE             30    /* {height[31:16], width[15:0]}, 0 = do not change */
#define GEOM_W_SEQ              31
#define GEOM_MAGIC              0x47454F4DU   /* "GEOM" */

// Tell the FPGA what length of rows and how many of them to expect from the
// sensor. A 0 in a field leaves that axis as it was. Applied under a D-PHY
// receiver reset, i.e. with a ~10 ms pause, and only between frames.
void video_geom_set(uint16_t line_bytes, uint16_t frame_lines);

// Frame height from word 4 of the slot header (the FPGA writes {H, source}).
// If the header is not set or is outside 64..1280, returns VIDEO_FRAME_HEIGHT.
uint32_t video_slot_height(uint32_t slot);

#endif /* APP_VIDEO_H_ */
