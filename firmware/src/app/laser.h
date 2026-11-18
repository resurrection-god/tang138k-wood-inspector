/*
 * laser.h - laser line finder: LASR mailbox, telemetry, settings, profile block, calibration
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_LASER_H_
#define APP_LASER_H_

#include <stdint.h>
#include <stdbool.h>
#include "memory_map.h"

// ---- Laser line finder: the mailbox, "LASR" blocks ----------------------------
//
// The same burst 0x06000100 as the TRIG generator (words 0..5) and the sync
// block (6..20). Words 21..28 are left for the sync block to grow, 29..31 are
// the geometry. The finder has its own nine-word block per camera: camera 0 is
// 32..40, camera 1 is 41..49 (since 24.09.2026, stage L2a; before that there
// was one block). The camera 0 FPGA channel reads both (only it has Lane 5) and
// hands camera 1's block to its channel over wires (fpga/src/lane5_reader.v).
//
// The FPGA applies the block's words at once when it reads APPLY_SEQ, the last
// word of the block; the write order is the same as SYNC's. From the THRESH
// word the FPGA takes the threshold [7:0] and the floor [15:8], from GAP [3:0],
// the windows [10:0] (window "to" = 0 means the whole axis), from CTRL
// WRITE_PROF and SOFT. MIN_SUM and WIDTH_MAX were for the firmware only (point
// refinement), the FPGA did not read them. Since 30.09.2026 it does: the FPGA
// computes the point (centre of gravity and flags), block version 2.
//
// ⚠ The FPGA reads the mailbox over Lane 5, and on DDR3-1066 Lane 5 fails from
// boot to boot (ERRORS former 31.44, measurement 6). So everything written here
// must have an echo in telemetry, and the firmware must survive its absence.
#define LASR_W_BASE(cam)        (32U + 9U * (uint32_t)(cam))
#define LASR_O_MAGIC            0
#define LASR_O_CTRL             1
#define LASR_O_THRESH           2    /* [7:0] threshold, [15:8] floor */
#define LASR_O_GAP              3
#define LASR_O_MIN_SUM          4
#define LASR_O_WIN_LO           5
#define LASR_O_WIN_HI           6
#define LASR_O_WIDTH_MAX        7
#define LASR_O_APPLY_SEQ        8    /* changed: apply the block's words */
#define LASR_O_COUNT            9
#define LASR_W_END              LASR_W_BASE(LASER_CAM_COUNT)   /* 50: the first free one */
#define LASR_MAGIC              0x4C415352U   /* "LASR" */
#define LASER_CAM_COUNT         VIDEO_CAM_COUNT

#define LASR_CTRL_MODE_MASK     0x003U   /* [1:0] system mode (WORK_MODE_*)    */
#define LASR_CTRL_KEEP_FRAME    0x004U   /* [2]   write the frame to the slot  */
#define LASR_CTRL_WRITE_PROF    0x008U   /* [3]   write the profile block      */
#define LASR_CTRL_AXIS_SHIFT    4        /* [5:4] search axis (LASER_AXIS_*)   */
#define LASR_CTRL_AXIS_MASK     0x030U
#define LASR_CTRL_CAM1          0x100U   /* [8]   second camera (stage 4)      */
// [10] do not write the frame, profile block only (stage 2, 01.10.2026;
// lane5_reader.v -> LASR bus [52], cam_channel.v dry_frame). Set only by the
// profile stream (laser_no_frame_set), not by the 0x0150 setting: the FPGA does
// not read the setting's KEEP_FRAME flag, and a setting saved without it will
// not turn frames off.
#define LASR_CTRL_NO_FRAME      0x400U
#define LASR_CTRL_SOFT          0x200U   /* [9]   the CPU computes the profile */

// Finder telemetry: words 36 and 37 of the telemetry block of ITS OWN camera
// (0x06000000 / 0x06000200). Two words rather than the six planned: every word
// is a mux input on the lane4_clk path (ERRORS former 31.53).
//   36: {'L' [31:24], APPLY_SEQ [23:16] (low byte), mailbox read [15],
//        write block [14], frame without rows [13] (since 01.10.2026),
//        frameless blocks go to the dense ring [12] (since 19.10.2026), gap [11:8],
//        threshold [7:0]}
//   37: {blocks finished [31:16], points without a line in the last block [15:0]}// In builds before L2a word 36 is the filler 0xA5A55A5A: the top byte is not
// 'L', and that means "the FPGA cannot", not an error.
#define LASR_T_ECHO             36
#define LASR_T_BLOCKS           37
#define LASR_T_ID_BYTE          0x4CU   /* 'L' */

typedef struct {
    bool     valid;                  // the telemetry has the finder words
    bool     loaded;                 // the FPGA read the LASR block from the mailbox
    bool     enable;                 // the FPGA writes the profile block
    bool     no_frame;               // the FPGA does not write the frame rows (stage 2)
    bool     dense;                  // ... and puts such blocks into the dense ring (LASER_DRY_*)
    uint8_t  apply_seq;              // low byte of the accepted APPLY_SEQ
    uint8_t  thresh, gap;
    uint16_t blocks, no_line;
} laser_telemetry_t;

bool laser_read_telemetry(uint8_t cam, laser_telemetry_t *t);
// Wait until the cameras in the mask (bit 0 = camera 0) reflect the last
// APPLY_SEQ. -1 = no finder in the FPGA, 1 = confirmed, 0 = timeout.
#define LASER_ACK_TIMEOUT_MS    800U   /* telemetry every 0.5 s + mailbox poll */
int  laser_fpga_ack(uint8_t cam_mask, uint32_t timeout_ms);

// ---- Laser line finder: settings ----------------------------------------------
#define LASER_AXIS_ROW          0   /* a point per row, search along the row   */
#define LASER_AXIS_COL          1   /* a point per column, search across rows  */

#define LASER_CFG_F_KEEP_FRAME  0x01
#define LASER_CFG_F_WRITE_PROF  0x02
#define LASER_CFG_F_SOFT        0x04   /* the CPU computes the profile, not the FPGA */

// Per camera and one for both WORK/SERVICE modes (decision of 24.09.2026,
// docs/DECISIONS.md R-19: the parameters depend on the camera, optics and
// scene, not on the mode; and one per camera per mode does not fit in the
// store partition).
typedef struct {
    uint8_t  axis;
    uint8_t  thresh;      // brightness threshold 1..255
    uint8_t  gap;         // how many pixels below the threshold close a spot
    uint8_t  flags;       // LASER_CFG_F_*
    uint16_t win_lo;      // search window along the axis, inclusive
    uint16_t win_hi;
    uint16_t min_sum;     // less means "no line"
    uint16_t width_max;   // wider sets bit LASER_Q_WIDE
    uint8_t  offset;      // floor: the sums get I - offset, 0..thresh (L0, ERRORS 4.8)
} laser_cfg_t;

void laser_cfg_get(uint8_t cam, laser_cfg_t *cfg);
// Checks, saves and writes this camera's block + APPLY_SEQ to the mailbox. A VCP_ST_* code.
int  laser_cfg_set(uint8_t cam, const laser_cfg_t *cfg);

// ---- Laser line finder: the profile block in the slot -------------------------
//
// NORMATIVE LAYOUT (docs/ARCHITECTURE.md § 6.2).
// Three sides must keep it identical: this firmware, the RTL (stage L2) and
// tools/laser/laser_block.py. A mismatch, as with frame rows, is silent: the
// block is simply not recognised.
//
// A 1080x1280 frame occupies the slot up to 0x1900FF; 448 KB are free up to the
// end of the 2 MB slot.
//
//   slot + 0x1A0000   block header (256 B)
//   slot + 0x1A0100   records: LASER_POINTS_MAX of 16 B, padded to a burst boundary
//   slot + 0x1A5100   final burst (256 B), the last word is 0x12345678
//
// The "block finished" marker is the final burst, not the header: the order of
// words inside a Lane 4 burst was guaranteed, the order between bursts only by
// the state machine writing them in sequence, and the final one goes physically
// last.
#define LASER_BLOCK_OFF         0x001A0000U
#define LASER_HDR_BYTES         256U
#define LASER_REC_BYTES         16U
// Room for records is for 1280 points, although the FPGA (COL axis) writes
// 1080: so the layout is the same for both axes and FOOT_OFF does not depend
// on the axis.
#define LASER_POINTS_MAX        VIDEO_FRAME_HEIGHT
#define LASER_RECS_BYTES        (((LASER_POINTS_MAX * LASER_REC_BYTES) + 255U) & ~255U)
#define LASER_FOOT_OFF          (LASER_HDR_BYTES + LASER_RECS_BYTES)
// Version 2 (30.09.2026): after the final burst come the points computed by the
// FPGA, a word per point {q[31:24], i[23:16], x_sub[15:0]} (laser_prof.v,
// tools/laser/model.py refine_fixed). The final burst is still written last,
// after the points.
#define LASER_PTS_OFF           (LASER_FOOT_OFF + 256U)
#define LASER_PT_BYTES          4U
// Version 3 (16.10.2026, R-38): after the points the mm words by the "pixel -> mm"
// table (laser_mm.v, mm_table.h): {depth[31:16], along[15:0]}, along = 0x8000 -
// no mm, depth then the reason (1 no line, 2 table off, 3 outside the band, 4
// invalid node). Units are the table's (mm_hdr_t.unit_shift).
#define LASER_MM_OFF            (LASER_PTS_OFF + LASER_POINTS_MAX * LASER_PT_BYTES)
#define LASER_MM_ADDR(slot, i)  (LASER_BLOCK_ADDR(slot) + LASER_MM_OFF + (uint32_t)(i) * 4U)
#define LASER_MM_NONE(reason)   ((((uint32_t)(reason)) << 16) | 0x8000U)
#define LASER_BLOCK_BYTES       (LASER_MM_OFF + LASER_POINTS_MAX * LASER_PT_BYTES)
#define LASER_BLOCK_ADDR(slot)  ((uint32_t)(slot) + LASER_BLOCK_OFF)
#define LASER_REC_ADDR(slot, i) (LASER_BLOCK_ADDR(slot) + LASER_HDR_BYTES + \
                                 (uint32_t)(i) * LASER_REC_BYTES)
#define LASER_PT_ADDR(slot, i)  (LASER_BLOCK_ADDR(slot) + LASER_PTS_OFF + \
                                 (uint32_t)(i) * LASER_PT_BYTES)
// Dense ring: a frame without rows (LASR_CTRL_NO_FRAME) is only its VID0 header and
// block, so the FPGA packs it into a 32 KB slot instead of 2 MB. Since 20.10.2026 the
// ring is its own 256 MB per camera (DENSE_RING_ADDR), 8192 slots, ~8 s at 1000 Hz
// (19.10: 2048 slots inside the frame ring). Frame F goes to slot F mod
// LASER_DRY_SLOTS, the block right after the header (cam_channel.v DRY_SLOT_LOG2 /
// DRY_SLOT_BITS, tools/laser/laser_block.py). Telemetry word 36 bit 12 is read as this
// ring; with a 19.10 bitstream (2048 slots in the frame ring) the stream finds no blocks.
#define LASER_DRY_SLOT_BYTES    0x8000U
#define LASER_DRY_SLOT_BITS     13U
#define LASER_DRY_SLOTS         (1U << LASER_DRY_SLOT_BITS)
#define LASER_DRY_BLOCK_OFF     0x100U
#define LASER_DRY_BLOCK_ADDR(cam, i) (DENSE_RING_ADDR(cam) + (uint32_t)(i) * LASER_DRY_SLOT_BYTES + \
                                      LASER_DRY_BLOCK_OFF)
#define LASER_BLOCK_MAGIC       0x50524F46U   /* "PROF" */
#define LASER_BLOCK_VERSION     3U

// Block header words (uint32).
#define LASER_H_MAGIC           0
#define LASER_H_FRAME_ID        1
#define LASER_H_TIMESTAMP       2    /* hw_timestamp at frame start            */
#define LASER_H_STROBE_TS       3    /* exposure strobe, like slot word 21     */
#define LASER_H_KIND            4    /* {axis 8, version 8, cam_id 8, flags 8} */
#define LASER_H_GEOM            5    /* {points 16, bytes per record 16}       */
#define LASER_H_TUNE            6    /* {threshold 8, gap 8, floor 8, 0 8}     */
#define LASER_H_ROWS            7    /* {rows counted 16, points w/o line 16}  */
#define LASER_H_DROPPED         8    /* rows refused by the FIFO (frame incomplete) */
#define LASER_H_WINDOW          9    /* {window to 16, window from 16}         */
#define LASER_H_PIX_WORDS       10   /* pixel words that passed the finder     */
#define LASER_H_PTS             11   /* version 2: {points offset 16, bytes per point 16} */
#define LASER_H_LIMITS          12   /* version 2: {width_max 16, min_sum 16}  */
#define LASER_H_MM              13   /* version 3: {mm offset 16, bytes per mm word 16} */
#define LASER_H_MM_ID           14   /* table id the mm were computed with     */
#define LASER_H_MM_CRC          15   /* CRC of the band (checked at load)      */
#define LASER_H_MM_OFFS         16   /* {kofs 16, vofs 16}                     */
#define LASER_H_MM_STATE        17   /* [31] mm on, [30] table bad, [28:24] rows, [15:0] points with mm */
#define LASER_H_MM_COUNTS       18   /* {invalid node 16, outside the band 16} */
#define LASER_H_CRC_PTS         19   /* LASER_KIND_F_CRC: CRC-32 of the point words */
#define LASER_H_CRC_MMW         20   /* LASER_KIND_F_CRC: CRC-32 of the mm words    */
// The FPGA block (laser_prof.v) writes bursts in the order "records, points,
// header, final": the number of points without a line is known only after the
// records.

#define LASER_KIND_F_FPGA       0x01   /* the FPGA computed the block, not the CPU */
#define LASER_KIND_F_PARTIAL    0x02   /* the frame was incomplete             */
// Words 19 and 20 are CRC-32 (zlib, words as LE bytes) of the point and mm words,
// computed by the FPGA as it writes them (laser_prof.v, 20.10.2026); the host checks.
#define LASER_KIND_F_CRC        0x04

// The raw point record: what the FPGA can compute on the fly, and exactly what
// the CPU must reproduce bit for bit (docs/ARCHITECTURE.md § 6: RTL acceptance
// is a comparison with this block). There is no division here on purpose.
#pragma pack(push, 1)
typedef struct {
    uint32_t sum_i;      // sum of brightness of the best spot
    uint32_t sum_iv;     // sum of I * coordinate
    uint8_t  max_i;      // spot maximum
    uint8_t  i_minus;    // neighbour left of the maximum
    uint8_t  i_plus;     // neighbour on the right
    uint8_t  flags;      // spot flags from the finder
    uint16_t argmax;     // coordinate of the maximum, integer
    uint16_t n;          // pixels above the threshold in the spot
} laser_rec_t;
#pragma pack(pop)

// STP point (section 4.6): 6 B Big-Endian on the wire, here in host order.
typedef struct {
    uint16_t x_sub;      // coordinate along the search axis, 1/16 pixel
    uint16_t z;          // height, 1/64 mm from the calibration origin
    uint8_t  i;          // brightness
    uint8_t  q;          // quality, LASER_Q_*
} laser_point_t;

#define LASER_Q_NO_LINE         0x01   /* no line: x_sub and z are invalid     */
#define LASER_Q_SATURATED       0x02   /* max_i == 255                         */
#define LASER_Q_WIDE            0x04   /* spot wider than width_max            */
#define LASER_Q_MULTI           0x08   /* there was more than one spot         */
#define LASER_Q_DISAGREE        0x10   /* COG and Gaussian disagree            */
#define LASER_Q_UNCALIB         0x20   /* no calibration: z = 0                */
#define LASER_Q_PARTIAL         0x40   /* the row did not take part            */

#define LASER_X_SUB_SHIFT       4      /* 1/16 pixel */
#define LASER_Z_SHIFT           6      /* 1/64 mm    */

// Block header parsed into host fields.
typedef struct {
    uint32_t frame_id;
    uint32_t timestamp;
    uint32_t strobe_ts;
    uint16_t points;
    uint8_t  axis, version, cam_id, kind_flags;
    uint16_t rows_used, rows_no_line;
    uint32_t rows_dropped;
    bool     has_points;  // version 2 from the FPGA: the points are already computed (laser_block_point)
    // Version 3 (R-38): mm words. mm_on - the block's mm were computed with a valid
    // band (table mm_id, band mm_crc); whether that is the band of the current ROI
    // is mm_table's question (mm_expected).
    bool     has_mm, mm_on, mm_bad;
    uint32_t mm_id, mm_crc;
    uint16_t mm_points;
    uint32_t crc_pts, crc_mmw;  // kind_flags & LASER_KIND_F_CRC, else 0 (laser_block_peek)
} laser_block_hdr_t;

// The block in the slot is finished and belongs to this frame.
bool laser_block_valid(uint32_t slot, uint32_t frame_id, laser_block_hdr_t *hdr);
// One raw record from the block. Returns false outside the block.
bool laser_block_rec(uint32_t slot, uint32_t idx, laser_rec_t *rec);
// One FPGA point (block version 2, hdr.has_points): x_sub, i, q as computed by
// laser_prof.v; z = 0 (there is no calibration in the FPGA, bit LASER_Q_UNCALIB
// is set). The cache was flushed by laser_block_valid(). false: outside the block.
bool laser_block_point(uint32_t slot, uint32_t idx, laser_point_t *pt);

// ---- Profile stream (stage 2, 01.10.2026) -------------------------------------
//
// laser_block_valid() flushes the cache of the whole block (26 KB, ~830 CCTL
// operations); for a stream of 900 profiles/s that is too much. Here is the
// light path: flush only the header and final burst lines, then only the points.
//
// The block is finished and holds FPGA points (version 2), any frame number:
// the stream does not know it in advance. hdr is filled as in
// laser_block_valid(). These take the block address, not the slot: in the dense
// ring (LASER_DRY_BLOCK_ADDR) the block is not at slot + LASER_BLOCK_OFF.
bool laser_block_peek(uint32_t blk, laser_block_hdr_t *hdr);
// Point words of the block {q, i, x_sub} as they lie in memory, n of them.
// Before this, laser_block_peek(); after, laser_block_still(blk, frame_id):
// the FPGA may have rewritten the slot while the points were being read (ring
// overtake).
void laser_block_points_read(uint32_t blk, uint32_t n, uint32_t *dst);
// mm words of a version 3 block (hdr.has_mm), n of them, the same way.
void laser_block_mm_read(uint32_t blk, uint32_t n, uint32_t *dst);
// Point read measurement: cache flush and copy, MTIME clocks (10 ns), cumulative.
extern volatile uint64_t g_laser_pt_t_flush, g_laser_pt_t_copy;
bool laser_block_still(uint32_t blk, uint32_t frame_id);
// Erase one block's markers (the header magic and the final burst): after an FPGA
// reboot its frame numbers start from 1 again, while DDR3 keeps the previous
// session's blocks with larger numbers. The same trick as video_ring_prepare() for
// frame headers; the stream erases every block of its ring.
void laser_block_erase(uint32_t blk);

// Frame without rows, profile block only (LASR_CTRL_NO_FRAME). For the cameras
// in the mask sets or clears the flag, writes their blocks to the mailbox and
// waits for the FPGA echo. 1 = the FPGA confirmed (APPLY_SEQ echo and the
// no_frame bit as ordered), 2 = confirmed and every camera in the mask puts
// frameless blocks into the dense ring (echo bit [12]), 0 = no echo within
// timeout_ms or the wrong bit (a bitstream before stage 2 still writes the
// frame), -1 = no finder in the FPGA.
int  laser_no_frame_set(uint8_t cam_mask, bool on, uint32_t timeout_ms);
// The same flag without waiting for the echo: the FPGA takes it at its next mailbox read
// (~10 ms) and at the next frame's slot open (the guaranteed-delivery gate, gvsp.c).
void laser_no_frame_post(uint8_t cam_mask, bool on);

// ========================= ALGORITHM HOOK POINTS ===========================
//
// Below are the two functions in which the whole detection algorithm lives,
// and not a line of it is anywhere else. Everything else (block layout,
// mailbox, telemetry, opcodes 0x0210/0x0211/0x0150/0x0130, modes) is already
// built around them and works.
//
// 1. laser_profile_build(): compute the profile block from the frame lying in
//    a ring slot and write it there. This is the reference the RTL is then
//    compared with (stage L2). Call under video_ring_lock() with the sensor
//    stopped: it reads a megabyte from the ring (the old invariant 5).
//
// 2. laser_refine(): turn a raw record into an STP point: the division
//    sum_iv / sum_i, Gaussian refinement by max_i / i_minus / i_plus,
//    conversion to Z by the calibration. Deliberately kept out of the RTL
//    (docs/DECISIONS.md R-17): changing the formula in firmware is cheaper than
//    in a bitstream.
//
// Both are stubs now: build writes nothing and returns -1, refine returns a
// point with LASER_Q_NO_LINE. The network path still goes all the way and
// answers correctly; that is what the frame was made for.
int  laser_profile_build(uint32_t slot, uint32_t frame_id);
void laser_refine(const laser_rec_t *rec, uint32_t idx, laser_point_t *pt);
// ==========================================================================

// ---- Calibration (0x0130) -----------------------------------------------------
//
// 72 B per STP 3.4: float[4] fx, fy, cx, cy; float[9] rotation matrix;
// float[3] offset in mm; float[4] laser plane Ax+By+Cz+D=0. Kept in RAM,
// survives a reboot through the calibration partition of settings store (below;
// written by an explicit 0x0160, read at start-up). The SPI flash sector from
// the old plan's stage 5 is not needed (docs/DECISIONS.md R-14).
#define LASER_CALIB_BYTES       72U

bool laser_calib_valid(void);
int  laser_calib_set(const uint8_t *body, uint32_t len);
uint32_t laser_calib_get(uint8_t *out, uint32_t max);


// The default for both cameras is the same numbers as the RTL defaults
// (lane5_reader.v, LASR_DEFAULT): without the mailbox the FPGA searches exactly
// the same way. Since 24.09.2026 (L2a) the axis is COL (a point per column,
// docs/ARCHITECTURE.md § 6.1) and the FPGA computes the profile, not the CPU
// (docs/DECISIONS.md R-18).
#define LASER_CFG_DEFAULT_INIT {                                              \
    LASER_AXIS_COL,          /* axis: a point per column, search across rows  */ \
    40U,                     /* threshold: well above noise, tuned at L0      */ \
    2U,                      /* gap                                           */ \
    LASER_CFG_F_KEEP_FRAME | LASER_CFG_F_WRITE_PROF,                             \
    0U, 0U,                  /* window: 0/0 = the whole axis                  */ \
    64U,                     /* min_sum                                       */ \
    64U,                     /* width_max                                     */ \
    0U                       /* floor                                         */ \
}

// For work_mode.c: value check, writing both cameras' shadows to the mailbox,
// a shadow without a mailbox write, and the apply counter the FPGA echoes back.
int      laser_cfg_check(const laser_cfg_t *cfg);
void     laser_cfg_write(void);
// System mode that goes into the LASR control word (called by work_mode_set()).
void     laser_set_mode(uint8_t mode);
void     laser_cfg_shadow_set(uint8_t cam, const laser_cfg_t *cfg);

#endif /* APP_LASER_H_ */
