/*
 * memory_map.h - memory map: mailbox, FPGA telemetry, frame rings and slot layout.
 * Mirror on the PC: tools/common/memory_map.py; in the FPGA: the writer parameters.
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_MEMORY_MAP_H_
#define APP_MEMORY_MAP_H_

#include <stdint.h>

// ---- Mailbox: the CPU writes, the FPGA reads over Lane 5 ---------------------
//
// The control block the CPU writes and the FPGA reads through Lane 5. It sits
// right after the FPGA telemetry block and outside the ring: a 256-byte burst,
// telemetry occupies 0x06000000..0x060000FF, control is the next burst.
//
// Through memory rather than the bus, because the external AHB slave port
// (0xE8000000) is taken entirely by the Ethernet controller, and cutting into
// its bus means touching the most fragile path of the project
// (docs/DECISIONS.md R-13, docs/ARCHITECTURE.md § 4.4).
//
// The write order matters: CAM_TRIG_W_SEQ is written LAST. Its change is what
// the FPGA treats as an "apply" command, so by then the period, count, width
// and flags must already be in memory. And after the write a cache flush is
// mandatory, otherwise the FPGA reads the old values.
#define MAILBOX_ADDR       0x06000100

// Video Stream Header Magic ("VID0" = 0x56494430)
#define VIDEO_FRAME_MAGIC       0x56494430


// DDR3 Video Ring Buffer Configuration (64 Slots x 2 MB = 128 MB Ring Buffer)
//
// The ring occupies 0x08000000..0x0FFFFFFF, the whole upper half of the
// addressable memory. The core addresses 256 MB (checked by writing and reading
// back over JTAG; the space repeats with a 256 MB period); the bottom holds the
// firmware, the stack and the FPGA telemetry block at 0x06000000, and the upper
// 128 MB used to be unused altogether.
//
// The 256 MB ceiling is not a property of the chip, which has 512 MB. It is set
// by DDR3_ADDR[13:0] in the vendor SoC, see the ROW_WIDTH comment in
// fpga/src/riscv_ae350_soc/custom_ddr3/gwmc_param.v (Shared DDR3 SoC, removed
// 15.10.2026, see commit 7ce963e; the AXI4 back end's IP has its own
// fpga/src/ddr3_axi/gwmc_param.v).
//
// Changes together with NUM_SLOTS and RING_BASE_ADDR in
// fpga/src/scanner_top.v; a mismatch silently breaks the slot search.
//
// A 2 MB slot: the header in megabyte zero, the whole frame in megabyte one.
// The slot was 4 MB while the frame was cut in half to dodge a "faulty bit 19";
// the cause turned out to be the floating pin U5 (A14), and once it was
// declared the memory is linear (ERRORS 1.4, overrides former 31.28 and 31.29).
// The same 128 MB now hold 64 slots.
// Two cameras (18.09.2026): the ring is split in half, 32 slots per camera.
// Camera 0 (connector J15, D-PHY quad M0) is 0x08000000..0x0BFFFFFF, camera 1
// (J16, quad M1) is 0x0C000000..0x0FFFFFFF. Each has its own writer in the FPGA
// and its own telemetry block; the Lane 4 write channel was shared and split by
// an arbiter (lane4_arbiter.v, removed 15.10.2026) until 09.10.2026, now each writer has
// its own AXI port (docs/ARCHITECTURE.md § 3.4, § 4.1). Mirrors:
// RING_BASE_ADDR/NUM_SLOTS of both cam_channel instances (formerly
// fpga_camera_ddr3_streamer) in fpga/src/scanner_top.v and RING_SLOTS/RING_BASES
// in tools/common/memory_map.py (one copy on the PC).
#define VIDEO_RING_BASE_ADDR    0x08000000
#define VIDEO_RING1_BASE_ADDR   0x0C000000
#define VIDEO_RING_SLOTS        32             // per camera, it was 64 for one
#define VIDEO_CAM_COUNT         2
#define VIDEO_SLOT_BYTES        (2048 * 1024)  // 2,097,152 Bytes (2 MB) per slot
#define VIDEO_RING_BASE(cam)    ((cam) ? (uint32_t)VIDEO_RING1_BASE_ADDR \
                                       : (uint32_t)VIDEO_RING_BASE_ADDR)
// Writer telemetry block. For camera 0 it is where it was (host utilities read
// 0x06000000), for camera 1 it is the next burst after the control block.
// Camera 0 telemetry block; it also holds the sync block words (sync.h).
#define VIDEO_DIAG0_ADDR          0x06000000U
#define VIDEO_DIAG1_ADDR        0x06000200U
#define VIDEO_DIAG_ADDR(cam)    ((cam) ? VIDEO_DIAG1_ADDR : (uint32_t)VIDEO_DIAG0_ADDR)

// Slot layout written by cam_channel.v (formerly fpga_camera_ddr3_streamer.v;
// docs/ARCHITECTURE.md § 4.3). Rows are NOT packed end to end: Lane 4 could
// only write whole bursts, so each row is rounded up to a burst boundary. The
// spare words at the end of a row carry a per-row trailer:
//
//   row + VIDEO_ROW_TAG_OFFSET     : {0xA5A5, sensor line index}
//   row + VIDEO_ROW_TAG_OFFSET + 4 : frame counter
//
// That is what makes a misplaced row detectable on any image. A ramp test
// pattern cannot show it - every row of a ramp is identical - and on a live
// scene a displaced row looks just like corrupted content.
//
//   slot + 0                     : 256-byte VID0 header + telemetry
//   slot + VIDEO_ROW_OFFSET(r)   : row r, VIDEO_FRAME_WIDTH pixels
//
// Full sensor frame 1080 x 1280 (17.09.2026): a 1080 B row + an 8 B trailer
// takes a 1280 B stride (5 bursts), the frame is 1 638 400 B. Rows follow each
// other right after the header burst, and the whole 2 MB slot is used:
//
//   slot + 0x100 + r*1280           rows 0..1279
//
// The header used to get megabyte zero and the 1016 x 1024 frame megabyte one;
// the full frame does not fit there. The bottleneck that kept the width at 1016
// (paragraph below) was removed by DDR3-1066: with a 133.3 MHz Lane 4 clock
// fpga/sim gives 0 refused rows at 1080 x 1280 (ERRORS 1.5). The historical
// paragraph about 100 MHz is left as is.
//
// Native TCP services emit the 24-byte VID0 header followed by packed pixels
// extracted from the fixed-stride DDR3 rows.
// The header and the row stride are both rounded to the Lane 4 burst size. Change
// VIDEO_HW_BURST_BYTES together with BURST_LOG2 in cam_channel.v - they
// describe the same slot layout from two sides.
//
// 256-byte bursts and a 1016-pixel width, not 64 and 1080, and the two go
// together. A row has to be written inside one sensor line period (~3.7 us); at
// 64-byte bursts and 1080 pixels it takes 9.5 us and roughly half the lines are
// lost, which is the banding of ERRORS 2.5. At 256-byte bursts and a 100 MHz
// lane clock a row is four bursts and takes 3.32 us. 1080 would need a fifth,
// mostly-empty burst (1088 pads to 1280) and does not fit.
//
// 1016 rather than 1024 leaves two spare words in the 256-word stride for the
// trailer above, at no bus cost. Total width given up against the sensor's
// 1080: 64 columns. See ERRORS former 31.8 and 31.10.
//
// A 256-byte burst was tried once before and recorded as worse (1 boot in 3),
// but that was at 62.5 MHz with the old row addressing and no explanation was
// ever found - see the BURST_LOG2 comment in the RTL before trusting either
// result over the other.
#define VIDEO_FRAME_WIDTH       1080
#define VIDEO_FRAME_HEIGHT      1280
#define VIDEO_HW_BURST_BYTES    256
#define VIDEO_HW_HDR_BYTES      VIDEO_HW_BURST_BYTES
// Pixels plus the 8-byte trailer, rounded up to a burst, like ROW_BURSTS in the RTL.
#define VIDEO_ROW_STRIDE        (((VIDEO_FRAME_WIDTH + 8 + (VIDEO_HW_BURST_BYTES - 1)) \
                                  / VIDEO_HW_BURST_BYTES) * VIDEO_HW_BURST_BYTES)  // 1280

// Offset of row r from the slot base. Mirror of row_slot_off in cam_channel.v
// (formerly fpga_camera_ddr3_streamer.v); change only together with it and
// with PIX_BASE_OFF.
//
// 🪦 There used to be a carry of bit 19 into bit 20 here, a workaround for
// "faulty" memory. The losses were caused by the floating pin U5 (A14); the
// memory is linear (ERRORS 1.4).
#define VIDEO_PIX_BASE_OFF      0x00000100U   /* right after the header burst */
#define VIDEO_ROW_OFFSET(r)     (VIDEO_PIX_BASE_OFF + (uint32_t)(r) * VIDEO_ROW_STRIDE)
#define VIDEO_ROW_ADDR(slot, r) ((uint32_t)(slot) + VIDEO_ROW_OFFSET(r))

// Byte offset of the per-row trailer inside a row's stride.
#define VIDEO_ROW_TAG_OFFSET    VIDEO_FRAME_WIDTH
#define VIDEO_ROW_TAG_MAGIC     0xA5A5

// Word 5 of the hardware slot header; written last, so it doubles as a
// "this slot is complete" marker (cam_channel.v, formerly
// fpga_camera_ddr3_streamer.v).
#define VIDEO_HW_HDR_INTEGRITY  0x12345678

// "pixel -> mm" table of each camera (mm_table.c, R-38): 44 436 B written by the
// CPU, read by the FPGA's band loader over Lane 5 (lane5_reader.v). In the free
// space after the rings, 1 MB per camera. Mirror: MM_TABLE_ADDRS in
// tools/common/memory_map.py.
#define MM_TABLE0_ADDR          0x10000000U
#define MM_TABLE1_ADDR          0x10100000U
#define MM_TABLE_ADDR(cam)      ((cam) ? MM_TABLE1_ADDR : MM_TABLE0_ADDR)

// Watchdog record (watchdog.c, 21.10.2026): why the last WDT reset happened. CPU-only,
// 64 B after the udp_tx result; it survives a WDT reset (the SoC resets, DDR3 does not).
#define WATCHDOG_REC_ADDR       0x06000700U

// Dense ring of frames without rows (20.10.2026, laser.h LASER_DRY_*): 256 MB per
// camera, 8192 slots of 32 KB, in the upper half of the 1 GB the CPU and the FPGA
// both see (ERRORS 1.22). Mirrors: DENSE_BASE_ADDR of both cam_channel instances in
// fpga/src/scanner_top.v, DENSE_BASES in tools/common/memory_map.py, DENSE_BASE in
// fpga/sim/tb_streamer.v.
#define DENSE_RING0_ADDR        0x20000000U
#define DENSE_RING1_ADDR        0x30000000U
#define DENSE_RING_ADDR(cam)    ((cam) ? DENSE_RING1_ADDR : DENSE_RING0_ADDR)

#endif /* APP_MEMORY_MAP_H_ */
