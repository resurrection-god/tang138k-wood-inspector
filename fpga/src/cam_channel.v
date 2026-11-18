// ============================================================================
// Module      : cam_channel
// One camera channel: receive, frame write into the DDR3 ring over AXI4, telemetry.
// Platform    : Tang Mega 138K Pro / Gowin GW5AST-138B / Andes AE350 (AXI4, axi_mux)
//
//   cam_rx.v        D-PHY, watchdog, CSI-2 bridge, packer, FIFO (mipi_byte_clk)
//   this file       writer: VID0 header + telemetry, frame rows
//                   by sensor row index, camera strobe, geometry
//   cam_axi_wr.v    AXI4 write port: words -> 256-bit beats, burst queue
//   lane5_reader.v  Lane 5: mailbox and trailer check (camera 0 only)
//   trigger_gen.v   trigger series generator              (camera 0 only)
//
// Since 09.10.2026 the writer talks to axi_mux itself: the Lane 4 port (wr_go, wr_en,
// wr_done, wr_wait) and the l4_axi_wr adapter behind it are removed (AXI4 migration,
// stage 5, ERRORS 1.24). A burst is still an FSM burst (header, row, telemetry,
// profile); cam_axi_wr collects its words into beats and issues it on AXI whole.
//
// Until 24.09.2026 all of this was one file, fpga_camera_ddr3_streamer.v; it was split
// without any logic change (step 4): the fpga/sim bench gives the same Lane 4 /
// Lane 5 trace to the cycle and to the bit (tb_streamer +TRACE).
// ============================================================================

`timescale 1ns / 1ps

module cam_channel #(
    parameter [31:0] RING_BASE_ADDR = 32'h0800_0000, // Base DDR3 Address
    // 64 slots x 2 MB = 128 MB. The ring sits at 0x08000000..0x0FFFFFFF, which
    // is the whole upper half of what the CPU can address and was previously
    // untouched by anything.
    //
    // The slot was 4 MB while the frame was cut in half to get around "the faulty
    // address bit 19" (ERRORS former 31.28). The bit had nothing to do with it: pin U5 -
    // A14 of both chips - was not declared, it floated and picked up coupling from the
    // neighbouring BGA address lines, the chip opened a foreign row and the write went
    // past the array. With U5 declared and tied to zero the memory is linear (ERRORS
    // 1.4), the frame lies contiguously, the slot went back to 2 MB, and there are twice
    // as many slots.
    //
    // Measured, not assumed (JTAG writes and reads back in a later session):
    // distinct words survive at 0x08100000, 0x0A000000, 0x0C000000, 0x0F000000
    // and 0x0FFFFFF0, and the address space repeats with a period of 256 MB -
    // 0x16100000 reads back what was written to 0x06100000. 256 MB is exactly
    // what ROW_WIDTH = 14 spans, so the ceiling is the vendor SoC's
    // DDR3_ADDR[13:0] and not the DRAM, which holds 512 MB (see the note in
    // custom_ddr3/gwmc_param.v).
    //
    // slot_idx is 7 bits, so NUM_SLOTS may go to 127 - but not the ring: 64
    // slots of 2 MB already fill the free space exactly, and slot 64 would land
    // on 0x10000000, which aliases 0x00000000 and would overwrite the firmware.
    parameter [6:0]  NUM_SLOTS      = 7'd32,
    // Telemetry lives outside the ring and stays where the host tools expect it.
    // It used to be derived as RING_BASE_ADDR + 64 MB, which only worked while
    // the ring was 64 MB long and based at 0x02000000; with a 128 MB ring that
    // formula lands inside the ring itself.
    parameter [31:0] DIAG_ADDR      = 32'h0600_0000,
    // Dense ring of frames without rows (dry_frame): 8192 slots of 32 KB = 256 MB
    // of its own, above everything else (20.10.2026). Must be 256 MB aligned.
    parameter [31:0] DENSE_BASE_ADDR = 32'h2000_0000,

    // Trigger control block: the processor writes, the FPGA reads through Lane 5.
    //
    // Why through memory and not through the bus: the external AHB slave port
    // (AHB_EXT_BASE = 0xE8000000) is taken entirely by the Ethernet controller - it both
    // returns data and drives HREADY. Cutting a decoder in there means touching the
    // path that broke most often in this project (ERRORS former 31.20; 3.1).
    // Reading from DDR3, on the other hand, the writer already has, and it is debugged.
    //
    // It lies right after the telemetry: a 256-byte burst (BURST_LOG2 = 6), telemetry
    // occupies DIAG_ADDR..+0xFF, so the control block sits at +0x100. Both blocks are
    // outside the ring, so a write here does not break invariant 5 - it is one word
    // beside the ring, not a walk over the slots.
    parameter [31:0] CFG_ADDR       = 32'h0600_0100,

    // Geometry, burst and clock defaults are the ones set in scanner_top.v,
    // so that a module built without overrides (bench, lint) is the same build as on
    // the board. Why exactly 1016 - the comment at FRAME_WIDTH there.
    //
    // The parameters MID_FRAME_PAUSE_CYCLES (a mid-frame pause) and ROW_DECIMATION
    // (one row in N per exposure) were removed on 16.09.2026: both were experiments
    // against row losses whose cause was found - the undeclared pin U5
    // (ERRORS 1.4) - and on the board both stood at their neutral value. Results of
    // the experiments - ERRORS former 31.7 (summary of conclusions in the archive; the
    // raw log is in git history).
    parameter [15:0] FRAME_WIDTH    = 16'd1080,     // + 8-byte trailer + 192-byte pad = 1280-byte stride
    parameter [15:0] FRAME_HEIGHT   = 16'd1280,     // 1280 rows

    // Words per Lane 4 burst, as a power of two. 6 -> 64 words -> 256 bytes.
    //
    // A parameter so the testbench can sweep it: the overhead per burst is roughly
    // fixed (~19 cycles), so a longer burst is the only way to raise the fraction
    // of the bus that carries payload. The vendor's own master uses 128 words.
    //
    // Since 29.09.2026 this is the size of the header, telemetry and profile bursts
    // and the step of the row layout (the 1280 B stride = 5 x 256 B does not change).
    // A row is written by ROW_BURST_LOG2 bursts (below) and a short tail.
    parameter integer BURST_LOG2 = 6,

    // Row burst, a power of two: 7 -> 128 words, like the vendor's master
    // (test_ddr3_rw.v in DDR3_Shared). A row is written without padding: 1080 B +
    // 8 B trailer = 272 words = 128 + 128 + 16, and the 48 padding words up to the
    // stride are no longer sent (29.09.2026). With a 100 MHz Lane 4 clock (memory 400,
    // invariant 2) five bursts of 64 words take ~4.15 us per row against ~3.7 us of a
    // sensor row; 128 + 128 + 16 take ~3.3 us. Must be >= BURST_LOG2.
    // Header and telemetry bursts stay at 64 words: a 512 B burst at
    // DIAG_ADDR 0x06000000 would overwrite the mailbox at 0x06000100.
    parameter integer ROW_BURST_LOG2 = 7,

    // FIFO depth, as a power of two. 13 -> 8192 words.
    //
    // The sensor's average output is only ~10.5 MB/s at 1024x1024 Mono8 @10 fps -
    // twenty times less than this writer sustains. What does not fit is the SHAPE:
    // the array is read out in a ~3.75 ms burst at ~280 MB/s and the sensor is
    // then silent for 96 ms. Buffer the burst and the average is comfortable, and
    // no faster memory clock is needed at all.
    //
    // The buffer that would take is large: the writer drains ~420 KB during the
    // burst against ~1080 KB arriving, so the deficit is ~660 KB. Free BSRAM on
    // this device is 292 of 340 blocks, about 657 KB - the same order, which is
    // why this is worth measuring rather than dismissing.
    //
    // Read ERRORS former 28.2 / 28.7 (archive summary) before raising it: at 16384 the loss did not improve and at
    // 32768 the design stopped building - the Gray pointer chains reach 16 bits in
    // both domains and mipi_byte_clk fell to 153.093 MHz against 161.290. That was
    // a timing failure in the pointer arithmetic, not a capacity one, so it is
    // fixable in principle - but it is the known blocker, not a footnote.
    parameter integer FIFO_ADDR_BITS = 13,

    // Frequency of `clk` (Lane 4 = DDR3_MEMORY_CLK/4), in Hz.
    //
    // Every timeout and heartbeat period in this module is derived from it. They
    // used to be written out as literal cycle counts with the frequency only in
    // a comment, so raising the memory clock silently stretched or shrank all of
    // them at once - the telemetry heartbeat, the MIPI watchdog and both frame
    // abandon timers - and each had to be found and re-derived by hand.
    parameter integer LANE_CLK_HZ = 133_333_333,

    // Diagnostic: 1 replaces every pixel word with {row, word-in-row} while
    // leaving the FIFO, the FSM and the bus traffic exactly as they are.
    //
    // It splits the capture path in two. A frame captured with this on is
    // generated entirely inside the writer, so if it comes back corrupted the
    // fault is at or after the Lane 4 write; if it comes back clean while a real
    // frame does not, the fault is upstream in the D-PHY, the CSI-2 bridge or
    // the packer. Nothing else distinguishes those two halves - the row content
    // is the only thing that changes, so timing, addresses and FIFO behaviour
    // stay identical and the comparison is honest.
    parameter integer WRITE_TEST_PATTERN = 0,

    // Idle threshold of the D-PHY timer watchdog in clk cycles. 0 - 3 s, as on the
    // board; a non-zero value is needed only by the bench (tb_streamer +WD_IDLE).
    parameter [31:0] WD_IDLE_CYCLES = 32'd0,

    // Lane 5 read channel: 1 - mailbox polling and row trailer check by
    // read-back, 0 - the channel is not used at all.
    //
    // Zero is needed for the SECOND writer instance (camera 1,
    // docs/ARCHITECTURE.md § 3.4): there is one mailbox per board, there is no reason
    // for both to read it, and there is no arbiter on Lane 5 - unlike Lane 4, where the
    // channel was shared by necessity. With zero the read FSM never leaves
    // V_IDLE: rd_go stays low, the read bus is free for master 0.
    parameter integer ENABLE_LANE5 = 1,

    // Camera number for the profile block (the cam_id field of the header).
    parameter [7:0]   CAM_ID = 8'd0,

    // 1 - laser line finder and profile block (laser_prof.v); 0 - none of them
    // at all, the channel writes only the frame, as before L2a. Telemetry words 36..37
    // then show zero block counters.
    parameter integer ENABLE_LASER = 1
)(
    // System Clock & Reset (DDR3 Domain)
    input  wire        clk,                 // Lane 4 clock = DDR3_MEMORY_CLK/4, 100 MHz
    input  wire        rst_n,               // System Reset (soc_rst_n)
    input  wire        system_ready,        // System ready from Power-On Sequencer

    // CPU Control & Configuration Inputs
    input  wire        cam_enable,          // 1: Enable Camera Stream, 0: Standby
    input  wire        cam_port_sel,        // 1: Connector 1 (Cam 1 / J11), 0: Connector 0 (Cam 2 / J12)

    // Physical MIPI Differential Pairs (Port 1 / J11 Header)
    inout  wire        cam1_ck_p,
    inout  wire        cam1_ck_n,
    inout  wire        cam1_d0_p,
    inout  wire        cam1_d0_n,
    inout  wire        cam1_d1_p,
    inout  wire        cam1_d1_n,

    // AXI4 write master: one port of axi_mux (cam_axi_wr.v). 256-bit data,
    // INCR, full width; AxSIZE, AxBURST and the ID are axi_mux's.
    output wire          m_awvalid,
    input  wire          m_awready,
    output wire [29:0]   m_awaddr,
    output wire [7:0]    m_awlen,
    output wire          m_wvalid,
    input  wire          m_wready,
    output wire [255:0]  m_wdata,
    output wire [31:0]   m_wstrb,
    output wire          m_wlast,
    input  wire          m_bvalid,
    output wire          m_bready,
    input  wire [1:0]    m_bresp,

    // Lane 5 - the read channel of the mailbox poll. Since 30.10.2026 it is
    // answered by apb_ctl (the mailbox moved from DDR3 to APB registers), before
    // that by l5_axi_rd from DDR3; the row read-back check it also carried is gone
    // (ERRORS 1.7).
    //
    // Same presentation rule as Lane 4: addr and burstcount are driven only
    // while rd_go is high, zero otherwise (vendor's test_ddr3_rw.v). rd_go is
    // ONE cycle here, not two.
    output wire        rd_go,
    output wire [7:0]  rd_burstcount,
    output wire [31:0] rd_addr,
    output wire        rd_en,
    input  wire        rd_rdy,
    input  wire        rd_valid,
    input  wire [63:0] rd_data,

    // Camera hardware trigger (connector PMOD2, camera J2).
    //
    //   cam_trig   -> A18, camera input "Trigger In",  3.3 V
    //   cam_strobe <- A17, camera output "Strobe Out", 3.3 V
    //
    // The trigger is formed here, not by the processor: period and pulse count are
    // given in clk cycles (lane4_clk), so the jitter is one cycle, not a FreeRTOS
    // delay. The processor puts the parameters into DDR3 at CFG_ADDR, from where
    // this module periodically reads them through Lane 5.
    output wire        cam_trig,
    input  wire        cam_strobe,

    // Sync block (sync_block in the top level, docs/ARCHITECTURE.md § 5).
    // Control - words 7..20 of the same CFG_ADDR mailbox (word 6 is the
    // "SYNC" magic), state - words 23..30 of the header/telemetry. There are no registers
    // on the AHB bus: they did not get along with the MAC timing (ERRORS former 31.35).
    output wire [14*32-1:0] sync_mbx,     // [0] CTRL ... [12] APPLY_SEQ [13] SW_SEQ
    output wire             sync_loaded,  // a mailbox with the "SYNC" magic has been read at least once
    input  wire [7:0]       sync_status,
    input  wire [31:0]      sync_accepted,
    input  wire [31:0]      sync_overrun,
    input  wire [31:0]      sync_ext_edges,
    input  wire [31:0]      sync_last_ts,

    // Diagnostics & Hardware Indicators
    output reg  [6:0]  o_current_slot,   // 64 ring slots -> 7 bits
    output reg  [31:0] o_frame_counter,
    output wire        o_frame_done_irq,    // Tied to 1'b0 (Interrupt-Free)
    output wire        o_cam_pixel_seen,    // Activity indicator for LED
    output wire        o_mipi_byte_clk,
    // o_wants_bus ("I need the Lane 4 bus" for lane4_arbiter) was removed on 09.10.2026
    // together with the arbiter: every writer has its own axi_mux port.

    // Laser line finder parameters (the bus is described at lasr_cfg below).
    // The channel with Lane 5 reads them from the mailbox itself and hands camera 1's
    // block out; the channel without Lane 5 (camera 1) takes them from the input.
    input  wire [95:0] lasr_cfg_in,
    output wire [95:0] lasr1_cfg_out,

    // "pixel -> mm" table (docs/DECISIONS.md R-38): the loader lives in the channel
    // with Lane 5 (lane5_reader.v) and serves camera 1 by wire, like LASR.
    //   *_tw  - band writes {en, bank[1:0], addr[8:0], data[31:0]}
    //   *_cfg - {status 64 (o_mmN_stat), laser_prof's mm_cfg 128}
    input  wire [43:0]  mm_tw_in,
    input  wire [191:0] mm_cfg_in,
    output wire [43:0]  mm1_tw_out,
    output wire [191:0] mm1_cfg_out,
    input  wire         mm1_ack_in,       // camera 1's acknowledge, to the loader here
    output wire         mm_ack_out,       // this channel's acknowledge (camera 1 -> camera 0)
    // band words from the CPU (apb_ctl) and the loader state for it (lane5_reader.v)
    input  wire         mm_bw_v,
    input  wire [31:0]  mm_bw_d,
    output wire [31:0]  mm_ld_stat,

    // Frame geometry (height H, rows 64..1280):
    // The channel with Lane 5 reads it from the GEOM mailbox and hands it out on
    // geom_lines_out; the channel without Lane 5 (camera 1) takes it on geom_lines_in.
    input  wire [15:0] geom_lines_in,
    output wire [15:0] geom_lines_out
);

    // =========================================================================
    // 1-4. Receive: D-PHY, watchdog, bridge, packer, FIFO - cam_rx.v
    // =========================================================================
    localparam integer LINE_IDX_BITS = 12;

    // Receive geometry at run time (docs/ARCHITECTURE.md § 3.3).
    //
    // The default is the build parameters, so right after reset the behaviour is
    // exactly as before. The mailbox may override the height; the writer takes it
    // at the next slot it opens, see "Applying the receive geometry" below.
    reg [15:0] cfg_line_bytes;
    reg [15:0] cfg_frame_lines;
    assign geom_lines_out = cfg_frame_lines;

    wire        wd_reset;
    wire        fifo_rd_en;
    wire [32+2+LINE_IDX_BITS-1:0] fifo_rd_data;
    wire [FIFO_ADDR_BITS:0]       fifo_rd_used;
    wire [31:0] diag_header_count, diag_line_count, diag_bad_wc_count;
    wire [15:0] diag_last_wc;
    wire [1:0]  diag_phase;
    wire        diag_overflow;
    wire [31:0] diag_seen_wc;
    wire [15:0] diag_seen_hits;
    wire [15:0] diag_lines_drop;
    wire [15:0] diag_byte_clk_ticks;

    cam_rx #(
        .FRAME_WIDTH    (FRAME_WIDTH),
        .FRAME_HEIGHT   (FRAME_HEIGHT),
        .FIFO_ADDR_BITS (FIFO_ADDR_BITS),
        .LANE_CLK_HZ    (LANE_CLK_HZ),
        .WD_IDLE_CYCLES (WD_IDLE_CYCLES),
        .LINE_IDX_BITS  (LINE_IDX_BITS)
    ) u_rx (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .cam_enable            (cam_enable),
        .cam_strobe            (cam_strobe),
        .cam1_ck_p             (cam1_ck_p),
        .cam1_ck_n             (cam1_ck_n),
        .cam1_d0_p             (cam1_d0_p),
        .cam1_d0_n             (cam1_d0_n),
        .cam1_d1_p             (cam1_d1_p),
        .cam1_d1_n             (cam1_d1_n),
        .cfg_line_bytes        (cfg_line_bytes),
        .cfg_frame_lines       (cfg_frame_lines),
        .o_mipi_byte_clk       (o_mipi_byte_clk),
        .o_cam_pixel_seen      (o_cam_pixel_seen),
        .o_wd_reset            (wd_reset),
        .fifo_rd_en            (fifo_rd_en),
        .fifo_rd_data          (fifo_rd_data),
        .fifo_rd_used          (fifo_rd_used),
        .o_diag_header_count   (diag_header_count),
        .o_diag_line_count     (diag_line_count),
        .o_diag_bad_wc_count   (diag_bad_wc_count),
        .o_diag_last_wc        (diag_last_wc),
        .o_diag_phase          (diag_phase),
        .o_diag_overflow       (diag_overflow),
        .o_diag_seen_wc        (diag_seen_wc),
        .o_diag_seen_hits      (diag_seen_hits),
        .o_diag_lines_drop     (diag_lines_drop),
        .o_diag_byte_clk_ticks (diag_byte_clk_ticks)
    );

    // Bursts actually completed for the frame most recently closed out.

    // Heartbeat telemetry ----------------------------------------------------
    // Telemetry normally rides in burst 0 of a frame, which is useless while
    // bringing the pipeline up: if the receiver accepts no lines, no frame ever
    // starts and nothing is ever written, so there is nothing to read back. This
    // writes one 64-byte telemetry burst to a fixed address outside the ring
    // roughly twice a second, whatever else is happening.
    localparam [31:0] DIAG_PERIOD    = LANE_CLK_HZ / 2;                // 0.5 s
    reg [25:0] diag_hb_cnt;
    reg        diag_burst;        // this burst is telemetry, not pixel data
    // Profile block burst (laser_prof.v): like diag_burst - outside the frame rows,
    // the profile module supplies the address and the words.
    reg        prof_burst;

    // =========================================================================
    // Frame geometry, in 32-bit words and whole bursts.
    //
    // A row is stored on a burst boundary rather than packed end-to-end. The
    // Lane 4 protocol can only write whole bursts, so a row that is not a whole
    // number of bursts would straddle them and there would be no point at which
    // the writer could re-synchronise to a line boundary. Rounding each row up to
    // ROW_BURSTS bursts (1016 -> 1024 bytes, the last 8 carry the row trailer)
    // makes every row independently addressable and independently recoverable.
    //
    // NOTE: FRAME_WIDTH must be a multiple of 4 (32-bit FIFO words).
    // =========================================================================
    // Burst size, in 32-bit words. Everything below is derived from it.
    //
    // 64 words (256 bytes). The Lane 4 handshake costs a fixed ~14-20 cycles per
    // burst - wr_go 2, the mandatory idle, waiting for wr_done, the inter-burst
    // gap - so at 16 words more than a third of the writer's time was overhead and
    // it could not keep up with the 296.9 MB/s two MIPI lanes deliver.
    //
    // 64 words once collapsed on hardware (1 boot in 3), and this note used to say
    // "do not raise this before the address fault is fixed". The faults were found
    // - a lane clock that was not DDR3_MEMORY_CLK/4 (ERRORS 1.1) and the floating U5
    // pin (ERRORS 1.4) - and 64 words with the 400 MHz memory clock is what reaches
    // full-frame coverage (ERRORS former 31.8, 31.10).
    localparam [7:0]  BURST_WORDS      = 8'd1 << BURST_LOG2;           // 64 words = 256 B
    localparam integer BURST_ADDR_LOG2 = BURST_LOG2 + 2;               // 256 bytes -> 8 address bits

    localparam [15:0] ROW_WORDS        = FRAME_WIDTH >> 2;             // 254 payload words for 1016 B
    localparam [15:0] BURST_WORDS16    = {8'd0, BURST_WORDS};
    localparam [15:0] ROW_BURSTS       = (ROW_WORDS + BURST_WORDS16 - 16'd1) >> BURST_LOG2;
    localparam [15:0] ROW_TOTAL_WORDS  = ROW_BURSTS << BURST_LOG2;     // 256 words = 1024 B stride
    localparam [31:0] ROW_PAD_WORD     = 32'hA5A5_5A5A;               // fills the tail of the last burst

    // Byte distance between the start of row r and row r+1 in a slot.
    localparam [31:0] ROW_STRIDE_BYTES = {16'd0, ROW_TOTAL_WORDS} << 2;

    // Per-row trailer, written into the padding at the end of each row.
    //
    // A row is rounded up to whole bursts, and if FRAME_WIDTH is chosen to leave
    // at least two spare words there, they carry where the row believes it
    // belongs instead of the constant pad:
    //
    //   word ROW_WORDS     = {A5A5, 0, sensor line index}
    //   word ROW_WORDS + 1 = frame counter
    //
    // This is what makes a misplaced row visible on ANY image. A ramp test
    // pattern cannot show it - every row of a ramp is identical, so any
    // permutation of rows looks perfect - and on a live scene a displaced row is
    // easy to mistake for corrupted content. With the trailer the host can say
    // which sensor line each DDR3 row actually holds, spot rows left over from a
    // previous frame, and reject a frame instead of trusting it.
    //
    // The magic doubles as the row-alignment check that the old 0xA5A55A5A
    // padding used to provide and that was lost when a 1024-wide row started
    // filling four 64-word bursts exactly.
    //
    // At FRAME_WIDTH = 1016 a row is 254 payload words in a 256-word stride, so
    // the two trailer words cost 8 of 1024 columns and no bus time at all.
    // Selected with a SINGLE BIT of row_word, not by comparing it against two
    // constants. Equality comparators here cost more than this design can pay:
    // the first version of this trailer used `row_word == ROW_WORDS` and
    // `row_word == ROW_WORDS + 1`, and PnR came back with lane4_clk down from
    // 128.4 to 106.2 MHz and ae350_ahb_clk at 100.068 - a build that would not
    // boot. docs/WORKFLOWS.md § 3 states the rule outright: keep magnitude comparators
    // out of this FSM's combinational cone, register the flag or use one counter bit.
    //
    // ROW_WORDS is even, so bit 0 of row_word distinguishes the two words and
    // `row_word >= ROW_WORDS` is a test the pad path already needed.
    //
    // Full frame (17.09.2026): at FRAME_WIDTH = 1080 the tail is not two words
    // but 50 (270 payload words in a 320-word stride). This used to demand
    // exactly two, so HAS_ROW_TAIL went false, the whole tail became padding, the
    // Lane 5 verifier stopped checking and the firmware - which finds a closed
    // frame by the last row's trailer - would never have seen one. fpga/sim
    // caught it (3840 of 3840 rows "trailer wrong"). Now any tail of two or more
    // words carries the trailer, and the same bit-0 select simply repeats
    // {tag, frame counter} over the rest of it instead of the pad word - still no
    // comparator in the cone. The host reads only the first two tail words.
    localparam         HAS_ROW_TAIL   = (ROW_TOTAL_WORDS >= ROW_WORDS + 16'd2)
                                        && (ROW_WORDS[0] == 1'b0);
    localparam [15:0]  ROW_TAG_MAGIC  = 16'hA5A5;

    // Row without padding (29.09.2026). Only the pixels and the two trailer words
    // are written - ROW_WRITE_WORDS, 272 at width 1080; the stride in memory stays
    // ROW_TOTAL_WORDS (320), so the slot layout, the row offset formula in the
    // firmware and on the host and all tools do not change, and words 272..319 of a
    // row are simply not written (the previous memory contents stay there). Row bursts
    // are ROW_BURST_LOG2 (128 words), the last one shorter: 128 + 128 + 16. Each
    // burst starts on a 256 B boundary (0, 512, 1024 from the row start), as
    // before. The trailer lies in the tail burst on the same words 14..15 from its
    // start as with 64-word bursts - the Lane 5 verifier (reads 64 words from the
    // start of the row's last burst) needs no change.
    localparam [15:0]  ROW_WRITE_WORDS   = HAS_ROW_TAIL ? (ROW_WORDS + 16'd2) : ROW_TOTAL_WORDS;
    localparam [15:0]  ROW_BURST_WORDS16 = 16'd1 << ROW_BURST_LOG2;
    localparam [15:0]  ROW_TAIL_WORDS    = ROW_WRITE_WORDS & (ROW_BURST_WORDS16 - 16'd1);   // 0 - no tail
    localparam [15:0]  ROW_TAIL_START    = ROW_WRITE_WORDS -
                                           ((ROW_TAIL_WORDS != 16'd0) ? ROW_TAIL_WORDS : ROW_BURST_WORDS16);
    localparam [15:0]  ROW_WRITE_BURSTS  = (ROW_WRITE_WORDS + ROW_BURST_WORDS16 - 16'd1) >> ROW_BURST_LOG2;
    localparam [7:0]   ROW_BURST_LEN     = ROW_BURST_WORDS16[7:0];                           // 128
    localparam [7:0]   ROW_TAIL_LEN      = (ROW_TAIL_WORDS != 16'd0) ? ROW_TAIL_WORDS[7:0]   // 16
                                                                     : ROW_BURST_LEN;
    localparam [31:0]  ROW_BURST_BYTES   = {16'd0, ROW_BURST_WORDS16} << 2;                  // cur_addr step within a row

    // Bytes in one burst, i.e. how far cur_addr walks between two bursts of a row.
    //
    // It is NO LONGER the offset of row 0: the header now occupies a separate
    // megabyte of the slot, and the pixels start at PIX_BASE_OFF (ERRORS former 31.28).
    //
    // NOT a hard-coded 64. That is the right number only while a burst is 16
    // words, and at 64 words it silently placed both the first row and every
    // burst inside a row at a quarter of their proper spacing - rows landed in
    // the right place but their second, third and fourth bursts overwrote the
    // first one's tail. The bench caught it as payload words carrying the wrong
    // index from word 16 onward.
    localparam [31:0] BURST_BYTES = {24'd0, BURST_WORDS} << 2;

    // A FIFO word carries the sensor's own line index alongside the pixel data.
    //
    // Without it the writer can only count the rows it has written, and a row
    // refused at FIFO admission then shifts every following row up by one - which
    // is what turns lost lines into the stacked bands of ERRORS 2.5 instead of
    // holes. 12 bits covers any FRAME_HEIGHT this sensor can produce (1280).
    // LINE_IDX_BITS = 12 is declared above, at the cam_rx instance.
    localparam integer FIFO_WIDTH    = 32 + 2 + LINE_IDX_BITS;   // data, 2 tags, index
    localparam integer FIFO_IDX_LSB  = 34;

    // The FIFO row admission threshold and why FIFO depth does not cure row losses -
    // in cam_rx.v at FIFO_ADMIT_LIMIT. Here only the width of the fill counter
    // is needed.
    localparam integer FIFO_CNT_W = FIFO_ADDR_BITS + 1;


    // Words that must be buffered before a 64-byte burst may start. A burst pops
    // exactly 16 words back to back with no way to stall, so this is the minimum.
    //
    // Raising it to 64 was tried, on the theory that the residual defect - short
    // runs where the gradient advances four times too slowly, i.e. one 32-bit word
    // written several times - was the read path running dry. It is not: with 64 the
    // defect got MUCH worse (609 of 1024 rows affected instead of ~290, and
    // correlation against the reference fell from 97.74% to 86.06%). A deeper
    // backlog makes the writer issue its bursts in tighter clusters, so the real
    // mechanism is contention on the DDR3 bridge, not FIFO starvation. Measurements
    // in ERRORS 1.9 and former 28.2 (archive summary); do not raise this again without addressing
    // the burst cadence first.
    //
    // 29.09.2026: a row burst is now ROW_BURST_LOG2 (128 words) and pops up to 128
    // words from the FIFO back to back, so the threshold follows the longest burst,
    // otherwise a burst would start with a draining FIFO and finish with foreign words.
    localparam [23:0] BURST_FIFO_LOW = {8'd0, ROW_BURST_WORDS16};

    // 🪦 There used to be a pause between Lane 4 bursts here (BURST_GAP_CYCLES = 5; "24
    // cycles by contract" had no source and booted 0/3 - ERRORS 1.2). With the AXI port
    // (09.10.2026) there is no pause: a burst starts as soon as the FIFO has words and
    // cam_axi_wr has room (axi_room). The FIFO flags are fresh by then: ST_BURST_END
    // after the last word lasts one cycle, and the fifo_has_burst / fifo_has_tail
    // registers already see all of the burst's pops in it.

    // How long ST_BURST_WAIT / ST_ROW_SYNC wait for data before giving up on the
    // frame. Must stay comfortably longer than the sensor's inter-frame gap
    // (~96 ms at 10 fps) or every blanking period restarts the slot - see the
    // note in ST_BURST_WAIT.
    localparam [31:0] FRAME_ABANDON_CYCLES = LANE_CLK_HZ / 3;   // ~0.33 s

    // =========================================================================
    // 5. High-Throughput DDR3 Streamer FSM
    // 64-byte burst streaming with 24-byte hardware header
    // =========================================================================
    

    // Burst 0 is entirely header/telemetry, then FRAME_HEIGHT rows of ROW_BURSTS
    // bursts each. The header lies at the start of the slot, and row r at offset
    // PIX_BASE_OFF + r * ROW_STRIDE_BYTES (row_slot_off, linear since ERRORS 1.4);
    // see the layout at PIX_BASE_OFF.
    //
    // Zero-extend each operand with an explicit concatenation to force a 32-bit
    // multiply. FRAME_WIDTH/FRAME_HEIGHT are 16-bit parameters and multiplying them
    // in a 16-bit context silently truncates: 1280 * 1024 = 20 * 65_536 wraps to 0,
    // which once made BURSTS_PER_FRAME = 0 and limited a whole "frame" to 64 bytes.
    // Do NOT rewrite this as `localparam integer A = FRAME_WIDTH;` and multiply
    // those: the Gowin front-end keeps the 16-bit width through that form.
    //
    // It is kept although nothing reads it any more. It used to fill telemetry
    // word 6, which was a mistake of a different kind - an elaborated constant
    // reported back as though it were a measurement - and word 6 now carries the
    // writer's own count of the rows it opened. The declaration stays because it
    // is the worked example the comment above is about, and because the number
    // the host compares that count against is this one.
    /* verilator lint_off UNUSEDPARAM */
    localparam [31:0] BURSTS_PER_FRAME = 32'd1 + ({16'd0, ROW_WRITE_BURSTS} * {16'd0, FRAME_HEIGHT});
    /* verilator lint_on UNUSEDPARAM */

    // Words 0..15 of burst 0 are the VID0 header + capture telemetry (the whole
    // first 64-byte burst). Pixel data therefore starts at burst 1.
    // It was 16, exactly the number of words the mux described. Now 23: seven
    // trigger generator status words were added to the original sixteen (see
    // HDR_EXT_BASE). Words 0..15 are unchanged, so host tools that read 64 bytes
    // keep seeing the same thing.
    //
    // Raising the bound is mandatory: words past HEADER_WORDS are forced to
    // ROW_PAD_WORD, and without this edit a mux extension simply never reaches
    // memory - it reads back as 0xA5A55A5A.
    //
    // 31 (15.09): words 23..30 - sync block status (see header_ext).
    // 34 (17.09): words 31..33 - Lane 5 health (ERRORS former 31.44); with 31 they
    // read as filler, exactly as the paragraph above warns.
    // 35 (22.09): word 34 - row header candidates BEFORE the ECC check.
    // 36 (23.09): word 35 - receive geometry as the FPGA sees it, plus its apply
    // counter, packed into one word (two words broke lane4_clk timing).
    // 38 (24.09): words 36..37 - laser line finder, two words rather than the six
    // planned in laser.h: every word is an input of this mux on the
    // word_in_burst -> wr_data path (ERRORS former 31.53).
    // 40 (16.10.2026): words 38..39 - the "pixel -> mm" table loader (R-38).
    localparam [BURST_LOG2:0] HEADER_WORDS = 40;

    // =========================================================================
    // Slot layout: the frame lies in one contiguous piece (ERRORS 1.4)
    //
    // 🪦 There used to be a workaround for a "faulty address bit 19" here: the frame
    // was cut in half, the halves went into the lower 512 KB of two megabytes, and
    // the slot grew to 4 MB. Disproved. Writes were lost not because of the memory
    // or bit 19 but because of pin U5 (A14 of both chips), which was not declared
    // in the .cst at all, floated in Hi-Z and picked up crosstalk from neighbouring
    // BGA address lines: the chip latched a false one on A14 and opened the wrong
    // row. With U5 declared and tied hard to zero the memory is linear over the
    // whole space - 0 errors over 524 288 words of the 16 MB test (ERRORS 1.4,
    // overrides former 31.28 and 31.29).
    //
    //   slot + 0x000000            VID0 header + telemetry (one burst, 256 B)
    //   slot + 0x000100 + r*stride rows 0..FRAME_HEIGHT-1
    //
    // Full frame 1080 x 1280 (17.09.2026): a 1080 B row + trailer = 1280 B stride,
    // a 1 638 400 B frame, and it did not fit the old layout "header in megabyte
    // zero, pixels in megabyte one". Pixels now follow the header burst directly,
    // and the whole 2 MB slot is used. The host still finds the header at the slot
    // base. Geometry requirement:
    // 0x100 + FRAME_HEIGHT * ROW_STRIDE_BYTES <= 2 MB.
    localparam integer SLOT_LOG2     = 21;               // 2 MB per slot
    localparam [31:0]  PIX_BASE_OFF  = 32'h0000_0100;    // pixels right after the header

    // Declared ahead of slot_base_address, which reads it - same reason as
    // fifo_overflow_sticky above.
    reg [6:0]  slot_idx;

    // slot_base_address is below dry_frame, which selects the ring (dense ring).

    // The row offset from the slot base is computed below, next to fifo_line_idx:
    // Verilog resolves the identifier at its point of use, and Icarus flatly
    // refuses to parse a reference to a wire not yet declared (ERRORS former 31.6).

    // Byte address of the burst being written, set in ST_GO and held until the
    // next ST_GO: cam_axi_wr reads it with the burst's first word, and the
    // row-complete branch hands it to the Lane 5 verifier. (On Lane 4 it was
    // presented only while wr_go was high, as the vendor's test_ddr3_rw.v did -
    // CLAUDE.md invariant 3, gone with the port on 09.10.2026.)
    reg [31:0] burst_addr;

    // Address of the burst about to be issued, carried in a register rather than
    // recomputed from a burst counter.
    //
    // It used to be slot_base + burst_idx * 64, i.e. derived from how many bursts
    // had been written. That is what made a refused line shift every later row up
    // by one and stack three exposures into one slot (ERRORS 2.5). Now a row's base
    // comes from the sensor's OWN line index, carried through the FIFO, and
    // cur_addr simply walks forward inside the row. A line the FIFO refuses leaves
    // a hole at its own address and disturbs nothing after it.
    reg [31:0] cur_addr;

    // Burst length (29.09.2026): header, telemetry and profile use BURST_WORDS
    // (64), a row uses ROW_BURST_LEN (128) and the tail ROW_TAIL_LEN (16).
    // Set in ST_GO together with burst_addr as burst_last = length - 1, for the
    // end of burst in ST_WRITE (compare with a register, no arithmetic in the cone).
    reg [ROW_BURST_LOG2-1:0] burst_last;

    // Burst words for cam_axi_wr: one per clock, registered (wd_first is the first
    // word of the burst, whose address is burst_addr; wd_last is the last).
    reg        wd_en, wd_first, wd_last;
    reg [31:0] wd_data;
    wire       axi_room;            // cam_axi_wr: the next burst will fit
    wire [15:0] axi_aw_cnt, axi_b_cnt;
    wire [2:0]  axi_bresp_err;

    reg [31:0] frame_counter;
    reg [31:0] hw_timestamp;
    reg [15:0] burst_idx;
    // burst_nz_q == (burst_idx != 0) and row_nl_q == (row_word < ROW_WORDS - 1),
    // written next to every burst_idx / row_word assignment one clock early.
    // The 16-bit compares used to sit on the pixel FIFO read enable (CEB) and on
    // the wd/row_word clock enables: the worst clk_x1 paths in B4 (RTL_RULES 3.1).
    // The sim-only check at the end of the module proves both copies every clock.
    reg        burst_nz_q;
    reg        row_nl_q;
    reg [ROW_BURST_LOG2-1:0] word_in_burst;
    reg [1:0]  go_cnt;
    reg [2:0]  state;

    // Row tracking. row_word counts the words written into the current row,
    // including the padding that rounds it up to a whole number of bursts.
    reg [15:0] row_idx;
    reg [15:0] row_word;

    // The sensor line index this row belongs to, latched when the row opens.
    // Taken from the same FIFO word the destination address is computed from, so
    // the trailer says exactly what the addressing believed.
    reg [LINE_IDX_BITS-1:0] row_line_idx;
    reg        row_start_pending;   // the next burst begins a row -> resynchronise first
    reg        need_frame_start;    // ... and that row must be line 0 of a frame
    reg [15:0] sync_pops;           // words discarded by the current resynchronisation
    reg [15:0] diag_resync_cnt;     // rows that needed more than the expected single pop

    // Two counters that measure what diag_bursts_done only appeared to.
    //
    // diag_bursts_done is {burst_idx} + 1 sampled where the frame ends, and
    // burst_idx advances once per burst along the frame, so on any frame that
    // reaches the end it reads BURSTS_PER_FRAME and can read nothing else. It is
    // a restatement of the geometry, not a measurement, and "bursts written
    // 4097/4097" was therefore never evidence that 4097 bursts were written.
    //
    //   frame_rows_cnt  row openings: 1 for the header sync + 1 per row
    //   frame_jump_cnt  rows whose sensor line index was not the previous + 1
    //
    // The second is the one that can explain stale rows on a frame where
    // everything else reads clean. A row's address comes from its sensor line
    // index, so 1024 rows completing does not mean 1024 distinct addresses were
    // written: if the index sequence repeats or skips, some rows are written
    // twice and others keep what the slot held a whole ring revolution ago -
    // which is exactly the multiple-of-32 ageing seen in ERRORS 2.3.
    //
    // Both count the frame being written; the diag_* copies hold the frame just
    // finished and reach the host in the NEXT slot's header. Neither saturates:
    // they wrap, because a comparator per counter in this block is what cost the
    // design a build once already (see the HAS_ROW_TAIL note).
    //
    // frame_go_cnt / frame_done_cnt (wr_go strobes and the wr_done that answered
    // them) and wait_to_cnt (ST_WAIT timeouts) went with Lane 4 on 09.10.2026;
    // their words now carry cam_axi_wr's AW/B counts and BRESP errors.
    reg [10:0] frame_rows_cnt, diag_frame_rows;
    reg [7:0]  frame_jump_cnt, diag_frame_jumps;

    // Frames given up on: both FRAME_ABANDON_CYCLES escapes reset burst_idx and
    // return to ST_IDLE without advancing slot_idx, so the slot is rewritten from
    // row 0 and every row the abandoned attempt never reached still holds the
    // previous revolution's pixels. Cumulative since reset, wrapping at 256.
    reg [7:0]  diag_abandons;


    // ---- Camera strobe and trigger generator --------------------------------
    //
    // Declared here rather than next to their logic: the header word mux below
    // reads them, and Icarus requires declaration before use (ERRORS former 31.6).
    // The same applies to the wires from lane5_reader.v and trigger_gen.v: their
    // instances sit at the end of the file.
    //
    // The strobe timestamp is taken from hw_timestamp, the same counter that goes
    // into header word 2. So the host can subtract one from the other and learn
    // how much earlier than the frame write its exposure happened. Before, there
    // was no such link at all: the frame write time was known, the capture time
    // was not.
    reg [31:0] strobe_ts;              // hw_timestamp at the last strobe edge
    reg [23:0] strobe_width;           // strobe width in clk cycles
    reg [7:0]  strobe_count;           // number of strobes seen, wraps
    reg [31:0] strobe_ts_pub;          // ... the same, published at the frame boundary
    reg [23:0] strobe_width_pub;
    reg [7:0]  strobe_count_pub;

    // Parameters read from the control block in DDR3 (lane5_reader.v).
    wire [31:0] cfg_period;            // trigger period in clk cycles
    wire [7:0]  cfg_count;             // 1..64 pulses; 0 = continuous mode
    wire [31:0] cfg_width;             // pulse width in clk cycles
    wire [31:0] cfg_seq;               // command counter: changed means start
    wire [31:0] cfg_flags;             // bit 0 = invert, bit 1 = stop
    wire        cfg_magic_ok;          // the magic was found in the block read
    wire [31:0] geom_size;             // {height[31:16], width[15:0]}, 0 = do not change
    wire [31:0] geom_seq;              // geometry apply counter
    // Driven by the geometry apply block below; declared here because the header
    // word mux reads them (ERRORS former 31.6).
    reg [31:0] geom_seq_seen;
    reg        geom_pending;           // cfg_frame_lines not yet taken by the writer (wr_frame_lines)

    // Generator state (trigger_gen.v) for header words 16..20.
    wire [31:0] trig_pulses;           // pulses issued, cumulative
    wire [7:0]  trig_left;             // pulses still to start
    wire        trig_run;              // burst in progress
    wire        trig_cont;             // continuous mode
    wire        trig_active;           // pulse is high right now
    wire [31:0] cfg_seq_seen;          // command the FPGA has accepted

    // One cycle at frame end, from the write FSM: publish and restart.
    reg        frame_end_q;

    // ---- Laser line finder (laser_prof.v, stage L2a) -------------------------
    // Declared before the header mux that reads them (ERRORS former 31.6).
    // Finder parameter bus (one camera), 96 bits:
    //   [7:0] threshold  [15:8] floor  [19:16] gap  [30:20] window from
    //   [41:31] window to (inclusive)  [42] write block  [43] mailbox read
    //   [51:44] APPLY_SEQ, low byte (echoed in telemetry)
    //   [52] do not write the frame, profile block only (stage 2, 01.10.2026)
    //   [79:64] min_sum  [95:80] width_max (points in the FPGA, 30.09.2026)
    wire [95:0] lasr_cfg;
    // "pixel -> mm" table of this camera: laser_prof's bus, loader status, band
    // writes, acknowledge (lane5_reader.v, laser_prof.v).
    wire [127:0] mm_cfg;
    wire [63:0]  mm_stat;
    wire         mm_tw_en;
    wire [1:0]   l5_tw_en;                 // the loader's per-camera write enables
    wire [1:0]   mm_tw_bank;
    wire [8:0]   mm_tw_addr;
    wire [31:0]  mm_tw_data;
    wire         mm_ack;
    // Telemetry words 38..39 from a register: for camera 1 the loader's state comes
    // from the camera 0 channel, and the header mux is on the word_in_burst -> wr_data
    // path.
    reg  [63:0]  mm_tel_q;
    always @(posedge clk)
        mm_tel_q <= {8'h4D, mm_stat[39:32], mm_cfg[33], mm_stat[40], mm_stat[41], mm_cfg[34],
                     7'd0, mm_cfg[32:28], mm_stat[31:0]};
    wire        prof_busy, prof_ready;
    wire [31:0] prof_addr, prof_q;
    wire [15:0] prof_blocks, prof_no_line;
    // Frame close pulses and values for the profile module: writer registers, set
    // in the same place where the frame telemetry is published.
    reg         lz_frame_start, lz_close;
    reg  [31:0] lz_frame_id, lz_block_base;
    reg  [10:0] lz_rows;

    // ---- Frame without rows: profile block only (stage 2, 01.10.2026) --------
    //
    // Profile stream to the host: frame rows in DDR3 are not needed, the block
    // is. A row still goes through the whole FSM path - ST_BURST_WAIT, ST_GO,
    // ST_WRITE, ST_BURST_END - the FIFO is drained on the same clocks and the
    // finder sees the same words (lz_valid below does not know about the mode),
    // but the burst is "dry": words do not go to cam_axi_wr (wd_en low), and the
    // FSM does not wait for room there. The VID0 header (burst 0) and the profile
    // block are written as usual.
    //
    //   dry_frame - frame mode, latched at slot open from lasr_cfg[52]: a setting
    //               change mid-frame does not split it;
    //   dry_row_q - "the current burst is a dry row" = dry_frame && burst_idx != 0,
    //               registered: one bit enters the FSM cone (docs/WORKFLOWS.md
    //               § 3, ae350_ahb_clk margin). It lags by a clock, and that is
    //               safe: burst_idx changes in ST_BURST_END and in ST_IDLE, where
    //               it is 0 anyway. Inside a row (k -> k+1) the flag does not
    //               change; after the header (0 -> 1) the FSM goes through
    //               ST_ROW_SYNC, and in the first clock of ST_BURST_WAIT the stale
    //               0 only demands room (axi_room), and by ST_ROW_SYNC the flag is
    //               fresh; dry_frame changes only in ST_IDLE with burst_idx = 0.
    wire        cfg_no_frame = (ENABLE_LASER != 0) && lasr_cfg[52];
    reg         dry_frame, dry_row_q;

    // Dense ring (19.10.2026): a dry frame is only its header and profile block
    // (31.5 KB), so it takes a 32 KB slot instead of 2 MB. Since 20.10.2026 the
    // dense ring is its own 256 MB at DENSE_BASE_ADDR, 8192 slots, ~8 s at 1000 Hz
    // (it was 2048 inside the frame ring, ~2 s, and one 10-minute run at
    // 2 x 1000 Hz still lost a lap behind TCP stalls, ERRORS 1.25). Frame F goes
    // to slot F mod 8192: no second counter, and the firmware knows the slot from
    // the frame number (laser.h LASER_DRY_*, tools/laser/laser_block.py).
    // dry_frame changes only at slot open (ST_IDLE), so a frame's header and block
    // land in the same ring. Telemetry word 36 bit 12 says dry frames go to the
    // dense ring. A word naming the ring (base, slot bits) cost clk_x1 its margin
    // on 20.10.2026 (word 40 past HEADER_WORDS: +0.287 / +0.102 ns from scratch), so
    // the firmware of that day takes bit 12 as this ring; a 19.10 bitstream with the
    // 2048-slot ring and newer firmware find no blocks at all - loud, not wrong.
    localparam integer DRY_SLOT_LOG2 = 15;
    localparam integer DRY_SLOT_BITS = 13;
    localparam [31:0]  DRY_BLOCK_OFF = 32'h0000_0100;   // right after the VID0 header
    localparam [31:0]  BLOCK_OFF     = 32'h001A_0000;   // in a 2 MB frame slot
    wire [31:0] slot_base_address = dry_frame
        ? DENSE_BASE_ADDR + {{(32-DRY_SLOT_BITS-DRY_SLOT_LOG2){1'b0}}, frame_counter[DRY_SLOT_BITS-1:0],
                             {DRY_SLOT_LOG2{1'b0}}}
        : RING_BASE_ADDR + {4'd0, slot_idx, 21'd0};

    // Encodings unchanged; 2 (ST_PAUSE, the mandatory Lane 4 clock before wr_en)
    // and 5 (ST_DONE, unreachable) are free since 09.10.2026. ST_BURST_END is the
    // former ST_WAIT: there is no wr_done to wait for any more, it lasts exactly
    // one clock.
    localparam ST_IDLE       = 3'd0;
    localparam ST_GO         = 3'd1;
    localparam ST_WRITE      = 3'd3;
    localparam ST_BURST_END  = 3'd4;
    localparam ST_BURST_WAIT = 3'd6;
    localparam ST_ROW_SYNC   = 3'd7;

    assign o_frame_done_irq = 1'b0;

    // Continuous 50 MHz hardware timestamp
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            hw_timestamp <= 32'd0;
        else
            hw_timestamp <= hw_timestamp + 1'b1;
    end

    // "A whole burst is buffered", computed one cycle ahead.
    //
    // The FSM tests this in four places; feeding the raw fifo_rd_used comparator
    // into all of them puts the FIFO's gray-code pointer subtraction directly in
    // front of the state logic, which pushed ae350_ahb_clk's critical path from 5
    // logic levels to 12 on a design that has no margin there. Registering it keeps
    // that arithmetic out of the FSM's combinational cone.
    //
    // The +2 pays for the register's own staleness: the value is one cycle old, and
    // ST_ROW_SYNC pops a word in the very cycle it decides to start the row, so a
    // bare comparison against BURST_FIFO_LOW could let a burst begin with 15 words
    // buffered and pop 16.
    reg fifo_has_burst;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fifo_has_burst <= 1'b0;
        else        fifo_has_burst <= ({{(24-FIFO_CNT_W){1'b0}}, fifo_rd_used} >= (BURST_FIFO_LOW + 24'd2));
    end

    // Tail burst of a row: it needs only the rest of the row, not a whole burst.
    //
    // A row takes ROW_WORDS - 1 words from the FIFO in ST_WRITE (ST_ROW_SYNC has
    // already popped the first word), and the last burst gets TAIL_POPS of them:
    // at 1016 x 64 that is 61 words, while fifo_has_burst requires 66. So this
    // burst used to wait for words of the NEXT row, and the last row of a frame
    // for the first row of the next frame; if they did not arrive within
    // FRAME_ABANDON_CYCLES, the writer abandoned the frame. That is how the last
    // frame of every series was lost, and all frames with a trigger slower than
    // 3 Hz (ERRORS 2.5, cause 1; fix in ERRORS former 31.37).
    //
    // Threshold without the +2 margin: this flag is looked at only in
    // ST_BURST_WAIT, which the FSM enters after ST_BURST_END (before 09.10.2026,
    // after ST_WAIT and the inter-burst pause): the flag is latched in
    // ST_BURST_END, where row_word is already final and all pops of the row are
    // counted in fifo_rd_used (the FIFO read pointer is a register, rd_used is
    // combinational from it). The writer is the only reader of the FIFO, so a
    // stale value can only understate the word count. TAIL_POPS words in the FIFO
    // at row_word = TAIL_START is exactly the rest of THIS row: words of the next
    // row can only add to it.
    //
    // The comparisons are kept in a register, like fifo_has_burst: one bit enters
    // the FSM's combinational cone (ERRORS 6.1, ae350_ahb_clk margin).
    // 29.09.2026: the tail is the last burst of a row out of ROW_WRITE_WORDS, not
    // the last burst of the stride (ROW_TAIL_START = 256 at 1080: 128 + 128 + 16).
    // TAIL_POPS requires ROW_TAIL_START < ROW_WORDS, i.e. the tail holds pixels.
    localparam [15:0] TAIL_START = ROW_TAIL_START;
    localparam [23:0] TAIL_POPS  = {8'd0, ROW_WORDS - 16'd1 - TAIL_START};
    reg fifo_has_tail;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fifo_has_tail <= 1'b0;
        else        fifo_has_tail <= (row_word >= TAIL_START) &&
                                     ({{(24-FIFO_CNT_W){1'b0}}, fifo_rd_used} >= TAIL_POPS);
    end

    // Dry row (frame without rows, see dry_frame), registered like the flags above.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) dry_row_q <= 1'b0;
        else        dry_row_q <= dry_frame && burst_nz_q;
    end

    // The sensor line index at the head of the FIFO, and whether it continues the
    // previous row's. Kept out of the FSM's always block on purpose: this is an
    // EQUALITY test, not a magnitude one, and it feeds a counter only - never the
    // next-state logic - so it stays off the critical path the way the multiply in
    // ST_ROW_SYNC's address calculation already does.
    wire [LINE_IDX_BITS-1:0] fifo_line_idx =
        fifo_rd_data[FIFO_IDX_LSB+LINE_IDX_BITS-1:FIFO_IDX_LSB];
    wire line_idx_jump =
        (fifo_line_idx != (row_line_idx + {{(LINE_IDX_BITS-1){1'b0}}, 1'b1}));

    // Row offset from the slot base.
    //
    // lin is the "logical" row offset in the frame, r * ROW_STRIDE_BYTES.
    // 🪦 It used to have bit 19 carried into bit 20, moving the two frame halves a
    // megabyte apart to dodge a "faulty A19". That was the floating U5 pin, and
    // the carry is gone (ERRORS 1.4; see the slot layout at PIX_BASE_OFF above).
    //
    // ⚠ With the full frame (17.09.2026) the stride is 1280 B, and the multiply is
    // a sum of two shifts (r<<10 + r<<8), a 22-bit adder before cur_addr; it is
    // pipelined through row_addr_q below. docs/WORKFLOWS.md § 3 forbids magnitude
    // comparators here: ae350_ahb_clk has no margin.
    //
    // The upper bits of lin are dropped on purpose. The row index comes from the
    // bridge as 12 bits, and with a healthy stream it is below FRAME_HEIGHT; if it
    // turns out larger, the row lands inside its own slot rather than in the
    // neighbouring one, as it used to.
    wire [31:0] row_lin_off =
        {{(32-LINE_IDX_BITS){1'b0}}, fifo_line_idx} * ROW_STRIDE_BYTES;
    // Memory is linear (ERRORS 1.4): the row offset is simply the pixel base plus
    // the logical offset. The bit 19 carry is gone.
    wire [31:0] row_slot_off = PIX_BASE_OFF + row_lin_off;

    // Row address pipelined (17.09.2026, ERRORS former 31.45). With a 1280 stride
    // the multiply is already an adder, and the path "BSRAM FIFO output ->
    // r<<10 + r<<8 -> + slot base -> cur_addr" did not fit in 7.5 ns: -1.11 ns,
    // 7 of the 13 lane4_clk violations in the full-frame build. The row index is
    // latched from the FIFO output, the address is computed from the latched
    // value, and ST_ROW_SYNC takes the ready row_addr_q only if nothing was read
    // in the last two clocks (fifo_rd_hist == 0): then both registers already
    // reflect the word at the FIFO head. One extra clock per row against ~500
    // clocks of writing it. slot_idx changes when the slot closes (ST_BURST_END),
    // and before ST_ROW_SYNC the FSM passes through ST_IDLE, so the base in
    // row_addr_q is fresh by then.
    reg [LINE_IDX_BITS-1:0] row_line_idx_q;
    reg [31:0]              row_addr_q;
    reg [1:0]               fifo_rd_hist;
    // The word at the FIFO head starts a sensor frame (bits 32 and 33), registered
    // like row_line_idx_q: under row_addr_ready it describes the same word as
    // fifo_rd_data, and the BSRAM output stays out of ST_ROW_SYNC's restart branch.
    reg                     head_frame_start_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_line_idx_q     <= {LINE_IDX_BITS{1'b0}};
            row_addr_q         <= 32'd0;
            fifo_rd_hist       <= 2'b11;
            head_frame_start_q <= 1'b0;
        end else begin
            row_line_idx_q     <= fifo_line_idx;
            row_addr_q         <= slot_base_address + PIX_BASE_OFF +
                                  {{(32-LINE_IDX_BITS){1'b0}}, row_line_idx_q} * ROW_STRIDE_BYTES;
            fifo_rd_hist       <= {fifo_rd_hist[0], fifo_rd_en};
            head_frame_start_q <= fifo_rd_data[32] && fifo_rd_data[33];
        end
    end
    wire row_addr_ready = (fifo_rd_hist == 2'b00);

    // "The row index does not continue the previous one", registered (25.09.2026).
    // The direct line_idx_jump compare sat between the BSRAM FIFO output and the
    // CE of frame_jump_cnt: the worst lane4_clk path of build 17FDDAC6, slack
    // 0.014 ns. The row open branch in ST_ROW_SYNC is taken only on
    // row_addr_ready - two clocks without a FIFO read - and then row_line_idx_q
    // already holds the index from the FIFO head (latched a clock earlier), and
    // row_line_idx has not changed since the previous row opened. So in the
    // branch's clock line_idx_jump_q equals line_idx_jump: the counter is the
    // same, and the BSRAM output has left the CE cone.
    // "Row full" (row_word >= ROW_TOTAL_WORDS) and "last row of the frame"
    // (row_idx + 1 >= FRAME_HEIGHT) are registered (25.09.2026, build E2). Both
    // magnitude compares sat in the CE cone of the diag_frame_* snapshots at
    // frame close in ST_WAIT: row_word[13] -> diag_frame_done/CE, -0.187 ns on
    // lane4_clk, exactly what docs/WORKFLOWS.md § 3 warns against. The flags are
    // updated in THE SAME assignments as row_word and row_idx, and from THE SAME
    // next values, so in every clock row_full_q equals
    // (row_word >= ROW_TOTAL_WORDS) and row_last_q equals
    // (row_idx + 1 >= FRAME_HEIGHT). There are no other assignments of
    // row_word/row_idx in the module; a new assignment must update the flag too.
    // row_tail_q (29.09.2026) does the same for (row_word >= ROW_TAIL_START): the
    // next burst of the row is the tail (ROW_TAIL_LEN words). Maintained in the
    // same assignments as row_word and row_full_q.
    reg row_full_q, row_last_q, row_tail_q;
    // row_idx != 0, kept in the same assignments as row_idx (19.10.2026): a row
    // other than row 0 is being opened, so a frame start at the FIFO head means
    // the sensor frame ended before the slot did (see the restart in ST_ROW_SYNC).
    reg row_past0_q;
    reg [15:0] wr_frame_lines;
    reg line_idx_jump_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) line_idx_jump_q <= 1'b0;
        else        line_idx_jump_q <= (row_line_idx_q != (row_line_idx + {{(LINE_IDX_BITS-1){1'b0}}, 1'b1}));
    end

    // 🪦 wait_elapsed (wait_timeout[8], the wr_done wait timeout in ST_WAIT) went
    // with Lane 4 on 09.10.2026. The lesson still holds: a threshold is a counter
    // bit, not a comparator (>= 250 cost ae350_ahb_clk 97.4 MHz against 100).

    // Row / frame resynchronisation.
    //
    // async_fifo_simple has one cycle of read latency, so a word is inspected the
    // cycle AFTER it is popped and then stays in fifo_rd_data until the next pop.
    // The whole streaming path is built around that: the word written in a given
    // ST_WRITE cycle is the one popped in the previous cycle. ST_ROW_SYNC therefore
    // pops until the tagged first-word-of-a-line is sitting in fifo_rd_data, and it
    // is then consumed as the first word of the row - no extra buffering needed.
    wire sync_ok = fifo_rd_data[32] && (!need_frame_start || fifo_rd_data[33]);

    // FIFO read enable.
    //   ST_ROW_SYNC : discard words until a line boundary is at the head.
    //   ST_WRITE    : prefetch the NEXT payload word of the row. The last payload
    //                 word must not prefetch, or the following line's tagged first
    //                 word would be swallowed and every row would need a resync.
    // Burst 0 of a slot is pure header/telemetry, so it never pops.
    //
    // "Something to discard" in ST_ROW_SYNC is the register fifo_has_word, not
    // fifo_rd_used != 0 right here (24.09.2026, build L2a): otherwise the Gray
    // decoder of the FIFO pointers and the subtraction sit in the rd_en cone, and
    // from there in the read pointer, the BSRAM address and sync_pops: 24 levels,
    // -1.2 ns on lane4_clk for camera 1. The flag is a clock old, so the threshold
    // is two words: the writer pops at most one per clock, and it will not touch
    // an empty FIFO. A single word at the head in ST_ROW_SYNC simply waits for
    // the next one: a one-word row does not exist.
    // Declared here, before its first use (Icarus rejects a use before the
    // declaration); it is driven further down, after hdr_ptr.
    reg fifo_has_word;
    assign fifo_rd_en =
        (state == ST_ROW_SYNC) ? (!sync_ok && fifo_has_word) :
        (state == ST_WRITE)    ? (!diag_burst && burst_nz_q && row_nl_q) :
                                 1'b0;

    // Words 16 and up. They used not to exist as separate words: the mux looked
    // only at the low four bits of the counter, so a 64-word burst held four
    // identical copies of words 0..15. Nobody needs the copies, but the room is
    // needed, so the second block is given to the trigger generator status.
    // Words 0..15 are untouched, so all existing tools keep reading exactly the
    // same thing.
    localparam        HAS_EXT_WORDS = (BURST_LOG2 > 4);
    localparam [31:0] HDR_EXT_BASE  = 32'd16;

    // The header word is prepared a clock early and goes to wd_data (before
    // 09.10.2026, wr_data) from register hdr_q (24.09.2026, build L2a).
    // The 38-word mux sat on the path
    // word_in_burst -> wr_data, and with the two finder words this path gave
    // -1.43 ns on lane4_clk (111.95 MHz against 133.33; ERRORS former 31.53 is the
    // same path). Now the mux is indexed by the leading pointer hdr_ptr: it is 0
    // in ST_GO and grows every ST_WRITE clock, so by the clock that writes word w
    // hdr_q already holds word w. Values are sampled one clock earlier than they
    // used to be: the timestamp in word 2 is one clock smaller, nothing else
    // changes within a burst clock.
    reg [BURST_LOG2-1:0] hdr_ptr;
    reg [31:0]           hdr_q;
    //
    // There are two stages (the second L2a build: one stage was not enough for
    // the 38-word mux, -0.70 ns on the hdr_ptr -> hdr_q path): the low words and
    // the extension are latched separately (hdr_lo_q / hdr_ext_q), and hdr_q
    // selects from the ready ones. So the pointer leads the write by two clocks:
    // in the first ST_GO clock it is 0, in the second 1, in the ST_WRITE clock of
    // word w it is w + 2. ST_GO lasts exactly two clocks (go_cnt) and is entered
    // only from a wait state, where the pointer is 0. (Before 09.10.2026 the lead
    // came from the second ST_GO clock and ST_PAUSE, the mandatory empty Lane 4
    // clock before wr_en.)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                          hdr_ptr <= {BURST_LOG2{1'b0}};
        else if ((state == ST_GO) || (state == ST_WRITE))    hdr_ptr <= hdr_ptr + 1'b1;
        else                                                 hdr_ptr <= {BURST_LOG2{1'b0}};
    end

    // "At least two words in the FIFO", registered; see fifo_rd_en above.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fifo_has_word <= 1'b0;
        else        fifo_has_word <= ({{(24-FIFO_CNT_W){1'b0}}, fifo_rd_used} >= 24'd2);
    end


    // Lane 5 diagnostics for mailbox polling (header words 31..33, ERRORS former
    // 31.44). Driven by lane5_reader.v; declared here, before the mux.
    wire [31:0] l5_counts;             // {go_to, rd_to, rdy_cnt, last_words}
    wire [31:0] l5_word0, l5_word1;

    reg [31:0] header_ext_lo;
    always @(*) begin
        case (hdr_ptr)
            6'd16: header_ext_lo = cfg_seq_seen;  // accepted command
            6'd17: header_ext_lo = {cfg_magic_ok, trig_run, trig_cont,
                                    trig_active, 4'd0,
                                    trig_left, cfg_count, cfg_flags[7:0]};
            6'd18: header_ext_lo = cfg_period;    // as the FPGA sees them
            6'd19: header_ext_lo = cfg_width;
            6'd20: header_ext_lo = trig_pulses;   // pulses issued
            6'd21: header_ext_lo = strobe_ts;     // raw, not per frame
            6'd22: header_ext_lo = {strobe_width, strobe_count};
            // Sync block. The CPU reads these words from the telemetry at 0x06000000
            // (updated every 0.5 s): the block has no registers on the bus.
            6'd23: header_ext_lo = 32'h5359_4E43;              // "SYNC"
            6'd24: header_ext_lo = {23'd0, sync_loaded, sync_status};
            6'd25: header_ext_lo = sync_accepted;
            6'd26: header_ext_lo = sync_overrun;
            default: header_ext_lo = 32'h0;
        endcase
    end

    reg [31:0] header_ext_hi;
    always @(*) begin
        case (hdr_ptr)
            6'd27: header_ext_hi = sync_ext_edges;
            6'd28: header_ext_hi = sync_last_ts;
            6'd29: header_ext_hi = sync_mbx[12*32 +: 32];     // APPLY_SEQ as the FPGA sees it
            6'd30: header_ext_hi = sync_mbx[13*32 +: 32];     // SW_SEQ
            // Lane 5 health during mailbox polling (ERRORS former 31.44): on DDR3-1066 the
            // mailbox stopped being readable, and without these words "no rd_rdy",
            // "no rd_valid" and "shifted data" cannot be told apart.
            6'd31: header_ext_hi = l5_counts;
            6'd32: header_ext_hi = l5_word0;
            6'd33: header_ext_hi = l5_word1;
            // Row header candidates, counted BEFORE the ECC check
            6'd34: header_ext_hi = {diag_seen_hits, diag_seen_wc[31:16]};
            // Receive geometry as the FPGA SEES it, and the apply counter.
            // [23:12] is the height the writer frames slots with (wr_frame_lines),
            // [31] "a newer height is waiting for the next slot" (19.10.2026,
            // ERRORS 2.11); until then [23:12] was cfg_frame_lines and [31] waited
            // for a D-PHY watchdog reset. Firmware and host read the same fields.
            6'd35: header_ext_hi = {geom_pending, geom_seq_seen[6:0],
                                    wr_frame_lines[11:0], cfg_line_bytes[11:0]};
            // Laser line finder (L2a). Word 36 is the parameter echo
            // [13] = frame without rows (lasr_cfg[52], stage 2): echo for the firmware;
            // [12] = 1: dry frames go to the dense ring (19.10.2026).
            6'd36: header_ext_hi = {8'h4C, lasr_cfg[51:44], lasr_cfg[43],
                                    lasr_cfg[42], cfg_no_frame, 1'b1, lasr_cfg[19:16],
                                    lasr_cfg[7:0]};
            6'd37: header_ext_hi = {prof_blocks, prof_no_line};
            // "pixel -> mm" table (R-38): {'M', APPLY_SEQ of the last job, band valid,
            // CRC mismatch, load error, load in progress, 0, node rows}; 39 - the CRC
            // of the last load.
            6'd38: header_ext_hi = mm_tel_q[63:32];
            6'd39: header_ext_hi = mm_tel_q[31:0];
            // Never reaches memory: HEADER_WORDS is 40, so word 40 is filler. Kept so the
            // source reproduces the builds placed with it (D405191E, 79BAFAB7); the netlist
            // without it, placed before the .sdc max delays, dropped camera 1 frames
            // (478B2272, ERRORS 1.26, 6.17).
            6'd40: header_ext_hi = {8'h44, 3'd0, DRY_SLOT_BITS[4:0], DENSE_BASE_ADDR[31:16]};
            default: header_ext_hi = 32'h0;
        endcase
    end

    // Burst 0 Hardware Header (Bytes 0..23 = Words 0..5)
    reg [31:0] header_lo;
    always @(*) begin
        case (hdr_ptr[3:0])
            4'd0: header_lo = 32'h56494430;                  // "VID0" Magic
            4'd1: header_lo = frame_counter;                 // 32-bit Frame Sequence
            4'd2: header_lo = hw_timestamp;                  // 32-bit 50 MHz Timestamp
            // 10'd0, not 8'd0: this used to build a 30-bit value that Verilog
            // zero-extended to 32. It happened to land slot_idx on bits 21:16,
            // which is where the host reads it, so it worked by luck.
            // 9'd0 since slot_idx grew to 7 bits: the field still occupies bits
            // 22:16 and the host still reads the slot from the upper half-word.
            4'd3: header_lo = {9'd0, slot_idx, FRAME_WIDTH};  // Metadata (Slot, Width)
            4'd4: header_lo = {wr_frame_lines, 16'd2};        // Height + Source (2: Camera MIPI)
            4'd5: header_lo = 32'h12345678;                  // Integrity constant
            // Words 6..15: capture telemetry, readable over JTAG without the network.
            // Burst 0 is header-only, so none of this overlaps pixel data: row 0
            // starts at slot offset +64.
            // Words 6 and 7 used to carry BURSTS_PER_FRAME and diag_bursts_done.
            // Neither measured anything: the first is an elaborated constant the
            // host can recompute from the width and height in words 3 and 4, and
            // the second is burst_idx + 1 sampled at the end of the frame, which
            // on a frame that completes can only ever equal the first. "Bursts
            // written 4097/4097" was that tautology, and it is what made the
            // write path look healthy while 5 % of rows went unwritten.
            //
            // What replaces them is measured. Word 6 is the row path: how many
            // rows the writer opened, how many of them arrived with a line index
            // that did not continue the previous, and how many frames it gave up
            // on. Word 7 is the write path: until 09.10.2026 the frame's wr_go
            // strobes against the wr_done that answered them; since the AXI port,
            // AXI write bursts issued against write responses received, both
            // cumulative and wrapping (cam_axi_wr) - equal whenever the writer is
            // idle, the difference is the bursts in flight.
            4'd6:  header_lo = {diag_frame_jumps,            // [31:24] index discontinuities
                                  diag_abandons,               // [23:16] frames abandoned (cumulative)
                                  5'd0,
                                  diag_frame_rows};            // [10:0]  rows opened (expect H + 1)
            4'd7:  header_lo = {axi_b_cnt,                   // [31:16] AXI write responses received
                                  axi_aw_cnt};                 // [15:0]  AXI write bursts issued
            4'd8:  header_lo = diag_header_count;            // RAW8 line headers accepted
            4'd9:  header_lo = diag_bad_wc_count;            // line headers rejected on word count
            4'd10: header_lo = diag_line_count;              // complete lines emitted
            // Bits 15:13: non-OKAY write responses, cumulative, saturating at 7
            // (cam_axi_wr). Until 09.10.2026 they held diag_wait_to, the frame's
            // ST_WAIT timeouts on Lane 4.
            4'd11: header_lo = {diag_last_wc, axi_bresp_err, diag_resync_cnt[9:0], diag_overflow, diag_phase};
            // Ground truth about the wire, independent of LINE_BYTES being right:
            // Upper half is the measured byte-clock frequency in 100 kHz units -
            // the only way to see the real MIPI link rate, since the sensor sets it.
            4'd12: header_lo = {diag_byte_clk_ticks, diag_seen_wc[15:0]};
            // Lines the FIFO refused because a whole line no longer fitted. Non-zero
            // means rows were skipped - the image loses rows but stays aligned.
            // Low half used to be diag_seen_hits (header candidates at the
            // configured word count), then the read-back verifier's per-frame
            // bad and skipped rows; zero since the verifier was removed
            // (30.10.2026, ERRORS 1.7).
            //
            // 22.09.2026: diag_seen_hits turned out to be needed again: without it
            // "ECC rejects good headers" cannot be told from "D-PHY is not synced"
            // when 0 are accepted and millions rejected. It came back as a separate
            // word 34, not here: host tools read words 0..15 as a 64-byte dump, and
            // changing their layout would break them silently.
            4'd13: header_lo = {diag_lines_drop[15:0], 16'd0};
            // Raw per-lane D-PHY activity: the ground truth for "is lane 1 alive".
            // Words 14 and 15 used to carry the D-PHY lane activity counters
            // (diag_lane_vld / diag_lane_nz). They had long hit the 65535 ceiling
            // and said exactly one thing: "data flows on both lanes". Useful while
            // bringing MIPI up, pointless now that the link has held for ages.
            // Both signals are still in the module, so bringing them back is a
            // one-line change.
            //
            // Instead: tying the frame to the moment of its exposure.
            //   14: hw_timestamp at the strobe edge, the same counter as in word 2
            //   15: {strobe width, strobe count}
            // The difference of words 2 and 14 gives the delay between capture and
            // frame write, and word 15 the measured exposure and a sign of missed
            // strobes.
            4'd14: header_lo = strobe_ts_pub;
            4'd15: header_lo = {strobe_width_pub, strobe_count_pub};
            default: header_lo = fifo_rd_data[31:0];
        endcase
    end

    // Words past HEADER_WORDS are filler.
    // First stage: three small muxes (0..15, 16..26, 27..39).
    // Second stage: select among the registered stages by hdr_sel_q.
    reg [31:0] hdr_lo_q, hdr_ext_lo_q, hdr_ext_hi_q;
    reg [1:0]  hdr_sel_q;
    reg        hdr_pad_q;
    always @(posedge clk) begin
        hdr_lo_q      <= header_lo;
        hdr_ext_lo_q  <= header_ext_lo;
        hdr_ext_hi_q  <= header_ext_hi;
        hdr_sel_q     <= (!HAS_EXT_WORDS || (hdr_ptr < 6'd16)) ? 2'd0 :
                         (hdr_ptr <= 6'd26) ? 2'd1 : 2'd2;
        hdr_pad_q     <= ({1'b0, hdr_ptr} >= HEADER_WORDS);
        hdr_q         <= hdr_pad_q          ? ROW_PAD_WORD :
                         (hdr_sel_q == 2'd1) ? hdr_ext_lo_q :
                         (hdr_sel_q == 2'd2) ? hdr_ext_hi_q : hdr_lo_q;
    end

    // Burst 0 of a slot, and every telemetry heartbeat burst, is header words only.
    // Inside a row, words past the payload pad it out to a whole 64-byte burst; the
    // pattern is distinctive so the host can verify row alignment directly.
    // The header only fills the first HEADER_WORDS words of burst 0. With a 64-word
    // burst the remaining 48 words must be padded explicitly - header_word decodes
    // word_in_burst[3:0], so without this the whole 16-word header would simply
    // repeat four times and overwrite itself with a stale copy.
    wire header_area = (diag_burst || !burst_nz_q);

    // Diagnostic pattern. Each word says three things, so a frame read back tells
    // not only whether the content survived but whether the ROW LANDED WHERE IT
    // SHOULD - which a ramp test pattern cannot, because on a ramp every row is
    // identical and any permutation of rows looks perfect.
    //
    //   [31:24] row_idx    - how many rows had been written when this one went out
    //   [23:12] line index - the sensor line this row claims to be, and the value
    //                        the destination address was computed from
    //   [11:0]  word_in_row
    //
    // A row sitting at DDR3 position P whose stored line index is not P means the
    // address was computed from a wrong index; a stored row_idx out of step with
    // its neighbours means rows arrived out of order.
    wire [31:0] test_pattern_word =
        {row_idx[7:0],
         fifo_rd_data[FIFO_IDX_LSB+LINE_IDX_BITS-1:FIFO_IDX_LSB],
         row_word[11:0]};

    wire [31:0] row_tag_word   = {ROW_TAG_MAGIC,
                                  {(16-LINE_IDX_BITS){1'b0}}, row_line_idx};

    // The profile block burst goes first: its word is already in register
    // prof_q, and it adds one mux input to this path, not logic.
    wire [31:0] current_data_word =
        prof_burst                                        ? prof_q           :
        header_area                                       ? hdr_q            :
        // Past the payload: the two trailer words, told apart by bit 0 of
        // row_word rather than by comparing it against two constants. See the
        // HAS_ROW_TAIL note - equality comparators here cost this design a build
        // that would not boot.
        (row_word >= ROW_WORDS) ? (HAS_ROW_TAIL
                                     ? (row_word[0] ? frame_counter : row_tag_word)
                                     : ROW_PAD_WORD) :
        (WRITE_TEST_PATTERN != 0)                         ? test_pattern_word :
                                                            fifo_rd_data[31:0];

    reg [25:0] burst_timeout_cnt;

    // Main streaming FSM (Lane 4 until 09.10.2026, cam_axi_wr since)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            frame_counter       <= 32'd1;
            slot_idx            <= 7'd0;
            burst_idx           <= 16'd0;
            burst_nz_q          <= 1'b0;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
            word_in_burst       <= {ROW_BURST_LOG2{1'b0}};
            burst_last          <= BURST_WORDS[ROW_BURST_LOG2-1:0] - 1'b1;
            go_cnt              <= 2'd0;
            wd_en               <= 1'b0;
            wd_first            <= 1'b0;
            wd_last             <= 1'b0;
            wd_data             <= 32'd0;
            burst_addr          <= 32'd0;
            cur_addr            <= RING_BASE_ADDR;

            burst_timeout_cnt   <= 26'd0;
            o_current_slot      <= 7'd0;
            o_frame_counter     <= 32'd0;
            diag_hb_cnt         <= 26'd0;
            diag_burst          <= 1'b0;
            row_idx             <= 16'd0;
            row_past0_q         <= 1'b0;
            row_word            <= 16'd0;
            row_nl_q            <= (16'd0 < ROW_WORDS - 16'd1);   // = (row_word < ROW_WORDS - 1) next cycle
            row_full_q          <= 1'b0;
            row_tail_q          <= (16'd0 >= ROW_TAIL_START);
            wr_frame_lines      <= FRAME_HEIGHT;
            row_last_q          <= (16'd1 >= FRAME_HEIGHT);
            row_line_idx        <= {LINE_IDX_BITS{1'b0}};
            row_start_pending   <= 1'b0;
            need_frame_start    <= 1'b1;
            sync_pops           <= 16'd0;
            diag_resync_cnt     <= 16'd0;
            frame_rows_cnt      <= 11'd0;  diag_frame_rows   <= 11'd0;
            frame_jump_cnt      <= 8'd0;   diag_frame_jumps  <= 8'd0;
            diag_abandons       <= 8'd0;
            frame_end_q         <= 1'b0;
            prof_burst          <= 1'b0;
            lz_frame_start      <= 1'b0;
            lz_close            <= 1'b0;
            lz_frame_id         <= 32'd0;
            lz_block_base       <= 32'd0;
            lz_rows             <= 11'd0;
            dry_frame           <= 1'b0;
        end else if (system_ready) begin
            if ({6'd0, diag_hb_cnt} < DIAG_PERIOD) diag_hb_cnt <= diag_hb_cnt + 26'd1;

            // Both are pulses: default low, raised for exactly one cycle by the
            // row-complete branch below.
            frame_end_q      <= 1'b0;
            lz_frame_start   <= 1'b0;
            lz_close         <= 1'b0;
            // Words to cam_axi_wr: only ST_WRITE raises them.
            wd_en            <= 1'b0;
            wd_first         <= 1'b0;
            wd_last          <= 1'b0;

            case (state)
                ST_IDLE: begin
                    // A burst starts only when cam_axi_wr has room for a whole one
                    // (axi_room, a register): it is posted there in full and
                    // nothing waits for it afterwards.
                    //
                    // The telemetry heartbeat is checked FIRST, on purpose.
                    //
                    // If frame start had priority, a stream that never locks would
                    // still keep the FIFO topped up with garbage, the FSM would start
                    // a frame on every pass through ST_IDLE and the heartbeat branch
                    // would never run - leaving the pipeline completely unobservable
                    // in exactly the situation where the telemetry is needed. One
                    // 64-byte burst twice a second costs nothing.
                    if (axi_room && ({6'd0, diag_hb_cnt} >= DIAG_PERIOD)) begin
                        diag_hb_cnt <= 26'd0;
                        diag_burst  <= 1'b1;
                        state       <= ST_GO;
                        go_cnt      <= 2'd0;
                    end else if (axi_room && prof_ready) begin
                        // The next profile block burst is ready. Before the slot
                        // opens: until the block is finished, prof_busy holds back
                        // the next frame (the finder's memory read port is shared).
                        prof_burst  <= 1'b1;
                        state       <= ST_GO;
                        go_cnt      <= 2'd0;
                    end else if (axi_room && cam_enable && fifo_has_burst && !prof_busy) begin
                        // Room for a burst and the camera has pixels waiting.
                        // A slot is only opened once the head of the FIFO is the
                        // first line of a frame, so row r of the slot really is row r
                        // of the sensor frame.
                        burst_idx         <= 16'd0;
                        burst_nz_q        <= 1'b0;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                        row_idx           <= 16'd0;
                        row_past0_q       <= 1'b0;
                        row_word          <= 16'd0;
                        row_nl_q          <= (16'd0 < ROW_WORDS - 16'd1);   // = (row_word < ROW_WORDS - 1) next cycle
                        row_full_q        <= 1'b0;
                        row_tail_q        <= (16'd0 >= ROW_TAIL_START);
                        // The frame boundary where a new height takes effect
                        // (see "Applying the receive geometry").
                        wr_frame_lines    <= cfg_frame_lines;
                        row_last_q        <= (16'd1 >= cfg_frame_lines);
                        diag_burst        <= 1'b0;
                        need_frame_start  <= 1'b1;
                        row_start_pending <= 1'b1;
                        sync_pops         <= 16'd0;
                        burst_timeout_cnt <= 26'd0;
                        // A slot opens here, so the per-frame measurements start
                        // here too - including after an abandoned attempt, whose
                        // partial counts must not be added to the retry's.
                        frame_rows_cnt    <= 11'd0;
                        frame_jump_cnt    <= 8'd0;
                        // The finder starts the frame afresh, after an abandoned
                        // attempt too (epoch bit, laser_col_extract.v).
                        lz_frame_start    <= 1'b1;
                        // Whether to write this frame's rows is decided here, once
                        // per frame (see dry_frame).
                        dry_frame         <= cfg_no_frame;
                        state             <= ST_ROW_SYNC;
                    end
                end

                // Discard FIFO words until a row (and, at the start of a slot, a
                // frame) boundary is at the head. Normally this costs exactly one
                // pop: the previous row consumed its payload and left the next
                // line's tagged first word as the very next entry.
                ST_ROW_SYNC: begin
                    burst_timeout_cnt <= burst_timeout_cnt + 1'b1;

                    // Both conditions have to be re-tested here, not inherited from
                    // ST_BURST_WAIT, because this state sits BETWEEN that check and
                    // the start of the burst and can linger for many cycles:
                    //
                    //   * axi_room: the burst is handed to cam_axi_wr in full, with
                    //     no way to stall - its queues must have room for all of it.
                    //     (On Lane 4 this was wr_wait, and a wr_go issued against it
                    //     landed bursts at wrong addresses, over the firmware.)
                    //   * fifo_has_burst: the resync pops eat the margin that was
                    //     verified in ST_BURST_WAIT, and a burst pops its words back
                    //     to back with no way to stall.
                    // A dry row needs no room: its words do not go to cam_axi_wr,
                    // and waiting for room would overflow the receive FIFO.
                    // The slot header (burst_idx = 0) is never dry.
                    //
                    // Restart on a frame start (19.10.2026, ERRORS 2.11): row 1 or
                    // later is due, but the head of the FIFO is line 0 of the NEXT
                    // sensor frame - the frame ended with fewer rows than the slot
                    // was opened for (a lower ROI height not yet in the writer, or
                    // the frame's last lines refused by the FIFO). Until this day
                    // the writer went on and filled the slot with the next frames'
                    // rows under one header: 1280/H frames merged into one "whole"
                    // slot (ERRORS 2.11, OPEN_ISSUES 1.9 "mixed frame"). Now the
                    // attempt is dropped like an abandoned one - slot_idx stays,
                    // the frame-start word stays at the head (sync_ok holds it) and
                    // ST_IDLE reopens the slot with it and with the current height.
                    // A frame is lost rather than passed off as whole. Taken only
                    // under row_addr_ready, so head_frame_start_q is the head word.
                    if (row_past0_q && head_frame_start_q && row_addr_ready) begin
                        burst_timeout_cnt <= 26'd0;
                        burst_idx         <= 16'd0;
                        burst_nz_q        <= 1'b0;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                        row_start_pending <= 1'b0;
                        need_frame_start  <= 1'b1;
                        state             <= ST_IDLE;
                        diag_abandons     <= diag_abandons + 8'd1;
                        diag_frame_rows   <= frame_rows_cnt;
                        diag_frame_jumps  <= frame_jump_cnt;
                    end else if (sync_ok && fifo_has_burst && (axi_room || dry_row_q) && row_addr_ready) begin
                        row_word          <= 16'd0;
                        row_nl_q          <= (16'd0 < ROW_WORDS - 16'd1);   // = (row_word < ROW_WORDS - 1) next cycle
                        row_full_q        <= 1'b0;
                        row_tail_q        <= (16'd0 >= ROW_TAIL_START);
                        row_start_pending <= 1'b0;
                        need_frame_start  <= 1'b0;
                        // The word at the head is this row's first, and it carries
                        // the sensor line index this row belongs to. That index -
                        // not a count of rows written - decides where the row goes.
                        row_line_idx      <= fifo_line_idx;
                        // The row address is the slot base plus row_slot_off (above),
                        // a linear offset (ERRORS 1.4). The header sits in its own
                        // 256-byte burst at the slot base, and PIX_BASE_OFF already
                        // accounts for it, so BURST_BYTES plays no part here.
                        //
                        // Inside the row cur_addr steps forward burst by burst
                        // (see burst_last); nothing has to be carried along the row.
                        cur_addr          <= row_addr_q;   // = slot_base_address + row_slot_off, pipelined
                        burst_timeout_cnt <= 26'd0;
                        go_cnt            <= 2'd0;
                        state             <= ST_GO;
                        // More than the one expected pop means the stream had lost
                        // its line phase and this row realigned it.
                        if (sync_pops > 16'd1 && diag_resync_cnt != 16'hFFFF)
                            diag_resync_cnt <= diag_resync_cnt + 16'd1;

                        // Rows the writer actually opened, counted where it opens
                        // them: 1 for the sync that precedes the header burst,
                        // then 1 per row. A completed frame must show
                        // FRAME_HEIGHT + 1.
                        frame_rows_cnt <= frame_rows_cnt + 11'd1;

                        // Does this row's sensor line index continue the last
                        // one's? row_idx is still 0 while row 0 opens, and there
                        // is no previous index then - the header sync loaded
                        // row_line_idx from this same FIFO word without popping
                        // it, so comparing here would report a jump every frame.
                        if ((row_idx != 16'd0) && line_idx_jump_q)
                            frame_jump_cnt <= frame_jump_cnt + 8'd1;
                    end else if (!sync_ok && fifo_has_word) begin
                        sync_pops <= sync_pops + 16'd1;
                    end

                    // Same long escape as ST_BURST_WAIT: never park here forever, or
                    // the heartbeat telemetry stops and the pipeline goes dark.
                    if ({6'd0, burst_timeout_cnt} >= FRAME_ABANDON_CYCLES) begin
                        burst_timeout_cnt <= 26'd0;
                        burst_idx         <= 16'd0;
                        burst_nz_q        <= 1'b0;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                        row_start_pending <= 1'b0;
                        need_frame_start  <= 1'b1;
                        state             <= ST_IDLE;
                        // Abandoning here leaves slot_idx where it is, so the slot
                        // is rewritten from row 0 and every row this attempt never
                        // reached still holds the previous revolution's pixels.
                        // Publish what the abandoned attempt managed, so the host
                        // sees a short frame instead of a plausible one.
                        diag_abandons     <= diag_abandons + 8'd1;
                        diag_frame_rows   <= frame_rows_cnt;
                        diag_frame_jumps  <= frame_jump_cnt;
                    end
                end

                // Two cycles of burst setup: the address and length are latched,
                // and the header word pipeline (hdr_ptr, two stages) and the
                // profile word register (prof_first) get their lead.
                ST_GO: begin
                    // Burst 0 is the slot header and always sits at the slot base.
                    // It must NOT come from cur_addr: a slot opens through
                    // ST_ROW_SYNC, which runs before the header burst is issued
                    // and has already loaded cur_addr with row 0's address. Taking
                    // it from there put the header on top of row 0 and left the
                    // slot base - where the host looks for "VID0" - never written.
                    burst_addr <= diag_burst          ? DIAG_ADDR         :
                                  prof_burst          ? prof_addr         :
                                  !burst_nz_q ? slot_base_address :
                                                        cur_addr;
                    // Burst length by the same signs as the address: header,
                    // telemetry and profile use BURST_WORDS, a row 128 or the tail.
                    if (diag_burst || prof_burst || !burst_nz_q)
                        burst_last <= BURST_WORDS[ROW_BURST_LOG2-1:0] - 1'b1;
                    else if (row_tail_q)
                        burst_last <= ROW_TAIL_LEN[ROW_BURST_LOG2-1:0] - 1'b1;
                    else
                        burst_last <= ROW_BURST_LEN[ROW_BURST_LOG2-1:0] - 1'b1;
                    word_in_burst <= {ROW_BURST_LOG2{1'b0}};

                    // 🪦 ST_GO held wr_go for exactly two cycles, then ST_PAUSE gave
                    // Lane 4 its idle cycle before wr_en (CLAUDE.md invariant 3).
                    // Neither exists on the AXI port (09.10.2026). The two cycles
                    // stay because the header and profile word pipelines need them.
                    if (go_cnt == 2'd0) begin
                        go_cnt <= 2'd1;
                    end else begin
                        go_cnt <= 2'd0;
                        state  <= ST_WRITE;
                    end
                end

                ST_WRITE: begin
                    // Dry row: the words are popped from the FIFO and go to the
                    // finder, but not to cam_axi_wr (or memory).
                    wd_en    <= !dry_row_q;
                    wd_first <= (word_in_burst == {ROW_BURST_LOG2{1'b0}});
                    wd_last  <= (word_in_burst == burst_last);

                    // Output 32-bit data word per clock cycle
                    wd_data  <= current_data_word;

                    // Row position advances only for real row bursts, never for the
                    // header burst or a telemetry heartbeat.
                    if (!diag_burst && burst_nz_q) begin
                        row_word   <= row_word + 16'd1;
                        row_nl_q   <= (row_word < ROW_WORDS - 16'd2);   // = (row_word < ROW_WORDS - 1) next cycle
                        // A row is closed when the pixels and trailer are written
                        // (ROW_WRITE_WORDS), not the whole stride (29.09.2026).
                        row_full_q <= (row_word + 16'd1 >= ROW_WRITE_WORDS);
                        row_tail_q <= (row_word + 16'd1 >= ROW_TAIL_START);
                    end

                    if (word_in_burst == burst_last) begin
                        word_in_burst <= {ROW_BURST_LOG2{1'b0}};
                        state         <= ST_BURST_END;
                    end else begin
                        word_in_burst <= word_in_burst + 1'b1;
                    end
                end

                // One cycle after the burst's last word. The burst is posted:
                // cam_axi_wr holds all of it, so there is nothing to wait for.
                //
                // 🪦 This was ST_WAIT: wait for the bridge's wr_done, or give up
                // after wait_timeout[8] and walk on (wait_to_cnt, word 11 bits
                // 15:13). Giving up was never what misplaced bursts - the lane
                // clock ratio (CLAUDE.md invariant 2) and the floating U5 pin
                // (ERRORS 1.4) were - and with the AXI port (09.10.2026) the
                // state is one cycle.
                ST_BURST_END: begin
                    begin
                        if (diag_burst) begin
                            // Telemetry burst: never touches frame state.
                            diag_burst <= 1'b0;
                            state      <= ST_IDLE;
                        end else if (prof_burst) begin
                            // Profile block burst: it does not touch the frame either.
                            prof_burst <= 1'b0;
                            state      <= ST_IDLE;
                        end else if (!burst_nz_q) begin
                            // Header burst written; row 0 comes next.
                            burst_idx         <= 16'd1;
                            burst_nz_q        <= 1'b1;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                            row_idx           <= 16'd0;
                            row_past0_q       <= 1'b0;
                            row_last_q        <= (16'd1 >= wr_frame_lines);
                            row_start_pending <= 1'b1;
                            state             <= ST_BURST_WAIT;
                        end else if (row_full_q) begin          // = (row_word >= ROW_TOTAL_WORDS), see row_full_q
                            // Row complete: pixels and trailer (ROW_WRITE_WORDS); the padding
                            // up to the stride is not written any more (29.09.2026).
                            //
                            if (row_last_q) begin              // = (row_idx + 1 >= FRAME_HEIGHT), see row_last_q
                                burst_idx        <= 16'd0;
                                burst_nz_q       <= 1'b0;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                                o_current_slot   <= slot_idx;
                                o_frame_counter  <= frame_counter;
                                // Publish this frame's counts, start the next at
                                // 0. They reach the host in the NEXT slot's header.
                                diag_frame_rows  <= frame_rows_cnt;
                                diag_frame_jumps <= frame_jump_cnt;
                                frame_end_q      <= 1'b1;   // the strobe values are published on it
                                frame_rows_cnt   <= 11'd0;
                                frame_jump_cnt   <= 8'd0;
                                frame_counter    <= frame_counter + 1'b1;
                                // Frame closed: the profile module will unload the block
                                // into this same slot. Values are taken before they change
                                // on this edge.
                                lz_close         <= 1'b1;
                                lz_frame_id      <= frame_counter;
                                lz_block_base    <= slot_base_address + (dry_frame ? DRY_BLOCK_OFF : BLOCK_OFF);
                                lz_rows          <= frame_rows_cnt;
                                // Equality, not ">=": slot_idx starts at 0 and
                                // only ever steps by one, so it cannot overshoot,
                                // and docs/WORKFLOWS.md § 3 warns against magnitude
                                // comparators in this FSM's combinational cone -
                                // ae350_ahb_clk closes with almost no margin.
                                slot_idx         <= (slot_idx == NUM_SLOTS - 7'd1) ? 7'd0 : (slot_idx + 1'b1);
                                need_frame_start <= 1'b1;
                                state            <= ST_IDLE;
                            end else begin
                                row_idx           <= row_idx + 16'd1;
                                row_past0_q       <= 1'b1;
                                row_last_q        <= (row_idx + 16'd2 >= wr_frame_lines);
                                burst_idx         <= burst_idx + 16'd1;
                                burst_nz_q        <= (burst_idx != 16'hFFFF);      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                                row_start_pending <= 1'b1;
                                state             <= ST_BURST_WAIT;
                            end
                        end else begin
                            // Still inside the row: walk forward one burst. A row
                            // start reloads cur_addr from the line index instead.
                            burst_idx <= burst_idx + 16'd1;
                            burst_nz_q <= (burst_idx != 16'hFFFF);      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                            cur_addr  <= cur_addr + ROW_BURST_BYTES;   // 512 B: a row burst (29.09.2026)
                            state     <= ST_BURST_WAIT;
                        end
                    end
                end

                ST_BURST_WAIT: begin
                    burst_timeout_cnt  <= burst_timeout_cnt + 1'b1;
                    // No gap between bursts any more (see the tombstone where
                    // BURST_GAP_CYCLES was): the next one starts as soon as the
                    // FIFO has its words and cam_axi_wr has room.
                    // A dry row needs no room in cam_axi_wr (see ST_ROW_SYNC).
                    // In the first clock after the header dry_row_q is still 0
                    // (burst_idx has just become 1); that only demands room, and
                    // then the FSM goes through ST_ROW_SYNC, where the flag is fresh.
                    if (axi_room || dry_row_q) begin
                        // fifo_has_tail: the row's tail burst only needs the rest of
                        // the row (see the register above). For the start of a row
                        // ST_ROW_SYNC re-checks fifo_has_burst anyway.
                        if (fifo_has_burst || fifo_has_tail) begin
                            burst_timeout_cnt  <= 26'd0;
                            go_cnt             <= 2'd0;
                            sync_pops          <= 16'd0;
                            state             <= row_start_pending ? ST_ROW_SYNC : ST_GO;
                        end
                    end
                    // Abandon a frame only after a LONG dry spell.
                    //
                    // This escape was originally 20 ms, which is shorter than the
                    // ~90 ms gap between frames at 10 fps, so every inter-frame
                    // blanking period restarted the slot from burst 0 and no frame
                    // could ever finish. Removing it entirely was worse: the FSM
                    // then parked in ST_BURST_WAIT forever on a stalled stream and
                    // stopped emitting the heartbeat telemetry, leaving no way to
                    // observe anything. ~0.34 s is comfortably longer than any
                    // legitimate inter-frame gap but still recovers.
                    if ({6'd0, burst_timeout_cnt} >= FRAME_ABANDON_CYCLES) begin
                        burst_timeout_cnt  <= 26'd0;
                        burst_idx          <= 16'd0;
                        burst_nz_q         <= 1'b0;      // = (burst_idx != 0) next cycle (RTL_RULES 3.1)
                        row_start_pending  <= 1'b0;
                        need_frame_start   <= 1'b1;
                        state              <= ST_IDLE;
                        // The other abandon escape; see ST_ROW_SYNC's copy.
                        diag_abandons      <= diag_abandons + 8'd1;
                        diag_frame_rows    <= frame_rows_cnt;
                        diag_frame_jumps   <= frame_jump_cnt;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Mailbox poll on Lane 5 and the mm band loader — lane5_reader.v
    // =========================================================================

    // ======================================================================
    // Camera strobe: synchronisation and timestamp
    // ======================================================================
    //
    // The strobe comes from its own domain and is unrelated to clk, so two
    // synchroniser stages are mandatory. Without them metastability goes straight
    // into the counter, and the timestamp lies silently - the worst kind of error.
    // The third bit is only there to get the edges.
    reg [2:0] strobe_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) strobe_sync <= 3'b000;
        else        strobe_sync <= {strobe_sync[1:0], cam_strobe};
    end
    wire strobe_rise = (strobe_sync[2:1] == 2'b01);
    wire strobe_fall = (strobe_sync[2:1] == 2'b10);

    reg [23:0] strobe_run;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strobe_ts        <= 32'd0;
            strobe_width     <= 24'd0;
            strobe_count     <= 8'd0;
            strobe_run       <= 24'd0;
            strobe_ts_pub    <= 32'd0;
            strobe_width_pub <= 24'd0;
            strobe_count_pub <= 8'd0;
        end else begin
            if (strobe_rise) begin
                strobe_ts    <= hw_timestamp;
                strobe_run   <= 24'd0;
                strobe_count <= strobe_count + 8'd1;
            end else if (strobe_sync[2] && !(&strobe_run)) begin
                // Saturation, not wrap: a strobe longer than 168 ms reads as "maximum"
                // rather than as a deceptively small number.
                strobe_run <= strobe_run + 24'd1;
            end

            if (strobe_fall) strobe_width <= strobe_run;

            // Published at the frame boundary, the same scheme as diag_frame_*: the slot
            // header carries the values of the frame that has just ended, and so they
            // match its own word 2.
            if (frame_end_q) begin
                strobe_ts_pub    <= strobe_ts;
                strobe_width_pub <= strobe_width;
                strobe_count_pub <= strobe_count;
            end
        end
    end

    // ======================================================================
    // Trigger generator: trigger_gen.v; instance at the bottom, next to Lane 5.
    // ======================================================================

    // ----------------------------------------------------------------------
    // Applying the receive geometry
    // ----------------------------------------------------------------------
    //
    // The order matters and is the same as the generator's: the apply counter is
    // written LAST, so by the time it changes the size is already in memory.
    //
    // The height is applied at a FRAME boundary of the writer, not at a receiver
    // reset (19.10.2026, ERRORS 2.11). A counter change loads cfg_frame_lines in
    // this domain at once; the writer copies it into wr_frame_lines only when it
    // opens a slot (ST_IDLE), so a slot already open keeps the height it was
    // opened with. Nothing else reads cfg_frame_lines: the CSI-2 bridge counts
    // lines up to the build's FRAME_HEIGHT and starts a frame on the strobe fall
    // and on the gap between frames (cam_rx.v, RUNTIME_FRAME_LINES = 0), so no
    // value crosses into byte_clk.
    //
    // 🪦 Until 19.10.2026 a counter change only remembered the size
    // (geom_pending), and it reached cfg_frame_lines while the D-PHY watchdog held
    // the receiver in reset (wd_reset): byte_clk stopped was the domain crossing
    // into the bridge, which then read the height. A change of the height alone
    // keeps the rows valid and never wakes the watchdog, so the new height waited
    // for a lost exposure - re-arms in the firmware (ERRORS 2.13, 2.14) and on the
    // host (vmsp.profile_rig_*) that sometimes did not help in 5 tries
    // (19.10.2026: 0 profiles in 6 s at ROI 208 after "find line"). Camera 1 took
    // the height over the wire at any moment, outside that reset, into its own
    // bridge: an unconstrained crossing. A receiver reset of its own is still not
    // an option (ERRORS former 31.53, the comment at mipi_rx_rst_n).
    //
    // A height of 0 or above the build's FRAME_HEIGHT is not applied: 0 means
    // "do not change", and a taller frame does not fit the slot layout. Telemetry
    // word 35 shows what the writer uses, so a refused value is visible.
    //
    // Width from the mailbox is NOT applied since 24.09.2026: the bridge compares
    // the row length with the constant FRAME_WIDTH (RUNTIME_LINE_BYTES = 0,
    // ERRORS former 31.53: a length signal ate camera 0's timing margin).
    // cfg_line_bytes stays FRAME_WIDTH, and telemetry word 35 shows the length the
    // bridge actually compares with.
    wire [15:0] geom_h = geom_size[31:16];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            geom_seq_seen   <= 32'd0;
            geom_pending    <= 1'b0;
            cfg_line_bytes  <= FRAME_WIDTH;
            cfg_frame_lines <= FRAME_HEIGHT;
        end else begin
            // Registered, a clock late: it only feeds header word 35.
            geom_pending <= (cfg_frame_lines != wr_frame_lines);
            if (ENABLE_LANE5 != 0) begin
                if (geom_seq != geom_seq_seen) begin
                    geom_seq_seen <= geom_seq;
                    if ((geom_h != 16'd0) && (geom_h <= FRAME_HEIGHT))
                        cfg_frame_lines <= geom_h;
                end
            end else begin
                // Channel without Lane 5 (camera 1): height relayed from camera 0
                if (geom_lines_in != 16'd0) begin
                    cfg_frame_lines <= geom_lines_in;
                end
            end
        end
    end

    // ---- Lane 5 and the generator: only in the channel that reads the mailbox --
    //
    // Camera 1 (ENABLE_LANE5 = 0) has neither module, and their outputs are
    // constants equal to the post-reset values: exactly how these registers stood
    // for it when everything was one module. The netlist hierarchy is kept
    // (Netlist_Hierarchy), and synthesis would not see through the module boundary
    // that the mailbox parameters are zero for camera 1, and would
    // keep both parts.
    generate
        if (ENABLE_LANE5 != 0) begin : g_lane5
            lane5_reader #(
                .CFG_ADDR     (CFG_ADDR),
                .BURST_LOG2   (BURST_LOG2),
                .ENABLE_LANE5 (ENABLE_LANE5)
            ) u_l5 (
                .clk                 (clk),
                .rst_n               (rst_n),
                .rd_go               (rd_go),
                .rd_burstcount       (rd_burstcount),
                .rd_addr             (rd_addr),
                .rd_en               (rd_en),
                .rd_rdy              (rd_rdy),
                .rd_valid            (rd_valid),
                .rd_data             (rd_data),
                .o_l5_counts         (l5_counts),
                .o_l5_word0          (l5_word0),
                .o_l5_word1          (l5_word1),
                .o_cfg_magic_ok      (cfg_magic_ok),
                .o_cfg_period        (cfg_period),
                .o_cfg_count         (cfg_count),
                .o_cfg_width         (cfg_width),
                .o_cfg_seq           (cfg_seq),
                .o_cfg_flags         (cfg_flags),
                .sync_mbx            (sync_mbx),
                .sync_loaded         (sync_loaded),
                .o_geom_size         (geom_size),
                .o_geom_seq          (geom_seq),
                .o_lasr0_cfg         (lasr_cfg),
                .o_lasr1_cfg         (lasr1_cfg_out),
                .o_tw_en             (l5_tw_en),
                .o_tw_bank           (mm_tw_bank),
                .o_tw_addr           (mm_tw_addr),
                .o_tw_data           (mm_tw_data),
                .o_mm0_cfg           (mm_cfg),
                .o_mm1_cfg           (mm1_cfg_out[127:0]),
                .o_mm0_stat          (mm_stat),
                .o_mm1_stat          (mm1_cfg_out[191:128]),
                .i_mm_ack            ({mm1_ack_in, mm_ack}),
                .i_bw_v              (mm_bw_v),
                .i_bw_d              (mm_bw_d),
                .o_ld_stat           (mm_ld_stat)
            );
            assign mm_tw_en   = l5_tw_en[0];
            assign mm1_tw_out = {l5_tw_en[1], mm_tw_bank, mm_tw_addr, mm_tw_data};

            trigger_gen u_trig (
                .clk            (clk),
                .rst_n          (rst_n),
                .cfg_period     (cfg_period),
                .cfg_count      (cfg_count),
                .cfg_width      (cfg_width),
                .cfg_seq        (cfg_seq),
                .cfg_flags      (cfg_flags),
                .cam_trig       (cam_trig),
                .o_trig_pulses  (trig_pulses),
                .o_trig_left    (trig_left),
                .o_trig_run     (trig_run),
                .o_trig_cont    (trig_cont),
                .o_trig_active  (trig_active),
                .o_cfg_seq_seen (cfg_seq_seen)
            );
        end else begin : g_no_lane5
            assign rd_go             = 1'b0;
            assign rd_burstcount     = 8'd0;
            assign rd_addr           = 32'd0;
            assign rd_en             = 1'b0;
            assign mm_ld_stat        = 32'd0;
            assign l5_counts         = 32'd0;
            assign l5_word0          = 32'd0;
            assign l5_word1          = 32'd0;
            assign cfg_magic_ok      = 1'b0;
            assign cfg_period        = 32'd0;
            assign cfg_count         = 8'd0;
            assign cfg_width         = 32'd0;
            assign cfg_seq           = 32'd0;
            assign cfg_flags         = 32'd0;
            assign sync_mbx          = {14*32{1'b0}};
            assign sync_loaded       = 1'b0;
            assign geom_size         = 32'd0;
            assign geom_seq          = 32'd0;
            assign cam_trig          = 1'b0;
            assign trig_pulses       = 32'd0;
            assign trig_left         = 8'd0;
            assign trig_run          = 1'b0;
            assign trig_cont         = 1'b0;
            assign trig_active       = 1'b0;
            assign cfg_seq_seen      = 32'd0;
            assign lasr_cfg          = lasr_cfg_in;
            assign lasr1_cfg_out     = 96'd0;
            assign l5_tw_en          = 2'b00;
            assign mm_tw_en          = mm_tw_in[43];
            assign mm_tw_bank        = mm_tw_in[42:41];
            assign mm_tw_addr        = mm_tw_in[40:32];
            assign mm_tw_data        = mm_tw_in[31:0];
            assign mm_cfg            = mm_cfg_in[127:0];
            assign mm_stat           = mm_cfg_in[191:128];
            assign mm1_tw_out        = 44'd0;
            assign mm1_cfg_out       = 192'd0;
        end
    endgenerate
    assign mm_ack_out = mm_ack;

    // 🪦 o_wants_bus was here: the Lane 4 bus request for lane4_arbiter (request
    // only with a burst in progress or words in the FIFO, ERRORS 1.12). There has
    // been no arbiter and no shared bus since 09.10.2026: each writer has its own
    // axi_mux port.

    // =========================================================================
    // AXI4 write port: cam_axi_wr.v
    // =========================================================================
    // The FSM's burst goes here whole, one word per clock (wd_*), and leaves on
    // AXI once fully assembled. axi_room is a register, one bit enters the FSM
    // cone. MAX_WORDS is the longest burst: a row of 1 << ROW_BURST_LOG2.
    cam_axi_wr #(
        .DW        (256),
        .AW        (30),
        .MAX_WORDS (1 << ROW_BURST_LOG2)
    ) u_axi_wr (
        .clk         (clk),
        .rst_n       (rst_n),
        .s_en        (wd_en),
        .s_data      (wd_data),
        .s_first     (wd_first),
        .s_last      (wd_last),
        .s_addr      (burst_addr),
        .s_room      (axi_room),
        .m_awvalid   (m_awvalid),
        .m_awready   (m_awready),
        .m_awaddr    (m_awaddr),
        .m_awlen     (m_awlen),
        .m_wvalid    (m_wvalid),
        .m_wready    (m_wready),
        .m_wdata     (m_wdata),
        .m_wstrb     (m_wstrb),
        .m_wlast     (m_wlast),
        .m_bvalid    (m_bvalid),
        .m_bready    (m_bready),
        .m_bresp     (m_bresp),
        .o_aw_cnt    (axi_aw_cnt),
        .o_b_cnt     (axi_b_cnt),
        .o_bresp_err (axi_bresp_err)
    );

    // =========================================================================
    // Laser line finder and profile block: laser_prof.v (stage L2a)
    // =========================================================================
    //
    // Pixel words are taken on the same ST_WRITE clock in which they go into the
    // frame row (in a frame without rows, dry_frame, they do not go, but the clock
    // is the same), and handed to the module through one register: nothing comes
    // back from here into the writer FSM except registers prof_ready and prof_busy.
    // The row is the sensor line index (row_line_idx, which also gives the row
    // address), the word is row_word. The header burst, telemetry and the row tail
    // (row_word >= ROW_WORDS) do not go to the finder.
    reg        lz_valid;
    reg [31:0] lz_word;
    reg [10:0] lz_row;
    reg [15:0] lz_k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) lz_valid <= 1'b0;
        else        lz_valid <= system_ready && (state == ST_WRITE) && !diag_burst &&
                                burst_nz_q && (row_word < ROW_WORDS);
    end
    always @(posedge clk) begin
        lz_word <= fifo_rd_data[31:0];
        lz_row  <= row_line_idx[10:0];
        lz_k    <= row_word;
    end

    generate if (ENABLE_LASER != 0) begin : g_laser
    laser_prof #(
        .COLS     ({16'd0, FRAME_WIDTH}),
        .ROW_BITS (11),
        .CAM_ID   (CAM_ID)
    ) u_prof (
        .clk             (clk),
        .rst_n           (rst_n),
        .hw_timestamp    (hw_timestamp),
        .cfg_thresh      (lasr_cfg[7:0]),
        .cfg_offset      (lasr_cfg[15:8]),
        .cfg_gap         (lasr_cfg[19:16]),
        .cfg_win_lo      (lasr_cfg[30:20]),
        .cfg_win_hi      (lasr_cfg[41:31]),
        .cfg_enable      (lasr_cfg[42]),
        .cfg_min_sum     (lasr_cfg[79:64]),
        .cfg_width_max   (lasr_cfg[95:80]),
        .frame_start     (lz_frame_start),
        .pix_valid       (lz_valid),
        .pix_word        (lz_word),
        .pix_row         (lz_row),
        .pix_k           (lz_k),
        .lines_dropped   (diag_lines_drop),
        .frame_close     (lz_close),
        .close_frame_id  (lz_frame_id),
        .close_block_base (lz_block_base),
        .close_rows      (lz_rows),
        .strobe_ts       (strobe_ts),
        .prof_busy       (prof_busy),
        .prof_ready      (prof_ready),
        .prof_addr       (prof_addr),
        // prof_first is the second ST_GO clock (before 09.10.2026, ST_PAUSE): one
        // clock before the first word, like the header pointer.
        .prof_first      ((state == ST_GO) && go_cnt[0] && prof_burst),
        .prof_next       ((state == ST_WRITE) && prof_burst),
        .prof_q          (prof_q),
        .prof_done       ((state == ST_BURST_END) && prof_burst),
        .tw_en           (mm_tw_en),
        .tw_bank         (mm_tw_bank),
        .tw_addr         (mm_tw_addr),
        .tw_data         (mm_tw_data),
        .mm_cfg          (mm_cfg),
        .mm_load_ack     (mm_ack),
        .o_blocks        (prof_blocks),
        .o_no_line       (prof_no_line)
    );
    end else begin : g_no_laser
        assign mm_ack       = 1'b0;
        assign prof_busy    = 1'b0;
        assign prof_ready   = 1'b0;
        assign prof_addr    = 32'd0;
        assign prof_q       = 32'd0;
        assign prof_blocks  = 16'd0;
        assign prof_no_line = 16'd0;
    end endgenerate

    // synthesis translate_off
    // Shadow-flag check (RTL_RULES 3.1): the early flags must equal the compares
    // they replace on every clock, or the timing refactor changed behaviour.
    always @(posedge clk or negedge rst_n) begin
        if (rst_n && (burst_nz_q !== (burst_idx != 16'd0))) begin
            $display("ERROR cam_channel %m: burst_nz_q=%b burst_idx=%0d at %0t", burst_nz_q, burst_idx, $time);
            $fatal(1);
        end
        if (rst_n && (row_nl_q !== (row_word < (ROW_WORDS - 16'd1)))) begin
            $display("ERROR cam_channel %m: row_nl_q=%b row_word=%0d at %0t", row_nl_q, row_word, $time);
            $fatal(1);
        end
    end
    // synthesis translate_on

endmodule
