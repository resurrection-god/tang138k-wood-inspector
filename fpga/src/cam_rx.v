// ============================================================================
// Module      : cam_rx
// Receive of one camera: D-PHY watchdog, hard D-PHY RX, CSI-2 RAW8 bridge,
// packer into 32-bit words and the FIFO between mipi_byte_clk and clk (lane4_clk).
// docs/ARCHITECTURE.md § 3.1-3.2.
//
// Split out of fpga_camera_ddr3_streamer.v on 24.09.2026 (step 4, together with
// cam_channel.v, lane5_reader.v, trigger_gen.v) WITHOUT any logic change: the blocks
// below were moved line for line, the fpga/sim bench gives the same bus trace to the
// cycle. Only the FIFO read, the watchdog reset (geometry was applied under it until
// 19.10.2026) and the telemetry counters, already brought into the clk domain, are
// brought out.
//
// The instance names u_mipi_dphy_rx / u_stream_bridge / u_pixel_fifo are unchanged:
// create_clock in scanner.sdc refers to u_mipi_dphy_rx/.../RX_CLK_O.
// ============================================================================

`timescale 1ns / 1ps

module cam_rx #(
    parameter [15:0]  FRAME_WIDTH    = 16'd1080,
    parameter [15:0]  FRAME_HEIGHT   = 16'd1280,
    parameter integer FIFO_ADDR_BITS = 13,
    parameter integer LANE_CLK_HZ    = 133_333_333,
    parameter [31:0]  WD_IDLE_CYCLES = 32'd0,
    // Width of the row index in the FIFO word; set by the channel (cam_channel.v).
    parameter integer LINE_IDX_BITS  = 12
)(
    input  wire        clk,                 // lane4_clk
    input  wire        rst_n,
    input  wire        cam_enable,
    input  wire        cam_strobe,

    inout  wire        cam1_ck_p,
    inout  wire        cam1_ck_n,
    inout  wire        cam1_d0_p,
    inout  wire        cam1_d0_n,
    inout  wire        cam1_d1_p,
    inout  wire        cam1_d1_n,

    // Receive geometry. Neither reaches the bridge's logic: the row length and
    // the line count wrap are build constants (RUNTIME_LINE_BYTES = 0,
    // RUNTIME_FRAME_LINES = 0 below); the channel applies the height in its own
    // writer (19.10.2026, ERRORS 2.11).
    input  wire [15:0] cfg_line_bytes,
    input  wire [15:0] cfg_frame_lines,

    output wire        o_mipi_byte_clk,
    output wire        o_cam_pixel_seen,
    output wire        o_wd_reset,          // the watchdog holds the receiver in reset

    // FIFO read, clk domain. Word: [31:0] data, [32] first word of a row,
    // [33] ... of row 0 of the frame, [34+:LINE_IDX_BITS] sensor row index.
    input  wire                            fifo_rd_en,
    output wire [32+2+LINE_IDX_BITS-1:0]   fifo_rd_data,
    output wire [FIFO_ADDR_BITS:0]         fifo_rd_used,

    // Telemetry, already in the clk domain (header words 8..13 and 34).
    output wire [31:0] o_diag_header_count,
    output wire [31:0] o_diag_line_count,
    output wire [31:0] o_diag_bad_wc_count,
    output wire [15:0] o_diag_last_wc,
    output wire [1:0]  o_diag_phase,
    output wire        o_diag_overflow,
    output wire [31:0] o_diag_seen_wc,
    output wire [15:0] o_diag_seen_hits,
    output wire [15:0] o_diag_lines_drop,
    output wire [15:0] o_diag_byte_clk_ticks
);

    // Geometry needed by the packer and by the FIFO row admission threshold. The same
    // as in cam_channel.v (ROW_WORDS, FIFO_WIDTH, FIFO_CNT_W): computed from the same
    // parameters.
    localparam [15:0]  ROW_WORDS  = FRAME_WIDTH >> 2;
    localparam integer FIFO_WIDTH = 32 + 2 + LINE_IDX_BITS;

    // =========================================================================
    // 1. Gowin Hard MIPI D-PHY Receiver for Port 1 (J11)
    // Board mapping: Cam D0/D1 physical pairs route to Hard D-PHY Lanes 2 and 3
    // =========================================================================
    wire [15:0] dphy_lane2_data, dphy_lane3_data;
    wire        dphy_lane2_vld,  dphy_lane3_vld;
    wire        mipi_byte_clk;

    assign o_mipi_byte_clk = mipi_byte_clk;

    // =========================================================================
    // MIPI RX reset control + watchdog
    //
    // The hard D-PHY must NOT be released while the sensor is silent. If it is,
    // its word aligner latches onto the LP/preamble pattern instead of the 0xB8
    // sync byte and never recovers - visible as RAW8 candidates whose word count
    // is made of preamble bytes (0xA8 / 0xAA) and zero accepted line headers.
    //
    // The reference design (camera_mipi_rx_frontend in
    // sc132m_mipi_hdmi_pointsearch_...) releases the receiver ~100 ms AFTER the
    // I2C init has started the camera, and re-resets it if frame activity stops.
    // Here there is no direct "I2C done" signal, so the same effect is obtained
    // with a self-correcting watchdog: while no complete line is being seen, the
    // receiver is re-reset roughly once a second until it locks.
    //
    // This runs in the 50 MHz `clk` domain on purpose - it cannot live in the
    // byte_clk domain, because resetting the D-PHY stops byte_clk itself.
    // =========================================================================
    // Same intervals as the reference design: 3 s of no frame activity before a
    // 0.2 s reset pulse. A shorter idle timeout makes the watchdog thrash - it
    // then holds the D-PHY in reset a large fraction of the time, which stops
    // byte_clk and makes the receiver look dead.
    // WD_IDLE_CYCLES != 0 replaces the 3 s only in the bench: otherwise the check
    // "no reset after a pause longer than the threshold" would take 300 million cycles.
    localparam [31:0] MIPI_WD_IDLE_CYCLES  = (WD_IDLE_CYCLES != 0) ? WD_IDLE_CYCLES
                                                                   : LANE_CLK_HZ * 3;   // 3.0 s
    localparam [31:0] MIPI_WD_RESET_CYCLES = LANE_CLK_HZ / 5;   // 0.2 s

    // Strobe-timed release (ERRORS 2.4, 2.5; former 31.30, failure (b)).
    //
    // The receiver only locks when it is released INTO active lines (ERRORS 2.5).
    // The timer watchdog above releases it at a random moment; at 10 fps the
    // lines occupy ~4 % of the time, so in trigger mode - where silence between
    // frames is normal - it almost always lands in silence and the aligner
    // sticks on the preamble: the camera exposes every trigger, the writer gets
    // no line. Measured 11 series of 13 lost this way.
    //
    // The camera's Strobe Out marks the exposure, and readout follows its end
    // deterministically: on the board the frame header reaches the writer
    // 15.19 ms after the strobe rises with a 15.04 ms strobe, jitter of tens of
    // microseconds. So: if the exposure that just started was preceded by a
    // frame with no complete line, hold the D-PHY in reset through the exposure
    // and release it STROBE_RELEASE_CYCLES after the strobe falls - inside the
    // readout. That frame arrives partial (and is dropped by the writer's index
    // checks); the next one is whole. While lines keep arriving the strobe only
    // opens a new observation window and never touches the reset.
    //
    // Every strobe also clears the idle count, so the timer watchdog stays out
    // of the way while the camera is demonstrably alive. With Strobe Out not
    // connected none of this fires and the behaviour is exactly the old one.
    //
    // Once a strobe has been seen the timer watchdog is switched off altogether
    // (wd_strobe_present). Clearing the idle count on each strobe was not
    // enough for capture by single triggers (stage 3, ERRORS 2.5): between two
    // captures the camera is silent for seconds, the 3 s timer fired in that
    // silence and released the receiver INTO it - the aligner stuck on the
    // preamble, the next trigger then took the hold path below and lost its
    // frame, and a second trigger was needed. Measured on 15.09.2026: 8 of 10
    // "trigger + 2 frames" cycles failed, every trigger counted by the camera,
    // the FPGA frame counter still. With strobes present, silence between
    // frames is normal and a real loss of lock is already caught by the
    // strobe path (a strobe with no line since the previous one). The timer
    // stays as the only recovery while no strobe has ever arrived.
    localparam [31:0] STROBE_RELEASE_CYCLES = LANE_CLK_HZ / 2000;  // 0.5 ms

    wire       bridge_line_ok_toggle;
    reg [2:0]  wd_line_tog_sync;
    reg [28:0] wd_idle_cnt;
    reg [28:0] wd_rst_cnt;
    reg        wd_reset;
    reg [2:0]  wd_strobe_sync;
    reg        wd_lines_seen;      // a complete line since the last strobe
    reg        wd_strobe_hold;     // holding reset for a strobe-timed release
    reg [19:0] wd_release_cnt;     // counts down after the strobe falls
    reg        wd_anchor_tog;      // flips on every strobe fall: frame-start anchor for the bridge
    reg        wd_strobe_present;  // a strobe since capture was enabled: timer watchdog off

    wire wd_line_edge   = (wd_line_tog_sync[2] != wd_line_tog_sync[1]);
    wire wd_strobe_rise = (wd_strobe_sync[2:1] == 2'b01);
    wire wd_strobe_fall = (wd_strobe_sync[2:1] == 2'b10);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wd_line_tog_sync <= 3'b000;
            wd_idle_cnt      <= 29'd0;
            wd_rst_cnt       <= 29'd0;
            wd_reset         <= 1'b1;   // hold the receiver in reset out of reset
            wd_strobe_sync   <= 3'b000;
            wd_lines_seen    <= 1'b0;
            wd_strobe_hold   <= 1'b0;
            wd_release_cnt   <= 20'd0;
            wd_anchor_tog    <= 1'b0;
            wd_strobe_present <= 1'b0;
        end else begin
            wd_line_tog_sync <= {wd_line_tog_sync[1:0], bridge_line_ok_toggle};
            wd_strobe_sync   <= {wd_strobe_sync[1:0], cam_strobe};
            // Kept apart from the priority chain below: it must be set by a
            // strobe whichever branch that cycle takes.
            if (!cam_enable)          wd_strobe_present <= 1'b0;
            else if (wd_strobe_rise)  wd_strobe_present <= 1'b1;
            // The readout of a frame begins right after its exposure ends
            // (0.15 ms on the board); the bridge re-anchors its line count on
            // this, in its own domain (anchor_sync_byte further down).
            if (wd_strobe_fall)
                wd_anchor_tog <= ~wd_anchor_tog;

            if (wd_rst_cnt != 29'd0) begin
                // Reset pulse in progress.
                wd_rst_cnt  <= wd_rst_cnt - 29'd1;
                wd_idle_cnt <= 29'd0;
                wd_reset    <= 1'b1;
            end else if (!cam_enable) begin
                // Capture disabled: keep the receiver parked in reset so it is
                // released into an already-active line rather than an idle one.
                //
                wd_idle_cnt    <= 29'd0;
                wd_reset       <= 1'b1;
                wd_lines_seen  <= 1'b0;
                wd_strobe_hold <= 1'b0;
                wd_release_cnt <= 20'd0;
            end else if (wd_strobe_rise && !wd_lines_seen && !wd_line_edge && !wd_strobe_hold) begin
                // The previous exposure produced no line: the receiver is not
                // locked. Hold it in reset through this exposure.
                wd_strobe_hold <= 1'b1;
                wd_release_cnt <= 20'd0;
                wd_idle_cnt    <= 29'd0;
                wd_reset       <= 1'b1;
            end else if (wd_strobe_hold) begin
                wd_reset <= 1'b1;
                if (wd_strobe_fall) begin
                    wd_release_cnt <= STROBE_RELEASE_CYCLES[19:0];
                end else if (wd_release_cnt == 20'd1 || wd_idle_cnt[27]) begin
                    // Release inside the readout - or give up after ~1.3 s if
                    // the strobe never falls.
                    wd_strobe_hold <= 1'b0;
                    wd_release_cnt <= 20'd0;
                    wd_idle_cnt    <= 29'd0;
                    wd_reset       <= 1'b0;
                end else begin
                    if (wd_release_cnt != 20'd0)
                        wd_release_cnt <= wd_release_cnt - 20'd1;
                    wd_idle_cnt <= wd_idle_cnt + 29'd1;
                end
            end else if (wd_line_edge) begin
                // A complete line was seen - the receiver is locked, stay running.
                wd_idle_cnt   <= 29'd0;
                wd_reset      <= 1'b0;
                wd_lines_seen <= 1'b1;
            end else if (wd_strobe_rise) begin
                // Locked camera, new exposure: open the next observation window.
                wd_idle_cnt   <= 29'd0;
                wd_reset      <= 1'b0;
                wd_lines_seen <= 1'b0;
            end else if (!wd_strobe_present && ({3'd0, wd_idle_cnt} >= MIPI_WD_IDLE_CYCLES)) begin
                // No line for 3 s and no strobe ever seen: kick the receiver and
                // try again. With strobes present silence is normal (see
                // wd_strobe_present above) and this branch must not fire.
                wd_rst_cnt    <= MIPI_WD_RESET_CYCLES[28:0];
                wd_idle_cnt   <= 29'd0;
                wd_reset      <= 1'b1;
                wd_lines_seen <= 1'b0;
            end else begin
                wd_idle_cnt <= wd_idle_cnt + 29'd1;
                wd_reset    <= 1'b0;
            end
        end
    end

    // ⚠ Do NOT add sources here. This net is an asynchronous reset with a large
    // fan-out and sits on one of the eight long-wire global networks, which are
    // occupied 8/8 (the comment at rst_n in camera_csi2_raw8_stream_bridge.sv).
    // On 22.09.2026 geom_hold was added here for changing the receive geometry - all
    // timing PASS, the camera works, and Ethernet receive degraded for three builds
    // in a row (ERRORS former 31.53). Geometry was then applied inside the existing
    // watchdog reset; since 19.10.2026 the bridge does not read it at all and the
    // writer takes it at a slot boundary (cam_channel.v, ERRORS 2.11).
    wire mipi_rx_rst_n = rst_n & ~wd_reset;

    Gowin_MIPI_DPHY_RX4 u_mipi_dphy_rx (
        .d0ln_hsrxd        (),
        .d1ln_hsrxd        (),
        .d2ln_hsrxd        (dphy_lane2_data),
        .d3ln_hsrxd        (dphy_lane3_data),
        .d0ln_hsrxd_vld    (),
        .d1ln_hsrxd_vld    (),
        .d2ln_hsrxd_vld    (dphy_lane2_vld),
        .d3ln_hsrxd_vld    (dphy_lane3_vld),
        .di_lprx0_n        (), .di_lprx0_p(), .di_lprx1_n(), .di_lprx1_p(),
        .di_lprx2_n        (), .di_lprx2_p(), .di_lprx3_n(), .di_lprx3_p(),
        .di_lprxck_n       (), .di_lprxck_p(),
        .rx_clk_o          (mipi_byte_clk),
        .deskew_error      (), .d0ln_deskew_done(), .d1ln_deskew_done(),
        .d2ln_deskew_done  (), .d3ln_deskew_done(),
        .ck_p              (cam1_ck_p),
        .ck_n              (cam1_ck_n),
        .rx0_p             (), .rx0_n(), .rx1_p(), .rx1_n(),
        .rx2_p             (cam1_d0_p),
        .rx2_n             (cam1_d0_n),
        .rx3_p             (cam1_d1_p),
        .rx3_n             (cam1_d1_n),
        .lprx_en_ck        (1'b1),
        .lprx_en_d0        (1'b0), .lprx_en_d1(1'b0),
        .lprx_en_d2        (1'b1),
        .lprx_en_d3        (1'b1),
        .hsrx_odten_ck     (1'b1),
        .hsrx_odten_d0     (1'b0), .hsrx_odten_d1(1'b0),
        .hsrx_odten_d2     (1'b1),
        .hsrx_odten_d3     (1'b1),
        .d0ln_hsrx_dren    (1'b0), .d1ln_hsrx_dren(1'b0),
        .d2ln_hsrx_dren    (1'b1),
        .d3ln_hsrx_dren    (1'b1),
        .hsrx_en_ck        (1'b1),
        // 8-bit mode, matching the working reference design
        // (sc132m_mipi_hdmi_pointsearch_720p60_overlay_aligned_debug).
        // Each lane then delivers ONE byte per byte_clk in hsrxd[7:0] and a lane
        // pair carries two consecutive CSI-2 bytes. Running this in 16-bit mode
        // (hs_8bit_mode = 0, rx_clk_1x = 0) is what left the receiver unable to
        // lock onto a line header at all. hs_8bit_mode and rx_clk_1x belong
        // together; walign_by enables word alignment on the sync byte.
        .hs_8bit_mode      (1'b1),
        .rx_clk_1x         (1'b1),
        .rx_invert         (1'b0),
        .lalign_en         (1'b0),
        .walign_by         (1'b1),
        .do_lptx0_n        (1'b0), .do_lptx0_p(1'b0), .do_lptx1_n(1'b0), .do_lptx1_p(1'b0),
        .do_lptx2_n        (1'b0), .do_lptx2_p(1'b0), .do_lptx3_n(1'b0), .do_lptx3_p(1'b0),
        .do_lptxck_n       (1'b0), .do_lptxck_p(1'b0),
        .lptx_en_ck        (1'b0), .lptx_en_d0(1'b0), .lptx_en_d1(1'b0), .lptx_en_d2(1'b0), .lptx_en_d3(1'b0),
        .byte_lendian      (1'b0),
        .hsrx_stop         (1'b0),
        .pwron             (mipi_rx_rst_n),
        .reset             (~mipi_rx_rst_n),
        .deskew_lnsel      (3'd0),
        .deskew_mth        (13'd0),
        .deskew_owval      (7'd0),
        .deskew_req        (1'b0),
        .drst_n            (rst_n),
        .one_byte0_match   (1'b0),
        .word_lendian      (1'b0),
        .fifo_rd_std       (3'd0),
        .deskew_by         (1'b0),
        .deskew_en_oedge   (1'b0),
        .deskew_half_opening(6'd0),
        .deskew_lsb_mode   (2'd0),
        .deskew_m          (3'd0),
        .deskew_mset       (7'd0),
        .deskew_oclkedg_en (1'b0),
        .eqcs_lane0        (3'b100),
        .eqcs_lane1        (3'b100),
        .eqcs_lane2        (3'b100),
        .eqcs_lane3        (3'b100),
        .eqcs_ck           (3'b100),
        .eqrs_lane0        (3'b100),
        .eqrs_lane1        (3'b100),
        .eqrs_lane2        (3'b100),
        .eqrs_lane3        (3'b100),
        .eqrs_ck           (3'b100),
        .hsrx_dlydir_lane0 (1'b0), .hsrx_dlydir_lane1(1'b0), .hsrx_dlydir_lane2(1'b0), .hsrx_dlydir_lane3(1'b0), .hsrx_dlydir_ck(1'b0),
        .hsrx_dlyldn_lane0 (1'b0), .hsrx_dlyldn_lane1(1'b0), .hsrx_dlyldn_lane2(1'b0), .hsrx_dlyldn_lane3(1'b0), .hsrx_dlyldn_ck(1'b0),
        .hsrx_dlymv_lane0  (1'b0),  .hsrx_dlymv_lane1(1'b0),  .hsrx_dlymv_lane2(1'b0),  .hsrx_dlymv_lane3(1'b0),  .hsrx_dlymv_ck(1'b0),
        .walign_dvld       (1'b0)
    );

    // =========================================================================
    // 2. Hardware Auto-Phase RAW8 Stream Bridge
    // =========================================================================
    wire [1:0]  bridge_payload_dv;
    wire [15:0] bridge_payload;
    wire        bridge_line_start_pulse;
    wire        bridge_pixel_seen;
    wire        bridge_frame_done_toggle;

    assign o_cam_pixel_seen = bridge_pixel_seen;

    wire        fifo_wr_full;

    // Declared BEFORE the instantiation below: connecting an undeclared name to a
    // port makes Verilog implicitly create a 1-bit net, silently truncating these
    // 32- and 16-bit counters (Gowin reports that as WARN EX3638).
    wire [31:0] bridge_header_count;
    wire [31:0] bridge_line_count;
    wire [31:0] bridge_bad_wc_count;
    wire [15:0] bridge_last_wc;
    wire        bridge_last_lane_swap;
    wire        bridge_last_phase_high;
    wire        bridge_overflow_seen;
    wire [15:0] bridge_seen_wc_last;
    wire [15:0] bridge_seen_wc_hits_exp;
    wire [15:0] bridge_seen_wc_hits_alt;
    wire [15:0] bridge_line_index;

    wire bridge_frame_anchor;      // strobe fall, synchronised to mipi_byte_clk

    camera_csi2_raw8_stream_bridge #(
        .LINE_BYTES           (FRAME_WIDTH),
        .FRAME_LINES          (FRAME_HEIGHT),
        .RUNTIME_LINE_BYTES   (0),           // row length is a constant, ERRORS former 31.53
        // Line count wraps at FRAME_HEIGHT, not at the ROI height: the strobe fall
        // and the inter-frame gap start every frame (19.10.2026, ERRORS 2.11).
        .RUNTIME_FRAME_LINES  (0)
    ) u_stream_bridge (
        .frame_anchor         (bridge_frame_anchor),
        .byte_clk             (mipi_byte_clk),
        .rst_n                (mipi_rx_rst_n),   // reset together with the D-PHY
        .enable               (cam_enable),
        .cfg_line_bytes       (cfg_line_bytes),
        .cfg_frame_lines      (cfg_frame_lines),

        .lane0_data           (dphy_lane2_data),
        .lane1_data           (dphy_lane3_data),
        .lane0_valid          (dphy_lane2_vld),
        .lane1_valid          (dphy_lane3_vld),

        .stream_sp_en         (),
        .stream_lp_av_en      (bridge_line_start_pulse),   // pulses on the first payload beat of a line
        .stream_dt            (),
        .stream_wc            (),
        .stream_payload_dv    (bridge_payload_dv),
        .stream_payload       (bridge_payload),

        .raw8_header_toggle   (),
        .raw8_line_ok_toggle  (bridge_line_ok_toggle),   // feeds the MIPI RX watchdog
        .raw8_bad_wc_toggle   (),
        .frame_done_toggle    (bridge_frame_done_toggle),
        .pixel_seen           (bridge_pixel_seen),
        .overflow_seen        (bridge_overflow_seen),
        .raw8_header_count    (bridge_header_count),
        .raw8_line_count      (bridge_line_count),
        .raw8_bad_wc_count    (bridge_bad_wc_count),
        .last_word_count      (bridge_last_wc),
        .last_line_byte_count (),
        .last_payload_word    (),
        .last_lane_swap       (bridge_last_lane_swap),
        .last_phase_high      (bridge_last_phase_high),
        .line_index           (bridge_line_index),
        .seen_wc_last          (bridge_seen_wc_last),
        .seen_wc_hits_expected (bridge_seen_wc_hits_exp),
        .seen_wc_hits_alt      (bridge_seen_wc_hits_alt)
    );

    // -------------------------------------------------------------------------
    // Per-lane D-PHY activity counters (byte_clk domain).
    //
    // These answer, without any interpretation of CSI-2 structure, the question
    // "is this lane receiving anything at all?". A lane that strobes valid but
    // only ever delivers 0x00 is physically dead even though the receiver thinks
    // it is active - which is exactly the failure mode seen here.
    // Counters saturate rather than wrap so a glance is enough.
    // -------------------------------------------------------------------------
    reg [15:0] lane0_vld_cnt, lane1_vld_cnt;
    reg [15:0] lane0_nz_cnt,  lane1_nz_cnt;

    // Free-running byte_clk tick counter, used to measure the recovered MIPI byte
    // clock frequency from the 50 MHz domain. The byte clock comes from the sensor,
    // so this is the only way to know the actual link rate: byte_clk = datarate/8
    // in 8-bit mode, i.e. 75 MHz means 600 Mbps per lane.
    // The counter is published as GRAY code, because it is read from another clock
    // domain. It used to be plain binary with a two-flop synchroniser and a comment
    // saying small tearing was fine here. It is not: a binary counter rolling over
    // (0x0000FFFF -> 0x00010000) changes every low bit at once, so a sample taken
    // across that edge can read almost anything, and the difference between two
    // such samples is meaningless. In Gray code exactly one bit changes per
    // increment, so the worst a torn sample can be is off by one count.
    //
    // A /64 prescaler runs first so the counter that crosses domains can be small.
    // At the full byte rate the 163.84 us window counts ~26k ticks and needed 16
    // bits, which cost too much: a 16-bit adder in the byte_clk domain and a
    // 15-deep Gray decode chain in the DDR3 domain took ae350_ahb_clk to 99.825 MHz
    // and mipi_byte_clk to 161.176, both under their constraints. Prescaled by 64
    // the same window is ~410 counts, 12 bits carry it with 10x headroom, and the
    // host multiplies back. Resolution drops to ~0.39 MHz, which is nothing next to
    // the question being asked (is the link near 160 MHz or somewhere else).
    reg [5:0]  byte_clk_pre;
    reg [11:0] byte_clk_ticks;
    reg [11:0] byte_clk_gray;
    always @(posedge mipi_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_clk_pre   <= 6'd0;
            byte_clk_ticks <= 12'd0;
            byte_clk_gray  <= 12'd0;
        end else begin
            byte_clk_pre <= byte_clk_pre + 6'd1;
            if (byte_clk_pre == 6'd63) begin
                byte_clk_ticks <= byte_clk_ticks + 12'd1;
                byte_clk_gray  <= (byte_clk_ticks + 12'd1) ^ ((byte_clk_ticks + 12'd1) >> 1);
            end
        end
    end

    always @(posedge mipi_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            lane0_vld_cnt <= 16'd0; lane1_vld_cnt <= 16'd0;
            lane0_nz_cnt  <= 16'd0; lane1_nz_cnt  <= 16'd0;
        end else begin
            if (dphy_lane2_vld && (lane0_vld_cnt != 16'hFFFF))
                lane0_vld_cnt <= lane0_vld_cnt + 16'd1;
            if (dphy_lane3_vld && (lane1_vld_cnt != 16'hFFFF))
                lane1_vld_cnt <= lane1_vld_cnt + 16'd1;
            if (dphy_lane2_vld && (dphy_lane2_data[7:0] != 8'd0) && (lane0_nz_cnt != 16'hFFFF))
                lane0_nz_cnt <= lane0_nz_cnt + 16'd1;
            if (dphy_lane3_vld && (dphy_lane3_data[7:0] != 8'd0) && (lane1_nz_cnt != 16'hFFFF))
                lane1_nz_cnt <= lane1_nz_cnt + 16'd1;
        end
    end

    // -------------------------------------------------------------------------
    // Diagnostic counters, byte_clk -> clk. These are read-only telemetry that
    // ends up in VID0 header words 6..15, so a torn sample is acceptable; the
    // double register is only there to keep metastability out of the DDR3 domain.
    // -------------------------------------------------------------------------
    // Byte-clock frequency measurement.
    //
    // The gate is a power of two (2^13 cycles of the 50 MHz clock = 163.84 us) so
    // no divider is needed - a division by a non-power-of-two constant here made
    // GowinSynthesis fall over. With the /64 prescaler the host converts:
    //     f_byte = ticks * 64 * 50e6 / 8192 = ticks * 390625 Hz
    // At 160 MHz the count is about 410; at 75 MHz about 192.
    reg [11:0] bclk_s0, bclk_s1, bclk_prev;
    reg [13:0] bclk_win_cnt;
    reg [15:0] diag_byte_clk_ticks;

    // Gray -> binary: bin[11] = g[11], bin[i] = bin[i+1] ^ g[i]. This is a leaf
    // path feeding only the subtraction below, never the streaming FSM.
    function [11:0] gray2bin;
        input [11:0] g;
        integer i;
        begin
            gray2bin[11] = g[11];
            for (i = 10; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i + 1] ^ g[i];
        end
    endfunction

    wire [11:0] bclk_now   = gray2bin(bclk_s1);
    wire [11:0] bclk_delta = bclk_now - bclk_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bclk_s0 <= 12'd0; bclk_s1 <= 12'd0; bclk_prev <= 12'd0;
            bclk_win_cnt <= 14'd0;
            diag_byte_clk_ticks <= 16'd0;
        end else begin
            bclk_s0 <= byte_clk_gray;    // two-flop synchroniser, Gray-coded source
            bclk_s1 <= bclk_s0;
            bclk_win_cnt <= bclk_win_cnt + 14'd1;
            if (bclk_win_cnt == 14'd0) begin
                bclk_prev <= bclk_now;
                // Last NON-ZERO window, not a peak hold.
                //
                // Peak hold was here so that a window caught while the watchdog
                // parks the D-PHY in reset - byte_clk stopped, delta 0 - would not
                // report 0 as the link rate. Combined with the torn binary samples
                // above it did something much worse: one bogus delta pinned the
                // reading at its maximum forever, and the telemetry drifted up to
                // 65525 (a claimed 400 MHz byte clock against an expected ~160).
                // Keeping the last window that actually counted something answers
                // the watchdog case without ever ratcheting upward.
                if (bclk_delta != 12'd0)
                    diag_byte_clk_ticks <= {4'd0, bclk_delta};
            end
        end
    end

    // These two belong to the packer further down, but the diagnostic
    // synchroniser below samples them, and Verilog does not allow a reg to be
    // referenced before it is declared. GowinSynthesis accepts the reversed
    // order silently; Icarus refuses to elaborate ("Unable to bind wire/reg
    // ... Check for declaration after use"), which is what kept fpga/sim from
    // building. Keep the declarations here.
    reg        fifo_overflow_sticky;
    reg [15:0] lines_dropped_cnt;    // refused for want of FIFO space - the defect
                                     // (16 bits: telemetry only ever carried [15:0])

    reg [31:0] diag_lane_vld_s0, diag_lane_vld;
    reg [31:0] diag_lane_nz_s0,  diag_lane_nz;
    reg [31:0] diag_header_count_s0, diag_header_count;
    reg [31:0] diag_line_count_s0,   diag_line_count;
    reg [31:0] diag_bad_wc_count_s0, diag_bad_wc_count;
    reg [15:0] diag_last_wc_s0,      diag_last_wc;
    reg [1:0]  diag_phase_s0,        diag_phase;
    reg        diag_overflow_s0,     diag_overflow;
    reg [31:0] diag_seen_wc_s0,      diag_seen_wc;
    reg [15:0] diag_seen_hits_s0,    diag_seen_hits;
    reg [15:0] diag_lines_drop_s0,   diag_lines_drop;

    // No reset. These are two-flop copies of byte_clk-domain telemetry that are
    // overwritten on every clk edge, so a reset value would survive exactly two
    // cycles. Taking them off rst_n removes ~300 loads from the clk-domain reset
    // net, which sits on a long-wire clock network (ERRORS 6.12).
    always @(posedge clk) begin
        begin
            diag_header_count_s0 <= bridge_header_count;  diag_header_count <= diag_header_count_s0;
            diag_line_count_s0   <= bridge_line_count;    diag_line_count   <= diag_line_count_s0;
            diag_bad_wc_count_s0 <= bridge_bad_wc_count;  diag_bad_wc_count <= diag_bad_wc_count_s0;
            diag_last_wc_s0      <= bridge_last_wc;       diag_last_wc      <= diag_last_wc_s0;
            diag_phase_s0        <= {bridge_last_phase_high, bridge_last_lane_swap};
            diag_phase           <= diag_phase_s0;
            diag_overflow_s0     <= fifo_overflow_sticky; diag_overflow     <= diag_overflow_s0;
            diag_seen_wc_s0      <= {bridge_seen_wc_hits_alt, bridge_seen_wc_last};
            diag_seen_wc         <= diag_seen_wc_s0;
            diag_seen_hits_s0    <= bridge_seen_wc_hits_exp;
            diag_seen_hits       <= diag_seen_hits_s0;
            diag_lane_vld_s0     <= {lane1_vld_cnt, lane0_vld_cnt};
            diag_lane_vld        <= diag_lane_vld_s0;
            diag_lane_nz_s0      <= {lane1_nz_cnt, lane0_nz_cnt};
            diag_lane_nz         <= diag_lane_nz_s0;
            diag_lines_drop_s0   <= lines_dropped_cnt;
            diag_lines_drop      <= diag_lines_drop_s0;
        end
    end

    // FIFO admission limit: a line is only accepted into the FIFO if the whole
    // line still fits. See the packer below.
    //
    // DEPTH IS NOT THE ANSWER TO THE DROPPED LINES - do not reach for it again.
    // The FIFO was rebuilt at 16384 and 32768 words (ADDR_BITS 14 and 15) on the
    // theory that the backlog across a frame's readout just needed somewhere to
    // go. Measured over the same 12 s streaming window:
    //
    //   8192 words   36,563 lines dropped, fifo_overflow set
    //   16384 words  39,392 lines dropped, fifo_overflow set   (no better)
    //   32768 words  did not build - the Gray pointer chains grow to 16 bits in
    //                both domains and mipi_byte_clk falls to 153.093 MHz against
    //                161.290, TNS -2.058 over 26 endpoints
    //
    // Doubling the buffer changed nothing because the deficit is sustained, not a
    // transient: two lanes at 148.4 MHz deliver 296.9 MB/s while a line is active
    // and this writer manages ~144 MB/s, so a bigger buffer only postpones the
    // overflow.
    //
    // AND THE WRITER CANNOT BE MADE FAST ENOUGH EITHER - measured, ERRORS former 31.5.
    // wr_data_lane4 is 32 bits wide (the vendor's riscv_ae350_soc_tmp.v declares
    // `input [31:0] wr_data_lane4`; the DDR3_Shared ReadMe says "Data Width: 32"),
    // and clk_lane4 is nailed to DDR3_MEMORY_CLK/4 = 62.5 MHz. That caps this bus
    // at 250 MB/s with zero protocol overhead - 16 % SHORT of the 296.9 MB/s the
    // sensor puts on the wire, before a single handshake cycle is spent. No FSM
    // tuning, burst length or gap constant can close that; there is also no second
    // write lane to parallelise onto (the SoC exposes lane4 for writes, lane5 for
    // reads, and nothing else).
    //
    // Lowering the sensor's frame rate does not help either, and that was measured
    // rather than assumed: 0x0814 sets the idle time between frames, not the line
    // rate. Cutting 10.00 fps to 2.47 fps left the loss at 59.3 % against 48.8 %.
    // The sensor reads the array out in a fixed ~3.75 ms burst whatever fps says
    // (0x0810 MaxFrame_Rate = 148.08 fps confirms the whole frame fits in 6.75 ms).
    //
    // What is left is lowering the SOURCE rate: one MIPI lane instead of two
    // (0x083C = 1; 0x0034 LaneNum Cap reads 2, so the module supports it) halves
    // the wire to 148.4 MB/s. That needs the bridge to stop pairing lane0/lane1
    // bytes. Lowering the link rate directly is not available - 0x0840
    // MIPI_DataRate is unimplemented on this module and reads 0xFFFFFFFF.
    // Occupancy counters are FIFO_ADDR_BITS+1 wide; the thresholds they are
    // compared against are held at a fixed 24 bits and the counter is
    // zero-extended to meet them. Sizing the constants to the parameter instead
    // needs a different concatenation for every depth, and getting one wrong is
    // a silent truncation of an admission threshold - the one comparison in this
    // module that must never be wrong.
    localparam integer FIFO_CNT_W = FIFO_ADDR_BITS + 1;
    localparam [23:0] FIFO_ADMIT_LIMIT = (24'd1 << FIFO_ADDR_BITS) - 24'd32 - {8'd0, ROW_WORDS};

    // =========================================================================
    // 3. Pixel packing (2 x 16-bit bridge words -> one 32-bit FIFO word)
    //
    // In 8-bit D-PHY mode the bridge emits 16 bits (2 pixels) per beat, so two
    // beats are combined into one 32-bit FIFO word.
    //
    // FIFO admission is LINE-ATOMIC. Previously each word was written on its own
    // and simply dropped when the FIFO was full; a single dropped word shifts the
    // byte stream and therefore every following row of the image, permanently -
    // that is what produced the striped, row-by-row displaced frames. Here the
    // decision is taken once, at the line start: either the entire line is written
    // or none of it is. A rejected line is counted and leaves the alignment of
    // everything after it untouched.
    //
    // Each line's first word is tagged in bit 32, and bit 33 additionally marks
    // line 0 of a frame, so the DDR3 writer can find a row / frame boundary in the
    // stream instead of assuming one.
    // =========================================================================
    wire [FIFO_ADDR_BITS:0] fifo_wr_used;

    reg [15:0] mipi_word_prev;
    reg        mipi_word_phase;
    reg [FIFO_WIDTH-1:0] fifo_wr_data;
    reg        fifo_wr_en;
    reg [15:0] pk_word_cnt;      // payload words written for the current line
    reg        pk_line_active;   // current line was admitted
    reg        pk_first_word;    // next word written is this line's first
    reg [15:0] pk_line_index;

    // fifo_overflow_sticky and lines_dropped_cnt are declared with the
    // diagnostic synchroniser above, which references them.

    // Admission threshold, registered.
    //
    // Comparing fifo_wr_used against the limit inline at the line start put the
    // FIFO's pointer arithmetic straight into the clock-enable cone of the drop
    // counter - the worst mipi_byte_clk path, 0.168 ns of slack
    // (ERRORS 6.12). The flag is one cycle old when used,
    // and occupancy grows by at most one word per cycle, so the strict '<' keeps
    // exactly the guarantee the inline '<=' gave.
    reg fifo_admit_ok;

    // Packer datapath: no reset. pk_line_index is loaded at every line start
    // and read only while pk_line_active (reset) is set; mipi_word_prev is
    // loaded on beat 0 before beat 1 reads it; fifo_wr_data only matters in
    // the cycle fifo_wr_en is high, and is computed from the same terms as that
    // strobe. Keeping these off the D-PHY watchdog reset is what shrinks that
    // net's fan-out (see the bridge).
    always @(posedge mipi_byte_clk) begin
        fifo_admit_ok <= ({{(24-FIFO_CNT_W){1'b0}}, fifo_wr_used} < FIFO_ADMIT_LIMIT);
        fifo_wr_data  <= {pk_line_index[LINE_IDX_BITS-1:0],
                          pk_first_word && (pk_line_index == 16'd0),
                          pk_first_word,
                          bridge_payload, mipi_word_prev};
        if (bridge_line_start_pulse)
            pk_line_index <= bridge_line_index;
        if ((bridge_payload_dv == 2'b11) && (bridge_line_start_pulse || !mipi_word_phase))
            mipi_word_prev <= bridge_payload;
    end

    // Control: reset with the D-PHY so the half-word phase restarts cleanly
    // after every watchdog kick.
    always @(posedge mipi_byte_clk or negedge mipi_rx_rst_n) begin
        if (!mipi_rx_rst_n) begin
            mipi_word_phase      <= 1'b0;
            fifo_wr_en           <= 1'b0;
            fifo_overflow_sticky <= 1'b0;
            pk_word_cnt          <= 16'd0;
            pk_line_active       <= 1'b0;
            pk_first_word        <= 1'b0;
            lines_dropped_cnt    <= 16'd0;
        end else begin
            fifo_wr_en <= 1'b0;

            // Admission decision, taken once per line. The margin of 32 words on
            // top of a full line keeps the FIFO from ever reaching wr_full mid-line.
            if (bridge_line_start_pulse) begin
                pk_word_cnt   <= 16'd0;
                pk_first_word <= 1'b1;

                // A line refused for want of FIFO space is the defect worth
                // watching: it is counted and leaves a hole at its own address.
                if (fifo_admit_ok) begin
                    pk_line_active <= 1'b1;
                end else begin
                    pk_line_active       <= 1'b0;
                    fifo_overflow_sticky <= 1'b1;
                    if (lines_dropped_cnt != 16'hFFFF)
                        lines_dropped_cnt <= lines_dropped_cnt + 16'd1;
                end
            end

            if (bridge_payload_dv == 2'b11) begin
                // The line-start beat is always beat 0 of a word pair, whatever the
                // previous line left behind. mipi_word_prev is loaded in the
                // datapath block above on the same condition.
                if (bridge_line_start_pulse || !mipi_word_phase) begin
                    mipi_word_phase <= 1'b1;
                end else begin
                    mipi_word_phase <= 1'b0;
                    if (pk_line_active && (pk_word_cnt < ROW_WORDS)) begin
                        if (fifo_wr_full) begin
                            // Cannot happen with the admission check above; treated
                            // as a hard error rather than silently losing a word.
                            fifo_overflow_sticky <= 1'b1;
                        end else begin
                            fifo_wr_en    <= 1'b1;
                            pk_word_cnt   <= pk_word_cnt + 16'd1;
                            pk_first_word <= 1'b0;
                        end
                    end
                end
            end
        end
    end

    // =========================================================================
    // 4. Dual-Clock Asynchronous FIFO (byte_clk -> 50 MHz DDR3_CLK_IN)
    // Depth: 8192 words of FIFO_WIDTH bits
    //   [31:0]  pixel data
    //   [32]    first word of a line
    //   [33]    ... and that line is line 0 of a sensor frame
    //   [45:34] the sensor's line index for this word
    // Widening from 34 to 46 costs BSRAM but nothing in timing: it is data, not
    // pointer arithmetic, and it is the pointer chains that broke when the DEPTH
    // was raised (ERRORS former 28.2 / 28.7). Depth is unchanged here.
    // =========================================================================

    // Reset synchronisers, one per clock domain.
    //
    // rst_n is soc_rst_n from the power-on sequencer, generated in the raw CLK
    // (50 MHz input) domain, and it used to be wired straight into the pixel
    // FIFO's asynchronous reset ports. PnR reports that path at -5.4 ns of setup
    // slack - CLK_ibuf -> u_por_sequencer/soc_rst_n_s1 -> u_pixel_fifo BSRAM -
    // and its slack moves with placement. That is a far better explanation for
    // the board booting or not on nominally identical builds than ae350_ahb_clk
    // missing its constraint by 0.05 MHz, which is where the blame has been.
    //
    // Assert asynchronously, deassert synchronously. Both sides still reset
    // together on the same edge, and the FIFO is empty while they come out, so
    // the couple of cycles of deassertion skew are harmless - what changes is
    // that the long combinational path from the sequencer no longer has to make
    // setup at a memory it is not clocked with.
    // SYNCASYNCNET is waived here and nowhere else. The linter objects that these
    // are "flopped as both synchronous and async", which is precisely what a reset
    // synchroniser is: reset asynchronously by the very signal it synchronises,
    // then shifting a 1 in on the clock. Switching the check off project-wide
    // would hide a genuine mixed-reset mistake somewhere else.
    //
    // Do not begin a comment line with the linter's own name - it reads such a
    // comment as a pragma and fails with BADVLTPRAGMA.
    /* verilator lint_off SYNCASYNCNET */
    reg [1:0] rst_sync_byte;
    always @(posedge mipi_byte_clk or negedge rst_n) begin
        if (!rst_n) rst_sync_byte <= 2'b00;
        else        rst_sync_byte <= {rst_sync_byte[0], 1'b1};
    end

    reg [1:0] rst_sync_lane;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync_lane <= 2'b00;
        else        rst_sync_lane <= {rst_sync_lane[0], 1'b1};
    end
    /* verilator lint_on SYNCASYNCNET */

    wire rst_n_byte = rst_sync_byte[1];
    wire rst_n_lane = rst_sync_lane[1];

    // Frame-start anchor into the byte_clk domain: a toggle from the clk
    // domain through two flops, edge-detected. byte_clk stops while the D-PHY
    // is held in reset, so a toggle rather than a pulse; an edge that slips
    // through right after a release only re-zeros a count the reset already
    // zeroed.
    reg [2:0] anchor_sync_byte;
    always @(posedge mipi_byte_clk or negedge rst_n_byte) begin
        if (!rst_n_byte) anchor_sync_byte <= 3'b000;
        else             anchor_sync_byte <= {anchor_sync_byte[1:0], wd_anchor_tog};
    end
    assign bridge_frame_anchor = anchor_sync_byte[2] ^ anchor_sync_byte[1];

    async_fifo_simple #(
        .DATA_WIDTH(FIFO_WIDTH),
        .ADDR_BITS(FIFO_ADDR_BITS)
    ) u_pixel_fifo (
        .wr_clk    (mipi_byte_clk),
        .wr_rst_n (rst_n_byte),
        .wr_en     (fifo_wr_en),
        .wr_data   (fifo_wr_data),
        .wr_full   (fifo_wr_full),
        .wr_used   (fifo_wr_used),

        .rd_clk    (clk),
        .rd_rst_n (rst_n_lane),
        .rd_en     (fifo_rd_en),
        .rd_data   (fifo_rd_data),
        .rd_empty  (),
        .rd_used   (fifo_rd_used)
    );

    assign o_wd_reset            = wd_reset;
    assign o_diag_header_count   = diag_header_count;
    assign o_diag_line_count     = diag_line_count;
    assign o_diag_bad_wc_count   = diag_bad_wc_count;
    assign o_diag_last_wc        = diag_last_wc;
    assign o_diag_phase          = diag_phase;
    assign o_diag_overflow       = diag_overflow;
    assign o_diag_seen_wc        = diag_seen_wc;
    assign o_diag_seen_hits      = diag_seen_hits;
    assign o_diag_lines_drop     = diag_lines_drop;
    assign o_diag_byte_clk_ticks = diag_byte_clk_ticks;

endmodule
