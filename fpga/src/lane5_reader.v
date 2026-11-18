// ============================================================================
// Module      : lane5_reader
// Lane 5 read channel of one camera: polling of the CFG_ADDR mailbox (TRIG,
// SYNC, GEOM, LASR, MMTB; docs/ARCHITECTURE.md § 4.4) and the "pixel -> mm"
// band loader.
//
// Split out of fpga_camera_ddr3_streamer.v on 24.09.2026 (step 4) without any
// logic change. Only the channel with ENABLE_LANE5 = 1 (camera 0) has an
// instance; camera 1 does not have it at all rather than having it disabled by a
// parameter: the netlist hierarchy is kept (Netlist_Hierarchy).
//
// 30.10.2026 (design "Регистры управления ПЛИС на APB", revises DECISIONS R-13):
// the mailbox is not in DDR3 any more but in apb_ctl, whose Lane 5 slave
// answers these bursts the way l5_axi_rd did; the row read-back check is gone
// (ERRORS 1.7, Kelvin 30.10), and the mm band is not fetched from DDR3 but
// written by the CPU word by word into the APB band register, which apb_ctl
// passes here as i_bw_v / i_bw_d (see the loader below).
// ============================================================================

`timescale 1ns / 1ps

module lane5_reader #(
    parameter [31:0]  CFG_ADDR     = 32'h0600_0100,
    parameter integer BURST_LOG2   = 6,
    parameter integer ENABLE_LANE5 = 1
)(
    input  wire        clk,
    input  wire        rst_n,

    output reg         rd_go,
    output wire [7:0]  rd_burstcount,
    output wire [31:0] rd_addr,
    output reg         rd_en,
    input  wire        rd_rdy,
    input  wire        rd_valid,
    input  wire [63:0] rd_data,

    // Lane 5 health during mailbox polling (header words 31..33).
    output wire [31:0] o_l5_counts,         // {go_to, rd_to, rdy_cnt, last_words}
    output wire [31:0] o_l5_word0,
    output wire [31:0] o_l5_word1,

    // Mailbox: TRIG (words 0..5), SYNC (6..20), GEOM (29..31).
    output wire        o_cfg_magic_ok,
    output wire [31:0] o_cfg_period,
    output wire [7:0]  o_cfg_count,
    output wire [31:0] o_cfg_width,
    output wire [31:0] o_cfg_seq,
    output wire [31:0] o_cfg_flags,
    output reg  [14*32-1:0] sync_mbx,
    output reg              sync_loaded,
    output wire [31:0] o_geom_size,
    output wire [31:0] o_geom_seq,

    // Laser line finder: LASR words of camera 0 (32..40) and camera 1
    // (41..49). Camera 1 is fed through a second bus by the camera 0 channel - it has
    // no Lane 5 of its own.
    output wire [95:0] o_lasr0_cfg,
    output wire [95:0] o_lasr1_cfg,

    // "pixel -> mm" table (MMTB block, words 50..61; docs/DECISIONS.md R-38): the band
    // of each camera comes from the CPU (i_bw_*) and is written here into its laser_mm.v.
    //   o_mmN_cfg  - laser_prof's mm_cfg bus: [15:0] vofs, [27:16] kofs, [32:28] node
    //                rows, [33] band valid, [34] load request, [95:64] table id,
    //                [127:96] band CRC (expected = loaded when valid);
    //   o_mmN_stat - telemetry: [31:0] CRC of the last load, [39:32] APPLY_SEQ low
    //                byte of the last job, [40] CRC mismatch, [41] load error
    //                (band timeout, no acknowledge, bad rows);
    //   i_mm_ack   - laser_prof's mm_load_ack per camera (camera 1 by wire).
    output reg  [1:0]   o_tw_en,
    output reg  [1:0]   o_tw_bank,
    output reg  [8:0]   o_tw_addr,
    output reg  [31:0]  o_tw_data,
    output wire [127:0] o_mm0_cfg,
    output wire [127:0] o_mm1_cfg,
    output wire [63:0]  o_mm0_stat,
    output wire [63:0]  o_mm1_stat,
    input  wire [1:0]   i_mm_ack,
    // band words from the CPU (apb_ctl, one clock each) and the loader state it polls:
    //   [31:29] state (2 = waiting for band words), [28] camera, [23:16] words
    //   dropped (came while no band was wanted, saturating), [10:0] words still wanted
    input  wire         i_bw_v,
    input  wire [31:0]  i_bw_d,
    output wire [31:0]  o_ld_stat
);

    localparam [7:0] BURST_WORDS = 8'd1 << BURST_LOG2;

    reg [31:0] cfg_period;             // trigger period in clk cycles
    reg [7:0]  cfg_count;              // 1..64 pulses; 0 - continuous mode
    reg [31:0] cfg_width;              // pulse width in clk cycles
    reg [31:0] cfg_seq;                // command counter: changed - start
    reg [31:0] cfg_flags;              // bit 0 - inversion, bit 1 - stop
    reg        cfg_magic_ok;           // the magic was found in the block read
    reg        sync_magic_ok;          // ... and the "SYNC" magic in word 6
    reg        geom_magic_ok;          // ... and the "GEOM" magic in word 29
    reg [31:0] geom_size;              // {height[31:16], width[15:0]}, 0 - do not change
    reg [31:0] geom_seq;               // geometry apply counter

    reg [7:0]  l5_go_to, l5_rd_to, l5_rdy_cnt, l5_last_words;
    reg [31:0] l5_word0, l5_word1;

    // =========================================================================
    // Mailbox poll on Lane 5
    // =========================================================================
    localparam V_IDLE = 2'd0;
    localparam V_GO   = 2'd1;
    localparam V_READ = 2'd2;

    reg [1:0]  vstate;
    reg [BURST_LOG2-1:0] v_word;
    // "The next read burst is mailbox word CW_S_FIRST + k", one-hot, kept in step
    // with v_word (25.09.2026). The synchroniser word number used to be decoded from
    // v_word in the same cycle the bridge's rd_valid arrives, and the path
    // rd_valid -> 3 LUT -> CE of 448 sync_mbx flops was the worst on lane4_clk
    // (-0.166 ns in build E2b; 4 levels, two of them spans of 2.5-2.7 ns).
    // v_word only clears or increments by 1, so the shift is exact: bit 0 rises
    // when the next value = CW_S_FIRST, and moves past bit 13 after
    // CW_S_LAST. Every assignment to v_word must update it too.
    reg [13:0] sync_we_q;
    integer    sk;
    reg [11:0] v_timeout;

    reg [19:0] cfg_poll;               // poll timer
    reg        cfg_req;                // time to poll
    // Once every 2^20 cycles, i.e. roughly every 10 ms at 100 MHz. Fast enough for
    // a UART command to take effect unnoticeably for a human.
    localparam [31:0] CFG_MAGIC = 32'h5452_4947;   // "TRIG"

    // Control block layout. The magic first, and the command counter LAST:
    // its change is what starts a series, so by the time the generator sees it,
    // period, count and width are already written. Put it before the flags
    // and a series could start with the old polarity.
    localparam [BURST_LOG2-1:0] CW_MAGIC  = 0;
    localparam [BURST_LOG2-1:0] CW_PERIOD = 1;
    localparam [BURST_LOG2-1:0] CW_COUNT  = 2;
    localparam [BURST_LOG2-1:0] CW_WIDTH  = 3;
    localparam [BURST_LOG2-1:0] CW_FLAGS  = 4;
    localparam [BURST_LOG2-1:0] CW_SEQ    = 5;
    // The sync block in the same burst: word 6 - magic, 7..20 - sync_mbx
    // (CTRL, delays/widths of cam0, cam1, laser, laser mode, pre-pulse,
    // polarity, EXT filter, APPLY_SEQ, SW_SEQ). The counters are last for the same
    // reason as CW_SEQ: by the time they change the parameters are already in memory.
    // 32 bits and a compare with the extended word number: with a burst of 16
    // words (BURST_LOG2 = 4, the module default and tb_streamer) 20 does not fit into
    // v_word and would truncate to 4 - exactly CW_FLAGS. There words 7..20 simply do
    // not exist.
    localparam [31:0] CW_S_MAGIC = 32'd6;
    localparam [31:0] CW_S_FIRST = 32'd7;
    localparam [31:0] CW_S_LAST  = 32'd20;
    localparam [31:0] SYNC_MAGIC = 32'h5359_4E43;   // "SYNC"

    // Receive geometry: magic, size, apply counter. The TOP of the free range
    // 21..31 is taken, so that the sync block can grow from below and the LASR block
    // from word 32 stays untouched (docs/ARCHITECTURE.md § 4.4).
    localparam [31:0] CW_G_MAGIC = 32'd29;
    localparam [31:0] CW_G_SIZE  = 32'd30;          // {height[31:16], width[15:0]}
    localparam [31:0] CW_G_SEQ   = 32'd31;
    localparam [31:0] GEOM_MAGIC = 32'h4745_4F4D;   // "GEOM"

    // Presented only while rd_go is high, zero otherwise - the same rule the
    // write side follows, and for the same reason: holding them between
    // transactions is the kind of protocol difference that misplaces an access.
    assign rd_addr       = rd_go ? CFG_ADDR : 32'd0;
    assign rd_burstcount = rd_go ? BURST_WORDS : 8'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vstate         <= V_IDLE;
            rd_go          <= 1'b0;
            rd_en          <= 1'b0;
            v_word         <= {BURST_LOG2{1'b0}};
            sync_we_q      <= 14'd0;
            v_timeout      <= 12'd0;
            l5_go_to       <= 8'd0;
            l5_rd_to       <= 8'd0;
            l5_rdy_cnt     <= 8'd0;
            l5_last_words  <= 8'd0;
            l5_word0       <= 32'd0;
            l5_word1       <= 32'd0;
            cfg_poll       <= 20'd0;
            cfg_req        <= 1'b0;
            cfg_magic_ok   <= 1'b0;
            cfg_period     <= 32'd0;
            cfg_count      <= 8'd0;
            cfg_width      <= 32'd0;
            cfg_seq        <= 32'd0;
            cfg_flags      <= 32'd0;
            sync_magic_ok  <= 1'b0;
            sync_mbx       <= {14*32{1'b0}};
            sync_loaded    <= 1'b0;
            // Geometry is reset too (25.09.2026): a register that is assigned
            // in a block with reset but not reset itself is held by synthesis
            // through CE, and the reset net enters CE as data - the worst
            // lane4_clk path of build 5F8B5D11 (lane4_rst_sync -> geom_seq/CE,
            // 0.158 ns). Zero is the same as after FPGA configuration.
            geom_magic_ok  <= 1'b0;
            geom_size      <= 32'd0;
            geom_seq       <= 32'd0;
        end else begin
            rd_go          <= 1'b0;

            // Control block poll timer.
            cfg_poll <= cfg_poll + 20'd1;
            // With ENABLE_LANE5 = 0 the request is never raised: the read FSM
            // stays in V_IDLE, rd_go low.
            if (&cfg_poll && (ENABLE_LANE5 != 0)) cfg_req <= 1'b1;

            case (vstate)
                V_IDLE: begin
                    rd_en <= 1'b0;
                    if (cfg_req) begin
                        cfg_req      <= 1'b0;
                        cfg_magic_ok <= 1'b0;
                        sync_magic_ok <= 1'b0;
                        v_word       <= {BURST_LOG2{1'b0}};
                        sync_we_q    <= 14'd0;
                        v_timeout    <= 12'd0;
                        rd_go        <= 1'b1;      // ONE cycle, unlike wr_go
                        vstate       <= V_GO;
                    end
                end

                // Wait for the slave to accept the request. Bounded: parking here
                // forever would silently stop the poll and leave the counters
                // looking healthy.
                V_GO: begin
                    v_timeout <= v_timeout + 12'd1;
                    if (rd_rdy) begin
                        rd_en      <= 1'b1;
                        v_word     <= {BURST_LOG2{1'b0}};
                        sync_we_q  <= 14'd0;
                        v_timeout  <= 12'd0;
                        vstate     <= V_READ;
                        l5_rdy_cnt <= l5_rdy_cnt + 8'd1;
                    end else if (v_timeout[11]) begin
                        l5_go_to <= l5_go_to + 8'd1;
                        vstate   <= V_IDLE;
                    end
                end

                V_READ: begin
                    v_timeout <= v_timeout + 12'd1;
                    if (rd_valid) begin
                        // Only the low 32 bits carry payload.
                        if (v_word == {BURST_LOG2{1'b0}}) l5_word0 <= rd_data[31:0];
                        if (v_word == {{(BURST_LOG2-1){1'b0}}, 1'b1}) l5_word1 <= rd_data[31:0];
                        if (v_word == (BURST_WORDS[BURST_LOG2-1:0] - 1'b1))
                            l5_last_words <= l5_last_words + 8'd1;
                        // The magic is checked first, and without it no field
                        // is accepted. Otherwise an unwritten block would become
                        // trigger parameters right after power-up.
                        if (v_word == CW_MAGIC)
                            cfg_magic_ok <= (rd_data[31:0] == CFG_MAGIC);
                        if (cfg_magic_ok) begin
                            if (v_word == CW_PERIOD) cfg_period <= rd_data[31:0];
                            if (v_word == CW_COUNT)  cfg_count  <= rd_data[7:0];
                            if (v_word == CW_WIDTH)  cfg_width  <= rd_data[31:0];
                            if (v_word == CW_FLAGS)  cfg_flags  <= rd_data[31:0];
                            if (v_word == CW_SEQ)    cfg_seq    <= rd_data[31:0];
                        end
                        // The sync block has its own magic: firmware without
                        // it (or that has not written it yet) leaves words 6+
                        // as they were, and the FPGA does not accept them.
                        if ({{(32-BURST_LOG2){1'b0}}, v_word} == CW_S_MAGIC)
                            sync_magic_ok <= (rd_data[31:0] == SYNC_MAGIC);
                        // Words CW_S_FIRST..CW_S_LAST - by the pre-decoded
                        // sync_we_q (see the declaration):
                        // sync_we_q[k] = (v_word == CW_S_FIRST + k).
                        for (sk = 0; sk < 14; sk = sk + 1)
                            if (sync_magic_ok && sync_we_q[sk])
                                sync_mbx[sk*32 +: 32] <= rd_data[31:0];
                        if (sync_magic_ok && sync_we_q[13])      // = (v_word == CW_S_LAST)
                            sync_loaded <= 1'b1;
                        // Receive geometry. Its own magic for the same reason
                        // as the sync block.
                        if ({{(32-BURST_LOG2){1'b0}}, v_word} == CW_G_MAGIC)
                            geom_magic_ok <= (rd_data[31:0] == GEOM_MAGIC);
                        if (geom_magic_ok) begin
                            if ({{(32-BURST_LOG2){1'b0}}, v_word} == CW_G_SIZE)
                                geom_size <= rd_data[31:0];
                            if ({{(32-BURST_LOG2){1'b0}}, v_word} == CW_G_SEQ)
                                geom_seq <= rd_data[31:0];
                        end

                        if (v_word == (BURST_WORDS[BURST_LOG2-1:0] - 1'b1)) begin
                            rd_en  <= 1'b0;
                            vstate <= V_IDLE;
                        end else begin
                            v_word <= v_word + 1'b1;
                            sync_we_q <= {sync_we_q[12:0],
                                          (({{(32-BURST_LOG2){1'b0}}, v_word} + 32'd1) == CW_S_FIRST)};
                        end
                    end else if (v_timeout[11]) begin
                        rd_en    <= 1'b0;
                        l5_rd_to <= l5_rd_to + 8'd1;
                        vstate   <= V_IDLE;
                    end
                end

                default: vstate <= V_IDLE;
            endcase
        end
    end

    assign o_l5_counts         = {l5_go_to, l5_rd_to, l5_rdy_cnt, l5_last_words};
    assign o_l5_word0          = l5_word0;
    assign o_l5_word1          = l5_word1;
    assign o_cfg_magic_ok      = cfg_magic_ok;
    assign o_cfg_period        = cfg_period;
    assign o_cfg_count         = cfg_count;
    assign o_cfg_width         = cfg_width;
    assign o_cfg_seq           = cfg_seq;
    assign o_cfg_flags         = cfg_flags;
    assign o_geom_size         = geom_size;
    assign o_geom_seq          = geom_seq;

    // ======================================================================
    // Laser line finder: LASR blocks (stage L2a)
    // ======================================================================
    //
    // Each camera has its own block of nine words (firmware/src/app/laser.h):
    //   +0 magic "LASR"  +1 CTRL  +2 {offset[15:8], threshold[7:0]}  +3 gap
    //   +4 min_sum  +5 window from  +6 window to  +7 width_max  +8 APPLY_SEQ
    // Since 30.09.2026 the FPGA reads min_sum and width_max too: the point (centre of
    // gravity and quality flags) is computed by laser_prof.v, block version 2. Camera 0
    // from word 32, camera 1 from word 41.
    //
    // The words are read into shadow registers and applied at once on the APPLY_SEQ
    // word, the last in the block: the finder takes the parameters at frame start,
    // and a frame start in the middle of a poll must not catch half of the new set.
    // Without the magic - build defaults: threshold 40, gap 2, offset 0, window - the
    // whole axis, the block is written. Window "to" = 0 means "the whole axis", as in
    // tools/laser/model.py extract_raw (0/0 - the whole axis).
    // Finder parameter bus (one camera), 96 bits:
    //   [7:0] threshold  [15:8] offset  [19:16] gap  [30:20] window from
    //   [41:31] window to (inclusive)  [42] write the block  [43] mailbox read
    //   [51:44] APPLY_SEQ, low byte (echoed in telemetry)
    //   [52] do not write the frame, profile block only (CTRL[10] NO_FRAME with
    //   WRITE_PROF and not SOFT; stage 2, 01.10.2026 - the profile stream)
    //   [79:64] min_sum  [95:80] width_max  (since 30.09.2026; defaults 64/64 -
    //   the same as LASER_CFG_DEFAULT in laser.h)
    // Without the magic and with CTRL[10] = 0 the frame is written: older firmware
    // never sets this bit, and the behaviour with it does not change.
    localparam [31:0] LASR_MAGIC   = 32'h4C41_5352;   // "LASR"
    localparam [95:0] LASR_DEFAULT = {16'd64, 16'd64,
                                      11'd0, 1'b0, 8'd0, 1'b0, 1'b1, 11'h7FF, 11'd0,
                                      4'd2, 8'd0, 8'd40};
    wire [31:0] l_word = {{(32-BURST_LOG2){1'b0}}, v_word};
    wire        l_beat = (vstate == V_READ) && rd_valid;
    wire [95:0] lasr_cfg_w [0:1];

    genvar gl;
    generate
        for (gl = 0; gl < 2; gl = gl + 1) begin : g_lasr
            localparam [31:0] LB = 32'd32 + 32'd9 * gl;
            reg        magic_ok;
            reg [95:0] cfg;
            reg [15:0] s_ctrl, s_thr, s_msum, s_wmax;
            reg [3:0]  s_gap;
            reg [10:0] s_lo, s_hi;
            // Applied one cycle later, from the apply_q register (09.10.2026).
            // The enable of the 96 cfg flops was decoded from rd_valid and v_word in
            // the cycle the word arrived: rd_valid -> 2 LUT -> CE, and placement puts
            // camera 1's cfg registers next to it, across the die - in the stage 5
            // build this path gave 0.009 ns on clk_x1 (own-logic threshold
            // 0.3). The same trick as sync_we_q above: a single flop in the CE.
            reg        apply_q;
            reg [7:0]  apply_seq_q;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    magic_ok    <= 1'b0;
                    apply_q     <= 1'b0;
                    apply_seq_q <= 8'd0;
                    cfg         <= LASR_DEFAULT;
                end else begin
                    apply_q     <= l_beat && magic_ok && (l_word == LB + 32'd8);
                    // No enable: in the apply_q cycle the word here is APPLY_SEQ.
                    apply_seq_q <= rd_data[7:0];
                    if (l_beat && (l_word == LB))
                        magic_ok <= (rd_data[31:0] == LASR_MAGIC);
                    if (apply_q)
                        cfg <= {s_wmax, s_msum,
                                11'd0,
                                s_ctrl[10] && s_ctrl[3] && !s_ctrl[9], // NO_FRAME: only with the FPGA block
                                apply_seq_q, 1'b1,
                                s_ctrl[3] && !s_ctrl[9],        // WRITE_PROF and not SOFT
                                (s_hi == 11'd0) ? 11'h7FF : s_hi, s_lo,
                                s_gap, s_thr[15:8], s_thr[7:0]};
                end
            end
            // Shadows without reset: applied only after the magic of the same poll.
            always @(posedge clk) begin
                if (l_beat && magic_ok) begin
                    if (l_word == LB + 32'd1) s_ctrl <= rd_data[15:0];
                    if (l_word == LB + 32'd2) s_thr  <= rd_data[15:0];
                    if (l_word == LB + 32'd3) s_gap  <= rd_data[3:0];
                    if (l_word == LB + 32'd4) s_msum <= rd_data[15:0];
                    if (l_word == LB + 32'd5) s_lo   <= rd_data[10:0];
                    if (l_word == LB + 32'd6) s_hi   <= rd_data[10:0];
                    if (l_word == LB + 32'd7) s_wmax <= rd_data[15:0];
                end
            end
            assign lasr_cfg_w[gl] = cfg;
        end
    endgenerate

    assign o_lasr0_cfg = lasr_cfg_w[0];
    assign o_lasr1_cfg = lasr_cfg_w[1];

    // ======================================================================
    // "pixel -> mm" table loader (16.10.2026, docs/DECISIONS.md R-38)
    // ======================================================================
    //
    // MMTB block of the mailbox (firmware/src/app/mm_table.c):
    //   50 magic "MMTB"
    //   51 + 5c: unused since 30.10.2026 (was the band's DDR3 address)
    //   52 + 5c: CRC-32 of the band (zlib, words little-endian)
    //   53 + 5c: table id (echoed in every profile block)
    //   54 + 5c: {kofs [27:16], vofs [15:0]} - ROI offsets: vofs signed, 1/16 px
    //   55 + 5c: [0] enable, [12:8] node rows in the band (2..28)
    //   61 APPLY_SEQ - a change (or the first block after reset) starts a job
    //
    // A job, camera by camera: raise the request to its laser_prof (the band is
    // invalid from now on), wait for the acknowledge (it comes between unloads),
    // take rows * NODE_COLS words from the CPU (i_bw_*: the CPU sees state 2 and
    // the camera in o_ld_stat, then writes that many words into the APB band
    // register), write them into the four banks by node row and column parity, CRC
    // them on the way, and set the band valid only if the CRC matches the mailbox;
    // then drop the request. 2^20 clocks (~10 ms) without a band word, a missing
    // acknowledge or bad rows leave the band invalid and set the error bit - the
    // firmware sees it in telemetry word 38 and the profile blocks carry "no mm".
    // Words that come while no band is wanted are dropped and counted.
    //
    // Every decision is taken from registers: the beat and its data are registered
    // first (tb_v, tb_d), the shadows are copied on the cycle after the APPLY_SEQ
    // word, as for LASR.
    localparam integer NODE_COLS   = 69;
    localparam integer BANK_STRIDE = 35;
    localparam [31:0]  MM_MAGIC    = 32'h4D4D_5442;   // "MMTB"
    localparam [31:0]  MW_MAGIC    = 32'd50;
    localparam [31:0]  MW_SEQ      = 32'd61;

    reg        mm_magic_ok, mm_apply_q;
    reg [31:0] mm_seq_q;
    // Two cameras side by side in one vector, camera c at [W*c +: W]: as two-entry
    // arrays with a variable index they were inferred as block RAM (BSRAM 318 instead
    // of 310 in the first build, 16.10.2026).
    reg [63:0] s_mcrc, s_mid, s_moffs;
    reg [25:0] s_mctrl;
    integer    mc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mm_magic_ok <= 1'b0;
            mm_apply_q  <= 1'b0;
        end else begin
            mm_apply_q <= l_beat && mm_magic_ok && (l_word == MW_SEQ);
            if (l_beat && (l_word == MW_MAGIC))
                mm_magic_ok <= (rd_data[31:0] == MM_MAGIC);
        end
    end
    // Shadows without reset: copied into a job only after the magic of the same poll.
    always @(posedge clk) begin
        mm_seq_q <= rd_data[31:0];       // in the mm_apply_q cycle: the APPLY_SEQ word
        for (mc = 0; mc < 2; mc = mc + 1)
            if (l_beat && mm_magic_ok) begin
                if (l_word == 32'd52 + 5 * mc) s_mcrc[32*mc +: 32]  <= rd_data[31:0];
                if (l_word == 32'd53 + 5 * mc) s_mid[32*mc +: 32]   <= rd_data[31:0];
                if (l_word == 32'd54 + 5 * mc) s_moffs[32*mc +: 32] <= rd_data[31:0];
                if (l_word == 32'd55 + 5 * mc) s_mctrl[13*mc +: 13] <= rd_data[12:0];
            end
    end

    // CRC-32, reflected (zlib), one 32-bit word, bits LSB first = bytes little-endian.
    function [31:0] crc32_word;
        input [31:0] c;
        input [31:0] d;
        integer b;
        reg [31:0] x;
        begin
            x = c;
            for (b = 0; b < 32; b = b + 1)
                x = (x >> 1) ^ ((x[0] ^ d[b]) ? 32'hEDB8_8320 : 32'd0);
            crc32_word = x;
        end
    endfunction

    localparam [2:0] T_IDLE = 3'd0, T_REQ = 3'd1, T_FETCH = 3'd2, T_CHECK = 3'd3,
                     T_REL = 3'd4, T_FAIL = 3'd5;
    reg [2:0]  tstate;
    reg        j_go, t_once;
    reg [31:0] j_seq, t_seq_done;
    reg [63:0] j_crc, j_id, j_offs;             // camera c at [32*c +: 32]
    reg [25:0] j_ctrl;                          // camera c at [13*c +: 13]
    reg        t_cam;
    reg [1:0]  ack_q;                  // laser_prof's acknowledge, registered
    reg [19:0] t_wait;                 // acknowledge timeout
    reg [31:0] t_crc_exp, t_crc;
    reg [10:0] t_total, t_n;           // words of the band, words written
    reg [7:0]  t_drop;                 // band words that came while none was wanted
    reg [4:0]  t_r;
    reg [6:0]  t_c;
    reg [8:0]  t_rowbase;
    reg        tb_v;
    reg [31:0] tb_d;
    reg [1:0]  m_valid, m_req, m_fail, m_err;
    // Per camera, the same packing: c at [W*c +: W].
    reg [31:0] m_vofs;
    reg [23:0] m_kofs;
    reg [9:0]  m_rows;
    reg [63:0] m_id, m_crc, m_crc_got;
    reg [15:0] m_seq;

    wire [12:0] j_ctrl_c   = j_ctrl[13*t_cam +: 13];
    wire [31:0] j_offs_c   = j_offs[32*t_cam +: 32];
    wire [4:0]  j_rows     = j_ctrl_c[12:8];
    localparam [6:0] NC_LAST = NODE_COLS[6:0] - 7'd1;
    wire       t_beat = i_bw_v && (tstate == T_FETCH) && (t_n != t_total);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tstate     <= T_IDLE;
            j_go       <= 1'b0;
            t_once     <= 1'b0;
            t_seq_done <= 32'd0;
            t_cam      <= 1'b0;
            ack_q      <= 2'b00;
            t_wait     <= 20'd0;
            t_drop     <= 8'd0;
            t_n        <= 11'd0;
            t_total    <= 11'd0;
            tb_v       <= 1'b0;
            o_tw_en    <= 2'b00;
            m_valid    <= 2'b00;
            m_req      <= 2'b00;
            m_fail     <= 2'b00;
            m_err      <= 2'b00;
        end else begin
            ack_q   <= i_mm_ack;
            tb_v    <= t_beat;
            o_tw_en <= 2'b00;
            if (i_bw_v && !t_beat && t_drop != 8'hFF) t_drop <= t_drop + 8'd1;

            // A new job: copy the shadows on the cycle after APPLY_SEQ.
            if (mm_apply_q && (!t_once || (mm_seq_q != t_seq_done))) begin
                j_go   <= 1'b1;
                j_seq  <= mm_seq_q;
                j_crc  <= s_mcrc;
                j_id   <= s_mid;
                j_offs <= s_moffs;
                j_ctrl <= s_mctrl;
            end

            case (tstate)
                T_IDLE: begin
                    if (j_go && !mm_apply_q) begin
                        j_go       <= 1'b0;
                        t_once     <= 1'b1;
                        t_seq_done <= j_seq;
                        t_cam      <= 1'b0;
                        tstate     <= T_REQ;
                    end
                end

                // Request: the band is invalid from this cycle; its parameters may
                // change, laser_prof does not take them while the request is up.
                T_REQ: begin
                    m_req[t_cam]   <= 1'b1;
                    m_valid[t_cam] <= 1'b0;
                    m_fail[t_cam]  <= 1'b0;
                    m_err[t_cam]   <= 1'b0;
                    m_seq[8*t_cam +: 8]   <= j_seq[7:0];
                    m_vofs[16*t_cam +: 16] <= j_offs_c[15:0];
                    m_kofs[12*t_cam +: 12] <= j_offs_c[27:16];
                    m_rows[5*t_cam +: 5]  <= j_rows;
                    m_id[32*t_cam +: 32]  <= j_id[32*t_cam +: 32];
                    m_crc[32*t_cam +: 32] <= j_crc[32*t_cam +: 32];
                    t_crc_exp      <= j_crc[32*t_cam +: 32];
                    t_crc          <= 32'hFFFF_FFFF;
                    // rows * 69 = rows * 64 + rows * 4 + rows
                    t_total        <= {j_rows, 6'd0} + {4'd0, j_rows, 2'd0} + {6'd0, j_rows};
                    t_n            <= 11'd0;
                    t_r            <= 5'd0;
                    t_c            <= 7'd0;
                    t_rowbase      <= 9'd0;
                    t_wait         <= t_wait + 20'd1;
                    // The acknowledge is trusted only 8 cycles after the request rose:
                    // one left over from the previous job of this camera needs about
                    // three cycles to fall (three registers round trip).
                    if (!j_ctrl_c[0]) begin
                        // Disabled: invalid from this cycle, nothing to write - no
                        // acknowledge needed.
                        t_wait <= 20'd0;
                        tstate <= T_REL;
                    end else if (ack_q[t_cam] && m_req[t_cam] && t_wait[3]) begin
                        t_wait <= 20'd0;
                        if (j_rows < 5'd2 || j_rows > 5'd28)
                            tstate <= T_FAIL;
                        else
                            tstate <= T_FETCH;
                    end else if (&t_wait) begin
                        t_wait <= 20'd0;
                        tstate <= T_FAIL;
                    end
                end

                T_FETCH: begin
                    t_wait <= t_beat ? 20'd0 : t_wait + 20'd1;
                    // Registered beat: write and CRC the first t_total words.
                    if (tb_v && (t_n != t_total)) begin
                        o_tw_en[t_cam] <= 1'b1;
                        o_tw_bank      <= {t_r[0], t_c[0]};
                        o_tw_addr      <= t_rowbase + {3'd0, t_c[6:1]};
                        o_tw_data      <= tb_d;
                        t_crc          <= crc32_word(t_crc, tb_d);
                        t_n            <= t_n + 11'd1;
                        if (t_c == NC_LAST) begin
                            t_c <= 7'd0;
                            t_r <= t_r + 5'd1;
                            if (t_r[0]) t_rowbase <= t_rowbase + BANK_STRIDE[8:0];
                        end else begin
                            t_c <= t_c + 7'd1;
                        end
                    end
                    if (t_n == t_total && !tb_v) begin
                        t_wait <= 20'd0;
                        tstate <= T_CHECK;
                    end else if (&t_wait) begin
                        t_wait <= 20'd0;               // the CPU gave the band up
                        tstate <= T_FAIL;
                    end
                end

                T_CHECK: begin
                    m_crc_got[32*t_cam +: 32] <= ~t_crc;
                    m_valid[t_cam]   <= (~t_crc == t_crc_exp);
                    m_fail[t_cam]    <= (~t_crc != t_crc_exp);
                    tstate           <= T_REL;
                end

                T_FAIL: begin
                    m_err[t_cam]   <= 1'b1;
                    m_valid[t_cam] <= 1'b0;
                    tstate         <= T_REL;
                end

                // Request down one cycle after the band state: laser_prof sees the
                // new state no later than the end of the request.
                T_REL: begin
                    m_req[t_cam] <= 1'b0;
                    if (t_cam == 1'b0) begin
                        t_cam  <= 1'b1;
                        tstate <= T_REQ;
                    end else begin
                        tstate <= T_IDLE;
                    end
                end

                default: tstate <= T_IDLE;
            endcase
        end
    end
    always @(posedge clk) tb_d <= i_bw_d;
    assign o_ld_stat = {tstate, t_cam, 4'd0, t_drop, 5'd0, t_total - t_n};

    genvar gm;
    generate
        for (gm = 0; gm < 2; gm = gm + 1) begin : g_mm
            wire [127:0] cfg = {m_crc[32*gm +: 32], m_id[32*gm +: 32], 29'd0, m_req[gm], m_valid[gm],
                                m_rows[5*gm +: 5], m_kofs[12*gm +: 12], m_vofs[16*gm +: 16]};
            wire [63:0]  st  = {22'd0, m_err[gm], m_fail[gm], m_seq[8*gm +: 8], m_crc_got[32*gm +: 32]};
            if (gm == 0) begin : g0
                assign o_mm0_cfg  = cfg;
                assign o_mm0_stat = st;
            end else begin : g1
                assign o_mm1_cfg  = cfg;
                assign o_mm1_stat = st;
            end
        end
    endgenerate

endmodule
