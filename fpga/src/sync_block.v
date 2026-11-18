// ============================================================================
// sync_block - camera and laser sync block (v3, stage 2 of the sync plan;
// description and mailbox words: docs/ARCHITECTURE.md § 5, decision R-13)
// ============================================================================
//
// A merge of sinc/sync_block.v (v1) and sinc/sync_block (1).v (v2) with these
// changes:
//   * lane4_clk at 100 MHz, 1 cycle = 10 ns (as in the STP), not 128 MHz;
//   * delay and width of each channel are 32 bits in cycles, not 4 codes;
//   * no magnitude compares on every cycle: an output is set and cleared on
//     equality of the counter with bounds computed in advance (docs/WORKFLOWS.md
//     § 3 on the timing margin);
//   * enable is the block enable (v1), the source is a separate field;
//   * separate synchronisation and a filter of the external input with a
//     configurable window;
//   * v2 laser modes with an unambiguous encoding; "always 1" and any laser pulse
//     only with laser_enable;
//   * overload (v1) + counters of accepted and lost triggers, EXT edges, and a
//     timestamp of the last accepted trigger.
//
// Sources: CORE - a SW_TRIGGER write (a toggle bit from the AHB domain),
// TIMER - the streamer's series generator (TRIG mailbox), EXT - an FPGA pin
// (not routed yet). For the camera any of them is an external trigger.
//
// Reset values repeat the previous behaviour: TIMER source, cameras
// 1 ms with no delay, laser off. TRIG series work as before the block.
// ============================================================================

`timescale 1ns/1ps

module sync_block #(
    parameter [31:0] DEF_CAM_WIDTH   = 32'd100000,   // 1 ms
    parameter [31:0] DEF_LASER_WIDTH = 32'd10,       // 100 ns
    parameter [31:0] DEF_EXT_FILTER  = 32'd100       // 1 us
) (
    input  wire        clk,            // lane4_clk
    input  wire        rst_n,

    // From the AHB domain. Toggle bits are synchronised here; the cfg_* fields are
    // quasi-static - the firmware writes them BEFORE toggling apply, and by the time
    // the toggle has passed the synchroniser they have long settled.
    input  wire        cfg_apply_tog,
    input  wire        sw_tog,
    input  wire        cfg_enable,
    input  wire [1:0]  cfg_source,     // 0 CORE, 1 TIMER, 2 EXT
    input  wire        cfg_ext_fall,   // 0 - EXT rising edge, 1 - falling
    input  wire [31:0] cfg_cam0_delay,
    input  wire [31:0] cfg_cam0_width,
    input  wire [31:0] cfg_cam1_delay,
    input  wire [31:0] cfg_cam1_width,
    input  wire [31:0] cfg_laser_delay,
    input  wire [31:0] cfg_laser_width,
    input  wire [1:0]  cfg_laser_mode, // 0 pulse, 1 always 0, 2 always 1, 3 pre-pulse
    input  wire        cfg_laser_enable,
    input  wire [31:0] cfg_pre_lead,   // pre-pulse: starts this many cycles before the main one
    input  wire [31:0] cfg_pre_width,
    input  wire [2:0]  cfg_polarity,   // [0] cam0, [1] cam1, [2] laser: 1 - active low
    input  wire [31:0] cfg_ext_filter, // the EXT input must stay stable this many cycles + 1

    input  wire        timer_in,       // series generator level, clk domain
    input  wire        ext_in,         // asynchronous pin
    input  wire        strobe_in,      // asynchronous Strobe Out of camera 0

    output reg         cam0_trig,
    output reg         cam1_trig,
    output reg         laser_gate,
    output reg         laser_enable,

    output reg         enabled,
    output reg         active,
    output reg         exposure_done,  // strobe fall after the last accepted trigger
    output reg         overrun_flag,   // a trigger arrived during a pulse
    output reg [31:0]  cnt_accepted,
    output reg [31:0]  cnt_overrun,
    output reg [31:0]  cnt_ext_edges,
    output reg [31:0]  last_trig_ts    // block cycle counter at the moment of acceptance
);

    localparam [1:0] SRC_CORE    = 2'd0;
    localparam [1:0] SRC_TIMER   = 2'd1;
    localparam [1:0] SRC_EXT     = 2'd2;
    localparam [1:0] LM_PULSE    = 2'd0;
    localparam [1:0] LM_FORCE0   = 2'd1;
    localparam [1:0] LM_FORCE1   = 2'd2;
    localparam [1:0] LM_PREPULSE = 2'd3;

    // ---- synchronisers ------------------------------------------------------
    reg [2:0] apply_s, sw_s, ext_s, strobe_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apply_s  <= 3'd0;
            sw_s     <= 3'd0;
            ext_s    <= 3'd0;
            strobe_s <= 3'd0;
        end else begin
            apply_s  <= {apply_s[1:0],  cfg_apply_tog};
            sw_s     <= {sw_s[1:0],     sw_tog};
            ext_s    <= {ext_s[1:0],    ext_in};
            strobe_s <= {strobe_s[1:0], strobe_in};
        end
    end
    wire apply_pulse = apply_s[2] ^ apply_s[1];
    wire sw_pulse    = sw_s[2] ^ sw_s[1];
    wire strobe_fall = strobe_s[2] & ~strobe_s[1];

    // ---- working configuration ------------------------------------------------
    // Bounds are computed once on apply, not on every cycle.
    reg [1:0]  src, lmode;
    reg        ext_fall, len;
    reg [2:0]  pol;
    reg [31:0] d0, e0, d1, e1, dl, el, ps, pe, filt, max_end;
    // Intermediate stages of max_end: a cascade of four compares did not fit into
    // a 7.5 ns lane4_clk cycle (ERRORS 4.3).
    reg [31:0] max_a, max_b;
    reg        apply_d, apply_d2;

    wire [31:0] pre_start = (cfg_pre_lead > cfg_laser_delay) ? 32'd0
                                                             : (cfg_laser_delay - cfg_pre_lead);

    function [31:0] max2(input [31:0] a, input [31:0] b);
        max2 = (a > b) ? a : b;
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enabled  <= 1'b1;
            src      <= SRC_TIMER;
            ext_fall <= 1'b0;
            d0 <= 32'd0;  e0 <= DEF_CAM_WIDTH;
            d1 <= 32'd0;  e1 <= DEF_CAM_WIDTH;
            dl <= 32'd0;  el <= DEF_LASER_WIDTH;
            lmode <= LM_FORCE0;
            len   <= 1'b0;
            ps <= 32'd0;  pe <= 32'd0;
            pol   <= 3'd0;
            filt  <= DEF_EXT_FILTER;
            max_end <= DEF_CAM_WIDTH;
            max_a   <= DEF_CAM_WIDTH;
            max_b   <= 32'd0;
            apply_d  <= 1'b0;
            apply_d2 <= 1'b0;
        end else begin
            apply_d  <= apply_pulse;
            apply_d2 <= apply_d;
            if (apply_pulse) begin
                enabled  <= cfg_enable;
                src      <= cfg_source;
                ext_fall <= cfg_ext_fall;
                d0 <= cfg_cam0_delay;   e0 <= cfg_cam0_delay  + cfg_cam0_width;
                d1 <= cfg_cam1_delay;   e1 <= cfg_cam1_delay  + cfg_cam1_width;
                dl <= cfg_laser_delay;  el <= cfg_laser_delay + cfg_laser_width;
                lmode <= cfg_laser_mode;
                len   <= cfg_laser_enable;
                ps <= pre_start;        pe <= pre_start + cfg_pre_width;
                pol   <= cfg_polarity;
                filt  <= cfg_ext_filter;
            end
            // End of the pulse series - the latest bound. Computed in THREE
            // stages, not in one cycle.
            //
            // Was: max_end <= max2(max2(e0,e1), max2(el,pe)) - four
            // 32-bit compares in a row. At lane4_clk 133.333 MHz (7.5 ns)
            // this does not pass: 13 logic levels, Fmax 131.162 against 133.333,
            // critical path e1 -> max_end (21.09.2026, ERRORS 4.3).
            //
            // The stages deliberately come AFTER the cycle in which e0..pe are computed:
            // that is a non-blocking assignment, and in the apply_pulse cycle these
            // registers still hold the previous values.
            if (apply_d) begin
                max_a <= max2(e0, e1);
                max_b <= max2(el, pe);
            end
            if (apply_d2) max_end <= max2(max_a, max_b);
        end
    end

    // ---- external input: filter and edge -------------------------------------------
    reg        ext_filt, ext_filt_d;
    reg [31:0] fcnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ext_filt   <= 1'b0;
            ext_filt_d <= 1'b0;
            fcnt       <= 32'd0;
        end else begin
            ext_filt_d <= ext_filt;
            if (ext_s[1] == ext_filt)  fcnt <= 32'd0;
            else if (fcnt == filt) begin
                ext_filt <= ext_s[1];
                fcnt     <= 32'd0;
            end else                   fcnt <= fcnt + 32'd1;
        end
    end
    wire ext_edge = ext_fall ? (ext_filt_d & ~ext_filt) : (~ext_filt_d & ext_filt);

    reg timer_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) timer_d <= 1'b0;
        else        timer_d <= timer_in;
    wire timer_edge = timer_in & ~timer_d;

    // For THREE cycles after apply triggers are not accepted: max_end is computed in
    // three stages (it was two cycles while the compare cascade sat in one).
    wire hold     = apply_pulse | apply_d | apply_d2;
    wire src_edge = (src == SRC_CORE)  ? sw_pulse   :
                    (src == SRC_TIMER) ? timer_edge :
                    (src == SRC_EXT)   ? ext_edge   : 1'b0;
    wire trig     = enabled & ~hold & src_edge;

    // ---- pulses ---------------------------------------------------------------------
    reg [31:0] ts, cnt;
    // The end of THIS series, latched when it starts (19.10.2026, ERRORS 4.10): the
    // end is an equality test, and an apply that moved max_end below a running cnt
    // kept "active" until cnt wrapped (~43 s) - every trigger an overrun.
    reg [31:0] run_end;
    reg        o0, o1, ol, op;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ts <= 32'd0;  cnt <= 32'd0;  run_end <= 32'd0;
            o0 <= 1'b0;  o1 <= 1'b0;  ol <= 1'b0;  op <= 1'b0;
            active        <= 1'b0;
            exposure_done <= 1'b0;
            overrun_flag  <= 1'b0;
            cnt_accepted  <= 32'd0;
            cnt_overrun   <= 32'd0;
            cnt_ext_edges <= 32'd0;
            last_trig_ts  <= 32'd0;
        end else begin
            ts <= ts + 32'd1;
            if (ext_edge)    cnt_ext_edges <= cnt_ext_edges + 32'd1;
            if (strobe_fall) exposure_done <= 1'b1;

            if (!enabled) begin
                active <= 1'b0;
                o0 <= 1'b0;  o1 <= 1'b0;  ol <= 1'b0;  op <= 1'b0;
            end else if (trig && !active) begin
                active        <= 1'b1;
                cnt           <= 32'd0;
                run_end       <= max_end;     // valid: hold keeps triggers out until it is
                cnt_accepted  <= cnt_accepted + 32'd1;
                last_trig_ts  <= ts;
                exposure_done <= 1'b0;
                overrun_flag  <= 1'b0;
                o0 <= 1'b0;  o1 <= 1'b0;  ol <= 1'b0;  op <= 1'b0;
            end else if (active) begin
                if (trig) begin
                    cnt_overrun  <= cnt_overrun + 32'd1;
                    overrun_flag <= 1'b1;
                end
                cnt <= cnt + 32'd1;
                // The clear is written after the set: with zero width the output
                // is never raised at all.
                if (cnt == d0) o0 <= 1'b1;
                if (cnt == e0) o0 <= 1'b0;
                if (cnt == d1) o1 <= 1'b1;
                if (cnt == e1) o1 <= 1'b0;
                if (cnt == dl) ol <= 1'b1;
                if (cnt == el) ol <= 1'b0;
                if (cnt == ps) op <= 1'b1;
                if (cnt == pe) op <= 1'b0;
                if (cnt == run_end) begin
                    active <= 1'b0;
                    o0 <= 1'b0;  o1 <= 1'b0;  ol <= 1'b0;  op <= 1'b0;
                end
            end
        end
    end

    // ---- outputs, registered ---------------------------------------------------------
    wire laser_raw = (lmode == LM_FORCE0)   ? 1'b0 :
                     (lmode == LM_FORCE1)   ? 1'b1 :
                     (lmode == LM_PREPULSE) ? (ol | op) : ol;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cam0_trig    <= 1'b0;
            cam1_trig    <= 1'b0;
            laser_gate   <= 1'b0;
            laser_enable <= 1'b0;
        end else begin
            cam0_trig    <= (enabled & o0) ^ pol[0];
            cam1_trig    <= (enabled & o1) ^ pol[1];
            laser_gate   <= (enabled & len & laser_raw) ^ pol[2];
            laser_enable <= enabled & len;
        end
    end

endmodule
