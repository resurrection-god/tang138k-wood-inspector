// ============================================================================
// Module      : trigger_gen
// Trigger series generator (the TIMER source of the sync block). Parameters are
// the TRIG words of the mailbox (docs/ARCHITECTURE.md § 4.4), read by lane5_reader.v.
//
// Split out of fpga_camera_ddr3_streamer.v on 24.09.2026 (step 4) without any
// logic change. Only the channel with ENABLE_LANE5 = 1 (camera 0) has an
// instance: camera 1 does not read the mailbox, and its generator was always
// held in reset.
// ============================================================================

`timescale 1ns / 1ps

module trigger_gen (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] cfg_period,
    input  wire [7:0]  cfg_count,
    input  wire [31:0] cfg_width,
    input  wire [31:0] cfg_seq,
    input  wire [31:0] cfg_flags,

    output reg         cam_trig,

    // State for header words 16..20.
    output wire [31:0] o_trig_pulses,
    output wire [7:0]  o_trig_left,
    output wire        o_trig_run,
    output wire        o_trig_cont,
    output wire        o_trig_active,
    output wire [31:0] o_cfg_seq_seen
);

    // Generator state. Brought out (header words 16..20) because without it
    // debugging comes down to guessing: the processor sees only what it wrote
    // itself, and whether that reached the FPGA and whether the FPGA produced
    // pulses is unknown.
    reg [31:0] trig_pulses;            // pulses issued, cumulative
    reg [31:0] trig_cnt;               // period count
    reg [31:0] trig_hi;                // pulse width count
    reg [7:0]  trig_left;              // pulses still to start
    reg        trig_run;               // series running
    reg        trig_cont;              // continuous mode
    reg        trig_active;            // pulse is high now
    reg [31:0] cfg_seq_seen;           // command accepted by the FPGA

    // ======================================================================
    // Trigger generator
    // ======================================================================
    //
    // Period and width are given in clk cycles, not in hertz: the firmware does
    // the conversion, here there is only counting. The 0.5...1200 Hz range fits
    // into 32 bits with a wide margin - 0.5 Hz is 200 000 000 cycles at 100 MHz,
    // 1200 Hz is 83 333 cycles.
    //
    // Equality compares, not "greater or equal": the counters start from zero
    // and step by one, they cannot jump past, and a 32-bit magnitude comparator
    // is exactly what docs/WORKFLOWS.md § 3 warns against at this clock
    // frequency.

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trig_pulses  <= 32'd0;
            trig_cnt     <= 32'd0;
            trig_hi      <= 32'd0;
            trig_left    <= 8'd0;
            trig_run     <= 1'b0;
            trig_cont    <= 1'b0;
            trig_active  <= 1'b0;
            cfg_seq_seen <= 32'd0;
            cam_trig     <= 1'b0;
        end else begin
            if (cfg_seq != cfg_seq_seen) begin
                // The processor incremented the command counter - accept the new parameters.
                cfg_seq_seen <= cfg_seq;
                trig_active  <= 1'b0;
                trig_hi      <= 32'd0;
                trig_cnt     <= 32'd0;
                if (cfg_flags[1] || cfg_period == 32'd0 || cfg_width == 32'd0) begin
                    // Stop, or obviously meaningless parameters. A zero
                    // period would silently turn into a constant one on the
                    // output, and a zero width into a pulse the camera
                    // would not see.
                    trig_run  <= 1'b0;
                    trig_cont <= 1'b0;
                    trig_left <= 8'd0;
                end else begin
                    trig_run  <= 1'b1;
                    trig_cont <= (cfg_count == 8'd0);
                    trig_left <= (cfg_count == 8'd0) ? 8'd0 : cfg_count;
                end
            end else if (trig_run) begin
                if (trig_active) begin
                    if (trig_hi == cfg_width - 32'd1) trig_active <= 1'b0;
                    else                              trig_hi <= trig_hi + 32'd1;
                end

                // Period counter at zero - the moment the next pulse starts.
                if (trig_cnt == 32'd0) begin
                    if (trig_cont) begin
                        trig_active <= 1'b1;
                        trig_hi     <= 32'd0;
                        trig_pulses <= trig_pulses + 32'd1;
                    end else if (trig_left != 8'd0) begin
                        trig_active <= 1'b1;
                        trig_hi     <= 32'd0;
                        trig_left   <= trig_left - 8'd1;
                        trig_pulses <= trig_pulses + 32'd1;
                    end else if (!trig_active) begin
                        trig_run <= 1'b0;      // series done
                    end
                end

                if (trig_cnt == cfg_period - 32'd1) trig_cnt <= 32'd0;
                else                                trig_cnt <= trig_cnt + 32'd1;
            end

            cam_trig <= cfg_flags[0] ? ~trig_active : trig_active;
        end
    end

    assign o_trig_pulses  = trig_pulses;
    assign o_trig_left    = trig_left;
    assign o_trig_run     = trig_run;
    assign o_trig_cont    = trig_cont;
    assign o_trig_active  = trig_active;
    assign o_cfg_seq_seen = cfg_seq_seen;

endmodule
