// ============================================================================
// Module      : laser_cog_div
// Pipelined divider for the centre of gravity of the laser spot (30.09.2026,
// profile block version 2, docs/ARCHITECTURE.md § 6.1, ERRORS 1.14).
//
//   q = floor(num / den), num = 16*sum_iv + floor(sum_i/2), den = sum_i
//
// i.e. the COG in 1/16 pixel with rounding - tools/laser/model.py
// refine_fixed(). Restoring division, one quotient bit per stage:
// 15 stages, a result every cycle, latency IN_REG + 15 = 16 cycles.
//
// The bounds the width relies on (they are guaranteed by the finder, not by
// this module):
//   * sum_i <= 1280 rows * 255 < 2^19 - a 20-bit divisor;
//   * the COG is a weighted mean row of the frame (<= 1279), all terms >= 0, so
//     the quotient < 16*1280 < 2^15 and num < den * 2^15 < 2^35 - a 35-bit
//     remainder, and quotient bits above 14 are zero by construction.
// If num >= den*2^15 (sums not from a single spot - cannot happen), the
// quotient saturates at 0x7FFF instead of wrapping.
//
// Everything on lane4_clk; one stage is a 36-bit subtraction and a mux, with no
// feedback. The divider does not distinguish den = 0: the caller itself sets
// "no line" and x_sub = 0 (the flag travels in the tag).
// ============================================================================

`timescale 1ns / 1ps

module laser_cog_div #(
    parameter integer TAG_W = 27
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              in_valid,
    input  wire [34:0]       in_num,
    input  wire [19:0]       in_den,
    input  wire [TAG_W-1:0]  in_tag,
    output wire              out_valid,
    output wire [14:0]       out_q,
    output wire [TAG_W-1:0]  out_tag
);

    localparam integer ST = 15;

    // Stage s (0..ST): s = 0 is the input register, s >= 1 is after quotient
    // bit 15 - s.
    reg              v   [0:ST];
    reg [34:0]       rem [0:ST];
    reg [19:0]       den [0:ST];
    reg [14:0]       q   [0:ST];
    reg [TAG_W-1:0]  tag [0:ST];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) v[0] <= 1'b0;
        else        v[0] <= in_valid;
    end
    always @(posedge clk) begin
        rem[0] <= in_num;
        den[0] <= in_den;
        q[0]   <= 15'd0;
        tag[0] <= in_tag;
    end

    genvar s;
    generate
        for (s = 1; s <= ST; s = s + 1) begin : g_st
            localparam integer B = ST - s;            // quotient bit: 14 .. 0
            // den << B in 35 bits: den < 2^20, B <= 14 - it fits.
            wire [35:0] sub = {1'b0, rem[s-1]} - {1'b0, {{(15 - B){1'b0}}, den[s-1], {B{1'b0}}}};
            wire        ge  = !sub[35];
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) v[s] <= 1'b0;
                else        v[s] <= v[s-1];
            end
            always @(posedge clk) begin
                rem[s] <= ge ? sub[34:0] : rem[s-1];
                den[s] <= den[s-1];
                q[s]   <= {q[s-1][13:0], ge};
                tag[s] <= tag[s-1];
            end
        end
    endgenerate

    // Saturation: the remainder after the last stage must be < den; if not, the
    // quotient did not fit into 15 bits (see the header).
    wire over = (rem[ST] >= {15'd0, den[ST]});

    assign out_valid = v[ST];
    assign out_q     = over ? 15'h7FFF : q[ST];
    assign out_tag   = tag[ST];

endmodule
