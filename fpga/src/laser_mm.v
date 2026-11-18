// ============================================================================
// Module      : laser_mm
// Profile point -> millimetres by the "pixel -> mm" table (16.10.2026,
// docs/DECISIONS.md R-38, docs/ARCHITECTURE.md § 6.3).
//
// The table is built on the PC from the calibration (tools/laser/mm_table.py):
// a node every 16 sensor columns and every 8 sensor rows, a node is a word
// {depth[31:16], along[15:0]}, int16 each, units chosen by the PC; along =
// -32768 marks an invalid node. Here only the band of node rows under the current
// ROI is held, loaded by lane5_reader.v from DDR3 and checked there by CRC-32.
// The band is four BSRAM banks of 512 words by the parity of the node row and
// column, so the four nodes around a point are read in one clock and a point can
// enter every clock.
//
// Arithmetic - the bit-exact contract tools/laser/mm_table.py interp_fixed():
//   vr  = x_sub + vofs           (1/16 px from the band's first node row)
//   kab = column + kofs
//   nr, fv = vr >> 7, vr[6:0];  nc, fk = kab >> 4, kab[3:0]
//   L = P00*(16-fk) + P01*fk,  R = P10*(16-fk) + P11*fk
//   out = (L*(128-fv) + R*fv + 1024) >>> 11
// A convex combination of int16 nodes, so it cannot overflow.
//
// mm word: {depth[31:16], along[15:0]}; along = 0x8000 - no mm, and then depth is
// the reason: 1 no line, 2 table off for this block, 3 outside the band, 4 an
// invalid node.
//
// Fixed latency of 9 clocks from in_valid to out_valid, a point every clock,
// in_tag travels along. Everything on lane4_clk. The band is written only while
// laser_prof holds the loader's acknowledge, i.e. never during an unload that
// uses it (laser_prof.v, mm_load_req/ack).
// ============================================================================

`timescale 1ns / 1ps

module laser_mm #(
    parameter integer NODE_COLS = 69,      // mm_table.NODE_COLS
    parameter integer BANK_STRIDE = 35,    // words per node-row pair in a bank
    parameter integer TAG_W = 1
)(
    input  wire              clk,
    input  wire              rst_n,

    // Band write (lane5_reader.v through laser_prof): bank {row parity, column parity}.
    input  wire              tw_en,
    input  wire [1:0]        tw_bank,
    input  wire [8:0]        tw_addr,
    input  wire [31:0]       tw_data,

    // Table state of this block (latched by laser_prof at the unload start).
    input  wire              cfg_en,
    input  wire [15:0]       cfg_vofs,     // signed, 1/16 px
    input  wire [11:0]       cfg_kofs,
    input  wire [4:0]        cfg_rows,     // node rows in the band

    // Point.
    input  wire              in_valid,
    input  wire [31:0]       in_px,        // {q, i, x_sub}
    input  wire [10:0]       in_col,
    input  wire [TAG_W-1:0]  in_tag,

    output reg               out_valid,
    output reg  [31:0]       out_mm,
    output reg  [TAG_W-1:0]  out_tag
);

    localparam [15:0] INVALID = 16'h8000;
    localparam [2:0]  R_NO_LINE = 3'd1, R_OFF = 3'd2, R_OUT = 3'd3, R_NODE = 3'd4;

    // ---- band memory: four banks ----------------------------------------------
    reg [31:0] bank0 [0:511];
    reg [31:0] bank1 [0:511];
    reg [31:0] bank2 [0:511];
    reg [31:0] bank3 [0:511];
    reg        we0, we1, we2, we3;
    reg [8:0]  wa;
    reg [31:0] wd;
    // One register on the write side: the bus comes from the camera 0 channel,
    // across the die for camera 1.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            we0 <= 1'b0; we1 <= 1'b0; we2 <= 1'b0; we3 <= 1'b0;
        end else begin
            we0 <= tw_en && (tw_bank == 2'd0);
            we1 <= tw_en && (tw_bank == 2'd1);
            we2 <= tw_en && (tw_bank == 2'd2);
            we3 <= tw_en && (tw_bank == 2'd3);
        end
    end
    always @(posedge clk) begin
        wa <= tw_addr;
        wd <= tw_data;
    end

    reg [8:0]  ra0, ra1, ra2, ra3;
    reg [31:0] rq0, rq1, rq2, rq3;
    reg [31:0] rr0, rr1, rr2, rr3;
    always @(posedge clk) begin
        if (we0) bank0[wa] <= wd;
        if (we1) bank1[wa] <= wd;
        if (we2) bank2[wa] <= wd;
        if (we3) bank3[wa] <= wd;
        rq0 <= bank0[ra0];
        rq1 <= bank1[ra1];
        rq2 <= bank2[ra2];
        rq3 <= bank3[ra3];
        rr0 <= rq0; rr1 <= rq1; rr2 <= rq2; rr3 <= rq3;   // BSRAM output register
    end

    // ---- pipeline ------------------------------------------------------------
    // Valid bits without reset of the data: data is meaningful only with its valid.
    // v[k] - the point is in the registers after k + 1 clocks (s1 .. s8); out
    // after nine.
    localparam integer ST = 8;
    reg [ST-1:0] v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) v <= {ST{1'b0}};
        else        v <= {v[ST-2:0], in_valid};
    end

    // s1: offsets
    reg               s1_noline;
    reg signed [17:0] s1_vr;
    reg [12:0]        s1_kab;
    reg [TAG_W-1:0]   s1_tag;
    always @(posedge clk) begin
        s1_noline <= in_px[24];                                  // Q_NO_LINE
        s1_vr     <= $signed({3'd0, in_px[14:0]}) + $signed({{2{cfg_vofs[15]}}, cfg_vofs});
        s1_kab    <= {2'd0, in_col} + {1'b0, cfg_kofs};
        s1_tag    <= in_tag;
    end

    // s2: node, fractions, reason, row-pair base
    reg [2:0]         s2_reason;
    reg [6:0]         s2_fv;
    reg [3:0]         s2_fk;
    reg               s2_r0, s2_c0;
    reg [8:0]         s2_rbase;        // (nr >> 1) * BANK_STRIDE
    reg [5:0]         s2_ch;           // nc >> 1
    reg [TAG_W-1:0]   s2_tag;
    wire [10:0] nr = s1_vr[17:7];
    wire [8:0]  nc = s1_kab[12:4];
    localparam [8:0] NC_MAX = NODE_COLS[8:0] - 9'd2;  // last node column a cell can start at
    wire        oob = s1_vr[17] || ({1'b0, nr[9:0]} + 11'd2 > {6'd0, cfg_rows}) ||
                      (nc > NC_MAX);
    wire [3:0]  nrh = nr[4:1];
    always @(posedge clk) begin
        s2_reason <= s1_noline ? R_NO_LINE : !cfg_en ? R_OFF : oob ? R_OUT : 3'd0;
        s2_fv     <= s1_vr[6:0];
        s2_fk     <= s1_kab[3:0];
        s2_r0     <= nr[0];
        s2_c0     <= nc[0];
        s2_rbase  <= {nrh, 5'd0} + {3'd0, nrh, 1'b0} + {5'd0, nrh};   // * 35
        s2_ch     <= nc[6:1];
        s2_tag    <= s1_tag;
    end

    // s3: bank addresses. Corner (dr, dc) lies in bank {r0 ^ dr, c0 ^ dc}; the bank
    // with row parity rb holds row nr if nr[0] == rb, else nr + 1, whose pair index
    // is one more only when nr is odd (and rb = 0) - same for columns.
    wire [8:0] rb_lo = s2_rbase;
    wire [8:0] rb_hi = s2_rbase + (s2_r0 ? BANK_STRIDE[8:0] : 9'd0);   // for rb = 0
    wire [5:0] ch_lo = s2_ch;
    wire [5:0] ch_hi = s2_ch + {5'd0, s2_c0};                           // for cb = 0
    reg [2:0]         s3_reason;
    reg [6:0]         s3_fv;
    reg [3:0]         s3_fk;
    reg               s3_r0, s3_c0;
    reg [TAG_W-1:0]   s3_tag;
    always @(posedge clk) begin
        ra0 <= rb_hi + {3'd0, ch_hi};       // bank 0: row parity 0, column parity 0
        ra1 <= rb_hi + {3'd0, ch_lo};       // bank 1: row parity 0, column parity 1
        ra2 <= rb_lo + {3'd0, ch_hi};       // bank 2: row parity 1, column parity 0
        ra3 <= rb_lo + {3'd0, ch_lo};       // bank 3: row parity 1, column parity 1
        s3_reason <= s2_reason;
        s3_fv     <= s2_fv;
        s3_fk     <= s2_fk;
        s3_r0     <= s2_r0;
        s3_c0     <= s2_c0;
        s3_tag    <= s2_tag;
    end

    // s4, s5: memory read and its output register (rq*, rr*); the side data waits.
    reg [2:0]       s4_reason, s5_reason;
    reg [6:0]       s4_fv, s5_fv;
    reg [3:0]       s4_fk, s5_fk;
    reg             s4_r0, s4_c0, s5_r0, s5_c0;
    reg [TAG_W-1:0] s4_tag, s5_tag;
    always @(posedge clk) begin
        s4_reason <= s3_reason; s4_fv <= s3_fv; s4_fk <= s3_fk;
        s4_r0 <= s3_r0; s4_c0 <= s3_c0; s4_tag <= s3_tag;
        s5_reason <= s4_reason; s5_fv <= s4_fv; s5_fk <= s4_fk;
        s5_r0 <= s4_r0; s5_c0 <= s4_c0; s5_tag <= s4_tag;
    end

    // s6: corners by parity, node validity
    function [31:0] pick;
        input [1:0]  b;
        input [31:0] q0, q1, q2, q3;
        begin
            case (b)
                2'd0: pick = q0;
                2'd1: pick = q1;
                2'd2: pick = q2;
                default: pick = q3;
            endcase
        end
    endfunction
    wire [31:0] p00 = pick({ s5_r0,  s5_c0}, rr0, rr1, rr2, rr3);
    wire [31:0] p01 = pick({ s5_r0, ~s5_c0}, rr0, rr1, rr2, rr3);
    wire [31:0] p10 = pick({~s5_r0,  s5_c0}, rr0, rr1, rr2, rr3);
    wire [31:0] p11 = pick({~s5_r0, ~s5_c0}, rr0, rr1, rr2, rr3);
    reg signed [15:0] a00, a01, a10, a11, d00, d01, d10, d11;
    reg [2:0]         s6_reason;
    reg [6:0]         s6_fv;
    reg [3:0]         s6_fk;
    reg [TAG_W-1:0]   s6_tag;
    always @(posedge clk) begin
        a00 <= p00[15:0];  d00 <= p00[31:16];
        a01 <= p01[15:0];  d01 <= p01[31:16];
        a10 <= p10[15:0];  d10 <= p10[31:16];
        a11 <= p11[15:0];  d11 <= p11[31:16];
        s6_reason <= (s5_reason != 3'd0) ? s5_reason :
                     ((p00[15:0] == INVALID) || (p01[15:0] == INVALID) ||
                      (p10[15:0] == INVALID) || (p11[15:0] == INVALID)) ? R_NODE : 3'd0;
        s6_fv  <= s5_fv;
        s6_fk  <= s5_fk;
        s6_tag <= s5_tag;
    end

    // s7: along the column fraction. Weights as signed 6-bit (0..16).
    wire signed [5:0] wk1 = {2'd0, s6_fk};
    wire signed [5:0] wk0 = 6'sd16 - wk1;
    reg signed [22:0] la, ra, ld, rd;
    reg [2:0]         s7_reason;
    reg [6:0]         s7_fv;
    reg [TAG_W-1:0]   s7_tag;
    always @(posedge clk) begin
        la <= a00 * wk0 + a01 * wk1;
        ra <= a10 * wk0 + a11 * wk1;
        ld <= d00 * wk0 + d01 * wk1;
        rd <= d10 * wk0 + d11 * wk1;
        s7_reason <= s6_reason;
        s7_fv     <= s6_fv;
        s7_tag    <= s6_tag;
    end

    // s8: along the row fraction. Weights as signed 9-bit (0..128).
    wire signed [8:0] wv1 = {2'd0, s7_fv};
    wire signed [8:0] wv0 = 9'sd128 - wv1;
    reg signed [32:0] sa, sd;
    reg [2:0]         s8_reason;
    reg [TAG_W-1:0]   s8_tag;
    always @(posedge clk) begin
        sa <= la * wv0 + ra * wv1;
        sd <= ld * wv0 + rd * wv1;
        s8_reason <= s7_reason;
        s8_tag    <= s7_tag;
    end

    // s9 = out: round and the word.
    wire signed [32:0] ka = (sa + 33'sd1024) >>> 11;
    wire signed [32:0] kd = (sd + 33'sd1024) >>> 11;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) out_valid <= 1'b0;
        else        out_valid <= v[7];
    end
    always @(posedge clk) begin
        out_mm  <= (s8_reason != 3'd0) ? {13'd0, s8_reason, INVALID} : {kd[15:0], ka[15:0]};
        out_tag <= s8_tag;
    end

endmodule
