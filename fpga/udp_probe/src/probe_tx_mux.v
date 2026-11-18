// UDP probe: two packet sources share the MAC's client transmit port.
//
// A is Gowin's AHB wrapper (gw_ahb_ethernet, the CPU's path), B the probe's
// generator. The port is granted per packet: the owner's bytes pass, the other
// source sees ready = 0 and simply waits with its first byte up - what the MAC
// itself does to every source during the gap and preamble (IPUG538 4.3.2).
// After the owner's last byte the mux waits for the MAC's tx_statistics_valid
// and hands that pulse to the owner only, then grants again; Gowin's
// reference user logic waits for that pulse before its next packet, and the
// wrapper may too. When both wait, the one that did not send last goes first.
//
// Fully registered towards both sides (first build: 107 MHz of 125). The MAC's
// client inputs go straight into its CRC and state logic, and the wrapper's
// outputs come out of its own logic, so a mux LUT between them had no room;
// likewise tx_mac_ready and tx_statistics_valid must not reach the sources'
// counters through logic. Hence a two-entry register slice in front of the
// MAC (valid/data/last from flops, ready to the sources from a flop) and a
// registered statistics pulse. The slice keeps valid up through a frame as
// long as the owner offers a byte whenever it sees ready.
`timescale 1ns / 1ps

module probe_tx_mux #(
    parameter [11:0] STAT_TIMEOUT = 12'd4095
) (
    input  wire        clk,
    input  wire        rst_n,
    // source A
    input  wire        a_valid,
    input  wire [7:0]  a_data,
    input  wire        a_last,
    input  wire        a_error,
    output wire        a_ready,
    output reg         a_stat,
    // source B
    input  wire        b_valid,
    input  wire [7:0]  b_data,
    input  wire        b_last,
    output wire        b_ready,
    output reg         b_stat,
    // MAC client port
    output wire        m_valid,
    output wire [7:0]  m_data,
    output wire        m_last,
    output wire        m_error,
    input  wire        m_ready,
    input  wire        m_stat,
    // counters
    output reg  [31:0] n_a,
    output reg  [31:0] n_b,
    output reg  [15:0] n_stat_to,
    output reg  [15:0] n_abort,     // owner dropped valid before its last byte
    output reg  [31:0] max_hold_a   // cycles A waited for the port, worst case
);

    localparam [2:0] M_IDLE = 3'd0, M_A = 3'd1, M_B = 3'd2, M_STAT_A = 3'd3, M_STAT_B = 3'd4;

    reg [2:0]  st;
    reg        last_was_b;
    reg [11:0] stat_cnt;
    reg [31:0] hold_a;
    reg        stat_r;

    // ---------------------------------------------------------------- slice
    reg       mv, kv;               // main (to the MAC) and skid entries
    reg [9:0] md, kd;               // {error, last, data}
    wire      s_ready = !kv;        // a flop

    wire own_a = (st == M_A);
    wire own_b = (st == M_B);
    wire       u_valid = own_a ? a_valid : own_b ? b_valid : 1'b0;
    wire [9:0] u_word  = own_a ? {a_error, a_last, a_data} : {1'b0, b_last, b_data};
    wire       u_acc   = u_valid && s_ready;

    assign a_ready = own_a && s_ready;
    assign b_ready = own_b && s_ready;
    assign m_valid = mv;
    assign m_data  = md[7:0];
    assign m_last  = md[8];
    assign m_error = md[9];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mv <= 1'b0; kv <= 1'b0; md <= 10'd0; kd <= 10'd0;
        end else if (mv && m_ready) begin       // the MAC takes the main entry
            if (kv) begin md <= kd; kv <= 1'b0; end
            else if (u_acc) md <= u_word;
            else mv <= 1'b0;
        end else if (!mv) begin
            if (u_acc) begin mv <= 1'b1; md <= u_word; end
        end else if (u_acc) begin               // main full and held: fill the skid
            kv <= 1'b1; kd <= u_word;
        end
    end

    // ---------------------------------------------------------------- owner
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= M_IDLE; last_was_b <= 1'b0; stat_cnt <= 12'd0; hold_a <= 32'd0; stat_r <= 1'b0;
            a_stat <= 1'b0; b_stat <= 1'b0;
            n_a <= 32'd0; n_b <= 32'd0; n_stat_to <= 16'd0; n_abort <= 16'd0; max_hold_a <= 32'd0;
        end else begin
            stat_r <= m_stat;
            a_stat <= 1'b0;
            b_stat <= 1'b0;

            // how long A has been waiting with valid up and no port
            if (a_valid && !own_a) begin
                if (hold_a != 32'hFFFFFFFF) hold_a <= hold_a + 32'd1;
            end else begin
                hold_a <= 32'd0;
            end
            if (hold_a > max_hold_a) max_hold_a <= hold_a;

            case (st)
            M_IDLE: begin
                if (a_valid && (!b_valid || last_was_b)) begin
                    st <= M_A; n_a <= n_a + 32'd1;
                end else if (b_valid) begin
                    st <= M_B; n_b <= n_b + 32'd1;
                end
            end
            M_A: begin
                if (!a_valid) begin
                    n_abort <= n_abort + 16'd1; st <= M_IDLE; last_was_b <= 1'b0;
                end else if (u_acc && a_last) begin
                    st <= M_STAT_A; stat_cnt <= 12'd0; last_was_b <= 1'b0;
                end
            end
            M_B: begin
                if (!b_valid) begin
                    n_abort <= n_abort + 16'd1; st <= M_IDLE; last_was_b <= 1'b1;
                end else if (u_acc && b_last) begin
                    st <= M_STAT_B; stat_cnt <= 12'd0; last_was_b <= 1'b1;
                end
            end
            M_STAT_A, M_STAT_B: begin
                stat_cnt <= stat_cnt + 12'd1;
                if (stat_r) begin
                    if (st == M_STAT_A) a_stat <= 1'b1; else b_stat <= 1'b1;
                    st <= M_IDLE;
                end else if (stat_cnt == STAT_TIMEOUT) begin
                    n_stat_to <= n_stat_to + 16'd1; st <= M_IDLE;
                end
            end
            default: st <= M_IDLE;
            endcase
        end
    end

endmodule
