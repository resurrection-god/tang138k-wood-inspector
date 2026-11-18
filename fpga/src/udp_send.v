// ============================================================================
// Module      : udp_send
// The tx_mac_clk half of the FPGA frame stream (docs/ARCHITECTURE.md § 8.1):
// it takes whole packets out of the packet
// FIFO that udp_rd fills, prepends Ethernet / IPv4 / UDP headers (the IPv4
// checksum here; UDP checksum 0) and hands the bytes to the MAC through
// eth_tx_mux, a byte on every ready.
//
// The structure is fpga/udp_probe's generator, proven on the board (ERRORS
// 3.12): a producer writes a byte per cycle into an 8-byte FIFO and the sender
// starts a packet only with 6 bytes queued, so the MAC never sees a gap inside
// a frame (IPUG538: the MAC has no transmit buffer). Two things the probe
// taught on top of it: the sender's outputs go through a two-entry register
// slice, so tx_valid/tx_data/tx_last are flops and tx_ready reaches only the
// slice (a handshake across a module boundary is flop -> <= 2 LUTs -> flop);
// and a packet's end moves the sender on from a flop, one cycle later.
//
// FIFO word {ctrl, 4 bytes}, byte 0 in bits 7:0 first; a packet is
// [ctrl: rows, udp_len][udp_len / 4 payload words], and a packet is started
// only once udp_rd has said it is all there (pkt_go), so the FIFO cannot run
// dry inside one.
// ============================================================================

`timescale 1ns / 1ps

module udp_send #(
    parameter [11:0] STAT_TIMEOUT = 12'd4095
) (
    input  wire        clk,
    input  wire        rst_n,

    // job: fields held by udp_rd from before job_go until the job's end
    input  wire        job_go,
    input  wire [47:0] dmac,
    input  wire [47:0] smac,
    input  wire [31:0] dip,
    input  wire [31:0] sip,
    input  wire [15:0] dport,
    input  wire [15:0] sport,
    input  wire [31:0] gap,
    // a whole packet is in the FIFO (one pulse per packet)
    input  wire        pkt_go,

    // packet FIFO, read side (rd_data valid the cycle after rd)
    output wire        f_rd,
    input  wire [32:0] f_rdata,
    input  wire        f_empty,

    // towards eth_tx_mux (flops)
    output wire        tx_valid,
    output wire [7:0]  tx_data,
    output wire        tx_last,
    input  wire        tx_ready,
    input  wire        stat_valid,

    output reg         sent_tgl,        // one toggle per packet on the wire
    output reg  [31:0] n_sent,
    output reg  [15:0] n_stat_to,
    output reg  [15:0] n_underrun,
    output reg  [15:0] n_bad_ctrl
);

    // ---------------------------------------------------------------- job
    reg [47:0] c_dmac, c_smac;
    reg [31:0] c_dip, c_sip, c_gap;
    reg [15:0] c_dport, c_sport;
    reg [3:0]  settle;
    reg [19:0] cs_base;              // constant words of the IPv4 header checksum
    always @(posedge clk) begin
        cs_base <= 20'h4500 + 20'h4000 + 20'h4011
                 + {4'd0, c_sip[31:16]} + {4'd0, c_sip[15:0]} + {4'd0, c_dip[31:16]} + {4'd0, c_dip[15:0]};
    end

    // ---------------------------------------------------------------- byte FIFO
    reg [8:0]  bf [0:7];             // {last, byte}
    reg [2:0]  wp, rp;
    reg [3:0]  bfcnt;
    reg        p_v, p_last;
    reg [7:0]  p_b;
    wire       room = (bfcnt <= 4'd5);
    always @(posedge clk) if (p_v) bf[wp] <= {p_last, p_b};

    // ---------------------------------------------------------------- slice to the mux
    reg        mv, kv;
    reg [8:0]  md, kd;
    wire       s_ready = !kv;
    assign tx_valid = mv;
    assign tx_data  = md[7:0];
    assign tx_last  = md[8];

    // ---------------------------------------------------------------- producer
    localparam [2:0] Q_IDLE = 3'd0, Q_CTRL = 3'd1, Q_CS = 3'd2, Q_HDR = 3'd3, Q_PAY = 3'd4, Q_DONE = 3'd5;
    reg [2:0]   q;
    reg [15:0]  pkts_avail;
    reg [15:0]  udp_len, ip_total, bytes_left, words_left;
    reg [1:0]   cs_step;
    reg [19:0]  cs_sum;
    reg [16:0]  cs_f1;
    reg [15:0]  ip_csum;
    reg [335:0] sr;                  // 42 header bytes, first in [335:328]
    reg [5:0]   hdr_left;
    reg         rd_fly;              // a word is on its way out of the FIFO's RAM
    reg         nw_v, pw_v;
    reg [31:0]  nw, pw;
    reg [1:0]   pb;
    reg         start_rdy;

    // ---------------------------------------------------------------- sender
    localparam [1:0] C_IDLE = 2'd0, C_SEND = 2'd1, C_STAT = 2'd2;
    reg [1:0]   cs;
    reg         c_valid;
    reg         last_rd;
    reg [31:0]  gap_cnt;
    reg [11:0]  stat_cnt;
    wire        c_take = c_valid && s_ready;              // a byte enters the slice
    wire [8:0]  c_word = bf[rp];

    wire rd_ctrl = (q == Q_IDLE) && start_rdy && (settle == 4'd0) && !job_go;
    wire rd_pay  = (q == Q_HDR || q == Q_PAY) && (words_left != 16'd0) && !rd_fly && !nw_v;
    assign f_rd  = (rd_ctrl || rd_pay) && !f_empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_dmac <= 48'd0; c_smac <= 48'd0; c_dip <= 32'd0; c_sip <= 32'd0; c_gap <= 32'd0;
            c_dport <= 16'd0; c_sport <= 16'd0; settle <= 4'd0;
            wp <= 3'd0; rp <= 3'd0; bfcnt <= 4'd0; p_v <= 1'b0; p_last <= 1'b0; p_b <= 8'd0;
            mv <= 1'b0; kv <= 1'b0; md <= 9'd0; kd <= 9'd0;
            q <= Q_IDLE; pkts_avail <= 16'd0; udp_len <= 16'd0; ip_total <= 16'd0;
            bytes_left <= 16'd0; words_left <= 16'd0; cs_step <= 2'd0;
            cs_sum <= 20'd0; cs_f1 <= 17'd0; ip_csum <= 16'd0; sr <= 336'd0; hdr_left <= 6'd0;
            rd_fly <= 1'b0; nw_v <= 1'b0; pw_v <= 1'b0; nw <= 32'd0; pw <= 32'd0; pb <= 2'd0;
            start_rdy <= 1'b0;
            cs <= C_IDLE; c_valid <= 1'b0; last_rd <= 1'b0; gap_cnt <= 32'd0; stat_cnt <= 12'd0;
            sent_tgl <= 1'b0; n_sent <= 32'd0; n_stat_to <= 16'd0; n_underrun <= 16'd0; n_bad_ctrl <= 16'd0;
        end else begin
            // ---- job
            if (job_go) begin
                c_dmac <= dmac; c_smac <= smac; c_dip <= dip; c_sip <= sip; c_gap <= gap;
                c_dport <= dport; c_sport <= sport; settle <= 4'd8;
            end else if (settle != 4'd0) begin
                settle <= settle - 4'd1;
            end
            pkts_avail <= pkts_avail + {15'd0, pkt_go} - {15'd0, (rd_ctrl && !f_empty)};

            // ---- slice
            if (mv && tx_ready) begin
                if (kv) begin md <= kd; kv <= 1'b0; end
                else if (c_take) md <= c_word;
                else mv <= 1'b0;
            end else if (!mv) begin
                if (c_take) begin mv <= 1'b1; md <= c_word; end
            end else if (c_take) begin
                kv <= 1'b1; kd <= c_word;
            end

            // ---- byte FIFO pointers
            if (p_v) wp <= wp + 3'd1;
            bfcnt <= bfcnt + {3'd0, p_v} - {3'd0, c_take};

            // ---- payload words: the FIFO's read latency is one cycle
            if (rd_pay && !f_empty) begin rd_fly <= 1'b1; words_left <= words_left - 16'd1; end
            if (rd_fly) begin rd_fly <= 1'b0; nw <= f_rdata[31:0]; nw_v <= 1'b1; end

            // ---- producer
            start_rdy <= (cs == C_IDLE) && (bfcnt == 4'd0) && !p_v && (pkts_avail != 16'd0)
                         && (gap_cnt == 32'd0) && !mv && !kv;
            p_v <= 1'b0;
            case (q)
            Q_IDLE: if (rd_ctrl && !f_empty) q <= Q_CTRL;
            Q_CTRL: begin
                // the ctrl word is in f_rdata now
                if (!f_rdata[32] && n_bad_ctrl != 16'hFFFF) n_bad_ctrl <= n_bad_ctrl + 16'd1;
                udp_len    <= f_rdata[15:0];
                ip_total   <= f_rdata[15:0] + 16'd28;
                bytes_left <= f_rdata[15:0];
                words_left <= f_rdata[15:0] >> 2;
                cs_step <= 2'd0;
                q <= Q_CS;
            end
            Q_CS: begin
                cs_step <= cs_step + 2'd1;
                case (cs_step)
                    2'd0: cs_sum  <= cs_base + {4'd0, ip_total};
                    2'd1: cs_f1   <= {1'b0, cs_sum[15:0]} + {13'd0, cs_sum[19:16]};
                    default: begin
                        ip_csum <= ~(cs_f1[15:0] + {15'd0, cs_f1[16]});
                        q <= Q_HDR;
                        hdr_left <= 6'd42;
                    end
                endcase
            end
            Q_HDR: begin
                if (hdr_left == 6'd42) begin
                    // first cycle in Q_HDR: ip_csum is ready, build the header
                    sr <= {c_dmac, c_smac, 16'h0800,
                           8'h45, 8'h00, ip_total, 16'h0000, 16'h4000, 8'h40, 8'h11, ip_csum,
                           c_sip, c_dip, c_sport, c_dport, udp_len + 16'd8, 16'h0000};
                    hdr_left <= 6'd41;
                end else if (room) begin
                    p_v <= 1'b1; p_b <= sr[335:328]; p_last <= 1'b0;
                    sr <= {sr[327:0], 8'd0};
                    if (hdr_left == 6'd0) q <= Q_PAY;
                    else hdr_left <= hdr_left - 6'd1;
                end
            end
            Q_PAY: begin
                if (room) begin
                    if (pw_v) begin
                        p_v <= 1'b1; p_b <= pw[8 * pb +: 8]; p_last <= (bytes_left == 16'd1);
                        bytes_left <= bytes_left - 16'd1;
                        pb <= pb + 2'd1;
                        if (pb == 2'd3) begin
                            if (nw_v) begin pw <= nw; nw_v <= 1'b0; end
                            else pw_v <= 1'b0;
                        end
                        if (bytes_left == 16'd1) q <= Q_DONE;
                    end else if (nw_v) begin
                        pw <= nw; pw_v <= 1'b1; nw_v <= 1'b0; pb <= 2'd0;
                    end
                end
            end
            default: if (cs == C_STAT) q <= Q_IDLE;          // Q_DONE: wait for the sender
            endcase
            if (q == Q_DONE) pw_v <= 1'b0;

            // ---- sender
            last_rd <= c_take && c_word[8];
            case (cs)
            C_IDLE: begin
                if (gap_cnt != 32'd0) gap_cnt <= gap_cnt - 32'd1;
                if (bfcnt >= 4'd6 || (q == Q_DONE && bfcnt != 4'd0 && !p_v)) begin
                    c_valid <= 1'b1; cs <= C_SEND;
                end
            end
            C_SEND: begin
                if (bfcnt == 4'd0 && !last_rd && n_underrun != 16'hFFFF) n_underrun <= n_underrun + 16'd1;
                if (c_take) rp <= rp + 3'd1;
                if (c_take && c_word[8]) c_valid <= 1'b0;   // nothing after the last byte
                if (last_rd) begin cs <= C_STAT; stat_cnt <= 12'd0; end
            end
            C_STAT: begin
                stat_cnt <= stat_cnt + 12'd1;
                if (stat_valid || stat_cnt == STAT_TIMEOUT) begin
                    if (!stat_valid && n_stat_to != 16'hFFFF) n_stat_to <= n_stat_to + 16'd1;
                    sent_tgl <= ~sent_tgl;
                    n_sent   <= n_sent + 32'd1;
                    gap_cnt  <= c_gap;
                    cs <= C_IDLE;
                end
            end
            default: cs <= C_IDLE;
            endcase
        end
    end

endmodule
