// UDP probe, source B: the stream and ARP replies.
//
// The MAC has no transmit buffer (IPUG538 4.3.2): once tx_valid rises, a byte
// must be ready on every tx_ready until tx_last. A producer writes the packet
// a byte per cycle into an 8-byte FIFO - header from a shift register, data
// from a pattern counter, CRC-32 over registered bytes - and the sender starts
// a packet only with 6 bytes queued, so the producer's one pause (3 cycles
// while the CRC settles) never empties it. Nothing deeper than a FIFO read
// sits between a register and the port (the first build, with the byte
// chosen and fed into the CRC in one cycle, closed at 107 MHz of 125).
//
// Stream packet (UDP payload = udp_len bytes, all big-endian):
//   Ethernet 14 | IPv4 20 (no options, DF, ID 0, UDP checksum 0) | UDP 8 |
//   app header 16: 'UPSB', cmd_id, udp_len, seq, count |
//   data udp_len-20 bytes: data[k] = (seq + k) & 0xFF |
//   CRC-32 4 bytes: zlib.crc32(app header + data), most significant byte first.
// ARP reply: 42 bytes, the MAC pads it to 60.
//
// After each packet the sender waits for the MAC's tx_statistics_valid (as
// Gowin's reference user_mac_tx_ctrl.v does), which the mux routes to the
// owner of the packet; a missing pulse is counted and the stream goes on.
`timescale 1ns / 1ps

module probe_udp_gen #(
    parameter [47:0] BOARD_MAC    = 48'h00_12_34_56_78_9A,
    parameter [31:0] BOARD_IP     = {8'd10, 8'd0, 8'd0, 8'd125},
    parameter [15:0] BOARD_PORT   = 16'd39153,
    parameter [11:0] STAT_TIMEOUT = 12'd4095
) (
    input  wire        clk,
    input  wire        rst_n,
    // command, captured in this domain by the top; cmd_go starts it
    input  wire        cmd_go,
    input  wire [47:0] host_mac,
    input  wire [31:0] host_ip,
    input  wire [15:0] host_port,
    input  wire [15:0] cmd_id,
    input  wire [15:0] udp_len,
    input  wire [31:0] count,
    input  wire [15:0] gap,
    // ARP request for our IP seen by the receiver
    input  wire        arp_go,
    input  wire [47:0] arp_mac,
    input  wire [31:0] arp_ip,
    // Gratuitous ARP, once a second from the top. A silent device is invisible
    // to the office switch after its link comes up: unicast to our MAC was not
    // delivered until we had sent something (board session of 13.10.2026).
    input  wire        garp_go,
    // towards the mux
    output reg         tx_valid,
    output wire [7:0]  tx_data,
    output wire        tx_last,
    input  wire        tx_ready,
    input  wire        stat_valid,
    // counters
    output reg  [31:0] n_pkt,
    output reg  [15:0] n_arp,       // ARP replies
    output reg  [15:0] n_garp,      // gratuitous ARP announcements
    output reg  [15:0] n_stat_to,
    output reg  [15:0] max_first,
    output reg  [15:0] n_underrun,  // FIFO empty in the middle of a packet - must stay 0
    output wire        streaming
);

    localparam [15:0] LEN_MIN = 16'd20;     // app header + CRC, no data
    localparam [15:0] LEN_MAX = 16'd8972;   // frame 9014 + FCS

    function [31:0] crc32_byte(input [31:0] c, input [7:0] d);
        integer i;
        reg [31:0] x;
        begin
            x = c ^ {24'd0, d};
            for (i = 0; i < 8; i = i + 1)
                x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
            crc32_byte = x;
        end
    endfunction

    // ---------------------------------------------------------------- command
    reg [47:0] c_mac;
    reg [31:0] c_ip;
    reg [15:0] c_port, c_id, c_len, c_gap;
    reg [31:0] c_count;
    reg [15:0] c_dlen, c_dlen_m2, ip_total, udp_total;     // registered from c_len
    reg        c_has_data, c_dlen_is1;
    reg [31:0] cnt_left, seq;
    reg [3:0]  settle;          // lets the derived registers fill after cmd_go
    reg        arp_pend;
    reg        garp_pend, is_garp;
    reg [47:0] a_mac;
    reg [31:0] a_ip;

    // the constant words of the IPv4 header checksum: 4500, 4000 (DF), 4011, our IP
    localparam [19:0] CS_CONST = 20'h4500 + 20'h4000 + 20'h4011
                               + {4'd0, BOARD_IP[31:16]} + {4'd0, BOARD_IP[15:0]};
    reg [16:0] cs_a;
    reg [19:0] cs_b;
    reg [19:0] cs_sum;
    reg [16:0] cs_f1;
    reg [15:0] ip_csum;
    always @(posedge clk) begin
        c_dlen    <= c_len - LEN_MIN;
        c_has_data <= (c_dlen != 16'd0);
        c_dlen_is1 <= (c_dlen == 16'd1);
        c_dlen_m2  <= c_dlen - 16'd2;
        ip_total  <= c_len + 16'd28;
        udp_total <= c_len + 16'd8;
        // two adds per stage at most (one nine-term sum closed at +0.033 ns);
        // settle (10 cycles) covers the five stages from c_len to ip_csum
        cs_a      <= {1'b0, ip_total} + {1'b0, c_ip[31:16]};
        cs_b      <= {4'd0, c_ip[15:0]} + CS_CONST;
        cs_sum    <= {3'd0, cs_a} + cs_b;
        cs_f1     <= {1'b0, cs_sum[15:0]} + {13'd0, cs_sum[19:16]};
        ip_csum   <= ~(cs_f1[15:0] + cs_f1[16]);
    end

    assign streaming = (cnt_left != 32'd0);

    // ---------------------------------------------------------------- FIFO
    reg [8:0] fifo [0:7];       // {last, byte}
    reg [2:0] wp, rp;
    reg [3:0] fcnt;
    reg       p_v, p_last, p_crc;
    reg [7:0] p_b;
    wire      rd = tx_valid && tx_ready;

    assign tx_data = fifo[rp][7:0];
    assign tx_last = fifo[rp][8];

    always @(posedge clk)
        if (p_v) fifo[wp] <= {p_last, p_b};

    // ---------------------------------------------------------------- producer
    localparam [2:0] Q_IDLE = 3'd0, Q_HDR = 3'd1, Q_DATA = 3'd2, Q_CRCW = 3'd3, Q_CRC = 3'd4, Q_DONE = 3'd5;
    reg [2:0]   q;
    reg [463:0] sr;             // header, first byte in [463:456]
    reg [5:0]   hdr_left;
    // Data bytes are counted UP from 0: loading c_dlen into a down-counter let
    // synthesis share one subtractor between the load and the decrement, and
    // the carry ran from c_dlen[0] through all 16 bits (third build, -0.153 ns).
    reg [15:0]  data_cnt;
    reg [15:0]  p_dlen_m2;      // this packet's data length - 2, taken at its start: a command
                                // arriving mid-packet must not change the packet on the wire
    reg [1:0]   crcw;
    reg [2:0]   crc_left;
    reg         crc_en, is_arp;
    reg [31:0]  crc, crc_sr;
    reg [7:0]   pat;
    reg [5:0]   hdr_no;         // header bytes issued; the CRC starts at byte 42
    reg         has_data, dl_is1;
    // Start conditions one cycle ahead, as flops (second build: +0.002 ns with
    // the 16- and 32-bit compares feeding the CRC's enable in the same cycle).
    reg         start_rdy, stream_rdy;
    wire        room = (fcnt <= 4'd5);

    // ---------------------------------------------------------------- sender
    localparam [1:0] C_IDLE = 2'd0, C_SEND = 2'd1, C_STAT = 2'd2;
    reg [1:0]  cs;
    reg [15:0] gap_cnt;
    reg [11:0] stat_cnt;
    reg [15:0] first_cnt;
    reg        first;
    reg        last_rd;         // the packet's last byte was taken in the previous cycle

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_mac <= 48'd0; c_ip <= 32'd0; c_port <= 16'd0; c_id <= 16'd0;
            c_len <= LEN_MIN; c_gap <= 16'd0; c_count <= 32'd0;
            cnt_left <= 32'd0; seq <= 32'd0; settle <= 4'd0;
            arp_pend <= 1'b0; a_mac <= 48'd0; a_ip <= 32'd0; garp_pend <= 1'b0; is_garp <= 1'b0; n_garp <= 16'd0;
            wp <= 3'd0; rp <= 3'd0; fcnt <= 4'd0;
            p_v <= 1'b0; p_last <= 1'b0; p_crc <= 1'b0; p_b <= 8'd0;
            q <= Q_IDLE; sr <= 464'd0; hdr_left <= 6'd0; data_cnt <= 16'd0; p_dlen_m2 <= 16'd0; crcw <= 2'd0;
            has_data <= 1'b0; dl_is1 <= 1'b0; start_rdy <= 1'b0; stream_rdy <= 1'b0;
            crc_left <= 3'd0; crc_en <= 1'b0; is_arp <= 1'b0; crc <= 32'hFFFFFFFF; crc_sr <= 32'd0;
            pat <= 8'd0; hdr_no <= 6'd0;
            cs <= C_IDLE; gap_cnt <= 16'd0; stat_cnt <= 12'd0; first_cnt <= 16'd0; first <= 1'b0; last_rd <= 1'b0;
            tx_valid <= 1'b0;
            n_pkt <= 32'd0; n_arp <= 16'd0; n_stat_to <= 16'd0; max_first <= 16'd0; n_underrun <= 16'd0;
        end else begin
            // ---- command and ARP request
            if (cmd_go) begin
                // a new command replaces the running stream; a packet already
                // under way finishes with the old fields
                c_mac <= host_mac; c_ip <= host_ip; c_port <= host_port; c_id <= cmd_id;
                c_len <= (udp_len < LEN_MIN) ? LEN_MIN : (udp_len > LEN_MAX) ? LEN_MAX : udp_len;
                c_gap <= gap; c_count <= count;
                cnt_left <= count; seq <= 32'd0; settle <= 4'd10;
            end else if (settle != 4'd0) begin
                settle <= settle - 4'd1;
            end
            if (arp_go) begin
                arp_pend <= 1'b1; a_mac <= arp_mac; a_ip <= arp_ip;
            end
            if (garp_go) garp_pend <= 1'b1;

            // ---- FIFO write pointer (the array is written below) and the CRC
            if (p_v) begin
                wp <= wp + 3'd1;
                if (p_crc) crc <= crc32_byte(crc, p_b);
            end
            fcnt <= fcnt + {3'd0, p_v} - {3'd0, rd};

            // ---- producer: at most one byte per cycle, only with room
            start_rdy  <= (cs == C_IDLE) && (fcnt == 4'd0) && !p_v && (q == Q_IDLE);
            stream_rdy <= (cnt_left != 32'd0) && (gap_cnt == 16'd0);
            p_v <= 1'b0;
            case (q)
            Q_IDLE: begin
                // a new packet only when the previous one is fully out
                // settle and cmd_go stay direct: a stop must win over a stale flag
                if (start_rdy && settle == 4'd0 && !cmd_go) begin
                    if (arp_pend && !arp_go) begin
                        arp_pend <= 1'b0;
                        sr       <= {a_mac, BOARD_MAC, 16'h0806, 16'h0001, 16'h0800, 8'h06, 8'h04,
                                     16'h0002, BOARD_MAC, BOARD_IP, a_mac, a_ip, 128'd0};
                        hdr_left <= 6'd42; data_cnt <= 16'd0; crc_en <= 1'b0; is_arp <= 1'b1; is_garp <= 1'b0;
                        has_data <= 1'b0; dl_is1 <= 1'b0;
                        hdr_no <= 6'd0; crc <= 32'hFFFFFFFF; q <= Q_HDR;
                    end else if (garp_pend && !garp_go) begin
                        // gratuitous ARP request: who-has BOARD_IP, from BOARD_IP
                        garp_pend <= 1'b0;
                        sr       <= {48'hFFFF_FFFF_FFFF, BOARD_MAC, 16'h0806, 16'h0001, 16'h0800, 8'h06, 8'h04,
                                     16'h0001, BOARD_MAC, BOARD_IP, 48'd0, BOARD_IP, 128'd0};
                        hdr_left <= 6'd42; data_cnt <= 16'd0; crc_en <= 1'b0; is_arp <= 1'b1; is_garp <= 1'b1;
                        has_data <= 1'b0; dl_is1 <= 1'b0;
                        hdr_no <= 6'd0; crc <= 32'hFFFFFFFF; q <= Q_HDR;
                    end else if (stream_rdy) begin
                        sr       <= {c_mac, BOARD_MAC, 16'h0800,
                                     8'h45, 8'h00, ip_total, 16'h0000, 16'h4000, 8'h40, 8'h11, ip_csum,
                                     BOARD_IP, c_ip,
                                     BOARD_PORT, c_port, udp_total, 16'h0000,
                                     32'h55505342, c_id, c_len, seq, c_count};
                        hdr_left <= 6'd58; data_cnt <= 16'd0; p_dlen_m2 <= c_dlen_m2; crc_en <= 1'b1; is_arp <= 1'b0; is_garp <= 1'b0;
                        has_data <= c_has_data; dl_is1 <= c_dlen_is1;
                        pat <= seq[7:0]; hdr_no <= 6'd0; crc <= 32'hFFFFFFFF; q <= Q_HDR;
                    end
                end
            end
            Q_HDR: if (room) begin
                p_v <= 1'b1; p_b <= sr[463:456]; p_crc <= crc_en && (hdr_no >= 6'd42);
                sr <= {sr[455:0], 8'd0};
                hdr_no <= hdr_no + 6'd1;
                hdr_left <= hdr_left - 6'd1;
                p_last <= (hdr_left == 6'd1) && !crc_en;
                if (hdr_left == 6'd1)
                    q <= !crc_en ? Q_DONE : has_data ? Q_DATA : Q_CRCW;
                crcw <= 2'd2;
            end
            Q_DATA: if (room) begin
                p_v <= 1'b1; p_b <= pat; p_crc <= 1'b1; p_last <= 1'b0;
                pat <= pat + 8'd1;
                data_cnt  <= data_cnt + 16'd1;
                dl_is1    <= (data_cnt == p_dlen_m2);     // the next byte is the last
                if (dl_is1) begin q <= Q_CRCW; crcw <= 2'd2; end
            end
            Q_CRCW: begin
                // the last CRC byte is written and folded in this cycle or the
                // previous one; crc is final one cycle after that
                crcw <= crcw - 2'd1;
                if (crcw == 2'd1) begin crc_sr <= ~crc; crc_left <= 3'd4; q <= Q_CRC; end
            end
            Q_CRC: if (room) begin
                p_v <= 1'b1; p_b <= crc_sr[31:24]; p_crc <= 1'b0; p_last <= (crc_left == 3'd1);
                crc_sr <= {crc_sr[23:0], 8'd0};
                crc_left <= crc_left - 3'd1;
                if (crc_left == 3'd1) q <= Q_DONE;
            end
            default: if (cs == C_STAT) q <= Q_IDLE;     // Q_DONE: wait for the sender
            endcase

            // ---- sender
            last_rd <= rd && tx_last;
            case (cs)
            C_IDLE: begin
                if (gap_cnt != 16'd0 && !cmd_go) gap_cnt <= gap_cnt - 16'd1;
                if (fcnt >= 4'd6 || (q == Q_DONE && fcnt != 4'd0 && !p_v)) begin
                    tx_valid <= 1'b1; cs <= C_SEND; first <= 1'b1; first_cnt <= 16'd0;
                end
            end
            C_SEND: begin
                if (first && !rd && first_cnt != 16'hFFFF) first_cnt <= first_cnt + 16'd1;
                if (fcnt == 4'd0 && !last_rd && n_underrun != 16'hFFFF) n_underrun <= n_underrun + 16'd1;
                if (rd) begin
                    if (first && first_cnt > max_first) max_first <= first_cnt;
                    first <= 1'b0;
                    rp <= rp + 3'd1;
                end
                // One cycle after the last byte, from a flop (sixth build: ready and
                // the FIFO's last flag straight into this state's enable, +0.021 ns).
                // tx_valid lingers that cycle with nothing behind it; the mux has
                // already moved to waiting for statistics and keeps ready low.
                if (last_rd) begin
                    tx_valid <= 1'b0; cs <= C_STAT; stat_cnt <= 12'd0;
                end
            end
            C_STAT: begin
                stat_cnt <= stat_cnt + 12'd1;
                if (stat_valid || stat_cnt == STAT_TIMEOUT) begin
                    if (!stat_valid) n_stat_to <= n_stat_to + 16'd1;
                    if (is_garp) begin
                        n_garp <= n_garp + 16'd1;
                    end else if (is_arp) begin
                        n_arp <= n_arp + 16'd1;
                    end else begin
                        n_pkt <= n_pkt + 32'd1;
                        if (!cmd_go) begin      // a new command has just reset these
                            seq     <= seq + 32'd1;
                            gap_cnt <= c_gap;
                            if (cnt_left != 32'd0) cnt_left <= cnt_left - 32'd1;
                        end
                    end
                    cs <= C_IDLE;
                end
            end
            default: cs <= C_IDLE;
            endcase
        end
    end

endmodule
