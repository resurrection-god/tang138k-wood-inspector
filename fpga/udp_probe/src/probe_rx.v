// UDP probe receiver: answers nothing itself, only recognises two kinds of
// frame on the MAC's client receive port (rx_mac_clk domain):
//
//   * an ARP request for BOARD_IP -> arp_tgl toggles with the sender's MAC/IP
//     (the generator replies; without it the PC never sends us anything);
//   * a UDP datagram to BOARD_IP:BOARD_PORT whose payload is a command
//     (18 bytes, big-endian, so the frame is exactly the 60-byte minimum):
//         +0  'UPRC'          magic 0x55505243
//         +4  cmd_id          echoed in every stream packet
//         +6  udp_len         UDP payload of each stream packet, 20..8972
//         +8  count           packets to send, 0 stops the stream
//         +12 gap             extra idle tx_mac_clk cycles after each packet
//         +14 ifg             0: MAC default 12 bytes; else tx_ifg_delay (>= 8)
//         +15 flags           bit 0: heartbeat through the wrapper on
//         +16 hb_us           heartbeat period, microseconds
//     -> cmd_tgl toggles; the sender's MAC, IP and port become the stream's
//        destination.
//
// The fields are held stable until the next toggle; the consumer domains take
// them a few of their own cycles after seeing the synchronised toggle.
`timescale 1ns / 1ps

module probe_rx #(
    parameter [47:0] BOARD_MAC  = 48'h00_12_34_56_78_9A,
    parameter [31:0] BOARD_IP   = {8'd10, 8'd0, 8'd0, 8'd125},
    parameter [15:0] BOARD_PORT = 16'd39153
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    input  wire        rx_last,
    input  wire        rx_error,
    // command
    output reg         cmd_tgl,
    output reg  [47:0] c_mac,
    output reg  [31:0] c_ip,
    output reg  [15:0] c_port,
    output reg  [15:0] c_id,
    output reg  [15:0] c_len,
    output reg  [31:0] c_count,
    output reg  [15:0] c_gap,
    output reg  [7:0]  c_ifg,
    output reg  [7:0]  c_flags,
    output reg  [15:0] c_hb_us,
    // ARP request
    output reg         arp_tgl,
    output reg  [47:0] a_mac,
    output reg  [31:0] a_ip,
    // counters
    output reg  [31:0] n_rx,
    output reg  [15:0] n_err,
    output reg  [15:0] n_arp,
    output reg  [15:0] n_cmd
);

    reg [479:0] buf_;           // first 60 bytes of the frame, byte 0 in [479:472]
    reg [10:0]  bi;
    reg         done;           // frame ended cleanly with >= 60 bytes: decode next cycle

    // Field extractors take the buffer as an argument: a function that reads a
    // module variable directly is not re-evaluated in a continuous assignment
    // when that variable changes (IEEE 1364 sensitivity is the argument list),
    // so the simulator would decode stale bytes while synthesis would not.
    function [7:0] B(input [479:0] v, input integer i);
        B = v[479 - 8 * i -: 8];
    endfunction
    function [15:0] H(input [479:0] v, input integer i);
        H = {B(v, i), B(v, i + 1)};
    endfunction
    function [31:0] W(input [479:0] v, input integer i);
        W = {H(v, i), H(v, i + 2)};
    endfunction
    function [47:0] M(input [479:0] v, input integer i);
        M = {H(v, i), W(v, i + 2)};
    endfunction

    // Decided in two register steps after the frame ends (one step closed at
    // 109 MHz of 125): the field compares, then their AND. The buffer holds
    // still until the next frame's first byte, preamble + gap away.
    reg        k_v;
    reg        k_dst_us, k_dst_bc, k_et_arp, k_arp_req, k_tpa, k_et_ip, k_ip_hdr, k_dip, k_dport, k_magic;
    wire       is_arp = (k_dst_us || k_dst_bc) && k_et_arp && k_arp_req && k_tpa;
    wire       is_cmd = k_dst_us && k_et_ip && k_ip_hdr && k_dip && k_dport && k_magic;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_ <= 480'd0; bi <= 11'd0; done <= 1'b0;
            cmd_tgl <= 1'b0; c_mac <= 48'd0; c_ip <= 32'd0; c_port <= 16'd0; c_id <= 16'd0;
            c_len <= 16'd0; c_count <= 32'd0; c_gap <= 16'd0; c_ifg <= 8'd0; c_flags <= 8'd0;
            c_hb_us <= 16'd0;
            arp_tgl <= 1'b0; a_mac <= 48'd0; a_ip <= 32'd0;
            n_rx <= 32'd0; n_err <= 16'd0; n_arp <= 16'd0; n_cmd <= 16'd0;
            k_v <= 1'b0;
        end else begin
            done <= 1'b0;
            if (rx_valid) begin
                if (bi < 11'd60) buf_ <= {buf_[471:0], rx_data};
                if (rx_last) begin
                    bi   <= 11'd0;
                    n_rx <= n_rx + 32'd1;
                    if (rx_error) n_err <= n_err + 16'd1;
                    // byte 59 arrives with rx_last on a minimum frame: bi is 59 then
                    done <= !rx_error && (bi >= 11'd59);
                end else if (bi != 11'h7FF) begin
                    bi <= bi + 11'd1;
                end
            end
            k_v <= done;
            if (done) begin
                k_dst_us  <= (M(buf_, 0) == BOARD_MAC);
                k_dst_bc  <= (M(buf_, 0) == 48'hFFFF_FFFF_FFFF);
                k_et_arp  <= (H(buf_, 12) == 16'h0806);
                k_arp_req <= ({H(buf_, 14), H(buf_, 16), H(buf_, 20)} == 48'h0001_0800_0001);
                k_tpa     <= (W(buf_, 38) == BOARD_IP);
                k_et_ip   <= (H(buf_, 12) == 16'h0800);
                k_ip_hdr  <= (B(buf_, 14) == 8'h45) && (B(buf_, 23) == 8'h11);
                k_dip     <= (W(buf_, 30) == BOARD_IP);
                k_dport   <= (H(buf_, 36) == BOARD_PORT);
                k_magic   <= (W(buf_, 42) == 32'h55505243);
            end
            if (k_v) begin
                if (is_arp) begin
                    arp_tgl <= ~arp_tgl; a_mac <= M(buf_, 22); a_ip <= W(buf_, 28);
                    n_arp <= n_arp + 16'd1;
                end else if (is_cmd) begin
                    cmd_tgl <= ~cmd_tgl;
                    c_mac <= M(buf_, 6); c_ip <= W(buf_, 26); c_port <= H(buf_, 34);
                    c_id <= H(buf_, 46); c_len <= H(buf_, 48); c_count <= W(buf_, 50); c_gap <= H(buf_, 54);
                    c_ifg <= B(buf_, 56); c_flags <= B(buf_, 57); c_hb_us <= H(buf_, 58);
                    n_cmd <= n_cmd + 16'd1;
                end
            end
        end
    end

endmodule
