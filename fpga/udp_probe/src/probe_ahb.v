// UDP probe: a tiny AHB master that plays the CPU for Gowin's encrypted MAC
// wrapper (gw_ahb_ethernet), so the probe tests the wrapper itself sharing the
// MAC with a second source - the open question the probe answered (ERRORS 3.12).
//
// It does what the firmware does, register for register (firmware/src/app/
// ethernet.h, ethernet.c, arch/ethernetif.c), with single 32-bit transfers:
//   init      ETH_MODE = 1 (1000M full duplex: drives speedis1000 of the MAC),
//             TX/RX interrupt enables off;
//   PHY poll  once a second, MIIM read of RTL8211F PHYSR (reg 0x1A, PHY 0):
//             [5:4] speed, [3] full duplex, [2] link;
//   heartbeat every hb_us microseconds when enabled: wait ETH_TX_EN == 0,
//             clear TX/RX interrupt status, ETH_TX_LENGTH = 64, 16 words into
//             the window (byte 0 in bits 7:0, as the little-endian CPU writes
//             them), ETH_TX_EN = 1 - a 64-byte UDP datagram to the host,
//             payload 'UPHB', sequence, PHYSR, TX_EN timeouts.
// A poll that runs out (TX_EN never clears, MIIM never answers) is counted and
// the routine is abandoned.
`timescale 1ns / 1ps

module probe_ahb #(
    parameter [47:0] BOARD_MAC  = 48'h00_12_34_56_78_9A,
    parameter [31:0] BOARD_IP   = {8'd10, 8'd0, 8'd0, 8'd125},
    parameter [15:0] BOARD_PORT = 16'd39153,
    parameter [31:0] BASE       = 32'hE800_0000,
    parameter integer CLK_HZ    = 50_000_000,
    parameter [15:0] POLL_MAX   = 16'd50000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,           // level: MAC and wrapper out of reset
    // AHB master
    output reg  [31:0] haddr,
    output reg  [1:0]  htrans,
    output reg         hwrite,
    output wire [2:0]  hsize,
    output wire [2:0]  hburst,
    output reg  [31:0] hwdata,
    output reg         hsel,
    input  wire        hready,
    input  wire [31:0] hrdata,
    // heartbeat destination and period, captured in this domain by the top
    input  wire        cfg_go,
    input  wire [47:0] host_mac,
    input  wire [31:0] host_ip,
    input  wire [15:0] host_port,
    input  wire        hb_en,
    input  wire [15:0] hb_us,
    input  wire        phy_poll,        // one-cycle request, once a second
    // results
    output reg  [15:0] physr,
    output reg  [15:0] n_miim_fail,
    output reg  [31:0] n_hb,
    output reg  [15:0] n_txen_to,
    output reg         init_done
);

    assign hsize  = 3'b010;
    assign hburst = 3'b000;

    // ------------------------------------------------------------ heartbeat
    reg [47:0] h_mac;
    reg [31:0] h_ip;
    reg [15:0] h_port, h_us;
    reg        h_en;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_mac <= 48'd0; h_ip <= 32'd0; h_port <= 16'd0; h_us <= 16'd0; h_en <= 1'b0;
        end else if (cfg_go) begin
            h_mac <= host_mac; h_ip <= host_ip; h_port <= host_port; h_us <= hb_us;
            h_en <= hb_en && (hb_us != 16'd0);
        end
    end

    reg [19:0] cs_sum;
    reg [16:0] cs_f1;
    reg [15:0] hb_csum;
    always @(posedge clk) begin
        cs_sum  <= 20'h4500 + 20'h0032 + 20'h4000 + 20'h4011
                 + {4'd0, BOARD_IP[31:16]} + {4'd0, BOARD_IP[15:0]} + {4'd0, h_ip[31:16]} + {4'd0, h_ip[15:0]};
        cs_f1   <= {1'b0, cs_sum[15:0]} + {13'd0, cs_sum[19:16]};
        hb_csum <= ~(cs_f1[15:0] + cs_f1[16]);
    end

    wire [511:0] hb_pkt = {h_mac, BOARD_MAC, 16'h0800,
                           8'h45, 8'h00, 16'd50, 16'h0000, 16'h4000, 8'h40, 8'h11, hb_csum,
                           BOARD_IP, h_ip,
                           BOARD_PORT, h_port, 16'd30, 16'h0000,
                           32'h55504842, n_hb, physr, n_txen_to, 80'd0};

    // Functions take what they read as arguments (see probe_rx.v).
    function [31:0] hb_word(input [511:0] pkt, input [3:0] k);
        reg [31:0] be;
        begin
            be = pkt[511 - 32 * k -: 32];                // bytes 4k .. 4k+3
            hb_word = {be[7:0], be[15:8], be[23:16], be[31:24]};
        end
    endfunction

    // ------------------------------------------------------------ routines
    localparam [1:0] K_W = 2'd0, K_R = 2'd1, K_P0 = 2'd2, K_P1 = 2'd3;  // write, read, poll bit0 == 0 / == 1
    localparam [1:0] R_INIT = 2'd0, R_MIIM = 2'd1, R_HB = 2'd2;

    // one step: {last, kind, offset[11:0], data}
    function [46:0] step(input [1:0] r, input [4:0] s, input [511:0] pkt, input [3:0] mreq);
        begin
            step = {1'b1, K_W, 12'h000, 32'd0};
            case (r)
            R_INIT: case (s)
                5'd0: step = {1'b0, K_W, 12'h724, 32'd1};       // ETH_MODE: 1000M FD
                5'd1: step = {1'b0, K_W, 12'h614, 32'd0};       // ETH_TX_IE
                default: step = {1'b1, K_W, 12'h688, 32'd0};    // ETH_RX_IE
            endcase
            R_MIIM: case (s)
                5'd0: step = {1'b0, K_W, 12'h71C, 32'd3};       // MIIM_IC
                5'd1: step = {1'b0, K_W, 12'h718, 32'd0};       // MIIM_IE
                5'd2: step = {1'b0, K_W, 12'h700, 32'd0};       // MIIM_OP_MODE: read
                5'd3: step = {1'b0, K_W, 12'h704, 32'd0};       // MIIM_PHY_ADDR
                5'd4: step = {1'b0, K_W, 12'h708, 32'h1A};      // MIIM_REG_ADDR: PHYSR
                5'd5: step = {1'b0, K_W, 12'h720, 32'd1};       // MIIM_OP_EN
                5'd6: step = {1'b0, K_P1, 12'h714, 32'd0};      // MIIM_IS bit 0: read data valid
                5'd7: step = {1'b0, K_R, 12'h710, 32'd0};       // MIIM_RD_DATA
                5'd8: step = {1'b0, K_W, 12'h71C, 32'd3};
                // follow the link: ETH_MODE as the firmware sets it, only on a change
                default: step = mreq[3] ? {1'b1, K_W, 12'h724, {29'd0, mreq[2:0]}}
                                        : {1'b1, K_W, 12'h71C, 32'd3};
            endcase
            R_HB: case (s)
                5'd0:  step = {1'b0, K_P0, 12'h604, 32'd0};     // ETH_TX_EN clear
                5'd1:  step = {1'b0, K_W, 12'h610, 32'd1};      // ETH_TX_IC
                5'd2:  step = {1'b0, K_W, 12'h68C, 32'd1};      // ETH_RX_IC
                5'd3:  step = {1'b0, K_W, 12'h600, 32'd64};     // ETH_TX_LENGTH
                5'd20: step = {1'b1, K_W, 12'h604, 32'd1};      // ETH_TX_EN
                default: step = {1'b0, K_W, {5'd0, s - 5'd4, 2'b00}, hb_word(pkt, s[3:0] - 4'd4)};
            endcase
            default: ;
            endcase
        end
    endfunction

    // ------------------------------------------------------------ scheduler
    localparam integer US_DIV = CLK_HZ / 1_000_000;
    localparam integer US_DIV_M1_I = US_DIV - 1;
    localparam [7:0]   US_DIV_M1 = US_DIV_M1_I[7:0];
    reg [7:0]  us_div;
    reg [15:0] us_cnt;
    reg        hb_due, miim_due;

    localparam [2:0] B_IDLE = 3'd0, B_ADDR = 3'd1, B_DATA = 3'd2, B_NEXT = 3'd3;
    reg [2:0]  bst;
    reg        busy;
    reg [1:0]  rt;
    reg [4:0]  si;
    reg [46:0] cur;
    reg [15:0] polls;

    wire       c_last = cur[46];
    wire [1:0] c_kind = cur[45:44];
    // ETH_MODE for the speed PHYSR reports (firmware/src/app/ethernet.h):
    // 1 = 1000M FD, 0 = 100M FD, 4 = 100M HD, 2 = 10M FD, 6 = 10M HD. The probe
    // forced 1000M until a direct cable to the PC came up at 100M (13.10.2026).
    reg  [2:0]  mode_cur;
    wire [2:0]  mode_want = (physr[5:4] == 2'd2) ? 3'd1 :
                            (physr[5:4] == 2'd1) ? (physr[3] ? 3'd0 : 3'd4) : (physr[3] ? 3'd2 : 3'd6);
    wire        mode_chg  = (physr != 16'hFFFF) && physr[2] && (mode_want != mode_cur);
    wire [46:0] nxt   = step(rt, si, hb_pkt, {mode_chg, mode_want});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            us_div <= 8'd0; us_cnt <= 16'd0; hb_due <= 1'b0; miim_due <= 1'b0;
            bst <= B_IDLE; busy <= 1'b0; rt <= R_INIT; si <= 5'd0; cur <= 47'd0; polls <= 16'd0;
            haddr <= 32'd0; htrans <= 2'b00; hwrite <= 1'b0; hwdata <= 32'd0; hsel <= 1'b0;
            physr <= 16'hFFFF; n_miim_fail <= 16'd0; n_hb <= 32'd0; n_txen_to <= 16'd0;
            init_done <= 1'b0; mode_cur <= 3'd1;
        end else begin
            // heartbeat timer, microseconds
            if (us_div == US_DIV_M1) begin
                us_div <= 8'd0;
                if (h_en && us_cnt + 16'd1 >= h_us) begin us_cnt <= 16'd0; hb_due <= 1'b1; end
                else us_cnt <= us_cnt + 16'd1;
            end else begin
                us_div <= us_div + 8'd1;
            end
            if (!h_en) hb_due <= 1'b0;
            if (phy_poll) miim_due <= 1'b1;

            case (bst)
            B_IDLE: begin
                hsel <= 1'b0; htrans <= 2'b00;
                if (start && !busy) begin
                    polls <= 16'd0;
                    if (!init_done)    begin busy <= 1'b1; rt <= R_INIT; si <= 5'd0; end
                    else if (miim_due) begin busy <= 1'b1; rt <= R_MIIM; si <= 5'd0; miim_due <= 1'b0; end
                    else if (hb_due)   begin busy <= 1'b1; rt <= R_HB;   si <= 5'd0; hb_due <= 1'b0; end
                end else if (busy) begin
                    // issue the step: address phase
                    cur    <= nxt;
                    haddr  <= BASE | {20'd0, nxt[43:32]};
                    hwrite <= (nxt[45:44] == K_W);
                    htrans <= 2'b10;                    // NONSEQ
                    hsel   <= 1'b1;
                    bst    <= B_ADDR;
                end
            end
            B_ADDR: if (hready) begin                   // address accepted, data phase next
                htrans <= 2'b00; hsel <= 1'b0;
                hwdata <= cur[31:0];
                bst    <= B_DATA;
            end
            B_DATA: if (hready) begin                   // transfer complete
                bst <= B_NEXT;
                case (c_kind)
                K_R: if (rt == R_MIIM) physr <= hrdata[15:0];
                K_W: if (cur[43:32] == 12'h724) mode_cur <= cur[2:0];
                K_P0, K_P1: begin
                    if (hrdata[0] == (c_kind == K_P1)) begin
                        // condition met: on to the next step
                    end else if (polls == POLL_MAX) begin
                        // gave up: count it and abandon the routine
                        if (rt == R_MIIM) begin n_miim_fail <= n_miim_fail + 16'd1; physr <= 16'hFFFF; end
                        else              n_txen_to <= n_txen_to + 16'd1;
                        busy <= 1'b0; bst <= B_IDLE;
                    end else begin
                        polls <= polls + 16'd1;
                        bst   <= B_IDLE;                // same step again
                    end
                end
                default: ;
                endcase
            end
            B_NEXT: begin
                if (c_last) begin
                    busy <= 1'b0;
                    if (rt == R_INIT) init_done <= 1'b1;
                    if (rt == R_HB)   n_hb <= n_hb + 32'd1;
                end else begin
                    si <= si + 5'd1;
                end
                polls <= 16'd0;
                bst <= B_IDLE;
            end
            default: bst <= B_IDLE;
            endcase
        end
    end

endmodule
