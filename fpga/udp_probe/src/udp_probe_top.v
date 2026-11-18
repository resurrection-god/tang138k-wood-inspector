// udp_probe: stage 1 of the UDP frame stream from the FPGA (ERRORS 3.12;
// fpga/udp_probe/README.md).
//
// No SoC, no DDR3. The same Gowin MAC and the same encrypted AHB wrapper as
// fpga/src/scanner_top.v, the same pins and clocks; the wrapper is driven by a
// small AHB master instead of the CPU (probe_ahb), and the MAC's client
// transmit port is shared, packet by packet, between the wrapper and a UDP
// generator (probe_tx_mux) - the arrangement the frame stream will need.
//
// The PC drives everything with one UDP datagram (probe_rx): the stream goes
// to whoever sent it. Counters go out on UART2 (COM7, 115200 8N1) once a
// second in fps_probe's frame format: 0x5A 0xA5, NW words big-endian, a byte
// sum. Host side: tools/net/udp_rx_bench.py. SRAM only (--run 2).
//
// Domains: clk50 (CLK pin: resets, AHB master, report), tx_mac_clk (= GTX_CLK
// 125 MHz: generator, mux), rx_mac_clk (RGMII_RXC: receiver). Multi-bit values
// cross only while held still, announced by a synchronised toggle.
`timescale 1ns / 1ps

module udp_probe_top #(
    // The probe's IPv4 address, i.e. the network the PC reaches it on:
    //   192.168.31.125 - a direct cable to the PC's Ethernet (192.168.31.10/24),
    //                    the build in use since 13.10.2026, session 4;
    //   10.0.0.125     - the office LAN, the board's own address (board.h);
    //                    sessions 1-3, build 52CFF213; the bench passes it.
    // A new value is a new build - and a new placement roll (README, "Timing").
    parameter [31:0] BOARD_IP = {8'd192, 8'd168, 8'd31, 8'd125}
) (
    input  wire       CLK,          // 50 MHz, P16
    input  wire       RSTN,         // button S3, active low
    output wire [4:0] LED,
    output wire       UART2_TXD,
    input  wire       UART2_RXD,
    output wire       PHY_CLK,
    output wire       PHY_RST_N,
    output wire       RGMII_TXC,
    output wire       RGMII_TX_CTL,
    output wire [3:0] RGMII_TXD,
    input  wire       RGMII_RXC,
    input  wire       RGMII_RX_CTL,
    input  wire [3:0] RGMII_RXD,
    output wire       MDC,
    inout  wire       MDIO
);

    localparam integer CLK_HZ     = 50_000_000;
    localparam [47:0]  BOARD_MAC  = 48'h00_12_34_56_78_9A;    // firmware/src/app/board.h
    localparam [15:0]  BOARD_PORT = 16'd39153;

`ifdef UDP_PROBE_SIM
    localparam [24:0] T_PHY_RST = 25'd200;           // clk50 cycles
    localparam [24:0] T_ETH_RST = 25'd400;
    localparam [25:0] T_SEC     = 26'd5_000;
`else
    localparam [24:0] T_PHY_RST = 25'd1_250_000;     // 25 ms, as power_on_reset_sequencer
    localparam [24:0] T_ETH_RST = 25'd1_750_000;     // MAC and wrapper 10 ms after the PHY
    localparam [25:0] T_SEC     = 26'd50_000_000;
`endif

    // ------------------------------------------------------------ clocks
    wire gtx_clk, pll_lock;
    Gowin_PLL_ETH u_pll (
        .lock(pll_lock), .clkout0(gtx_clk), .clkout1(), .clkout2(), .clkout3(), .clkout4(),
        .clkin(CLK), .reset(1'b0), .enclk0(1'b1), .enclk1(1'b1), .enclk2(1'b1), .enclk3(1'b1));

    // PHY_CLK 25 MHz = GTX_CLK / 5 through an ODDR, exactly as scanner_top.v
    reg [2:0] phy_clk_cnt = 3'd0;
    always @(posedge gtx_clk)
        phy_clk_cnt <= (phy_clk_cnt == 3'd4) ? 3'd0 : (phy_clk_cnt + 3'd1);
    ODDR u_phy_clk_oddr (
        .Q0(PHY_CLK), .Q1(), .D0(phy_clk_cnt <= 3'd2), .D1(phy_clk_cnt <= 3'd1), .TX(1'b0), .CLK(gtx_clk));

    // ------------------------------------------------------------ resets (clk50)
    reg [2:0]  btn_s = 3'b000;
    reg [7:0]  por = 8'd0;
    always @(posedge CLK) begin
        btn_s <= {btn_s[1:0], RSTN};
        if (!btn_s[2]) por <= 8'd0;
        else if (por != 8'hFF) por <= por + 8'd1;
    end
    wire rst50_n = (por == 8'hFF);

    reg [2:0]  lock_s;
    reg [24:0] rst_t;
    reg        phy_rst_n_r, eth_rstn;
    always @(posedge CLK or negedge rst50_n) begin
        if (!rst50_n) begin
            lock_s <= 3'd0; rst_t <= 25'd0; phy_rst_n_r <= 1'b0; eth_rstn <= 1'b0;
        end else begin
            lock_s <= {lock_s[1:0], pll_lock};
            if (!lock_s[2]) begin
                rst_t <= 25'd0; phy_rst_n_r <= 1'b0; eth_rstn <= 1'b0;
            end else begin
                if (rst_t != T_ETH_RST) rst_t <= rst_t + 25'd1;
                if (rst_t >= T_PHY_RST) phy_rst_n_r <= 1'b1;
                if (rst_t == T_ETH_RST) eth_rstn <= 1'b1;
            end
        end
    end
    assign PHY_RST_N = phy_rst_n_r;

    // ------------------------------------------------------------ MAC + wrapper
    wire        tx_mac_clk, rx_mac_clk;
    wire        speedis1000, speedis10, duplex_status;
    wire        m_valid, m_last, m_error, m_ready;
    wire [7:0]  m_data;
    wire        tx_collision, tx_retransmit, tx_stat_valid;
    wire [28:0] tx_stat_vec;
    wire        tx_pause_req;
    wire [15:0] tx_pause_val;
    wire [47:0] tx_pause_src;
    wire        rx_valid, rx_last, rx_error, rx_stat_valid, rx_pause_req;
    wire [7:0]  rx_data;
    wire [26:0] rx_stat_vec;
    wire [15:0] rx_pause_val;
    wire [4:0]  miim_phyad, miim_regad;
    wire [15:0] miim_wrdata, miim_rddata;
    wire        miim_wren, miim_rden, miim_rddata_valid, miim_busy;
    wire        mdio_in, mdio_out, mdio_oen;

    reg         ifg_ena;
    reg  [7:0]  ifg_val;

    assign mdio_in = MDIO;
    assign MDIO    = (!mdio_oen) ? mdio_out : 1'bz;

    Triple_Speed_Ethernet_MAC_Top u_mac (
        .rgmii_rxc(RGMII_RXC), .rgmii_rx_ctl(RGMII_RX_CTL), .rgmii_rxd(RGMII_RXD), .gtx_clk(gtx_clk),
        .rgmii_txc(RGMII_TXC), .rgmii_tx_ctl(RGMII_TX_CTL), .rgmii_txd(RGMII_TXD),
        .speedis1000(speedis1000), .speedis10(speedis10), .duplex_status(duplex_status), .rstn(eth_rstn),
        .rx_mac_clk(rx_mac_clk), .rx_mac_valid(rx_valid), .rx_mac_data(rx_data), .rx_mac_last(rx_last),
        .rx_mac_error(rx_error), .rx_statistics_valid(rx_stat_valid), .rx_statistics_vector(rx_stat_vec),
        .tx_mac_clk(tx_mac_clk), .tx_mac_valid(m_valid), .tx_mac_data(m_data), .tx_mac_last(m_last),
        .tx_mac_error(m_error), .tx_mac_ready(m_ready), .tx_collision(tx_collision),
        .tx_retransmit(tx_retransmit), .tx_statistics_valid(tx_stat_valid),
        .tx_statistics_vector(tx_stat_vec),
        .rx_fcs_fwd_ena(1'b0), .rx_jumbo_ena(1'b0), .rx_pause_req(rx_pause_req), .rx_pause_val(rx_pause_val),
        .tx_fcs_fwd_ena(1'b0), .tx_ifg_delay_ena(ifg_ena), .tx_ifg_delay(ifg_val),
        .tx_pause_req(tx_pause_req), .tx_pause_val(tx_pause_val), .tx_pause_source_addr(tx_pause_src),
        .clk(CLK), .miim_phyad(miim_phyad), .miim_regad(miim_regad), .miim_wrdata(miim_wrdata),
        .miim_wren(miim_wren), .miim_rden(miim_rden), .miim_rddata(miim_rddata),
        .miim_rddata_valid(miim_rddata_valid), .miim_busy(miim_busy),
        .mdc(MDC), .mdio_in(mdio_in), .mdio_out(mdio_out), .mdio_oen(mdio_oen));

    // the wrapper's side of the mux
    wire        a_valid, a_last, a_error, a_ready, a_stat;
    wire [7:0]  a_data;
    wire [31:0] haddr, hwdata, hrdata;
    wire [1:0]  htrans, hresp;
    wire [2:0]  hsize, hburst;
    wire        hwrite, hsel, hready, eth_int;

    gw_ahb_ethernet_top u_wrap (
        .rstn(eth_rstn),
        .tx_mac_clk(tx_mac_clk), .tx_mac_valid(a_valid), .tx_mac_data(a_data), .tx_mac_last(a_last),
        .tx_mac_error(a_error), .tx_mac_ready(a_ready), .tx_collision(tx_collision),
        .tx_retransmit(tx_retransmit), .tx_pause_req(tx_pause_req), .tx_pause_val(tx_pause_val),
        .tx_pause_source_addr(tx_pause_src), .tx_statistics_valid(a_stat),
        .tx_statistics_vector(tx_stat_vec),
        .rx_mac_clk(rx_mac_clk), .rx_mac_valid(rx_valid), .rx_mac_data(rx_data), .rx_mac_last(rx_last),
        .rx_mac_error(rx_error), .rx_statistics_valid(rx_stat_valid), .rx_statistics_vector(rx_stat_vec),
        .rx_pause_req(rx_pause_req), .rx_pause_val(rx_pause_val),
        .clk(CLK), .miim_phyad(miim_phyad), .miim_regad(miim_regad), .miim_wrdata(miim_wrdata),
        .miim_wren(miim_wren), .miim_rden(miim_rden), .miim_rddata(miim_rddata),
        .miim_rddata_valid(miim_rddata_valid), .miim_busy(miim_busy),
        .hclk(CLK), .hresetn(eth_rstn), .haddr(haddr), .htrans(htrans), .hwrite(hwrite), .hsize(hsize),
        .hburst(hburst), .hwdata(hwdata), .hsel(hsel), .hreadyin(hready), .hrdata(hrdata), .hresp(hresp),
        .hready(hready), .eth_int(eth_int),
        .speedis1000(speedis1000), .speedis10(speedis10), .duplex_status(duplex_status));

    // ------------------------------------------------------------ rx_mac_clk
    reg [1:0] rx_rst_s;
    always @(posedge rx_mac_clk or negedge eth_rstn)
        if (!eth_rstn) rx_rst_s <= 2'b00; else rx_rst_s <= {rx_rst_s[0], 1'b1};
    wire rx_rst_n = rx_rst_s[1];

    wire        cmd_tgl, arp_tgl;
    wire [47:0] r_mac, r_amac;
    wire [31:0] r_ip, r_aip, r_count;
    wire [15:0] r_port, r_id, r_len, r_gap, r_hb_us;
    wire [7:0]  r_ifg, r_flags;
    wire [31:0] r_n_rx;
    wire [15:0] r_n_err, r_n_arp, r_n_cmd;

    probe_rx #(.BOARD_MAC(BOARD_MAC), .BOARD_IP(BOARD_IP), .BOARD_PORT(BOARD_PORT)) u_rx (
        .clk(rx_mac_clk), .rst_n(rx_rst_n),
        .rx_valid(rx_valid), .rx_data(rx_data), .rx_last(rx_last), .rx_error(rx_error),
        .cmd_tgl(cmd_tgl), .c_mac(r_mac), .c_ip(r_ip), .c_port(r_port), .c_id(r_id), .c_len(r_len),
        .c_count(r_count), .c_gap(r_gap), .c_ifg(r_ifg), .c_flags(r_flags), .c_hb_us(r_hb_us),
        .arp_tgl(arp_tgl), .a_mac(r_amac), .a_ip(r_aip),
        .n_rx(r_n_rx), .n_err(r_n_err), .n_arp(r_n_arp), .n_cmd(r_n_cmd));

    // ------------------------------------------------------------ tx_mac_clk
    reg [1:0] tx_rst_s;
    always @(posedge tx_mac_clk or negedge eth_rstn)
        if (!eth_rstn) tx_rst_s <= 2'b00; else tx_rst_s <= {tx_rst_s[0], 1'b1};
    wire tx_rst_n = tx_rst_s[1];

    // command and ARP from the receiver: toggle -> 3 flops -> capture one cycle
    // after the edge -> go one cycle after the capture
    reg [2:0]  t_cmd_s, t_arp_s;
    reg        t_cmd_e1, t_cmd_go, t_arp_e1, t_arp_go;
    reg [47:0] t_mac, t_amac;
    reg [31:0] t_ip, t_aip, t_count;
    reg [15:0] t_port, t_id, t_len, t_gap;
    always @(posedge tx_mac_clk or negedge tx_rst_n) begin
        if (!tx_rst_n) begin
            t_cmd_s <= 3'd0; t_arp_s <= 3'd0; t_cmd_e1 <= 1'b0; t_cmd_go <= 1'b0; t_arp_e1 <= 1'b0; t_arp_go <= 1'b0;
            t_mac <= 48'd0; t_amac <= 48'd0; t_ip <= 32'd0; t_aip <= 32'd0; t_count <= 32'd0;
            t_port <= 16'd0; t_id <= 16'd0; t_len <= 16'd0; t_gap <= 16'd0;
            ifg_ena <= 1'b0; ifg_val <= 8'd0;
        end else begin
            t_cmd_s  <= {t_cmd_s[1:0], cmd_tgl};
            t_arp_s  <= {t_arp_s[1:0], arp_tgl};
            t_cmd_e1 <= t_cmd_s[2] ^ t_cmd_s[1];
            t_arp_e1 <= t_arp_s[2] ^ t_arp_s[1];
            t_cmd_go <= t_cmd_e1;
            t_arp_go <= t_arp_e1;
            if (t_cmd_e1) begin
                t_mac <= r_mac; t_ip <= r_ip; t_port <= r_port; t_id <= r_id; t_len <= r_len;
                t_count <= r_count; t_gap <= r_gap;
                ifg_ena <= (r_ifg != 8'd0); ifg_val <= r_ifg;
            end
            if (t_arp_e1) begin t_amac <= r_amac; t_aip <= r_aip; end
        end
    end

    wire        b_valid, b_last, b_ready, b_stat, streaming;
    wire [7:0]  b_data;
    wire [31:0] g_n_pkt;
    wire [15:0] g_n_arp, g_n_garp, g_n_stat_to, g_max_first, g_n_underrun;
    wire        t_garp_go;      // once a second: the report snapshot's edge, below

    probe_udp_gen #(.BOARD_MAC(BOARD_MAC), .BOARD_IP(BOARD_IP), .BOARD_PORT(BOARD_PORT)) u_gen (
        .clk(tx_mac_clk), .rst_n(tx_rst_n),
        .cmd_go(t_cmd_go), .host_mac(t_mac), .host_ip(t_ip), .host_port(t_port), .cmd_id(t_id),
        .udp_len(t_len), .count(t_count), .gap(t_gap),
        .arp_go(t_arp_go), .arp_mac(t_amac), .arp_ip(t_aip), .garp_go(t_garp_go),
        .tx_valid(b_valid), .tx_data(b_data), .tx_last(b_last), .tx_ready(b_ready), .stat_valid(b_stat),
        .n_pkt(g_n_pkt), .n_arp(g_n_arp), .n_garp(g_n_garp), .n_stat_to(g_n_stat_to), .max_first(g_max_first), .n_underrun(g_n_underrun),
        .streaming(streaming));

    wire [31:0] x_n_a, x_n_b, x_max_hold_a;
    wire [15:0] x_n_stat_to, x_n_abort;

    probe_tx_mux u_mux (
        .clk(tx_mac_clk), .rst_n(tx_rst_n),
        .a_valid(a_valid), .a_data(a_data), .a_last(a_last), .a_error(a_error), .a_ready(a_ready), .a_stat(a_stat),
        .b_valid(b_valid), .b_data(b_data), .b_last(b_last), .b_ready(b_ready), .b_stat(b_stat),
        .m_valid(m_valid), .m_data(m_data), .m_last(m_last), .m_error(m_error), .m_ready(m_ready),
        .m_stat(tx_stat_valid),
        .n_a(x_n_a), .n_b(x_n_b), .n_stat_to(x_n_stat_to), .n_abort(x_n_abort), .max_hold_a(x_max_hold_a));

    // ------------------------------------------------------------ clk50
    // heartbeat config from the receiver
    reg [2:0]  c_cmd_s;
    reg        c_cmd_e1, c_cmd_go;
    reg [47:0] c_mac;
    reg [31:0] c_ip;
    reg [15:0] c_port, c_hb_us;
    reg        c_hb_en;
    always @(posedge CLK or negedge rst50_n) begin
        if (!rst50_n) begin
            c_cmd_s <= 3'd0; c_cmd_e1 <= 1'b0; c_cmd_go <= 1'b0;
            c_mac <= 48'd0; c_ip <= 32'd0; c_port <= 16'd0; c_hb_us <= 16'd0; c_hb_en <= 1'b0;
        end else begin
            c_cmd_s  <= {c_cmd_s[1:0], cmd_tgl};
            c_cmd_e1 <= c_cmd_s[2] ^ c_cmd_s[1];
            c_cmd_go <= c_cmd_e1;
            if (c_cmd_e1) begin
                c_mac <= r_mac; c_ip <= r_ip; c_port <= r_port; c_hb_us <= r_hb_us; c_hb_en <= r_flags[0];
            end
        end
    end

    reg [25:0] sec_cnt;
    reg        sec_tick;
    reg [31:0] uptime;
    always @(posedge CLK or negedge rst50_n) begin
        if (!rst50_n) begin
            sec_cnt <= 26'd0; sec_tick <= 1'b0; uptime <= 32'd0;
        end else begin
            sec_tick <= 1'b0;
            if (sec_cnt == T_SEC - 26'd1) begin
                sec_cnt <= 26'd0; sec_tick <= 1'b1; uptime <= uptime + 32'd1;
            end else begin
                sec_cnt <= sec_cnt + 26'd1;
            end
        end
    end

    wire [15:0] physr, n_miim_fail, n_txen_to;
    wire [31:0] n_hb;
    wire        init_done;
    probe_ahb #(.BOARD_MAC(BOARD_MAC), .BOARD_IP(BOARD_IP), .BOARD_PORT(BOARD_PORT), .CLK_HZ(CLK_HZ)) u_ahb (
        .clk(CLK), .rst_n(rst50_n), .start(eth_rstn),
        .haddr(haddr), .htrans(htrans), .hwrite(hwrite), .hsize(hsize), .hburst(hburst), .hwdata(hwdata),
        .hsel(hsel), .hready(hready), .hrdata(hrdata),
        .cfg_go(c_cmd_go), .host_mac(c_mac), .host_ip(c_ip), .host_port(c_port),
        .hb_en(c_hb_en), .hb_us(c_hb_us), .phy_poll(sec_tick),
        .physr(physr), .n_miim_fail(n_miim_fail), .n_hb(n_hb), .n_txen_to(n_txen_to), .init_done(init_done));

    // ------------------------------------------------------------ snapshot
    // clk50 toggles snap at the second; each fast domain copies its counters on
    // the synchronised edge and holds them; the report starts 1024 clk50
    // cycles later, long after both have done so.
    reg snap = 1'b0;
    always @(posedge CLK) if (sec_tick) snap <= ~snap;

    reg [2:0]  ts_s;
    reg [31:0] s_g_pkt, s_x_n_a, s_x_n_b, s_x_hold;
    reg [15:0] s_g_arp, s_g_garp, s_g_sto, s_g_first, s_g_under, s_x_sto, s_x_abort;
    assign t_garp_go = ts_s[2] ^ ts_s[1];
    reg        s_stream;
    always @(posedge tx_mac_clk) begin
        ts_s <= {ts_s[1:0], snap};
        if (ts_s[2] ^ ts_s[1]) begin
            s_g_pkt <= g_n_pkt; s_g_arp <= g_n_arp; s_g_sto <= g_n_stat_to; s_g_first <= g_max_first; s_g_under <= g_n_underrun; s_g_garp <= g_n_garp;
            s_x_n_a <= x_n_a; s_x_n_b <= x_n_b; s_x_hold <= x_max_hold_a; s_x_sto <= x_n_stat_to;
            s_x_abort <= x_n_abort; s_stream <= streaming;
        end
    end

    reg [2:0]  rs_s;
    reg [31:0] s_r_rx, s_r_ip;
    reg [15:0] s_r_err, s_r_arp, s_r_cmd, s_r_id, s_r_port, s_r_len;
    reg [7:0]  s_r_ifg, s_r_flags;
    always @(posedge rx_mac_clk) begin
        rs_s <= {rs_s[1:0], snap};
        if (rs_s[2] ^ rs_s[1]) begin
            s_r_rx <= r_n_rx; s_r_err <= r_n_err; s_r_arp <= r_n_arp; s_r_cmd <= r_n_cmd; s_r_id <= r_id;
            s_r_ip <= r_ip; s_r_port <= r_port; s_r_len <= r_len; s_r_ifg <= r_ifg; s_r_flags <= r_flags;
        end
    end

    // ------------------------------------------------------------ report
    // Word order is WORDS in tools/net/udp_rx_bench.py - change them together.
    localparam integer NW = 16;
    localparam integer RP_END_I = 2 + 4 * NW;
    localparam [7:0]   RP_END = RP_END_I[7:0];
    wire [32 * NW - 1:0] rep_vec = {
        uptime,                                         // 0
        {n_miim_fail, physr},                           // 1
        s_g_pkt,                                        // 2
        {s_g_arp, s_g_sto},                             // 3
        n_hb,                                           // 4
        {n_txen_to, s_g_under[14:0], init_done},        // 5
        s_x_n_a,                                        // 6
        s_x_n_b,                                        // 7
        {s_x_sto, s_x_abort},                           // 8
        s_x_hold,                                       // 9
        s_r_rx,                                         // 10
        {s_r_err, s_r_arp},                             // 11
        {s_r_cmd, s_r_id},                              // 12
        {s_g_first, s_g_garp[7:0], s_r_flags[6:0], s_stream}, // 13
        s_r_ip,                                         // 14
        {s_r_port, s_r_len}};                           // 15

    reg  [10:0] rep_wait;
    reg         rep_arm;
    reg         tx_go;
    reg  [7:0]  tx_byte;
    wire        tx_busy;
    uart_tx #(.DIV(CLK_HZ / 115200)) u_uart (
        .clk(CLK), .rst_n(rst50_n), .go(tx_go), .data(tx_byte), .busy(tx_busy), .tx(UART2_TXD));

    reg  [7:0]  rp_idx;
    reg         rp_active;
    reg  [7:0]  rp_sum;
    reg  [31:0] rp_word;
    wire [7:0]  rp_rel  = rp_idx - 8'd2;
    wire [4:0]  rp_widx = rp_rel[6:2];
    wire [1:0]  rp_bsel = rp_rel[1:0];
    wire [31:0] rp_cur  = rep_vec[32 * NW - 1 - 32 * rp_widx -: 32];
    always @(posedge CLK or negedge rst50_n) begin
        if (!rst50_n) begin
            rep_wait <= 11'd0; rep_arm <= 1'b0;
            rp_idx <= 8'd0; rp_active <= 1'b0; rp_sum <= 8'd0; rp_word <= 32'd0; tx_go <= 1'b0; tx_byte <= 8'd0;
        end else begin
            tx_go <= 1'b0;
            if (sec_tick) begin rep_arm <= 1'b1; rep_wait <= 11'd0; end
            else if (rep_arm && rep_wait != 11'd1024) rep_wait <= rep_wait + 11'd1;
            if (!rp_active) begin
                if (rep_arm && rep_wait == 11'd1024) begin
                    rep_arm <= 1'b0; rp_active <= 1'b1; rp_idx <= 8'd0; rp_sum <= 8'd0;
                end
            end else if (!tx_busy && !tx_go) begin
                if (rp_idx == 8'd0) begin
                    tx_byte <= 8'h5A; tx_go <= 1'b1; rp_idx <= rp_idx + 8'd1;
                end else if (rp_idx == 8'd1) begin
                    tx_byte <= 8'hA5; tx_go <= 1'b1; rp_idx <= rp_idx + 8'd1;
                end else if (rp_idx < RP_END) begin
                    if (rp_bsel == 2'd0) begin
                        rp_word <= rp_cur; tx_byte <= rp_cur[31:24]; rp_sum <= rp_sum + rp_cur[31:24];
                    end else begin
                        case (rp_bsel)
                            2'd1: begin tx_byte <= rp_word[23:16]; rp_sum <= rp_sum + rp_word[23:16]; end
                            2'd2: begin tx_byte <= rp_word[15:8];  rp_sum <= rp_sum + rp_word[15:8];  end
                            default: begin tx_byte <= rp_word[7:0]; rp_sum <= rp_sum + rp_word[7:0]; end
                        endcase
                    end
                    tx_go <= 1'b1; rp_idx <= rp_idx + 8'd1;
                end else begin
                    tx_byte <= rp_sum; tx_go <= 1'b1; rp_active <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------ LED
    assign LED[0] = uptime[0];          // alive
    assign LED[1] = pll_lock;
    assign LED[2] = physr[2];           // PHY link (PHYSR bit 2)
    assign LED[3] = s_stream;           // stream running (as of the last snapshot)
    assign LED[4] = s_r_rx[3];          // frames arriving

endmodule
