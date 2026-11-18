// ============================================================================
// Memory: AXI4 back end since 09.10.2026 (docs/ARCHITECTURE.md § 4,
// ERRORS 1.19-1.21) - the SoC with Customized Data Memory, ahb_axi256, the two
// camera channels' own AXI write ports (cam_axi_wr; l4_axi_wr x2 until stage 5,
// 09.10.2026), apb_ctl (l5_axi_rd until 30.10.2026), axi_mux and the separate DDR3 Memory Interface IP
// (x32, 1 GB), the block at the end of this file; DDR_CLK and LANE4_CLK are the
// IP's clk_out.
// Until then: the SoC's Shared DDR3 with Lane 4/5 and lane4_arbiter (git 7ce963e).
// ============================================================================

// scanner_top - the top module of the FPGA project. Until 24.09.2026 it was
// ae350_ext_ethernet_demo, after the vendor Ethernet example the project started from.
module scanner_top
(
    input CLK,
    input RSTN,
    // I2C for the MCU
    inout I2C_SCL,
    inout I2C_SDA,
    // GPIO
    // S1/S2 were declared output and gave ERROR (PA2009) in every build: the GPIO in the SoC
    // is connected through an IOBUF and wants inout, like S0 next to them and like EEPROM_WP.
    //
    // ⚠ Making them inout WAS TRIED on 22.09.2026: the errors leave the log, all
    // timing PASS, the camera works - but Ethernet receive degrades
    // (RX_PBUF +2005, WD cmp +515 per run, no frame delivered). Two builds in a
    // row the same; the previous bitstream with the same firmware is clean meanwhile.
    // This is the same class of failure as ERRORS 6.2: receive depends on the placement.
    // So the change is not cosmetic, and it must not be touched without a network stress.
    //
    // 06.10.2026 - inout again. The "receive degradation" of 22.09 was most likely
    // sick boots (ERRORS 3.7, 3.10): about half of them on ANY bitstream, and one
    // boot per build does not tell the placement apart from them. With firmware
    // EAE4BA37 (2 ms receive watchdog) a sick boot gives ping 20/20 and profiles without
    // loss. This build is accepted by the share of sick boots over several boots.
    inout TWI_MUX_S0,
    inout TWI_MUX_S1,
    inout TWI_MUX_S2,
    inout [4:0] LED,    // 1:0
    // UART2
    output UART2_TXD,
    input  UART2_RXD,
    // SPI Flash Memory
    inout FLASH_SPI_CSN,
    inout FLASH_SPI_MISO,
    inout FLASH_SPI_MOSI,
    inout FLASH_SPI_CLK,
    inout FLASH_SPI_HOLDN,
    inout FLASH_SPI_WPN,
    // DDR3 Memory - AXI4 back end (AXI4 migration stage 3,
    // ERRORS 1.20): the separate DDR3 Memory Interface IP drives both chips, x32,
    // 15 row bits (A14 on U5 is a real address line now) - 1 GB. Port names and
    // pins as in fpga/axi_probe, which ran this geometry on the board.
    output DDR3_INIT,
    output [14:0] ddr_addr,
    output [2:0]  ddr_bank,
    output        ddr_cs,
    output        ddr_ras,
    output        ddr_cas,
    output        ddr_we,
    output        ddr_ck,
    output        ddr_ck_n,
    output        ddr_cke,
    output        ddr_odt,
    output        ddr_reset_n,
    output [3:0]  ddr_dm,
    inout  [31:0] ddr_dq,
    inout  [3:0]  ddr_dqs,
    inout  [3:0]  ddr_dqs_n,
    // Ethernet
    output PHY_CLK,
    output PHY_RST_N,
    output RGMII_TXC,
    output RGMII_TX_CTL,
    output [3:0] RGMII_TXD,
    input RGMII_RXC,
    input RGMII_RX_CTL,
    input [3:0] RGMII_RXD,
    output MDC,
    inout MDIO,
    inout  EEPROM_WP,
    // JTAG
    input TCK_IN,
    input TMS_IN,
    input TRST_IN,
    input TDI_IN,
    output TDO_OUT
);


wire CORE_CLK;
wire DDR_CLK;
wire AHB_CLK;
wire RTC_CLK;

wire DDR3_MEMORY_CLK;
wire DDR3_CLK_IN;

// Lane 4 writer clock: 133.333 MHz since DDR3-1066 (17.09.2026, ERRORS 1.5) -
// was 100 MHz with the 400 MHz memory clock described below - from the same
// PLL as DDR3_CLK_IN.
//
// At 62.5 MHz with 16-word bursts the writer managed 112 MB/s against the
// 280 MB/s two MIPI lanes deliver during a line, so the FIFO refused about half
// the lines and a slot spanned three exposures (ERRORS 2.5;
// former 31.6). At 100 MHz with
// 64-word bursts it keeps up completely: measured 0 lines dropped, 4097 of 4097
// bursts written (ERRORS former 31.10). PnR reports Fmax ~128 MHz on this path.
//
// The rate is not a free choice. Gowin's own Lane 4 example (DDR3_Shared SDK,
// ae350_shared_ddr3.v) drives clk_lane4 from a dedicated PLL output and documents
// it: "There is a relationship (1/4) with DDR3_MEMORY_CLK". Its memory clock is
// 200 MHz and its WR_CLK is 50. Ours is 400 MHz, so the lane clock must be 100.
//
// DDR3_CLK_IN itself stays at 50 MHz: the memory controller keeps the clock it was
// configured for, and the example likewise keeps them as separate PLL outputs.
wire LANE4_CLK;
wire DDR3_LOCK;
wire DDR3_STOP;

wire GTX_CLK;
wire gpio_nc_5;   // GPIO[5]: no pin assignment, left dangling on purpose
wire [31:10] gpio_nc_31_10;   // GPIO[9] is taken by EEPROM_WP
wire unused_hresp_msb;


// Gowin_PLL_AE350 instantiation
Gowin_PLL_AE350 u_Gowin_PLL_AE350
(
    .clkout0(),                 // was DDR_CLK 100MHz; from Gowin_PLL clkout1 since E4 (29.09.2026), below
    .clkout1(CORE_CLK),         // 800MHz
    .clkout2(AHB_CLK),          // 100MHz
    .clkout3(),                 // was APB_CLK 100MHz; APB runs on AHB_CLK since 30.09.2026
    .clkout4(RTC_CLK),          // 10MHz
    .clkin(CLK),
    .enclk0(1'b1),
    .enclk1(1'b1),
    .enclk2(1'b1),
    .enclk3(1'b1),
    .enclk4(1'b1)
);


// Gowin_PLL instantiation - the DDR3 clocks. VCO 800 MHz since E3 (29.09.2026):
// 50 / 400 / 100 / 100 MHz. It has no integer divider for 125 MHz, so the
// Ethernet clocks come from Gowin_PLL_ETH below.
//
// E4 (29.09.2026): DDR_CLK of the SoC - the AHB side of the vendor AHB -> DDR3
// bridge - comes from THIS PLL, 100 MHz = ddr3_sysclk (memory/4), phase-related
// to the memory controller. It came from Gowin_PLL_AE350 at the same 100 MHz but
// with no phase relation, and the bridge crossing ae350_ddr_clk <-> ddr3_sysclk
// was cut from timing by an asynchronous group - which left CPU writes to DDR3
// to placement luck (ERRORS 6.7, 6.11, 1.11: E1 176 -> 5 corrupted words, E2 0).
// AXI back end (stage 3): DDR_CLK of the SoC, the camera writers (LANE4_CLK
// keeps its name) and the user side of the DDR3 IP are one clock - the IP's
// clk_out, memory / 4 = 100 MHz. clkout1 and clkout4 are free.
wire clk_x1;
assign DDR_CLK   = clk_x1;
assign LANE4_CLK = clk_x1;

Gowin_PLL u_Gowin_PLL
(
    .lock(DDR3_LOCK),
    .clkout0(DDR3_CLK_IN),          // 50MHz, clk of the DDR3 IP
    .clkout1(),                     // was DDR_CLK (E4) - now clk_x1
    .clkout2(DDR3_MEMORY_CLK),      // 400MHz, CL 6 / CWL 5 in the AXI IP
    .clkout3(),                     // unused (was ETH_CLK)
    .clkout4(),                     // was LANE4_CLK - now clk_x1
    .clkin(CLK),
    .reset(1'b0),                   // Enforce
    .enclk0(1'b1),
    .enclk1(1'b1),
    .enclk2(DDR3_STOP),
    .enclk3(1'b1)
);

Gowin_PLL_ETH u_Gowin_PLL_ETH
(
    .lock(),
    .clkout0(GTX_CLK),              // 125MHz
    .clkout1(),                     // was ETH_CLK 25MHz -> PHY_CLK; now made from GTX_CLK by ODDR (30.09.2026)
    .clkout2(),
    .clkout3(),
    .clkout4(),
    .clkin(CLK),
    .reset(1'b0),
    .enclk0(1'b1),
    .enclk1(1'b1),
    .enclk2(1'b1),
    .enclk3(1'b1)
);


// ============================================================================
// Multi-Stage Power-On Reset & Stabilization Sequencer FSM
wire ddr3_rstn;
wire ddr3_init_completed;
wire soc_rst_n;
wire system_ready;

assign DDR3_INIT = ~ddr3_init_completed;

power_on_reset_sequencer u_por_sequencer
(
    .clk             (DDR3_CLK_IN),
    .rst_n_btn       (RSTN),
    .pll_ae350_lock  (1'b1),
    .pll_ddr3_lock   (DDR3_LOCK),
    .ddr3_init_done  (ddr3_init_completed),
    .phy_rst_n       (PHY_RST_N),
    .ddr3_rst_n      (ddr3_rstn),
    .soc_rst_n       (soc_rst_n),
    .system_ready    (system_ready)
);


wire [31:0] EXTS_HRDATA;
wire EXTS_HREADYIN;
wire EXTS_HRESP;
wire [31:0] EXTS_HADDR;
wire [2:0] EXTS_HBURST;
wire [3:0] EXTS_HPROT;
wire EXTS_HSEL;
wire [2:0] EXTS_HSIZE;
wire [1:0] EXTS_HTRANS;
wire [31:0] EXTS_HWDATA;
wire EXTS_HWRITE;
wire EXTS_HCLK;
wire EXTS_HRSTN;


wire eth_rstn;
wire miim_clk;
wire speedis1000;
wire speedis10;
wire duplex_status;
wire tx_mac_clk;
wire tx_mac_valid;
wire [7:0] tx_mac_data;
wire tx_mac_last;
wire tx_mac_error;
wire tx_mac_ready;
wire tx_collision;
wire tx_retransmit;
wire tx_pause_req;
wire [15:0] tx_pause_val;
wire [47:0] tx_pause_source_addr;
wire tx_statistics_valid;
wire [28:0] tx_statistics_vector;
wire intr_ethernet;
// The wrapper's side of eth_tx_mux (docs/ARCHITECTURE.md § 8.1): since
// 13.10.2026 the wrapper and udp_tx share the MAC's client transmit port.
wire w_tx_valid;
wire [7:0] w_tx_data;
wire w_tx_last;
wire w_tx_error;
wire w_tx_ready;
wire w_tx_stat;

wire rx_mac_clk;
wire rx_mac_valid;
wire [7:0] rx_mac_data;
wire rx_mac_last;
wire rx_mac_error;
wire rx_statistics_valid;
wire [26:0] rx_statistics_vector;
wire rx_pause_req;
wire [15:0] rx_pause_val;

wire [4:0] miim_phyad;
wire [4:0] miim_regad;
wire [15:0] miim_wrdata;
wire miim_wren;
wire miim_rden;
wire [15:0] miim_rddata;
wire miim_rddata_valid;
wire miim_busy;
wire mdio_in;
wire mdio_out;
wire mdio_oen;

assign miim_clk = EXTS_HCLK;
assign eth_rstn = EXTS_HRSTN;

// Tang Mega TWI multiplexer address: all three select lines are now driven by the
// CPU through GPIO[8:6], so the firmware can SCAN the multiplexer for the camera
// instead of relying on a hard-coded position.
//
// This mirrors the reference bring-up design, which never assumes an address: it
// walks the selector and accepts a route only when register 0x0000 reads back
// 0x56455945 ("VEYE"). Hard-coding was the reason the camera "disappeared" twice
// in this project - once when the constant changed, once when it was moved to the
// other connector.
//
// Note the bit numbering is the reverse of the reference's:
//     this project   reference (top.cst)   pin
//     TWI_MUX_S0     twi_sel2              P26   <- GPIO[6]
//     TWI_MUX_S1     twi_sel1              P19   <- GPIO[7]
//     TWI_MUX_S2     twi_sel0              N19   <- GPIO[8]
// That does not matter as long as the firmware scans all eight combinations.

wire [1:0] eth_hresp;
// The external AHB slave is given entirely to the MAC. Sync block registers here
// were tried and abandoned (ERRORS former 31.35): the MAC -> SoC -> MAC path has no
// margin, and any addition on this bus is a timing lottery for Ethernet. The block is
// controlled through the DDR3 mailbox (docs/ARCHITECTURE.md § 5, decision R-13).
assign EXTS_HRESP = eth_hresp[0];


// gw_ahb_ethernet_top instantiation
gw_ahb_ethernet_top u_gw_ahb_ethernet_top
(
    .rstn(eth_rstn),
    .tx_mac_clk(tx_mac_clk),
    .tx_mac_valid(w_tx_valid),
    .tx_mac_data(w_tx_data),
    .tx_mac_last(w_tx_last),
    .tx_mac_error(w_tx_error),
    .tx_mac_ready(w_tx_ready),
    .tx_collision(tx_collision),
    .tx_retransmit(tx_retransmit),
    .tx_pause_req(tx_pause_req),
    .tx_pause_val(tx_pause_val),
    .tx_pause_source_addr(tx_pause_source_addr),
    .tx_statistics_valid(w_tx_stat),            // eth_tx_mux hands the MAC's pulse to its owner
    .tx_statistics_vector(tx_statistics_vector),
    .rx_mac_clk(rx_mac_clk),
    .rx_mac_valid(rx_mac_valid),
    .rx_mac_data(rx_mac_data),
    .rx_mac_last(rx_mac_last),
    .rx_mac_error(rx_mac_error),
    .rx_statistics_valid(rx_statistics_valid),
    .rx_statistics_vector(rx_statistics_vector),
    .rx_pause_req(rx_pause_req),
    .rx_pause_val(rx_pause_val),
    .clk(miim_clk),
    .miim_phyad(miim_phyad),
    .miim_regad(miim_regad),
    .miim_wrdata(miim_wrdata),
    .miim_wren(miim_wren),
    .miim_rden(miim_rden),
    .miim_rddata(miim_rddata),
    .miim_rddata_valid(miim_rddata_valid),
    .miim_busy(miim_busy),
    .hclk(EXTS_HCLK),
    .hresetn(EXTS_HRSTN),
    .haddr(EXTS_HADDR),
    .htrans(EXTS_HTRANS),
    .hwrite(EXTS_HWRITE),
    .hsize(EXTS_HSIZE),
    .hburst(EXTS_HBURST),
    .hwdata(EXTS_HWDATA),
    .hsel(EXTS_HSEL),
    .hreadyin(EXTS_HREADYIN),
    .hrdata(EXTS_HRDATA),
    .hresp(eth_hresp),
    .hready(EXTS_HREADYIN),
    .eth_int(intr_ethernet),
    .speedis1000(speedis1000),
    .speedis10(speedis10),
    .duplex_status(duplex_status)
);


// Triple_Speed_Ethernet_MAC_Top instantiation
Triple_Speed_Ethernet_MAC_Top u_Triple_Speed_Ethernet_MAC_Top
(
    .rgmii_rxc(RGMII_RXC),
    .rgmii_rx_ctl(RGMII_RX_CTL),
    .rgmii_rxd(RGMII_RXD),
    .gtx_clk(GTX_CLK),
    .rgmii_txc(RGMII_TXC),
    .rgmii_tx_ctl(RGMII_TX_CTL),
    .rgmii_txd(RGMII_TXD),
    .speedis1000(speedis1000),
    .speedis10(speedis10),
    .duplex_status(duplex_status),
    .rstn(eth_rstn),
    .rx_mac_clk(rx_mac_clk),
    .rx_mac_valid(rx_mac_valid),
    .rx_mac_data(rx_mac_data),
    .rx_mac_last(rx_mac_last),
    .rx_mac_error(rx_mac_error),
    .rx_statistics_valid(rx_statistics_valid),
    .rx_statistics_vector(rx_statistics_vector),
    .rx_pause_req(rx_pause_req),
    .rx_pause_val(rx_pause_val),
    .tx_mac_clk(tx_mac_clk),
    .tx_mac_valid(tx_mac_valid),
    .tx_mac_data(tx_mac_data),
    .tx_mac_last(tx_mac_last),
    .tx_mac_error(tx_mac_error),
    .tx_mac_ready(tx_mac_ready),
    .tx_collision(tx_collision),
    .tx_retransmit(tx_retransmit),
    .tx_pause_req(tx_pause_req),
    .tx_pause_val(tx_pause_val),
    .tx_pause_source_addr(tx_pause_source_addr),
    .tx_statistics_valid(tx_statistics_valid),
    .tx_statistics_vector(tx_statistics_vector),
    .rx_fcs_fwd_ena(1'b0),
    .rx_jumbo_ena(1'b0),
    .tx_fcs_fwd_ena(1'b0),
    .tx_ifg_delay_ena(1'b0),
    .tx_ifg_delay(8'b0),
    .clk(miim_clk),
    .miim_phyad(miim_phyad),
    .miim_regad(miim_regad),
    .miim_wrdata(miim_wrdata),
    .miim_wren(miim_wren),
    .miim_rden(miim_rden),
    .miim_rddata(miim_rddata),
    .miim_rddata_valid(miim_rddata_valid),
    .miim_busy(miim_busy),
    .mdc(MDC),
    .mdio_in(mdio_in),
    .mdio_out(mdio_out),
    .mdio_oen(mdio_oen)
);

assign mdio_in = MDIO;
assign MDIO = (!mdio_oen) ? mdio_out : 1'bz;

// ---- the MAC's client transmit port, shared (docs/ARCHITECTURE.md § 8.1)
// A = the wrapper (the CPU's lwIP), B = udp_tx (frames straight from DDR3).
// Proven in fpga/udp_probe: ~480 000 wrapper packets interleaved with the
// probe's stream, 0 faults (ERRORS 3.12). Everything on tx_mac_clk resets from
// eth_rstn through a local synchroniser.
reg [1:0] tx_rst_sync;
always @(posedge tx_mac_clk or negedge eth_rstn)
    if (!eth_rstn) tx_rst_sync <= 2'b00; else tx_rst_sync <= {tx_rst_sync[0], 1'b1};
wire tx_rst_n = tx_rst_sync[1];

// Unused TX Mux port B tied idle
wire        u_tx_valid = 1'b0;
wire        u_tx_last  = 1'b0;
wire        u_tx_stat;
wire [7:0]  u_tx_data  = 8'd0;
wire        u_tx_ready;
wire [31:0] ethmux_n_a, ethmux_n_b, ethmux_max_hold_a;
wire [15:0] ethmux_n_stat_to, ethmux_n_abort;

eth_tx_mux u_eth_tx_mux (
    .clk(tx_mac_clk), .rst_n(tx_rst_n),
    .a_valid(w_tx_valid), .a_data(w_tx_data), .a_last(w_tx_last), .a_error(w_tx_error),
    .a_ready(w_tx_ready), .a_stat(w_tx_stat),
    .b_valid(u_tx_valid), .b_data(u_tx_data), .b_last(u_tx_last), .b_ready(u_tx_ready), .b_stat(u_tx_stat),
    .m_valid(tx_mac_valid), .m_data(tx_mac_data), .m_last(tx_mac_last), .m_error(tx_mac_error),
    .m_ready(tx_mac_ready), .m_stat(tx_statistics_valid),
    .n_a(ethmux_n_a), .n_b(ethmux_n_b), .n_stat_to(ethmux_n_stat_to), .n_abort(ethmux_n_abort),
    .max_hold_a(ethmux_max_hold_a));

// PHY_CLK, the 25 MHz PHY reference clock (30.09.2026, ERRORS 6.12). It used to be
// a separate PLL output (Gowin_PLL_ETH clkout1) that went only to pin E18, but took
// a whole PRIMARY global network (PHY_CLK_d). Now 25 MHz = 125 / 5 is made by an ODDR
// from GTX_CLK (which is on a global network anyway): the same PLL, exactly the same
// frequency. The ODDR outputs D0 in the first half of the CLK cycle and D1 in the
// second (the model in gw5a/prim_sim.v: Q0 = CLK ? d0 : d1), so the pattern (D0,D1)
// over five cycles (1,1) (1,1) (1,0) (0,0) (0,0) is 5 half-cycles of 1 and 5
// half-cycles of 0 at 4 ns, a 40 ns period, exactly 50 % duty.
reg [2:0] phy_clk_cnt = 3'd0;
always @(posedge GTX_CLK)
    phy_clk_cnt <= (phy_clk_cnt == 3'd4) ? 3'd0 : (phy_clk_cnt + 3'd1);
wire phy_clk_d0 = (phy_clk_cnt <= 3'd2);
wire phy_clk_d1 = (phy_clk_cnt <= 3'd1);
ODDR u_phy_clk_oddr (
    .Q0  (PHY_CLK),
    .Q1  (),
    .D0  (phy_clk_d0),
    .D1  (phy_clk_d1),
    .TX  (1'b0),
    .CLK (GTX_CLK)
);


// ============================================================================
// Stage 5 HDMI 720p60 AXI4 & Reset Wiring (CPU AHB -> axi_mux -> DDR3)
// ============================================================================
localparam integer AXI_DW = 256, AXI_AW = 30, AXI_IDW = 4, AXI_NW = 1, AXI_NR = 1;

wire [AXI_NW-1:0]          ax_awvalid, ax_awready, ax_wvalid, ax_wready, ax_wlast, ax_bvalid, ax_bready;
wire [AXI_NW*AXI_AW-1:0]   ax_awaddr;
wire [AXI_NW*8-1:0]        ax_awlen;
wire [AXI_NW*AXI_DW-1:0]   ax_wdata;
wire [AXI_NW*AXI_DW/8-1:0] ax_wstrb;
wire [1:0]                 ax_bresp;
wire [AXI_NR-1:0]          ax_arvalid, ax_arready, ax_rvalid, ax_rready, ax_rlast;
wire [AXI_NR*AXI_AW-1:0]   ax_araddr;
wire [AXI_NR*8-1:0]        ax_arlen;
wire [AXI_NR*AXI_DW-1:0]   ax_rdata;
wire [AXI_NR*2-1:0]        ax_rresp;

(* ASYNC_REG = "TRUE" *) reg [1:0] lane4_rst_sync;
always @(posedge LANE4_CLK or negedge soc_rst_n) begin
    if (!soc_rst_n) lane4_rst_sync <= 2'b00;
    else            lane4_rst_sync <= {lane4_rst_sync[0], 1'b1};
end
wire soc_rst_n_lane4 = lane4_rst_sync[1];

wire        lane4_irq = 1'b0;

// ---- CPU: the SoC's "Customized Data Memory" port, 64-bit AHB on DDR_CLK ----
wire [63:0] DDR_HRDATA, DDR_HWDATA;
wire        DDR_HREADY, DDR_HRESP, DDR_HWRITE, DDR_HCLK, DDR_HRSTN;
wire [31:0] DDR_HADDR;
wire [2:0]  DDR_HBURST, DDR_HSIZE;
wire [3:0]  DDR_HPROT;
wire [1:0]  DDR_HTRANS;
wire [15:0] cpu_axi_err_resp;
wire [31:0] cpu_axi_line_reads, cpu_axi_line_writes;

ahb_axi256 #(.DW(AXI_DW), .AW(AXI_AW)) u_ahb_axi256 (
    .clk(clk_x1), .rst_n(DDR_HRSTN),
    .haddr(DDR_HADDR), .htrans(DDR_HTRANS), .hwrite(DDR_HWRITE), .hsize(DDR_HSIZE), .hburst(DDR_HBURST),
    .hwdata(DDR_HWDATA), .hrdata(DDR_HRDATA), .hready(DDR_HREADY), .hresp(DDR_HRESP),
    .m_awvalid(ax_awvalid[0]), .m_awready(ax_awready[0]), .m_awaddr(ax_awaddr[0 +: AXI_AW]),
    .m_awlen(ax_awlen[0 +: 8]), .m_wvalid(ax_wvalid[0]), .m_wready(ax_wready[0]),
    .m_wdata(ax_wdata[0 +: AXI_DW]), .m_wstrb(ax_wstrb[0 +: AXI_DW/8]), .m_wlast(ax_wlast[0]),
    .m_bvalid(ax_bvalid[0]), .m_bready(ax_bready[0]), .m_bresp(ax_bresp),
    .m_arvalid(ax_arvalid[0]), .m_arready(ax_arready[0]), .m_araddr(ax_araddr[0 +: AXI_AW]),
    .m_arlen(ax_arlen[0 +: 8]), .m_rvalid(ax_rvalid[0]), .m_rready(ax_rready[0]),
    .m_rdata(ax_rdata[0 +: AXI_DW]), .m_rresp(ax_rresp[0 +: 2]), .m_rlast(ax_rlast[0]),
    .o_err_resp(cpu_axi_err_resp), .o_line_reads(cpu_axi_line_reads), .o_line_writes(cpu_axi_line_writes)
);

// Unused apb_ctl stub wires (no MIPI/udp_tx in isolated HDMI build)
wire        ring_rd_go;
wire [7:0]  ring_rd_burstcount;
wire [31:0] ring_rd_addr;
wire        ring_rd_en;
wire        c_arvalid, c_rready, c_awvalid, c_wvalid, c_wlast, c_bready;
wire [29:0] c_araddr, c_awaddr;
wire [7:0]  c_arlen, c_awlen;
wire [255:0] c_wdata;
wire        ctl_bw_v;
wire [31:0] ctl_bw_d;

// ---- interconnect and the DDR3 IP --------------------------------------------
wire                  s_awvalid, s_awready, s_wvalid, s_wready, s_wlast, s_bvalid, s_bready;
wire [AXI_IDW-1:0]    s_awid, s_bid, s_arid, s_rid;
wire [AXI_AW-1:0]     s_awaddr, s_araddr;
wire [7:0]            s_awlen, s_arlen;
wire [2:0]            s_awsize, s_arsize;
wire [1:0]            s_awburst, s_arburst, s_bresp, s_rresp;
wire [AXI_DW-1:0]     s_wdata, s_rdata;
wire [AXI_DW/8-1:0]   s_wstrb;
wire                  s_arvalid, s_arready, s_rvalid, s_rready, s_rlast;
wire [15:0]           axi_mux_bad_id;

axi_mux #(.NW(AXI_NW), .NR(AXI_NR), .DW(AXI_DW), .AW(AXI_AW), .IDW(AXI_IDW)) u_axi_mux (
    .clk(clk_x1), .rst_n(soc_rst_n_lane4),
    .m_awvalid(ax_awvalid), .m_awready(ax_awready), .m_awaddr(ax_awaddr), .m_awlen(ax_awlen),
    .m_wvalid(ax_wvalid), .m_wready(ax_wready), .m_wdata(ax_wdata), .m_wstrb(ax_wstrb),
    .m_wlast(ax_wlast), .m_bvalid(ax_bvalid), .m_bready(ax_bready), .m_bresp(ax_bresp),
    .m_arvalid(ax_arvalid), .m_arready(ax_arready), .m_araddr(ax_araddr), .m_arlen(ax_arlen),
    .m_rvalid(ax_rvalid), .m_rready(ax_rready), .m_rdata(ax_rdata), .m_rresp(ax_rresp), .m_rlast(ax_rlast),
    .s_awvalid(s_awvalid), .s_awready(s_awready), .s_awid(s_awid), .s_awaddr(s_awaddr),
    .s_awlen(s_awlen), .s_awsize(s_awsize), .s_awburst(s_awburst),
    .s_wvalid(s_wvalid), .s_wready(s_wready), .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
    .s_bvalid(s_bvalid), .s_bready(s_bready), .s_bid(s_bid), .s_bresp(s_bresp),
    .s_arvalid(s_arvalid), .s_arready(s_arready), .s_arid(s_arid), .s_araddr(s_araddr),
    .s_arlen(s_arlen), .s_arsize(s_arsize), .s_arburst(s_arburst),
    .s_rvalid(s_rvalid), .s_rready(s_rready), .s_rid(s_rid), .s_rdata(s_rdata),
    .s_rresp(s_rresp), .s_rlast(s_rlast),
    .o_bad_id(axi_mux_bad_id)
);

DDR3_AXI_Top u_ddr3 (
    .clk(DDR3_CLK_IN), .pll_stop(DDR3_STOP), .memory_clk(DDR3_MEMORY_CLK), .pll_lock(DDR3_LOCK),
    .rst_n(ddr3_rstn), .clk_out(clk_x1), .ddr_rst(), .init_calib_complete(ddr3_init_completed),
    .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready), .s_axi_awid(s_awid), .s_axi_awaddr(s_awaddr),
    .s_axi_awlen(s_awlen), .s_axi_awsize(s_awsize), .s_axi_awburst(s_awburst),
    .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready), .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb),
    .s_axi_wlast(s_wlast),
    .s_axi_bvalid(s_bvalid), .s_axi_bready(s_bready), .s_axi_bresp(s_bresp), .s_axi_bid(s_bid),
    .s_axi_arvalid(s_arvalid), .s_axi_arready(s_arready), .s_axi_arid(s_arid), .s_axi_araddr(s_araddr),
    .s_axi_arlen(s_arlen), .s_axi_arsize(s_arsize), .s_axi_arburst(s_arburst),
    .s_axi_rvalid(s_rvalid), .s_axi_rready(s_rready), .s_axi_rdata(s_rdata), .s_axi_rresp(s_rresp),
    .s_axi_rid(s_rid), .s_axi_rlast(s_rlast),
    .sr_req(1'b0), .ref_req(1'b0), .sr_ack(), .ref_ack(), .burst(1'b0),
    .O_ddr_addr(ddr_addr), .O_ddr_ba(ddr_bank), .O_ddr_cs_n(ddr_cs), .O_ddr_ras_n(ddr_ras),
    .O_ddr_cas_n(ddr_cas), .O_ddr_we_n(ddr_we), .O_ddr_clk(ddr_ck), .O_ddr_clk_n(ddr_ck_n),
    .O_ddr_cke(ddr_cke), .O_ddr_odt(ddr_odt), .O_ddr_reset_n(ddr_reset_n), .O_ddr_dqm(ddr_dm),
    .IO_ddr_dq(ddr_dq), .IO_ddr_dqs(ddr_dqs), .IO_ddr_dqs_n(ddr_dqs_n)
);

// SPI flash: our own controller (R-39, ERRORS 5.12) instead of the SoC's embedded
// one, whose register port missed timing at 100 MHz. The SoC takes its
// instruction memory 0x80000000.. from the ROM_H* port ("Customized Instruction
// Memory", AE350_EXT_I_MEM_EN, as in Gowin's example\Ext_Customized) and reaches
// the command port through the extended APB slave 0xF8000000. Clocked by AHB_CLK,
// the clock of the SoC bus and of its APB_CLK input (rom_hclk / fc_pclk, the
// SoC's copies of it, are left unused: AHB_CLK is on the global network).
wire [31:0] rom_haddr, rom_hrdata;
wire [1:0]  rom_htrans;
wire        rom_hwrite, rom_hready, rom_hresp, rom_hrstn, rom_hclk;
wire [31:0] fc_paddr, fc_pwdata, fc_prdata;
wire        fc_psel, fc_penable, fc_pwrite, fc_pready, fc_pclk, fc_prstn;
wire        fc_cs_n, fc_sclk, fc_mosi;
wire [31:0] fpga_build_id;                  // build_id.v, written by build_fpga.tcl

build_id u_build_id (.id(fpga_build_id));

// The extended APB slave is split by PADDR[12]: 0xF8000000.. the flash
// controller (0x000..0x7FF) and lcd_hud (0x800..0xFFF, PADDR[11]=1),
// 0xF8001000.. apb_ctl (the FPGA control registers, 30.10.2026).
// PADDR is stable through the whole transfer, so the PRDATA mux is steady.
// All three slaves register PSEL/PADDR at setup and drive q_rdy=0 when idle
// or unselected, pulsing pready=1 for one clock only at their own transfer end.
// ORing pready eliminates APB_PADDR -> APB_PREADY -> EXTS_HSEL combinational loops
// through the hard AE350_SOC to Ethernet.
wire [31:0] fl_prdata, npu_prdata, ctl_prdata;
wire        fl_pready, npu_pready, ctl_pready;
wire        fl_psel  = fc_psel && !fc_paddr[12] && !fc_paddr[11];
wire        npu_psel = fc_psel && !fc_paddr[12] &&  fc_paddr[11];
wire        ctl_psel = fc_psel &&  fc_paddr[12];
assign fc_prdata = fc_paddr[12] ? ctl_prdata : (fc_paddr[11] ? npu_prdata : fl_prdata);
assign fc_pready = fl_pready | npu_pready | ctl_pready;

npu_apb_wrapper u_npu_apb_wrapper (
    .clk     (AHB_CLK),
    .rst_n   (rom_hrstn),
    .psel    (npu_psel),
    .penable (fc_penable),
    .pwrite  (fc_pwrite),
    .paddr   (fc_paddr[10:0]),
    .pwdata  (fc_pwdata),
    .prdata  (npu_prdata),
    .pready  (npu_pready)
);

flash_ctl u_flash_ctl (
    .clk(AHB_CLK), .rst_n(rom_hrstn),
    .rom_haddr(rom_haddr), .rom_htrans(rom_htrans), .rom_hwrite(rom_hwrite),
    .rom_hrdata(rom_hrdata), .rom_hready(rom_hready), .rom_hresp(rom_hresp),
    .psel(fl_psel), .penable(fc_penable), .pwrite(fc_pwrite), .paddr(fc_paddr),
    .pwdata(fc_pwdata), .prdata(fl_prdata), .pready(fl_pready), .build_id(fpga_build_id),
    .spi_cs_n(fc_cs_n), .spi_sclk(fc_sclk), .spi_mosi(fc_mosi), .spi_miso(FLASH_SPI_MISO)
);
assign FLASH_SPI_CSN   = fc_cs_n;
assign FLASH_SPI_CLK   = fc_sclk;
assign FLASH_SPI_MOSI  = fc_mosi;
assign FLASH_SPI_MISO  = 1'bz;       // input
assign FLASH_SPI_WPN   = 1'b1;       // single-lane SPI: WP# and HOLD# inactive
assign FLASH_SPI_HOLDN = 1'b1;

wire [15:0] ctl_l5_stray;

apb_ctl u_apb_ctl (
    .pclk(AHB_CLK), .prst_n(rom_hrstn), .psel(ctl_psel), .penable(fc_penable), .pwrite(fc_pwrite),
    .paddr(fc_paddr[11:0]), .pwdata(fc_pwdata), .prdata(ctl_prdata), .pready(ctl_pready),
    .clk(clk_x1), .rst_n(soc_rst_n_lane4),
    .rd_go(1'b0), .rd_burstcount(8'd0), .rd_addr(32'd0), .rd_en(1'b0),
    .rd_rdy(), .rd_valid(), .rd_data(), .o_l5_stray(ctl_l5_stray),
    .s_arvalid(1'b0), .s_arready(), .s_araddr(30'd0), .s_arlen(8'd0),
    .s_rvalid(), .s_rready(1'b1), .s_rdata(), .s_rlast(),
    .s_awvalid(1'b0), .s_awready(), .s_awaddr(30'd0), .s_awlen(8'd0),
    .s_wvalid(1'b0), .s_wready(), .s_wdata(256'd0), .s_wlast(1'b0),
    .s_bvalid(), .s_bready(1'b1),
    .o_bw_v(ctl_bw_v), .o_bw_d(ctl_bw_d), .i_ld_stat(32'd0)
);

// RiscV_AE350_SOC_Top with "Customized Data Memory" (AE350_EXT_D_MEM_EN): no
// DDR3 controller and no Lane 4/5 inside; its data memory is the DDR_H* port.
RiscV_AE350_SOC_Top u_RiscV_AE350_SOC_Top
(
    .ROM_HRDATA(rom_hrdata),
    .ROM_HREADY(rom_hready),
    .ROM_HRESP(rom_hresp),
    .ROM_HADDR(rom_haddr),
    .ROM_HTRANS(rom_htrans),
    .ROM_HWRITE(rom_hwrite),
    .ROM_HCLK(rom_hclk),
    .ROM_HRSTN(rom_hrstn),
    .APB_PRDATA(fc_prdata),
    .APB_PREADY(fc_pready),
    .APB_PSLVERR(1'b0),
    .APB_PADDR(fc_paddr),
    .APB_PENABLE(fc_penable),
    .APB_PSEL(fc_psel),
    .APB_PWDATA(fc_pwdata),
    .APB_PWRITE(fc_pwrite),
    .APB_PPROT(),
    .APB_PSTRB(),
    .APB_PCLK(fc_pclk),
    .APB_PRSTN(fc_prstn),
    .DDR_HRDATA(DDR_HRDATA),
    .DDR_HREADY(DDR_HREADY),
    .DDR_HRESP(DDR_HRESP),
    .DDR_HADDR(DDR_HADDR),
    .DDR_HBURST(DDR_HBURST),
    .DDR_HPROT(DDR_HPROT),
    .DDR_HSIZE(DDR_HSIZE),
    .DDR_HTRANS(DDR_HTRANS),
    .DDR_HWDATA(DDR_HWDATA),
    .DDR_HWRITE(DDR_HWRITE),
    .DDR_HCLK(DDR_HCLK),
    .DDR_HRSTN(DDR_HRSTN),
    .TCK_IN(TCK_IN),
    .TMS_IN(TMS_IN),
    .TRST_IN(TRST_IN),
    .TDI_IN(TDI_IN),
    .TDO_OUT(TDO_OUT),
    .TDO_OE(),
    .EXT_INT({14'b0, lane4_irq, intr_ethernet}),
    .EXTS_HRDATA(EXTS_HRDATA),
    .EXTS_HREADYIN(EXTS_HREADYIN),
    .EXTS_HRESP(EXTS_HRESP),
    .EXTS_HADDR(EXTS_HADDR),
    .EXTS_HBURST(EXTS_HBURST),
    .EXTS_HPROT(EXTS_HPROT),
    .EXTS_HSEL(EXTS_HSEL),
    .EXTS_HSIZE(EXTS_HSIZE),
    .EXTS_HTRANS(EXTS_HTRANS),
    .EXTS_HWDATA(EXTS_HWDATA),
    .EXTS_HWRITE(EXTS_HWRITE),
    .EXTS_HCLK(EXTS_HCLK),
    .EXTS_HRSTN(EXTS_HRSTN),
    .I2C_SCL(I2C_SCL),
    .I2C_SDA(I2C_SDA),
    .UART2_TXD(UART2_TXD),
    .UART2_RTSN(),
    .UART2_RXD(UART2_RXD),
    .UART2_CTSN(1'b0),
    .UART2_DCDN(1'b0),
    .UART2_DSRN(1'b0),
    .UART2_RIN(1'b0),
    .UART2_DTRN(),
    .UART2_OUT1N(),
    .UART2_OUT2N(),
    // GPIO[9] = EEPROM_WP, GPIO[8:6] = TWI multiplexer select lines,
    // GPIO[4:0] = LEDs.
    .GPIO({gpio_nc_31_10, EEPROM_WP, TWI_MUX_S2, TWI_MUX_S1, TWI_MUX_S0, gpio_nc_5, LED}),
    .CORE_CLK(CORE_CLK),
    .DDR_CLK(DDR_CLK),                 // = clk_x1
    .AHB_CLK(AHB_CLK),
    .APB_CLK(AHB_CLK),
    .RTC_CLK(AHB_CLK),
    .POR_RSTN(soc_rst_n),              // released after the DDR3 IP has calibrated
    .HW_RSTN(soc_rst_n)
);

endmodule
