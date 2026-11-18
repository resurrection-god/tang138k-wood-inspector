create_clock -name gtx_clk -period 8 -waveform {0 4} [get_pins {u_Gowin_PLL_ETH/PLL_inst/CLKOUT0}]
create_clock -name clk50m -period 20 -waveform {0 10} [get_ports {CLK}]
create_clock -name clk_x1 -period 10 -waveform {0 5} [get_pins {u_ddr3/gw3_top/u_GW_DDR3_PHY_MC/u_ddr_phy_top/fclkdiv/CLKOUT}]
create_clock -name rgmii_rxclk -period 8 -waveform {0 4} [get_ports {RGMII_RXC}]

set_input_delay -clock rgmii_rxclk -max 2.8 [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
set_input_delay -clock rgmii_rxclk -min 1.2 -add_delay [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
set_input_delay -clock rgmii_rxclk -clock_fall -max 2.8 -add_delay [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
set_input_delay -clock rgmii_rxclk -clock_fall -min 1.2 -add_delay [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]

create_clock -name ddr3_clkin -period 20 -waveform {0 10} [get_pins {u_Gowin_PLL/PLL_inst/CLKOUT0}]
create_clock -name ddr3_memory_clk -period 2.5 -waveform {0 1.25} [get_pins {u_Gowin_PLL/PLL_inst/CLKOUT2}]
create_clock -name ae350_ahb_clk -period 10 -waveform {0 5} [get_pins {u_Gowin_PLL_AE350/PLL_inst/CLKOUT2}]

// Separate asynchronous clock groups
set_clock_groups -exclusive -group [get_clocks {rgmii_rxclk}] -group [get_clocks {gtx_clk}] -group [get_clocks {ae350_ahb_clk}] -group [get_clocks {clk_x1}]
set_clock_groups -asynchronous -group [get_clocks {ddr3_clkin}] -group [get_clocks {clk_x1}]
set_clock_groups -asynchronous -group [get_clocks {clk_x1}] -group [get_clocks {ddr3_memory_clk}]
set_clock_groups -asynchronous -group [get_clocks {clk50m}] -group [get_clocks {clk_x1}]
set_clock_groups -exclusive -group [get_clocks {ddr3_memory_clk}] -group [get_clocks {ddr3_clkin}]

// Reset bridge
set_false_path -from [get_regs {lane4_rst_sync*}]

set_operating_conditions -grade c -model slow -speed 2 -setup

report_timing -setup -max_paths 50 -max_common_paths 1
report_timing -hold -max_paths 25 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {ae350_ahb_clk}] -to_clock [get_clocks {ae350_ahb_clk}] -max_paths 40 -max_common_paths 1
report_timing -setup -from_clock [get_clocks {clk_x1}] -to_clock [get_clocks {clk_x1}] -max_paths 40 -max_common_paths 1
report_high_fanout_nets -max_nets 25
