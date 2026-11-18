// udp_probe: the Ethernet constraints of fpga/src/scanner.sdc, unchanged.
create_clock -name clk50m -period 20 -waveform {0 10} [get_ports {CLK}]
create_clock -name gtx_clk -period 8 -waveform {0 4} [get_pins {u_pll/PLL_inst/CLKOUT0}]
// RGMII receive clock: 125 MHz at gigabit (a 50 MHz constraint here once left
// the whole receive domain unconstrained, ERRORS 3.1, 6.3).
create_clock -name rgmii_rxclk -period 8 -waveform {0 4} [get_ports {RGMII_RXC}]
set_input_delay -clock rgmii_rxclk -max 2.8 [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
set_input_delay -clock rgmii_rxclk -min 1.2 -add_delay [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
set_input_delay -clock rgmii_rxclk -clock_fall -max 2.8 -add_delay [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
set_input_delay -clock rgmii_rxclk -clock_fall -min 1.2 -add_delay [get_ports {RGMII_RX_CTL RGMII_RXD[0] RGMII_RXD[1] RGMII_RXD[2] RGMII_RXD[3]}]
// The three domains meet only through synchronised toggles with held data
// (udp_probe_top.v) and inside Gowin's wrapper and MAC, as in the main design,
// where the same clocks sit in one -exclusive set_clock_groups.
set_clock_groups -asynchronous -group [get_clocks {clk50m}] -group [get_clocks {gtx_clk}] -group [get_clocks {rgmii_rxclk}]
