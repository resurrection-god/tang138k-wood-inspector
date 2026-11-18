create_clock -name clk50m -period 20 -waveform {0 10} [get_ports {CLK}]
// Байт-клок D-PHY. Константа — как в основном проекте (148.5 МГц, гл. 31.9).
create_clock -name mipi_byte_clk -period 6.734 -waveform {0 3.367} [get_pins {u_dphy/mipi_dphy_rx_inst/RX_CLK_O}]
// Все пересечения — через синхронизаторы однобитных переключателей или при
// стоящем byte_clk (геометрия моста меняется под сбросом D-PHY).
set_clock_groups -asynchronous -group [get_clocks {clk50m}] -group [get_clocks {mipi_byte_clk}]
