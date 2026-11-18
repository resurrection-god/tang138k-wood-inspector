open_project "udp_probe.gprj"
# Options: CPU, MSPI and DONE pins as GPIO, route option 1 (as in fpga/axi_probe,
# removed 15.10.2026).
# Place option from $env:GOWIN_PLACE_OPTION ("0".."4", a string), default 2.
# PnR is deterministic, so the option is the only re-roll. gtx_clk (125 MHz):
#   without gratuitous ARP  place 0 116.9 FAIL, 1 112.9 FAIL, 2 141.3 (+0.92 ns),
#                           3 127.3 (+0.145), 4 110.6 FAIL
#   with it (10.0.0.125)    place 0 94.1 FAIL, 1 116.7 FAIL, 2 128.6 (+0.225),
#                           3 128.6 (+0.226), 4 126.8 (+0.111)
#   192.168.31.125, MAC mode follows the link (accepted): place 2 140.8 (+0.897) <- default
# The spread is routing, not logic: a failing path had five LUTs and 7.1 ns of
# nets between the generator and the mux (README, "Timing").
proc opt {k v} { if {[catch {set_option -$k $v} e]} { puts "!! set_option -$k $v -> $e" } }
opt top_module udp_probe_top
if {[info exists ::env(GOWIN_PLACE_OPTION)]} {
    puts "build: place_option = $::env(GOWIN_PLACE_OPTION)"
    opt place_option $::env(GOWIN_PLACE_OPTION)
} else {
    puts "build: place_option = 2 (default)"
    opt place_option 2
}
opt route_option 1
opt use_cpu_as_gpio 1
opt use_mspi_as_gpio 1
opt use_done_as_gpio 1
opt gen_text_timing_rpt 1
run all
