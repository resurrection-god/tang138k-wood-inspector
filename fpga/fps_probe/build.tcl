open_project "fps_probe.gprj"
# Placement algorithm: $env:GOWIN_PLACE_OPTION = "0".."4" (as a string).
# The CSI-2 bridge byte clock is at its limit and its fate is decided by the
# placement - as in the main project (fpga/build_fpga.tcl). PnR is deterministic:
# only the algorithm can be varied; rebuilding the same sources gives the same bitstream.
# Sweep of 23.09.2026 (mipi_byte_clk against a 148.5 MHz constraint):
#   place 0  148.40 FAIL   place 1  149.29   place 2  142.66 FAIL
#   place 3  149.69 <- default              place 4  148.62
if {[info exists ::env(GOWIN_PLACE_OPTION)]} {
    puts "build: place_option = $::env(GOWIN_PLACE_OPTION)"
    set_option -place_option $::env(GOWIN_PLACE_OPTION)
} else {
    puts "build: place_option = 3 (default)"
    set_option -place_option 3
}
run all
