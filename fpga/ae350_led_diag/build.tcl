open_project "ae350_customized_demo.gprj"
if {[info exists ::env(GOWIN_PLACE_OPTION)]} {
    set_option -place_option $::env(GOWIN_PLACE_OPTION)
} else {
    set_option -place_option 3
}
run all
