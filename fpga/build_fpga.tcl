open_project "scanner.gprj"

# Placement/routing algorithm selection.
#
# ae350_ahb_clk has effectively zero margin - it closes somewhere between 99.6
# and 100.4 MHz depending on how the netlist happens to place, so any RTL change
# is a re-roll (ERRORS 6.1). Re-running PnR on unchanged sources
# is deterministic and gives the SAME placement, so a build that misses cannot be
# fixed by simply building again - the only lever the tool offers is a different
# place/route algorithm.
#
# Without a variable an option keeps the value stored in the project
# (fpga/impl/scanner_process_config.json), NOT Gowin's factory default.
# And a variable is STICKY: set_option is saved into that json, so the next
# build without the variable keeps the value (seen 25.09.2026:
# GOWIN_REPLICATE=1 left "Replicate_Resources": true behind). To undo a
# lever, set it back explicitly or revert the json; check `git diff` on the
# json before calling two builds comparable. This
# comment used to say both default to 0; the project in fact has
# Route_Option "1" and Optimization_Goal "Timing", so every build so far ran
# route 1 and GOWIN_OPT_GOAL=timing changes nothing (25.09.2026, checked
# against the json and SUG1220). To re-roll:
#
#   $env:GOWIN_PLACE_OPTION = "1"   # 0..4 - the tool accepts five, not three
#   $env:GOWIN_ROUTE_OPTION = "1"   # 0..2 (project: 1)
#   $env:GOWIN_OPT_GOAL     = "timing"   # auto|area|timing (project: timing)
#   $env:GOWIN_RETIMING     = "1"   # NOT in SUG1220 nor in the json - unverified,
#                                   # never actually used; confirm in the log
#   & gw_sh.exe build_fpga.tcl
#
# Levers documented in SUG1220-2.0E (Gowin Tcl Commands User Guide, 06.2026)
# and present in our project json, i.e. accepted by IDE 1.9.12.03, but never
# tried here (25.09.2026):
#
#   $env:GOWIN_MAP_OPTION   = "4"   # -map_option 1..4: 3 = LUT5 mapping, 4 =
#                                   # LUT5/LUT6, both spend LUTs for timing
#                                   # (85 % of LUTs are free). Project: 1
#   $env:GOWIN_REPLICATE    = "1"   # -replicate_resources: PnR duplicates
#                                   # high-fanout drivers (resets, FSM state)
#   $env:GOWIN_MAX_FANOUT   = "200" # -max_fanout: the same at synthesis (1000)
#   $env:GOWIN_CLOCK_ROUTE_ORDER = "1"  # -clock_route_order: route non-primitive
#                                   # clocks by frequency, not by fanout
#                                   # (RGMII_RXC and the byte clocks ride
#                                   # generic routing)
#   $env:GOWIN_INC_PNR      = "auto"  # -inc_pnr 0|auto|<file.p>: incremental
#                                   # place & route from a kept placement - the
#                                   # candidate way to freeze the SoC placement
#                                   # of a good build across RTL edits
#   $env:GOWIN_INC_PLACE    = "C:/.../good.p"  # -inc_place: the same for placement
#                                   # only. impl/pnr/<name>.p is written by every
#                                   # build (encrypted) - KEEP the .p of a good
#                                   # build in andesprojects_builds, a rebuild
#                                   # overwrites it. Sticky like the rest: set
#                                   # "0" to switch it off again.
#
# The Tcl has no report_timing and no region/floorplan commands: regions come
# only from the .cst (GROUP/INS_LOC). Report commands do exist, but in the .sdc
# (SUG940 5.5: report_timing -max_paths/-from_clock/-mod_ins,
# report_high_fanout_nets, report_route_congestion, report_max_frequency) -
# the default report shows only the 25 worst paths (ERRORS 6.9).
#
# Record which option produced a booting build - it is not obvious that a higher
# number is better, they are alternative algorithms rather than effort levels.
#
# Assign the values as STRINGS. `$env:X = 1` has been observed not to reach
# gw_sh at all: the script then prints none of the lines below and the build
# silently uses the defaults, which reads as "the lever changed nothing".
# Confirm the "build_fpga: ..." lines appear in the log before believing a
# re-roll actually happened.
#
# Measured so far, on the RTT_NOM = 60 netlist:
#   place 0 (default)  ae350_ahb_clk 99.486, worst path -0.052 ns
#   place 1            IDENTICAL to three decimals - this lever moved nothing
#   place 2            PnR aborts at "Optimizing Phase 1"
# The default is now 2, not 0.
#
# After moving to the electrical settings from the (removed)
# docs/ddr3/DDR3_KNOWN_GOOD_CONFIG.md (DRIVE=12 on address, commands, banks, DQ and
# DM; DRIVE=4 on CKE) the default placement stopped closing timing:
#
#   place 0   ae350_ahb_clk  99.767  FAIL,  mipi_byte_clk 147.089 FAIL, path -0.065 ns
#   place 2   ae350_ahb_clk 100.154  PASS,  mipi_byte_clk 148.782 PASS, path +0.013 ns
#
# The margin is thin, and this must be known: missing ae350_ahb_clk by even half a
# megahertz kills Ethernet receive (ERRORS former 31.20; 3.1). Do not flash a build
# without reading the report.
#
# The earlier note "place 2 aborts at Optimizing Phase 1" was about a different
# netlist and no longer reproduces.
#
# The default is now 3 (14.09, frame layout in halves + set_input_delay on the
# RGMII inputs, ERRORS former 31.29). Same sources, three algorithms:
#
#   place 2   ae350_ahb_clk 100.000  slack 0.000 (path in the SoC flash controller)
#   place 3   ae350_ahb_clk 113.990  PASS, all domains PASS          <- committed
#   place 4   ae350_ahb_clk  99.672  FAIL, TNS -0.080 (Ethernet receive bridge on AHB)
#
# On place 2 Ethernet receive died in every test; on place 3 the hang also
# happens, but less often, and the receive watchdog in the firmware clears it. PnR
# is deterministic (ERRORS 6.2): this build from these sources gives the committed
# .fs bit for bit.
#
# 15.09, netlist with the sync block (ERRORS former 31.35), HRDATA stable outside
# a new transaction:
#
#   place 3             ae350_ahb_clk  98.834  FAIL, path -0.118 (MAC -> SoC -> MAC)
#   place 3 + route 1   ae350_ahb_clk  98.834  THE SAME to three decimals - route_option
#                       moves nothing on this netlist
#
# 15.09, netlist with the sync block through the DDR3 mailbox (no registers on
# AHB). Timing PASS does not yet mean the build is fit: look at the network.
#
#   place 3   ae350_ahb_clk 100.163, path +0.016 PASS; MD5 75F138A9
#             boots, sync block verified, BUT Ethernet degrades:
#             stp_client --stress 3 - 56-57 s per cycle (stage 1: 8.7-9.9 s),
#             1 timeout of 3, receive watchdog WD cmp 261 in ~3.7 min (stage 1: 6
#             over the whole 15-cycle stress)
#   place 2   ae350_ahb_clk 100.932, path +0.092 PASS; MD5 22DE3239
#             DOES NOT BOOT: network silent, core at PC=0, mcause=2, all registers
#             zero - the firmware did not get into DDR3
#   place 4   ae350_ahb_clk 100.139, path +0.014 PASS; MD5 2D465509   <- committed
#             boots; Ethernet healthy: stp_client --stress 10 - 8.7-9.0 s per
#             cycle, 1 timeout of 10, receive watchdog WD cmp 0 over the whole run;
#             sync_check 14/14, a triggered series of 4 frames without stale rows
#
# 15.09, stage 3 (ERRORS 2.5): the row tail burst and the timer watchdog
# after the first strobe.
#
#   place 4   tail burst:           ae350_ahb_clk 100.161, path +0.016 PASS; MD5 E6236D7B
#   place 4   + wd_strobe_present:  ae350_ahb_clk 100.942, lane4_clk 100.685,
#             path +0.068 PASS; MD5 68AB7B15   <- committed
#             Ethernet healthy: stp_client --stress 10 - 0 failures, 3.5 s per
#             cycle, WD cmp 0; sync_check 14/14
#
# 16.09, netlist with the DDR3 hardware fix (pins U5/E3/A3, ERRORS 1.4) and the
# linear frame layout. All three variants gave timing PASS, and all three
# behaved differently on the board - the report predicted nothing:
#
#   place 4  ae350_ahb_clk 103.656 (the best margin in the project's history)
#            capture works, but Ethernet receive degrades:
#            WD cmp +29 in 5 cycles, cycle 5.6...12.1 s against 3.5 s
#   place 2  ae350_ahb_clk 100.503
#            the network is clean (WD cmp +0), but the FPGA does not read the mailbox
#            through Lane 5: APPLY 0/106, SW 0/569, the camera counted 0 triggers,
#            all 5 cycles - timeout on 0x0222
#   place 3  ae350_ahb_clk 100.062  - FIT: WD cmp +0, cycle 3.5 s, 0 failures
#
# Default - place 3, so that a build from these sources gives the committed .fs.
# On the next RTL change pick the placement again: fitness is decided not by the
# report but by a network stress with the watchdog trigger count (ERRORS 6.2, 6.4).
#
# 24.09, RTL with receive geometry at run time (GEOM, ERRORS former 31.53), the
# repository still has the .fs from 10a9d9b (without GEOM):
#
#   place 4   ahb 103.442, lane4 136.357, mipi 152.719, mipi1 174.087 - PASS;
#             MD5 99F7692B. NOT FIT: camera 0 0/9 (rows dropped, more complete
#             rows than headers), camera 1 3/3, WD cmp +3
#   place 2   ahb 90.545, lane4 123.247, mipi 146.555 - FAIL; MD5 90687A2C
#   place 0   ahb 86.525, lane4 111.423, mipi 120.612 - FAIL; MD5 12FD78BB
#   place 3   (23.09) PASS, camera 0 without sync - the signature of former 31.51
#             (ERRORS 6.2)
#   base 10a9d9b, place 3, rebuild - timing to three decimals as the working
#             one; the .fs differs only in the "//Created Time" line
#
# The worst mipi_byte_clk path on place 4 and 0 is the header decision in the bridge
# (lane*_buf -> line_index CE), i.e. the compare with the length as a signal.
#
# 24.09, the bridge again compares with a constant (RUNTIME_LINE_BYTES = 0), the
# height from GEOM at run time:
#
#   place 3   ahb 100.695, lane4 134.192, mipi 177.437, mipi1 154.847 - PASS;
#             MD5 A7822B8E. FIT: 4 cold starts, camera 0 36/36,
#             camera 1 11/12, stress 1/40, WD cmp +0, cycle 0.8 s (the base in the
#             same hour - the same single failures)
#
# 24.09, stage L2a: the streamer split into cam_channel/cam_rx/lane5_reader/
# trigger_gen, a laser line finder inserted into each channel (laser_prof +
# laser_col_extract: +46 BSRAM and ~3000 LUT per camera, an 820-bit state word).
# With the finder NOT A SINGLE variant closes:
#
#   place 3        lane4 111.953, ahb 87.751                    MD5 6114B25C
#   place 3        lane4 117.540, ahb 84.951, mipi1 146.179     (+ hdr_q, fifo_has_word)
#   place 4        lane4 114.968, ahb 90.516, mipi 127.956      MD5 BE75C7BD
#   place 2        lane4 125.812, ahb 90.934, ddr3_sysclk 120.2 (+ hdr_q in two stages)
#   place 0        lane4 102.586, ahb 81.608
#   place 3+timing lane4 110.283, ahb 85.716                    MD5 40C88198
#
# The worst paths are not in the finder: the SoC SPI flash FIFO -> AE350 (up to 6 ns
# of pure routing), the Lane 4 arbiter -> DDR3 bridge and the camera 1 FSM, the CSI-2
# bridge. The channels got pushed apart across the die. The same build with
# ENABLE_LASER = 0 on both channels (place 3): all 11 clocks PASS, lane4 136.211,
# ahb 100.011 (slack 0.011), mipi 161.804, mipi1 186.521; MD5 5F8B5D11 - i.e. the
# split and the header pipelines are fit by themselves, and the finder breaks the
# layout.
#
# 25.09, the finder on camera 0 only (u_cam1: ENABLE_LASER = 0):
#
#   place 3   lane4 135.451 PASS, ahb 85.063 FAIL - the only path: the SoC SPI flash
#             receive FIFO -> AE350 (24 paths, up to -1.756 ns of routing)
#   place 4   ALL 11 PASS: lane4 133.575 (slack 0.242), ahb 100.213,
#             mipi 185.757, mipi1 155.449, path +0.014; MD5 17FDDAC6
#             <- committed. NOT FIT ON THE BOARD (25.09, SRAM, 3 boots of 3):
#             corrupts DDR3 - the firmware crashes after 1..15 s (mcause 2). In the
#             code over JTAG: 16 B of the pattern 11223344 55667788 99aabbcc ddeeff00
#             0000ffff in column 0 of banks 0..4 (0x0, 0x800 .. 0x2000; at 0x0
#             all 64 B) and single bits 1->0 (0x8ce8, 0xbbcc, 0x1a738,
#             0x1b9e8), stable on re-reading. With THE SAME firmware
#             AAF10CC4, A7822B8E (flash) and 5F8B5D11 (no finder) work,
#             the code in DDR3 = the .bin to the word. With the core halted, 1 MB of
#             ones and the marks in column 0 hold for 90 s - corruption only under
#             traffic. Report: all Fmax PASS, but recovery soc_rst_n -> lane4_rst_sync
#             -2.781 ns (4.8 ns of routing; on 5F8B5D11 +0.085).
#   place 3   (8DCE2F61) does not boot on the board at all - the SPI flash FIFO ->
#             AHB path is not closed, and the loader reads the firmware through it.
#
# 25.09, RTL rules (line_idx_jump_q, reset of geom_*, the bridge's line_index
# unconditional, a register on the arbiter output; benches: traces = HEAD bit for bit):
#
#   place 4            lane4 0.004, ahb 0.005, mipi 1.00/mipi1 0.98 ns,
#                      recovery -0.515; MD5 28A7C0E4
#   + .sdc clk50m<->lane4 async, GOWIN_REPLICATE=1:
#                      lane4 0.016, ahb 1.121 (!), mipi 0.75/1.04,
#                      ddr3_sysclk 0.028, recovery +2.10; MD5 BBB0D1AA
#
#   BOTH NOT FIT ON THE BOARD (SRAM, firmware AAF10CC4): 28A7C0E4 - 0 UART lines,
#   238 corrupted code words; BBB0D1AA - stops after "[BOOT]", 176 words.
#   The corruption is stable on re-reading, and for BBB0D1AA ALL 176 words are the
#   neighbouring word (+-4 bytes): a data shift inside a burst on a processor WRITE
#   (the loader copies the firmware through the AHB -> DDR3 bridge). Outcome over five
#   builds: without the finder (A7822B8E, 5F8B5D11) fit, with the finder on camera 0
#   (17FDDAC6, 28A7C0E4, BBB0D1AA) they corrupt DDR3 - with any timing PASS.
#   Analysis and versions - ERRORS 6.7.
#
# 25.09, experiment E1: SoC DDR_CLK = 133.333 MHz from Gowin_PLL clkout1 (= ddr3_sysclk,
# as in DDR3_Shared), the ae350_ddr_clk <-> ddr3_sysclk cut removed from the .sdc:
#
#   place 4   the bridge paths are now checked and close: ddr->sys setup +2.807,
#             hold +0.229; sys->ddr +5.933 / +0.435; ddr3_sysclk 0.525 (was
#             0.028), ae350_ddr_clk 238 MHz, ahb 1.158, lane4 0.018; MD5 0979BF0E
#             ON THE BOARD: boots further (cameras, UDP), then FATAL mcause 2 at
#             0x10504; 5 corrupted words in the code instead of 176, and no longer the
#             neighbouring word: sources +-64 / -16 / +4 bytes, 4 of 5 at the start
#             of a 64-byte line. E1 removed the bulk of the corruption, but not all.
#
# 25.09, experiment E2: memory within the datasheet - 400 MHz / CL = 6 (the SoC
# netlist from the version before 10a9d9b), VCO 800 (PA1019 gone), LANE4_CLK 133.333
# = 1/3 of memory (datasheet: >= 1/4), DDR_CLK 100 = ddr3_sysclk from the same PLL
# (ERRORS 1.11):
#
#   place 4   ddr3_sysclk +3.677 ns, ae350_ddr_clk +4.702, memory +2.115,
#             ahb +1.038, mipi 1.215/1.337 - but lane4 130.091 FAIL (-0.187):
#             row_word[13] -> CE of the diag_frame_* snapshots (magnitude compares
#             row_word >= ROW_TOTAL_WORDS and row_idx+1 >= FRAME_HEIGHT at
#             frame close); MD5 8D448DDC, not put on the board.
#             -> flags row_full_q / row_last_q in cam_channel.v.
#             ON THE BOARD (SRAM, AAF10CC4): boots fully, THE CODE IN DDR3 IS INTACT -
#             0 differing words (the first build with the finder without corruption);
#             Lane 5 reads the mailbox (word 31 = 0x5555, 0 timeouts), the LASR echo
#             is there. BUT: camera 0 does not receive MIPI (0 headers, rejected by WC
#             4724 and +45/s, WC candidate 40960 - garbage, the signature of former
#             31.51); sync_check 11/14 (CORE: 9 of 20 accepted, overload 0 of 1);
#             capture with camera 1 -> "[FATAL] Stack overflow in task: IDLE!".
#             Control A7822B8E + the same firmware: sync_check 14/14, camera 0
#             3/3, camera 1 2/2, CRC ok - the firmware is fine, the failures are the
#             bitstream.
#   + flags   E2b: lane4 -0.166 (Lane 5 rd_valid -> CE sync_mbx), ahb 96.514
#             FAIL (TNS -2.559); MD5 E74E48C9, not put on the board.
#
# 25.09, ROLLBACK (owner's decision): the finder disabled on both cameras
# (ENABLE_LASER = 0, as 5F8B5D11), DDR3-1066 and the clocks as in HEAD; the short-path
# RTL fixes, clk50m<->lane4 async and the reports in the .sdc stay. Experiments
# E1/E2 - patch andesprojects_builds\E2_mem400_p4\worktree_E2_full.patch.
#
#   R0 place 4   lane4 -0.699 (the arbiter's own -> wr_done -> CE diag_frame_* /
#                burst_idx of channel 0), ahb 91.193 FAIL (SPI flash FIFO ->
#                ROM_HRDATA, -0.966), ddr3_sysclk 1.328, mipi 1.03/0.60;
#                MD5 6D35C808. No finder, but timing is still decided by
#                placement: half a dozen families of lane4 paths sit at the edge (the
#                arbiter's own, diag_burst, hdr_ptr, cfg_period -> trig_cnt, internal
#                paths of the vendor's Lane 5 bridge). 5F8B5D11 (place 3) got lucky.
#   R1 (+ pre-decoding of the Lane 5 mailbox sync_we_q, GOWIN_REPLICATE=1):
#                lane4 +0.240 (was -0.699), ahb 95.959 FAIL (-0.421: inside the
#                vendor SPI flash controller ctrl_cs_r -> CE data_cnt_r and the
#                Ethernet receive bridge on AHB), ddr3_sysclk 1.115; MD5 05D8705E.
#   R2 = R1 + GOWIN_INC_PLACE from A7822B8E (ae350_ext_ethernet.p): NOT applied -
#                "WARN (PA2136): Placement matching(66.11) is less than the
#                threshold value(80) needed to run incremental flow, the default
#                placement flow will be run". Incremental PnR needs >= 80 %
#                match with the reference netlist: take the reference from a fit build
#                of THIS SAME project (the finder adds ~15 % logic - within limits).
#   place sweep on the R1 sources (+ replicate), two builds in parallel in a
#   second copy of the directory (the licence gives two sessions):
#     place 3   ALL check_timing CHECKS, including the 0.3 ns margin: lane4 +0.726,
#               ahb +0.393 (104.1), mipi +0.508/+1.634, ddr3_sysclk 1.041,
#               hold/recovery/removal ok; MD5 0A4FA272, placement saved:
#               andesprojects_builds\R1_p3_GOOD\scanner.p.
#               ON THE BOARD: the code in DDR3 intact (0 words), sync_check 14/14,
#               camera 0 3/3, camera 1 2/2, CRC ok - BUT Ethernet receive degrades:
#               WD cmp 0 -> 23 over a capture and stress 5, frame delivery 11-20 s,
#               stress cycle 0.8-3.7 s. Control A7822B8E in the same minute: WD cmp
#               0, delivery 0.3-1.0 s, cycle 0.8-0.9 s. Not fit for the network.
#     place 2   ahb 89.941 FAIL, lane4 0.207; MD5 625993F7.
#   Observation over all builds of the day: with -replicate_resources ahb 111-113 MHz
#   in three of four (BBB0D1AA, 0979BF0E, 8D448DDC; E74E48C9 - 96.5), without it
#   91-100.2. The AHB path is the vendor's SPI flash FIFO, the fan-out of gray_rd_ptr.
#   30.09.2026, o_wants_bus fix (ERRORS 1.12, stereo capture): every placement
#   from scratch unfit - place 4 7CD947C7 (timing PASS, on the board WD cmp +432, frame
#   53 s), place 3 F0343B9B (ahb +0.009, the network did not come up), place 2 56DE0023
#   (ahb 93 FAIL). RGMII receive is a "lottery" (RGMII_RXC on C23 without a global
#   network), a fix of a few LUTs re-placed it (ERRORS 6.13).
#   The way out - -inc_place from the fit EDDD651B: a rebuild of HEAD ecd2329..e234a74
#   gives the same placement (timing to the digit, the .fs differs in 6 service
#   bytes, MD5 CF00AD15), its impl/pnr/scanner.p is saved as
#   andesprojects_builds\20260930_incplace\EDDD651B_scanner.p. With it the fix
#   built into 992A0F19: rgmii_rxclk and ae350_ddr_clk = EDDD651B to the digit, ahb
#   115.0. No PA2136 (< 80 % match).
#   ⚠ 992A0F19 can be reproduced only like this:
#     $env:GOWIN_INC_PLACE = "C:/Users/ryazanov/Documents/andesprojects_builds/20260930_incplace/EDDD651B_scanner.p"
#   Without it the same sources give 7CD947C7. The option is sticky - after the build
#   restore scanner_process_config.json (INCREMENTAL_PLACE_ONLY "0").
#   30.09.2026, both line finders + points in the FPGA (ERRORS 1.14): -inc_place from
#   992A0F19 hung in routing (> 1 h, ~15 % new logic); the same build from scratch,
#   place 4, in a copy of the fpga/ directory (a second Gowin session) - 12 min,
#   1E8324B3, all PASS, on the board the network and DDR3 are fine. Its scanner.p -
#   andesprojects_builds\20260930_l2b_1E8324B3\ (the reference for the next small change).
#   01.10.2026, stage 2 - frame without rows (ERRORS 1.15): -inc_place from 1E8324B3
#   gave 7A233128 - lane4_clk +0.011, mipi +0.114, the 0.3 ns rule not met (the paths
#   are not through the new logic: geom_seq_seen -> cfg_frame_lines and the FIFO
#   write). From scratch, place 4 - B43AD15A, all PASS, fit on the board. Its scanner.p -
#   andesprojects_builds\20261001_stage2_B43AD15A\ (the new reference).
#
# Default - place 4, so that a build from these sources gives this .fs.
#
# Run gw_sh directly: from a "pwsh -NoProfile -File" wrapper it does not get the
# licence ("Server not responding").
catch { set_option -include_path {src} }
if {[info exists ::env(GOWIN_PLACE_OPTION)]} {
    puts "build_fpga: place_option = $::env(GOWIN_PLACE_OPTION)"
    set_option -place_option $::env(GOWIN_PLACE_OPTION)
} else {
    puts "build_fpga: place_option = 3 (default)"
    set_option -place_option 3
}
if {[info exists ::env(GOWIN_ROUTE_OPTION)]} {
    puts "build_fpga: route_option = $::env(GOWIN_ROUTE_OPTION)"
    set_option -route_option $::env(GOWIN_ROUTE_OPTION)
}
# Effort levers rather than algorithm choices. opt_goal steers optimisation at
# area or timing; retiming lets the tool move registers through combinational
# logic, which is a legal transformation and does not change cycle counts.
if {[info exists ::env(GOWIN_OPT_GOAL)]} {
    puts "build_fpga: opt_goal = $::env(GOWIN_OPT_GOAL)"
    set_option -opt_goal $::env(GOWIN_OPT_GOAL)
}
if {[info exists ::env(GOWIN_RETIMING)]} {
    puts "build_fpga: retiming = $::env(GOWIN_RETIMING)"
    set_option -retiming $::env(GOWIN_RETIMING)
}
# SUG1220 levers, see the list at the top.
foreach {var opt} {GOWIN_MAP_OPTION map_option GOWIN_REPLICATE replicate_resources
                   GOWIN_MAX_FANOUT max_fanout GOWIN_CLOCK_ROUTE_ORDER clock_route_order
                   GOWIN_INC_PNR inc_pnr GOWIN_INC_PLACE inc_place} {
    if {[info exists ::env($var)]} {
        puts "build_fpga: $opt = $::env($var)"
        set_option -$opt $::env($var)
    }
}

# Always on (25.09.2026, SUG1220): by the guide these change reports only, not
# the netlist or the placement. Not yet confirmed here: the first build with
# them should give the same timing numbers as the one before on the same RTL.
#   -gen_text_timing_rpt  plain-text timing report impl/pnr/scanner.tr, easier
#                         to parse than the HTML check_timing.ps1 reads now
#   -print_all_synthesis_warning / -show_all_warn
#                         synthesis and PnR print only part of their warnings
#                         by default; a warning that is never printed is one
#                         nobody can act on
catch { set_option -gen_text_timing_rpt 1 }
catch { set_option -print_all_synthesis_warning 1 }
catch { set_option -show_all_warn 1 }

# Bitstream identity (src/build_id.v -> flash_ctl offset 0x014 -> GigE Vision DeviceFpgaVersion,
# since 23.10.2026): CRC-32 of every file the project lists, sorted by path, build_id.v left out,
# carriage returns dropped (autocrlf must not change the id). The same sources give the same id;
# commit build_id.v with the RTL and the .fs it built.
set bid_files {}
set fh [open "scanner.gprj" r]; set gprj [read $fh]; close $fh
foreach {m p} [regexp -all -inline {<File path="([^"]+)"} $gprj] {
    if {![string match "*build_id.v" $p]} { lappend bid_files $p }
}
set bid_crc 0
foreach f [lsort $bid_files] {
    set fh [open $f rb]; set data [string map [list "\r" ""] [read $fh]]; close $fh
    set bid_crc [zlib crc32 $data $bid_crc]
}
set bid [format %08X [expr {$bid_crc & 0xFFFFFFFF}]]
set fh [open "src/build_id.v" r]; set bv [read $fh]; close $fh
regsub {32'h[0-9A-Fa-f]{8}} $bv "32'h$bid" bv2
if {$bv2 ne $bv} { set fh [open "src/build_id.v" w]; puts -nonewline $fh $bv2; close $fh }
puts "build_fpga: BUILD_ID = $bid ([llength $bid_files] files)"

run all
