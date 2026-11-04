# synth_detect.tcl  (Flexlink_py/HW_Development/Packet_Detector/HW)
#
# Out-of-context synthesis and implementation of the detector (RTL/pd_detect_top.v: front end,
# detection core, CFO lane, tone check, Preamble B timing, control field) on the
# Zynq-7020 at 100 MHz, to check timing and resources before the block goes into the board
# design. No project, no block design; about 3-5 minutes.
#
#   cd Flexlink_py/HW_Development/Packet_Detector/HW
#   vivado -mode batch -source synth_detect.tcl
#
# Output: build/detect_ooc/utilization.rpt, timing.rpt, and a one-line summary at the end.

set here  [file dirname [file normalize [info script]]]
set rtl   [file normalize [file join $here .. RTL]]
set out   [file join $here build detect_ooc]
set part  xc7z020clg484-1
set period 10.0
file mkdir $out

read_verilog [list \
    [file join $rtl pd_halfband.v] [file join $rtl pd_dc_blocker.v] [file join $rtl pd_lpf.v] [file join $rtl pd_div16.v] \
    [file join $rtl pd_detect_core.v] [file join $rtl pd_cordic.v] [file join $rtl pd_cfo_cascade.v] \
    [file join $rtl pd_cfo_lane.v] [file join $rtl pd_tone_rom.v] [file join $rtl pd_sdp_ram.v] [file join $rtl pd_tone_check.v] \
    [file join $rtl pd_timing.v] [file join $rtl cf_roms.v] [file join $rtl cf_tdp_ram.v] [file join $rtl pd_fft2.v] \
    [file join $rtl cf_chest.v] [file join $rtl cf_viterbi.v] [file join $rtl cf_crc10.v] [file join $rtl cf_rx.v] \
    [file join $rtl pd_cf.v] [file join $rtl pl_chest.v] [file join $rtl pl_roms.v] [file join $rtl pl_mulseq.v] \
    [file join $rtl pl_demap.v] [file join $rtl pl_comb.v] [file join $rtl pl_soft_roms.v] \
    [file join $rtl pl_dec.v] [file join $rtl pl_ldpc_core.v] [file join $rtl pl_ldpc_rom.v] [file join $rtl pl_lutram1.v] \
    [file join $rtl pd_arm.v] [file join $rtl pd_detect_top.v]]

synth_design -top pd_detect_top -part $part -mode out_of_context -include_dirs [list $rtl]
create_clock -name clk -period $period [get_ports clk]
# configuration ports are quasi-static (set in reset): no timing paths from them
set_false_path -from [get_ports {input_stage dc_shift* eta_* qual_len* blank_len* arm_holdoff* n_cfo* tone_lin* t_pk*}]

opt_design
# the same timing-driven steps as the board build (build_hw.tcl)
place_design -directive ExtraTimingOpt
phys_opt_design -directive AggressiveExplore
route_design -directive AggressiveExplore
phys_opt_design -directive AggressiveExplore

report_utilization    -file [file join $out utilization.rpt]
report_timing_summary -file [file join $out timing.rpt]
# the 50 worst endpoints, one line each (where the failing paths are)
report_timing -setup -max_paths 50 -nworst 1 -unique_pins -path_type summary -file [file join $out timing_paths.rpt]
report_utilization -hierarchical -hierarchical_depth 3 -file [file join $out utilization_hier.rpt]

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set u   [report_utilization -return_string]
proc pick {txt name} {
    if {[regexp "\\|\\s*$name\\s*\\|\\s*(\\d+)" $txt -> v]} { return $v }
    return "?"
}
puts [format "pd_detect_top @ %.0f MHz: WNS %.3f ns (%s) | LUT %s (as memory %s), FF %s, DSP %s, BRAM tiles %s" \
          [expr {1000.0 / $period}] $wns [expr {$wns >= 0 ? "timing met" : "TIMING NOT MET"}] \
          [pick $u "Slice LUTs"] [pick $u "  LUT as Memory"] [pick $u "Slice Registers"] [pick $u "DSPs"] \
          [pick $u "Block RAM Tile"]]
puts "Reports: $out"
