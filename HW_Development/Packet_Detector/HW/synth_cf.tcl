# synth_cf.tcl  (Flexlink_py/HW_Development/Packet_Detector/HW)
#
# Out-of-context synthesis and implementation of the control-field receiver (RTL/cf_rx.v:
# de-rotator, FFT, channel estimate, soft values, Viterbi, CRC) on the Zynq-7020 at 100 MHz,
# to check timing and resources before it goes into the detector. About 3-5 minutes.
#
#   cd Flexlink_py/HW_Development/Packet_Detector/HW
#   vivado -mode batch -source synth_cf.tcl
#
# Output: build/cf_ooc/utilization.rpt, timing.rpt, and a one-line summary at the end.

set here  [file dirname [file normalize [info script]]]
set rtl   [file normalize [file join $here .. RTL]]
set out   [file join $here build cf_ooc]
set part  xc7z020clg484-1
set period 10.0
file mkdir $out

read_verilog [list \
    [file join $rtl cf_roms.v] [file join $rtl cf_tdp_ram.v] [file join $rtl pd_fft2.v] [file join $rtl cf_chest.v] \
    [file join $rtl cf_viterbi.v] [file join $rtl cf_crc10.v] [file join $rtl pd_cordic.v] \
    [file join $rtl pd_tone_rom.v] [file join $rtl pd_sdp_ram.v] [file join $rtl cf_rx.v] \
    [file join $here cf_rx_ooc.v]]

synth_design -top cf_rx_ooc -part $part -mode out_of_context
create_clock -name clk -period $period [get_ports clk]
# cfo_inc, t_trig, n_end are captured at start and held: no timing paths needed from the ports
set_false_path -from [get_ports {cfo_inc* t_trig* n_end*}]

opt_design
place_design
route_design

report_utilization    -file [file join $out utilization.rpt]
report_timing_summary -file [file join $out timing.rpt]

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set u   [report_utilization -return_string]
proc pick {txt name} {
    if {[regexp "\\|\\s*$name\\s*\\|\\s*(\\d+)" $txt -> v]} { return $v }
    return "?"
}
puts [format "cf_rx @ %.0f MHz: WNS %.3f ns (%s) | LUT %s (as memory %s), FF %s, DSP %s, BRAM tiles %s" \
          [expr {1000.0 / $period}] $wns [expr {$wns >= 0 ? "timing met" : "TIMING NOT MET"}] \
          [pick $u "Slice LUTs"] [pick $u "  LUT as Memory"] [pick $u "Slice Registers"] [pick $u "DSPs"] \
          [pick $u "Block RAM Tile"]]
puts "Reports: $out"
