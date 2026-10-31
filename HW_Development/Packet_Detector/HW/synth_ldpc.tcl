# synth_ldpc.tcl  (Flexlink_py/HW_Development/Packet_Detector/HW)
#
# Out-of-context synthesis and implementation of the LDPC decoder core (RTL/pl_ldpc_core.v,
# step 12a of Docs/Specification_PayloadReceiver.md) on the Zynq-7020 at 100 MHz, to measure its
# real size and timing before it goes into the payload receiver. About 5-10 minutes.
#
#   cd Flexlink_py/HW_Development/Packet_Detector/HW
#   vivado -mode batch -source synth_ldpc.tcl
#
# Output: build/ldpc_ooc/utilization.rpt, utilization_hier.rpt (per sub-block), timing.rpt,
# timing_paths.rpt, and a one-line summary at the end.

set here  [file dirname [file normalize [info script]]]
set rtl   [file normalize [file join $here .. RTL]]
set out   [file join $here build ldpc_ooc]
set part  xc7z020clg484-1
set period 10.0
file mkdir $out

read_verilog [list \
    [file join $rtl pl_ldpc_rom.v] [file join $rtl pl_lutram1.v] [file join $rtl cf_tdp_ram.v] \
    [file join $rtl pd_sdp_ram.v] [file join $rtl pl_ldpc_core.v]]

synth_design -top pl_ldpc_core -part $part -mode out_of_context
create_clock -name clk -period $period [get_ports clk]

opt_design
place_design
route_design

report_utilization    -file [file join $out utilization.rpt]
report_utilization -hierarchical -hierarchical_depth 3 -file [file join $out utilization_hier.rpt]
report_timing_summary -file [file join $out timing.rpt]
report_timing -setup -max_paths 20 -nworst 1 -path_type summary -file [file join $out timing_paths.rpt]

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set u   [report_utilization -return_string]
proc pick {txt name} {
    if {[regexp "\\|\\s*$name\\s*\\|\\s*(\[0-9.\]+)" $txt -> v]} { return $v }
    return "?"
}
puts [format "pl_ldpc_core @ %.0f MHz: WNS %.3f ns (%s) | LUT %s (as memory %s), FF %s, DSP %s, BRAM tiles %s" \
          [expr {1000.0 / $period}] $wns [expr {$wns >= 0 ? "timing met" : "TIMING NOT MET"}] \
          [pick $u "Slice LUTs"] [pick $u "  LUT as Memory"] [pick $u "Slice Registers"] [pick $u "DSPs"] \
          [pick $u "Block RAM Tile"]]
puts "Reports: $out"
