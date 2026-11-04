# synth_ptx.tcl - out-of-context synthesis, place and route of ptx_core on the Pluto's Zynq-7010
# (xc7z010clg225-1) at 100 MHz: timing and resources before the full Pluto build.
#   cd HW && vivado -mode batch -source synth_ptx.tcl      # reports in HW/build/ptx_ooc/
set here [file dirname [file normalize [info script]]]
set rtl  [file join $here .. RTL]
set out  [file join $here build ptx_ooc]
file mkdir $out
read_verilog [list $rtl/ptx_core.v $rtl/ptx_enc.v $rtl/ptx_ifft.v $rtl/ptx_roms.v]
synth_design -top ptx_core -part xc7z010clg225-1 -mode out_of_context
create_clock -name clk -period 10.0 [get_ports clk]
opt_design
place_design
route_design
report_utilization -file $out/utilization.rpt
report_utilization -hierarchical -file $out/utilization_hier.rpt
report_timing_summary -file $out/timing.rpt
report_timing -max_paths 10 -file $out/timing_paths.rpt
puts "WNS: [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] ns"
