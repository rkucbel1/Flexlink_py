# util_dump.tcl - where the LUTs are and where it is congested, from the last Pluto build.
# Run in ~/plutosdr-fw/hdl/projects/pluto:
#   vivado -mode batch -nojournal -nolog -source <path>/util_dump.tcl
open_checkpoint pluto.runs/impl_1/system_top_routed.dcp
report_utilization -hierarchical -hierarchical_depth 6 -file ptx_util_hier.rpt
report_design_analysis -congestion -file ptx_congestion.rpt
report_design_analysis -complexity -hierarchical_depth 6 -file ptx_complexity.rpt
puts "wrote ptx_util_hier.rpt ptx_congestion.rpt ptx_complexity.rpt"
