# timing_dump.tcl - list every failing setup path of the last Pluto build (one per endpoint) for
# analysis. Run in ~/plutosdr-fw/hdl/projects/pluto:
#   vivado -mode batch -nojournal -nolog -source <path>/timing_dump.tcl
open_checkpoint pluto.runs/impl_1/system_top_routed.dcp
set paths [get_timing_paths -setup -max_paths 20000 -nworst 1 -slack_lesser_than 0]
set f [open ptx_timing_fail.csv w]
puts $f "slack,levels,datapath,logic,route,start_clk,end_clk,start,end"
foreach p $paths {
  puts $f [join [list [get_property SLACK $p] [get_property LOGIC_LEVELS $p] [get_property DATAPATH_DELAY $p] \
    [get_property DATAPATH_LOGIC_DELAY $p] [get_property DATAPATH_NET_DELAY $p] \
    [get_property STARTPOINT_CLOCK $p] [get_property ENDPOINT_CLOCK $p] \
    [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]] ,]
}
close $f
# full detail (cells along the path) for the 300 worst endpoints
report_timing -setup -max_paths 300 -nworst 1 -slack_lesser_than 0 -file ptx_timing_paths.rpt
puts "wrote ptx_timing_fail.csv ([llength $paths] paths) and ptx_timing_paths.rpt"
