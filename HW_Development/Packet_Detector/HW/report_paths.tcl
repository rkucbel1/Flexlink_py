# report_paths.tcl  (Flexlink_py/HW_Development/Packet_Detector/HW)
#
# The 40 worst setup paths of the last board build (build/pd_zed, impl_1), one line each, without
# building again (build_hw.tcl writes the same report itself). About 1 minute.
#
#   cd Flexlink_py/HW_Development/Packet_Detector/HW
#   vivado -mode batch -source report_paths.tcl
set here [file dirname [file normalize [info script]]]
open_project [file join $here build pd_zed pd_zed.xpr]
open_run impl_1
report_timing -setup -max_paths 40 -nworst 1 -path_type summary -file [file join $here build timing_paths.rpt]
puts "Wrote [file join $here build timing_paths.rpt]"
