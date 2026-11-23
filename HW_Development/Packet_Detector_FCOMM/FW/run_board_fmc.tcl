# run_board_fmc.tcl  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)
#
# Programs the Zedboard over JTAG (USB "PROG" port J17) and starts the firmware:
# FMCOMMS3 bitstream -> PS7 init -> download fmc_server.elf -> run. Board boot jumpers: JTAG (all to GND). VADJ (J18) at 2.5 V.
#
#   cd Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW
#   xsdb run_board_fmc.tcl              # receiver (fmc_pd_server.elf)
#   xsdb run_board_fmc.tcl capture      # capture server (fmc_server.elf)
#
# Watch the UART (USB "UART" port J14, 115200 8N1, e.g. `screen /dev/ttyACM0 115200`) for the
# IP address and the AD9361 set-up, then from the PC:  python ../PC/fmc_capture.py --status

set here  [file dirname [file normalize [info script]]]
set hw    [file normalize [file join $here .. HW build_fmcomms3 fmc_zed]]
set bit   [file join $hw fmc_zed.runs impl_1 system_wrapper.bit]
# the receiver (fmc_pd_server) by default; "xsdb run_board_fmc.tcl capture" for fmc_server
set app   [expr {[llength $argv] > 0 && [lindex $argv 0] eq "capture" ? "fmc_server" : "fmc_pd_server"}]
set elf   [file join $here workspace $app build $app.elf]
set inits [glob -nocomplain [file join $hw fmc_zed.gen sources_1 bd system ip * ps7_init.tcl]]

foreach {what f} [list bitstream $bit firmware $elf] {
    if {![file exists $f]} { error "$what not found: $f" }
}
if {[llength $inits] != 1} { error "ps7_init.tcl not found (or not unique) under $hw" }

connect
targets -set -nocase -filter {name =~ "arm*#0"}
rst -system
after 500
puts "Programming FPGA: $bit"
fpga -file $bit
targets -set -nocase -filter {name =~ "arm*#0"}
source [lindex $inits 0]
ps7_init
ps7_post_config
puts "Downloading $elf"
dow $elf
con
puts "Running - see the UART for the IP address"
disconnect
