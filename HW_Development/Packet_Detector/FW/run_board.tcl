# run_board.tcl  (Flexlink_py/HW_Development/Packet_Detector/FW)
#
# Programs the Zedboard over JTAG (USB "PROG" port J17) and starts the firmware:
# bitstream -> PS7 init -> download pd_server.elf -> run. Board boot jumpers: JTAG (all to GND).
#
#   cd Flexlink_py/HW_Development/Packet_Detector/FW
#   xsdb run_board.tcl
#
# Watch the UART (USB "UART" port J14, 115200 8N1, e.g. `screen /dev/ttyACM0 115200`) for the
# IP address, then from the PC:  python ../PC/pd_hw_test.py <vector>.json --pl-loopback

set here  [file dirname [file normalize [info script]]]
set hw    [file normalize [file join $here .. HW build pd_zed]]
set bit   [file join $hw pd_zed.runs impl_1 system_wrapper.bit]
set elf   [file join $here workspace pd_server build pd_server.elf]
set inits [glob -nocomplain [file join $hw pd_zed.gen sources_1 bd system ip * ps7_init.tcl]]

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
