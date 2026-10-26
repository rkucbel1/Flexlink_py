# build_hw.tcl  (Flexlink_py/HW_Development/Packet_Detector/HW)
#
# Creates the Zedboard hardware for the packet-detector test bench from scratch, builds the
# bitstream and exports the .xsa for Vitis. Nothing in the generated project needs editing:
# change this script and re-run it.
#
#   cd Flexlink_py/HW_Development/Packet_Detector/HW
#   vivado -mode batch -source build_hw.tcl                 # full build (~10 min)
#   vivado -mode batch -source build_hw.tcl -tclargs bd     # block design only, then open it:
#   vivado build/pd_zed/pd_zed.xpr
#
# Output: build/pd_zed.xsa (with bitstream), build/pd_zed/ (Vivado project, can be deleted)
#
# Phase 1 design (plumbing / loopback):
#
#   PS7 (DDR, Ethernet, UART) --M_AXI_GP0--> smartconnect --> AXI DMA registers
#   DDR <--S_AXI_HP0-- smartconnect <-- AXI DMA M_AXI_MM2S / M_AXI_S2MM
#   AXI DMA MM2S --AXIS 32b--> pl_core --AXIS 32b--> AXI DMA S2MM
#
#   pl_core = AXI4-Stream data FIFO: samples come back unchanged (RUN with PL loopback).
#   Phase 2 replaces it with the detector (same stream ports plus an AXI-Lite register port).
#
# One stream word = one complex sample: bits [15:0] = I, [31:16] = Q (12-bit values,
# sign-extended), which is the byte order the PC sends.
# All PL logic runs on FCLK_CLK0 = 100 MHz.

set script_dir [file dirname [file normalize [info script]]]
set build_dir  [file join $script_dir build]
set proj_name  pd_zed
set bd_name    system
set part       xc7z020clg484-1
set board      digilentinc.com:zedboard:part0:1.1
set jobs       4
set mode       [expr {[llength $argv] > 0 ? [lindex $argv 0] : "all"}]

# --------------------------------------------------------------------------
# Project
# --------------------------------------------------------------------------
file mkdir $build_dir

# Board files: Vivado Store boards live in the home directory, which batch mode does not
# always search. Add every store folder found (plus $env(PD_BOARD_REPO) if set) to the
# search path, then take the requested board, or else the newest Zedboard version found.
set repos {}
if {[info exists env(PD_BOARD_REPO)]} { lappend repos $env(PD_BOARD_REPO) }
set home [file normalize ~]
foreach d [glob -nocomplain -type d [file join $home .Xilinx Vivado [version -short] xhub board_store \
                                         xilinx_board_store XilinxBoardStore Vivado * boards]] {
    lappend repos $d                    ;# e.g. ~/.Xilinx/Vivado/2025.2/.../XilinxBoardStore/Vivado/2025.2/boards
}
if {[llength $repos] > 0} {
    set old [get_param board.repoPaths]
    set_param board.repoPaths [join [lsort -unique [concat $old $repos]] " "]
    puts "Board repositories: [get_param board.repoPaths]"
}

create_project -force $proj_name [file join $build_dir $proj_name] -part $part
if {[llength [get_board_parts -quiet $board]] == 0} {
    set found [lsort -dictionary [get_board_parts -quiet -filter {PART_NAME == xc7z020clg484-1} *zed*]]
    if {[llength $found] == 0} {
        error "No Zedboard board files found. Install them in the GUI (Tools > Vivado Store > Boards >\
               ZedBoard), or set PD_BOARD_REPO to the folder that contains them."
    }
    puts "Board $board not found, using [lindex $found end] (available: $found)"
    set board [lindex $found end]
}
set_property board_part $board [current_project]
create_bd_design $bd_name

# --------------------------------------------------------------------------
# Zynq PS: Zedboard preset (DDR, MIO, Ethernet, UART, USB), plus HP0 slave and FCLK0 100 MHz
# --------------------------------------------------------------------------
set ps7 [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 ps7]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps7
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_EN_RST0_PORT {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {0} \
] $ps7

# Ethernet PHY (Marvell 88E1518): data on MIO 16..27 and management (MDIO) on MIO 52..53.
# The board preset can leave MDIO on EMIO (unconnected fabric pins): the PHY still links by
# itself at power-on, but every register read returns 0 and lwIP reports "Auto negotiation
# error" / "Ethernet Link down". Pin both explicitly and check.
set_property -dict [list \
    CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {1} \
    CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27} \
    CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {1} \
    CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53} \
] $ps7
foreach {param want} {PCW_ENET0_ENET0_IO {MIO 16 .. 27} PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53}} {
    set got [get_property CONFIG.$param $ps7]
    if {$got ne $want} { error "$param is '$got', expected '$want'" }
}

set clk [get_bd_pins ps7/FCLK_CLK0]

# Reset synchroniser for the 100 MHz domain
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_100]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins rst_100/ext_reset_in]
connect_bd_net $clk [get_bd_pins rst_100/slowest_sync_clk]
set aresetn   [get_bd_pins rst_100/peripheral_aresetn]
set icresetn  [get_bd_pins rst_100/interconnect_aresetn]

# --------------------------------------------------------------------------
# AXI DMA, simple (register) mode, 32-bit streams, 26-bit length field (64 MB per transfer)
# --------------------------------------------------------------------------
set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma axi_dma_0]
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_include_stscntrl_strm {0} \
    CONFIG.c_sg_length_width {26} \
    CONFIG.c_include_mm2s {1} \
    CONFIG.c_include_s2mm {1} \
    CONFIG.c_m_axi_mm2s_data_width {32} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_m_axi_s2mm_data_width {32} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
    CONFIG.c_mm2s_burst_size {64} \
    CONFIG.c_s2mm_burst_size {64} \
    CONFIG.c_addr_width {32} \
] $dma

# --------------------------------------------------------------------------
# PL core: phase 1 = AXIS data FIFO (loopback)
# --------------------------------------------------------------------------
set core [create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo pl_core]
set_property -dict [list \
    CONFIG.FIFO_DEPTH {1024} \
    CONFIG.TDATA_NUM_BYTES {4} \
    CONFIG.HAS_TKEEP {1} \
    CONFIG.HAS_TLAST {1} \
] $core
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXIS_MM2S] [get_bd_intf_pins pl_core/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins pl_core/M_AXIS]        [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM]

# --------------------------------------------------------------------------
# Interconnect (explicit smartconnects, no automation, so the result is always the same)
# --------------------------------------------------------------------------
# PS GP0 master -> DMA control registers
set sc_ctrl [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_ctrl]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $sc_ctrl
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0]  [get_bd_intf_pins sc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M00_AXI] [get_bd_intf_pins axi_dma_0/S_AXI_LITE]

# DMA masters -> PS HP0 (DDR)
set sc_mem [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_mem]
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $sc_mem
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_MM2S] [get_bd_intf_pins sc_mem/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_S2MM] [get_bd_intf_pins sc_mem/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_mem/M00_AXI]       [get_bd_intf_pins ps7/S_AXI_HP0]

# Clocks and resets
connect_bd_net $clk \
    [get_bd_pins ps7/M_AXI_GP0_ACLK] [get_bd_pins ps7/S_AXI_HP0_ACLK] \
    [get_bd_pins axi_dma_0/s_axi_lite_aclk] [get_bd_pins axi_dma_0/m_axi_mm2s_aclk] \
    [get_bd_pins axi_dma_0/m_axi_s2mm_aclk] \
    [get_bd_pins pl_core/s_axis_aclk] \
    [get_bd_pins sc_ctrl/aclk] [get_bd_pins sc_mem/aclk]
connect_bd_net $aresetn  [get_bd_pins axi_dma_0/axi_resetn] [get_bd_pins pl_core/s_axis_aresetn]
connect_bd_net $icresetn [get_bd_pins sc_ctrl/aresetn] [get_bd_pins sc_mem/aresetn]

# --------------------------------------------------------------------------
# Addresses, validate, wrapper
# --------------------------------------------------------------------------
assign_bd_address
regenerate_bd_layout
validate_bd_design
save_bd_design

set bd_file [get_files ${bd_name}.bd]
set wrapper [make_wrapper -files $bd_file -top]
add_files -norecurse $wrapper
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

catch {
    puts "Address map:"
    foreach seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces -of_objects [get_bd_cells -hier]]] {
        puts [format "  %-50s 0x%08X  %s" $seg [get_property OFFSET $seg] [get_property RANGE $seg]]
    }
}

if {$mode eq "bd"} {
    puts "Block design created: [file join $build_dir $proj_name ${proj_name}.xpr]"
    return
}

# --------------------------------------------------------------------------
# Build and export
# --------------------------------------------------------------------------
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "Implementation failed - see [get_property DIRECTORY [get_runs impl_1]]/runme.log"
}
open_run impl_1
report_timing_summary -file [file join $build_dir timing_summary.rpt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
if {$wns < 0} {
    puts "WARNING: timing not met (WNS $wns ns) - see build/timing_summary.rpt"
}
report_utilization -file [file join $build_dir utilization.rpt]

set xsa [file join $build_dir ${proj_name}.xsa]
write_hw_platform -fixed -include_bit -force -file $xsa
puts "Done: $xsa (WNS $wns ns)"
