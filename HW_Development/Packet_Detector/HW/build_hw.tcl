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
# Phase 2 design (detector):
#
#   PS7 (DDR, Ethernet, UART) --M_AXI_GP0--> smartconnect --> AXI DMA registers
#                                                         --> pd_0 registers (0x43C00000, 4 KB)
#   DDR <--S_AXI_HP0-- smartconnect <-- AXI DMA M_AXI_MM2S / M_AXI_S2MM
#   AXI DMA MM2S --AXIS 32b (samples)--> pd_0 --AXIS 32b (debug records)--> AXI DMA S2MM
#
#   pd_0 = RTL/pd_axi.v (module reference): the packet detector with its register interface,
#   event FIFO and debug stream; with CTRL.LOOPBACK it passes the samples straight through
#   (RUN with PL loopback, as phase 1). The firmware expects the registers at 0x43C00000.
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
# parallel synthesis jobs: each needs 2-3 GB of memory next to Vivado's own ~2 GB, so 2 is the
# safe default on a 12-16 GB PC; JOBS=4 vivado ... is faster with enough memory
set jobs       [expr {[info exists ::env(JOBS)] ? $::env(JOBS) : 2}]
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
# PL core: the packet detector (RTL, added to the project and used as a module reference)
# --------------------------------------------------------------------------
set rtl_dir [file normalize [file join $script_dir .. RTL]]
set rtl_files {}
foreach f {pd_halfband.v pd_dc_blocker.v pd_lpf.v pd_div16.v pd_detect_core.v pd_cordic.v pd_cfo_cascade.v
           pd_cfo_lane.v pd_tone_rom.v pd_sdp_ram.v pd_tone_check.v pd_timing.v cf_roms.v cf_tdp_ram.v
           pd_fft2.v cf_chest.v cf_viterbi.v cf_crc10.v cf_rx.v pd_cf.v pl_chest.v pl_roms.v pl_mulseq.v
           pd_arm.v pd_detect_top.v pd_axi.v} {
    lappend rtl_files [file join $rtl_dir $f]
}
add_files -norecurse $rtl_files
# pl_chest.v includes pl_coef_base.vh (generated by gen_pl_roms.py)
add_files -norecurse [file join $rtl_dir pl_coef_base.vh]
set_property file_type {Verilog Header} [get_files pl_coef_base.vh]
set_property include_dirs [list $rtl_dir] [current_fileset]
update_compile_order -fileset sources_1
set core [create_bd_cell -type module -reference pd_axi pd_0]
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXIS_MM2S] [get_bd_intf_pins pd_0/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins pd_0/M_AXIS]           [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM]

# --------------------------------------------------------------------------
# Interconnect (explicit smartconnects, no automation, so the result is always the same)
# --------------------------------------------------------------------------
# PS GP0 master -> DMA control registers and detector registers
set sc_ctrl [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_ctrl]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2}] $sc_ctrl
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0]  [get_bd_intf_pins sc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M00_AXI] [get_bd_intf_pins axi_dma_0/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M01_AXI] [get_bd_intf_pins pd_0/S_AXI]

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
    [get_bd_pins pd_0/aclk] \
    [get_bd_pins sc_ctrl/aclk] [get_bd_pins sc_mem/aclk]
connect_bd_net $aresetn  [get_bd_pins axi_dma_0/axi_resetn] [get_bd_pins pd_0/aresetn]
connect_bd_net $icresetn [get_bd_pins sc_ctrl/aresetn] [get_bd_pins sc_mem/aresetn]

# --------------------------------------------------------------------------
# Addresses, validate, wrapper
# --------------------------------------------------------------------------
# detector registers at the address the firmware expects; everything else automatic
set pd_seg [get_bd_addr_segs pd_0/S_AXI/*]
assign_bd_address -offset 0x43C00000 -range 4K -target_address_space [get_bd_addr_spaces ps7/Data] $pd_seg
assign_bd_address
set got [get_property OFFSET [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7/Data] -filter {NAME =~ *pd_0*}]]
if {$got != 0x43C00000} { error "pd_0 registers at $got, the firmware expects 0x43C00000" }
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
# the 40 worst setup paths, one line each (start, end, slack, logic levels), for timing work
report_timing -setup -max_paths 40 -nworst 1 -path_type summary -file [file join $build_dir timing_paths.rpt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
if {$wns < 0} {
    puts "WARNING: timing not met (WNS $wns ns) - see build/timing_summary.rpt"
}
report_utilization -file [file join $build_dir utilization.rpt]
# per block (which part of the design uses the LUTs, BRAMs and DSPs)
report_utilization -hierarchical -hierarchical_depth 4 -file [file join $build_dir utilization_hier.rpt]
set u [report_utilization -return_string]
set lut ?; set bram ?; set dsp ?
regexp {\| Slice LUTs\s+\|\s+(\d+)} $u -> lut
regexp {\| Block RAM Tile\s+\|\s+([\d.]+)} $u -> bram
regexp {\| DSPs\s+\|\s+(\d+)} $u -> dsp

set xsa [file join $build_dir ${proj_name}.xsa]
write_hw_platform -fixed -include_bit -force -file $xsa
puts "Done: $xsa (WNS $wns ns)"
puts "Used: LUT $lut of 53200, BRAM tiles $bram of 140, DSP $dsp of 220 (details: build/utilization.rpt, per block: build/utilization_hier.rpt)"
