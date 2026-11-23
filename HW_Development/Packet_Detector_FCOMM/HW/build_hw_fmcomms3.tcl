# build_hw_fmcomms3.tcl  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/HW)
#
# FMCOMMS3 build variant of the FlexLink receiver (step 4 of the handover plan): the AD9361 on
# the FMC connector feeds the PL receiver pd_0 through a DDR ring. Creates the Zedboard hardware
# from scratch, builds the bitstream and exports the .xsa.
# FMC_NO_PD=1 builds the step-2 design instead: capture only (AD9361 -> DDR), no pd_0.
# Separate from the CoreBoard build (../../Packet_Detector); this script writes only to build_fmcomms3/.
#
#   export ADI_HDL_DIR=~/SiskinRF/hdl          # ADI HDL checkout (main), library IPs built
#   cd Flexlink_py/HW_Development/Packet_Detector_FCOMM/HW
#   vivado -mode batch -source build_hw_fmcomms3.tcl                  # full build (~10 min)
#   FMC_NO_PD=1 vivado -mode batch -source build_hw_fmcomms3.tcl      # capture only (step 2)
#   vivado -mode batch -source build_hw_fmcomms3.tcl -tclargs bd      # block design only
#   vivado -mode batch -source build_hw_fmcomms3.tcl -tclargs synth   # stop after synthesis
#   vivado build_fmcomms3/fmc_zed/fmc_zed.xpr                         # open the project
#
# Output (build_fmcomms3/): fmc_zed.xsa (with bitstream), summary.txt, utilization.rpt,
# utilization_hier.rpt, timing_summary.rpt, fmc_pins.xdc (pins taken from ADI's constraints),
# fmc_pins_dropped.txt (ADI constraint lines not used), fmc_zed/ (Vivado project).
#
# Design:
#
#   AD9361 --LVDS--> axi_ad9361 (1R1T) --{Q0,I0} 32b on l_clk--> axi_dmac (FIFO -> AXI3 64b)
#                                                                     --S_AXI_HP2--> DDR ring
#   DDR ring --S_AXI_HP0--> AXI4->AXI3 converter --> AXI DMA MM2S --AXIS 32b--> pd_0
#                          (pd_0's record output M_AXIS is tied ready: no S2MM in this variant)
#   PS7 --M_AXI_GP0--> smartconnect --> axi_ad9361 registers (0x79020000, 64 KB)
#                                   --> axi_dmac registers   (0x7C400000, 4 KB)
#                                   --> pd_0 registers       (0x43C00000, 4 KB)
#                                   --> AXI DMA registers    (automatic, 0x40400000)
#   PS7 SPI0 (EMIO) --> AD9361 SPI          PS7 GPIO (EMIO, 64) --> fmc_glue --> AD9361 control
#   axi_dmac irq --> PS7 IRQ_F2P
#
# Addresses and GPIO bit numbers are the same as in ADI's fmcomms2/zed design, so no-OS's
# defaults fit. Sample word: [15:0] = I, [31:16] = Q (12 bit, sign-extended), as pd_0 expects.
# Clocks: FCLK_CLK0 100 MHz (AXI), FCLK_CLK1 200 MHz (IDELAYCTRL reference), l_clk from the
# AD9361 (sample side of axi_ad9361 and of axi_dmac). pd_0 and the AXI DMA run on FCLK_CLK0, as
# in the CoreBoard design; their RTL is ../../Packet_Detector/RTL, the same files build_hw.tcl uses.

set script_dir [file dirname [file normalize [info script]]]
set build_dir  [file join $script_dir build_fmcomms3]
set proj_name  fmc_zed
set bd_name    system
set part       xc7z020clg484-1
set board      digilentinc.com:zedboard:part0:1.1
set jobs       [expr {[info exists ::env(JOBS)] ? $::env(JOBS) : 2}]
set mode       [expr {[llength $argv] > 0 ? [lindex $argv 0] : "all"}]
if {[lsearch -exact {all bd synth} $mode] < 0} { error "unknown mode '$mode' (all, bd or synth)" }
set with_pd    [expr {!([info exists ::env(FMC_NO_PD)] && $::env(FMC_NO_PD) ne "" && $::env(FMC_NO_PD) ne "0")}]
set pd_dir     [file normalize [file join $script_dir .. .. Packet_Detector]]
# rx_clk (the AD9361 DATA_CLK, l_clk): ADI's constraint is 4 ns, the 2R2T clock at 61.44 Msps.
# In 1R1T the clock is 2 x the sample rate: 80 MHz at 40 Msps, at most 122.88 MHz at the AD9361's
# 61.44 Msps limit (the rate of no-OS's digital interface tune). 8 ns covers every 1R1T rate; the
# LVDS input timing itself is set by the IDELAY tune at run time, not by this constraint.
set rx_clk_ns  [expr {[info exists ::env(FMC_RX_CLK_NS)] ? $::env(FMC_RX_CLK_NS) : 8.0}]
# placement directive for the full design (default as build_hw.tcl); others to try when timing
# misses by a little: ExtraNetDelay_high, AltSpreadLogic_high, Explore, ExtraPostPlacementOpt
set place_dir  [expr {[info exists ::env(PLACE_DIRECTIVE)] ? $::env(PLACE_DIRECTIVE) : "ExtraTimingOpt"}]

# ADI HDL checkout: $ADI_HDL_DIR, else ~/SiskinRF/hdl (next to Flexlink_py)
if {[info exists ::env(ADI_HDL_DIR)]} {
    set adi [file normalize $::env(ADI_HDL_DIR)]
} else {
    set adi [file normalize [file join $script_dir .. .. .. .. hdl]]
}
set adi_xdc [file join $adi projects fmcomms2 zed system_constr.xdc]
foreach f [list [file join $adi library] $adi_xdc] {
    if {![file exists $f]} { error "$f not found. Clone ADI HDL (main) and set ADI_HDL_DIR (now: $adi)" }
}
puts "ADI HDL: $adi"

set notes {}        ;# lines for summary.txt
proc note {s} { global notes; lappend notes $s; puts "NOTE: $s" }
note [expr {$with_pd ? "design: AD9361 -> DDR ring -> pd_0 (step 4)" : "design: capture only, no pd_0 (FMC_NO_PD)"}]

# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
# a block-design pin that must exist; the error lists the cell's pins
proc pin {path} {
    set p [get_bd_pins -quiet $path]
    if {$p eq ""} {
        set cell [lindex [split $path /] 0]
        set names [lsort [get_property NAME [get_bd_pins -quiet -of_objects [get_bd_cells $cell]]]]
        error "pin $path not found. Pins of $cell: $names"
    }
    return $p
}
proc intf {path} {
    set p [get_bd_intf_pins -quiet $path]
    if {$p eq ""} {
        set cell [lindex [split $path /] 0]
        set names [lsort [get_property NAME [get_bd_intf_pins -quiet -of_objects [get_bd_cells $cell]]]]
        error "interface $path not found. Interfaces of $cell: $names"
    }
    return $p
}
# set IP parameters; required = 1: a missing parameter is an error, else a note
proc set_params {cell required plist} {
    set have [list_property $cell CONFIG.*]
    set d {}
    foreach {k v} $plist {
        if {[lsearch -exact $have CONFIG.$k] < 0} {
            if {$required} { error "$cell has no parameter $k. It has: [lsort $have]" }
            note "$cell: no parameter $k in this HDL version, skipped"
            continue
        }
        lappend d CONFIG.$k $v
    }
    if {[llength $d] > 0} { set_property -dict $d $cell }
}
# newest ADI IP of that name in the catalog
proc adi_vlnv {name} {
    global adi
    set v [lindex [lsort -dictionary [get_ipdefs -all -quiet *:$name:*]] end]
    if {$v eq ""} {
        error "ADI IP $name is not in the IP catalog. Build it first:\
               (export ADI_IGNORE_VERSION_CHECK=1; make -C $adi/library/$name)"
    }
    return $v
}
# ADI constraint lines whose ports all exist in this design -> dst; returns {kept dropped}
proc filter_xdc {src dst ports} {
    set fh [open $src]; set text [read $fh]; close $fh
    regsub -all {\\\n} $text " " text
    set out [open $dst w]
    puts $out "# Generated by build_hw_fmcomms3.tcl from\n# $src\n# (only lines for ports of this design)\n"
    set kept 0; set dropped {}
    foreach line [split $text \n] {
        set t [string trim $line]
        if {$t eq "" || [string index $t 0] eq "#"} continue
        set refs [regexp -all -inline {get_ports\s+\{?\s*([A-Za-z0-9_]+)} $t]
        set ok [expr {[llength $refs] > 0}]
        foreach {- name} $refs { if {[lsearch -exact $ports $name] < 0} { set ok 0 } }
        if {$ok && [regexp {^create_clock} $t] && [regexp -- {-name\s+rx_clk\y} $t]} {
            # see rx_clk_ns at the top
            regsub -- {-period\s+[0-9.]+} $t "-period $::rx_clk_ns" t
            set t "$t   ;# period set by build_hw_fmcomms3.tcl (1R1T), ADI: 4 ns"
        }
        if {$ok} { puts $out $t; incr kept } else { lappend dropped $t }
    }
    close $out
    return [list $kept $dropped]
}

# --------------------------------------------------------------------------
# Project, board files (as ../../Packet_Detector/HW/build_hw.tcl), ADI IP repository
# --------------------------------------------------------------------------
file mkdir $build_dir
set repos {}
if {[info exists env(PD_BOARD_REPO)]} { lappend repos $env(PD_BOARD_REPO) }
set home [file normalize ~]
foreach d [glob -nocomplain -type d [file join $home .Xilinx Vivado [version -short] xhub board_store \
                                         xilinx_board_store XilinxBoardStore Vivado * boards]] {
    lappend repos $d
}
if {[llength $repos] > 0} {
    set_param board.repoPaths [join [lsort -unique [concat [get_param board.repoPaths] $repos]] " "]
}
create_project -force $proj_name [file join $build_dir $proj_name] -part $part
if {[llength [get_board_parts -quiet $board]] == 0} {
    set found [lsort -dictionary [get_board_parts -quiet -filter {PART_NAME == xc7z020clg484-1} *zed*]]
    if {[llength $found] == 0} { error "No Zedboard board files found (see ../../Packet_Detector/HW/build_hw.tcl)" }
    set board [lindex $found end]
}
set_property board_part $board [current_project]
set_property ip_repo_paths [list [file join $adi library]] [current_project]
update_ip_catalog
set v_ad9361 [adi_vlnv axi_ad9361]
set v_dmac   [adi_vlnv axi_dmac]
note "IP: $v_ad9361, $v_dmac"

add_files -norecurse [file join $script_dir fmcomms3 fmc_glue.v]
update_compile_order -fileset sources_1
create_bd_design $bd_name

# --------------------------------------------------------------------------
# Zynq PS
# --------------------------------------------------------------------------
set ps7 [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 ps7]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps7
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP2 {1} \
    CONFIG.PCW_S_AXI_HP2_DATA_WIDTH {64} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_EN_RST0_PORT {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_EN_CLK1_PORT {1} \
    CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {200} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR {1} \
    CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {1} \
    CONFIG.PCW_GPIO_EMIO_GPIO_IO {64} \
    CONFIG.PCW_SPI0_PERIPHERAL_ENABLE {1} \
    CONFIG.PCW_SPI0_SPI0_IO {EMIO} \
] $ps7
if {$with_pd} {
    # AXI DMA MM2S reads the ring on HP0 (32 bit, as in build_hw.tcl)
    set_property -dict [list CONFIG.PCW_USE_S_AXI_HP0 {1} CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {32}] $ps7
}
# Ethernet PHY pins, as Packet_Detector/HW/build_hw.tcl (MDIO must be on MIO 52..53)
set_property -dict [list \
    CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {1} \
    CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27} \
    CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {1} \
    CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53} \
] $ps7
foreach {param want} {PCW_ENET0_ENET0_IO {MIO 16 .. 27} PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53}
                      PCW_GPIO_EMIO_GPIO_IO 64 PCW_SPI0_SPI0_IO EMIO PCW_FPGA1_PERIPHERAL_FREQMHZ 200} {
    set got [get_property CONFIG.$param $ps7]
    if {$got ne $want} { error "$param is '$got', expected '$want'" }
}

set clk   [pin ps7/FCLK_CLK0]
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_100]
connect_bd_net [pin ps7/FCLK_RESET0_N] [pin rst_100/ext_reset_in]
connect_bd_net $clk [pin rst_100/slowest_sync_clk]
set aresetn  [pin rst_100/peripheral_aresetn]
set icresetn [pin rst_100/interconnect_aresetn]

# --------------------------------------------------------------------------
# axi_ad9361: LVDS, one Rx channel, everything the AD9361 does itself left out
# --------------------------------------------------------------------------
# ADC_INIT_DELAY (IDELAY start value) is carrier-specific: take ADI's value for zed
set init_delay 23
set found_delay 0
foreach f [list [file join $adi projects fmcomms2 zed system_bd.tcl] \
                [file join $adi projects fmcomms2 common fmcomms2_bd.tcl]] {
    if {![file exists $f]} continue
    set fh [open $f]; set t [read $fh]; close $fh
    if {[regexp {axi_ad9361\s+CONFIG\.ADC_INIT_DELAY\s+(\d+)} $t -> d]} { set init_delay $d; set found_delay 1; break }
}
if {!$found_delay} { note "ADC_INIT_DELAY not found in ADI's fmcomms2 scripts, using $init_delay" }

set ad [create_bd_cell -type ip -vlnv $v_ad9361 axi_ad9361]
set_params $ad 1 [list ID 0 MODE_1R1T 1]
set_params $ad 0 [list \
    ADC_INIT_DELAY $init_delay \
    ADC_DCFILTER_DISABLE 1 \
    ADC_IQCORRECTION_DISABLE 1 \
    DAC_DDS_DISABLE 1 \
    DAC_IQCORRECTION_DISABLE 1 \
    TDD_DISABLE 1 \
]
# Kept for now: ADC_DATAFORMAT (gives the sign-extended 12-bit word) and the DAC datapath
# (its PN generator is used by the digital interface tune on the Tx side).

# FMC ports, names as in ADI's fmcomms2/zed system_constr.xdc
set lvds_in  {rx_clk_in_p rx_clk_in_n rx_frame_in_p rx_frame_in_n}
set lvds_inb {rx_data_in_p rx_data_in_n}
set lvds_out {tx_clk_out_p tx_clk_out_n tx_frame_out_p tx_frame_out_n enable txnrx}
set lvds_outb {tx_data_out_p tx_data_out_n}
foreach p $lvds_in  { create_bd_port -dir I $p;             connect_bd_net [get_bd_ports $p] [pin axi_ad9361/$p] }
foreach p $lvds_inb { create_bd_port -dir I -from 5 -to 0 $p; connect_bd_net [get_bd_ports $p] [pin axi_ad9361/$p] }
foreach p $lvds_out { create_bd_port -dir O $p;             connect_bd_net [pin axi_ad9361/$p] [get_bd_ports $p] }
foreach p $lvds_outb { create_bd_port -dir O -from 5 -to 0 $p; connect_bd_net [pin axi_ad9361/$p] [get_bd_ports $p] }

set l_clk [pin axi_ad9361/l_clk]
connect_bd_net $l_clk [pin axi_ad9361/clk]
connect_bd_net [pin ps7/FCLK_CLK1] [pin axi_ad9361/delay_clk]

# --------------------------------------------------------------------------
# axi_dmac: FIFO (l_clk, 32 bit) -> AXI3 64 bit on HP2
# --------------------------------------------------------------------------
set dma [create_bd_cell -type ip -vlnv $v_dmac axi_ad9361_adc_dma]
set_params $dma 1 [list \
    DMA_TYPE_SRC 2 \
    DMA_TYPE_DEST 0 \
    DMA_DATA_WIDTH_SRC 32 \
    DMA_DATA_WIDTH_DEST 64 \
]
set_params $dma 0 [list \
    DMA_AXI_PROTOCOL_DEST 1 \
    SYNC_TRANSFER_START 0 \
    CYCLIC 1 \
    DMA_2D_TRANSFER 0 \
    DMA_LENGTH_WIDTH 24 \
]

# --------------------------------------------------------------------------
# Glue: GPIO bits, SPI ties, {Q0, I0}
# --------------------------------------------------------------------------
create_bd_cell -type module -reference fmc_glue fmc_glue
connect_bd_net [pin ps7/GPIO_O] [pin fmc_glue/gpio_o]
connect_bd_net [pin fmc_glue/gpio_i] [pin ps7/GPIO_I]
connect_bd_net [pin fmc_glue/up_enable] [pin axi_ad9361/up_enable]
connect_bd_net [pin fmc_glue/up_txnrx]  [pin axi_ad9361/up_txnrx]
create_bd_port -dir I -from 7 -to 0 gpio_status
create_bd_port -dir O -from 3 -to 0 gpio_ctl
foreach p {gpio_en_agc gpio_sync gpio_resetb} { create_bd_port -dir O $p }
connect_bd_net [get_bd_ports gpio_status] [pin fmc_glue/gpio_status]
foreach p {gpio_ctl gpio_en_agc gpio_sync gpio_resetb} { connect_bd_net [pin fmc_glue/$p] [get_bd_ports $p] }

# SPI0 on EMIO -> AD9361
foreach p {spi_csn spi_clk spi_mosi} { create_bd_port -dir O $p }
create_bd_port -dir I spi_miso
connect_bd_net [pin ps7/SPI0_SS_O]   [get_bd_ports spi_csn]
connect_bd_net [pin ps7/SPI0_SCLK_O] [get_bd_ports spi_clk]
connect_bd_net [pin ps7/SPI0_MOSI_O] [get_bd_ports spi_mosi]
connect_bd_net [get_bd_ports spi_miso] [pin ps7/SPI0_MISO_I]
connect_bd_net [pin fmc_glue/spi0_ss_i]   [pin ps7/SPI0_SS_I]
connect_bd_net [pin fmc_glue/spi0_sclk_i] [pin ps7/SPI0_SCLK_I]
connect_bd_net [pin fmc_glue/spi0_mosi_i] [pin ps7/SPI0_MOSI_I]

# samples -> DMA
connect_bd_net [pin axi_ad9361/adc_data_i0] [pin fmc_glue/adc_data_i0]
connect_bd_net [pin axi_ad9361/adc_data_q0] [pin fmc_glue/adc_data_q0]
connect_bd_net $l_clk                        [pin axi_ad9361_adc_dma/fifo_wr_clk]
connect_bd_net [pin axi_ad9361/adc_valid_i0] [pin axi_ad9361_adc_dma/fifo_wr_en]
connect_bd_net [pin fmc_glue/adc_data_iq]    [pin axi_ad9361_adc_dma/fifo_wr_din]
# fifo_wr_sync exists only with SYNC_TRANSFER_START = 1 (not in this design)
if {[get_bd_pins -quiet axi_ad9361_adc_dma/fifo_wr_sync] ne ""} {
    connect_bd_net [pin fmc_glue/dma_wr_sync] [pin axi_ad9361_adc_dma/fifo_wr_sync]
}
connect_bd_net [pin axi_ad9361_adc_dma/fifo_wr_overflow] [pin axi_ad9361/adc_dovf]
connect_bd_net [pin axi_ad9361_adc_dma/irq] [pin ps7/IRQ_F2P]

# --------------------------------------------------------------------------
# pd_0 (the FlexLink receiver) fed by the AXI DMA from the ring, as in build_hw.tcl
# --------------------------------------------------------------------------
if {$with_pd} {
    # RTL file list: taken from build_hw.tcl, so both builds always use the same files
    set fh [open [file join $pd_dir HW build_hw.tcl]]; set t [read $fh]; close $fh
    if {![regexp {foreach f \{([^\}]*)\} \{\s*lappend rtl_files} $t -> rtl_names]} {
        error "RTL file list not found in [file join $pd_dir HW build_hw.tcl]"
    }
    set rtl_dir [file join $pd_dir RTL]
    set rtl_files {}
    foreach f [string map {"\n" " "} $rtl_names] {
        set path [file join $rtl_dir $f]
        if {![file exists $path]} { error "$path not found" }
        lappend rtl_files $path
    }
    add_files -norecurse $rtl_files
    add_files -norecurse [file join $rtl_dir pl_coef_base.vh]
    set_property file_type {Verilog Header} [get_files pl_coef_base.vh]
    set_property include_dirs [list $rtl_dir] [current_fileset]
    update_compile_order -fileset sources_1
    note "pd_0: [llength $rtl_files] RTL files from [file join $pd_dir RTL]"

    set dma0 [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma axi_dma_0]
    set_property -dict [list \
        CONFIG.c_include_sg {0} \
        CONFIG.c_sg_include_stscntrl_strm {0} \
        CONFIG.c_sg_length_width {26} \
        CONFIG.c_include_mm2s {1} \
        CONFIG.c_include_s2mm {0} \
        CONFIG.c_m_axi_mm2s_data_width {32} \
        CONFIG.c_m_axis_mm2s_tdata_width {32} \
        CONFIG.c_mm2s_burst_size {64} \
        CONFIG.c_addr_width {32} \
    ] $dma0
    create_bd_cell -type module -reference pd_axi pd_0
    connect_bd_intf_net [intf axi_dma_0/M_AXIS_MM2S] [intf pd_0/S_AXIS]
    # debug records: no S2MM here; always ready, so pd_0 can never stall on them (it sends
    # records only with CTRL.DEBUG or CTRL.LOOPBACK, which this variant's firmware does not set)
    connect_bd_net [pin fmc_glue/pd_rec_tready] [pin pd_0/m_axis_tready]

    set pc_rd [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_protocol_converter pc_rd]
    connect_bd_intf_net [intf axi_dma_0/M_AXI_MM2S] [intf pc_rd/S_AXI]
    connect_bd_intf_net [intf pc_rd/M_AXI]          [intf ps7/S_AXI_HP0]
}

# --------------------------------------------------------------------------
# AXI: GP0 -> registers, DMA -> HP2
# --------------------------------------------------------------------------
set sc_ctrl [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc_ctrl]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI [expr {$with_pd ? 4 : 2}]] $sc_ctrl
connect_bd_intf_net [intf ps7/M_AXI_GP0]    [intf sc_ctrl/S00_AXI]
connect_bd_intf_net [intf sc_ctrl/M00_AXI] [intf axi_ad9361/s_axi]
connect_bd_intf_net [intf sc_ctrl/M01_AXI] [intf axi_ad9361_adc_dma/s_axi]
connect_bd_intf_net [intf axi_ad9361_adc_dma/m_dest_axi] [intf ps7/S_AXI_HP2]
if {$with_pd} {
    connect_bd_intf_net [intf sc_ctrl/M02_AXI] [intf pd_0/S_AXI]
    connect_bd_intf_net [intf sc_ctrl/M03_AXI] [intf axi_dma_0/S_AXI_LITE]
}

connect_bd_net $clk \
    [pin ps7/M_AXI_GP0_ACLK] [pin ps7/S_AXI_HP2_ACLK] [pin sc_ctrl/aclk] \
    [pin axi_ad9361/s_axi_aclk] \
    [pin axi_ad9361_adc_dma/s_axi_aclk] [pin axi_ad9361_adc_dma/m_dest_axi_aclk]
connect_bd_net $aresetn \
    [pin axi_ad9361/s_axi_aresetn] \
    [pin axi_ad9361_adc_dma/s_axi_aresetn] [pin axi_ad9361_adc_dma/m_dest_axi_aresetn]
connect_bd_net $icresetn [pin sc_ctrl/aresetn]
if {$with_pd} {
    connect_bd_net $clk [pin ps7/S_AXI_HP0_ACLK] [pin pd_0/aclk] [pin pc_rd/aclk] \
        [pin axi_dma_0/s_axi_lite_aclk] [pin axi_dma_0/m_axi_mm2s_aclk]
    connect_bd_net $aresetn [pin pd_0/aresetn] [pin axi_dma_0/axi_resetn]
    connect_bd_net $icresetn [pin pc_rd/aresetn]
}

# --------------------------------------------------------------------------
# Addresses (as ADI's design), validate, wrapper
# --------------------------------------------------------------------------
set space [get_bd_addr_spaces ps7/Data]
assign_bd_address -offset 0x79020000 -range 64K -target_address_space $space [get_bd_addr_segs axi_ad9361/s_axi/*]
assign_bd_address -offset 0x7C400000 -range 4K  -target_address_space $space [get_bd_addr_segs axi_ad9361_adc_dma/s_axi/*]
if {$with_pd} {
    assign_bd_address -offset 0x43C00000 -range 4K -target_address_space $space [get_bd_addr_segs pd_0/S_AXI/*]
}
assign_bd_address
set segs [get_bd_addr_segs -of_objects $space]
foreach {want match skip} {0x79020000 *SEG_axi_ad9361_* *adc_dma* 0x7C400000 *SEG_axi_ad9361_adc_dma_* {}} {
    set got ?
    foreach s $segs {
        if {[string match $match $s] && ($skip eq "" || ![string match $skip $s])} { set got [get_property OFFSET $s] }
    }
    if {$got eq "?" || $got != $want} { error "registers matching $match at $got, expected $want" }
}
if {$with_pd} {
    set got ?
    foreach s $segs { if {[string match *pd_0* $s]} { set got [get_property OFFSET $s] } }
    if {$got eq "?" || $got != 0x43C00000} { error "pd_0 registers at $got, the firmware expects 0x43C00000" }
}
regenerate_bd_layout
validate_bd_design
save_bd_design

set bd_file [get_files ${bd_name}.bd]
add_files -norecurse [make_wrapper -files $bd_file -top]
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

# Pins: ADI's constraints, only the lines for this design's ports
set ports [lsort -unique [get_property NAME [get_bd_ports]]]
set xdc [file join $build_dir fmc_pins.xdc]
lassign [filter_xdc $adi_xdc $xdc $ports] kept dropped
set fh [open [file join $build_dir fmc_pins_dropped.txt] w]
puts $fh [join $dropped \n]; close $fh
note "fmc_pins.xdc: $kept lines kept from ADI's system_constr.xdc, [llength $dropped] dropped (fmc_pins_dropped.txt)"
set fh [open $xdc]; set xt [read $fh]; close $fh
if {![regexp -line {^create_clock.*-period\s+([0-9.]+)\s.*rx_clk_in_p} $xt -> per]} {
    error "no create_clock on rx_clk_in_p in $xdc - check ADI's system_constr.xdc"
}
note "rx_clk constraint: $per ns"
add_files -fileset constrs_1 -norecurse $xdc

foreach p {ID MODE_1R1T ADC_INIT_DELAY ADC_DCFILTER_DISABLE ADC_IQCORRECTION_DISABLE DAC_DDS_DISABLE
           DAC_IQCORRECTION_DISABLE TDD_DISABLE FPGA_TECHNOLOGY FPGA_FAMILY SPEED_GRADE DEV_PACKAGE} {
    if {[lsearch -exact [list_property $ad CONFIG.*] CONFIG.$p] >= 0} {
        note "axi_ad9361 $p = [get_property CONFIG.$p $ad]"
    }
}

catch {
    puts "Address map:"
    foreach seg [get_bd_addr_segs -of_objects $space] {
        set line [format "  %-50s 0x%08X  %s" $seg [get_property OFFSET $seg] [get_property RANGE $seg]]
        puts $line; lappend notes $line
    }
}

proc write_summary {} {
    global build_dir notes
    set fh [open [file join $build_dir summary.txt] w]
    puts $fh "build_hw_fmcomms3.tcl [clock format [clock seconds] -format {%Y-%m-%d %H:%M}], Vivado [version -short]"
    puts $fh [join $notes \n]
    close $fh
}

if {$mode eq "bd"} {
    write_summary
    puts "Block design created: [file join $build_dir $proj_name ${proj_name}.xpr]"
    return
}

# --------------------------------------------------------------------------
# Synthesis, pin check
# --------------------------------------------------------------------------
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    error "Synthesis failed - see [get_property DIRECTORY [get_runs synth_1]]/runme.log"
}
open_run synth_1 -name synth_1
set unplaced {}
foreach p [get_ports] {
    if {[string match DDR_* $p] || [string match FIXED_IO_* $p] || [string match *_n $p] || [string match *_n\[* $p]} continue
    if {[get_property PACKAGE_PIN $p] eq ""} { lappend unplaced $p }
}
if {[llength $unplaced] > 0} {
    write_summary
    error "No pin for: $unplaced. Port names differ from ADI's system_constr.xdc: compare with\
           build_fmcomms3/fmc_pins_dropped.txt and rename the ports in this script."
}
report_utilization -file [file join $build_dir utilization_synth.rpt]
report_utilization -hierarchical -hierarchical_depth 3 -file [file join $build_dir utilization_synth_hier.rpt]
close_design
if {$mode eq "synth"} {
    write_summary
    puts "Synthesis done: build_fmcomms3/utilization_synth.rpt, utilization_synth_hier.rpt"
    return
}

# --------------------------------------------------------------------------
# Implementation and export
# --------------------------------------------------------------------------
if {$with_pd} {
    # timing-driven implementation, as build_hw.tcl (the PL is about 93 % full)
    set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE $place_dir [get_runs impl_1]
    note "placement directive: $place_dir"
    set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
    set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
    set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
    set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
    set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
}
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    write_summary
    error "Implementation failed - see [get_property DIRECTORY [get_runs impl_1]]/runme.log"
}
open_run impl_1
report_timing_summary -file [file join $build_dir timing_summary.rpt]
report_clock_interaction -file [file join $build_dir clock_interaction.rpt]
report_timing -setup -max_paths 40 -nworst 1 -path_type summary -file [file join $build_dir timing_paths.rpt]
report_utilization -file [file join $build_dir utilization.rpt]
report_utilization -hierarchical -hierarchical_depth 4 -file [file join $build_dir utilization_hier.rpt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set u [report_utilization -return_string]
set lut ?; set ff ?; set bram ?; set dsp ?
regexp {\| Slice LUTs\s+\|\s+(\d+)}      $u -> lut
regexp {\| Slice Registers\s+\|\s+(\d+)} $u -> ff
regexp {\| Block RAM Tile\s+\|\s+([\d.]+)} $u -> bram
regexp {\| DSPs\s+\|\s+(\d+)}            $u -> dsp
note "WNS $wns ns"
note "Used: LUT $lut of 53200, FF $ff of 106400, BRAM tiles $bram of 140, DSP $dsp of 220"
if {$wns < 0} { note "WARNING: timing not met - see build_fmcomms3/timing_summary.rpt" }

set xsa [file join $build_dir ${proj_name}.xsa]
write_hw_platform -fixed -include_bit -force -file $xsa
write_summary
puts "Done: $xsa"
puts [join $notes \n]
