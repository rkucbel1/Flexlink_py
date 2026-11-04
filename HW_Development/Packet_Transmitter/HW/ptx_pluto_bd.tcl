# ptx_pluto_bd.tcl - put the Flexlink transmitter (RTL/ptx_pluto.v) into the ADI Pluto block design.
#
# Sourced at the end of hdl/projects/pluto/system_bd.tcl (patch_pluto_hdl.sh adds the line), so it
# runs inside adi_project_create with the ADI procs (ad_connect) loaded. It adds the RTL to the
# project, then re-routes:
#   axi_ad9361_dac_dma/m_axis -> ptx_0/s_dma, ptx_0/m_upack -> tx_upack/s_axis
#   tx_fir_interpolator/data_out_0/1 -> ptx_0/dac_in_i0/q0, ptx_0/dac_out_i0/q0 -> axi_ad9361/dac_data_i0/q0
#   axi_ad9361/up_dac_gpio_out -> ptx_0 (bit 4 = enable), ptx_0/up_dac_gpio_in -> axi_ad9361 (status)
# With enable 0 (the reset value) the design behaves exactly like the stock Pluto design.
#
# NOT YET RUN: written against hdl main (Oct 2026, projects/pluto/system_bd.tcl). Check the
# connections in the block design after the first build.

# Make room on the 7010: the stock Pluto axi_ad9361 (MODE_1R1T 0, 2R2T) builds a two-tone CORDIC
# DDS and IQ correction in each of its 4 DAC lanes and a DC filter + IQ correction in each of its 4
# ADC lanes (~8.2k LUTs). None of these are in the DMA data path (dac_data_sel = 2 passes straight
# through; disabled IQ correction is a pass-through), so DMA TX/RX and the transmitter are
# unchanged. Lost: the FPGA DDS test tones (iio altvoltage TX1_I_F1 ...) and FPGA-side
# calibscale/calibphase/DC filter; the AD9361's own on-chip calibrations are unaffected.
ad_ip_parameter axi_ad9361 CONFIG.DAC_DDS_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.DAC_IQCORRECTION_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.ADC_DCFILTER_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.ADC_IQCORRECTION_DISABLE 1
# axi_ad9361's own TDD controller (~700 LUTs, 2.6k FFs): unused on the Pluto (its tdd_sync is
# tied to GND; the separate axi_tdd_0 core stays). Disabled, it drives the FDD values it has anyway.
ad_ip_parameter axi_ad9361 CONFIG.TDD_DISABLE 1

# Implementation: the 7010 is nearly full, and routing detours made most of the delay. Spread the
# logic and run phys_opt after routing as well.
if {[catch {
  set_property strategy Congestion_SpreadLogic_high [get_runs impl_1]
  set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
} err]} { puts "WARNING: ptx_pluto_bd.tcl: implementation strategy not set: $err" }

set ptx_rtl [file normalize [file join [file dirname [info script]] rtl]]
add_files -norecurse [list \
  $ptx_rtl/ptx_pluto.v $ptx_rtl/ptx_core.v $ptx_rtl/ptx_enc.v $ptx_rtl/ptx_ifft.v $ptx_rtl/ptx_roms.v]
update_compile_order -fileset sources_1
set ptx_xdc [file normalize [file join [file dirname [info script]] ptx_pluto.xdc]]
add_files -norecurse -fileset constrs_1 $ptx_xdc
# after system_constr.xdc, which defines the clocks it names
set_property PROCESSING_ORDER LATE [get_files $ptx_xdc]

create_bd_cell -type module -reference ptx_pluto ptx_0

# DMA stream
delete_bd_objs [get_bd_intf_nets -of_objects [get_bd_intf_pins axi_ad9361_dac_dma/m_axis]]
connect_bd_intf_net [get_bd_intf_pins axi_ad9361_dac_dma/m_axis] [get_bd_intf_pins ptx_0/s_dma]
connect_bd_intf_net [get_bd_intf_pins ptx_0/m_upack] [get_bd_intf_pins tx_upack/s_axis]

# DAC data
delete_bd_objs [get_bd_nets -of_objects [get_bd_pins axi_ad9361/dac_data_i0]]
delete_bd_objs [get_bd_nets -of_objects [get_bd_pins axi_ad9361/dac_data_q0]]
ad_connect tx_fir_interpolator/data_out_0 ptx_0/dac_in_i0
ad_connect tx_fir_interpolator/data_out_1 ptx_0/dac_in_q0
ad_connect ptx_0/dac_out_i0 axi_ad9361/dac_data_i0
ad_connect ptx_0/dac_out_q0 axi_ad9361/dac_data_q0
ad_connect axi_ad9361/dac_valid_i0 ptx_0/dac_valid_i0

# clocks, resets, control
ad_connect axi_ad9361/l_clk ptx_0/l_clk
ad_connect axi_ad9361/rst ptx_0/l_rst
ad_connect sys_cpu_clk ptx_0/clk
ad_connect sys_cpu_resetn ptx_0/resetn
ad_connect axi_ad9361/up_dac_gpio_out ptx_0/up_dac_gpio_out
ad_connect ptx_0/up_dac_gpio_in axi_ad9361/up_dac_gpio_in
