# ptx_pluto.xdc - clock-domain crossings of the Flexlink transmitter wrapper (RTL/ptx_pluto.v).
# clk = sys_cpu_clk (clk_fpga_0, 10 ns), l_clk = AD9361 interface clock (rx_clk, 16.27 ns).
# Added to the Pluto project by ptx_pluto_bd.tcl.

# async FIFOs (wfifo: l_clk -> clk, sfifo: clk -> l_clk)
#   distributed-RAM data, written in one domain and read asynchronously in the other: bound the
#   delay, it is only used after the Gray pointer has crossed (2 destination clocks later)
set_max_delay -datapath_only 8.0 -from [get_pins -hier -filter {NAME =~ */ptx_0/inst/*fifo/mem_reg*/CLK}]
#   Gray pointers into the two-flop synchronizers: bound the skew between the bits
set_max_delay -datapath_only 8.0 \
  -from [get_cells -hier -filter {NAME =~ */ptx_0/inst/*fifo/wg_reg*}] \
  -to   [get_cells -hier -filter {NAME =~ */ptx_0/inst/*fifo/wg_r1_reg*}]
set_max_delay -datapath_only 8.0 \
  -from [get_cells -hier -filter {NAME =~ */ptx_0/inst/*fifo/rg_reg*}] \
  -to   [get_cells -hier -filter {NAME =~ */ptx_0/inst/*fifo/rg_w1_reg*}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {NAME =~ */ptx_0/inst/*fifo/wg_r?_reg*}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {NAME =~ */ptx_0/inst/*fifo/rg_w?_reg*}]

# quasi-static single bits: enable (clk -> l_clk), underflow counter for status (l_clk -> clk, a
# 4-bit counter that changes rarely and is only displayed)
set_false_path -from [get_clocks clk_fpga_0] -to [get_cells -hier -filter {NAME =~ */ptx_0/inst/en_l_s_reg*}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {NAME =~ */ptx_0/inst/en_l_s_reg*}]
set_false_path -from [get_clocks rx_clk] -to [get_cells -hier -filter {NAME =~ */ptx_0/inst/unf_c1_reg*}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {NAME =~ */ptx_0/inst/unf_c?_reg*}]
