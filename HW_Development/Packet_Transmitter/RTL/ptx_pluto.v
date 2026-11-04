// ptx_pluto.v - ptx_core in the ADALM-Pluto design (ADI hdl/projects/pluto), between the TX DMA and
// the AD9361 core. See ../HW/ptx_pluto_bd.tcl for the block-design changes.
//
//   axi_ad9361_dac_dma/m_axis --> s_dma --+--> m_upack --> tx_upack/s_axis       (enable = 0: stock)
//                                         +--> ptx_core  (enable = 1)
//   tx_fir_interpolator/data_out_0/1 --> dac_in_i0/q0 --+--> dac_out_i0/q0 --> axi_ad9361/dac_data_i0/q0
//   ptx_core samples (12 bit, {s, 4'b0}) ---------------+  (enable = 1)
//
// Control: axi_ad9361 DAC GPIO out (register 0x40BC of cf-ad9361-dds-core-lpc, up_clk domain):
//   bit 4  enable the Flexlink transmitter (0: stock data path; 1: the DMA stream is records -
//          parameter header + transport blocks - for ptx_core, see ptx_core.v). Going to 0 resets
//          the core and empties its FIFOs.
//   (bit 0 stays the stock FIR interpolator enable)
// Status: axi_ad9361 DAC GPIO in (register 0x40B8):
//   [15:0]  packets sent (mod 2^16)    [23:16] header errors (saturating)
//   [27:24] DAC underflows inside a packet (saturating)   [28] core busy   [29] enabled   [31:30] 0
//
// Clocks: clk (sys_cpu_clk, 100 MHz) runs the core; l_clk is the AD9361 interface clock: DMA words
// arrive in it and samples leave in it, one per dac_valid. Two small asynchronous FIFOs cross.
`timescale 1ns / 1ps
module ptx_pluto (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_RESET resetn" *)
  input  wire        clk,                 // core clock (sys_cpu_clk)
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 resetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        resetn,              // sys_cpu_resetn
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 l_clk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_dma:m_upack, ASSOCIATED_RESET l_rst" *)
  input  wire        l_clk,               // axi_ad9361/l_clk
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 l_rst RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
  input  wire        l_rst,               // axi_ad9361/rst (active high)
  // TX DMA stream (l_clk)
  input  wire [63:0] s_dma_tdata,
  input  wire        s_dma_tvalid,
  output wire        s_dma_tready,
  input  wire        s_dma_tlast,
  // to tx_upack (l_clk)
  output wire [63:0] m_upack_tdata,
  output wire        m_upack_tvalid,
  input  wire        m_upack_tready,
  output wire        m_upack_tlast,
  // DAC data (l_clk)
  input  wire        dac_valid_i0,
  input  wire [15:0] dac_in_i0,
  input  wire [15:0] dac_in_q0,
  output reg  [15:0] dac_out_i0,
  output reg  [15:0] dac_out_q0,
  // control / status (up_clk = clk)
  input  wire [31:0] up_dac_gpio_out,
  output wire [31:0] up_dac_gpio_in
);
  // enable, synchronised into both domains
  wire en_c = up_dac_gpio_out[4];
  (* ASYNC_REG = "TRUE" *) reg [2:0] en_l_s = 3'd0;
  always @(posedge l_clk) en_l_s <= {en_l_s[1:0], en_c};
  wire en_l = en_l_s[2];
  reg  core_rst_n = 1'b0;
  always @(posedge clk) core_rst_n <= resetn && en_c;

  // stock path when disabled
  assign m_upack_tdata  = s_dma_tdata;
  assign m_upack_tlast  = s_dma_tlast;
  assign m_upack_tvalid = s_dma_tvalid && !en_l;

  // DMA words l_clk -> clk
  wire        wf_full, wf_empty;
  wire [63:0] wf_dout;
  wire        wf_rd;
  assign s_dma_tready = en_l ? !wf_full : m_upack_tready;
  ptx_afifo #(.W(64), .AW(4)) wfifo (
    .wclk(l_clk), .wrst(!en_l), .we(en_l && s_dma_tvalid && !wf_full), .din(s_dma_tdata), .full(wf_full),
    .rclk(clk), .rrst(!core_rst_n), .re(wf_rd), .dout(wf_dout), .empty(wf_empty));

  // core
  wire        c_ready;
  wire        c_valid, c_active, c_busy;
  wire [11:0] c_i, c_q;
  wire [31:0] c_packets, c_bubbles;
  wire [15:0] c_hdr;
  wire        sf_full;
  assign wf_rd = !wf_empty && c_ready;
  ptx_core core (.clk(clk), .rst_n(core_rst_n), .s_tdata(wf_dout), .s_tvalid(!wf_empty), .s_tready(c_ready),
                 .m_valid(c_valid), .m_ready(!sf_full), .m_i(c_i), .m_q(c_q), .m_active(c_active),
                 .st_packets(c_packets), .st_hdr_errors(c_hdr), .st_bubbles(c_bubbles), .st_busy(c_busy));

  // samples clk -> l_clk, one per dac_valid_i0
  wire        sf_empty;
  wire [24:0] sf_dout;
  wire        take = en_l && dac_valid_i0;
  ptx_afifo #(.W(25), .AW(4)) sfifo (
    .wclk(clk), .wrst(!core_rst_n), .we(c_valid && !sf_full), .din({c_active, c_i, c_q}), .full(sf_full),
    .rclk(l_clk), .rrst(!en_l), .re(take && !sf_empty), .dout(sf_dout), .empty(sf_empty));

  // underflow inside a packet: the last sample taken was active and the FIFO is empty at a dac_valid
  reg        last_active = 1'b0;
  reg [3:0]  unf_l = 4'd0;
  always @(posedge l_clk) begin
    if (!en_l) begin
      dac_out_i0 <= dac_in_i0; dac_out_q0 <= dac_in_q0; last_active <= 1'b0; unf_l <= 4'd0;
    end else if (dac_valid_i0) begin
      if (!sf_empty) begin
        dac_out_i0 <= {sf_dout[23:12], 4'd0};
        dac_out_q0 <= {sf_dout[11:0], 4'd0};
        last_active <= sf_dout[24];
      end else begin
        dac_out_i0 <= 16'd0; dac_out_q0 <= 16'd0;
        if (last_active && unf_l != 4'hF) unf_l <= unf_l + 1'b1;
      end
    end
  end
  // the counter changes rarely; sample it twice in clk (status only)
  (* ASYNC_REG = "TRUE" *) reg [3:0] unf_c1 = 4'd0;
  (* ASYNC_REG = "TRUE" *) reg [3:0] unf_c2 = 4'd0;
  always @(posedge clk) begin unf_c1 <= unf_l; unf_c2 <= unf_c1; end

  wire [7:0] hdr8 = (c_hdr > 16'd255) ? 8'hFF : c_hdr[7:0];
  assign up_dac_gpio_in = {2'b00, en_c, c_busy, unf_c2, hdr8, c_packets[15:0]};
endmodule

// ---------------------------------------------------------------------------------------------
// asynchronous FIFO, 2^AW entries, Gray-coded pointers, first word fall through (dout valid
// while !empty). Each side has its own synchronous reset (assert both together).
module ptx_afifo #(parameter W = 64, parameter AW = 4) (
  input  wire         wclk,
  input  wire         wrst,
  input  wire         we,
  input  wire [W-1:0] din,
  output wire         full,
  input  wire         rclk,
  input  wire         rrst,
  input  wire         re,
  output wire [W-1:0] dout,
  output wire         empty
);
  reg [W-1:0] mem [0:(1 << AW) - 1];
  reg [AW:0] wb = 0, wg = 0, rb = 0, rg = 0;           // binary and Gray pointers
  (* ASYNC_REG = "TRUE" *) reg [AW:0] rg_w1 = 0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] rg_w2 = 0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] wg_r1 = 0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] wg_r2 = 0;
  wire [AW:0] wb_n = wb + 1'b1;
  wire [AW:0] rb_n = rb + 1'b1;
  always @(posedge wclk) begin
    if (wrst) begin
      wb <= 0; wg <= 0; rg_w1 <= 0; rg_w2 <= 0;
    end else begin
      rg_w1 <= rg; rg_w2 <= rg_w1;
      if (we && !full) begin
        mem[wb[AW-1:0]] <= din;
        wb <= wb_n;
        wg <= wb_n ^ (wb_n >> 1);
      end
    end
  end
  always @(posedge rclk) begin
    if (rrst) begin
      rb <= 0; rg <= 0; wg_r1 <= 0; wg_r2 <= 0;
    end else begin
      wg_r1 <= wg; wg_r2 <= wg_r1;
      if (re && !empty) begin
        rb <= rb_n;
        rg <= rb_n ^ (rb_n >> 1);
      end
    end
  end
  assign full  = (wg == {~rg_w2[AW:AW-1], rg_w2[AW-2:0]});
  assign empty = (rg == wg_r2);
  assign dout  = mem[rb[AW-1:0]];
endmodule
