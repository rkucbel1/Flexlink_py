// tb_ptx_pluto.v - ptx_pluto with two clocks: core 100 MHz, AD9361 interface clock l_clk 80 MHz with
// dac_valid every second l_clk (40 Msps). DMA words arrive in l_clk with random gaps. Checks every
// sample handed to the DAC against the core vectors (make_core_vectors.py), that the DAC never ran
// dry inside a packet, the status word, and the stock path (enable 0) passing DMA and FIR data.
// Run: ./run_sim.sh pluto [case] [seed]
`timescale 1ns / 1ps
module tb_ptx_pluto;
  reg clk = 0, l_clk = 0;
  always #5 clk = ~clk;
  always #6.25 l_clk = ~l_clk;
  reg resetn = 0, l_rst = 1;
  reg [63:0] s_tdata = 0; reg s_tvalid = 0; wire s_tready;
  wire [63:0] u_tdata; wire u_tvalid, u_tlast;
  reg dac_valid = 0; reg [15:0] fir_i = 16'h1230, fir_q = 16'h4560;
  wire [15:0] dac_i, dac_q;
  reg [31:0] gpio_out = 32'h1;
  wire [31:0] gpio_in;
  ptx_pluto dut (.clk(clk), .resetn(resetn), .l_clk(l_clk), .l_rst(l_rst),
    .s_dma_tdata(s_tdata), .s_dma_tvalid(s_tvalid), .s_dma_tready(s_tready), .s_dma_tlast(1'b0),
    .m_upack_tdata(u_tdata), .m_upack_tvalid(u_tvalid), .m_upack_tready(1'b1), .m_upack_tlast(u_tlast),
    .dac_valid_i0(dac_valid), .dac_in_i0(fir_i), .dac_in_q0(fir_q), .dac_out_i0(dac_i), .dac_out_q0(dac_q),
    .up_dac_gpio_out(gpio_out), .up_dac_gpio_in(gpio_in));

  integer fw, fe, fc, r, n_words, n_samp, seed, in_gap, out_mode, n_pkt, n_err_exp, wi_, si_, errors;
  integer ei, eq, ea, cyc;
  reg [63:0] w; reg [31:0] lf = 32'h9E37_79B9; reg done = 0; reg stock_ok = 0;

  initial begin
    fc = $fopen("work/core/cfg.txt", "r");
    r = $fscanf(fc, "%d %d %d %d %d %d %d", n_words, n_samp, seed, in_gap, out_mode, n_pkt, n_err_exp);
    fw = $fopen("work/core/words.txt", "r"); fe = $fopen("work/core/exp.txt", "r");
    errors = 0; wi_ = 0; si_ = 0; lf = lf ^ seed;
    repeat (10) @(posedge clk); resetn <= 1; l_rst <= 0;
    // stock path: DMA word passes to the upack, FIR data to the DAC
    @(posedge l_clk); s_tdata <= 64'hDEAD_BEEF_0123_4567; s_tvalid <= 1;
    @(posedge l_clk); #1;
    if (u_tvalid !== 1'b1 || u_tdata !== 64'hDEAD_BEEF_0123_4567 || s_tready !== 1'b1) errors = errors + 1;
    s_tvalid <= 0;
    @(posedge l_clk); dac_valid <= 1; @(posedge l_clk); dac_valid <= 0; @(posedge l_clk); #1;
    if (dac_i !== 16'h1230 || dac_q !== 16'h4560) begin $display("FAIL: stock DAC path"); errors = errors + 1; end
    else stock_ok = 1;
    // enable Flexlink
    @(posedge clk); gpio_out <= 32'h11;
    repeat (20) @(posedge clk);
  end
  // DMA driver
  always @(posedge l_clk) if (gpio_out[4] && dut.en_l) begin
    if (s_tvalid && s_tready) begin s_tvalid <= 0; wi_ = wi_ + 1; end
    lf = {lf[30:0], lf[31] ^ lf[21] ^ lf[1] ^ lf[0]};
    if ((!s_tvalid || s_tready) && wi_ < n_words && lf[2:0] != 3'd0) begin
      r = $fscanf(fw, "%h", w); s_tdata <= w; s_tvalid <= 1;
    end
  end
  // DAC: dac_valid every second l_clk
  reg ph = 0;
  always @(posedge l_clk) if (gpio_out[4]) begin ph <= ~ph; dac_valid <= ph; end
  // samples handed to the DAC
  always @(posedge l_clk) if (dut.take && !dut.sf_empty && !done) begin
    r = $fscanf(fe, "%d %d %d", ei, eq, ea);
    if ($signed(dut.sf_dout[23:12]) !== ei || $signed(dut.sf_dout[11:0]) !== eq || dut.sf_dout[24] !== ea[0]) begin
      if (errors < 10) $display("sample %0d: got %0d %0d %0d, expected %0d %0d %0d", si_,
        $signed(dut.sf_dout[23:12]), $signed(dut.sf_dout[11:0]), dut.sf_dout[24], ei, eq, ea);
      errors = errors + 1;
    end
    si_ = si_ + 1;
    if (si_ == n_samp) done <= 1;
  end
  initial begin
    cyc = 0;
    while (!done && cyc < 3 * n_samp + 400000) begin @(posedge clk); cyc = cyc + 1; end
    repeat (100) @(posedge clk);
    if (!done) begin $display("FAIL: timeout, %0d of %0d samples", si_, n_samp); errors = errors + 1; end
    if (gpio_in[15:0] !== n_pkt[15:0]) begin $display("FAIL: packets %0d", gpio_in[15:0]); errors = errors + 1; end
    if (gpio_in[23:16] !== n_err_exp[7:0]) begin $display("FAIL: header errors %0d", gpio_in[23:16]); errors = errors + 1; end
    if (gpio_in[27:24] !== 4'd0) begin $display("FAIL: %0d DAC underflows inside packets", gpio_in[27:24]); errors = errors + 1; end
    if (gpio_in[29] !== 1'b1) begin $display("FAIL: enabled bit"); errors = errors + 1; end
    if (errors == 0) $display("PASS: stock path, %0d packets, %0d samples to the DAC equal, no underflow, status %08x",
                              n_pkt, n_samp, gpio_in);
    else $display("FAIL: %0d errors", errors);
    $finish;
  end
endmodule
