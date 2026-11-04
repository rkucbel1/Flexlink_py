// tb_ptx_core.v - ptx_core against ptx_reference.packet (vectors from make_core_vectors.py).
// Input words with pseudo-random tvalid gaps; m_ready like a DAC FIFO (out_mode 1: 2 of 5 clocks,
// 40 Msps at 100 MHz, with jitter; 0: always ready; >= 2: 40 Msps for a core clock of out_mode MHz). Every output sample is compared in order.
// Run: ./run_sim.sh core [mix|default|big] [seed] [in_gap] [out_mode]
`timescale 1ns / 1ps
module tb_ptx_core;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #5 clk = ~clk;

  reg  [63:0] s_tdata = 64'd0;
  reg         s_tvalid = 1'b0;
  wire        s_tready;
  wire        m_valid, m_active, busy;
  reg         m_ready = 1'b0;
  wire [11:0] m_i, m_q;
  wire [31:0] st_packets, st_bubbles;
  wire [15:0] st_hdr_errors;

  ptx_core dut (.clk(clk), .rst_n(rst_n), .s_tdata(s_tdata), .s_tvalid(s_tvalid), .s_tready(s_tready),
                .m_valid(m_valid), .m_ready(m_ready), .m_i(m_i), .m_q(m_q), .m_active(m_active),
                .st_packets(st_packets), .st_hdr_errors(st_hdr_errors), .st_bubbles(st_bubbles),
                .st_busy(busy));

  integer fw, fe, fc, r, n_words, n_samp, seed, in_gap, out_mode, n_pkt, n_err_exp;
  integer wi_, si_, errors, ei, eq, ea, cyc, gapc;
  reg [63:0] w;
  reg [31:0] lfsr_i = 32'h1357_9BDF, lfsr_o = 32'h2468_ACE0;
  reg [2:0]  phase = 3'd0;
  integer    acc = 0;
  reg        done = 1'b0;

  initial begin
    fc = $fopen("work/core/cfg.txt", "r");
    r = $fscanf(fc, "%d %d %d %d %d %d %d", n_words, n_samp, seed, in_gap, out_mode, n_pkt, n_err_exp);
    fw = $fopen("work/core/words.txt", "r");
    fe = $fopen("work/core/exp.txt", "r");
    lfsr_i = lfsr_i ^ seed; lfsr_o = lfsr_o ^ (seed * 7);
    errors = 0; wi_ = 0; si_ = 0; gapc = 0;
    repeat (5) @(posedge clk);
    rst_n <= 1'b1;
  end

  // input driver
  always @(posedge clk) if (rst_n) begin
    if (s_tvalid && s_tready) begin
      s_tvalid <= 1'b0;
      wi_ = wi_ + 1;
    end
    if ((!s_tvalid || s_tready) && wi_ < n_words) begin
      lfsr_i = {lfsr_i[30:0], lfsr_i[31] ^ lfsr_i[21] ^ lfsr_i[1] ^ lfsr_i[0]};
      if (gapc > 0) begin
        gapc = gapc - 1;
      end else begin
        r = $fscanf(fw, "%h", w);
        s_tdata <= w;
        s_tvalid <= 1'b1;
        gapc = (in_gap > 0 && lfsr_i[3:0] > 4'd11) ? (lfsr_i[9:4] % (in_gap + 1)) : 0;
      end
    end
  end

  // output side
  always @(posedge clk) if (rst_n) begin
    lfsr_o = {lfsr_o[30:0], lfsr_o[31] ^ lfsr_o[21] ^ lfsr_o[1] ^ lfsr_o[0]};
    if (out_mode == 0) m_ready <= 1'b1;
    else if (out_mode >= 2) begin         // out_mode = core clock in MHz: exactly 40 Msps, +-1 clock jitter
      acc = acc + 40;
      m_ready <= (acc >= out_mode) ^ (lfsr_o[4:0] == 5'd0 && acc >= out_mode - 40 && acc < out_mode + 40);
      if (acc >= out_mode) acc = acc - out_mode;
    end else begin
      phase <= (phase == 3'd4) ? 3'd0 : phase + 1'b1;
      m_ready <= ((phase == 3'd1) || (phase == 3'd3)) ^ (lfsr_o[4:0] == 5'd0);
    end
    if (m_valid && m_ready && !done) begin
      r = $fscanf(fe, "%d %d %d", ei, eq, ea);
      if ($signed(m_i) !== ei || $signed(m_q) !== eq || m_active !== ea[0]) begin
        if (errors < 12)
          $display("sample %0d: got %0d %0d active %0d, expected %0d %0d %0d", si_, $signed(m_i),
                   $signed(m_q), m_active, ei, eq, ea);
        errors = errors + 1;
      end
      si_ = si_ + 1;
      if (si_ == n_samp) done <= 1'b1;
    end
  end

  initial begin
    cyc = 0;
    while (!done && cyc < 400 * n_samp / 10 + 200000) begin
      @(posedge clk);
      cyc = cyc + 1;
    end
    repeat (50) @(posedge clk);
    if (!done) begin
      $display("FAIL: timeout after %0d cycles, %0d of %0d samples, %0d of %0d words taken, packets %0d",
               cyc, si_, n_samp, wi_, n_words, st_packets);
      errors = errors + 1;
    end
    if (m_valid) begin $display("FAIL: extra samples after the expected ones"); errors = errors + 1; end
    if (st_packets !== n_pkt) begin $display("FAIL: st_packets %0d, expected %0d", st_packets, n_pkt); errors = errors + 1; end
    if (st_hdr_errors !== n_err_exp) begin
      $display("FAIL: st_hdr_errors %0d, expected %0d", st_hdr_errors, n_err_exp); errors = errors + 1;
    end
    if (busy) begin $display("FAIL: still busy at the end"); errors = errors + 1; end
    if (errors == 0)
      $display("PASS: %0d packets, %0d samples equal (%0d cycles, %0d output bubbles, %0d header errors)",
               n_pkt, n_samp, cyc, st_bubbles, st_hdr_errors);
    else
      $display("FAIL: %0d errors", errors);
    $finish;
  end
`ifdef DUMP_BINS
  // debug: IFFT input bins (ce count, xr, xi) of the first symbols
  integer fdb, ndb;
  initial begin fdb = $fopen("work/core/dbg_bins.txt", "w"); ndb = 0; end
  always @(posedge clk) if (dut.ce && ndb < 6 * 2048 + 2) begin
    $fwrite(fdb, "%0d %0d\n", dut.f2_r, dut.f2_i);
    ndb = ndb + 1;
  end
`endif
endmodule
