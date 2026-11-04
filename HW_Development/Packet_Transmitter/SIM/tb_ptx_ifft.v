// tb_ptx_ifft.v - ptx_ifft against ptx_reference.ifft_fixed (vectors from make_ifft_vectors.py).
// Feeds the symbols with pseudo-random gaps in ce (up to GAP idle clocks), then zero symbols to
// flush, and compares every output slot. Run: ./run_sim.sh ifft
`timescale 1ns / 1ps
module tb_ptx_ifft;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #5 clk = ~clk;

  reg               ce = 1'b0;
  reg signed [17:0] xr = 0, xi = 0;
  wire signed [17:0] yr, yi;
  ptx_ifft dut (.clk(clk), .rst_n(rst_n), .ce(ce), .xr(xr), .xi(xi), .yr(yr), .yi(yi));

  localparam LAT = 2073;
  integer fin, fexp, fcfg, r, nsym, seed, gap;
  integer n_in, n_total, n_out, errors, a, b, g;
  integer er, ei;
  reg [31:0] lfsr = 32'hACE1_2345;

  initial begin
    fcfg = $fopen("work/ifft/cfg.txt", "r");
    r = $fscanf(fcfg, "%d %d %d", nsym, seed, gap);
    fin  = $fopen("work/ifft/in.txt", "r");
    fexp = $fopen("work/ifft/exp.txt", "r");
    lfsr = lfsr ^ seed;
    n_total = (nsym + 2) * 2048;
    errors = 0; n_out = 0;
    repeat (4) @(posedge clk);
    rst_n <= 1'b1;
    @(posedge clk);
    for (n_in = 0; n_in < n_total; n_in = n_in + 1) begin
      // idle clocks
      lfsr = {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
      g = (gap > 0) ? (lfsr[7:0] % (gap + 1)) : 0;
      if (lfsr[11:8] > 4'd9) g = 0;            // mostly back to back
      ce <= 1'b0;
      repeat (g) @(posedge clk);
      if (n_in < nsym * 2048) begin
        r = $fscanf(fin, "%d %d", a, b);
        xr <= a; xi <= b;
      end else begin
        xr <= 0; xi <= 0;
      end
      ce <= 1'b1;
      @(posedge clk);
      // after this edge the output registers hold the result of the ce just taken; the output
      // that belongs to ce number n_in is the value present while that ce is high (checked below)
    end
    ce <= 1'b0;
    @(posedge clk);
    if (n_out != nsym * 2048) begin
      $display("FAIL: %0d outputs checked, expected %0d", n_out, nsym * 2048);
      errors = errors + 1;
    end
    if (errors == 0) $display("PASS: %0d symbols, %0d output slots equal", nsym, n_out);
    else             $display("FAIL: %0d errors", errors);
    $finish;
  end

  // check: on a cycle with ce, yr/yi belong to input count (ce count - LAT) ... the output register
  // of the last stage was updated by the previous ce, so it shows sample (ce_count - 1) - (LAT - 1)
  integer ce_count = 0;
  always @(posedge clk) begin
    if (rst_n && ce) begin
      if (ce_count >= LAT && n_out < nsym * 2048) begin
        r = $fscanf(fexp, "%d %d", er, ei);
        if (yr !== er || yi !== ei) begin
          if (errors < 10)
            $display("slot %0d (symbol %0d, p %0d): got %0d %0d, expected %0d %0d",
                     n_out, n_out / 2048, n_out % 2048, yr, yi, er, ei);
          errors = errors + 1;
        end
        n_out = n_out + 1;
      end
      ce_count = ce_count + 1;
    end
  end
endmodule
