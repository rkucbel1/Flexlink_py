// ptx_ifft.v - 2048-point radix-2^2 single-path delay-feedback (SDF) IFFT, bit-true to
// ../PC/ptx_reference.ifft_fixed().
//
//   y[n] = sum_k X[k] exp(+j 2 pi k n / 2048) / 32
//
// Decimation in frequency: natural-order input, bit-reversed output (output slot p holds y[bitrev(p)]).
// 18-bit data, 18-bit Q16 twiddles. Stage s (0 .. 10) output = sat18(rnd(butterfly, SHIFTS[s])) with
// SHIFTS = 0 0 1 0 1 0 1 0 1 0 1; twiddle products (a*w - b*v + 2^15) >> 16, sat18.
//
// One global clock enable: every register advances only when ce is 1, and each ce takes one input
// sample. So the output is a pure function of the input sequence: the sample taken at ce number
// n >= LAT is output slot (n - LAT) mod 2048 of symbol (n - LAT) / 2048, on the same ce. The last
// symbol comes out while the next symbol (or zeros) goes in. Latency LAT = 2073 samples.
// With IN_OFFSET = k, bin 0 of the first symbol is the sample taken at ce number k (output slot 0 at
// ce number k + LAT).
//
// Resources: 5 complex multipliers (4 DSP48 each), delay lines 1024 / 512 / 256 / 128 x 36 bit in
// block RAM, the rest in distributed RAM or registers.
`timescale 1ns / 1ps

// ---------------------------------------------------------------------------------------------
// delay line: dout = din of D enabled cycles ago, available in the same cycle as the current din
module ptx_delay #(parameter D = 1, parameter W = 36) (
  input  wire         clk,
  input  wire         ce,
  input  wire [W-1:0] din,
  output wire [W-1:0] dout
);
  generate
    if (D == 1) begin : g_reg
      reg [W-1:0] r = {W{1'b0}};
      always @(posedge clk) if (ce) r <= din;
      assign dout = r;
    end else if (D < 64) begin : g_ram
      localparam AW = $clog2(D);
      reg [W-1:0] mem [0:D-1];
      reg [AW-1:0] wa = {AW{1'b0}};
      reg [W-1:0]  q = {W{1'b0}};
      wire [AW-1:0] ra = wa + 1'b1;          // D is a power of two: wraps
      integer i;
      initial for (i = 0; i < D; i = i + 1) mem[i] = {W{1'b0}};
      always @(posedge clk) if (ce) begin
        mem[wa] <= din;
        q <= mem[ra];                         // prefetch the sample needed at the next ce
        wa <= ra;
      end
      assign dout = q;
    end else begin : g_bram
      // block RAM with its output register (timing: BRAM clock-to-out without it is ~2.5 ns):
      // read two ahead, q1 = RAM latch, q2 = output register; dout is still din of D ces ago
      localparam AW = $clog2(D);
      reg [W-1:0] mem [0:D-1];
      reg [AW-1:0] wa = {AW{1'b0}};
      reg [W-1:0]  q1 = {W{1'b0}};
      reg [W-1:0]  q2 = {W{1'b0}};
      wire [AW-1:0] ra = wa + 2'd2;          // D is a power of two: wraps
      integer i;
      initial for (i = 0; i < D; i = i + 1) mem[i] = {W{1'b0}};
      always @(posedge clk) if (ce) begin
        mem[wa] <= din;
        q1 <= mem[ra];
        q2 <= q1;
        wa <= wa + 1'b1;
      end
      assign dout = q2;
    end
  endgenerate
endmodule

// ---------------------------------------------------------------------------------------------
// butterfly stage, distance D. Block position c (mod 4D) of the current input:
//   c mod 2D <  D: store x, output the stored difference
//   c mod 2D >= D: output f + b, store f - b, with b = x, or j*x when TWOJ and c mod 4D >= 2D (BF2II)
// Latency D + 1 (output register).
module ptx_sdf_bf #(
  parameter D    = 1024,
  parameter TWOJ = 0,
  parameter SH   = 0,
  parameter INIT = 0,
  parameter W    = 18
) (
  input  wire                clk,
  input  wire                rst_n,
  input  wire                ce,
  input  wire signed [W-1:0] xr,
  input  wire signed [W-1:0] xi,
  output reg  signed [W-1:0] yr,
  output reg  signed [W-1:0] yi
);
  localparam CW = $clog2(D) + 2;
  localparam signed [W+1:0] MAXV = (1 <<< (W - 1)) - 1;
  localparam signed [W+1:0] MINV = -(1 <<< (W - 1));

  reg [CW-1:0] c;
  wire second = c[CW-2];
  wire use_j  = (TWOJ != 0) && c[CW-1];

  wire signed [W-1:0] fr, fi;
  wire signed [W-1:0] nr, ni;

  // b = x or j*x = (-xi, xr), in W+1 bits (no overflow on -(-2^(W-1)))
  wire signed [W:0] br = use_j ? -{xi[W-1], xi} : {xr[W-1], xr};
  wire signed [W:0] bi = use_j ?  {xr[W-1], xr} : {xi[W-1], xi};
  wire signed [W+1:0] sr = fr + br;
  wire signed [W+1:0] si = fi + bi;
  wire signed [W+1:0] dr = fr - br;
  wire signed [W+1:0] di = fi - bi;

  function signed [W-1:0] rs;              // sat_W(rnd(v, SH))
    input signed [W+1:0] v;
    reg signed [W+1:0] t;
    begin
      t = (SH == 0) ? v : ((v + 1) >>> 1);
      rs = (t > MAXV) ? MAXV[W-1:0] : (t < MINV) ? MINV[W-1:0] : t[W-1:0];
    end
  endfunction

  assign nr = second ? rs(dr) : xr;
  assign ni = second ? rs(di) : xi;

  ptx_delay #(.D(D), .W(2 * W)) dl (.clk(clk), .ce(ce), .din({nr, ni}), .dout({fr, fi}));

  always @(posedge clk) begin
    if (!rst_n) begin
      c  <= INIT[CW-1:0];
      yr <= {W{1'b0}};
      yi <= {W{1'b0}};
    end else if (ce) begin
      c  <= c + 1'b1;
      yr <= second ? rs(sr) : fr;
      yi <= second ? rs(si) : fi;
    end
  end
endmodule

// ---------------------------------------------------------------------------------------------
// twiddle multiplier after the BF2I/BF2II pair of level L: position p = c mod L,
// sub-block q = p / (L/4), n3 = p mod (L/4), factor W_L^(n3 * e), e = 0, 2, 1, 3 for q = 0 .. 3.
// Latency 3: ROM and input register, products, rounding and saturation.
module ptx_sdf_tw #(
  parameter L    = 2048,
  parameter INIT = 0,
  parameter W    = 18
) (
  input  wire                clk,
  input  wire                rst_n,
  input  wire                ce,
  input  wire signed [W-1:0] xr,
  input  wire signed [W-1:0] xi,
  output reg  signed [W-1:0] yr,
  output reg  signed [W-1:0] yi
);
  localparam LW = $clog2(L);
  localparam signed [37:0] MAXV = (1 <<< (W - 1)) - 1;
  localparam signed [37:0] MINV = -(1 <<< (W - 1));

  reg  [LW-1:0] c;
  wire [1:0]    q  = c[LW-1:LW-2];
  wire [10:0]   n3 = {{(11 - (LW - 2)){1'b0}}, c[LW-3:0]};
  wire [10:0]   idx = (q == 2'd0) ? 11'd0 :
                      (q == 2'd1) ? {n3[9:0], 1'b0} :
                      (q == 2'd2) ? n3 :
                                    n3 + {n3[9:0], 1'b0};

  wire signed [17:0] wr, wi;
  ptx_tw_rom #(.L(L)) rom (.clk(clk), .ce(ce), .addr(idx), .wr(wr), .wi(wi));

  reg signed [W-1:0]  ar, ai;
  reg signed [35:0]   p_rr, p_ii, p_ri, p_ir;
  wire signed [37:0]  sre = $signed({{2{p_rr[35]}}, p_rr}) - $signed({{2{p_ii[35]}}, p_ii}) + 38'sd32768;
  wire signed [37:0]  sim = $signed({{2{p_ri[35]}}, p_ri}) + $signed({{2{p_ir[35]}}, p_ir}) + 38'sd32768;
  wire signed [37:0]  tre = sre >>> 16;
  wire signed [37:0]  tim = sim >>> 16;

  always @(posedge clk) begin
    if (!rst_n) begin
      c  <= INIT[LW-1:0];
      ar <= {W{1'b0}}; ai <= {W{1'b0}};
      p_rr <= 36'd0; p_ii <= 36'd0; p_ri <= 36'd0; p_ir <= 36'd0;
      yr <= {W{1'b0}}; yi <= {W{1'b0}};
    end else if (ce) begin
      c    <= c + 1'b1;
      ar   <= xr;
      ai   <= xi;
      p_rr <= ar * wr;
      p_ii <= ai * wi;
      p_ri <= ar * wi;
      p_ir <= ai * wr;
      yr   <= (tre > MAXV) ? MAXV[W-1:0] : (tre < MINV) ? MINV[W-1:0] : tre[W-1:0];
      yi   <= (tim > MAXV) ? MAXV[W-1:0] : (tim < MINV) ? MINV[W-1:0] : tim[W-1:0];
    end
  end
endmodule

// ---------------------------------------------------------------------------------------------
module ptx_ifft #(
  parameter W = 18,
  parameter IN_OFFSET = 0          // enabled cycles before bin 0 of the first symbol (feeder pipeline)
) (
  input  wire                clk,
  input  wire                rst_n,
  input  wire                ce,
  input  wire signed [W-1:0] xr,
  input  wire signed [W-1:0] xi,
  output wire signed [W-1:0] yr,
  output wire signed [W-1:0] yi
);
  // samples before each stage (latency: butterfly D + 1, twiddle 3)
  localparam LB1  = 0;            // BF2I  D 1024
  localparam LB2  = LB1 + 1025;   // BF2II D 512
  localparam LB3  = LB2 + 513;    // TW 2048
  localparam LB4  = LB3 + 3;      // BF2I  D 256
  localparam LB5  = LB4 + 257;    // BF2II D 128
  localparam LB6  = LB5 + 129;    // TW 512
  localparam LB7  = LB6 + 3;      // BF2I  D 64
  localparam LB8  = LB7 + 65;     // BF2II D 32
  localparam LB9  = LB8 + 33;     // TW 128
  localparam LB10 = LB9 + 3;      // BF2I  D 16
  localparam LB11 = LB10 + 17;    // BF2II D 8
  localparam LB12 = LB11 + 9;     // TW 32
  localparam LB13 = LB12 + 3;     // BF2I  D 4
  localparam LB14 = LB13 + 5;     // BF2II D 2
  localparam LB15 = LB14 + 3;     // TW 8
  localparam LB16 = LB15 + 3;     // BF2I  D 1 (last radix-2 stage)
  localparam LAT  = LB16 + 2;     // 2073

  wire signed [W-1:0] r1, i1, r2, i2, r3, i3, r4, i4, r5, i5, r6, i6, r7, i7, r8, i8;
  wire signed [W-1:0] r9, i9, r10, i10, r11, i11, r12, i12, r13, i13, r14, i14, r15, i15;

`define PTX_INIT(lb, p) (((p) - (((lb) + IN_OFFSET) % (p))) % (p))
  ptx_sdf_bf #(.D(1024), .TWOJ(0), .SH(0), .INIT(`PTX_INIT(LB1, 4096)), .W(W)) s1 (clk, rst_n, ce, xr, xi, r1, i1);
  ptx_sdf_bf #(.D(512),  .TWOJ(1), .SH(0), .INIT(`PTX_INIT(LB2, 2048)), .W(W)) s2 (clk, rst_n, ce, r1, i1, r2, i2);
  ptx_sdf_tw #(.L(2048), .INIT(`PTX_INIT(LB3, 2048)), .W(W))                   t1 (clk, rst_n, ce, r2, i2, r3, i3);
  ptx_sdf_bf #(.D(256),  .TWOJ(0), .SH(1), .INIT(`PTX_INIT(LB4, 1024)), .W(W)) s3 (clk, rst_n, ce, r3, i3, r4, i4);
  ptx_sdf_bf #(.D(128),  .TWOJ(1), .SH(0), .INIT(`PTX_INIT(LB5, 512)),  .W(W)) s4 (clk, rst_n, ce, r4, i4, r5, i5);
  ptx_sdf_tw #(.L(512),  .INIT(`PTX_INIT(LB6, 512)),  .W(W))                   t2 (clk, rst_n, ce, r5, i5, r6, i6);
  ptx_sdf_bf #(.D(64),   .TWOJ(0), .SH(1), .INIT(`PTX_INIT(LB7, 256)),  .W(W)) s5 (clk, rst_n, ce, r6, i6, r7, i7);
  ptx_sdf_bf #(.D(32),   .TWOJ(1), .SH(0), .INIT(`PTX_INIT(LB8, 128)),  .W(W)) s6 (clk, rst_n, ce, r7, i7, r8, i8);
  ptx_sdf_tw #(.L(128),  .INIT(`PTX_INIT(LB9, 128)),  .W(W))                   t3 (clk, rst_n, ce, r8, i8, r9, i9);
  ptx_sdf_bf #(.D(16),   .TWOJ(0), .SH(1), .INIT(`PTX_INIT(LB10, 64)),  .W(W)) s7 (clk, rst_n, ce, r9, i9, r10, i10);
  ptx_sdf_bf #(.D(8),    .TWOJ(1), .SH(0), .INIT(`PTX_INIT(LB11, 32)),  .W(W)) s8 (clk, rst_n, ce, r10, i10, r11, i11);
  ptx_sdf_tw #(.L(32),   .INIT(`PTX_INIT(LB12, 32)),  .W(W))                   t4 (clk, rst_n, ce, r11, i11, r12, i12);
  ptx_sdf_bf #(.D(4),    .TWOJ(0), .SH(1), .INIT(`PTX_INIT(LB13, 16)),  .W(W)) s9 (clk, rst_n, ce, r12, i12, r13, i13);
  ptx_sdf_bf #(.D(2),    .TWOJ(1), .SH(0), .INIT(`PTX_INIT(LB14, 8)),   .W(W)) s10 (clk, rst_n, ce, r13, i13, r14, i14);
  ptx_sdf_tw #(.L(8),    .INIT(`PTX_INIT(LB15, 8)),   .W(W))                   t5 (clk, rst_n, ce, r14, i14, r15, i15);
  ptx_sdf_bf #(.D(1),    .TWOJ(0), .SH(1), .INIT(`PTX_INIT(LB16, 4)),   .W(W)) s11 (clk, rst_n, ce, r15, i15, yr, yi);
`undef PTX_INIT
endmodule
