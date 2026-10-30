// tb_fft2.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// pd_fft2 (two butterflies per clock) and cf_fft (one) against cf_reference.fft_fixed, on the
// cases of make_fft_vectors.py (+dir=work/fft_units, default "."; n.txt, in.txt, exp.txt).
// Prints the clocks per FFT of each engine and PASS / FAIL.
`timescale 1ns / 1ps

module tb_fft2;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg               in_we;
    reg  [9:0]        in_idx;
    reg signed [17:0] in_re, in_im;
    reg               start;
    reg  [9:0]        out_addr;
    wire              d2, d1;
    wire signed [17:0] r2, i2, r1, i1;
    pd_fft2 u2 (.clk(clk), .rst_n(rst_n), .in_we(in_we), .in_idx(in_idx), .in_re(in_re), .in_im(in_im),
                .start(start), .busy(), .done(d2), .out_addr(out_addr), .out_re(r2), .out_im(i2));
    cf_fft  u1 (.clk(clk), .rst_n(rst_n), .in_we(in_we), .in_idx(in_idx), .in_re(in_re), .in_im(in_im),
                .start(start), .busy(), .done(d1), .out_addr(out_addr), .out_re(r1), .out_im(i1));

    integer fn, fi, fe, ncase, c, k, rv, a, b, errs, e2, e1;
    reg [31:0] t0, t2, t1;            // (no integer non-blocking assignments: xsim)
    reg signed [17:0] xr [0:1023];
    reg signed [17:0] xi [0:1023];
    reg signed [17:0] er [0:1023];
    reg signed [17:0] ei [0:1023];
    reg got2, got1;
    reg [8*256-1:0] dir, fname;
    reg [31:0] cyc;
    always @(posedge clk) cyc <= cyc + 1;
    always @(posedge clk) begin
        if (d2) begin got2 <= 1'b1; t2 <= cyc; end
        if (d1) begin got1 <= 1'b1; t1 <= cyc; end
    end

    initial begin
        cyc = 0; errs = 0; in_we = 0; in_idx = 0; in_re = 0; in_im = 0; start = 0; out_addr = 0;
        if (!$value$plusargs("dir=%s", dir)) dir = ".";
        $sformat(fname, "%0s/n.txt", dir);   fn = $fopen(fname, "r");
        if (fn == 0) begin $display("FAIL fft_units: cannot open %0s", fname); $finish; end
        rv = $fscanf(fn, "%d", ncase); $fclose(fn);
        $sformat(fname, "%0s/in.txt", dir);  fi = $fopen(fname, "r");
        $sformat(fname, "%0s/exp.txt", dir); fe = $fopen(fname, "r");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        for (c = 0; c < ncase; c = c + 1) begin
            for (k = 0; k < 1024; k = k + 1) begin
                rv = $fscanf(fi, "%d %d", a, b); xr[k] = a; xi[k] = b;
                rv = $fscanf(fe, "%d %d", a, b); er[k] = a; ei[k] = b;
            end
            // load (natural order), both engines idle
            for (k = 0; k < 1024; k = k + 1) begin
                @(negedge clk); in_we = 1'b1; in_idx = k; in_re = xr[k]; in_im = xi[k];
            end
            @(negedge clk); in_we = 1'b0;
            got2 = 1'b0; got1 = 1'b0;
            @(negedge clk); start = 1'b1; t0 = cyc;
            @(negedge clk); start = 1'b0;
            wait (got2 && got1);
            @(negedge clk);
            e2 = 0; e1 = 0;
            for (k = 0; k < 1024; k = k + 1) begin
                @(negedge clk); out_addr = k;
                @(negedge clk);
                if (r2 !== er[k] || i2 !== ei[k]) begin
                    if (e2 < 3) $display("case %0d bin %0d: pd_fft2 %0d %0d, model %0d %0d", c, k, r2, i2, er[k], ei[k]);
                    e2 = e2 + 1;
                end
                if (r1 !== er[k] || i1 !== ei[k]) e1 = e1 + 1;
            end
            if (c == 0) $display("clocks per FFT: pd_fft2 %0d, cf_fft %0d", t2 - t0, t1 - t0);
            if (e2 || e1) $display("case %0d: %0d bins wrong in pd_fft2, %0d in cf_fft", c, e2, e1);
            errs = errs + e2 + e1;
        end
        if (errs == 0) $display("PASS fft_units: %0d cases, pd_fft2 and cf_fft equal the model", ncase);
        else $display("FAIL fft_units: %0d wrong bins", errs);
        $finish;
    end
endmodule
