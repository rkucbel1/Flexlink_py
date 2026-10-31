// tb_pl_ldpc.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// pl_ldpc_core against pl_reference.decode_ldpc on the cases of make_ldpc_vectors.py
// (+dir=work/ldpc_units, default "."; n.txt, cases.txt). Every hard bit and the iteration count
// must match. Input words come with random gaps and the output with random back-pressure
// (+gap=<max idle clocks>, default 2; +seed=). Prints clocks per code word and PASS / FAIL.
`timescale 1ns / 1ps

module tb_pl_ldpc #(parameter integer MAXIT = 10);
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg         start = 1'b0;
    reg  [3:0]  code = 4'd0;
    wire        busy;
    reg         in_valid = 1'b0;
    wire        in_ready;
    reg  [27*7-1:0] in_llr = 0;
    wire        out_valid;
    reg         out_ready = 1'b0;
    wire [26:0] out_bits;
    wire        out_last;
    wire [3:0]  iters;
    wire [15:0] cycles;

    pl_ldpc_core #(.MAXIT(MAXIT)) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .code(code), .busy(busy),
        .in_valid(in_valid), .in_ready(in_ready), .in_llr(in_llr),
        .out_valid(out_valid), .out_ready(out_ready), .out_bits(out_bits), .out_last(out_last),
        .iters(iters), .cycles(cycles));

    integer fn, fc, rv, ncase, c, k, n, m, b, v, errs, it_exp, ebits, eit, gap, seed, r, nw;
    integer tot_cyc [0:11];
    integer cnt [0:11];
    reg [8*256-1:0] dir, fname;
    reg signed [7:0] llr [0:1943];
    reg [1943:0] exp_bits;
    reg [1943:0] got_bits;
    reg [8*2000-1:0] line;
    integer ci, verbose;

    initial begin
        errs = 0;
        if (!$value$plusargs("dir=%s", dir)) dir = ".";
        if (!$value$plusargs("gap=%d", gap)) gap = 2;
        if (!$value$plusargs("seed=%d", seed)) seed = 1;
        if (!$value$plusargs("verbose=%d", verbose)) verbose = 0;
        r = $random(seed);
        for (k = 0; k < 12; k = k + 1) begin tot_cyc[k] = 0; cnt[k] = 0; end
        $sformat(fname, "%0s/n.txt", dir); fn = $fopen(fname, "r");
        if (fn == 0) begin $display("FAIL ldpc_units: cannot open %0s", fname); $finish; end
        rv = $fscanf(fn, "%d", ncase); $fclose(fn);
        $sformat(fname, "%0s/cases.txt", dir); fc = $fopen(fname, "r");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        for (c = 0; c < ncase; c = c + 1) begin
            rv = $fscanf(fc, "%d %d", ci, n);
            for (k = 0; k < n; k = k + 1) begin rv = $fscanf(fc, "%d", v); llr[k] = v; end
            for (k = 0; k < n; k = k + 1) begin rv = $fscanf(fc, "%d", v); exp_bits[k] = v[0]; end
            rv = $fscanf(fc, "%d", it_exp);
            nw = n / 27;
            // start
            @(negedge clk); start = 1'b1; code = ci;
            @(negedge clk); start = 1'b0;
            // input words with random gaps
            m = 0;
            while (m < nw) begin
                @(negedge clk);
                if (gap > 0 && ($random(seed) & 3) == 0) in_valid = 1'b0;
                else begin
                    in_valid = 1'b1;
                    for (b = 0; b < 27; b = b + 1) in_llr[b*7 +: 7] = llr[27*m + b];
                end
                @(posedge clk);
                if (in_valid && in_ready) m = m + 1;
            end
            @(negedge clk);
            in_valid = 1'b0;
            // output words with random back-pressure
            m = 0;
            while (m < nw) begin
                @(posedge clk);
                if (out_valid && out_ready) begin
                    got_bits[27*m +: 27] = out_bits;
                    if (out_last != (m == nw - 1)) begin
                        $display("case %0d: out_last %0d at word %0d of %0d", c, out_last, m, nw);
                        errs = errs + 1;
                    end
                    if (m == 0) eit = iters;
                    m = m + 1;
                end
                @(negedge clk);
                out_ready = (gap == 0) || (($random(seed) & 3) != 0);
            end
            @(negedge clk); out_ready = 1'b0;
            ebits = 0;
            for (k = 0; k < n; k = k + 1) if (got_bits[k] !== exp_bits[k]) ebits = ebits + 1;
            if (ebits || eit != it_exp) begin
                if (errs < 20)
                    $display("case %0d code %0d: %0d wrong bits, iterations %0d (model %0d)", c, ci, ebits, eit, it_exp);
                errs = errs + 1;
            end
            if (verbose) $display("case %0d code %0d: iterations %0d (model %0d), %0d wrong bits, %0d clocks", c, ci, eit, it_exp, ebits, cycles);
            tot_cyc[ci] = tot_cyc[ci] + cycles * 1000 / it_exp;   // clocks per iteration x 1000
            cnt[ci] = cnt[ci] + 1;
            wait (!busy);
        end
        for (k = 0; k < 12; k = k + 1)
            if (cnt[k]) $display("code %0d: %0d clocks per iteration (mean, incl. syndrome)", k, tot_cyc[k] / cnt[k] / 1000);
        if (errs == 0) $display("PASS ldpc_units: %0d code words, bits and iterations equal the model", ncase);
        else $display("FAIL ldpc_units: %0d code words wrong", errs);
        $finish;
    end

    initial begin
        #20000000;
        $display("FAIL ldpc_units: timeout in case %0d: st %0d p1_q %0d p1_fin %0d p2_q %0d p2_fin %0d p1_run %0d p2_run %0d syn_busy %0d syn_done %0d y_run %0d stop %0d e1 %0d haz %0d may1 %0d may2 %0d",
                 c, dut.st, dut.p1_q, dut.p1_fin, dut.p2_q, dut.p2_fin, dut.p1_run, dut.p2_run, dut.syn_busy,
                 dut.syn_done, dut.y_run, dut.stop, dut.e1, dut.haz1, dut.p1_may, dut.p2_may);
        $finish;
    end
endmodule
