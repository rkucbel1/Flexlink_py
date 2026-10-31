// tb_pl_chest.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// pl_chest against pl_reference.channel on the cases of make_chest_vectors.py (+dir=, default
// "."; n.txt, cases.txt). The testbench plays cf_rx: payload symbols only with room, the decision
// after a random symbol (at most 5 symbols speculative), a rejected packet (random bins, pl_ok 0)
// before every real one, and the FFT jobs on its own pd_fft2 between symbols. Every packet
// record, every H and every Y' must match, in order. Random gaps (+gap=, default 1) and output
// back-pressure; +seed=.
`timescale 1ns / 1ps

module tb_pl_chest;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg               pg_valid = 1'b0;
    reg  [6:0]        pg_sym = 7'd0;
    reg  [9:0]        pg_col = 10'd0;
    reg  signed [17:0] pg_re = 0, pg_im = 0;
    reg               pl_dec = 1'b0, pl_ok = 1'b0;
    reg  [53:0]       pl_msg = 0;
    reg  [31:0]       pl_t = 0;
    wire              room;
    wire              job_req, job_we, job_start;
    reg               job_gnt = 1'b0;
    wire [9:0]        job_idx, job_oaddr;
    wire signed [17:0] job_re, job_im, fft_ore, fft_oim;
    wire              fft_done;
    reg               out_room = 1'b1;
    wire              pk_valid, h_valid, busy, err;
    wire [31:0]       pk_t, pk_a, pk_b, h_t;
    wire [6:0]        h_sym;
    wire [9:0]        h_col;
    wire signed [15:0] h_re, h_im, y_re, y_im;

    pl_chest dut (.clk(clk), .rst_n(rst_n), .pg_valid(pg_valid), .pg_sym(pg_sym), .pg_col(pg_col),
                  .pg_re(pg_re), .pg_im(pg_im), .pl_dec(pl_dec), .pl_ok(pl_ok), .pl_msg(pl_msg), .pl_t(pl_t),
                  .room(room), .job_req(job_req), .job_gnt(job_gnt), .job_we(job_we), .job_idx(job_idx),
                  .job_re(job_re), .job_im(job_im), .job_start(job_start), .fft_done(fft_done),
                  .job_oaddr(job_oaddr), .fft_ore(fft_ore), .fft_oim(fft_oim), .out_room(out_room),
                  .pk_valid(pk_valid), .pk_t(pk_t), .pk_a(pk_a), .pk_b(pk_b), .h_valid(h_valid), .h_t(h_t),
                  .h_sym(h_sym), .h_col(h_col), .h_re(h_re), .h_im(h_im), .y_re(y_re), .y_im(y_im),
                  .busy(busy), .err(err));
    pd_fft2 u_fft (.clk(clk), .rst_n(rst_n), .in_we(job_we && job_gnt), .in_idx(job_idx), .in_re(job_re),
                   .in_im(job_im), .start(job_start && job_gnt), .busy(), .done(fft_done), .out_addr(job_oaddr),
                   .out_re(fft_ore), .out_im(fft_oim));

    integer fn, fc, rv, ncase, c, s, col, plen, a, b2, errs, nout, npk, gap, seed, dsym, t0, cyc, verbose, jobs;
    reg [63:0] msg64;
    reg [35:0] grid [0:121*841-1];
    reg [63:0] exph [0:121*841-1];
    reg [31:0] exp_a, exp_b;
    reg [8*256-1:0] dir, fname;
    reg feeding;
    integer exp_sym, exp_col, case_errs;
    reg [31:0] tt;
    always @(posedge clk) cyc <= cyc + 1;

    // job server: grant between symbols only (as cf_rx)
    always @(posedge clk) begin
        if (!rst_n) job_gnt <= 1'b0;
        else if (job_req && !feeding) job_gnt <= 1'b1;
        else if (!job_req) job_gnt <= 1'b0;
    end
    always @(posedge clk) if (job_start && job_gnt) jobs <= jobs + 1;
    // output checker
    always @(posedge clk) begin
        if (pk_valid) begin
            npk = npk + 1;
            if (pk_a !== exp_a || pk_b !== exp_b || pk_t !== tt) begin
                $display("case %0d: packet record a %h b %h t %0d, model a %h b %h", c, pk_a, pk_b, pk_t, exp_a, exp_b);
                case_errs = case_errs + 1;
            end
            if (verbose) $display("case %0d: packet record at %0d, a %h", c, cyc - t0, pk_a);
        end
        if (h_valid) begin
            if (npk == 0 || h_sym != exp_sym || h_col != exp_col) begin
                if (case_errs < 5) $display("case %0d: output sym %0d col %0d, expected sym %0d col %0d", c, h_sym, h_col, exp_sym, exp_col);
                case_errs = case_errs + 1;
            end else if ({h_re, h_im, y_re, y_im} !== exph[exp_sym*841 + exp_col]) begin
                if (case_errs < 5)
                    $display("case %0d sym %0d col %0d: H %0d %0d Y' %0d %0d, model H %0d %0d Y' %0d %0d", c, h_sym, h_col,
                             h_re, h_im, y_re, y_im, $signed(exph[exp_sym*841 + exp_col][63:48]),
                             $signed(exph[exp_sym*841 + exp_col][47:32]), $signed(exph[exp_sym*841 + exp_col][31:16]),
                             $signed(exph[exp_sym*841 + exp_col][15:0]));
                case_errs = case_errs + 1;
            end
            if (exp_col == 840) begin exp_col = 0; exp_sym = exp_sym + 1; end
            else exp_col = exp_col + 1;
            nout = nout + 1;
        end
    end
    // output back-pressure
    always @(posedge clk) out_room <= (gap == 0) || (($random(seed) & 7) != 0);

    task send_symbol(input integer sym, input integer rnd_bins);
        integer cc;
        begin
            while (!room || job_gnt) @(negedge clk);
            feeding = 1'b1;
            for (cc = 0; cc < 841; cc = cc + 1) begin
                @(negedge clk);
                pg_valid = 1'b1; pg_sym = sym; pg_col = cc;
                if (rnd_bins) begin pg_re = $random(seed) % 4000; pg_im = $random(seed) % 4000; end
                else begin pg_re = grid[sym*841 + cc][35:18]; pg_im = grid[sym*841 + cc][17:0]; end
            end
            @(negedge clk); pg_valid = 1'b0;
            feeding = 1'b0;
            repeat (($random(seed) & 255) * gap + 2) @(negedge clk);
        end
    endtask

    task decide(input ok);
        begin
            @(negedge clk); pl_dec = 1'b1; pl_ok = ok; pl_msg = msg64[53:0]; pl_t = tt;
            @(negedge clk); pl_dec = 1'b0;
        end
    endtask

    initial begin
        errs = 0; cyc = 0; jobs = 0; feeding = 1'b0;
        if (!$value$plusargs("dir=%s", dir)) dir = ".";
        if (!$value$plusargs("gap=%d", gap)) gap = 1;
        if (!$value$plusargs("seed=%d", seed)) seed = 1;
        if (!$value$plusargs("verbose=%d", verbose)) verbose = 0;
        rv = $random(seed);
        $sformat(fname, "%0s/n.txt", dir); fn = $fopen(fname, "r");
        if (fn == 0) begin $display("FAIL chest_units: cannot open %0s", fname); $finish; end
        rv = $fscanf(fn, "%d", ncase); $fclose(fn);
        if ($value$plusargs("ncase=%d", rv)) ncase = rv;
        $sformat(fname, "%0s/cases.txt", dir); fc = $fopen(fname, "r");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        for (c = 0; c < ncase; c = c + 1) begin
            rv = $fscanf(fc, "%h %d", msg64, plen);
            for (s = 0; s < plen * 841; s = s + 1) begin
                rv = $fscanf(fc, "%d %d", a, b2); grid[s] = {a[17:0], b2[17:0]};
            end
            rv = $fscanf(fc, "%d %d", exp_a, exp_b);
            for (s = 0; s < plen * 841; s = s + 1) begin
                rv = $fscanf(fc, "%d %d", a, b2); exph[s][63:32] = {a[15:0], b2[15:0]};
                rv = $fscanf(fc, "%d %d", a, b2); exph[s][31:0] = {a[15:0], b2[15:0]};
            end
            case_errs = 0; nout = 0; npk = 0; exp_sym = 0; exp_col = 0;
            tt = 1000 + c;
            // rejected packets first: 3 x (2 symbols, pl_ok 0, then the symbol still in the FFT)
            for (s = 0; s < 3; s = s + 1) begin
                send_symbol(0, 1); send_symbol(1, 1); decide(1'b0); send_symbol(2, 1);
            end
            // the real one, decided after symbol dsym
            dsym = $random(seed) & 3;
            if (dsym >= plen) dsym = plen - 1;
            t0 = cyc;
            for (s = 0; s < plen; s = s + 1) begin
                send_symbol(s, 0);
                if (s == dsym) decide(1'b1);
            end
            wait (nout == plen * 841 || cyc - t0 > 2000000);
            repeat (20) @(posedge clk);
            if (nout != plen * 841 || npk != 1) begin
                $display("case %0d: %0d of %0d outputs, %0d packet records", c, nout, plen * 841, npk);
                case_errs = case_errs + 1;
            end
            if (busy) begin $display("case %0d: still busy (es %0d q_n %0d nxt_sym %0d nxt_sm %0d nxt_row %0d)", c, dut.es, dut.q_n, dut.nxt_sym, dut.nxt_sm, dut.nxt_row); case_errs = case_errs + 1; end
            $display("case %0d: %0s, PacketLength %0d, %0d clocks, %0d jobs", c, case_errs ? "FAIL" : "ok", plen, cyc - t0, jobs);
            errs = errs + case_errs;
            jobs = 0;
        end
        if (err) begin $display("err flag set"); errs = errs + 1; end
        if (errs == 0) $display("PASS chest_units: %0d packets, every H, Y' and packet record equals the model", ncase);
        else $display("FAIL chest_units: %0d errors", errs);
        $finish;
    end
endmodule
