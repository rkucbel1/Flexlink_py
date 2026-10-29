// tb_cf_rx.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Control-field receiver (RTL/cf_rx.v) against the bit-true model (make_cf_vectors.py), run
// in work/cf_<case>/. Feeds the 4560 samples with random gaps (+gap=N: up to N idle clocks,
// +seed=S) and compares every grid value, every Hf0 / H3' value, every soft value, phi, the
// shift, the 70 decoded bits, the metric and the CRC result. Runs the case twice (+runs=N)
// without a reset in between, so state left over from a control field must not matter.
// Prints PASS or FAIL.
`timescale 1ns / 1ps
`default_nettype none

module tb_cf_rx;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg  [11:0] in_i [0:4559];
    reg  [11:0] in_q [0:4559];
    reg  [35:0] e_grid [0:3363];
    reg  [79:0] e_hf [0:840];
    reg  [45:0] e_v [0:2199];
    reg  [69:0] e_bits;
    integer e_inc, e_phi, e_shift, e_metric, e_crc;

    reg         start = 1'b0;
    reg  [23:0] cfo_inc;
    reg         s_valid = 1'b0;
    wire        s_ready;
    reg  [11:0] s_i, s_q;
    wire        rec_valid, rec_crc_ok;
    wire [53:0] rec_msg;
    wire [5:0]  rec_shift;
    wire signed [15:0] rec_metric, dbg_phi;
    wire [31:0] rec_t_trig, rec_n_end;
    wire        dbg_grid_we, dbg_hf_we, dbg_v_we;
    wire [11:0] dbg_grid_addr, dbg_v_i;
    wire [35:0] dbg_grid_data;
    wire [9:0]  dbg_hf_k;
    wire [79:0] dbg_hf;
    wire signed [45:0] dbg_v;
    wire [69:0] dbg_bits;

    cf_rx dut (.clk(clk), .rst_n(rst_n), .start(start), .cfo_inc(cfo_inc), .t_trig(32'd1234),
               .n_end(32'd5678), .busy(), .s_valid(s_valid), .s_ready(s_ready), .s_i(s_i), .s_q(s_q),
               .rec_valid(rec_valid), .rec_crc_ok(rec_crc_ok), .rec_msg(rec_msg), .rec_shift(rec_shift),
               .rec_metric(rec_metric), .rec_t_trig(rec_t_trig), .rec_n_end(rec_n_end),
               .dbg_grid_we(dbg_grid_we), .dbg_grid_addr(dbg_grid_addr), .dbg_grid_data(dbg_grid_data),
               .dbg_phi(dbg_phi), .dbg_hf_we(dbg_hf_we), .dbg_hf_k(dbg_hf_k), .dbg_hf(dbg_hf),
               .dbg_v_we(dbg_v_we), .dbg_v_i(dbg_v_i), .dbg_v(dbg_v), .dbg_bits(dbg_bits));

    integer err_grid = 0, err_hf = 0, err_v = 0, n_grid = 0, n_hf = 0, n_v = 0, errs = 0;
    integer gap = 3, seed = 1, fd, i, r, s, c, tmo, run, runs = 2, acc_base, total = 0;
    reg [69:0] bits_rd;
    reg [19:0] t0, t1, t2, t3;
    reg [45:0] t46;

    // ---- monitors
    reg [31:0] acc_n = 32'd0;
    always @(posedge clk) if (s_valid && s_ready) acc_n <= acc_n + 32'd1;
    always @(posedge clk) begin
        if (rst_n && dbg_grid_we) begin
            s = dbg_grid_addr[11:10]; c = dbg_grid_addr[9:0];
            n_grid = n_grid + 1;
            if (dbg_grid_data !== e_grid[s * 841 + c]) begin
                if (err_grid < 5) $display("grid s %0d c %0d: rtl %h model %h", s, c, dbg_grid_data, e_grid[s * 841 + c]);
                err_grid = err_grid + 1;
            end
        end
        if (rst_n && dbg_hf_we) begin
            n_hf = n_hf + 1;
            if (dbg_hf !== e_hf[dbg_hf_k]) begin
                if (err_hf < 5) $display("hf k %0d: rtl %h model %h", dbg_hf_k, dbg_hf, e_hf[dbg_hf_k]);
                err_hf = err_hf + 1;
            end
        end
        if (rst_n && dbg_v_we) begin
            n_v = n_v + 1;
            if (dbg_v !== e_v[dbg_v_i]) begin
                if (err_v < 5) $display("v i %0d: rtl %h model %h", dbg_v_i, dbg_v, e_v[dbg_v_i]);
                err_v = err_v + 1;
            end
        end
    end

    reg [8*256-1:0] dir, fname;
    reg [69:0] rd_bits;
    initial begin
        if (!$value$plusargs("dir=%s", dir)) dir = ".";
        if ($value$plusargs("gap=%d", gap)) ;
        if ($value$plusargs("seed=%d", seed)) ;
        $sformat(fname, "%0s/cfg.txt", dir);
        fd = $fopen(fname, "r");
        r = $fscanf(fd, "%d %d %d %d %d", e_inc, e_phi, e_shift, e_metric, e_crc);
        $fclose(fd);
        $sformat(fname, "%0s/in.txt", dir);
        fd = $fopen(fname, "r");
        for (i = 0; i < 4560; i = i + 1) begin
            r = $fscanf(fd, "%h %h", t0, t1); in_i[i] = t0[11:0]; in_q[i] = t1[11:0];
        end
        $fclose(fd);
        $sformat(fname, "%0s/exp_grid.txt", dir);
        fd = $fopen(fname, "r");
        for (i = 0; i < 3364; i = i + 1) begin
            r = $fscanf(fd, "%h %h", t0, t1); e_grid[i] = {t0[17:0], t1[17:0]};
        end
        $fclose(fd);
        $sformat(fname, "%0s/exp_hf.txt", dir);
        fd = $fopen(fname, "r");
        for (i = 0; i < 841; i = i + 1) begin
            r = $fscanf(fd, "%h %h %h %h", t0, t1, t2, t3); e_hf[i] = {t0, t1, t2, t3};
        end
        $fclose(fd);
        $sformat(fname, "%0s/exp_v.txt", dir);
        fd = $fopen(fname, "r");
        for (i = 0; i < 2200; i = i + 1) begin
            r = $fscanf(fd, "%h", t46); e_v[i] = t46;
        end
        $fclose(fd);
        $sformat(fname, "%0s/exp_bits.txt", dir);
        fd = $fopen(fname, "r");
        r = $fscanf(fd, "%b", rd_bits);              // bit 0 first in the file
        $fclose(fd);
        for (i = 0; i < 70; i = i + 1) e_bits[i] = rd_bits[69 - i];
        cfo_inc = e_inc[23:0];

        if ($value$plusargs("runs=%d", runs)) ;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        for (run = 0; run < runs; run = run + 1) begin
            // a second run starts with the RAMs and registers the first one left behind
            err_grid = 0; err_hf = 0; err_v = 0; n_grid = 0; n_hf = 0; n_v = 0; errs = 0;
            acc_base = acc_n;
            repeat (5) @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            // drive on the falling edge; acc_n counts the samples accepted at rising edges
            tmo = 0;
            while (acc_n - acc_base < 4560) begin
                @(negedge clk);
                tmo = tmo + 1;
                if (tmo > 400000) begin
                    $display("FAIL: input stalled at sample %0d", acc_n - acc_base);
                    $finish;
                end
                r = $random(seed);                  // seed is updated by $random
                r = (gap > 0) ? ((r < 0 ? -r : r) % (gap + 2)) : 0;
                s_valid = (r <= 1);
                s_i = in_i[acc_n - acc_base]; s_q = in_q[acc_n - acc_base];
            end
            @(negedge clk);
            s_valid = 1'b0;
            i = 0;
            while (!rec_valid && i < 200000) begin @(posedge clk); i = i + 1; end
            if (!rec_valid) begin
                $display("FAIL: no record (timeout)");
                $finish;
            end
            @(negedge clk);
            errs = err_grid + err_hf + err_v;
            if (n_grid != 3364 || n_hf != 841 || n_v != 2200) begin
                $display("counts: grid %0d hf %0d v %0d", n_grid, n_hf, n_v); errs = errs + 1;
            end
            if (dbg_phi != e_phi) begin $display("phi rtl %0d model %0d", dbg_phi, e_phi); errs = errs + 1; end
            if (rec_shift != e_shift) begin $display("shift rtl %0d model %0d", rec_shift, e_shift); errs = errs + 1; end
            if (dbg_bits !== e_bits) begin $display("bits rtl %b\n     model %b", dbg_bits, e_bits); errs = errs + 1; end
            if (rec_metric != e_metric) begin $display("metric rtl %0d model %0d", rec_metric, e_metric); errs = errs + 1; end
            if (rec_crc_ok != e_crc[0]) begin $display("crc rtl %0d model %0d", rec_crc_ok, e_crc); errs = errs + 1; end
            if (rec_msg !== e_bits[53:0]) begin $display("msg mismatch"); errs = errs + 1; end
            if (rec_t_trig != 32'd1234 || rec_n_end != 32'd5678) begin $display("tags"); errs = errs + 1; end
            total = total + errs;
            if (errs != 0)
                $display("run %0d: %0d grid, %0d hf, %0d soft mismatches, %0d other", run, err_grid, err_hf,
                         err_v, errs - err_grid - err_hf - err_v);
        end
        if (total == 0)
            $display("PASS: %0d runs x (grid %0d, hf %0d, soft %0d values), phi %0d, shift %0d, metric %0d, crc %0s",
                     runs, n_grid, n_hf, n_v, e_phi, e_shift, e_metric, e_crc ? "ok" : "fail");
        else
            $display("FAIL: %0d mismatches", total);
        $finish;
    end
endmodule

`default_nettype wire
