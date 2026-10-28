// tb_pd_timing.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Directed unit test of RTL/pd_timing.v against the bit-true model (make_tm_vectors.py): feeds
// DC-blocker samples from in.txt (random gaps of up to +gap=N clocks, +fixgap=1 for exactly N;
// held back while `stall`), pulses `trig` with each event's t when sample t goes in and the
// event (t, cfo_inc, fail) once sample t + N + 60 has gone in, then checks every timing record
// against exp_tm.txt (matched by t). Prints PASS / FAIL. Plain Verilog-2001.
`timescale 1ns / 1ps
`default_nettype none

module tb_pd_timing;
    reg clk;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    reg         rst_n, y_valid, trig, ev_valid, ev_fail;
    reg  [11:0] y_i, y_q;
    reg  [31:0] t_trig, ev_t, ev_inc;
    reg  [15:0] n_cfo, t_pk;
    wire        stall, tm_valid, busy, err;
    wire [31:0] tm_t, tm_n_end;
    wire [2:0]  tm_status;
    wire [15:0] tm_n_pos;
    wire [33:0] tm_p_max;
    wire [47:0] tm_p_sum;

    pd_timing dut (
        .clk(clk), .rst_n(rst_n), .en(1'b1), .n_cfo(n_cfo), .t_pk(t_pk),
        .y_valid(y_valid), .y_i(y_i), .y_q(y_q),
        .trig(trig), .t_trig(t_trig),
        .ev_valid(ev_valid), .ev_t(ev_t), .ev_cfo_inc(ev_inc), .ev_fail(ev_fail),
        .stall(stall),
        .tm_valid(tm_valid), .tm_t(tm_t), .tm_n_end(tm_n_end), .tm_status(tm_status),
        .tm_n_pos(tm_n_pos), .tm_p_max(tm_p_max), .tm_p_sum(tm_p_sum),
        .busy(busy), .err(err));

    integer f_cfg, f_in, f_ev, f_tm, r, rc, i, k, w, idle, rnd, hit, guard;
    integer cfg_n, cfg_tpk, n_samples, n_ev, n_tm, gap, fixgap, seed, errors, seen, stalls;
    integer e_t [0:255];
    integer e_inc [0:255];
    integer e_fail [0:255];
    integer next_trig, next_ev;
    reg [31:0] xt_t [0:255];
    reg [31:0] xt_n [0:255];
    reg [2:0]  xt_s [0:255];
    reg [15:0] xt_p [0:255];
    reg [33:0] xt_m [0:255];
    reg [47:0] xt_u [0:255];
    reg        used [0:255];
    reg [11:0] si, sq;

    always @(posedge clk) begin
        if (rst_n && tm_valid) begin
            hit = -1;
            for (k = 0; k < n_tm; k = k + 1)
                if (hit < 0 && !used[k] && xt_t[k] == tm_t) hit = k;
            if (hit < 0) begin
                errors = errors + 1;
                $display("ERROR: unexpected record t %0d n_end %0d status %0d", tm_t, tm_n_end, tm_status);
            end else begin
                used[hit] = 1'b1;
                if (tm_n_end !== xt_n[hit] || tm_status !== xt_s[hit] || tm_n_pos !== xt_p[hit] ||
                    tm_p_max !== xt_m[hit] || tm_p_sum !== xt_u[hit]) begin
                    errors = errors + 1;
                    $display("ERROR t %0d: n_end %0d status %0d n_pos %0d p_max %0d p_sum %0d, expected %0d %0d %0d %0d %0d",
                             tm_t, tm_n_end, tm_status, tm_n_pos, tm_p_max, tm_p_sum,
                             xt_n[hit], xt_s[hit], xt_p[hit], xt_m[hit], xt_u[hit]);
                end else
                    $display("  t %0d: status %0d n_end %0d (ok)", tm_t, tm_status, tm_n_end);
            end
            seen = seen + 1;
        end
        if (rst_n && err) begin errors = errors + 1; $display("ERROR: start queue overflow"); end
        if (rst_n && stall) stalls = stalls + 1;
    end

    initial begin
        gap = 2; fixgap = 0; seed = 1; errors = 0; seen = 0; stalls = 0;
        rc = $value$plusargs("gap=%d", gap);
        rc = $value$plusargs("fixgap=%d", fixgap);
        rc = $value$plusargs("seed=%d", seed);
        rst_n = 1'b0; y_valid = 1'b0; trig = 1'b0; ev_valid = 1'b0; ev_fail = 1'b0;
        y_i = 0; y_q = 0; t_trig = 0; ev_t = 0; ev_inc = 0;
        f_cfg = $fopen("cfg.txt", "r"); f_in = $fopen("in.txt", "r");
        f_ev = $fopen("ev.txt", "r"); f_tm = $fopen("exp_tm.txt", "r");
        if (f_cfg == 0 || f_in == 0 || f_ev == 0 || f_tm == 0) begin
            $display("FAIL: cannot open the vector files"); $finish;
        end
        r = $fscanf(f_cfg, "%d %d %d %d %d", cfg_n, cfg_tpk, n_samples, n_ev, n_tm);
        n_cfo = cfg_n[15:0]; t_pk = cfg_tpk[15:0];
        for (k = 0; k < n_ev; k = k + 1) r = $fscanf(f_ev, "%d %d %d\n", e_t[k], e_inc[k], e_fail[k]);
        for (k = 0; k < n_tm; k = k + 1) begin
            r = $fscanf(f_tm, "%h %h %h %h %h %h\n", xt_t[k], xt_n[k], xt_s[k], xt_p[k], xt_m[k], xt_u[k]);
            used[k] = 1'b0;
        end
        $display("Vector: %0d samples, N_CFO %0d, T_PK %0d, %0d events, %0d records, gap %0d", n_samples, cfg_n,
                 cfg_tpk, n_ev, n_tm, gap);
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
        next_trig = 0; next_ev = 0;
        for (i = 0; i < n_samples; i = i + 1) begin
            r = $fscanf(f_in, "%h %h\n", si, sq);
            rnd  = $random(seed);
            idle = fixgap ? gap : (gap > 0) ? ((rnd & 32'h7fffffff) % (gap + 1)) : 0;
            for (w = 0; w < idle; w = w + 1) begin @(negedge clk); y_valid = 1'b0; trig = 1'b0; ev_valid = 1'b0; end
            @(negedge clk);
            trig = 1'b0; ev_valid = 1'b0;
            while (stall) begin y_valid = 1'b0; @(negedge clk); end
            y_valid = 1'b1; y_i = si; y_q = sq;
            if (next_trig < n_ev && e_t[next_trig] == i) begin
                trig = 1'b1; t_trig = e_t[next_trig]; next_trig = next_trig + 1;
            end
            if (next_ev < n_ev && e_t[next_ev] + cfg_n + 60 == i) begin
                ev_valid = 1'b1; ev_t = e_t[next_ev]; ev_inc = e_inc[next_ev]; ev_fail = e_fail[next_ev][0];
                next_ev = next_ev + 1;
            end
        end
        @(negedge clk);
        y_valid = 1'b0; trig = 1'b0; ev_valid = 1'b0;
        guard = 0;
        @(negedge clk);
        while (busy && guard < 100000) begin @(negedge clk); guard = guard + 1; end
        repeat (20) @(negedge clk);
        if (busy) begin errors = errors + 1; $display("ERROR: still busy at the end"); end
        if (seen != n_tm) begin errors = errors + 1; $display("ERROR: %0d records, expected %0d", seen, n_tm); end
        $display("  input held for %0d clocks", stalls);
        if (errors == 0) $display("PASS: %0d timing record(s) match the bit-true model", seen);
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
