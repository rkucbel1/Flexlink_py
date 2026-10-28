// tb_pd_detect.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Self-checking testbench for pd_detect_top against the bit-true model. Reads the files written
// by make_tb_vectors.py (cfg.txt, in.txt, exp_lpf.txt, exp_dbg.txt, exp_trig.txt, exp_cfo.txt,
// exp_tone.txt) from the current directory (run_sim.sh runs the simulator inside the vector's
// folder), feeds the samples with random gaps in tvalid (+gap=N: up to N idle clocks between
// samples, 0 = one sample per clock, +seed=S; +fixgap=1: exactly N, e.g. 4 = 20 Msps), obeys tready, and checks every LPF output sample,
// every debug record, every trigger, every CFO result (window start, the four lag accumulators,
// the four CORDIC angles and cfo_inc), every event after the tone check (CFO fields, tone
// num / den, fail) and every Preamble B timing record (exp_tm.txt, matched by t_trig, since
// BUSY records may come first). Prints PASS / FAIL and the first +maxerr=N (default 10) mismatches.
// Plain Verilog-2001 (xsim, Verilator).
`timescale 1ns / 1ps
`default_nettype none

module tb_pd_detect;
    reg         clk;
    initial     clk = 1'b0;
    always #5   clk = ~clk;                     // 100 MHz

    reg         rst_n;
    reg  [1:0]  input_stage;
    reg  [3:0]  dc_shift;
    reg  [15:0] eta_hi_sq, eta_lo_sq, qual_len;
    reg  [31:0] blank_len, arm_holdoff;
    reg  [15:0] n_cfo;
    reg  [23:0] tone_lin;
    reg  [15:0] t_pk;
    reg         s_valid;
    reg  [31:0] s_data;
    wire        s_ready;
    wire        lpf_valid, trig, dbg_valid;
    wire [11:0] lpf_i, lpf_q;
    wire [31:0] t_trig;
    wire [15:0] dbg_metric, dbg_bits;
    wire        cfo_valid;
    wire [31:0] cfo_t, cfo_inc;
    wire [15:0] ang16, ang32, ang128, ang256;
    wire [36:0] a16_re, a16_im, a32_re, a32_im, a128_re, a128_im, a256_re, a256_im;
    wire        ev_valid, ev_fail;
    wire [31:0] ev_t, ev_inc;
    wire [15:0] ev_a16, ev_a32, ev_a128, ev_a256;
    wire [39:0] ev_num, ev_den;
    wire        busy;
    wire [3:0]  open_n;
    wire [31:0] open_t;
    wire        tm_valid, tm_err;
    wire [31:0] tm_t, tm_n_end;
    wire [2:0]  tm_status;
    wire [15:0] tm_n_pos;
    wire [33:0] tm_p_max;
    wire [47:0] tm_p_sum;

    pd_detect_top dut (
        .clk(clk), .rst_n(rst_n),
        .input_stage(input_stage), .dc_shift(dc_shift),
        .eta_hi_sq(eta_hi_sq), .eta_lo_sq(eta_lo_sq), .qual_len(qual_len),
        .blank_len(blank_len), .arm_holdoff(arm_holdoff), .n_cfo(n_cfo), .tone_lin(tone_lin), .t_pk(t_pk),
        .s_iq_tvalid(s_valid), .s_iq_tdata(s_data), .s_iq_tready(s_ready),
        .lpf_valid(lpf_valid), .lpf_i(lpf_i), .lpf_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .dbg_valid(dbg_valid), .dbg_metric(dbg_metric), .dbg_bits(dbg_bits),
        .cfo_valid(cfo_valid), .cfo_t(cfo_t), .cfo_inc(cfo_inc),
        .ang16(ang16), .ang32(ang32), .ang128(ang128), .ang256(ang256),
        .a16_re(a16_re), .a16_im(a16_im), .a32_re(a32_re), .a32_im(a32_im),
        .a128_re(a128_re), .a128_im(a128_im), .a256_re(a256_re), .a256_im(a256_im),
        .ev_valid(ev_valid), .ev_t(ev_t), .ev_cfo_inc(ev_inc),
        .ev_ang16(ev_a16), .ev_ang32(ev_a32), .ev_ang128(ev_a128), .ev_ang256(ev_a256),
        .ev_tone_num(ev_num), .ev_tone_den(ev_den), .ev_tone_fail(ev_fail),
        .tm_valid(tm_valid), .tm_t(tm_t), .tm_n_end(tm_n_end), .tm_status(tm_status), .tm_n_pos(tm_n_pos),
        .tm_p_max(tm_p_max), .tm_p_sum(tm_p_sum), .tm_err(tm_err),
        .busy(busy), .open_n(open_n), .open_t(open_t));

    // ------------------------------------------------------------------ files and settings
    integer f_cfg, f_in, f_lpf, f_dbg, f_trig, f_cfo, f_tone, f_open, r, rc, x_open, x_open_t, guard;
    integer n_samples, n_trig, n_cfo_ev, gap, seed, maxerr, fixgap, n_out;
    integer cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q, cfg_blank, cfg_hold, cfg_ncfo, cfg_lin, cfg_tpk, n_tm;
    integer f_tm, tm_seen, tm_good, tm_busy_n, k2, hit;
    reg [31:0] xt_t [0:4095];
    reg [31:0] xt_n [0:4095];
    reg [2:0]  xt_s [0:4095];
    reg [15:0] xt_p [0:4095];
    reg [33:0] xt_m [0:4095];
    reg [47:0] xt_u [0:4095];
    reg        xt_used [0:4095];

    // ------------------------------------------------------------------ checkers
    integer errors, lpf_seen, dbg_seen, trig_seen, cfo_seen, ev_seen, ev_fails, stalls;
    initial begin
        errors = 0; lpf_seen = 0; dbg_seen = 0; trig_seen = 0; cfo_seen = 0; ev_seen = 0;
        tm_seen = 0; tm_good = 0; tm_busy_n = 0; n_tm = 0;
        ev_fails = 0; stalls = 0;
        rst_n = 1'b0; s_valid = 1'b0; s_data = 32'd0;
    end
    reg [11:0] e_i, e_q;
    reg [15:0] e_m, e_b;
    integer    e_t;
    reg [31:0] x_t, x_inc;
    reg [36:0] x_a [0:7];
    reg [15:0] x_g [0:3];
    integer    j, bad;
    reg [31:0] y_t, y_inc;
    reg [15:0] y_g [0:3];
    reg [39:0] y_num, y_den;
    reg [3:0]  y_fail;

    always @(posedge clk) begin
        if (rst_n && lpf_valid) begin
            r = $fscanf(f_lpf, "%h %h\n", e_i, e_q);
            if (r != 2) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR lpf: extra output sample %0d", lpf_seen);
            end else if (lpf_i !== e_i || lpf_q !== e_q) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR lpf sample %0d: got %03h %03h expected %03h %03h", lpf_seen, lpf_i, lpf_q, e_i, e_q);
            end
            lpf_seen = lpf_seen + 1;
        end
        if (rst_n && dbg_valid) begin
            r = $fscanf(f_dbg, "%h %h\n", e_m, e_b);
            if (r != 2) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR dbg: extra record %0d", dbg_seen);
            end else if (dbg_metric !== e_m || dbg_bits !== e_b) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR dbg sample %0d: got metric %04h bits %04h expected %04h %04h",
                             dbg_seen, dbg_metric, dbg_bits, e_m, e_b);
            end
            dbg_seen = dbg_seen + 1;
        end
        if (rst_n && trig) begin
            r = $fscanf(f_trig, "%d\n", e_t);
            if (r != 1) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR trig: unexpected trigger at %0d", t_trig);
            end else if (t_trig !== e_t) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR trig %0d: t_trig %0d expected %0d", trig_seen, t_trig, e_t);
            end else
                $display("  trigger %0d at sample %0d (ok)", trig_seen, t_trig);
            trig_seen = trig_seen + 1;
        end
        if (rst_n && cfo_valid) begin
            r = $fscanf(f_cfo, "%h %h %h %h %h %h %h %h %h %h %h %h %h %h\n", x_t,
                        x_a[0], x_a[1], x_a[2], x_a[3], x_a[4], x_a[5], x_a[6], x_a[7],
                        x_g[0], x_g[1], x_g[2], x_g[3], x_inc);
            if (r != 14) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR cfo: unexpected CFO result for t %0d", cfo_t);
            end else begin
                bad = (cfo_t !== x_t) || (cfo_inc !== x_inc) ||
                      (a16_re !== x_a[0]) || (a16_im !== x_a[1]) || (a32_re !== x_a[2]) || (a32_im !== x_a[3]) ||
                      (a128_re !== x_a[4]) || (a128_im !== x_a[5]) || (a256_re !== x_a[6]) || (a256_im !== x_a[7]) ||
                      (ang16 !== x_g[0]) || (ang32 !== x_g[1]) || (ang128 !== x_g[2]) || (ang256 !== x_g[3]);
                if (bad) begin
                    errors = errors + 1;
                    if (errors <= maxerr) begin
                        $display("ERROR cfo %0d: t %0d (exp %0d), cfo_inc %08h (exp %08h)", cfo_seen, cfo_t, x_t, cfo_inc, x_inc);
                        $display("    A16 %h %h (exp %h %h)  A32 %h %h (exp %h %h)", a16_re, a16_im, x_a[0], x_a[1], a32_re, a32_im, x_a[2], x_a[3]);
                        $display("    A128 %h %h (exp %h %h)  A256 %h %h (exp %h %h)", a128_re, a128_im, x_a[4], x_a[5], a256_re, a256_im, x_a[6], x_a[7]);
                        $display("    angles %h %h %h %h (exp %h %h %h %h)", ang16, ang32, ang128, ang256, x_g[0], x_g[1], x_g[2], x_g[3]);
                    end
                end else if (cfo_seen < 20)
                    $display("  CFO %0d for t_trig %0d: cfo_inc %0d (ok)", cfo_seen, cfo_t, $signed(cfo_inc));
            end
            cfo_seen = cfo_seen + 1;
        end
        if (rst_n && ev_valid) begin
            r = $fscanf(f_tone, "%h %h %h %h %h %h %h %h %h\n", y_t, y_inc, y_g[0], y_g[1], y_g[2], y_g[3],
                        y_num, y_den, y_fail);
            if (r != 9) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR event: unexpected event for t %0d", ev_t);
            end else begin
                bad = (ev_t !== y_t) || (ev_inc !== y_inc) || (ev_a16 !== y_g[0]) || (ev_a32 !== y_g[1]) ||
                      (ev_a128 !== y_g[2]) || (ev_a256 !== y_g[3]) || (ev_num !== y_num) ||
                      (ev_den !== y_den) || (ev_fail !== y_fail[0]);
                if (bad) begin
                    errors = errors + 1;
                    if (errors <= maxerr)
                        $display("ERROR event %0d: t %0d (exp %0d) cfo_inc %08h (exp %08h) num %010h (exp %010h) den %010h (exp %010h) fail %0d (exp %0d)",
                                 ev_seen, ev_t, y_t, ev_inc, y_inc, ev_num, y_num, ev_den, y_den, ev_fail, y_fail[0]);
                end else if (ev_seen < 20)
                    $display("  event %0d for t_trig %0d: tone num %0d den %0d, %s (ok)", ev_seen, ev_t, ev_num, ev_den,
                             ev_fail ? "FAIL" : "pass");
                if (ev_fail) ev_fails = ev_fails + 1;
            end
            ev_seen = ev_seen + 1;
        end
        if (rst_n && tm_valid) begin
            hit = -1;
            for (k2 = 0; k2 < n_tm; k2 = k2 + 1)
                if (hit < 0 && !xt_used[k2] && xt_t[k2] == tm_t) hit = k2;
            if (hit < 0) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR timing: unexpected record for t %0d (n_end %0d status %0d)", tm_t, tm_n_end, tm_status);
            end else begin
                xt_used[hit] = 1'b1;
                if (tm_n_end !== xt_n[hit] || tm_status !== xt_s[hit] || tm_n_pos !== xt_p[hit] ||
                    tm_p_max !== xt_m[hit] || tm_p_sum !== xt_u[hit]) begin
                    errors = errors + 1;
                    if (errors <= maxerr)
                        $display("ERROR timing for t %0d: n_end %0d status %0d n_pos %0d p_max %0d p_sum %0d, expected %0d %0d %0d %0d %0d",
                                 tm_t, tm_n_end, tm_status, tm_n_pos, tm_p_max, tm_p_sum,
                                 xt_n[hit], xt_s[hit], xt_p[hit], xt_m[hit], xt_u[hit]);
                end else if (tm_seen < 20)
                    $display("  timing for t_trig %0d: %s n_end %0d, peak/mean %0d (ok)", tm_t,
                             tm_status[2] ? "BUSY" : tm_status == 3'd1 ? "good" : "abort", tm_n_end,
                             (tm_p_sum != 0) ? (tm_p_max * tm_n_pos / tm_p_sum) : 0);
                if (tm_status == 3'd1) tm_good = tm_good + 1;
                if (tm_status[2]) tm_busy_n = tm_busy_n + 1;
            end
            tm_seen = tm_seen + 1;
        end
        if (rst_n && tm_err) begin
            errors = errors + 1;
            if (errors <= maxerr) $display("ERROR timing: start queue overflow");
        end
        if (rst_n && s_valid && !s_ready) stalls = stalls + 1;
    end

    // ------------------------------------------------------------------ stimulus
    integer i, w, idle, rnd;
    reg [15:0] si, sq;
    initial begin
        gap = 3; seed = 1; maxerr = 10; fixgap = 0;
        rc = $value$plusargs("gap=%d", gap);
        rc = $value$plusargs("fixgap=%d", fixgap);
        rc = $value$plusargs("seed=%d", seed);
        rc = $value$plusargs("maxerr=%d", maxerr);

        f_cfg  = $fopen("cfg.txt", "r");
        f_in   = $fopen("in.txt", "r");
        f_lpf  = $fopen("exp_lpf.txt", "r");
        f_dbg  = $fopen("exp_dbg.txt", "r");
        f_trig = $fopen("exp_trig.txt", "r");
        f_cfo  = $fopen("exp_cfo.txt", "r");
        f_tone = $fopen("exp_tone.txt", "r");
        f_open = $fopen("exp_open.txt", "r");
        f_tm   = $fopen("exp_tm.txt", "r");
        if (f_cfg == 0 || f_in == 0 || f_lpf == 0 || f_dbg == 0 || f_trig == 0 || f_cfo == 0 || f_tone == 0 ||
            f_open == 0 || f_tm == 0) begin
            $display("FAIL: cannot open the vector files (cfg.txt ...) in the current directory");
            $finish;
        end
        r = $fscanf(f_cfg, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q,
                    cfg_blank, cfg_hold, n_samples, n_trig, cfg_ncfo, n_cfo_ev, cfg_lin, n_out, cfg_tpk, n_tm);
        if (r != 15) begin
            $display("FAIL: cfg.txt has %0d fields, expected 15 - rerun make_tb_vectors.py", r);
            $finish;
        end
        t_pk = cfg_tpk[15:0];
        for (k2 = 0; k2 < n_tm; k2 = k2 + 1) begin
            r = $fscanf(f_tm, "%h %h %h %h %h %h\n", xt_t[k2], xt_n[k2], xt_s[k2], xt_p[k2], xt_m[k2], xt_u[k2]);
            xt_used[k2] = 1'b0;
        end
        input_stage = cfg_stage[1:0];  dc_shift  = cfg_shift[3:0];
        eta_hi_sq   = cfg_hi[15:0];  eta_lo_sq = cfg_lo[15:0];  qual_len = cfg_q[15:0];
        blank_len   = cfg_blank;     arm_holdoff = cfg_hold;  n_cfo = cfg_ncfo[15:0];
        tone_lin    = cfg_lin[23:0];
        $display("Vector: %0d samples, input_stage %0d, dc_shift %0d, eta_hi_sq %0d, eta_lo_sq %0d, Q %0d, blank %0d, holdoff %0d, N_CFO %0d, %0d trigger(s), %0d CFO result(s); max gap %0d",
                 n_samples, cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q, cfg_blank, cfg_hold, cfg_ncfo, n_trig, n_cfo_ev, gap);

        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (20) @(negedge clk);

        for (i = 0; i < n_samples; i = i + 1) begin
            r = $fscanf(f_in, "%h %h\n", si, sq);
            rnd  = $random(seed);
            idle = fixgap ? gap : (gap > 0) ? ((rnd & 32'h7fffffff) % (gap + 1)) : 0;
            // inputs change on the falling edge only, so the DUT (rising edge) never races them;
            // a sample is taken at the first rising edge with tready = 1
            for (w = 0; w < idle; w = w + 1) begin
                @(negedge clk);
                s_valid = 1'b0;
            end
            @(negedge clk);
            s_valid = 1'b1;
            s_data  = {sq, si};
            while (!s_ready) @(negedge clk);
            if (i % 50000 == 49999) $display("  %0d samples in, %0d errors", i + 1, errors);
        end
        @(negedge clk);
        s_valid = 1'b0;
        // wait until nothing is in flight (the last checks take up to 2 x ceil(13 / NU) x N clocks)
        guard = 0;
        @(negedge clk);
        while (busy && guard < 30 * cfg_ncfo + 60000) begin @(negedge clk); guard = guard + 1; end
        if (busy) begin errors = errors + 1; $display("ERROR: still busy %0d clocks after the input ended", guard); end
        repeat (20) @(negedge clk);
        if (busy) begin errors = errors + 1; $display("ERROR: busy again after it went low"); end
        r = $fscanf(f_open, "%d %d\n", x_open, x_open_t);
        if (open_n !== x_open[3:0] || (x_open != 0 && open_t !== x_open_t)) begin
            errors = errors + 1;
            $display("ERROR: %0d open window(s) at the end (first t %0d), expected %0d (t %0d)", open_n, open_t, x_open, x_open_t);
        end else if (x_open != 0)
            $display("  window open at the end (truncated): t_trig %0d (ok)", open_t);

        if (lpf_seen != n_out) begin
            errors = errors + 1; $display("ERROR: %0d LPF samples out, expected %0d", lpf_seen, n_out);
        end
        if (dbg_seen != n_out) begin
            errors = errors + 1; $display("ERROR: %0d debug records out, expected %0d", dbg_seen, n_out);
        end
        if (trig_seen != n_trig) begin
            errors = errors + 1; $display("ERROR: %0d triggers, expected %0d", trig_seen, n_trig);
        end
        if (cfo_seen != n_cfo_ev) begin
            errors = errors + 1; $display("ERROR: %0d CFO results, expected %0d", cfo_seen, n_cfo_ev);
        end
        if (ev_seen != n_cfo_ev) begin
            errors = errors + 1; $display("ERROR: %0d events, expected %0d", ev_seen, n_cfo_ev);
        end
        if (tm_seen != n_tm) begin
            errors = errors + 1; $display("ERROR: %0d timing records, expected %0d", tm_seen, n_tm);
        end
        $display("  input stalled for %0d clocks (tone check catching up, timing searches)", stalls);
        if (errors == 0)
            $display("PASS: %0d samples, %0d debug records, %0d trigger(s), %0d CFO result(s), %0d event(s) (%0d failing the tone check), %0d timing record(s) (%0d good, %0d busy) match the bit-true model",
                     n_samples, dbg_seen, trig_seen, cfo_seen, ev_seen, ev_fails, tm_seen, tm_good, tm_busy_n);
        else
            $display("FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
