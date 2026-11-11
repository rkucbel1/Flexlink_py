// tb_pd_detect.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Self-checking testbench for pd_detect_top against the bit-true model. Reads the files written
// by make_tb_vectors.py (cfg.txt, in.txt, exp_lpf.txt, exp_dbg.txt, exp_trig.txt, exp_cfo.txt,
// exp_tone.txt) from the current directory (run_sim.sh runs the simulator inside the vector's
// folder), feeds the samples with random gaps in tvalid (+gap=N: up to N idle clocks between
// samples, 0 = one sample per clock, +seed=S; +fixgap=1: exactly N, e.g. 4 = 20 Msps; +burst=M
// +burstgap=L: after every M samples also L idle clocks, as the streaming DMA feeds it - the
// receiver catches up with the stream and waits; +burstoff=O: the first pause after O + M samples),
// obeys tready, and checks every LPF output sample,
// every debug record, every trigger, every CFO result (window start, the four lag accumulators,
// the four CORDIC angles and cfo_inc), every event after the tone check (CFO fields, tone
// num / den, fail), every control record (exp_cf.txt, in order) and every Preamble B timing
// record (exp_tm.txt, matched by t_trig, since
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
    wire        cf_valid, cf_crc_ok, cf_err, arm_late;
    wire [4:0]   pl_cnt;
    wire [127:0] pl_head;
    reg          pl_pop = 1'b0;
    wire [4:0]   ch_cnt;
    wire [127:0] ch_head;
    reg          ch_pop = 1'b0;
    wire         ch_err;
    wire [4:0]   lq_cnt;
    wire [127:0] lq_head;
    reg          lq_pop = 1'b0;
    wire [11:0]  tb_cnt;
    wire [31:0]  tb_word;
    reg          tb_pop = 1'b0;
    reg         rearm = 1'b0, arm_late_seen = 1'b0;
    wire [31:0] cf_t, cf_n_end;
    wire [53:0] cf_msg;
    wire [5:0]  cf_shift;
    wire [15:0] cf_metric;

    pd_detect_top dut (
        .clk(clk), .rst_n(rst_n),
        .input_stage(input_stage), .dc_shift(dc_shift),
        .eta_hi_sq(eta_hi_sq), .eta_lo_sq(eta_lo_sq), .qual_len(qual_len),
        .blank_len(blank_len), .arm_holdoff(arm_holdoff), .n_cfo(n_cfo), .tone_lin(tone_lin), .t_pk(t_pk),
        .rearm(rearm), .arm_late(arm_late), .pl_cnt(pl_cnt), .pl_head(pl_head), .pl_pop(pl_pop),
        .ch_cnt(ch_cnt), .ch_head(ch_head), .ch_pop(ch_pop), .ch_err(ch_err),
        .lq_cnt(lq_cnt), .lq_head(lq_head), .lq_pop(lq_pop),
        .tb_cnt(tb_cnt), .tb_word(tb_word), .tb_pop(tb_pop),
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
        .cf_valid(cf_valid), .cf_t(cf_t), .cf_n_end(cf_n_end), .cf_crc_ok(cf_crc_ok), .cf_msg(cf_msg),
        .cf_shift(cf_shift), .cf_metric(cf_metric), .cf_err(cf_err),
        .busy(busy), .open_n(open_n), .open_t(open_t));

    // ------------------------------------------------------------------ files and settings
    integer f_cfg, f_in, f_lpf, f_dbg, f_trig, f_cfo, f_tone, f_open, r, rc, x_open, x_open_t, guard;
    integer n_samples, n_trig, n_cfo_ev, gap, seed, maxerr, fixgap, n_out;
    integer cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q, cfg_blank, cfg_hold, cfg_ncfo, cfg_lin, cfg_tpk, n_tm, cfg_rearm;
    integer f_tm, tm_seen, tm_good, tm_busy_n, k2, j2, hit;
    reg [31:0] xt_t [0:4095];
    reg [31:0] xt_n [0:4095];
    reg [2:0]  xt_s [0:4095];
    reg [15:0] xt_p [0:4095];
    reg [33:0] xt_m [0:4095];
    reg [47:0] xt_u [0:4095];
    reg        xt_used [0:4095];
    integer f_cf, n_cf, cf_seen, cf_ok;
    reg [31:0] xc_t [0:255];
    reg [31:0] xc_n [0:255];
    reg [3:0]  xc_s [0:255];
    reg [7:0]  xc_h [0:255];
    reg [15:0] xc_m [0:255];
    reg [55:0] xc_g [0:255];
    integer f_pl, n_pl, pl_seen, pl_gap, pl_wait;
    reg [31:0] xp_t [0:4095];
    reg [7:0]  xp_s [0:4095];
    reg [31:0] xp_a [0:4095];
    reg [31:0] xp_b [0:4095];
    integer f_ch, n_ch, ch_seen, ch_wait;
    integer f_lq, n_lq, lq_seen, lq_wait;
    integer f_tb, n_tb, tb_seen, tb_wait, tb_k, tb_nw, tb_ok_n;
    reg [31:0] tb_t, tb_hdr;
    reg [31:0] xb_t [0:1023];
    reg [31:0] xb_h [0:1023];                  // {ok, 3'b0, it, w, 3'b0, nbits} as the hardware's header
    reg [31:0] xb_d [0:1023*50];
    reg [31:0] xl_t [0:4095];
    reg [15:0] xl_s [0:4095];
    reg [31:0] xl_a [0:4095];
    reg [31:0] xl_b [0:4095];
    reg [31:0] xh_t [0:4095];
    reg [15:0] xh_s [0:4095];
    reg [31:0] xh_a [0:4095];
    reg [31:0] xh_b [0:4095];

    // ------------------------------------------------------------------ checkers
    integer errors, lpf_seen, dbg_seen, trig_seen, cfo_seen, ev_seen, ev_fails, stalls;
    initial begin
        errors = 0; lpf_seen = 0; dbg_seen = 0; trig_seen = 0; cfo_seen = 0; ev_seen = 0;
        tm_seen = 0; tm_good = 0; tm_busy_n = 0; n_tm = 0; n_cf = 0; cf_seen = 0; cf_ok = 0;
        n_pl = 0; pl_seen = 0; pl_wait = 0; n_ch = 0; ch_seen = 0; ch_wait = 0; n_lq = 0; lq_seen = 0; lq_wait = 0; n_tb = 0; tb_seen = 0; tb_wait = 0; tb_k = 0; tb_ok_n = 0;
        if (!$value$plusargs("plgap=%d", pl_gap)) pl_gap = 0;
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
        if (rst_n && cf_valid) begin
            if (cf_seen >= n_cf) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR control: unexpected record for t %0d", cf_t);
            end else if (cf_t !== xc_t[cf_seen] || cf_n_end !== xc_n[cf_seen] || cf_crc_ok !== xc_s[cf_seen][0] ||
                         cf_shift !== xc_h[cf_seen][5:0] || cf_metric !== xc_m[cf_seen] ||
                         cf_msg !== xc_g[cf_seen][53:0]) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR control %0d: t %0d n_end %0d crc %0d shift %0d metric %0d msg %h, expected %0d %0d %0d %0d %0d %h",
                             cf_seen, cf_t, cf_n_end, cf_crc_ok, cf_shift, $signed(cf_metric), cf_msg, xc_t[cf_seen],
                             xc_n[cf_seen], xc_s[cf_seen][0], xc_h[cf_seen], $signed(xc_m[cf_seen]), xc_g[cf_seen][53:0]);
            end else
                $display("  control field for t_trig %0d: CRC %0s, message %h, metric %0d (ok)", cf_t,
                         cf_crc_ok ? "ok" : "FAIL", cf_msg, $signed(cf_metric));
            if (cf_crc_ok) cf_ok = cf_ok + 1;
            cf_seen = cf_seen + 1;
        end
        // payload records: read whenever there is one, at most one every PL_GAP clocks (tests the
        // hold-off of the speculative FFT when the reader is slow)
        pl_pop <= 1'b0;
        if (pl_wait > 0) pl_wait = pl_wait - 1;
        if (rst_n && pl_cnt != 5'd0 && !pl_pop && pl_wait == 0) begin
            pl_pop <= 1'b1; pl_wait = pl_gap;
            if (pl_seen >= n_pl) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR payload: unexpected record t %0d sym %0d", pl_head[31:0], pl_head[38:32]);
            end else if (pl_head[31:0] !== xp_t[pl_seen] || pl_head[38:32] !== xp_s[pl_seen][6:0] ||
                         pl_head[95:64] !== xp_a[pl_seen] || pl_head[127:96] !== xp_b[pl_seen]) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR payload %0d: t %0d sym %0d sig %h %h, expected %0d %0d %h %h", pl_seen,
                             pl_head[31:0], pl_head[38:32], pl_head[95:64], pl_head[127:96],
                             xp_t[pl_seen], xp_s[pl_seen], xp_a[pl_seen], xp_b[pl_seen]);
            end else if (pl_head[38:32] == 7'd0)
                $display("  payload of t_trig %0d: symbol 0 signature %h %h (ok)", pl_head[31:0], pl_head[95:64], pl_head[127:96]);
            pl_seen = pl_seen + 1;
        end
        // channel-estimate records: as the payload records (same gap)
        ch_pop <= 1'b0;
        if (ch_wait > 0) ch_wait = ch_wait - 1;
        if (rst_n && ch_cnt != 5'd0 && !ch_pop && ch_wait == 0) begin
            ch_pop <= 1'b1; ch_wait = pl_gap;
            if (ch_seen >= n_ch) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR chest: unexpected record t %0d sym %h", ch_head[31:0], ch_head[47:32]);
            end else if (ch_head[31:0] !== xh_t[ch_seen] || ch_head[63:32] !== {16'd0, xh_s[ch_seen]} ||
                         ch_head[95:64] !== xh_a[ch_seen] || ch_head[127:96] !== xh_b[ch_seen]) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR chest %0d: t %0d sym %h a %h b %h, expected %0d %h %h %h", ch_seen,
                             ch_head[31:0], ch_head[63:32], ch_head[95:64], ch_head[127:96],
                             xh_t[ch_seen], xh_s[ch_seen], xh_a[ch_seen], xh_b[ch_seen]);
            end else if (ch_head[47:32] == 16'h8000)
                $display("  channel estimate of t_trig %0d: k %0d, class %0d, %0s-SNR mode, n0 %0d (ok)", ch_head[31:0],
                         ch_head[71:64], ch_head[75:72], ch_head[76] ? "high" : "low", ch_head[95:80]);
            ch_seen = ch_seen + 1;
        end
        // decoder-input records: as the payload records (same gap)
        lq_pop <= 1'b0;
        if (lq_wait > 0) lq_wait = lq_wait - 1;
        if (rst_n && lq_cnt != 5'd0 && !lq_pop && lq_wait == 0) begin
            lq_pop <= 1'b1; lq_wait = pl_gap;
            if (lq_seen >= n_lq) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR decoder input: unexpected record t %0d w %0d", lq_head[31:0], lq_head[47:32]);
            end else if (lq_head[31:0] !== xl_t[lq_seen] || lq_head[63:32] !== {16'd0, xl_s[lq_seen]} ||
                         lq_head[95:64] !== xl_a[lq_seen] || lq_head[127:96] !== xl_b[lq_seen]) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR decoder input %0d: t %0d w %0d a %h b %h, expected %0d %0d %h %h", lq_seen,
                             lq_head[31:0], lq_head[63:32], lq_head[95:64], lq_head[127:96],
                             xl_t[lq_seen], xl_s[lq_seen], xl_a[lq_seen], xl_b[lq_seen]);
            end else
                $display("  decoder input of t_trig %0d, code word %0d: signature %h %h (ok)", lq_head[31:0],
                         lq_head[47:32], lq_head[95:64], lq_head[127:96]);
            lq_seen = lq_seen + 1;
        end
        // transport blocks: one word per read, at least 3 clocks apart (as AXI-Lite)
        tb_pop <= 1'b0;
        if (tb_wait > 0) tb_wait = tb_wait - 1;
        else if (rst_n && tb_cnt != 12'd0) begin
            tb_pop <= 1'b1; tb_wait = 3 + pl_gap / 64;
            if (tb_k == 0) tb_t = tb_word;
            else if (tb_k == 1) begin tb_hdr = tb_word; tb_nw = (tb_word[10:0] + 31) / 32; end
            else if (tb_seen < n_tb && tb_word !== xb_d[tb_seen * 50 + tb_k - 2]) begin
                errors = errors + 1;
                if (errors <= maxerr) $display("ERROR transport block %0d word %0d: %h, expected %h", tb_seen, tb_k - 2,
                                               tb_word, xb_d[tb_seen * 50 + tb_k - 2]);
            end
            tb_k = tb_k + 1;
            if (tb_k >= 2 && tb_k == tb_nw + 2) begin
                if (tb_seen >= n_tb) begin
                    errors = errors + 1;
                    if (errors <= maxerr) $display("ERROR transport block: unexpected, t %0d", tb_t);
                end else if (tb_t !== xb_t[tb_seen] || tb_hdr !== xb_h[tb_seen]) begin
                    errors = errors + 1;
                    if (errors <= maxerr) $display("ERROR transport block %0d: t %0d header %h, expected t %0d header %h",
                                                   tb_seen, tb_t, tb_hdr, xb_t[tb_seen], xb_h[tb_seen]);
                end else
                    $display("  transport block of t_trig %0d, code word %0d: %0d bits, CRC %0s, %0d iteration(s) (ok)",
                             tb_t, tb_hdr[23:14], tb_hdr[10:0], tb_hdr[31] ? "ok" : "FAIL", tb_hdr[27:24]);
                if (tb_hdr[31]) tb_ok_n = tb_ok_n + 1;
                tb_seen = tb_seen + 1; tb_k = 0;
            end
        end
        if (rst_n && ch_err) begin
            errors = errors + 1;
            if (errors <= maxerr) $display("ERROR chest: packet queue overflow");
        end
        if (rst_n && arm_late && !arm_late_seen) begin
            errors = errors + 1; arm_late_seen = 1'b1;
            if (errors <= maxerr) $display("ERROR re-arm: a decision came after its sample");
        end
        if (rst_n && cf_err) begin
            errors = errors + 1;
            if (errors <= maxerr) $display("ERROR control: start queue overflow");
        end
        if (rst_n && tm_err) begin
            errors = errors + 1;
            if (errors <= maxerr) $display("ERROR timing: start queue overflow");
        end
        if (rst_n && s_valid && !s_ready) stalls = stalls + 1;
    end

    // ------------------------------------------------------------------ stimulus
    integer i, w, idle, rnd, burst, burstgap, burstoff, hold;
    reg [15:0] si, sq;
    initial begin
        gap = 3; seed = 1; maxerr = 10; fixgap = 0;
        rc = $value$plusargs("gap=%d", gap);
        rc = $value$plusargs("fixgap=%d", fixgap);
        rc = $value$plusargs("seed=%d", seed);
        rc = $value$plusargs("maxerr=%d", maxerr);
        burst = 0; burstgap = 0; burstoff = 0;
        rc = $value$plusargs("burst=%d", burst);
        rc = $value$plusargs("burstgap=%d", burstgap);
        rc = $value$plusargs("burstoff=%d", burstoff);

        f_cfg  = $fopen("cfg.txt", "r");
        f_in   = $fopen("in.txt", "r");
        f_lpf  = $fopen("exp_lpf.txt", "r");
        f_dbg  = $fopen("exp_dbg.txt", "r");
        f_trig = $fopen("exp_trig.txt", "r");
        f_cfo  = $fopen("exp_cfo.txt", "r");
        f_tone = $fopen("exp_tone.txt", "r");
        f_open = $fopen("exp_open.txt", "r");
        f_tm   = $fopen("exp_tm.txt", "r");
        f_cf   = $fopen("exp_cf.txt", "r");
        f_pl   = $fopen("exp_pl.txt", "r");
        f_ch   = $fopen("exp_ch.txt", "r");
        f_lq   = $fopen("exp_lq.txt", "r");
        f_tb   = $fopen("exp_tb.txt", "r");
        if (f_cfg == 0 || f_in == 0 || f_lpf == 0 || f_dbg == 0 || f_trig == 0 || f_cfo == 0 || f_tone == 0 ||
            f_open == 0 || f_tm == 0 || f_cf == 0 || f_pl == 0 || f_ch == 0 || f_lq == 0 || f_tb == 0) begin
            $display("FAIL: cannot open the vector files (cfg.txt ...) in the current directory");
            $finish;
        end
        r = $fscanf(f_cfg, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q,
                    cfg_blank, cfg_hold, n_samples, n_trig, cfg_ncfo, n_cfo_ev, cfg_lin, n_out, cfg_tpk, n_tm, cfg_rearm);
        if (r != 16) begin
            $display("FAIL: cfg.txt has %0d fields, expected 16 - rerun make_tb_vectors.py", r);
            $finish;
        end
        t_pk = cfg_tpk[15:0];
        rearm = cfg_rearm[0];
        for (k2 = 0; k2 < n_tm; k2 = k2 + 1) begin
            r = $fscanf(f_tm, "%h %h %h %h %h %h\n", xt_t[k2], xt_n[k2], xt_s[k2], xt_p[k2], xt_m[k2], xt_u[k2]);
            xt_used[k2] = 1'b0;
        end
        r = $fscanf(f_cf, "%d\n", n_cf);
        for (k2 = 0; k2 < n_cf; k2 = k2 + 1)
            r = $fscanf(f_cf, "%h %h %h %h %h %h\n", xc_t[k2], xc_n[k2], xc_s[k2], xc_h[k2], xc_m[k2], xc_g[k2]);
        r = $fscanf(f_pl, "%d\n", n_pl);
        for (k2 = 0; k2 < n_pl; k2 = k2 + 1)
            r = $fscanf(f_pl, "%h %h %h %h\n", xp_t[k2], xp_s[k2], xp_a[k2], xp_b[k2]);
        r = $fscanf(f_tb, "%d\n", n_tb);
        for (k2 = 0; k2 < n_tb; k2 = k2 + 1) begin
            r = $fscanf(f_tb, "%h %h %h %h %h", xb_t[k2], tb_t, tb_hdr, tb_k, tb_nw);
            // header as the hardware writes it: {crc_ok, 3'b0, iterations, w, 3'b0, nbits}
            xb_h[k2] = {tb_hdr[0], 3'd0, tb_k[3:0], tb_t[9:0], 3'd0, tb_nw[10:0]};
            for (j2 = 0; j2 < (tb_nw + 31) / 32; j2 = j2 + 1) r = $fscanf(f_tb, "%h", xb_d[k2 * 50 + j2]);
        end
        tb_k = 0;
        r = $fscanf(f_lq, "%d\n", n_lq);
        for (k2 = 0; k2 < n_lq; k2 = k2 + 1)
            r = $fscanf(f_lq, "%h %h %h %h\n", xl_t[k2], xl_s[k2], xl_a[k2], xl_b[k2]);
        r = $fscanf(f_ch, "%d\n", n_ch);
        for (k2 = 0; k2 < n_ch; k2 = k2 + 1)
            r = $fscanf(f_ch, "%h %h %h %h\n", xh_t[k2], xh_s[k2], xh_a[k2], xh_b[k2]);
        input_stage = cfg_stage[1:0];  dc_shift  = cfg_shift[3:0];
        eta_hi_sq   = cfg_hi[15:0];  eta_lo_sq = cfg_lo[15:0];  qual_len = cfg_q[15:0];
        blank_len   = cfg_blank;     arm_holdoff = cfg_hold;  n_cfo = cfg_ncfo[15:0];
        tone_lin    = cfg_lin[23:0];
        $display("Vector: %0d samples, input_stage %0d, dc_shift %0d, eta_hi_sq %0d, eta_lo_sq %0d, Q %0d, blank %0d, holdoff %0d, N_CFO %0d, %0d trigger(s), %0d CFO result(s); max gap %0d",
                 n_samples, cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q, cfg_blank, cfg_hold, cfg_ncfo, n_trig, n_cfo_ev, gap);
        if (burst > 0) $display("Bursts: %0d idle clocks after every %0d samples (from sample %0d)", burstgap, burst, burstoff);

        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (20) @(negedge clk);

        for (i = 0; i < n_samples; i = i + 1) begin
            r = $fscanf(f_in, "%h %h\n", si, sq);
            rnd  = $random(seed);
            idle = fixgap ? gap : (gap > 0) ? ((rnd & 32'h7fffffff) % (gap + 1)) : 0;
            if (burst > 0 && i > burstoff && (i - burstoff) % burst == 0) idle = idle + burstgap;
            // inputs change on the falling edge only, so the DUT (rising edge) never races them;
            // a sample is taken at the first rising edge with tready = 1
            for (w = 0; w < idle; w = w + 1) begin
                @(negedge clk);
                s_valid = 1'b0;
            end
            @(negedge clk);
            s_valid = 1'b1;
            s_data  = {sq, si};
            // deadlock guard: the input held for 4 M clocks (no block needs that long) -> report
            // which stall keeps it and stop (a hang would otherwise run forever)
            hold = 0;
            while (!s_ready) begin
                @(negedge clk);
                hold = hold + 1;
                if (hold == 4000000) begin
                    $display("FAIL: DEADLOCK at input sample %0d (time %0t): input held %0d clocks; stall tone %b timing %b cf %b arm %b; busy: lane %b tone %b timing %b cf %b; arm state %0d",
                             i, $time, hold, dut.stall_tone, dut.stall_tm, dut.stall_cf, dut.stall_arm,
                             dut.lane_busy, dut.tone_busy, dut.tm_busy, dut.cf_busy, dut.u_arm.st);
                    $display("       FIFOs: pl %0d ch %0d lq %0d tb %0d words; open windows %0d", pl_cnt, ch_cnt, lq_cnt, tb_cnt, open_n);
                    // pd_cf: reader / start queue (which half of its stall), payload FIFO pointers
                    $display("  pd_cf: active %b rx_more %b cf_ready %b ff_n %0d infl %0d avail %b | wp %0d rp %0d age_rp %0d | queue %0d, first %0d, age_q %0d | pl wr %0d cm %0d rd %0d, pl_room %b",
                             dut.u_cf.active, dut.u_cf.rx_more, dut.u_cf.cf_ready, dut.u_cf.ff_n, dut.u_cf.infl, dut.u_cf.avail,
                             dut.u_cf.wp, dut.u_cf.rp, dut.u_cf.age_rp, dut.u_cf.q_n, dut.u_cf.q_s[dut.u_cf.q_rd], dut.u_cf.age_q,
                             dut.u_cf.pl_wr, dut.u_cf.pl_cm, dut.u_cf.pl_rd, dut.u_cf.pl_room);
                    $display("         busy parts: chest %b (waiting %b) demap %b comb %b dec %b",
                             dut.u_cf.ch_busy, dut.u_cf.ch_wait, dut.u_cf.dm_busy, dut.u_cf.cb_busy, dut.u_cf.dc_busy);
                    // cf_rx: front end (F_IDLE 0, F_IN 1, F_DRAIN 2, F_FFT 3, F_FWAIT 4, F_UNLOAD 5, F_JOB 6), back end
                    $display("  cf_rx: fst %0d bst %0d | isym %0d pos %0d at_pl %b pl_go %b decided %b pidx %0d pl_end %b | job_req %b st_pend %b",
                             dut.u_cf.u_rx.fst, dut.u_cf.u_rx.bst, dut.u_cf.u_rx.isym, dut.u_cf.u_rx.pos, dut.u_cf.u_rx.at_pl,
                             dut.u_cf.u_rx.pl_go, dut.u_cf.u_rx.decided, dut.u_cf.u_rx.pidx, dut.u_cf.u_rx.pl_end,
                             dut.u_cf.u_rx.job_req, dut.u_cf.u_rx.st_pend);
                    // pl_chest: state (E_* in pl_chest.v), symbol slots, packet queue
                    $display("  pl_chest: es %0d | slots wr %0d free %0d (room %b) | packet queue %0d | job_req %b",
                             dut.u_cf.u_ch.es, dut.u_cf.u_ch.wr_ptr, dut.u_cf.u_ch.fr_ptr, dut.u_cf.u_ch.room,
                             dut.u_cf.u_ch.q_n, dut.u_cf.u_ch.job_req);
                    $finish;
                end
            end
            if (i % 50000 == 49999) $display("  %0d samples in, %0d errors", i + 1, errors);
        end
        @(negedge clk);
        s_valid = 1'b0;
        // wait until nothing is in flight (the last checks take up to 2 x ceil(13 / NU) x N clocks)
        guard = 0;
        @(negedge clk);
        while ((busy || pl_cnt != 5'd0 || ch_cnt != 5'd0 || lq_cnt != 5'd0 || tb_cnt != 12'd0) && guard < 30 * cfg_ncfo + 400000 + 50 * pl_gap) begin
            @(negedge clk); guard = guard + 1;
        end
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
        if (cf_seen != n_cf) begin
            errors = errors + 1; $display("ERROR: %0d control records, expected %0d", cf_seen, n_cf);
        end
        if (pl_seen != n_pl) begin
            errors = errors + 1; $display("ERROR: %0d payload records, expected %0d", pl_seen, n_pl);
        end
        if (tb_seen != n_tb) begin
            errors = errors + 1; $display("ERROR: %0d transport blocks, expected %0d", tb_seen, n_tb);
        end
        if (lq_seen != n_lq) begin
            errors = errors + 1; $display("ERROR: %0d decoder-input records, expected %0d", lq_seen, n_lq);
        end
        if (ch_seen != n_ch) begin
            errors = errors + 1; $display("ERROR: %0d channel-estimate records, expected %0d", ch_seen, n_ch);
        end
        $display("  input stalled for %0d clocks (tone check catching up, timing searches)", stalls);
        if (errors == 0)
            $display("PASS: %0d samples, %0d debug records, %0d trigger(s), %0d CFO result(s), %0d event(s) (%0d failing the tone check), %0d timing record(s) (%0d good, %0d busy), %0d control record(s) (%0d CRC ok), %0d payload record(s), %0d channel-estimate record(s), %0d decoder-input record(s), %0d transport block(s) (%0d CRC ok) match the bit-true model",
                     n_samples, dbg_seen, trig_seen, cfo_seen, ev_seen, ev_fails, tm_seen, tm_good, tm_busy_n, cf_seen, cf_ok, pl_seen, ch_seen, lq_seen, tb_seen, tb_ok_n);
        else
            $display("FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
