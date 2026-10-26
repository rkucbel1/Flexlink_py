// tb_pd_detect.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Self-checking testbench for pd_detect_top against the bit-true model. Reads the files written
// by make_tb_vectors.py (cfg.txt, in.txt, exp_lpf.txt, exp_dbg.txt, exp_trig.txt) from the
// current directory (run_sim.sh runs the simulator inside the vector's folder), feeds the
// samples with random gaps in tvalid (+gap=N: up to N idle clocks between samples, 0 = one
// sample per clock, +seed=S), and checks every LPF output sample, every debug record and every
// trigger. Prints PASS / FAIL and the first +maxerr=N (default 10) mismatches.
// Plain Verilog-2001 (xsim, Verilator).
`timescale 1ns / 1ps
`default_nettype none

module tb_pd_detect;
    reg         clk;
    initial     clk = 1'b0;
    always #5   clk = ~clk;                     // 100 MHz

    reg         rst_n;
    reg         input_stage;
    reg  [3:0]  dc_shift;
    reg  [15:0] eta_hi_sq, eta_lo_sq, qual_len;
    reg  [31:0] blank_len, arm_holdoff;
    reg         s_valid;
    reg  [31:0] s_data;
    wire        s_ready;
    wire        lpf_valid, trig, dbg_valid;
    wire [11:0] lpf_i, lpf_q;
    wire [31:0] t_trig;
    wire [15:0] dbg_metric, dbg_bits;

    pd_detect_top dut (
        .clk(clk), .rst_n(rst_n),
        .input_stage(input_stage), .dc_shift(dc_shift),
        .eta_hi_sq(eta_hi_sq), .eta_lo_sq(eta_lo_sq), .qual_len(qual_len),
        .blank_len(blank_len), .arm_holdoff(arm_holdoff),
        .s_iq_tvalid(s_valid), .s_iq_tdata(s_data), .s_iq_tready(s_ready),
        .lpf_valid(lpf_valid), .lpf_i(lpf_i), .lpf_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .dbg_valid(dbg_valid), .dbg_metric(dbg_metric), .dbg_bits(dbg_bits));

    // ------------------------------------------------------------------ files and settings
    integer f_cfg, f_in, f_lpf, f_dbg, f_trig, r, rc;
    integer n_samples, n_trig, gap, seed, maxerr;
    integer cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q, cfg_blank, cfg_hold;

    // ------------------------------------------------------------------ checkers
    integer errors, lpf_seen, dbg_seen, trig_seen;
    initial begin
        errors = 0; lpf_seen = 0; dbg_seen = 0; trig_seen = 0;
        rst_n = 1'b0; s_valid = 1'b0; s_data = 32'd0;
    end
    reg [11:0] e_i, e_q;
    reg [15:0] e_m, e_b;
    integer    e_t;

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
    end

    // ------------------------------------------------------------------ stimulus
    integer i, w, idle, rnd;
    reg [15:0] si, sq;
    initial begin
        gap = 3; seed = 1; maxerr = 10;
        rc = $value$plusargs("gap=%d", gap);
        rc = $value$plusargs("seed=%d", seed);
        rc = $value$plusargs("maxerr=%d", maxerr);

        f_cfg  = $fopen("cfg.txt", "r");
        f_in   = $fopen("in.txt", "r");
        f_lpf  = $fopen("exp_lpf.txt", "r");
        f_dbg  = $fopen("exp_dbg.txt", "r");
        f_trig = $fopen("exp_trig.txt", "r");
        if (f_cfg == 0 || f_in == 0 || f_lpf == 0 || f_dbg == 0 || f_trig == 0) begin
            $display("FAIL: cannot open the vector files (cfg.txt ...) in the current directory");
            $finish;
        end
        r = $fscanf(f_cfg, "%d %d %d %d %d %d %d %d %d", cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q,
                    cfg_blank, cfg_hold, n_samples, n_trig);
        input_stage = cfg_stage[0];  dc_shift  = cfg_shift[3:0];
        eta_hi_sq   = cfg_hi[15:0];  eta_lo_sq = cfg_lo[15:0];  qual_len = cfg_q[15:0];
        blank_len   = cfg_blank;     arm_holdoff = cfg_hold;
        $display("Vector: %0d samples, input_stage %0d, dc_shift %0d, eta_hi_sq %0d, eta_lo_sq %0d, Q %0d, blank %0d, holdoff %0d, %0d trigger(s); max gap %0d",
                 n_samples, cfg_stage, cfg_shift, cfg_hi, cfg_lo, cfg_q, cfg_blank, cfg_hold, n_trig, gap);

        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (20) @(posedge clk);

        for (i = 0; i < n_samples; i = i + 1) begin
            r = $fscanf(f_in, "%h %h\n", si, sq);
            rnd  = $random(seed);
            idle = (gap > 0) ? ((rnd & 32'h7fffffff) % (gap + 1)) : 0;
            for (w = 0; w < idle; w = w + 1) begin
                s_valid <= 1'b0;
                @(posedge clk);
            end
            s_valid <= 1'b1;
            s_data  <= {sq, si};
            @(posedge clk);
            if (i % 50000 == 49999) $display("  %0d samples in, %0d errors", i + 1, errors);
        end
        s_valid <= 1'b0;
        repeat (100) @(posedge clk);

        if (lpf_seen != n_samples) begin
            errors = errors + 1; $display("ERROR: %0d LPF samples out, expected %0d", lpf_seen, n_samples);
        end
        if (dbg_seen != n_samples) begin
            errors = errors + 1; $display("ERROR: %0d debug records out, expected %0d", dbg_seen, n_samples);
        end
        if (trig_seen != n_trig) begin
            errors = errors + 1; $display("ERROR: %0d triggers, expected %0d", trig_seen, n_trig);
        end
        if (errors == 0)
            $display("PASS: %0d samples, %0d debug records, %0d trigger(s) match the bit-true model", n_samples, dbg_seen, trig_seen);
        else
            $display("FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
