// tb_pd_units.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Directed unit tests against the bit-true model (files from make_tb_vectors.py units):
//   cordic.txt   "re im angle"            pd_cordic, one vector at a time (37-bit re/im)
//   cascade.txt  "a16 a32 a128 a256 inc"  pd_cfo_cascade (3-clock pipeline, one set per clock)
//   tone.txt     "N inc lin num den fail" + N samples   pd_tone_check, one window at a time
// Run from the folder holding the files. Prints PASS / FAIL.
`timescale 1ns / 1ps
`default_nettype none

module tb_pd_units;
    reg clk;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    reg               rst_n, start;
    reg  signed [36:0] re, im;
    wire              busy, done;
    wire signed [15:0] angle;
    pd_cordic u_cordic (.clk(clk), .rst_n(rst_n), .start(start), .re(re), .im(im),
                        .busy(busy), .done(done), .angle(angle));

    integer f, r, n, errors, maxerr, rc;
    reg [36:0] x_re, x_im;
    reg [15:0] x_ang, x16, x32, x128, x256;
    reg [31:0] x_inc;

    reg               g_v;
    reg  signed [15:0] g16, g32, g128, g256;
    wire              o_v;
    wire signed [31:0] f_hat, inc;
    pd_cfo_cascade u_casc (.clk(clk), .rst_n(rst_n), .in_valid(g_v),
                           .ang16(g16), .ang32(g32), .ang128(g128), .ang256(g256),
                           .out_valid(o_v), .f_hat(f_hat), .cfo_inc(inc));

    // tone check: window samples in, trigger at the first one, CFO result after the last
    reg               t_v, t_trig, t_cv;
    reg  [11:0]       t_i, t_q;
    reg  [31:0]       t_tt, t_inc, t_cnt;
    reg  [15:0]       t_n;
    reg  [23:0]       t_lin;
    wire              t_stall, ev_v, ev_fail;
    wire [31:0]       ev_t, ev_inc;
    wire [15:0]       ev_a16, ev_a32, ev_a128, ev_a256;
    wire [39:0]       ev_num, ev_den;
    pd_tone_check u_tone (.clk(clk), .rst_n(rst_n), .n_cfo(t_n), .tone_lin(t_lin),
                          .in_valid(t_v), .in_i(t_i), .in_q(t_q), .trig(t_trig), .t_trig(t_tt),
                          .cfo_valid(t_cv), .cfo_t(t_tt), .cfo_inc(t_inc),
                          .ang16(16'sd1), .ang32(16'sd2), .ang128(16'sd3), .ang256(16'sd4),
                          .stall(t_stall), .ev_valid(ev_v), .ev_t(ev_t), .ev_cfo_inc(ev_inc),
                          .ev_ang16(ev_a16), .ev_ang32(ev_a32), .ev_ang128(ev_a128), .ev_ang256(ev_a256),
                          .ev_num(ev_num), .ev_den(ev_den), .ev_fail(ev_fail));
    reg  [15:0]       y_n;
    reg  [31:0]       y_inc;
    reg  [23:0]       y_lin;
    reg  [39:0]       y_num, y_den;
    reg  [3:0]        y_fail;
    reg  [11:0]       y_i, y_q;
    integer           nt, nf, j;

    // cascade results are checked as they come out of the pipeline, in order
    reg [31:0] exp_inc [0:262143];
    reg [15:0] exp_a16 [0:262143];
    integer    n_in, n_out;
    always @(posedge clk) begin
        if (rst_n && o_v) begin
            if (inc !== exp_inc[n_out]) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR cascade %0d: ang16 %h ... -> cfo_inc %h, expected %h",
                             n_out, exp_a16[n_out], inc, exp_inc[n_out]);
            end
            n_out = n_out + 1;
        end
    end


    initial begin
        errors = 0; maxerr = 10; rst_n = 1'b0; start = 1'b0; re = 0; im = 0;
        g_v = 1'b0; g16 = 0; g32 = 0; g128 = 0; g256 = 0; n_in = 0; n_out = 0;
        t_v = 1'b0; t_trig = 1'b0; t_cv = 1'b0; t_i = 0; t_q = 0; t_tt = 0; t_inc = 0; t_cnt = 0;
        t_n = 16'd2048; t_lin = 24'd0;
        rc = $value$plusargs("maxerr=%d", maxerr);
        repeat (3) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        // CORDIC
        f = $fopen("cordic.txt", "r");
        if (f == 0) begin $display("FAIL: cordic.txt not found"); $finish; end
        n = 0;
        r = $fscanf(f, "%h %h %h\n", x_re, x_im, x_ang);
        while (r == 3) begin
            re <= x_re; im <= x_im; start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;
            @(posedge clk);
            while (!done) @(posedge clk);
            if (angle !== x_ang) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR cordic %0d: re %h im %h -> angle %h, expected %h", n, x_re, x_im, angle, x_ang);
            end
            n = n + 1;
            r = $fscanf(f, "%h %h %h\n", x_re, x_im, x_ang);
        end
        $display("  CORDIC: %0d vectors", n);

        // cascade
        f = $fopen("cascade.txt", "r");
        if (f == 0) begin $display("FAIL: cascade.txt not found"); $finish; end
        r = $fscanf(f, "%h %h %h %h %h\n", x16, x32, x128, x256, x_inc);
        while (r == 5) begin
            exp_inc[n_in] = x_inc; exp_a16[n_in] = x16;
            g_v <= 1'b1; g16 <= x16; g32 <= x32; g128 <= x128; g256 <= x256;
            @(posedge clk);
            n_in = n_in + 1;
            r = $fscanf(f, "%h %h %h %h %h\n", x16, x32, x128, x256, x_inc);
        end
        g_v <= 1'b0;
        repeat (10) @(posedge clk);
        if (n_out != n_in) begin
            errors = errors + 1; $display("ERROR cascade: %0d results for %0d inputs", n_out, n_in);
        end
        $display("  cascade: %0d angle sets", n_in);

        // tone check (inputs change on the falling edge)
        f = $fopen("tone.txt", "r");
        if (f == 0) begin $display("FAIL: tone.txt not found"); $finish; end
        nt = 0; nf = 0;
        r = $fscanf(f, "%h %h %h %h %h %h\n", y_n, y_inc, y_lin, y_num, y_den, y_fail);
        while (r == 6) begin
            @(negedge clk);
            t_n = y_n; t_lin = y_lin; t_inc = y_inc; t_tt = t_cnt;
            for (j = 0; j < y_n; j = j + 1) begin
                r = $fscanf(f, "%h %h\n", y_i, y_q);
                t_v = 1'b1; t_i = y_i; t_q = y_q; t_trig = (j == 0);
                @(negedge clk);
                if (t_stall) begin errors = errors + 1; $display("ERROR tone: stall in the unit test"); end
                t_cnt = t_cnt + 1;
            end
            t_v = 1'b0; t_trig = 1'b0;
            t_cv = 1'b1;
            @(negedge clk);
            t_cv = 1'b0;
            while (!ev_v) @(negedge clk);
            if (ev_t !== t_tt || ev_inc !== y_inc || ev_num !== y_num || ev_den !== y_den || ev_fail !== y_fail[0] ||
                ev_a16 !== 16'd1 || ev_a256 !== 16'd4) begin
                errors = errors + 1;
                if (errors <= maxerr)
                    $display("ERROR tone %0d: N %0d inc %08h lin %06h -> num %010h den %010h fail %0d, expected %010h %010h %0d",
                             nt, y_n, y_inc, y_lin, ev_num, ev_den, ev_fail, y_num, y_den, y_fail[0]);
            end
            if (y_fail[0]) nf = nf + 1;
            nt = nt + 1;
            r = $fscanf(f, "%h %h %h %h %h %h\n", y_n, y_inc, y_lin, y_num, y_den, y_fail);
        end
        $display("  tone check: %0d windows (%0d failing)", nt, nf);

        if (errors == 0) $display("PASS: CORDIC, cascade and tone-check unit tests match the bit-true model");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
