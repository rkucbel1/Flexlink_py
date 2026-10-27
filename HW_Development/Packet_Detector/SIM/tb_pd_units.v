// tb_pd_units.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Directed unit tests against the bit-true model (files from make_tb_vectors.py units):
//   cordic.txt   "re im angle"            pd_cordic, one vector at a time (37-bit re/im)
//   cascade.txt  "a16 a32 a128 a256 inc"  pd_cfo_cascade (3-clock pipeline, one set per clock)
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

        if (errors == 0) $display("PASS: CORDIC and cascade unit tests match the bit-true model");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
