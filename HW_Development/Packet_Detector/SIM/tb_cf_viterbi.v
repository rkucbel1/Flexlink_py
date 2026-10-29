// tb_cf_viterbi.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// cf_viterbi and cf_crc10 alone against cf_reference (make_cf_vectors.py units): every case
// is 140 soft values (5-bit) with the model's 70 bits, metric and CRC; many cases have tiny
// values, so most decisions are ties. Run in work/cf_units/. Prints PASS or FAIL.
`timescale 1ns / 1ps
`default_nettype none

module tb_cf_viterbi;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;
    reg q_we = 1'b0, start = 1'b0, crc_start = 1'b0;
    reg [7:0] q_addr;
    reg [4:0] q_data;
    wire done, crc_done, crc_ok;
    wire [69:0] bits;
    wire signed [15:0] metric;
    wire [9:0] crc;
    cf_viterbi dut (.clk(clk), .rst_n(rst_n), .q_we(q_we), .q_addr(q_addr), .q_data(q_data),
                    .start(start), .busy(), .done(done), .bits(bits), .metric(metric));
    cf_crc10 #(.N(64)) ucrc (.clk(clk), .rst_n(rst_n), .start(crc_start), .bits(bits[63:0]),
                             .done(crc_done), .crc(crc), .ok(crc_ok));

    integer fd, r, n, k, p, e_metric, e_crc, errs = 0;
    reg [4:0] q [0:139];
    reg [69:0] e_bits;
    reg [7:0] t0;
    reg [8*256-1:0] dir, fname;
    reg [69:0] rd_bits;
    initial begin
        if (!$value$plusargs("dir=%s", dir)) dir = ".";
        $sformat(fname, "%0s/vit.txt", dir);
        fd = $fopen(fname, "r");
        r = $fscanf(fd, "%d", n);
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        for (k = 0; k < n; k = k + 1) begin
            for (p = 0; p < 140; p = p + 1) begin r = $fscanf(fd, "%h", t0); q[p] = t0[4:0]; end
            r = $fscanf(fd, "%b %d %d", rd_bits, e_metric, e_crc);
            for (p = 0; p < 70; p = p + 1) e_bits[p] = rd_bits[69 - p];
            for (p = 0; p < 140; p = p + 1) begin
                @(negedge clk); q_we = 1'b1; q_addr = p[7:0]; q_data = q[p];
            end
            @(negedge clk); q_we = 1'b0; start = 1'b1;
            @(negedge clk); start = 1'b0;
            while (!done) @(negedge clk);
            crc_start = 1'b1;
            @(negedge clk); crc_start = 1'b0;
            while (!crc_done) @(negedge clk);
            if (bits !== e_bits || metric != e_metric || crc_ok != e_crc[0]) begin
                if (errs < 5) $display("case %0d: bits %b\n   model %b metric %0d/%0d crc %0d/%0d",
                                       k, bits, e_bits, metric, e_metric, crc_ok, e_crc);
                errs = errs + 1;
            end
        end
        if (errs == 0) $display("PASS: %0d Viterbi + CRC cases", n);
        else $display("FAIL: %0d of %0d cases", errs, n);
        $finish;
    end
endmodule

`default_nettype wire
