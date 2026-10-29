// cf_crc10.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// CRC10 check of the control field (GenerateCRC Gen10: x^10 + x^9 + x^8 + x^7 + x^6 + x^4 +
// x^3 + 1, POLY = the terms below x^10, zero initial value), bit-true to
// cf_reference.crc10: for every bit in order: fb = crc[9] ^ bit, crc = crc << 1, crc ^= POLY if fb.
// The 54 message bits followed by their 10 CRC bits give crc = 0. One bit per clock:
// start -> done 64 clocks later with crc and ok (crc == 0).
`timescale 1ns / 1ps
`default_nettype none

module cf_crc10 #(
    parameter integer N = 64,
    parameter [9:0]   POLY = 10'b1111011001
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [N-1:0] bits,                   // bits[0] first
    output reg          done,
    output reg  [9:0]   crc,
    output wire         ok
);
    reg [N-1:0] sh;
    reg [7:0]   n;
    reg         run;
    assign ok = (crc == 10'd0);
    wire fb = crc[9] ^ sh[0];
    always @(posedge clk) begin
        if (!rst_n) begin
            run <= 1'b0; done <= 1'b0; crc <= 10'd0;
        end else begin
            done <= 1'b0;
            if (start && !run) begin
                sh <= bits; crc <= 10'd0; n <= 8'd0; run <= 1'b1;
            end else if (run) begin
                crc <= {crc[8:0], 1'b0} ^ (fb ? POLY : 10'd0);
                sh  <= sh >> 1;
                n   <= n + 8'd1;
                if (n == N - 1) begin run <= 1'b0; done <= 1'b1; end
            end
        end
    end
endmodule

`default_nettype wire
