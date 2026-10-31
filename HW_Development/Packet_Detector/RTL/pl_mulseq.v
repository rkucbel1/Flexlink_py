// pl_mulseq.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Sequential unsigned multiplier for the payload receiver's once-per-packet products:
// start (one clock, while idle) with a, b; done (one clock) WB + 1 clocks later with p = a * b.
`timescale 1ns / 1ps
`default_nettype none

module pl_mulseq #(
    parameter integer WA = 48,
    parameter integer WB = 48
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [WA-1:0]      a,
    input  wire [WB-1:0]      b,
    output reg                done,
    output reg  [WA+WB-1:0]   p
);
    reg [WA+WB-1:0] aa;
    reg [WB-1:0]    bb;
    reg [7:0]       n;
    reg             run;
    always @(posedge clk) begin
        done <= 1'b0;
        if (!rst_n) begin
            run <= 1'b0;
        end else if (start) begin
            aa <= {{WB{1'b0}}, a};
            bb <= b;
            p <= {(WA+WB){1'b0}};
            n <= 8'd0;
            run <= 1'b1;
        end else if (run) begin
            if (bb[0]) p <= p + aa;
            aa <= aa << 1;
            bb <= bb >> 1;
            n <= n + 8'd1;
            if (n == WB - 1) begin run <= 1'b0; done <= 1'b1; end
        end
    end
endmodule

`default_nettype wire
