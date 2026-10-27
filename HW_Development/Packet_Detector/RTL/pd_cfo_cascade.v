// pd_cfo_cascade.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Integer frequency cascade, bit-true to pd_reference.cfo_cascade. NCO units (Fs / 2^24):
//   f = 16 ang16;  for l = 32, 128, 256:
//     f_l = ang_l * 256 / l;  k = (f - f_l + step/2) >>> log2(step);  f = f_l + (k << log2(step))
//   step = 2^24 / l = 2^19, 2^17, 2^16.   Output: f_hat and cfo_inc = -f_hat.
// Pipelined, one lag per clock: in_valid -> out_valid 3 clocks later (one result per clock).
`timescale 1ns / 1ps
`default_nettype none

module pd_cfo_cascade (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_valid,
    input  wire signed [15:0] ang16,
    input  wire signed [15:0] ang32,
    input  wire signed [15:0] ang128,
    input  wire signed [15:0] ang256,
    output reg                out_valid,
    output reg  signed [31:0] f_hat,
    output reg  signed [31:0] cfo_inc
);
    function signed [31:0] sx16;                 // sign-extend 16 -> 32 bits
        input signed [15:0] a;
        begin sx16 = {{16{a[15]}}, a}; end
    endfunction

    // stage 1: lag 32
    wire signed [31:0] f16  = sx16(ang16) <<< 4;
    wire signed [31:0] fl32 = sx16(ang32) <<< 3;
    wire signed [31:0] k32  = (f16 - fl32 + 32'sd262144) >>> 19;
    reg  signed [31:0] f32_r;
    reg  signed [15:0] a128_1, a256_1;
    // stage 2: lag 128
    wire signed [31:0] fl128 = sx16(a128_1) <<< 1;
    wire signed [31:0] k128  = (f32_r - fl128 + 32'sd65536) >>> 17;
    reg  signed [31:0] f128_r;
    reg  signed [15:0] a256_2;
    // stage 3: lag 256
    wire signed [31:0] fl256 = sx16(a256_2);
    wire signed [31:0] k256  = (f128_r - fl256 + 32'sd32768) >>> 16;
    wire signed [31:0] f256  = fl256 + (k256 <<< 16);
    reg  [1:0]         v;

    always @(posedge clk) begin
        if (!rst_n) begin
            v <= 2'd0; out_valid <= 1'b0;
        end else begin
            v         <= {v[0], in_valid};
            out_valid <= v[1];
        end
        f32_r  <= fl32 + (k32 <<< 19);
        a128_1 <= ang128;  a256_1 <= ang256;
        f128_r <= fl128 + (k128 <<< 17);
        a256_2 <= a256_1;
        f_hat   <= f256;
        cfo_inc <= -f256;
    end
endmodule

`default_nettype wire
