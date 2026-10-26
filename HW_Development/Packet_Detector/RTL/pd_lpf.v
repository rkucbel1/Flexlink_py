// pd_lpf.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// 17-tap 2.75 MHz low-pass filter, bit-true to pd_reference.lpf:
//   z(n) = sat12(rnd(sum_k H[k] * y(n-k) >>> 17)),  y(n < 0) = 0
//   H = round(H_LP * 2^17): -197 -1402 -3552 -5086 -2831 5649 19032 31693 36923 (symmetric)
// Symmetric form: 8 pre-adds + centre tap, 9 multipliers per rail. The tap delay line shifts
// on in_valid only; the pipeline carries the valid bit. Latency 6 clocks.
`timescale 1ns / 1ps
`default_nettype none

module pd_lpf_rail (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,
    input  wire signed [11:0] x,
    input  wire [4:0]         v,        // valid pipeline from the parent: v[0] = en delayed 1
    output reg  signed [11:0] z
);
    // coefficients H[0..8] (H[16-k] = H[k])
    function signed [17:0] coef;
        input integer k;
        begin
            case (k)
                0: coef = -18'sd197;
                1: coef = -18'sd1402;
                2: coef = -18'sd3552;
                3: coef = -18'sd5086;
                4: coef = -18'sd2831;
                5: coef = 18'sd5649;
                6: coef = 18'sd19032;
                7: coef = 18'sd31693;
                default: coef = 18'sd36923;
            endcase
        end
    endfunction

    reg signed [11:0] tap [0:16];
    reg signed [12:0] pre [0:8];          // stage 1: tap[k] + tap[16-k]; pre[8] = centre
    reg signed [31:0] prod [0:8];         // stage 2
    reg signed [31:0] part [0:2];         // stage 3
    reg signed [31:0] acc;                // stage 4

    wire signed [31:0] acc_h = acc + 32'sd65536;           // + 2^16
    wire signed [31:0] acc_s = acc_h >>> 17;
    wire signed [11:0] z_sat = (acc_s > 2047)  ? 12'sd2047 :
                               (acc_s < -2048) ? -12'sd2048 : acc_s[11:0];
    integer k;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k <= 16; k = k + 1) tap[k] <= 12'sd0;
            z <= 12'sd0;
        end else begin
            if (en) begin
                tap[0] <= x;
                for (k = 1; k <= 16; k = k + 1) tap[k] <= tap[k-1];
            end
            if (v[0]) begin
                for (k = 0; k < 8; k = k + 1)
                    pre[k] <= {tap[k][11], tap[k]} + {tap[16-k][11], tap[16-k]};
                pre[8] <= {tap[8][11], tap[8]};
            end
            if (v[1])
                for (k = 0; k <= 8; k = k + 1)
                    prod[k] <= pre[k] * coef(k);
            if (v[2]) begin
                part[0] <= prod[0] + prod[1] + prod[2];
                part[1] <= prod[3] + prod[4] + prod[5];
                part[2] <= prod[6] + prod[7] + prod[8];
            end
            if (v[3])
                acc <= part[0] + part[1] + part[2];
            if (v[4])
                z <= z_sat;
        end
    end
endmodule


module pd_lpf (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_valid,
    input  wire signed [11:0] in_i,
    input  wire signed [11:0] in_q,
    output wire               out_valid,
    output wire signed [11:0] out_i,
    output wire signed [11:0] out_q
);
    reg [5:0] v;                          // v[0] = in_valid delayed 1, ..., v[5] = output valid
    always @(posedge clk)
        if (!rst_n) v <= 6'd0;
        else        v <= {v[4:0], in_valid};
    assign out_valid = v[5];

    pd_lpf_rail u_i (.clk(clk), .rst_n(rst_n), .en(in_valid), .x(in_i), .v(v[4:0]), .z(out_i));
    pd_lpf_rail u_q (.clk(clk), .rst_n(rst_n), .en(in_valid), .x(in_q), .v(v[4:0]), .z(out_q));
endmodule

`default_nettype wire
