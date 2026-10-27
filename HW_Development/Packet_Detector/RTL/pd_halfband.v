// pd_halfband.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// 43-tap half-band decimator 40 -> 20 Msps, bit-true to pd_reference.halfband:
//   y(m) = sat12(rnd(sum_k HB_Q[k] * x(2m+1-k) >>> 17)),  x(n < 0) = 0
// One output per input pair, after its second sample (input index 2m+1, counted from reset).
// Half-band: every odd tap offset from the centre is zero except the centre (65536 = 1/2), so
// the sum is 11 symmetric pairs (tap k + tap 42-k, k even) times HB_Q[k] plus the centre as a
// shift. 11 multipliers per rail. The delay line shifts on in_valid only; the pipeline carries
// the valid bit, so input gaps don't matter. Latency 6 clocks after the second sample.
`timescale 1ns / 1ps
`default_nettype none

module pd_halfband_rail (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,
    input  wire signed [11:0] x,
    input  wire [4:0]         v,        // v[0]: a pair is complete (delayed 1), ... from the parent
    output reg  signed [11:0] z
);
    // HB_Q[k] for k = 0, 2, ..., 20 (HB_Q[42-k] = HB_Q[k]; HB_Q[21] = 65536)
    function signed [17:0] coef;
        input integer j;                  // pair index, k = 2j
        begin
            case (j)
                0:  coef = 18'sd78;
                1:  coef = -18'sd178;
                2:  coef = 18'sd368;
                3:  coef = -18'sd676;
                4:  coef = 18'sd1148;
                5:  coef = -18'sd1851;
                6:  coef = 18'sd2894;
                7:  coef = -18'sd4500;
                8:  coef = 18'sd7238;
                9:  coef = -18'sd13212;
                default: coef = 18'sd41459;
            endcase
        end
    endfunction

    reg signed [11:0] tap [0:42];         // tap[k] = x(n - k) after sample n
    reg signed [12:0] pre [0:10];         // stage 1: tap[2j] + tap[42-2j]
    reg signed [11:0] ctr1;               //          centre tap[21]
    reg signed [31:0] prod [0:10];        // stage 2
    reg signed [31:0] ctr2;
    reg signed [31:0] part [0:3];         // stage 3
    reg signed [31:0] acc;                // stage 4

    wire signed [31:0] acc_h = acc + 32'sd65536;           // + 2^16
    wire signed [31:0] acc_s = acc_h >>> 17;
    wire signed [11:0] z_sat = (acc_s > 2047)  ? 12'sd2047 :
                               (acc_s < -2048) ? -12'sd2048 : acc_s[11:0];
    integer k;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k <= 42; k = k + 1) tap[k] <= 12'sd0;
            z <= 12'sd0;
        end else begin
            if (en) begin
                tap[0] <= x;
                for (k = 1; k <= 42; k = k + 1) tap[k] <= tap[k-1];
            end
            if (v[0]) begin
                for (k = 0; k <= 10; k = k + 1)
                    pre[k] <= {tap[2*k][11], tap[2*k]} + {tap[42-2*k][11], tap[42-2*k]};
                ctr1 <= tap[21];
            end
            if (v[1]) begin
                for (k = 0; k <= 10; k = k + 1)
                    prod[k] <= pre[k] * coef(k);
                ctr2 <= {{4{ctr1[11]}}, ctr1, 16'd0};            // x * 65536
            end
            if (v[2]) begin
                part[0] <= prod[0] + prod[1] + prod[2];
                part[1] <= prod[3] + prod[4] + prod[5];
                part[2] <= prod[6] + prod[7] + prod[8];
                part[3] <= prod[9] + prod[10] + ctr2;
            end
            if (v[3])
                acc <= part[0] + part[1] + part[2] + part[3];
            if (v[4])
                z <= z_sat;
        end
    end
endmodule


module pd_halfband (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_valid,
    input  wire signed [11:0] in_i,
    input  wire signed [11:0] in_q,
    output wire               out_valid,
    output wire signed [11:0] out_i,
    output wire signed [11:0] out_q
);
    reg       ph;                         // index of the next input is odd
    reg [5:0] v;                          // v[0] = pair complete delayed 1, ..., v[5] = output valid
    always @(posedge clk) begin
        if (!rst_n) begin
            ph <= 1'b0; v <= 6'd0;
        end else begin
            if (in_valid) ph <= !ph;
            v <= {v[4:0], in_valid && ph};
        end
    end
    assign out_valid = v[5];

    pd_halfband_rail u_i (.clk(clk), .rst_n(rst_n), .en(in_valid), .x(in_i), .v(v[4:0]), .z(out_i));
    pd_halfband_rail u_q (.clk(clk), .rst_n(rst_n), .en(in_valid), .x(in_q), .v(v[4:0]), .z(out_q));
endmodule

`default_nettype wire
