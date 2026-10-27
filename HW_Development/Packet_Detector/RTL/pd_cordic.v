// pd_cordic.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Iterative vectoring CORDIC, bit-true to pd_reference.cordic_angle:
//   1. Normalise: common shift s = bit_length(max(|re|, |im|)) - 22; right shifts (s > 0) are
//      arithmetic (floor), left shifts (s < 0) exact. re = im = 0 -> angle 0.
//   2. x < 0: x, y = -x, -y and z = half a turn (2^21).
//   3. 16 micro-rotations: y >= 0: x += y >>> i, y -= x >>> i, z += ATAN[i]; else the opposite.
//   4. angle = rnd(z >>> 6), signed 16-bit (-32768..32767 = -pi..pi).
// Angle accumulator: 2^22 = one full turn. ATAN[i] = round(atan(2^-i) / 2pi * 2^22).
// start (with re/im) -> done pulse with angle 21 clocks later: 3 clocks normalisation
// (magnitudes, leading bit, shift + pre-rotation), 16 iterations, rounding.
`timescale 1ns / 1ps
`default_nettype none

module pd_cordic (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [36:0] re,
    input  wire signed [36:0] im,
    output wire               busy,
    output reg                done,
    output reg  signed [15:0] angle
);
    localparam [2:0] S_IDLE = 3'd0, S_ABS = 3'd1, S_LEN = 3'd2, S_SHIFT = 3'd3, S_ITER = 3'd4,
                     S_OUT = 3'd5;

    function signed [23:0] atan_tab;
        input [3:0] i;
        begin
            case (i)
                4'd0:  atan_tab = 24'sd524288;
                4'd1:  atan_tab = 24'sd309505;
                4'd2:  atan_tab = 24'sd163534;
                4'd3:  atan_tab = 24'sd83012;
                4'd4:  atan_tab = 24'sd41667;
                4'd5:  atan_tab = 24'sd20854;
                4'd6:  atan_tab = 24'sd10430;
                4'd7:  atan_tab = 24'sd5215;
                4'd8:  atan_tab = 24'sd2608;
                4'd9:  atan_tab = 24'sd1304;
                4'd10: atan_tab = 24'sd652;
                4'd11: atan_tab = 24'sd326;
                4'd12: atan_tab = 24'sd163;
                4'd13: atan_tab = 24'sd81;
                4'd14: atan_tab = 24'sd41;
                default: atan_tab = 24'sd20;
            endcase
        end
    endfunction

    function [5:0] bitlen37;
        input [36:0] m;
        integer b;
        begin
            bitlen37 = 6'd0;
            for (b = 0; b < 37; b = b + 1) if (m[b]) bitlen37 = b[5:0] + 6'd1;
        end
    endfunction

    // ---- normalisation, over three clocks
    reg  signed [36:0] r_re, r_im;              // inputs captured at start
    reg  [36:0]        a_re, a_im;              // S_ABS:  magnitudes
    reg  [5:0]         bl;                      // S_LEN:  bit length of the larger one
    wire [36:0] m    = (a_re > a_im) ? a_re : a_im;
    wire        rsh  = (bl > 6'd22);
    wire [5:0]  sr   = bl - 6'd22;              // right shift when rsh
    wire [5:0]  sl   = 6'd22 - bl;              // left shift otherwise
    wire signed [36:0] re_r = r_re >>> sr;
    wire signed [36:0] im_r = r_im >>> sr;
    wire signed [36:0] re_l = r_re <<< sl;
    wire signed [36:0] im_l = r_im <<< sl;
    wire signed [24:0] xn   = rsh ? re_r[24:0] : re_l[24:0];
    wire signed [24:0] yn   = rsh ? im_r[24:0] : im_l[24:0];

    reg  [2:0]         state;
    reg  [3:0]         it;
    reg  signed [24:0] x, y;
    reg  signed [23:0] z;
    wire signed [24:0] xs = x >>> it;
    wire signed [24:0] ys = y >>> it;
    wire signed [23:0] zr = (z + 24'sd32) >>> 6;

    assign busy = (state != S_IDLE);

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; angle <= 16'sd0; it <= 4'd0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE:
                    if (start) begin
                        r_re <= re; r_im <= im;
                        state <= S_ABS;
                    end
                S_ABS: begin
                    a_re  <= r_re[36] ? -r_re : r_re;
                    a_im  <= r_im[36] ? -r_im : r_im;
                    state <= S_LEN;
                end
                S_LEN: begin
                    bl    <= bitlen37(m);
                    it    <= 4'd0;
                    state <= S_SHIFT;
                end
                S_SHIFT: begin                       // normalise and pre-rotate
                    if (bl == 6'd0) begin
                        x <= 25'sd0; y <= 25'sd0; z <= 24'sd0;
                        state <= S_OUT;              // re = im = 0: angle 0
                    end else begin
                        if (xn < 0) begin
                            x <= -xn; y <= -yn; z <= 24'sd2097152;   // half turn, 2^21
                        end else begin
                            x <= xn;  y <= yn;  z <= 24'sd0;
                        end
                        state <= S_ITER;
                    end
                end
                S_ITER: begin
                    if (y >= 0) begin
                        x <= x + ys; y <= y - xs; z <= z + atan_tab(it);
                    end else begin
                        x <= x - ys; y <= y + xs; z <= z - atan_tab(it);
                    end
                    it <= it + 4'd1;
                    if (it == 4'd15) state <= S_OUT;
                end
                S_OUT: begin
                    angle <= zr[15:0];
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
