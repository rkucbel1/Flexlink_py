// pd_dc_blocker.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// DC blocker, bit-true to pd_reference.dc_blocker (one rail per instance of pd_dc_rail):
//   acc(n) = acc(n-1) - rnd(acc(n-1) >>> k) + ((x(n) - x(n-1)) <<< FRAC)
//   y(n)   = sat12(rnd(acc(n) >>> FRAC))
//   rnd(v >>> k) = (v + 2^(k-1)) >>> k,  k = shift (1..15); shift = 0 bypasses (y = x).
// State changes only on in_valid. Latency 2 clocks. Reset: synchronous, active low; acc and
// x(n-1) start at zero. shift must be constant while samples flow.
`timescale 1ns / 1ps
`default_nettype none

module pd_dc_rail #(
    parameter integer FRAC  = 10,
    parameter integer ACC_W = 24
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [3:0]         shift,
    input  wire               en,
    input  wire signed [11:0] x,
    input  wire               v1,       // en delayed by one clock (from the parent)
    output reg  signed [11:0] y
);
    reg  signed [11:0]      x_prev;
    reg  signed [11:0]      x_d1;          // bypass path
    reg  signed [ACC_W-1:0] acc;

    // next accumulator value (ACC_W + 2 bits so nothing can overflow before the check)
    wire signed [ACC_W+1:0] acc_e = {{2{acc[ACC_W-1]}}, acc};
    wire signed [12:0]      d     = {x[11], x} - {x_prev[11], x_prev};
    wire signed [ACC_W+1:0] du    = {{(ACC_W+1-12-FRAC){d[12]}}, d, {FRAC{1'b0}}};
    wire [ACC_W+1:0]        half  = (shift == 4'd0) ? {(ACC_W+2){1'b0}}
                                                    : ({{(ACC_W+1){1'b0}}, 1'b1} << (shift - 4'd1));
    wire signed [ACC_W+1:0] acc_h = acc_e + $signed(half);
    wire signed [ACC_W+1:0] acc_s = acc_h >>> shift;
    wire signed [ACC_W+1:0] acc_n = acc_e - acc_s + du;

    // output: sat12(rnd(acc >>> FRAC))
    wire signed [ACC_W+1:0] y_h = acc_e + $signed({{(ACC_W+1-FRAC){1'b0}}, 1'b1, {(FRAC-1){1'b0}}});
    wire signed [ACC_W+1:0] y_s = y_h >>> FRAC;
    wire signed [11:0]      y_sat = (y_s > 2047)  ? 12'sd2047 :
                                    (y_s < -2048) ? -12'sd2048 : y_s[11:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            x_prev <= 12'sd0;
            x_d1   <= 12'sd0;
            acc    <= {ACC_W{1'b0}};
            y      <= 12'sd0;
        end else begin
            if (en) begin
                x_prev <= x;
                x_d1   <= x;
                if (shift != 4'd0)
                    acc <= acc_n[ACC_W-1:0];
            end
            if (v1)
                y <= (shift == 4'd0) ? x_d1 : y_sat;
        end
    end

`ifdef PD_SIM_CHECKS
    always @(posedge clk)
        if (rst_n && en && shift != 4'd0 &&
            (acc_n > $signed({1'b0, {(ACC_W-1){1'b1}}}) || acc_n < -$signed({1'b0, 1'b1, {(ACC_W-1){1'b0}}})))
            $display("ERROR pd_dc_rail: accumulator overflow");
`endif
endmodule


module pd_dc_blocker #(
    parameter integer FRAC  = 10,
    parameter integer ACC_W = 24
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [3:0]         shift,
    input  wire               in_valid,
    input  wire signed [11:0] in_i,
    input  wire signed [11:0] in_q,
    output reg                out_valid,
    output wire signed [11:0] out_i,
    output wire signed [11:0] out_q
);
    reg v1;
    always @(posedge clk) begin
        if (!rst_n) begin
            v1        <= 1'b0;
            out_valid <= 1'b0;
        end else begin
            v1        <= in_valid;
            out_valid <= v1;
        end
    end

    pd_dc_rail #(.FRAC(FRAC), .ACC_W(ACC_W)) u_i (
        .clk(clk), .rst_n(rst_n), .shift(shift), .en(in_valid), .x(in_i), .v1(v1), .y(out_i));
    pd_dc_rail #(.FRAC(FRAC), .ACC_W(ACC_W)) u_q (
        .clk(clk), .rst_n(rst_n), .shift(shift), .en(in_valid), .x(in_q), .v1(v1), .y(out_q));
endmodule

`default_nettype wire
