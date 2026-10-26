// pd_div16.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Pipelined unsigned divider for the debug metric:
//   quot = (den == 0) ? 0 : min(65535, floor(num / den))
// One result per clock, latency 17 clocks; side_in travels with the data.
// Stage 0 checks den == 0 and num >= den * 2^16 (saturation); stages 1..16 are a restoring
// division producing quotient bits 15..0.
`timescale 1ns / 1ps
`default_nettype none

module pd_div16 #(
    parameter integer NW = 52,                // numerator width
    parameter integer DW = 36,                // denominator width
    parameter integer SW = 16                 // side-band width
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          in_valid,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    input  wire [SW-1:0] side_in,
    output wire          out_valid,
    output wire [15:0]   quot,
    output wire [SW-1:0] side_out
);
    localparam integer RW = NW + 1;           // remainder / compare width

    reg [RW-1:0] rem  [0:16];
    reg [DW-1:0] dv   [0:16];
    reg [15:0]   q    [0:16];
    reg          ovr  [0:16];                 // result overridden (0 or 65535)
    reg [15:0]   ovq  [0:16];
    reg [SW-1:0] side [0:16];
    reg [16:0]   v;

    wire [RW-1:0] den_sat = {{(RW-DW-16){1'b0}}, den, 16'd0};
    integer s;

    always @(posedge clk) begin
        if (!rst_n) v <= 17'd0;
        else        v <= {v[15:0], in_valid};
    end

    always @(posedge clk) begin
        // stage 0
        rem[0]  <= {1'b0, num};
        dv[0]   <= den;
        q[0]    <= 16'd0;
        side[0] <= side_in;
        if (den == {DW{1'b0}}) begin
            ovr[0] <= 1'b1; ovq[0] <= 16'd0;
        end else if ({1'b0, num} >= den_sat) begin
            ovr[0] <= 1'b1; ovq[0] <= 16'hFFFF;
        end else begin
            ovr[0] <= 1'b0; ovq[0] <= 16'd0;
        end
        // stages 1..16: quotient bit 16 - s
        for (s = 1; s <= 16; s = s + 1) begin
            dv[s]   <= dv[s-1];
            ovr[s]  <= ovr[s-1];
            ovq[s]  <= ovq[s-1];
            side[s] <= side[s-1];
            if (rem[s-1] >= ({{(RW-DW){1'b0}}, dv[s-1]} << (16 - s))) begin
                rem[s] <= rem[s-1] - ({{(RW-DW){1'b0}}, dv[s-1]} << (16 - s));
                q[s]   <= q[s-1] | (16'd1 << (16 - s));
            end else begin
                rem[s] <= rem[s-1];
                q[s]   <= q[s-1];
            end
        end
    end

    assign out_valid = v[16];
    assign quot      = ovr[16] ? ovq[16] : q[16];
    assign side_out  = side[16];
endmodule

`default_nettype wire
