// pl_lutram1.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// 128 x 1 bit LUT RAM: one write port (clocked), one asynchronous read port (RAM128X1D).
`timescale 1ns / 1ps
`default_nettype none

module pl_lutram1 (
    input  wire       clk,
    input  wire       we,
    input  wire [6:0] waddr,
    input  wire       wd,
    input  wire [6:0] raddr,
    output wire       rd
);
    (* ram_style = "distributed" *) reg mem [0:127];
    always @(posedge clk) if (we) mem[waddr] <= wd;
    assign rd = mem[raddr];
endmodule

`default_nettype wire
