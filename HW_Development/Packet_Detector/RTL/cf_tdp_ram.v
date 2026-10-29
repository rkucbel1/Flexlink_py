// cf_tdp_ram.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// True dual-port RAM, two independent read/write ports, registered reads (one clock of
// latency, read-first), the template Vivado maps to block RAM. Used by the FFT banks.
`timescale 1ns / 1ps
`default_nettype none

module cf_tdp_ram #(
    parameter integer W = 36,
    parameter integer A = 10
) (
    input  wire         clk,
    input  wire         we_a,
    input  wire [A-1:0] addr_a,
    input  wire [W-1:0] wd_a,
    output reg  [W-1:0] rd_a,
    input  wire         we_b,
    input  wire [A-1:0] addr_b,
    input  wire [W-1:0] wd_b,
    output reg  [W-1:0] rd_b
);
    (* ram_style = "block" *) reg [W-1:0] mem [0:(1 << A) - 1];
    always @(posedge clk) begin
        if (we_a) mem[addr_a] <= wd_a;
        rd_a <= mem[addr_a];
    end
    always @(posedge clk) begin
        if (we_b) mem[addr_b] <= wd_b;
        rd_b <= mem[addr_b];
    end
endmodule

`default_nettype wire
