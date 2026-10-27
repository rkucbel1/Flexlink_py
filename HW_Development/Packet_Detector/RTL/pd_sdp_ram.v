// pd_sdp_ram.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Simple dual-port RAM, one write port, one read port with one clock of latency (registered
// read, so it maps to block RAM). Instantiate it with (* keep_hierarchy = "yes" *): otherwise
// Vivado may pull the read register into the DSPs that use the data, and a RAM without a read
// register can only become LUT RAM.
`timescale 1ns / 1ps
`default_nettype none

module pd_sdp_ram #(
    parameter integer W = 24,          // data bits
    parameter integer A = 13           // address bits (2^A words)
) (
    input  wire         clk,
    input  wire         we,
    input  wire [A-1:0] waddr,
    input  wire [W-1:0] wdata,
    input  wire [A-1:0] raddr,
    output reg  [W-1:0] rdata
);
    (* ram_style = "block" *) reg [W-1:0] mem [0:(1 << A) - 1];
    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule

`default_nettype wire
