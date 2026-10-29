// cf_rx_ooc.v  (Flexlink_py/HW_Development/Packet_Detector/HW)
//
// Synthesis-only wrapper for synth_cf.tcl: cf_rx with its functional ports only (the dbg_*
// testbench outputs left open, so synthesis removes them as it will in the board design).
`timescale 1ns / 1ps
`default_nettype none

module cf_rx_ooc (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [23:0]        cfo_inc,
    input  wire [31:0]        t_trig,
    input  wire [31:0]        n_end,
    output wire               busy,
    input  wire               s_valid,
    output wire               s_ready,
    input  wire signed [11:0] s_i,
    input  wire signed [11:0] s_q,
    output wire               rec_valid,
    output wire               rec_crc_ok,
    output wire [53:0]        rec_msg,
    output wire [5:0]         rec_shift,
    output wire signed [15:0] rec_metric,
    output wire [31:0]        rec_t_trig,
    output wire [31:0]        rec_n_end
);
    cf_rx u (.clk(clk), .rst_n(rst_n), .start(start), .cfo_inc(cfo_inc), .t_trig(t_trig), .n_end(n_end),
             .busy(busy), .s_valid(s_valid), .s_ready(s_ready), .s_i(s_i), .s_q(s_q),
             .rec_valid(rec_valid), .rec_crc_ok(rec_crc_ok), .rec_msg(rec_msg), .rec_shift(rec_shift),
             .rec_metric(rec_metric), .rec_t_trig(rec_t_trig), .rec_n_end(rec_n_end),
             .dbg_grid_we(), .dbg_grid_addr(), .dbg_grid_data(), .dbg_phi(), .dbg_hf_we(), .dbg_hf_k(),
             .dbg_hf(), .dbg_v_we(), .dbg_v_i(), .dbg_v(), .dbg_bits());
endmodule

`default_nettype wire
