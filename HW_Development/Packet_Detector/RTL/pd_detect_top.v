// pd_detect_top.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Packet detector, detection lane (spec 4.1, 4.2, 4.6): optional front end (DC blocker + LPF)
// and the detection core. Bit-true to pd_reference.detect (debug=True), events up to t_trig.
// The CFO lane (accumulators, CORDIC, cascade) and the tone check follow in a later step.
//
//   input_stage 0: s_iq is the LPF output, the front end is bypassed
//   input_stage 1: s_iq is the decimated 12-bit stream; DC blocker (dc_shift) and LPF run here
//
// s_iq_tdata: one complex sample per beat, I = [11:0], Q = [27:16] (12-bit values sign-extended
// to 16 bits by the PC, as in pd_protocol). No back-pressure: s_iq_tready is always 1.
`timescale 1ns / 1ps
`default_nettype none

module pd_detect_top (
    input  wire        clk,
    input  wire        rst_n,
    // configuration (constant while samples flow; change only in reset)
    input  wire        input_stage,
    input  wire [3:0]  dc_shift,
    input  wire [15:0] eta_hi_sq,
    input  wire [15:0] eta_lo_sq,
    input  wire [15:0] qual_len,
    input  wire [31:0] blank_len,
    input  wire [31:0] arm_holdoff,
    // sample stream
    input  wire        s_iq_tvalid,
    input  wire [31:0] s_iq_tdata,
    output wire        s_iq_tready,
    // LPF output (detector input), for verification and the downstream chain
    output wire        lpf_valid,
    output wire [11:0] lpf_i,
    output wire [11:0] lpf_q,
    // trigger
    output wire        trig,
    output wire [31:0] t_trig,
    // debug stream, one record per sample: metric and flag bits
    output wire        dbg_valid,
    output wire [15:0] dbg_metric,
    output wire [15:0] dbg_bits
);
    assign s_iq_tready = 1'b1;
    wire signed [11:0] in_i = s_iq_tdata[11:0];
    wire signed [11:0] in_q = s_iq_tdata[27:16];

    // front end
    wire               dc_valid;
    wire signed [11:0] dc_i, dc_q;
    wire               fe_valid;
    wire signed [11:0] fe_i, fe_q;

    pd_dc_blocker u_dc (
        .clk(clk), .rst_n(rst_n), .shift(dc_shift),
        .in_valid(s_iq_tvalid & input_stage), .in_i(in_i), .in_q(in_q),
        .out_valid(dc_valid), .out_i(dc_i), .out_q(dc_q));

    pd_lpf u_lpf (
        .clk(clk), .rst_n(rst_n),
        .in_valid(dc_valid), .in_i(dc_i), .in_q(dc_q),
        .out_valid(fe_valid), .out_i(fe_i), .out_q(fe_q));

    assign lpf_valid = input_stage ? fe_valid : s_iq_tvalid;
    assign lpf_i     = input_stage ? fe_i     : in_i;
    assign lpf_q     = input_stage ? fe_q     : in_q;

    pd_detect_core u_core (
        .clk(clk), .rst_n(rst_n),
        .eta_hi_sq(eta_hi_sq), .eta_lo_sq(eta_lo_sq), .qual_len(qual_len),
        .blank_len(blank_len), .arm_holdoff(arm_holdoff),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .dbg_valid(dbg_valid), .dbg_metric(dbg_metric), .dbg_bits(dbg_bits));
endmodule

`default_nettype wire
