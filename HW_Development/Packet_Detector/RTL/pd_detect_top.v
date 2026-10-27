// pd_detect_top.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Packet detector (spec 4.1-4.3, 4.6): optional front end (DC blocker + LPF), detection core
// (t_trig, debug stream), CFO lane (lag accumulators, CORDIC, cascade -> cfo_inc) and
// tone-structure check (13 DFT bins -> num, den, fail). Bit-true to pd_reference.detect
// (debug=True).
//
//   input_stage 0: s_iq is the LPF output, the front end is bypassed
//   input_stage 1: s_iq is the decimated 12-bit stream; DC blocker (dc_shift) and LPF run here
//
// s_iq_tdata: one complex sample per beat, I = [11:0], Q = [27:16] (12-bit values sign-extended
// to 16 bits by the PC, as in pd_protocol). s_iq_tready: AXI-stream back-pressure. It stays 1 at
// the real sample rate (one sample per 5 clocks or slower); faster input (DMA from memory) is
// held back while the tone check catches up. Results don't depend on the stalls.
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
    input  wire [15:0] n_cfo,          // CFO window, 128..4096 (2048 default)
    input  wire [23:0] tone_lin,       // tone-check threshold, linear Q.16 (13 dB: 1307615); 0 = off
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
    output wire [15:0] dbg_bits,
    // CFO result, one per trigger, about N_CFO + 32 samples + 80 clocks after t_trig
    output wire        cfo_valid,
    output wire [31:0] cfo_t,
    output wire [31:0] cfo_inc,         // NCO increment for -f_hat, LSB Fs / 2^24 (sign-extended)
    output wire [15:0] ang16,
    output wire [15:0] ang32,
    output wire [15:0] ang128,
    output wire [15:0] ang256,
    // the four lag accumulators of that window (verification / debug)
    output wire [36:0] a16_re, a16_im, a32_re, a32_im, a128_re, a128_im, a256_re, a256_im,
    // event, one per window, after the tone check (about 4 N + 40 clocks after cfo_valid)
    output wire        ev_valid,
    output wire [31:0] ev_t,
    output wire [31:0] ev_cfo_inc,
    output wire [15:0] ev_ang16,
    output wire [15:0] ev_ang32,
    output wire [15:0] ev_ang128,
    output wire [15:0] ev_ang256,
    output wire [39:0] ev_tone_num,    // 9 (sum P_tone - max P_tone)
    output wire [39:0] ev_tone_den,    // sum P_ref; ToneCheck = 10 log10(num / den)
    output wire        ev_tone_fail
);
    wire stall;
    assign s_iq_tready = !stall;
    wire   s_fire = s_iq_tvalid & !stall;
    wire signed [11:0] in_i = s_iq_tdata[11:0];
    wire signed [11:0] in_q = s_iq_tdata[27:16];

    // front end
    wire               dc_valid;
    wire signed [11:0] dc_i, dc_q;
    wire               fe_valid;
    wire signed [11:0] fe_i, fe_q;

    pd_dc_blocker u_dc (
        .clk(clk), .rst_n(rst_n), .shift(dc_shift),
        .in_valid(s_fire & input_stage), .in_i(in_i), .in_q(in_q),
        .out_valid(dc_valid), .out_i(dc_i), .out_q(dc_q));

    pd_lpf u_lpf (
        .clk(clk), .rst_n(rst_n),
        .in_valid(dc_valid), .in_i(dc_i), .in_q(dc_q),
        .out_valid(fe_valid), .out_i(fe_i), .out_q(fe_q));

    assign lpf_valid = input_stage ? fe_valid : s_fire;
    assign lpf_i     = input_stage ? fe_i     : in_i;
    assign lpf_q     = input_stage ? fe_q     : in_q;

    pd_detect_core u_core (
        .clk(clk), .rst_n(rst_n),
        .eta_hi_sq(eta_hi_sq), .eta_lo_sq(eta_lo_sq), .qual_len(qual_len),
        .blank_len(blank_len), .arm_holdoff(arm_holdoff), .n_cfo(n_cfo),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .dbg_valid(dbg_valid), .dbg_metric(dbg_metric), .dbg_bits(dbg_bits));

    pd_cfo_lane u_cfo (
        .clk(clk), .rst_n(rst_n), .n_cfo(n_cfo),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .cfo_valid(cfo_valid), .cfo_t(cfo_t), .cfo_inc(cfo_inc),
        .ang16(ang16), .ang32(ang32), .ang128(ang128), .ang256(ang256),
        .a16_re(a16_re), .a16_im(a16_im), .a32_re(a32_re), .a32_im(a32_im),
        .a128_re(a128_re), .a128_im(a128_im), .a256_re(a256_re), .a256_im(a256_im));

    pd_tone_check u_tone (
        .clk(clk), .rst_n(rst_n), .n_cfo(n_cfo), .tone_lin(tone_lin),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .cfo_valid(cfo_valid), .cfo_t(cfo_t), .cfo_inc(cfo_inc),
        .ang16(ang16), .ang32(ang32), .ang128(ang128), .ang256(ang256),
        .stall(stall),
        .ev_valid(ev_valid), .ev_t(ev_t), .ev_cfo_inc(ev_cfo_inc),
        .ev_ang16(ev_ang16), .ev_ang32(ev_ang32), .ev_ang128(ev_ang128), .ev_ang256(ev_ang256),
        .ev_num(ev_tone_num), .ev_den(ev_tone_den), .ev_fail(ev_tone_fail));
endmodule

`default_nettype wire
