// pd_detect_top.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Packet detector (spec 4.1-4.3, 4.6): optional front end (DC blocker + LPF), detection core
// (t_trig, debug stream), CFO lane (lag accumulators, CORDIC, cascade -> cfo_inc),
// tone-structure check (13 DFT bins -> num, den, fail) and Preamble B timing (spec 4.5, 1-bit
// tap correlator on the DC blocker output -> n_end; input_stage 1 and 2 only) and the control
// field after each good timing record (pd_cf -> cf_rx: CRC, 54 message bits). Bit-true to
// pd_reference.detect (debug=True).
//
//   input_stage 0: s_iq is the LPF output, the front end is bypassed
//   input_stage 1: s_iq is the decimated 12-bit stream (20 Msps); DC blocker (dc_shift) and
//                  LPF run here
//   input_stage 2: s_iq is the 12-bit radio stream at 40 Msps; half-band decimator, DC blocker
//                  and LPF run here. All sample indices (t_trig, ...) count 20 Msps samples.
//
// s_iq_tdata: one complex sample per beat, I = [11:0], Q = [27:16] (12-bit values sign-extended
// to 16 bits by the PC, as in pd_protocol). s_iq_tready: AXI-stream back-pressure. It stays 1 at
// the real sample rate (one sample per 5 clocks or slower); faster input (DMA from memory) is
// held back while the tone check catches up, and to one sample per 5 clocks while a timing
// search runs. Results don't depend on the stalls.
`timescale 1ns / 1ps
`default_nettype none

module pd_detect_top (
    input  wire        clk,
    input  wire        rst_n,
    // configuration (constant while samples flow; change only in reset)
    input  wire [1:0]  input_stage,    // 0 LPF output in, 1 20 Msps in, 2 40 Msps in
    input  wire [3:0]  dc_shift,
    input  wire [15:0] eta_hi_sq,
    input  wire [15:0] eta_lo_sq,
    input  wire [15:0] qual_len,
    input  wire [31:0] blank_len,
    input  wire [31:0] arm_holdoff,
    input  wire [15:0] n_cfo,          // CFO window, 128..4096 (2048 default)
    input  wire [23:0] tone_lin,       // tone-check threshold, linear Q.16 (13 dB: 1307615); 0 = off
    input  wire [15:0] t_pk,           // timing: minimum peak / mean, Q8.8 (18: 4608); 0 = no check
    input  wire        rearm,          // 1: the Rx state machine re-arms (pd_arm), 0: fixed holdoff
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
    output wire        ev_tone_fail,
    // Preamble B timing, one record per event that passed the tone check (BUSY records may
    // come before the result of an earlier event)
    output wire        tm_valid,
    output wire [31:0] tm_t,
    output wire [31:0] tm_n_end,
    output wire [2:0]  tm_status,      // bit0 PEAK_OK, bit1 LATE, bit2 BUSY
    output wire [15:0] tm_n_pos,
    output wire [33:0] tm_p_max,
    output wire [47:0] tm_p_sum,
    output wire        tm_err,
    // control field, one record per good timing record (cf_rx, about 0.45 ms after it)
    output wire        cf_valid,
    output wire [31:0] cf_t,
    output wire [31:0] cf_n_end,
    output wire        cf_crc_ok,
    output wire [53:0] cf_msg,
    output wire [5:0]  cf_shift,
    output wire [15:0] cf_metric,
    output wire        cf_err,
    // speculative payload FFT (rearm only): records kept after the control field's decision
    output wire [4:0]   pl_cnt,
    output wire [127:0] pl_head,       // {sig_b, sig_a, 25'b0, sym, t_trig}
    input  wire         pl_pop,
    // payload channel estimate (rearm only): packet and per-symbol H records
    output wire [4:0]   ch_cnt,
    output wire [127:0] ch_head,       // see pd_cf
    input  wire         ch_pop,
    output wire         ch_err,        // pl_chest packet queue overflow (sticky; cannot happen)
    // decoder-input records (rearm only): one per code word
    output wire [4:0]   lq_cnt,
    output wire [127:0] lq_head,       // see pd_cf
    input  wire         lq_pop,
    // transport blocks (rearm only): FIFO words of complete records (see pl_dec)
    output wire [11:0]  tb_cnt,
    output wire [31:0]  tb_word,
    input  wire         tb_pop,
    output wire        arm_late,       // a re-arm came too late (sticky; cannot happen)
    // state: busy = something still in flight; open_n / open_t = windows still waiting for
    // samples (at most 1 once busy is 0) and the first one's t_trig
    output wire        busy,
    output wire [3:0]  open_n,
    output wire [31:0] open_t
);
    wire stall_tone, stall_tm, stall_cf, lane_busy, tone_busy, tm_busy, cf_busy;
    wire stall_arm;
    wire stall = stall_tone || stall_tm || stall_cf || stall_arm;
    wire        arm_set;
    wire [31:0] arm_val;
    wire [23:0] tm_inc;
    assign s_iq_tready = !stall;
    wire   s_fire = s_iq_tvalid & !stall;
    wire signed [11:0] in_i = s_iq_tdata[11:0];
    wire signed [11:0] in_q = s_iq_tdata[27:16];

    // front end: [half-band] -> DC blocker -> LPF
    wire               hb_valid;
    wire signed [11:0] hb_i, hb_q;
    wire               stage2 = (input_stage == 2'd2);
    wire               stage1 = (input_stage == 2'd1);
    pd_halfband u_hb (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s_fire & stage2), .in_i(in_i), .in_q(in_q),
        .out_valid(hb_valid), .out_i(hb_i), .out_q(hb_q));

    wire               dc_valid;
    wire signed [11:0] dc_i, dc_q;
    wire               fe_valid;
    wire signed [11:0] fe_i, fe_q;

    pd_dc_blocker u_dc (
        .clk(clk), .rst_n(rst_n), .shift(dc_shift),
        .in_valid(stage2 ? hb_valid : (s_fire & stage1)),
        .in_i(stage2 ? hb_i : in_i), .in_q(stage2 ? hb_q : in_q),
        .out_valid(dc_valid), .out_i(dc_i), .out_q(dc_q));

    pd_lpf u_lpf (
        .clk(clk), .rst_n(rst_n),
        .in_valid(dc_valid), .in_i(dc_i), .in_q(dc_q),
        .out_valid(fe_valid), .out_i(fe_i), .out_q(fe_q));

    wire   fe_on     = stage1 || stage2;
    assign lpf_valid = fe_on ? fe_valid : s_fire;
    assign lpf_i     = fe_on ? fe_i     : in_i;
    assign lpf_q     = fe_on ? fe_q     : in_q;

    pd_detect_core u_core (
        .clk(clk), .rst_n(rst_n),
        .eta_hi_sq(eta_hi_sq), .eta_lo_sq(eta_lo_sq), .qual_len(qual_len),
        .blank_len(blank_len), .arm_holdoff(arm_holdoff), .n_cfo(n_cfo),
        .rearm(rearm), .arm_set(arm_set), .arm_val(arm_val), .arm_late(arm_late),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .dbg_valid(dbg_valid), .dbg_metric(dbg_metric), .dbg_bits(dbg_bits));

    pd_cfo_lane u_cfo (
        .clk(clk), .rst_n(rst_n), .n_cfo(n_cfo),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q), .rel(dbg_valid),
        .trig(trig), .t_trig(t_trig),
        .cfo_valid(cfo_valid), .cfo_t(cfo_t), .cfo_inc(cfo_inc),
        .ang16(ang16), .ang32(ang32), .ang128(ang128), .ang256(ang256),
        .a16_re(a16_re), .a16_im(a16_im), .a32_re(a32_re), .a32_im(a32_im),
        .a128_re(a128_re), .a128_im(a128_im), .a256_re(a256_re), .a256_im(a256_im),
        .busy(lane_busy));

    pd_tone_check u_tone (
        .clk(clk), .rst_n(rst_n), .n_cfo(n_cfo), .tone_lin(tone_lin),
        .in_valid(lpf_valid), .in_i(lpf_i), .in_q(lpf_q),
        .trig(trig), .t_trig(t_trig),
        .cfo_valid(cfo_valid), .cfo_t(cfo_t), .cfo_inc(cfo_inc),
        .ang16(ang16), .ang32(ang32), .ang128(ang128), .ang256(ang256),
        .stall(stall_tone),
        .ev_valid(ev_valid), .ev_t(ev_t), .ev_cfo_inc(ev_cfo_inc),
        .ev_ang16(ev_ang16), .ev_ang32(ev_ang32), .ev_ang128(ev_ang128), .ev_ang256(ev_ang256),
        .ev_num(ev_tone_num), .ev_den(ev_tone_den), .ev_fail(ev_tone_fail),
        .busy(tone_busy), .open_n(open_n), .open_t(open_t));

    pd_timing u_tm (
        .clk(clk), .rst_n(rst_n), .en(fe_on), .n_cfo(n_cfo), .t_pk(t_pk),
        .y_valid(dc_valid), .y_i(dc_i), .y_q(dc_q),
        .trig(trig), .t_trig(t_trig),
        .ev_valid(ev_valid), .ev_t(ev_t), .ev_cfo_inc(ev_cfo_inc), .ev_fail(ev_tone_fail),
        .stall(stall_tm),
        .tm_valid(tm_valid), .tm_t(tm_t), .tm_n_end(tm_n_end), .tm_status(tm_status),
        .tm_n_pos(tm_n_pos), .tm_p_max(tm_p_max), .tm_p_sum(tm_p_sum), .tm_inc(tm_inc),
        .busy(tm_busy), .err(tm_err));

    pd_cf u_cf (
        .clk(clk), .rst_n(rst_n), .en(fe_on), .pl_en(rearm),
        .y_valid(dc_valid), .y_i(dc_i), .y_q(dc_q),
        .tm_valid(tm_valid), .tm_t(tm_t), .tm_n_end(tm_n_end), .tm_status(tm_status), .tm_inc(tm_inc),
        .stall(stall_cf),
        .cf_valid(cf_valid), .cf_t(cf_t), .cf_n_end(cf_n_end), .cf_crc_ok(cf_crc_ok), .cf_msg(cf_msg),
        .cf_shift(cf_shift), .cf_metric(cf_metric), .busy(cf_busy), .err(cf_err),
        .pl_cnt(pl_cnt), .pl_head(pl_head), .pl_pop(pl_pop),
        .ch_cnt(ch_cnt), .ch_head(ch_head), .ch_pop(ch_pop), .ch_err(ch_err),
        .lq_cnt(lq_cnt), .lq_head(lq_head), .lq_pop(lq_pop),
        .tb_cnt(tb_cnt), .tb_word(tb_word), .tb_pop(tb_pop));

    pd_arm u_arm (
        .clk(clk), .rst_n(rst_n), .rearm(rearm), .fe_on(fe_on), .n_cfo(n_cfo), .arm_holdoff(arm_holdoff),
        .x_valid(lpf_valid), .trig(trig), .t_trig(t_trig),
        .ev_valid(ev_valid), .ev_t(ev_t), .ev_fail(ev_tone_fail),
        .tm_valid(tm_valid), .tm_t(tm_t), .tm_n_end(tm_n_end), .tm_status(tm_status),
        .cf_valid(cf_valid), .cf_t(cf_t), .cf_crc_ok(cf_crc_ok), .cf_msg(cf_msg),
        .arm_set(arm_set), .arm_val(arm_val), .stall(stall_arm), .pending());

    // samples inside the front end and core: debug records due minus records out. With the
    // half-band, a record is due with every second input (the first of a pair waits for its
    // partner and is not counted as in flight).
    reg       ph;
    reg [7:0] inflight;
    wire      rec_due = s_fire && (!stage2 || ph);
    always @(posedge clk) begin
        if (!rst_n) begin
            ph <= 1'b0; inflight <= 8'd0;
        end else begin
            if (s_fire && stage2) ph <= !ph;
            inflight <= inflight + {7'd0, rec_due} - {7'd0, dbg_valid};
        end
    end
    assign busy = (inflight != 8'd0) || rec_due || lane_busy || tone_busy || tm_busy || cf_busy;
endmodule

`default_nettype wire
