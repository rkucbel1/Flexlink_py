// pd_axi.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// AXI wrapper of the packet detector for the Zedboard block design (replaces the phase-1
// loopback FIFO "pl_core"):
//   S_AXIS  samples from the AXI DMA MM2S (one complex sample per beat, I [15:0], Q [31:16];
//           with CTRL.PACK8 two 8-bit samples per beat, {Q1, I1, Q0, I0}, unpacked to that
//           format by pd_unpack8.v)
//   M_AXIS  debug records to the AXI DMA S2MM, one per detector sample (20 Msps; with input
//           stage 2 one per input pair): {dbg_bits, dbg_metric}; TLAST on the record of each
//           input beat that had TLAST, so every S2MM transfer ends with its MM2S transfer
//           (stage 2: transfers must hold whole pairs, i.e. an even number of samples)
//   S_AXI   AXI4-Lite registers: control, status, parameters, event, timing and control FIFOs
//
// Register map (byte offsets; RW = read/write, RO = read only):
//   0x00 ID          RO  0x50440902 ("PD", version 9.2: CTRL.NOREC; 9.1: CTRL.PACK8, the int8 input unpacker;
//                        9.0: LDPC decoder, CRC24 and the transport-
//                        block FIFO; 8.0 had no TB FIFO; 8.0 added the payload demapper and
//                        combiner, decoder-input records; 7.0 had no LQ records; 7.0 added the payload channel
//                        estimate and its H records; 6.0 had no CH records, its control-field check accepted
//                        grids without pilots and NoOfTxAntenna 1; 6.0 added the shared fast
//                        FFT with the speculative payload FFT; 5.0 had no payload records, 4.0 no REARM, 3.0 no control
//                        field, 2.0 no timing, 1.0 no input stage 2)
//   0x04 CTRL        RW  bit0 RESET (1 = detector held in reset; 1 after power-up)
//                        bit1 DEBUG (send debug records on M_AXIS; change only when not BUSY)
//                        bit2 LOOPBACK (S_AXIS -> M_AXIS unchanged, detector not fed)
//                        bit3 PACK8 (9.1: S_AXIS carries two int8 samples per word, unpacked to
//                        one 12-bit sample per beat (x16) before the detector and the loopback;
//                        change only between transfers)
//                        bit4 NOREC (9.2: payload, channel-estimate and decoder-input records
//                        are removed from their FIFOs in the PL as they arrive, so the firmware
//                        need not read them; set it only with those FIFOs empty, after RESET)
//   0x08 STATUS      RO  bit0 BUSY (samples, CFO or tone results, events or debug records in
//                        flight), bit1 OPEN (a window still waits for samples), bits 7:4 number
//                        of open windows, bit8 EV_OVF (event lost; sticky until RESET),
//                        bit9 in reset, bit10 loopback, bit11 TM_OVF (timing record lost),
//                        bit12 TM_ERR (timing start queue overflow); both sticky, cannot happen,
//                        bit13 CF_OVF (control record lost), bit14 CF_ERR (control-field start
//                        queue overflow); both sticky, cannot happen, bit15 ARM_LATE (a re-arm
//                        decision came after its sample; sticky, cannot happen), bit16 CH_ERR
//                        (channel-estimate packet queue overflow; sticky, cannot happen)
//   0x0C SAMPLES     RO  input samples taken since RESET (stage 2: 40 Msps samples, twice the
//                        detector's sample_cnt)
//   0x10 OPEN_T      RO  t_trig of the first open window (valid when OPEN)
//   0x14 EV_LEVEL    RO  words in the event FIFO (8 per event)
//   0x18 EV_DATA     RO  next event word (reading removes it)
//   0x1C DBG_SENT    RO  debug records sent since RESET
//   0x20 ETA_HI_SQ   RW  \
//   0x24 ETA_LO_SQ   RW   |
//   0x28 QUAL_LEN    RW   |
//   0x2C N_CFO       RW   |  detector parameters (pd_protocol.Params); write them while RESET
//   0x30 BLANK_LEN   RW   |  is 1. Power-up values are the Params defaults.
//   0x34 ARM_HOLDOFF RW   |
//   0x38 INPUT_STAGE RW   |  0 LPF output, 1 20 Msps, 2 40 Msps (half-band in the PL)
//   0x3C DC_SHIFT    RW   |
//   0x40 TONE_LIN    RW  /   threshold as a linear ratio, Q.16 (firmware: tone_thresh_lin)
//   0x44 T_PK        RW      timing: minimum peak / mean, Q8.8 (default 18.0 = 4608), 0 = off
//   0x48 TM_LEVEL    RO  words in the timing FIFO (8 per record)
//   0x4C TM_DATA     RO  next timing word (reading removes it)
//   0x50 CF_LEVEL    RO  words in the control-record FIFO (8 per record)
//   0x54 CF_DATA     RO  next control-record word (reading removes it)
//   0x58 REARM       RW  1 = re-arm by the Rx state machine (default), 0 = fixed ARM_HOLDOFF;
//                        1 also enables the speculative payload FFT and its records
//   0x5C PL_LEVEL    RO  words in the payload-record FIFO (4 per record, 16 records)
//   0x60 PL_DATA     RO  next payload-record word (reading removes it)
//   0x64 CH_LEVEL    RO  words in the channel-estimate record FIFO (4 per record, 16 records)
//   0x68 CH_DATA     RO  next channel-estimate word (reading removes it)
//   0x6C LQ_LEVEL    RO  words in the decoder-input record FIFO (4 per record, 16 records)
//   0x70 LQ_DATA     RO  next decoder-input word (reading removes it)
//   0x74 TB_LEVEL    RO  words of complete transport-block records in their FIFO (2048 words)
//   0x78 TB_DATA     RO  next transport-block word (reading removes it; reads at least 3 clocks
//                        apart, as AXI-Lite reads always are)
//
// Event (8 words, in this order): t_trig, cfo_inc, {ang32, ang16}, {ang256, ang128},
//   num[31:0], {fail, 23'b0, num[39:32]}, den[31:0], {24'b0, den[39:32]}
//
// Timing record (8 words): t_trig, n_end, status (bit0 PEAK_OK, bit1 LATE, bit2 BUSY), n_pos,
//   p_max[31:0], p_max[33:32], p_sum[31:0], p_sum[47:32]
//
// Control record (8 words): t_trig, n_end, {metric[15:0], 2'b0, shift[5:0], 7'b0, crc_ok},
//   message[31:0], {10'b0, message[53:32]}, 0, 0, 0
//
// Payload record (4 words): t_trig, {25'b0, sym}, sig_a, sig_b (cf_reference.grid_signature
//   of the payload symbol's 841 FFT bins). A full FIFO holds the speculative FFT (not the input
//   directly), so the firmware may read them at any pace.
//
// Channel-estimate record (4 words; pl_reference.chest_records), in order per packet:
//   packet record   t_trig, 0x8000, {n0[15:0], 3'b0, hi, class[3:0], k[7:0]}, sum_N[31:0]
//   symbol record   t_trig, {25'b0, sym}, sig_a, sig_b (grid_signature of the 841 H values)
// A full FIFO holds the channel estimate (and through it the payload FFT), never the input.
//
// Decoder-input record (4 words; pl_reference.lq_records), one per code word of an accepted
//   packet: t_trig, w (code word), sig_a, sig_b (grid_signature of the N decoder inputs Lq in
//   code order). A full FIFO holds the combiner (and through it the channel estimate).
//
// Transport-block record (2 + ceil(nbits / 32) words; pl_reference.tb_records), one per code
//   word: t_trig, {crc_ok, 3'b0, iterations[3:0], w[9:0], 3'b0, nbits[10:0]}, then the bits
//   (bit j in word j / 32, bit j mod 32). A code word is decoded only with room for its record.
//
// Flow control: the sample input is held back (S_AXIS tready = 0) while the tone check is
// catching up, while a timing search runs (one sample per 5 clocks), while the event or the
// timing FIFO is nearly full (the firmware reads them during the DMA), and
// with DEBUG while the debug FIFO and the records in flight would exceed its 64 entries. Nothing
// is ever dropped, and none of this changes the results.
`timescale 1ns / 1ps
`default_nettype none

module pd_axi (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI:S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    // AXI4-Lite slave
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12" *)
    input  wire [11:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
    input  wire [2:0]  s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire        s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire        s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output wire [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output reg         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [11:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
    input  wire [2:0]  s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire        s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output reg  [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire        s_axi_rready,

    // samples in
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "TDATA_NUM_BYTES 4, HAS_TLAST 1, HAS_TKEEP 1" *)
    input  wire [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *)
    input  wire [3:0]  s_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,

    // debug records (or samples, in loopback) out
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "TDATA_NUM_BYTES 4, HAS_TLAST 1, HAS_TKEEP 1" *)
    output wire [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *)
    output wire [3:0]  m_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire        m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready
);
    localparam [31:0] ID = 32'h5044_0902;     // "PD", version 9.2 (9.2: NOREC, 9.1: PACK8 unpacker), 9.0 (9.0: LDPC decoder, 8.0: demapper / combiner, 7.0: channel estimate, 6.0: payload FFT, 5.0: re-arm, 4.0: control field)
    localparam integer CF_DEPTH = 8;             // control records
    localparam integer CF_LIMIT = CF_DEPTH - 2;  // hold the input above this
    localparam integer TM_DEPTH = 32;            // timing records
    localparam integer TM_LIMIT = TM_DEPTH - 8;  // hold the input above this
    localparam integer EV_DEPTH = 256;           // words (32 events)
    localparam integer EV_LIMIT = EV_DEPTH - 96; // hold the input above this (>= 11 events room)
    localparam integer DBG_BITS = 6;             // debug FIFO 64 records
    localparam integer DBG_LIMIT = (1 << DBG_BITS) - 4;

    // ------------------------------------------------------------------ registers
    reg        ctrl_reset, ctrl_debug, ctrl_loop, ctrl_pack, ctrl_norec;
    reg [15:0] r_eta_hi, r_eta_lo, r_qual, r_ncfo;
    reg [31:0] r_blank, r_hold;
    reg [1:0]  r_stage;
    reg [3:0]  r_shift;
    reg [23:0] r_lin;
    reg [15:0] r_tpk;
    reg        r_rearm;

    // ------------------------------------------------------------------ input unpacker
    // CTRL.PACK8 (9.1): S_AXIS words of two int8 samples -> one 12-bit sample per beat. With
    // PACK8 = 0 a wire. Everything below uses the u_* stream.
    wire [31:0] u_tdata;
    wire        u_tvalid, u_tlast, u_tready;
    pd_unpack8 u_unpack (
        .clk(aclk), .rst_n(aresetn), .en(ctrl_pack),
        .s_tdata(s_axis_tdata), .s_tvalid(s_axis_tvalid), .s_tlast(s_axis_tlast), .s_tready(s_axis_tready),
        .m_tdata(u_tdata), .m_tvalid(u_tvalid), .m_tlast(u_tlast), .m_tready(u_tready));

    // detector reset: registered, from the bus reset and CTRL.RESET
    reg det_rst_n;
    always @(posedge aclk) det_rst_n <= aresetn && !ctrl_reset;

    // ------------------------------------------------------------------ detector
    wire        det_tready, det_dbg_valid, det_ev_valid, det_ev_fail, det_busy;
    wire [15:0] det_dbg_metric, det_dbg_bits;
    wire [31:0] det_ev_t, det_ev_inc;
    wire [15:0] det_ev_a16, det_ev_a32, det_ev_a128, det_ev_a256;
    wire [39:0] det_ev_num, det_ev_den;
    wire [3:0]  det_open_n;
    wire [31:0] det_open_t;
    wire        det_tm_valid, det_tm_err;
    wire [31:0] det_tm_t, det_tm_n_end;
    wire [2:0]  det_tm_status;
    wire [15:0] det_tm_n_pos;
    wire [33:0] det_tm_p_max;
    wire [47:0] det_tm_p_sum;
    wire        det_cf_valid, det_cf_crc_ok, det_cf_err, det_arm_late;
    wire [31:0] det_cf_t, det_cf_n_end;
    wire [53:0] det_cf_msg;
    wire [5:0]  det_cf_shift;
    wire [15:0] det_cf_metric;
    wire [4:0]   det_pl_cnt;
    wire [127:0] det_pl_head;
    wire         pl_last;
    wire [4:0]   det_ch_cnt;
    wire [127:0] det_ch_head;
    wire         ch_last, det_ch_err;
    wire [4:0]   det_lq_cnt;
    wire [127:0] det_lq_head;
    wire         lq_last;
    wire [11:0]  det_tb_cnt;
    wire [31:0]  det_tb_word;
    reg          tb_rd_pulse;
    reg         gate;                             // input allowed (registered)
    wire        det_tvalid = u_tvalid && gate;
    wire        accept     = det_tvalid && det_tready;

    pd_detect_top u_det (
        .clk(aclk), .rst_n(det_rst_n),
        .input_stage(r_stage), .dc_shift(r_shift),
        .eta_hi_sq(r_eta_hi), .eta_lo_sq(r_eta_lo), .qual_len(r_qual),
        .blank_len(r_blank), .arm_holdoff(r_hold), .n_cfo(r_ncfo), .tone_lin(r_lin), .t_pk(r_tpk),
        .rearm(r_rearm),
        .s_iq_tvalid(det_tvalid), .s_iq_tdata(u_tdata), .s_iq_tready(det_tready),
        .lpf_valid(), .lpf_i(), .lpf_q(), .trig(), .t_trig(),
        .dbg_valid(det_dbg_valid), .dbg_metric(det_dbg_metric), .dbg_bits(det_dbg_bits),
        .cfo_valid(), .cfo_t(), .cfo_inc(), .ang16(), .ang32(), .ang128(), .ang256(),
        .a16_re(), .a16_im(), .a32_re(), .a32_im(), .a128_re(), .a128_im(), .a256_re(), .a256_im(),
        .ev_valid(det_ev_valid), .ev_t(det_ev_t), .ev_cfo_inc(det_ev_inc),
        .ev_ang16(det_ev_a16), .ev_ang32(det_ev_a32), .ev_ang128(det_ev_a128), .ev_ang256(det_ev_a256),
        .ev_tone_num(det_ev_num), .ev_tone_den(det_ev_den), .ev_tone_fail(det_ev_fail),
        .tm_valid(det_tm_valid), .tm_t(det_tm_t), .tm_n_end(det_tm_n_end), .tm_status(det_tm_status),
        .tm_n_pos(det_tm_n_pos), .tm_p_max(det_tm_p_max), .tm_p_sum(det_tm_p_sum), .tm_err(det_tm_err),
        .cf_valid(det_cf_valid), .cf_t(det_cf_t), .cf_n_end(det_cf_n_end), .cf_crc_ok(det_cf_crc_ok),
        .cf_msg(det_cf_msg), .cf_shift(det_cf_shift), .cf_metric(det_cf_metric), .cf_err(det_cf_err),
        .arm_late(det_arm_late),
        .pl_cnt(det_pl_cnt), .pl_head(det_pl_head), .pl_pop(pl_last),
        .ch_cnt(det_ch_cnt), .ch_head(det_ch_head), .ch_pop(ch_last), .ch_err(det_ch_err),
        .lq_cnt(det_lq_cnt), .lq_head(det_lq_head), .lq_pop(lq_last),
        .tb_cnt(det_tb_cnt), .tb_word(det_tb_word), .tb_pop(tb_rd_pulse),
        .busy(det_busy), .open_n(det_open_n), .open_t(det_open_t));

    wire       stage2 = (r_stage == 2'd2);
    reg [31:0] samp_cnt;                          // accepted samples since reset
    // a debug record is due with this beat (stage 2: with the second sample of each pair)
    wire       rec_due = accept && (!stage2 || samp_cnt[0]);
    wire [31:0] rec_of_beat = stage2 ? {1'b0, samp_cnt[31:1]} : samp_cnt;
    always @(posedge aclk) begin
        if (!det_rst_n)  samp_cnt <= 32'd0;
        else if (accept) samp_cnt <= samp_cnt + 32'd1;
    end

    // ------------------------------------------------------------------ event FIFO
    reg  [31:0] ev_mem [0:EV_DEPTH-1];
    reg  [7:0]  ev_wr, ev_rd;
    reg  [8:0]  ev_cnt;
    reg  [255:0] ser;                             // event being written, word 0 in bits 31:0
    reg  [3:0]  ser_n;                            // words left to write
    reg         ev_ovf;
    wire        ev_push = (ser_n != 4'd0) && (ev_cnt != EV_DEPTH);
    reg         ev_pop;                           // from the register read below
    always @(posedge aclk) begin
        if (ev_push) ev_mem[ev_wr] <= ser[31:0];
    end
    always @(posedge aclk) begin
        if (!det_rst_n) begin
            ev_wr <= 8'd0; ev_rd <= 8'd0; ev_cnt <= 9'd0; ser_n <= 4'd0; ev_ovf <= 1'b0;
        end else begin
            if (det_ev_valid) begin
                ser <= {24'd0, det_ev_den[39:32], det_ev_den[31:0],
                        det_ev_fail, 23'd0, det_ev_num[39:32], det_ev_num[31:0],
                        det_ev_a256, det_ev_a128, det_ev_a32, det_ev_a16, det_ev_inc, det_ev_t};
                ser_n <= 4'd8;
                if (ser_n != 4'd0) ev_ovf <= 1'b1;
            end else if (ser_n != 4'd0) begin
                if (ev_push) begin
                    ser   <= {32'd0, ser[255:32]};
                    ser_n <= ser_n - 4'd1;
                end else
                    ev_ovf <= 1'b1;               // FIFO full: cannot happen with the input gate
            end
            if (ev_push) ev_wr <= ev_wr + 8'd1;
            if (ev_pop)  ev_rd <= ev_rd + 8'd1;
            ev_cnt <= ev_cnt + {8'd0, ev_push} - {8'd0, ev_pop};
        end
    end

    // ------------------------------------------------------------------ timing FIFO
    // whole records (8 words) per entry; TM_DATA reads the head record word by word
    reg  [255:0] tm_mem [0:TM_DEPTH-1];
    reg  [4:0]   tm_wr, tm_rd;
    reg  [5:0]   tm_cnt;                          // records
    reg  [2:0]   tm_w;                            // next word of the head record
    reg          tm_ovf, tm_err_r;
    reg          tm_pop;                          // from the register read below
    wire         tm_push = det_tm_valid && (tm_cnt != TM_DEPTH);
    wire         tm_last = tm_pop && (tm_w == 3'd7);
    always @(posedge aclk) begin
        if (tm_push)
            tm_mem[tm_wr] <= {16'd0, det_tm_p_sum[47:32], det_tm_p_sum[31:0], 30'd0, det_tm_p_max[33:32],
                              det_tm_p_max[31:0], 16'd0, det_tm_n_pos, 29'd0, det_tm_status,
                              det_tm_n_end, det_tm_t};
    end
    always @(posedge aclk) begin
        if (!det_rst_n) begin
            tm_wr <= 5'd0; tm_rd <= 5'd0; tm_cnt <= 6'd0; tm_w <= 3'd0; tm_ovf <= 1'b0; tm_err_r <= 1'b0;
        end else begin
            if (tm_push) tm_wr <= tm_wr + 5'd1;
            if (det_tm_valid && tm_cnt == TM_DEPTH) tm_ovf <= 1'b1;
            if (det_tm_err) tm_err_r <= 1'b1;
            if (tm_pop) tm_w <= tm_w + 3'd1;
            if (tm_last) tm_rd <= tm_rd + 5'd1;
            tm_cnt <= tm_cnt + {5'd0, tm_push} - {5'd0, tm_last};
        end
    end
    wire [255:0] tm_head  = tm_mem[tm_rd];
    wire [31:0]  tm_word  = tm_head[{tm_w, 5'd0} +: 32];
    wire [8:0]   tm_level = {tm_cnt, 3'd0} - {6'd0, tm_w};

    // ------------------------------------------------------------------ control-record FIFO
    // as the timing FIFO: whole records per entry, CF_DATA reads the head record word by word
    reg  [255:0] cf_mem [0:CF_DEPTH-1];
    reg  [2:0]   cf_wr, cf_rd;
    reg  [3:0]   cf_cnt;
    reg  [2:0]   cf_w;
    reg          cf_ovf, cf_err_r;
    reg          cf_pop;
    wire         cf_push = det_cf_valid && (cf_cnt != CF_DEPTH);
    wire         cf_last = cf_pop && (cf_w == 3'd7);
    always @(posedge aclk) begin
        if (cf_push)
            cf_mem[cf_wr] <= {96'd0, 10'd0, det_cf_msg[53:32], det_cf_msg[31:0],
                              det_cf_metric, 2'd0, det_cf_shift, 7'd0, det_cf_crc_ok, det_cf_n_end, det_cf_t};
    end
    always @(posedge aclk) begin
        if (!det_rst_n) begin
            cf_wr <= 3'd0; cf_rd <= 3'd0; cf_cnt <= 4'd0; cf_w <= 3'd0; cf_ovf <= 1'b0; cf_err_r <= 1'b0;
        end else begin
            if (cf_push) cf_wr <= cf_wr + 3'd1;
            if (det_cf_valid && cf_cnt == CF_DEPTH) cf_ovf <= 1'b1;
            if (det_cf_err) cf_err_r <= 1'b1;
            if (cf_pop) cf_w <= cf_w + 3'd1;
            if (cf_last) cf_rd <= cf_rd + 3'd1;
            cf_cnt <= cf_cnt + {3'd0, cf_push} - {3'd0, cf_last};
        end
    end
    wire [255:0] cf_head  = cf_mem[cf_rd];
    wire [31:0]  cf_word  = cf_head[{cf_w, 5'd0} +: 32];
    wire [6:0]   cf_level = {cf_cnt, 3'd0} - {4'd0, cf_w};

    // ------------------------------------------------------------------ payload records
    // the FIFO is in pd_cf; PL_DATA reads its head word by word and the 4th word removes it
    reg  [1:0]   pl_w;
    reg          pl_rd_pulse;
    // 9.2 NOREC: drop the head record, every other clock (the count has settled by then)
    reg          pl_auto;
    always @(posedge aclk) begin
        if (!det_rst_n) pl_auto <= 1'b0;
        else pl_auto <= ctrl_norec && (det_pl_cnt != 0) && (pl_w == 2'd0) && !pl_auto && !pl_rd_pulse;
    end
    assign       pl_last = (pl_rd_pulse && (pl_w == 2'd3)) || pl_auto;
    always @(posedge aclk) begin
        if (!det_rst_n) pl_w <= 2'd0;
        else if (pl_rd_pulse) pl_w <= pl_w + 2'd1;
    end
    wire [31:0]  pl_word  = det_pl_head[{pl_w, 5'd0} +: 32];
    wire [6:0]   pl_level = {det_pl_cnt, 2'd0} - {5'd0, pl_w};

    // ------------------------------------------------------------------ channel-estimate records
    reg  [1:0]   ch_w;
    reg          ch_rd_pulse;
    // 9.2 NOREC: drop the head record, every other clock (the count has settled by then)
    reg          ch_auto;
    always @(posedge aclk) begin
        if (!det_rst_n) ch_auto <= 1'b0;
        else ch_auto <= ctrl_norec && (det_ch_cnt != 0) && (ch_w == 2'd0) && !ch_auto && !ch_rd_pulse;
    end
    assign       ch_last = (ch_rd_pulse && (ch_w == 2'd3)) || ch_auto;
    always @(posedge aclk) begin
        if (!det_rst_n) ch_w <= 2'd0;
        else if (ch_rd_pulse) ch_w <= ch_w + 2'd1;
    end
    wire [31:0]  ch_word  = det_ch_head[{ch_w, 5'd0} +: 32];
    wire [6:0]   ch_level = {det_ch_cnt, 2'd0} - {5'd0, ch_w};

    // ------------------------------------------------------------------ decoder-input records
    reg  [1:0]   lq_w;
    reg          lq_rd_pulse;
    // 9.2 NOREC: drop the head record, every other clock (the count has settled by then)
    reg          lq_auto;
    always @(posedge aclk) begin
        if (!det_rst_n) lq_auto <= 1'b0;
        else lq_auto <= ctrl_norec && (det_lq_cnt != 0) && (lq_w == 2'd0) && !lq_auto && !lq_rd_pulse;
    end
    assign       lq_last = (lq_rd_pulse && (lq_w == 2'd3)) || lq_auto;
    always @(posedge aclk) begin
        if (!det_rst_n) lq_w <= 2'd0;
        else if (lq_rd_pulse) lq_w <= lq_w + 2'd1;
    end
    wire [31:0]  lq_word  = det_lq_head[{lq_w, 5'd0} +: 32];
    wire [6:0]   lq_level = {det_lq_cnt, 2'd0} - {5'd0, lq_w};

    // ------------------------------------------------------------------ debug FIFO
    // res = records reserved: accepted samples whose record has not left the FIFO (or been
    // dropped with DEBUG = 0). The input is held while res is near the FIFO size.
    localparam integer DD = 1 << DBG_BITS;
    reg  [32:0]         dbg_mem [0:DD-1];
    reg  [DBG_BITS-1:0] dbg_wr, dbg_rd;
    reg  [DBG_BITS:0]   dbg_cnt;
    reg  [7:0]          res;
    reg  [31:0]         rec_n, dbg_sent;
    // input beats with TLAST (sample index), to mark their records
    reg  [31:0]         tl [0:3];
    reg  [1:0]          tl_wr, tl_rd;
    reg  [2:0]          tl_n;
    wire                tl_hit   = (tl_n != 3'd0) && (rec_n == tl[tl_rd]);
    wire                dbg_push = det_dbg_valid && ctrl_debug;
    wire                dbg_drop = det_dbg_valid && !ctrl_debug;
    wire                dbg_pop  = !ctrl_loop && (dbg_cnt != 0) && m_axis_tready;
    always @(posedge aclk) begin
        if (dbg_push) dbg_mem[dbg_wr] <= {tl_hit, det_dbg_bits, det_dbg_metric};
    end
    always @(posedge aclk) begin
        if (!det_rst_n) begin
            dbg_wr <= 0; dbg_rd <= 0; dbg_cnt <= 0; res <= 8'd0; rec_n <= 32'd0; dbg_sent <= 32'd0;
            tl_wr <= 2'd0; tl_rd <= 2'd0; tl_n <= 3'd0;
        end else begin
            if (dbg_push) dbg_wr <= dbg_wr + 1'b1;
            if (dbg_pop) begin dbg_rd <= dbg_rd + 1'b1; dbg_sent <= dbg_sent + 32'd1; end
            dbg_cnt <= dbg_cnt + {{DBG_BITS{1'b0}}, dbg_push} - {{DBG_BITS{1'b0}}, dbg_pop};
            res     <= res + {7'd0, rec_due} - {7'd0, dbg_pop} - {7'd0, dbg_drop};
            if (det_dbg_valid) rec_n <= rec_n + 32'd1;
            if (rec_due && u_tlast && ctrl_debug) begin
                tl[tl_wr] <= rec_of_beat; tl_wr <= tl_wr + 2'd1;
            end
            tl_n <= tl_n + {2'd0, rec_due && u_tlast && ctrl_debug} - {2'd0, det_dbg_valid && tl_hit};
            if (det_dbg_valid && tl_hit) tl_rd <= tl_rd + 2'd1;
        end
    end
    wire [32:0] dbg_head = dbg_mem[dbg_rd];

    // ------------------------------------------------------------------ input gate
    always @(posedge aclk) begin
        if (!det_rst_n) gate <= 1'b0;
        else gate <= !ctrl_loop && (ev_cnt < EV_LIMIT) && (!ctrl_debug || res < DBG_LIMIT) &&
                     (tl_n < 3'd3) && (tm_cnt < TM_LIMIT) && (cf_cnt < CF_LIMIT);
    end

    // ------------------------------------------------------------------ streams
    assign u_tready = ctrl_loop ? m_axis_tready : (gate && det_tready);
    assign m_axis_tvalid = ctrl_loop ? u_tvalid : (dbg_cnt != 0);
    assign m_axis_tdata  = ctrl_loop ? u_tdata  : dbg_head[31:0];
    assign m_axis_tlast  = ctrl_loop ? u_tlast  : dbg_head[32];
    assign m_axis_tkeep  = 4'hF;

    // ------------------------------------------------------------------ AXI4-Lite
    // det_busy registered first (a long path otherwise; one clock late is harmless for polling)
    reg  det_busy_r;
    always @(posedge aclk) det_busy_r <= det_busy;
    wire busy = det_busy_r || (ser_n != 4'd0) || (dbg_cnt != 0);
    wire [31:0] status = {15'd0, det_ch_err, det_arm_late, cf_err_r, cf_ovf, tm_err_r, tm_ovf, ctrl_loop, !det_rst_n, ev_ovf, det_open_n, 2'd0,
                          det_open_n != 4'd0, busy};
    reg  awready_r, wready_r, arready_r;
    reg  [11:0] wa;
    assign s_axi_awready = awready_r;
    assign s_axi_wready  = wready_r;
    assign s_axi_arready = arready_r;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_rresp   = 2'b00;

    always @(posedge aclk) begin
        if (!aresetn) begin
            awready_r <= 1'b0; wready_r <= 1'b0; s_axi_bvalid <= 1'b0;
            ctrl_reset <= 1'b1; ctrl_debug <= 1'b0; ctrl_loop <= 1'b0; ctrl_pack <= 1'b0; ctrl_norec <= 1'b0;
            r_eta_hi <= 16'd8028; r_eta_lo <= 16'd2621; r_qual <= 16'd128; r_ncfo <= 16'd2048;
            r_blank <= 32'd288; r_hold <= 32'd131072; r_stage <= 2'd0; r_shift <= 4'd10;
            r_lin <= 24'd1307615; r_tpk <= 16'd4608; r_rearm <= 1'b1;
        end else begin
            awready_r <= 1'b0;
            wready_r  <= 1'b0;
            // accept a write when address and data are both there; respond after the handshake
            if (!awready_r && !s_axi_bvalid && s_axi_awvalid && s_axi_wvalid) begin
                awready_r <= 1'b1;
                wready_r  <= 1'b1;
                case (s_axi_awaddr[7:2])
                    6'h01: begin ctrl_reset <= s_axi_wdata[0]; ctrl_debug <= s_axi_wdata[1];
                                 ctrl_loop <= s_axi_wdata[2]; ctrl_pack <= s_axi_wdata[3];
                                 ctrl_norec <= s_axi_wdata[4]; end
                    6'h08: r_eta_hi <= s_axi_wdata[15:0];
                    6'h09: r_eta_lo <= s_axi_wdata[15:0];
                    6'h0A: r_qual   <= s_axi_wdata[15:0];
                    6'h0B: r_ncfo   <= s_axi_wdata[15:0];
                    6'h0C: r_blank  <= s_axi_wdata;
                    6'h0D: r_hold   <= s_axi_wdata;
                    6'h0E: r_stage  <= s_axi_wdata[1:0];
                    6'h0F: r_shift  <= s_axi_wdata[3:0];
                    6'h10: r_lin    <= s_axi_wdata[23:0];
                    6'h11: r_tpk    <= s_axi_wdata[15:0];
                    6'h16: r_rearm  <= s_axi_wdata[0];
                    default: ;
                endcase
            end
            if (awready_r)                        s_axi_bvalid <= 1'b1;
            else if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end

    always @(posedge aclk) begin
        if (!aresetn) begin
            arready_r <= 1'b0; s_axi_rvalid <= 1'b0; ev_pop <= 1'b0; tm_pop <= 1'b0; cf_pop <= 1'b0; pl_rd_pulse <= 1'b0; ch_rd_pulse <= 1'b0; lq_rd_pulse <= 1'b0; tb_rd_pulse <= 1'b0;
        end else begin
            arready_r <= 1'b0;
            ev_pop    <= 1'b0;
            tm_pop    <= 1'b0;
            cf_pop    <= 1'b0;
            pl_rd_pulse <= 1'b0;
            ch_rd_pulse <= 1'b0;
            lq_rd_pulse <= 1'b0;
            tb_rd_pulse <= 1'b0;
            if (!arready_r && !s_axi_rvalid && s_axi_arvalid) begin
                arready_r <= 1'b1;
                case (s_axi_araddr[7:2])
                    6'h00: s_axi_rdata <= ID;
                    6'h01: s_axi_rdata <= {27'd0, ctrl_norec, ctrl_pack, ctrl_loop, ctrl_debug, ctrl_reset};
                    6'h02: s_axi_rdata <= status;
                    6'h03: s_axi_rdata <= samp_cnt;
                    6'h04: s_axi_rdata <= det_open_t;
                    6'h05: s_axi_rdata <= {23'd0, ev_cnt};
                    6'h06: begin
                        s_axi_rdata <= (ev_cnt != 0) ? ev_mem[ev_rd] : 32'd0;
                        ev_pop      <= (ev_cnt != 0) && det_rst_n;
                    end
                    6'h07: s_axi_rdata <= dbg_sent;
                    6'h08: s_axi_rdata <= {16'd0, r_eta_hi};
                    6'h09: s_axi_rdata <= {16'd0, r_eta_lo};
                    6'h0A: s_axi_rdata <= {16'd0, r_qual};
                    6'h0B: s_axi_rdata <= {16'd0, r_ncfo};
                    6'h0C: s_axi_rdata <= r_blank;
                    6'h0D: s_axi_rdata <= r_hold;
                    6'h0E: s_axi_rdata <= {30'd0, r_stage};
                    6'h0F: s_axi_rdata <= {28'd0, r_shift};
                    6'h10: s_axi_rdata <= {8'd0, r_lin};
                    6'h11: s_axi_rdata <= {16'd0, r_tpk};
                    6'h12: s_axi_rdata <= {23'd0, tm_level};
                    6'h13: begin
                        s_axi_rdata <= (tm_cnt != 0) ? tm_word : 32'd0;
                        tm_pop      <= (tm_cnt != 0) && det_rst_n;
                    end
                    6'h14: s_axi_rdata <= {25'd0, cf_level};
                    6'h16: s_axi_rdata <= {31'd0, r_rearm};
                    6'h17: s_axi_rdata <= {25'd0, pl_level};
                    6'h18: begin
                        s_axi_rdata <= (det_pl_cnt != 0) ? pl_word : 32'd0;
                        pl_rd_pulse <= (det_pl_cnt != 0) && det_rst_n;
                    end
                    6'h19: s_axi_rdata <= {25'd0, ch_level};
                    6'h1A: begin
                        s_axi_rdata <= (det_ch_cnt != 0) ? ch_word : 32'd0;
                        ch_rd_pulse <= (det_ch_cnt != 0) && det_rst_n;
                    end
                    6'h1B: s_axi_rdata <= {25'd0, lq_level};
                    6'h1C: begin
                        s_axi_rdata <= (det_lq_cnt != 0) ? lq_word : 32'd0;
                        lq_rd_pulse <= (det_lq_cnt != 0) && det_rst_n;
                    end
                    6'h1D: s_axi_rdata <= {20'd0, det_tb_cnt};
                    6'h1E: begin
                        s_axi_rdata <= (det_tb_cnt != 0) ? det_tb_word : 32'd0;
                        tb_rd_pulse <= (det_tb_cnt != 0) && det_rst_n;
                    end
                    6'h15: begin
                        s_axi_rdata <= (cf_cnt != 0) ? cf_word : 32'd0;
                        cf_pop      <= (cf_cnt != 0) && det_rst_n;
                    end
                    default: s_axi_rdata <= 32'd0;
                endcase
            end
            if (arready_r)                         s_axi_rvalid <= 1'b1;
            else if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
        end
    end
endmodule

`default_nettype wire
