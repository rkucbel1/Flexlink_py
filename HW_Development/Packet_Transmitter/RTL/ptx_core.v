// ptx_core.v - Flexlink packet transmitter: parameter header + transport blocks in, 12-bit samples out.
// Bit-true to ../PC/ptx_reference (record_words -> packet): every output sample equals the model's.
//
// Input: 64-bit words (AXI-Stream, from the TX DMA), records one after the other. A record is a
// parameter header - the Defined sections of Tx/Parameters_Tx_payload.json and
// Tx/ControlSignals/Parameters_Control_Field.json - and then n_packets x
// AvailableDataBlocks transport blocks, each TransportBlockSize bits starting in a new word (bit j
// of a block in word j / 64, bit j % 64). The PL derives the rest (Calculated values, control-field
// message, gains).
//   word 0  [31:0] magic 0x50584C46 ("FLXP")   [39:32] version 2
//   word 1  [1:0]   Mod (0 BPSK, 1 QPSK, 2 16QAM, 3 64QAM)   [5:2]   RefSymbolSpacing (1, 3, 6, 12)
//           [10:6]  RefSignalSpacing (3, 6, 12, 24)          [17:11] PacketLength (37 .. 121)
//           [18]    Port   [19] NumberOfAntennas - 1   [20] ClientFlag (1 Client)
//           [21]    Use_Preamble   [22] Use_ControlField
//           [24:23] EncoderCodeRate (0 1/2, 1 2/3, 2 3/4, 3 5/6)   [26:25] EncoderBlockSize (0 648, 1 1296, 2 1944)
//           [31:27] Repetition (1, 3, 5, 7, 11, 15, 19, 23, 31)   [39:32] C_init_Dat
//           [53:40] Tx_Ref_Clk_Cnt   [55:54] CRC (0 Gen24)
//   word 2  [30:0]  C_init_ID1   [61:31] C_init_ID2
//   word 3  [15:0], [31:16], [47:32] output gains preamble, control field, payload (0: the PL's
//           own value for 13 dB backoff)   [63:48] gap after each packet, in units of 16 samples
//   word 4  [31:0]  n_packets (0: parameters only)   [63:32] SampleRate in kHz (40000)
//   word 5  [30:0]  control field C_init_ID1 (its Ref_P0 pilots)   [61:31] control field C_init_Dat
//           (its scrambler; 129 and 130 in every Flexlink control field so far)
// A word that does not start a valid header is dropped (status: header errors) and the next one
// tried. Output: 12-bit I/Q samples (m_valid / m_ready), each packet then its gap; m_active marks
// packet samples.
//
// Blocks: FIFO -> bit buffer -> [header, parameters, control-field encoder] [ptx_enc: CRC24, LDPC,
// interleaver] -> bit sources (copies, zero padding, Gold scramblers) -> mapper (one RE per clock;
// pilots from the Gold code C_init_ID) -> frequency buffer -> feeder -> ptx_ifft -> output banks ->
// player (preamble ROM, cyclic prefix, gap) -> sat12(rnd(x * gain, 16)).
`timescale 1ns / 1ps
module ptx_core #(
  parameter FIFO_AW = 12                     // input FIFO 2^FIFO_AW x 64 bit (4096: 8 BRAM36)
) (
  input  wire        clk,
  input  wire        rst_n,
  input  wire [63:0] s_tdata,
  input  wire        s_tvalid,
  output wire        s_tready,
  output reg         m_valid,
  input  wire        m_ready,
  output reg  [11:0] m_i,
  output reg  [11:0] m_q,
  output reg         m_active,
  output reg  [31:0] st_packets,             // packets completely played
  output reg  [15:0] st_hdr_errors,          // words dropped looking for a header, rejected headers
  output reg  [31:0] st_bubbles,             // player cycles inside a packet without a sample ready
  output wire        st_busy
);
  localparam [31:0] MAGIC = 32'h50584C46;
  localparam LATF = 2075;                    // feeder 2 + ptx_ifft 2073

  // ------------------------------------------------------------------------------------------
  // input FIFO (first word fall through)
  localparam DEPTH = 1 << FIFO_AW;
  reg  [63:0] fmem [0:DEPTH-1];
  reg  [FIFO_AW:0] fwp, frp;
  reg         fov;
  reg  [63:0] fod;
  wire [FIFO_AW:0] fused = fwp - frp;
  wire        f_full = (fused == DEPTH);
  wire        f_mempty = (fwp == frp);
  wire        f_pop;
  wire        f_load = !f_mempty && (!fov || f_pop);
  wire [FIFO_AW+1:0] f_level = fused + fov;
  assign s_tready = !f_full;
  always @(posedge clk) begin
    if (!rst_n) begin
      fwp <= 0; frp <= 0; fov <= 1'b0;
    end else begin
      if (s_tvalid && !f_full) begin
        fmem[fwp[FIFO_AW-1:0]] <= s_tdata;
        fwp <= fwp + 1'b1;
      end
      if (f_load) begin
        fod <= fmem[frp[FIFO_AW-1:0]];
        frp <= frp + 1'b1;
      end
      fov <= f_load ? 1'b1 : (f_pop ? 1'b0 : fov);
    end
  end

  // ------------------------------------------------------------------------------------------
  // bit reader: the FIFO's output word fod (valid: fov) and the next bit in it, bp (LSB first).
  // Every reader is word aligned: header words are taken whole (bp = 0) and the encoder's align
  // at the end of a transport block drops the rest of the word. So no shifter is needed.
  reg  [5:0]  bp;
  wire        hdr_take;
  wire        enc_take, enc_align;
  wire        bit_in = fod[bp];
  assign f_pop = hdr_take || (enc_take && bp == 6'd63) || (enc_align && bp != 6'd0);
  always @(posedge clk) begin
    if (!rst_n)                  bp <= 6'd0;
    else if (hdr_take || enc_align) bp <= 6'd0;
    else if (enc_take)           bp <= bp + 1'b1;
  end

  // ------------------------------------------------------------------------------------------
  // record FSM: header, checks, derived values, control-field encoding
  localparam R_H0 = 4'd0, R_H1 = 4'd1, R_H2 = 4'd2, R_H3 = 4'd3, R_H4 = 4'd4, R_H5 = 4'd5, R_CHK = 4'd6,
             R_C1 = 4'd7, R_C2 = 4'd8, R_C3 = 4'd9, R_DIV = 4'd10, R_G = 4'd11, R_CF = 4'd12, R_RUN = 4'd13;
  reg [3:0]  rs;
  reg [63:0] hw1, hw2, hw3, hw4, hw5;
  wire [63:0] w64 = fod;
  assign     hdr_take = (rs <= R_H5) && fov && (bp == 6'd0);

  wire [1:0] r_mod  = hw1[1:0];
  wire [3:0] r_rsym = hw1[5:2];
  wire [4:0] r_rsig = hw1[10:6];
  wire [6:0] r_plen = hw1[17:11];
  wire       r_port = hw1[18];
  wire       r_ant  = hw1[19];
  wire       r_cli  = hw1[20];
  wire       r_upre = hw1[21];
  wire       r_ucf  = hw1[22];
  wire [1:0] r_rate = hw1[24:23];
  wire [1:0] r_cbs  = hw1[26:25];
  wire [4:0] r_rep  = hw1[31:27];
  wire [7:0] r_cdat = hw1[39:32];
  wire [13:0] r_clk = hw1[53:40];
  wire [1:0] r_crc  = hw1[55:54];
  wire [30:0] r_id1 = hw2[30:0];
  wire [30:0] r_id2 = hw2[61:31];
  wire [31:0] r_npk = hw4[31:0];
  wire [31:0] r_fs  = hw4[63:32];
  wire [30:0] r_cfid = hw5[30:0];
  wire [30:0] r_cfdat = hw5[61:31];

  // codes (control field) and validity
  wire [1:0] c_rsym = (r_rsym == 4'd1) ? 2'd0 : (r_rsym == 4'd3) ? 2'd1 : (r_rsym == 4'd6) ? 2'd2 : 2'd3;
  wire [1:0] c_rsig = (r_rsig == 5'd3) ? 2'd0 : (r_rsig == 5'd6) ? 2'd1 : (r_rsig == 5'd12) ? 2'd2 : 2'd3;
  wire [2:0] c_plen = (r_plen == 7'd37) ? 3'b011 : (r_plen == 7'd49) ? 3'b111 : (r_plen == 7'd73) ? 3'b100 :
                      (r_plen == 7'd97) ? 3'b110 : 3'b101;                      // PacketLength field
  wire [2:0] i_plen = (r_plen == 7'd37) ? 3'd0 : (r_plen == 7'd49) ? 3'd1 : (r_plen == 7'd73) ? 3'd2 :
                      (r_plen == 7'd97) ? 3'd3 : 3'd4;                          // gain ROM index
  reg  [3:0] c_rm;
  reg        rep_ok;
  always @* begin
    rep_ok = 1'b1;
    case (r_rep)
      5'd1: c_rm = 4'd0;  5'd3: c_rm = 4'd1;  5'd5: c_rm = 4'd2;  5'd7: c_rm = 4'd3;  5'd11: c_rm = 4'd4;
      5'd15: c_rm = 4'd5; 5'd19: c_rm = 4'd6; 5'd23: c_rm = 4'd7; 5'd31: c_rm = 4'd8;
      default: begin c_rm = 4'd0; rep_ok = 1'b0; end
    endcase
  end
  wire rsym_ok = (r_rsym == 4'd1) || (r_rsym == 4'd3) || (r_rsym == 4'd6) || (r_rsym == 4'd12);
  wire rsig_ok = (r_rsig == 5'd3) || (r_rsig == 5'd6) || (r_rsig == 5'd12) || (r_rsig == 5'd24);
  wire plen_ok = (r_plen == 7'd37) || (r_plen == 7'd49) || (r_plen == 7'd73) || (r_plen == 7'd97) ||
                 (r_plen == 7'd121);
  wire cfg_ok  = rsym_ok && rsig_ok && plen_ok && rep_ok && (r_cbs != 2'd3) && (r_crc == 2'd0) &&
                 (r_fs == 32'd40000);

  // derived
  reg [2:0]  d_nb;
  reg [6:0]  d_nref;
  reg [9:0]  d_dref;
  reg [16:0] d_dres;
  reg [19:0] d_dbits;
  reg [15:0] d_cw;
  reg [10:0] d_tb;
  reg [4:0]  d_nwtb;
  reg [9:0]  d_ndb;
  reg [14:0] d_wpk;
  reg [19:0] dv_rem;
  reg [4:0]  dv_i;
  reg [19:0] dv_q;
  reg [2:0]  g_sel;
  reg [30:0] d_x2dat, d_x2id1, d_x2id2, d_x2cfid, d_x2cfdat;
  wire [30:0] g_x1, g_x2;
  ptx_gold_start gs (.c_init((g_sel == 3'd0) ? {23'd0, r_cdat} : (g_sel == 3'd1) ? r_id1 : (g_sel == 3'd2) ? r_id2 :
                             (g_sel == 3'd3) ? r_cfid : r_cfdat), .x1(g_x1), .x2(g_x2));
  wire [15:0] rom_gpl, rom_gpre, rom_gcf;
  ptx_gain_rom grom (.addr({r_mod, c_rsym, c_rsig, i_plen}), .g_pl(rom_gpl), .g_pre(rom_gpre), .g_cf(rom_gcf));
  reg [15:0] d_gpre, d_gcf, d_gpl;

  // control-field encoder: 54-bit message, CRC10, 6 tail zeros, BCC K = 7 (133, 171)
  reg [53:0]  cf_msr;
  reg [9:0]   cf_crc;
  reg [5:0]   cf_s;
  reg [139:0] cf_code;
  reg [6:0]   cf_i;
  wire        cf_b = (cf_i < 7'd54) ? cf_msr[53] : (cf_i < 7'd64) ? cf_crc[9] : 1'b0;
  wire [6:0]  cf_v = {cf_b, cf_s};
  wire [139:0] cf_int;
  ptx_bcc_il bil (.code(cf_code), .il(cf_int));

  // mapper / encoder handshake
  reg        m_pk;                            // mapper inside a packet
  reg [31:0] m_started;
  reg        enc_start, enc_started;
  wire       enc_busy;
  wire [31:0] enc_tbs;
  reg [31:0] enc_tbs_seen;
  reg [31:0] w_need, w_got;
  // whole words not yet taken: FIFO, plus fod while none of its bits is taken
  wire [32:0] have_words = {1'b0, f_level} - {32'd0, (bp != 6'd0)};
  reg         w_enough;                       // have_words + w_got >= w_need, one clock late (timing)
  wire        dq_full;
  wire        pk_start;

  always @(posedge clk) begin
    if (!rst_n) begin
      rs <= R_H0; st_hdr_errors <= 16'd0; hw1 <= 64'd0; hw2 <= 64'd0; hw3 <= 64'd0; hw4 <= 64'd0;
      enc_start <= 1'b0; enc_started <= 1'b0; g_sel <= 3'd0; cf_i <= 7'd0; hw5 <= 64'd0;
    end else begin
      enc_start <= 1'b0;
      case (rs)
        R_H0: if (hdr_take) begin
          if (w64[31:0] == MAGIC && w64[39:32] == 8'd2) rs <= R_H1;
          else st_hdr_errors <= st_hdr_errors + 1'b1;
        end
        R_H1: if (hdr_take) begin hw1 <= w64; rs <= R_H2; end
        R_H2: if (hdr_take) begin hw2 <= w64; rs <= R_H3; end
        R_H3: if (hdr_take) begin hw3 <= w64; rs <= R_H4; end
        R_H4: if (hdr_take) begin hw4 <= w64; rs <= R_H5; end
        R_H5: if (hdr_take) begin hw5 <= w64; rs <= R_CHK; end
        R_CHK: if (cfg_ok) rs <= R_C1; else begin st_hdr_errors <= st_hdr_errors + 1'b1; rs <= R_H0; end
        R_C1: begin
          d_nb   <= (r_mod == 2'd0) ? 3'd1 : (r_mod == 2'd1) ? 3'd2 : (r_mod == 2'd2) ? 3'd4 : 3'd6;
          // reference symbols: (PacketLength - 1) / RefSymbolSpacing + 1
          d_nref <= (r_rsym == 4'd1) ? r_plen :
                    (r_rsym == 4'd3) ? (r_plen - 7'd1) / 7'd3 + 7'd1 :
                    (r_rsym == 4'd6) ? (r_plen - 7'd1) / 7'd6 + 7'd1 : (r_plen - 7'd1) / 7'd12 + 7'd1;
          d_dref <= (r_rsig == 5'd3) ? 10'd274 : (r_rsig == 5'd6) ? 10'd544 : (r_rsig == 5'd12) ? 10'd670 : 10'd726;
          d_cw   <= ((r_cbs == 2'd0) ? 16'd648 : (r_cbs == 2'd1) ? 16'd1296 : 16'd1944) * ({11'd0, r_rep} + 16'd1);
          d_tb   <= (11'd24 - ((r_rate == 2'd0) ? 11'd12 : (r_rate == 2'd1) ? 11'd8 : (r_rate == 2'd2) ? 11'd6 : 11'd4)) *
                    ((r_cbs == 2'd0) ? 11'd27 : (r_cbs == 2'd1) ? 11'd54 : 11'd81) - 11'd24;
          d_gpre <= (hw3[15:0]  != 16'd0) ? hw3[15:0]  : rom_gpre;
          d_gcf  <= (hw3[31:16] != 16'd0) ? hw3[31:16] : rom_gcf;
          d_gpl  <= (hw3[47:32] != 16'd0) ? hw3[47:32] : rom_gpl;
          rs <= R_C2;
        end
        R_C2: begin
          d_dres <= {10'd0, r_plen} * 17'd826 - {10'd0, d_nref} * (17'd826 - {7'd0, d_dref});
          d_nwtb <= (d_tb + 11'd63) >> 6;
          rs <= R_C3;
        end
        R_C3: begin
          d_dbits <= {3'd0, d_dres} * {17'd0, d_nb};
          dv_rem <= 20'd0; dv_i <= 5'd19; dv_q <= 20'd0;
          rs <= R_DIV;
        end
        R_DIV: begin                                // restoring division d_dbits / d_cw
          if ({dv_rem[18:0], d_dbits[dv_i]} >= {4'd0, d_cw}) begin
            dv_rem <= {dv_rem[18:0], d_dbits[dv_i]} - {4'd0, d_cw};
            dv_q[dv_i] <= 1'b1;
          end else dv_rem <= {dv_rem[18:0], d_dbits[dv_i]};
          if (dv_i == 5'd0) rs <= R_G;
          dv_i <= dv_i - 1'b1;
          g_sel <= 3'd0;
        end
        R_G: begin                                  // NDB, words per packet, Gold start states
          if (g_sel == 3'd0) begin
            if (dv_q == 20'd0 || dv_q > 20'd1023) begin
              st_hdr_errors <= st_hdr_errors + 1'b1; rs <= R_H0;
            end
            d_ndb <= dv_q[9:0];
            d_wpk <= dv_q[9:0] * d_nwtb;
            d_x2dat <= g_x2;
          end
          if (g_sel == 3'd1) d_x2id1 <= g_x2;
          if (g_sel == 3'd2) d_x2id2 <= g_x2;
          if (g_sel == 3'd3) d_x2cfid <= g_x2;
          if (g_sel == 3'd4) begin
            d_x2cfdat <= g_x2;
            cf_msr <= {c_rsym, c_rsig, c_plen, r_mod, r_ant, r_cbs, r_rate, c_rm, d_ndb, r_cli, r_clk, r_cdat, 3'b000};
            cf_crc <= 10'd0; cf_s <= 6'd0; cf_i <= 7'd0;
            rs <= R_CF;
          end
          g_sel <= g_sel + 1'b1;
        end
        R_CF: begin
          cf_code <= {^(cf_v & 7'b1111001), ^(cf_v & 7'b1011011), cf_code[139:2]};
          cf_s <= cf_v[6:1];
          if (cf_i < 7'd54) begin
            cf_msr <= {cf_msr[52:0], 1'b0};
            cf_crc <= {cf_crc[8:0], 1'b0} ^ ((cf_crc[9] ^ cf_b) ? 10'b1111011001 : 10'd0);
          end else cf_crc <= {cf_crc[8:0], 1'b0};
          cf_i <= cf_i + 1'b1;
          if (cf_i == 7'd69) begin
            rs <= (r_npk == 32'd0) ? R_H0 : R_RUN;
            enc_start <= (r_npk != 32'd0);
            enc_started <= 1'b0;
          end
        end
        R_RUN: begin
          if (enc_busy) enc_started <= 1'b1;
          if (enc_started && !enc_busy && m_started == r_npk && !pk_start) rs <= R_H0;
        end
        default: rs <= R_H0;
      endcase
    end
  end

  ptx_enc enc (.clk(clk), .rst_n(rst_n), .start(enc_start), .cbs_c(r_cbs), .rate_c(r_rate), .ndb(d_ndb),
               .n_pk(r_npk), .busy(enc_busy), .tbs_started(enc_tbs), .bit_in(bit_in), .bit_avail(fov),
               .bit_take(enc_take), .align(enc_align), .rd_addr(cw_addr), .rd_data(cw_data), .cw_full(cw_full),
               .rel(cw_rel), .rel_bank(s_bank));

  // the start rule's word count, registered: it is a 33-bit add and compare. One clock late is
  // harmless: pk_start sets m_pk, which blocks the next start for the whole packet.
  always @(posedge clk) w_enough <= (have_words + {1'b0, w_got} >= {1'b0, w_need});

  // ------------------------------------------------------------------------------------------
  // packet start: latch the mapper's settings, descriptor to the player
  reg [1:0]  m_mod;
  reg [3:0]  m_rsym;
  reg [1:0]  m_sigc;
  reg [6:0]  m_plen;
  reg        m_port, m_ucf;
  reg [2:0]  m_nb;
  reg [4:0]  m_rep;
  reg [9:0]  m_ndb;
  reg [7:0]  m_nw;                            // words per code word copy (N / 12)
  reg [30:0] m_x2dat, m_x2id1, m_x2id2, m_x2cfid;
  reg [72:0] dq [0:3];
  reg [2:0]  dq_wp, dq_rp;
  assign     dq_full = (dq_wp - dq_rp) == 3'd4;
  wire       dq_empty = (dq_wp == dq_rp);
  assign pk_start = (rs == R_RUN) && (m_started != r_npk) && !m_pk && !dq_full &&
                    (w_enough || f_full || (enc_started && !enc_busy));

  // mapper state
  reg        m_cf;                            // mapping control-field symbols
  reg [6:0]  m_sym;
  reg [3:0]  m_rcnt;
  reg        m_run;
  reg [9:0]  m_k;
  reg        m_wb;
  reg [1:0]  fb_full;
  reg [1:0]  fb_cf, fb_last;
  reg [1:0]  fb_mod [0:1];
  reg        b_v;
  reg [9:0]  b_k;
  reg        b_ref, b_cf, b_lastsym;
  wire [1:0] b_t;
  reg [7:0]  fbuf [0:2047];
  reg        r_bank;
  wire       fb_release;

  wire [1:0] sig_code = m_cf ? 2'd0 : m_sigc;
  wire       port_m = m_cf ? 1'b0 : m_port;
  wire [1:0] mod_m  = m_cf ? 2'd0 : m_mod;
  wire [2:0] nb     = m_cf ? 3'd1 : m_nb;
  wire       m_isref = m_cf ? ((m_sym == 7'd0) || (m_sym == 7'd3)) : (m_rcnt == 4'd0);
  wire [6:0] m_nsym = m_cf ? 7'd4 : m_plen;

  // bit sources
  //   control field: bit j = (j < 2100 ? interleaved code bit j mod 140 : 0) ^ Gold(control C_init_Dat)
  reg [139:0] cfr;
  reg [11:0]  cf_j;
  reg [30:0]  gc1, gc2;
  //   payload: code-word copies, zero padding, ^ Gold(C_init_Dat)
  reg         s_bank;
  reg [9:0]   s_tb;
  reg [4:0]   s_copy;
  reg [7:0]   s_w;
  reg [3:0]   s_off;
  reg [30:0]  gp1, gp2;
  wire [1:0]  cw_full;
  wire [8:0]  cw_addr = {s_bank, s_w};
  wire [11:0] cw_data;
  wire        s_zero = (s_tb == m_ndb);
  wire [11:0] raw12 = cw_data >> s_off;
  wire [5:0]  graw = gp1[5:0] ^ gp2[5:0];
  wire [5:0]  sb_pl = (s_zero ? 6'd0 : raw12[5:0]) ^ graw;
  wire        sb_cf = ((cf_j < 12'd2100) ? cfr[0] : 1'b0) ^ (gc1[0] ^ gc2[0]);
  wire [5:0]  sb = m_cf ? {5'd0, sb_cf} : sb_pl;
  wire        src_avail = m_cf || s_zero || cw_full[s_bank];
  //   pilots: 1, 0, 1, ... ^ Gold(control-field C_init_ID1, or the payload's C_init_ID1 / ID2),
  //   restarted for every reference symbol
  reg [30:0]  pg1, pg2;
  reg         p_alt;
  wire        pil_bit = p_alt ^ (pg1[0] ^ pg2[0]);

  function [61:0] gstep;                      // Gold registers {x1, x2} advanced by n bits (1 .. 6)
    input [30:0] x1, x2;
    input [2:0]  n;
    integer t;
    reg [30:0] a, b;
    begin
      a = x1; b = x2;
      for (t = 0; t < 6; t = t + 1)
        if (t < n) begin
          a = {a[0] ^ a[3], a[30:1]};
          b = {b[0] ^ b[1] ^ b[2] ^ b[3], b[30:1]};
        end
      gstep = {a, b};
    end
  endfunction

  // type of the RE in stage B
  wire       b_dc_dat = (b_k >= 10'd414) && (b_k <= 10'd428);
  wire [1:0] b_type = b_ref ? b_t : (b_dc_dat ? 2'd3 : 2'd0);   // 0 Data 1 P0 2 P1 3 DC
  wire       b_data = (b_type == 2'd0);
  wire       b_pilot = (b_type == (port_m ? 2'd2 : 2'd1));
  wire       b_go = b_v && (!b_data || src_avail) && !fb_full[m_wb];
  wire       a_adv = !b_v || b_go;
  wire       a_issue = m_run && a_adv && m_pk;
  wire       src_take = b_go && b_data;
  wire       pl_take = src_take && !m_cf;
  wire       copy_end = pl_take && !s_zero && (s_off + {1'b0, m_nb} == 4'd12) && (s_w == m_nw - 1'b1);
  wire       cw_rel = copy_end && (s_copy == m_rep);

  ptx_type_rom trom (.clk(clk), .ce(a_issue), .addr({sig_code, m_k}), .t(b_t));

  function [3:0] pam;
    input [1:0] half;
    input [2:0] lab;
    begin
      case (half)
        2'd1: pam = lab[0] ? 4'sd1 : -4'sd1;
        2'd2: case (lab[1:0]) 2'b00: pam = -4'sd3; 2'b01: pam = -4'sd1; 2'b11: pam = 4'sd1;
                              default: pam = 4'sd3; endcase
        default: case (lab) 3'b000: pam = -4'sd7; 3'b001: pam = -4'sd5; 3'b011: pam = -4'sd3;
                            3'b010: pam = -4'sd1; 3'b110: pam = 4'sd1; 3'b111: pam = 4'sd3;
                            3'b101: pam = 4'sd5; default: pam = 4'sd7; endcase
      endcase
    end
  endfunction
  wire [3:0] outer = (mod_m == 2'd3) ? 4'sd7 : (mod_m == 2'd2) ? 4'sd3 : 4'sd1;
  reg  [3:0] li, lq;
  always @* begin
    li = 4'd0; lq = 4'd0;
    if (b_data) begin
      case (mod_m)
        2'd0: begin li = sb[0] ? 4'sd1 : -4'sd1; lq = 4'd0; end
        2'd1: begin li = pam(2'd1, {2'b00, sb[0]}); lq = pam(2'd1, {2'b00, sb[1]}); end
        2'd2: begin li = pam(2'd2, {1'b0, sb[0], sb[1]}); lq = pam(2'd2, {1'b0, sb[2], sb[3]}); end
        default: begin li = pam(2'd3, {sb[0], sb[1], sb[2]}); lq = pam(2'd3, {sb[3], sb[4], sb[5]}); end
      endcase
    end else if (b_pilot) begin
      if (mod_m == 2'd0) begin li = pil_bit ? -4'sd1 : 4'sd1; lq = 4'd0; end
      else begin li = pil_bit ? outer : -outer; lq = pil_bit ? -outer : outer; end
    end
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      m_pk <= 1'b0; m_started <= 32'd0; m_run <= 1'b0; b_v <= 1'b0; m_wb <= 1'b0; fb_full <= 2'b00;
      dq_wp <= 3'd0; m_cf <= 1'b1; m_sym <= 7'd0; m_rcnt <= 4'd0; m_k <= 10'd0;
      s_bank <= 1'b0; s_tb <= 10'd0; s_copy <= 5'd0; s_w <= 8'd0; s_off <= 4'd0;
      enc_tbs_seen <= 32'd0; w_need <= 32'd0; w_got <= 32'd0; cf_j <= 12'd0;
      m_mod <= 2'd0; m_rsym <= 4'd1; m_sigc <= 2'd0; m_plen <= 7'd37; m_port <= 1'b0; m_ucf <= 1'b1;
      m_nb <= 3'd1; m_rep <= 5'd1; m_ndb <= 10'd1; m_nw <= 8'd54; p_alt <= 1'b1;
    end else begin
      if (fb_release) fb_full[r_bank] <= 1'b0;
      // words for the FIFO start rule
      if (rs == R_CF && cf_i == 7'd69) begin
        enc_tbs_seen <= 32'd0; w_got <= 32'd0; w_need <= {17'd0, d_wpk}; m_started <= 32'd0;
      end else begin
        if (enc_tbs != enc_tbs_seen && enc_busy) begin
          enc_tbs_seen <= enc_tbs_seen + 1'b1; w_got <= w_got + {27'd0, d_nwtb};
        end
        if (pk_start) w_need <= w_need + {17'd0, d_wpk};
      end
      // packet start
      if (pk_start) begin
        m_mod <= r_mod; m_rsym <= r_rsym; m_sigc <= c_rsig; m_plen <= r_plen; m_port <= r_port;
        m_ucf <= r_ucf; m_nb <= d_nb; m_rep <= r_rep; m_ndb <= d_ndb;
        m_nw <= (r_cbs == 2'd0) ? 8'd54 : (r_cbs == 2'd1) ? 8'd108 : 8'd162;
        m_x2dat <= d_x2dat; m_x2id1 <= d_x2id1; m_x2id2 <= d_x2id2; m_x2cfid <= d_x2cfid;
        dq[dq_wp[1:0]] <= {r_ucf, r_upre, r_plen, hw3[63:48], d_gpl, d_gcf, d_gpre};
        dq_wp <= dq_wp + 1'b1;
        m_started <= m_started + 1'b1;
        m_pk <= 1'b1; m_cf <= r_ucf; m_sym <= 7'd0; m_rcnt <= 4'd0; m_k <= 10'd0; m_run <= 1'b1;
        cfr <= cf_int; cf_j <= 12'd0; gc1 <= g_x1; gc2 <= d_x2cfdat;
        gp1 <= g_x1; gp2 <= d_x2dat;
        s_tb <= 10'd0; s_copy <= 5'd0; s_w <= 8'd0; s_off <= 4'd0;
        pg1 <= g_x1; pg2 <= r_ucf ? d_x2cfid : r_port ? d_x2id2 : d_x2id1; p_alt <= 1'b1;
      end
      // stage A: issue k
      if (a_issue) begin
        b_v <= 1'b1; b_k <= m_k; b_ref <= m_isref; b_cf <= m_cf;
        b_lastsym <= (m_sym == m_nsym - 1'b1);
        if (m_k == 10'd840) m_run <= 1'b0;
        m_k <= m_k + 1'b1;
      end else if (b_go) begin
        b_v <= 1'b0;
      end
      // bit sources
      if (src_take && m_cf) begin
        cf_j <= cf_j + 1'b1; cfr <= {cfr[0], cfr[139:1]};
        {gc1, gc2} <= gstep(gc1, gc2, 3'd1);
      end
      if (pl_take) begin
        {gp1, gp2} <= gstep(gp1, gp2, m_nb);
        if (!s_zero) begin
          if (s_off + {1'b0, m_nb} == 4'd12) begin
            s_off <= 4'd0;
            if (s_w == m_nw - 1'b1) begin
              s_w <= 8'd0;
              if (s_copy == m_rep) begin s_copy <= 5'd0; s_bank <= ~s_bank; s_tb <= s_tb + 1'b1; end
              else s_copy <= s_copy + 1'b1;
            end else s_w <= s_w + 1'b1;
          end else s_off <= s_off + {1'b0, m_nb};
        end
      end
      // stage B: write the level code
      if (b_go) begin
        fbuf[{m_wb, b_k}] <= {li, lq};
        if (b_pilot) begin {pg1, pg2} <= gstep(pg1, pg2, 3'd1); p_alt <= ~p_alt; end
        if (b_k == 10'd840) begin                    // symbol complete
          fb_full[m_wb] <= 1'b1;
          fb_cf[m_wb] <= b_cf;
          fb_last[m_wb] <= !b_cf && b_lastsym;
          fb_mod[m_wb] <= mod_m;
          m_wb <= ~m_wb;
          m_k <= 10'd0;
          pg1 <= g_x1; p_alt <= 1'b1;
          if (b_lastsym && !m_cf) begin
            m_pk <= 1'b0;                            // packet mapped
            pg2 <= m_x2id1;
          end else if (b_lastsym) begin              // control field done: payload next
            m_cf <= 1'b0; m_sym <= 7'd0; m_rcnt <= 4'd0; m_run <= 1'b1;
            pg2 <= m_port ? m_x2id2 : m_x2id1;
          end else begin
            m_sym <= m_sym + 1'b1;
            m_rcnt <= (m_rcnt + 1'b1 == m_rsym) ? 4'd0 : m_rcnt + 1'b1;
            m_run <= 1'b1;
            pg2 <= m_cf ? m_x2cfid : m_port ? m_x2id2 : m_x2id1;
          end
          if (!a_issue) b_v <= 1'b0;
        end
      end
    end
  end

  // feeder + IFFT (global clock enable ce: one bin per ce)
  reg        f_act;                           // inside a symbol
  reg [10:0] f_bin;
  reg        f_real;
  reg [15:0] pend_real;                       // real symbols issued, output not yet written
  wire       start_real = fb_full[r_bank];
  wire       start_dummy = (pend_real != 16'd0);
  wire       out_ok;
  wire       issue_ok = f_act || start_real || start_dummy;
  wire       ce = issue_ok && out_ok;
  wire [10:0] bin = f_act ? f_bin : 11'd0;
  wire       real_now = f_act ? f_real : start_real;
  wire       bin_used = (bin <= 11'd420) || (bin >= 11'd1628);
  wire [9:0] kk = (bin <= 11'd420) ? bin[9:0] + 10'd420 : (bin - 11'd1628);
  assign fb_release = ce && real_now && (bin == 11'd2047);

  // tag FIFO: one entry per issued symbol {real, cf, last}
  reg [2:0]  tq [0:7];
  reg [3:0]  tq_wp, tq_rp;
  wire [2:0] tq_head = tq[tq_rp[2:0]];

  // F1: frequency-buffer read; F2: level * AMP
  reg  [7:0] f1_code;
  reg        f1_zero;
  reg [13:0] f1_amp;
  reg signed [17:0] f2_r, f2_i;
  wire [1:0] cur_mod = fb_mod[r_bank];
  wire       cur_cf = fb_cf[r_bank];
  wire [13:0] amp_now = (cur_cf || cur_mod == 2'd0) ? 14'd8192 : (cur_mod == 2'd1) ? 14'd5793 :
                        (cur_mod == 2'd2) ? 14'd2591 : 14'd1264;
  reg [13:0] f_amp;

  always @(posedge clk) begin
    if (!rst_n) begin
      f_act <= 1'b0; f_bin <= 11'd0; f_real <= 1'b0; r_bank <= 1'b0;
      tq_wp <= 4'd0;
      f1_code <= 8'd0; f1_zero <= 1'b1; f1_amp <= 14'd0; f2_r <= 18'sd0; f2_i <= 18'sd0;
      f_amp <= 14'd0;
    end else if (ce) begin
      if (!f_act) begin                         // bin 0 of a new symbol
        f_act <= 1'b1; f_real <= start_real;
        f_amp <= amp_now;
        tq[tq_wp[2:0]] <= {start_real, start_real && cur_cf, start_real && fb_last[r_bank]};
        tq_wp <= tq_wp + 1'b1;
      end
      f_bin <= bin + 1'b1;
      if (bin == 11'd2047) begin
        f_act <= 1'b0;
        if (real_now) r_bank <= ~r_bank;
      end
      f1_code <= fbuf[{r_bank, kk}];
      f1_zero <= !real_now || !bin_used;
      f1_amp  <= f_act ? f_amp : amp_now;
      f2_r <= f1_zero ? 18'sd0 : $signed(f1_code[7:4]) * $signed({1'b0, f1_amp});
      f2_i <= f1_zero ? 18'sd0 : $signed(f1_code[3:0]) * $signed({1'b0, f1_amp});
    end
  end

  wire signed [17:0] y_r, y_i;
  ptx_ifft #(.IN_OFFSET(2)) ifft (.clk(clk), .rst_n(rst_n), .ce(ce), .xr(f2_r), .xi(f2_i), .yr(y_r), .yi(y_i));

  // ------------------------------------------------------------------------------------------
  // output banks: two x 2048 x 36 bit, written at bitrev(slot)
  reg [11:0] nce;                             // ce count, saturates at LATF
  wire       o_valid = (nce == LATF[11:0]);
  reg [10:0] o_slot;
  reg        o_wb;                            // bank being written
  reg [1:0]  ob_full;
  reg [1:0]  ob_cf, ob_last;
  reg [35:0] obuf [0:4095];
  wire       head_real = tq_head[2];
  assign out_ok = !(o_valid && (o_slot == 11'd0) && head_real && ob_full[o_wb]);
  wire [10:0] o_rev = {o_slot[0], o_slot[1], o_slot[2], o_slot[3], o_slot[4], o_slot[5], o_slot[6],
                       o_slot[7], o_slot[8], o_slot[9], o_slot[10]};
  wire       ob_release;                      // player finished bank p_rb
  reg        p_rb;

  always @(posedge clk) begin
    if (!rst_n) begin
      nce <= 12'd0; o_slot <= 11'd0; o_wb <= 1'b0; ob_full <= 2'b00; tq_rp <= 4'd0;
      pend_real <= 16'd0;
    end else begin
      if (ob_release) ob_full[p_rb] <= 1'b0;
      // pending real symbols: +1 at the start of a real symbol, -1 when its output is complete
      pend_real <= pend_real + ((ce && !f_act && start_real) ? 16'd1 : 16'd0)
                             - ((ce && o_valid && o_slot == 11'd2047 && head_real) ? 16'd1 : 16'd0);
      if (ce) begin
        if (!o_valid) nce <= nce + 1'b1;
        if (o_valid) begin
          if (head_real) obuf[{o_wb, o_rev}] <= {y_r, y_i};
          o_slot <= o_slot + 1'b1;
          if (o_slot == 11'd2047) begin
            tq_rp <= tq_rp + 1'b1;
            if (head_real) begin
              ob_full[o_wb] <= 1'b1;
              ob_cf[o_wb] <= tq_head[1];
              ob_last[o_wb] <= tq_head[0];
              o_wb <= ~o_wb;
            end
          end
        end
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // player: preamble, symbols with cyclic prefix, gap. Pipeline P0 (address) -> P1 (ROM/RAM
  // data) -> P2 (product) -> output register, all advancing with adv = !m_valid || m_ready.
  localparam P_IDLE = 2'd0, P_PRE = 2'd1, P_SYM = 2'd2, P_GAP = 2'd3;
  reg [1:0]  ps;
  reg [72:0] pd;                              // current descriptor
  reg [13:0] p_n;                             // sample within the part
  reg [6:0]  p_sym;                           // symbol within the packet (0 .. 3 + PacketLength)
  reg [19:0] p_gap;
  wire       adv = !m_valid || m_ready;
  wire [6:0] pd_plen = pd[70:64];
  wire       pd_pre  = pd[71];
  wire       pd_cf   = pd[72];
  wire [6:0] pd_last = pd_plen - (pd_cf ? 7'd0 : 7'd4) + 7'd3;   // index of the last symbol

  // P0 decisions
  reg        q_v;                             // P0 produces a sample this cycle
  reg        q_pre, q_zero;
  reg [11:0] q_rom;
  reg [11:0] q_ram;
  reg [15:0] q_gain;
  reg        q_act;
  reg        q_release;
  wire [11:0] sym_idx = (p_n < 14'd232) ? (12'd1816 + p_n[11:0]) : (p_n[11:0] - 12'd232);
  always @* begin
    q_v = 1'b0; q_pre = 1'b0; q_zero = 1'b0; q_rom = 12'd0; q_ram = 12'd0; q_gain = 16'd0;
    q_act = 1'b1; q_release = 1'b0;
    case (ps)
      P_PRE: begin
        q_v = 1'b1; q_pre = 1'b1; q_gain = pd[15:0];
        if (p_n < 14'd200)        q_rom = 12'd64 + p_n[11:0];                       // AGC burst
        else if (p_n < 14'd10200) q_rom = {6'd0, (p_n[5:0] - 6'd8)};                // Preamble A
        else if (p_n < 14'd10432) q_rom = 12'd64 + 12'd1816 + (p_n - 14'd10200);           // B prefix
        else                      q_rom = 12'd64 + (p_n - 14'd10432);                      // B body
      end
      P_SYM: begin
        q_v = ob_full[p_rb];
        q_ram = {p_rb, sym_idx[10:0]};
        q_gain = ob_cf[p_rb] ? pd[31:16] : pd[47:32];
        q_release = q_v && (p_n == 14'd2279);
      end
      P_GAP: begin q_v = (p_gap != 20'd0); q_zero = 1'b1; q_act = 1'b0; end
      default: ;
    endcase
  end
  assign ob_release = adv && q_release;

  // Preamble A index: p_n - 200 = 0 .. 9999, ROM address (p_n - 200) mod 64 = (p_n - 8) mod 64
  wire signed [15:0] rom_r, rom_i;
  ptx_pre_rom pre (.clk(clk), .ce(adv), .addr(q_rom), .re(rom_r), .im(rom_i));
  reg  [35:0] ram_q;
  reg         p1_v, p1_pre, p1_zero, p1_act;
  reg  [15:0] p1_gain;
  reg         p2_v, p2_act;
  reg signed [34:0] p2_r, p2_i;
  wire signed [17:0] s_r = p1_pre ? {{2{rom_r[15]}}, rom_r} : ram_q[35:18];
  wire signed [17:0] s_i = p1_pre ? {{2{rom_i[15]}}, rom_i} : ram_q[17:0];
  wire signed [34:0] t_r = (p2_r + 35'sd32768) >>> 16;
  wire signed [34:0] t_i = (p2_i + 35'sd32768) >>> 16;

  always @(posedge clk) begin
    if (!rst_n) begin
      ps <= P_IDLE; p_n <= 14'd0; p_sym <= 7'd0; p_gap <= 20'd0; p_rb <= 1'b0; dq_rp <= 3'd0;
      p1_v <= 1'b0; p2_v <= 1'b0; m_valid <= 1'b0; m_i <= 12'd0; m_q <= 12'd0; m_active <= 1'b0;
      st_packets <= 32'd0; st_bubbles <= 32'd0; pd <= 73'd0;
    end else if (adv) begin
      // P0: sequencing
      case (ps)
        P_IDLE: if (!dq_empty) begin
          pd <= dq[dq_rp[1:0]]; dq_rp <= dq_rp + 1'b1; ps <= dq[dq_rp[1:0]][71] ? P_PRE : P_SYM;
          p_n <= 14'd0; p_sym <= 7'd0;
        end
        P_PRE: begin
          if (p_n == 14'd12479) begin p_n <= 14'd0; ps <= P_SYM; end
          else p_n <= p_n + 1'b1;
        end
        P_SYM: if (q_v) begin
          if (p_n == 14'd2279) begin
            p_n <= 14'd0; p_rb <= ~p_rb;
            if (p_sym == pd_last) begin                        // last symbol of the packet
              ps <= P_GAP; p_gap <= {pd[63:48], 4'd0};
              st_packets <= st_packets + 1'b1;
            end
            p_sym <= p_sym + 1'b1;
          end else p_n <= p_n + 1'b1;
        end else st_bubbles <= st_bubbles + 1'b1;
        P_GAP: if (p_gap == 20'd0) ps <= P_IDLE; else p_gap <= p_gap - 1'b1;
        default: ps <= P_IDLE;
      endcase
      if (q_v && ps == P_SYM) ram_q <= obuf[q_ram];
      p1_v <= q_v; p1_pre <= q_pre; p1_zero <= q_zero; p1_gain <= q_gain; p1_act <= q_act;
      // P2: product
      p2_v <= p1_v; p2_act <= p1_act;
      p2_r <= p1_zero ? 35'sd0 : s_r * $signed({1'b0, p1_gain});
      p2_i <= p1_zero ? 35'sd0 : s_i * $signed({1'b0, p1_gain});
      // output: rnd, sat12
      m_valid  <= p2_v;
      m_active <= p2_act;
      m_i <= (t_r > 35'sd2047) ? 12'h7FF : (t_r < -35'sd2048) ? 12'h800 : t_r[11:0];
      m_q <= (t_i > 35'sd2047) ? 12'h7FF : (t_i < -35'sd2048) ? 12'h800 : t_i[11:0];
    end
  end
  assign st_busy = (rs != R_H0) || enc_busy || m_pk || (ps != P_IDLE) || f_act || (pend_real != 16'd0) ||
                   (fb_full != 2'b00) || !dq_empty;
endmodule
