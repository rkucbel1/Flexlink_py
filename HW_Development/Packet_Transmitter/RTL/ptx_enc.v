// ptx_enc.v - transport blocks -> interleaved LDPC code words, bit-true to
// ../PC/ptx_reference.payload_bits() (CRC24, 802.11n LDPC, interleaver).
//
// For each block: LOAD takes TransportBlockSize bits (one per clock) from the bit buffer and appends
// the CRC24 (Gen24, MSB first) -> K bits in 24 - m blocks of Z bits; then asks the bit buffer to drop
// the rest of the block's last 64-bit word. LAM: lambda_r ^= rot(block_c, P[r][c]) for every
// nonzero circulant of the message columns (one per clock), and their sum. P0: p0 = rot(sum, hroll).
// REC: p_(r+1) = lambda_r ^ [rot(p0, h_r)] ^ p_r (dual diagonal). IL: out[i] = cw[il[i]], one bit
// per clock, packed 12 per word into one of two code-word banks (N / 12 = 2Z words).
// rot(v, s)[z] = v[(z + s) mod Z]. Worst case per block (1944, 5/6) about 3700 clocks, against
// about 4400 clocks of 64QAM symbols that two copies of it fill.
// The consumer (ptx_core) reads the banks asynchronously and releases a bank after the last copy.
`timescale 1ns / 1ps
module ptx_enc (
  input  wire        clk,
  input  wire        rst_n,
  // job: n_pk packets of ndb blocks
  input  wire        start,
  input  wire [1:0]  cbs_c,                // 0 648, 1 1296, 2 1944
  input  wire [1:0]  rate_c,               // 0 1/2, 1 2/3, 2 3/4, 3 5/6
  input  wire [9:0]  ndb,
  input  wire [31:0] n_pk,
  output wire        busy,
  output reg  [31:0] tbs_started,          // blocks started since start (FIFO start rule)
  // bit buffer
  input  wire        bit_in,
  input  wire        bit_avail,
  output wire        bit_take,
  output wire        align,                // drop the rest of the current 64-bit word
  // code-word banks
  input  wire [8:0]  rd_addr,              // {bank, word}
  output wire [11:0] rd_data,
  output reg  [1:0]  cw_full,
  input  wire        rel,                  // consumer done with bank rel_bank
  input  wire        rel_bank
);
  localparam E_IDLE = 3'd0, E_LOAD = 3'd1, E_LAM = 3'd2, E_P0 = 3'd3, E_REC = 3'd4, E_IL = 3'd5;

  reg [2:0]  st;
  reg [1:0]  zc;                           // Z code = CBS code
  reg [1:0]  rc;
  reg [6:0]  z;                            // 27, 54, 81
  reg [3:0]  m;                            // 12, 8, 6, 4
  reg [4:0]  kb;
  reg [10:0] kbits, tbs;                   // K, TransportBlockSize
  reg [10:0] nbits;                        // N
  reg [9:0]  ndb_r, tb_in_pk;
  reg [31:0] npk_r, pk;
  assign busy = (st != E_IDLE);

  wire [3:0]  m_of  = (rate_c == 2'd0) ? 4'd12 : (rate_c == 2'd1) ? 4'd8 : (rate_c == 2'd2) ? 4'd6 : 4'd4;
  wire [6:0]  z_of  = (cbs_c == 2'd0) ? 7'd27 : (cbs_c == 2'd1) ? 7'd54 : 7'd81;

  // memories
  reg [80:0] blk [0:23];
  reg [80:0] lam [0:11];
  reg [11:0] lam_v;
  reg [11:0] cwm [0:511];
  assign rd_data = cwm[rd_addr];

  // ROMs
  wire [9:0] h_base; wire [7:0] h_nnz; wire [6:0] h0s, hxs, hroll; wire [3:0] hxr;
  ptx_ldpc_hdr hrom (.addr({zc, rc}), .base(h_base), .nnz(h_nnz), .h0s(h0s), .hxr(hxr), .hxs(hxs), .hroll(hroll));
  reg  [9:0]  nz_a;
  wire [15:0] nz_e;
  ptx_ldpc_nz nrom (.clk(clk), .addr(nz_a), .e(nz_e));
  reg  [10:0] il_base;
  reg  [10:0] ii;                                      // next index to issue
  wire [11:0] il_a = {1'b0, il_base} + {1'b0, ii};
  wire [11:0] il_e;                                    // entry of index io (one clock later)
  ptx_il_rom irom (.clk(clk), .addr(il_a), .e(il_e));

  // shared rotator
  reg  [80:0] rv;
  reg  [6:0]  rs;
  reg  [161:0] vv;
  reg  [80:0]  rot_out;
  always @* begin
    case (zc)
      2'd0: vv = {108'd0, rv[26:0], rv[26:0]};
      2'd1: vv = {54'd0, rv[53:0], rv[53:0]};
      default: vv = {rv, rv};
    endcase
    vv = vv >> rs;
    rot_out = vv[80:0] & ((zc == 2'd0) ? {54'd0, {27{1'b1}}} : (zc == 2'd1) ? {27'd0, {54{1'b1}}} : {81{1'b1}});
  end

  // LOAD state
  reg [10:0] p;
  reg [4:0]  bi;
  reg [6:0]  off;
  reg [80:0] stg;
  reg [23:0] crc;
  wire       in_tb = (p < tbs);
  wire       ld_go = (st == E_LOAD) && (!in_tb || bit_avail);
  wire       ld_bit = in_tb ? bit_in : crc[23];
  assign bit_take = ld_go && in_tb;
  assign align = (st == E_LOAD) && (p == tbs);          // first CRC bit: the block's bits are all in
  wire [80:0] stg_n = stg | ({80'd0, ld_bit} << off);

  // LAM / P0 / REC
  reg [7:0]  e_cnt;
  reg        nz_v;                                     // nz_e valid
  reg [80:0] tot, p0r, prev;
  reg [3:0]  r;
  wire [3:0] e_row = nz_e[15:12];
  wire [4:0] e_col = nz_e[11:7];
  wire [6:0] e_sh  = nz_e[6:0];
  wire [80:0] lam_r = lam_v[r] ? lam[r] : 81'd0;
  // LAM is two stages (timing): A reads blk[e_col] of the ROM entry into lr, B rotates lr and XORs
  // it into lam[lrow] and tot. blk is not written during LAM; lam is read and written in B only.
  reg [80:0] lr;
  reg [3:0]  lrow;
  reg [6:0]  lsh;
  reg        lv;                                       // lr valid (stage B works)
  reg        a_done;                                   // all entries read
  reg [7:0]  b_cnt;
  wire        use_h = (r == 4'd0) || (r == hxr);

  // IL
  reg        ebank;
  reg        il_go;                                    // bank free, issuing
  reg        il_v;                                     // il_e valid
  reg [10:0] io;                                       // index of il_e
  reg [3:0]  j12;
  reg [10:0] w12;
  reg [7:0]  wa;
  wire       il_bit = blk[il_e[11:7]][il_e[6:0]];
  reg        il_v2, il_b2;                             // il_bit registered (timing), its index io2
  reg [10:0] io2;

  always @* begin
    rv = 81'd0; rs = 7'd0;
    case (st)
      E_LAM: begin rv = lr; rs = lsh; end
      E_P0:  begin rv = tot; rs = hroll; end
      E_REC: begin rv = p0r; rs = (r == 4'd0) ? h0s : hxs; end
      default: ;
    endcase
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      st <= E_IDLE; cw_full <= 2'b00; ebank <= 1'b0; tbs_started <= 32'd0;
      zc <= 2'd0; rc <= 2'd0; z <= 7'd27; m <= 4'd12; kb <= 5'd12; kbits <= 11'd324; tbs <= 11'd300;
      nbits <= 11'd648; ndb_r <= 10'd1; npk_r <= 32'd0; pk <= 32'd0; tb_in_pk <= 10'd0;
      p <= 11'd0; bi <= 5'd0; off <= 7'd0; stg <= 81'd0; crc <= 24'd0; lam_v <= 12'd0;
      e_cnt <= 8'd0; nz_v <= 1'b0; nz_a <= 10'd0; il_v <= 1'b0; il_go <= 1'b0; ii <= 11'd0;
      lv <= 1'b0; a_done <= 1'b0; b_cnt <= 8'd0; il_v2 <= 1'b0;
      il_base <= 11'd0;
    end else begin
      if (rel) cw_full[rel_bank] <= 1'b0;
      case (st)
        E_IDLE: if (start) begin
          zc <= cbs_c; rc <= rate_c; z <= z_of; m <= m_of; kb <= 5'd24 - {1'b0, m_of};
          kbits <= (11'd24 - {7'd0, m_of}) * {4'd0, z_of};
          tbs   <= (11'd24 - {7'd0, m_of}) * {4'd0, z_of} - 11'd24;
          nbits <= 11'd24 * {4'd0, z_of};
          il_base <= (cbs_c == 2'd0) ? 11'd0 : (cbs_c == 2'd1) ? 11'd648 : 11'd1944;
          ndb_r <= ndb; npk_r <= n_pk; pk <= 32'd0; tb_in_pk <= 10'd0; tbs_started <= 32'd0;
          if (n_pk != 32'd0 && ndb != 10'd0) begin
            st <= E_LOAD; p <= 11'd0; bi <= 5'd0; off <= 7'd0; stg <= 81'd0; crc <= 24'd0; lam_v <= 12'd0;
            tbs_started <= 32'd1;
          end
        end
        E_LOAD: if (ld_go) begin
          if (in_tb) begin
            crc <= {crc[22:0], 1'b0} ^ ((crc[23] ^ bit_in) ? 24'h864CFB : 24'd0);
          end else begin
            crc <= {crc[22:0], 1'b0};
          end
          if (off == z - 1'b1) begin
            blk[bi] <= stg_n; bi <= bi + 1'b1; off <= 7'd0; stg <= 81'd0;
          end else begin
            stg <= stg_n; off <= off + 1'b1;
          end
          p <= p + 1'b1;
          if (p == kbits - 1'b1) begin
            st <= E_LAM; e_cnt <= 8'd0; nz_a <= h_base; nz_v <= 1'b0; tot <= 81'd0;
            lv <= 1'b0; a_done <= 1'b0; b_cnt <= 8'd0;
          end
        end
        E_LAM: begin
          // issue the next ROM address, process the entry that arrived
          // stage A: issue the next ROM address, read the block of the entry that arrived
          lv <= 1'b0;
          if (!nz_v) begin
            nz_v <= 1'b1; nz_a <= nz_a + 1'b1;
          end else if (!a_done) begin
            lr <= blk[e_col]; lrow <= e_row; lsh <= e_sh; lv <= 1'b1;
            nz_a <= nz_a + 1'b1;
            e_cnt <= e_cnt + 1'b1;
            if (e_cnt == h_nnz - 1'b1) a_done <= 1'b1;
          end
          // stage B
          if (lv) begin
            lam[lrow] <= (lam_v[lrow] ? lam[lrow] : 81'd0) ^ rot_out;
            lam_v[lrow] <= 1'b1;
            tot <= tot ^ rot_out;
            b_cnt <= b_cnt + 1'b1;
            if (b_cnt == h_nnz - 1'b1) begin st <= E_P0; nz_v <= 1'b0; lv <= 1'b0; end
          end
        end
        E_P0: begin
          blk[kb] <= rot_out; p0r <= rot_out; prev <= 81'd0; r <= 4'd0; st <= E_REC;
        end
        E_REC: begin
          blk[kb + 1'b1 + r] <= lam_r ^ (use_h ? rot_out : 81'd0) ^ prev;
          prev <= lam_r ^ (use_h ? rot_out : 81'd0) ^ prev;
          r <= r + 1'b1;
          if (r == m - 2'd2) begin
            st <= E_IL; ii <= 11'd0; il_v <= 1'b0; il_v2 <= 1'b0; il_go <= 1'b0; j12 <= 4'd0; wa <= 8'd0;
          end
        end
        E_IL: begin
          if (!il_go) begin
            if (!cw_full[ebank]) il_go <= 1'b1;
          end else begin
            // issue index ii (the ROM registers il_a at this edge)
            if (ii != nbits) begin ii <= ii + 1'b1; io <= ii; il_v <= 1'b1; end
            else il_v <= 1'b0;
            // read the bit of index io
            il_v2 <= il_v; il_b2 <= il_bit; io2 <= io;
            // collect the bit of index io2
            if (il_v2) begin
              if (j12 == 4'd11) begin
                cwm[{ebank, wa}] <= {il_b2, w12};
                wa <= wa + 1'b1; j12 <= 4'd0;
              end else begin
                w12[j12] <= il_b2; j12 <= j12 + 1'b1;
              end
              if (io2 == nbits - 1'b1) begin                   // code word complete
                cw_full[ebank] <= 1'b1; ebank <= ~ebank; il_v <= 1'b0; il_v2 <= 1'b0; il_go <= 1'b0;
                if (tb_in_pk == ndb_r - 1'b1) begin
                  tb_in_pk <= 10'd0;
                  if (pk == npk_r - 1'b1) st <= E_IDLE;
                  else begin pk <= pk + 1'b1; st <= E_LOAD; end
                end else begin
                  tb_in_pk <= tb_in_pk + 1'b1; st <= E_LOAD;
                end
                if (!(tb_in_pk == ndb_r - 1'b1 && pk == npk_r - 1'b1)) begin
                  p <= 11'd0; bi <= 5'd0; off <= 7'd0; stg <= 81'd0; crc <= 24'd0; lam_v <= 12'd0;
                  tbs_started <= tbs_started + 1'b1;
                end
              end
            end
          end
        end
        default: st <= E_IDLE;
      endcase
    end
  end
endmodule
