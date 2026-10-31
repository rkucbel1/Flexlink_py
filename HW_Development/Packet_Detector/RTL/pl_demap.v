// pl_demap.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Payload demapper (step 12c, Docs/Specification_PayloadReceiver.md section 4), bit-true to
// pl_reference.soft_bits (step 8): per Data RE z = Y' conj(H), e = |H|^2, per bit
// N = min over the bit-1 levels of D(m) - min over the bit-0 levels, D(m) = m^2 e - C1 m z_axis,
// then Lf = satm(rnd(N m, sh), 12) with the packet's scale (m, sh) from sumN and n0.
//
// Input: the packet record of pl_chest (pk_*, before the packet's first H) and its H stream
// (h_*: every tone of every symbol in order, with Y'). The H stream is credit controlled:
// h_take returns one credit per entry leaving the 16-entry input FIFO (pl_chest issues an H only
// with a credit). Only Data REs (pl_dmask_rom) give LLRs.
//
// The min over a level set is the level nearest to the parabola's vertex C1 z / (2 e), so it is
// picked by the threshold bits lt[j] = (A z < j e), j = -3 .. 3 (A = outer level; C1 = 4 A):
// exact, also for e = 0. N = D(m1) - D(m0) of the two picked levels.
//
// Rate: one RE per ceil(bits / 2) clocks (BPSK, QPSK 1; 16QAM 2; 64QAM 3), two LLRs per clock
// out; BPSK LLRs are paired. Output: aligned LLR pairs (sequence index 2p, 2p + 1) in a 32-entry
// FIFO for pl_comb (lf_*), lf_sop on the packet's first pair; the packet's settings for pl_comb
// on cfg_* (valid from the packet's first pair on).
//
// Per packet, at pk_valid: (m, sh) by a sequential search and division (pl_reference.llr_scale):
// num = n0 << 5, den = C2 sumN, sh = the smallest with (num << sh) // den >= 2^15,
// m = min((num << sh) // den, 2^16 - 1); den = 0: m = 2^16 - 1, sh = 0.
`timescale 1ns / 1ps
`default_nettype none

module pl_demap (
    input  wire               clk,
    input  wire               rst_n,
    // packet record (pl_chest)
    input  wire               pk_valid,
    input  wire [31:0]        pk_t,
    input  wire [53:0]        pk_msg,
    input  wire [9:0]         pk_n0,
    input  wire [47:0]        pk_sumn,
    // H stream (pl_chest), credit controlled
    input  wire               h_valid,
    input  wire [6:0]         h_sym,
    input  wire [9:0]         h_col,
    input  wire               h_ref,          // the symbol is a reference row
    input  wire signed [15:0] h_re,
    input  wire signed [15:0] h_im,
    input  wire signed [15:0] y_re,
    input  wire signed [15:0] y_im,
    output reg                h_take,
    // LLR pairs (to pl_comb)
    output wire               lf_valid,
    output wire               lf_sop,
    output wire signed [11:0] lf_l0,
    output wire signed [11:0] lf_l1,
    input  wire               lf_pop,
    // settings of the packet being demapped (pl_comb takes them at lf_sop)
    output reg  [1:0]         cfg_ncode,      // 648 / 1296 / 1944
    output reg  [1:0]         cfg_rate,       // 1/2, 2/3, 3/4, 5/6
    output reg  [4:0]         cfg_rep,        // copies - 1
    output reg  [9:0]         cfg_ndb,
    output reg  [7:0]         cfg_cinit,
    output reg  [31:0]        cfg_t,
    output wire               busy
);
    // ------------------------------------------------------------------ packet settings
    // next packet (from pk_valid), current packet (from its first RE on)
    reg  [1:0]  n_md, n_sc, n_nc, n_rt;
    reg  [4:0]  n_rep;
    reg  [9:0]  n_ndb;
    reg  [7:0]  n_ci;
    reg  [31:0] n_t;
    reg  [15:0] n_m;
    reg  [5:0]  n_sh;
    reg         n_ok;                          // next settings complete (scale done)
    reg  [1:0]  c_md, c_sc;
    reg  [15:0] c_m;
    reg  [5:0]  c_sh;
    function [1:0] fld2;                       // a 2-bit field, MSB first
        input [53:0] msg;
        input integer pos;
        fld2 = {msg[pos], msg[pos + 1]};
    endfunction

    // scale: sequential
    localparam [2:0] SC_IDLE = 0, SC_DEN = 1, SC_SH = 2, SC_DIV = 3, SC_DONE = 4;
    reg  [2:0]  sc;
    reg  [47:0] s_sumn;
    reg  [9:0]  s_n0;
    reg  [55:0] den;
    reg  [95:0] nn, rr;
    reg  [6:0]  shc;
    reg  [4:0]  qi;
    reg  [15:0] q;
    wire [95:0] den15 = {25'd0, den, 15'd0};
    wire [95:0] den16 = {24'd0, den, 16'd0};
    reg  [95:0] den_q;                         // den << qi (shifted right by one per step)

    always @(posedge clk) begin
        if (!rst_n) begin
            sc <= SC_IDLE; n_ok <= 1'b0;
        end else begin
            if (pk_valid) begin
                n_md <= fld2(pk_msg, 7); n_sc <= fld2(pk_msg, 2); n_nc <= fld2(pk_msg, 10); n_rt <= fld2(pk_msg, 12);
                case ({pk_msg[14], pk_msg[15], pk_msg[16], pk_msg[17]})
                    4'd0: n_rep <= 5'd1;  4'd1: n_rep <= 5'd3;  4'd2: n_rep <= 5'd5;  4'd3: n_rep <= 5'd7;
                    4'd4: n_rep <= 5'd11; 4'd5: n_rep <= 5'd15; 4'd6: n_rep <= 5'd19; 4'd7: n_rep <= 5'd23;
                    default: n_rep <= 5'd31;
                endcase
                n_ndb <= {pk_msg[18], pk_msg[19], pk_msg[20], pk_msg[21], pk_msg[22], pk_msg[23], pk_msg[24],
                          pk_msg[25], pk_msg[26], pk_msg[27]};
                n_ci <= {pk_msg[43], pk_msg[44], pk_msg[45], pk_msg[46], pk_msg[47], pk_msg[48], pk_msg[49], pk_msg[50]};
                n_t <= pk_t;
                s_sumn <= pk_sumn; s_n0 <= pk_n0;
                n_ok <= 1'b0;
                sc <= SC_DEN;
            end else case (sc)
                SC_DEN: begin
                    // C2 = 1, 4, 36, 196 (BPSK, QPSK, 16QAM, 64QAM)
                    case (n_md)
                        2'd0: den <= {8'd0, s_sumn};
                        2'd1: den <= {6'd0, s_sumn, 2'd0};
                        2'd2: den <= {3'd0, s_sumn, 5'd0} + {6'd0, s_sumn, 2'd0};
                        default: den <= {1'b0, s_sumn, 7'd0} + {2'd0, s_sumn, 6'd0} + {6'd0, s_sumn, 2'd0};
                    endcase
                    nn <= {81'd0, s_n0, 5'd0};
                    shc <= 7'd0;
                    sc <= SC_SH;
                end
                SC_SH: begin
                    if (den == 56'd0) begin n_m <= 16'hFFFF; n_sh <= 6'd0; sc <= SC_DONE; end
                    else if (nn >= den15) begin
                        n_sh <= (shc > 7'd63) ? 6'd63 : shc[5:0];
                        if (nn >= den16) begin n_m <= 16'hFFFF; sc <= SC_DONE; end
                        else begin rr <= nn; q <= 16'd0; qi <= 5'd15; den_q <= den15; sc <= SC_DIV; end
                    end else begin
                        nn <= {nn[94:0], 1'b0}; shc <= shc + 7'd1;
                    end
                end
                SC_DIV: begin
                    // restoring division, one quotient bit per clock (15 .. 0)
                    if (rr >= den_q) rr <= rr - den_q;
                    q <= q | ((rr >= den_q) ? (16'd1 << qi[3:0]) : 16'd0);
                    if (qi == 5'd0) begin
                        n_m <= q | ((rr >= den_q) ? 16'd1 : 16'd0);
                        sc <= SC_DONE;
                    end
                    qi <= qi - 5'd1;
                    den_q <= {1'b0, den_q[95:1]};
                end
                SC_DONE: begin n_ok <= 1'b1; sc <= SC_IDLE; end
                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------ input FIFO (16)
    // {first, ref, col, yr, yi, hr, hi}
    (* ram_style = "distributed" *) reg [75:0] fm [0:15];
    reg  [4:0]  f_wr, f_rd;
    wire [4:0]  f_n = f_wr - f_rd;
    always @(posedge clk) if (h_valid) fm[f_wr[3:0]] <= {h_sym == 7'd0 && h_col == 10'd0, h_ref, h_col, y_re, y_im, h_re, h_im};
    wire [75:0] hd = fm[f_rd[3:0]];
    wire        hd_first = hd[75];
    wire        hd_ref = hd[74];
    wire [9:0]  hd_col = hd[73:64];
    wire [1:0]  use_sc = hd_first ? n_sc : c_sc;
    wire [1:0]  use_md = hd_first ? n_md : c_md;
    wire        hd_data;
    pl_dmask_rom u_mask (.addr({hd_ref ? {1'b0, use_sc} : 3'd4, hd_col}), .d(hd_data));

    // ------------------------------------------------------------------ issue
    reg  [1:0]  hold;                          // clocks until the next data RE may go
    reg  [5:0]  p_wr, p_rd;                    // pair FIFO pointers
    wire [5:0]  pf_n;                          // pair FIFO entries
    reg  [5:0]  infl;                          // LLR events (pairs / BPSK singles) in flight
    reg         pend_sop;                      // the packet's first data RE still to come
    wire [1:0]  npairs = (use_md == 2'd3) ? 2'd3 : (use_md == 2'd2) ? 2'd2 : 2'd1;
    wire        room = ({1'b0, pf_n} + {1'b0, infl}) <= 7'd28;
    wire        can_pop = (f_n != 5'd0) && (!hd_first || n_ok) && (!hd_data || (hold == 2'd0 && room));
    wire        issue = can_pop && hd_data;
    wire        out_ev;                        // an LLR event leaves the scaling (o_v)
    reg         v1;
    reg         sop1;
    reg  [1:0]  md1;
    reg  [15:0] m1;
    reg  [5:0]  sh1;
    reg  signed [15:0] yr1, yi1, hr1, hi1;

    always @(posedge clk) begin
        h_take <= 1'b0;
        v1 <= 1'b0;
        if (!rst_n) begin
            f_wr <= 5'd0; f_rd <= 5'd0; hold <= 2'd0; infl <= 6'd0; pend_sop <= 1'b0;
        end else begin
            if (h_valid) f_wr <= f_wr + 5'd1;
            if (hold != 2'd0) hold <= hold - 2'd1;
            if (can_pop) begin
                f_rd <= f_rd + 5'd1;
                h_take <= 1'b1;
                if (hd_first) begin
                    c_md <= n_md; c_sc <= n_sc; c_m <= n_m; c_sh <= n_sh;
                    cfg_ncode <= n_nc; cfg_rate <= n_rt; cfg_rep <= n_rep; cfg_ndb <= n_ndb; cfg_cinit <= n_ci; cfg_t <= n_t;
                end
                if (issue) begin
                    v1 <= 1'b1;
                    sop1 <= hd_first || pend_sop;
                    md1 <= use_md;
                    m1 <= hd_first ? n_m : c_m;
                    sh1 <= hd_first ? n_sh : c_sh;
                    yr1 <= hd[63:48]; yi1 <= hd[47:32]; hr1 <= hd[31:16]; hi1 <= hd[15:0];
                    hold <= npairs - 2'd1;
                end
                pend_sop <= hd_first ? !hd_data : (pend_sop && !hd_data);
            end
            infl <= infl + ((issue) ? {4'd0, npairs} : 6'd0) - (out_ev ? 6'd1 : 6'd0);
        end
    end

    // ------------------------------------------------------------------ compute pipeline
    // P2: products
    reg         v2, sop2;
    reg  [1:0]  md2;
    reg  [15:0] m2;
    reg  [5:0]  sh2;
    reg  signed [31:0] pa, pb, pc, pd, pe, pf;
    // P3: z, e
    reg         v3, sop3;
    reg  [1:0]  md3;
    reg  [15:0] m3;
    reg  [5:0]  sh3;
    reg  signed [32:0] zr, zi, ee;
    // P4: multiples
    reg         v4, sop4;
    reg  [1:0]  md4;
    reg  [15:0] m4;
    reg  [5:0]  sh4;
    reg  signed [39:0] az_r, az_i, cz_r, cz_i, e1, e3, e9;
    // P5: level terms, thresholds
    reg         v5, sop5;
    reg  [1:0]  md5;
    reg  [15:0] m5;
    reg  [5:0]  sh5;
    reg  signed [43:0] q1r, q3r, q5r, q7r, q1i, q3i, q5i, q7i, ea1, ea9, ea25, ea49;
    reg  [6:0]  ltr, lti;                      // lt[j + 3] = (A z < j e)
    // P6: D(m) per axis, levels -7 .. 7 (index 0 .. 7)
    reg         v6, sop6;
    reg  [1:0]  md6;
    reg  [15:0] m6;
    reg  [5:0]  sh6;
    reg  signed [43:0] dr [0:7];
    reg  signed [43:0] di [0:7];
    reg  [6:0]  ltr6, lti6;
    // P7: N per (axis, k)
    reg         v7, sop7;
    reg  [1:0]  md7;
    reg  [15:0] m7;
    reg  [5:0]  sh7;
    reg  signed [43:0] nr0, nr1, nr2, ni0, ni1, ni2;

    // level index of the nearest level in the bit-0 / bit-1 set of bit k of an axis
    function [5:0] pick;                       // {m0 index, m1 index}
        input [1:0] md;
        input [1:0] kk;
        input [6:0] lt;                        // lt[0] = j -3 .. lt[6] = j 3
        reg [2:0] a, b;
        begin
            a = 3'd3; b = 3'd4;                // -1, +1
            if (md == 2'd2) begin
                if (kk == 2'd0) begin a = lt[2] ? 3'd2 : 3'd3; b = lt[4] ? 3'd4 : 3'd5; end
                else begin a = lt[3] ? 3'd2 : 3'd5; b = lt[3] ? 3'd3 : 3'd4; end
            end else if (md == 2'd3) begin
                if (kk == 2'd0) begin
                    a = lt[0] ? 3'd0 : lt[1] ? 3'd1 : lt[2] ? 3'd2 : 3'd3;
                    b = lt[4] ? 3'd4 : lt[5] ? 3'd5 : lt[6] ? 3'd6 : 3'd7;
                end else if (kk == 2'd1) begin
                    a = lt[0] ? 3'd0 : lt[3] ? 3'd1 : lt[6] ? 3'd6 : 3'd7;
                    b = lt[2] ? 3'd2 : lt[3] ? 3'd3 : lt[4] ? 3'd4 : 3'd5;
                end else begin
                    a = lt[1] ? 3'd0 : lt[3] ? 3'd3 : lt[5] ? 3'd4 : 3'd7;
                    b = lt[1] ? 3'd1 : lt[3] ? 3'd2 : lt[5] ? 3'd5 : 3'd6;
                end
            end
            pick = {a, b};
        end
    endfunction
    wire [5:0] sr0 = pick(md6, 2'd0, ltr6), sr1 = pick(md6, 2'd1, ltr6), sr2 = pick(md6, 2'd2, ltr6);
    wire [5:0] si0 = pick(md6, 2'd0, lti6), si1 = pick(md6, 2'd1, lti6), si2 = pick(md6, 2'd2, lti6);

    always @(posedge clk) begin
        v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0; v5 <= 1'b0; v6 <= 1'b0; v7 <= 1'b0;
        if (rst_n) begin
            // P2
            v2 <= v1; sop2 <= sop1; md2 <= md1; m2 <= m1; sh2 <= sh1;
            pa <= yr1 * hr1; pb <= yi1 * hi1; pc <= yi1 * hr1; pd <= yr1 * hi1; pe <= hr1 * hr1; pf <= hi1 * hi1;
            // P3: z = Y' conj(H), e = |H|^2
            v3 <= v2; sop3 <= sop2; md3 <= md2; m3 <= m2; sh3 <= sh2;
            zr <= {pa[31], pa} + {pb[31], pb};
            zi <= {pc[31], pc} - {pd[31], pd};
            ee <= {pe[31], pe} + {pf[31], pf};
            // P4: A z, C1 z (C1 = 2, 4, 12, 28; A = 1, 1, 3, 7), e, 3e, 9e
            v4 <= v3; sop4 <= sop3; md4 <= md3; m4 <= m3; sh4 <= sh3;
            case (md3)
                2'd2: begin
                    az_r <= ({{7{zr[32]}}, zr} <<< 1) + {{7{zr[32]}}, zr};
                    az_i <= ({{7{zi[32]}}, zi} <<< 1) + {{7{zi[32]}}, zi};
                    cz_r <= ({{7{zr[32]}}, zr} <<< 3) + ({{7{zr[32]}}, zr} <<< 2);
                    cz_i <= ({{7{zi[32]}}, zi} <<< 3) + ({{7{zi[32]}}, zi} <<< 2);
                end
                2'd3: begin
                    az_r <= ({{7{zr[32]}}, zr} <<< 3) - {{7{zr[32]}}, zr};
                    az_i <= ({{7{zi[32]}}, zi} <<< 3) - {{7{zi[32]}}, zi};
                    cz_r <= ({{7{zr[32]}}, zr} <<< 5) - ({{7{zr[32]}}, zr} <<< 2);
                    cz_i <= ({{7{zi[32]}}, zi} <<< 5) - ({{7{zi[32]}}, zi} <<< 2);
                end
                default: begin
                    az_r <= {{7{zr[32]}}, zr};
                    az_i <= {{7{zi[32]}}, zi};
                    cz_r <= {{7{zr[32]}}, zr} <<< ((md3 == 2'd0) ? 1 : 2);
                    cz_i <= {{7{zi[32]}}, zi} <<< ((md3 == 2'd0) ? 1 : 2);
                end
            endcase
            e1 <= {{7{ee[32]}}, ee};
            e3 <= ({{7{ee[32]}}, ee} <<< 1) + {{7{ee[32]}}, ee};
            e9 <= ({{7{ee[32]}}, ee} <<< 3) + {{7{ee[32]}}, ee};
            // P5: m C1 z (m = 1, 3, 5, 7), m^2 e, thresholds A z < j e
            v5 <= v4; sop5 <= sop4; md5 <= md4; m5 <= m4; sh5 <= sh4;
            q1r <= {{4{cz_r[39]}}, cz_r};
            q3r <= ({{4{cz_r[39]}}, cz_r} <<< 1) + {{4{cz_r[39]}}, cz_r};
            q5r <= ({{4{cz_r[39]}}, cz_r} <<< 2) + {{4{cz_r[39]}}, cz_r};
            q7r <= ({{4{cz_r[39]}}, cz_r} <<< 3) - {{4{cz_r[39]}}, cz_r};
            q1i <= {{4{cz_i[39]}}, cz_i};
            q3i <= ({{4{cz_i[39]}}, cz_i} <<< 1) + {{4{cz_i[39]}}, cz_i};
            q5i <= ({{4{cz_i[39]}}, cz_i} <<< 2) + {{4{cz_i[39]}}, cz_i};
            q7i <= ({{4{cz_i[39]}}, cz_i} <<< 3) - {{4{cz_i[39]}}, cz_i};
            ea1 <= {{4{e1[39]}}, e1};
            ea9 <= {{4{e9[39]}}, e9};
            ea25 <= {{4{e9[39]}}, e9} + ({{4{e1[39]}}, e1} <<< 4);
            ea49 <= ({{4{e3[39]}}, e3} <<< 4) + {{4{e1[39]}}, e1};
            ltr <= {az_r < e3, az_r < (e1 <<< 1), az_r < e1, az_r[39],
                    (az_r + e1) < 0, (az_r + (e1 <<< 1)) < 0, (az_r + e3) < 0};
            lti <= {az_i < e3, az_i < (e1 <<< 1), az_i < e1, az_i[39],
                    (az_i + e1) < 0, (az_i + (e1 <<< 1)) < 0, (az_i + e3) < 0};
            // P6: D(m) = m^2 e - m C1 z, index 0 .. 7 = m -7 .. 7
            v6 <= v5; sop6 <= sop5; md6 <= md5; m6 <= m5; sh6 <= sh5; ltr6 <= ltr; lti6 <= lti;
            dr[0] <= ea49 + q7r; dr[1] <= ea25 + q5r; dr[2] <= ea9 + q3r; dr[3] <= ea1 + q1r;
            dr[4] <= ea1 - q1r;  dr[5] <= ea9 - q3r;  dr[6] <= ea25 - q5r; dr[7] <= ea49 - q7r;
            di[0] <= ea49 + q7i; di[1] <= ea25 + q5i; di[2] <= ea9 + q3i; di[3] <= ea1 + q1i;
            di[4] <= ea1 - q1i;  di[5] <= ea9 - q3i;  di[6] <= ea25 - q5i; di[7] <= ea49 - q7i;
            // P7: N = D(m1) - D(m0)
            v7 <= v6; sop7 <= sop6; md7 <= md6; m7 <= m6; sh7 <= sh6;
            nr0 <= dr[sr0[2:0]] - dr[sr0[5:3]]; nr1 <= dr[sr1[2:0]] - dr[sr1[5:3]]; nr2 <= dr[sr2[2:0]] - dr[sr2[5:3]];
            ni0 <= di[si0[2:0]] - di[si0[5:3]]; ni1 <= di[si1[2:0]] - di[si1[5:3]]; ni2 <= di[si2[2:0]] - di[si2[5:3]];
        end
    end

    // ------------------------------------------------------------------ serializer: pairs in bit order
    // BPSK r0 (single); QPSK (r0, i0); 16QAM (r0, r1) (i0, i1); 64QAM (r0, r1) (r2, i0) (i1, i2)
    reg  signed [43:0] sn_r0, sn_r1, sn_r2, sn_i0, sn_i1, sn_i2;
    reg  [1:0]  s_md, s_p, s_left;
    reg  [15:0] s_m;
    reg  [5:0]  s_sh;
    reg         s_sop;
    reg         e_v, e_single, e_sop;
    reg  signed [43:0] e_n0, e_n1;
    reg  [15:0] e_m;
    reg  [5:0]  e_sh;
    always @(posedge clk) begin
        e_v <= 1'b0;
        if (!rst_n) s_left <= 2'd0;
        else begin
            // emit the current RE's next pair; a new RE (v7) arrives with its last pair at the
            // earliest (issue spacing)
            if (v7) begin
                sn_r0 <= nr0; sn_r1 <= nr1; sn_r2 <= nr2; sn_i0 <= ni0; sn_i1 <= ni1; sn_i2 <= ni2;
                s_md <= md7; s_m <= m7; s_sh <= sh7; s_sop <= sop7;
                s_p <= 2'd0;
                s_left <= (md7 == 2'd3) ? 2'd3 : (md7 == 2'd2) ? 2'd2 : 2'd1;
            end else if (s_left != 2'd0) begin
                s_left <= s_left - 2'd1; s_p <= s_p + 2'd1;
            end
            if (s_left != 2'd0) begin
                e_v <= 1'b1; e_m <= s_m; e_sh <= s_sh;
                e_single <= (s_md == 2'd0);
                e_sop <= s_sop && (s_p == 2'd0);
                case ({s_md, s_p})
                    {2'd1, 2'd0}: begin e_n0 <= sn_r0; e_n1 <= sn_i0; end
                    {2'd2, 2'd1}: begin e_n0 <= sn_i0; e_n1 <= sn_i1; end
                    {2'd3, 2'd1}: begin e_n0 <= sn_r2; e_n1 <= sn_i0; end
                    {2'd3, 2'd2}: begin e_n0 <= sn_i1; e_n1 <= sn_i2; end
                    default:      begin e_n0 <= sn_r0; e_n1 <= sn_r1; end
                endcase
            end
        end
    end
    // a new RE result (v7) never arrives before the previous RE's last pair (checked)
`ifdef PD_SIM_CHECKS
    always @(posedge clk) if (rst_n && v7 && s_left > 2'd1) $display("ERROR pl_demap: serializer overrun");
`endif

    // ------------------------------------------------------------------ scaling: Lf = satm(rnd(N m, sh), 12)
    reg         k1v, k1s, k1p, k2v, k2s, k2p, k3v, k3s, k3p, k4v, k4s, k4p, k5v, k5s, k5p;
    reg  signed [43:0] k1n0, k1n1;
    reg  signed [16:0] k1m;
    reg  [5:0]  k1sh, k2sh, k3sh, k4sh, k5sh;
    reg  signed [60:0] k2p0, k2p1, k3p0, k3p1, k4u0, k4u1, k5u0, k5u1;
    function signed [11:0] lsat;               // sat12 of rnd(.) given u = p >>> (sh - 1), or p (sh 0)
        input signed [60:0] u;
        input rnd;
        reg signed [60:0] r;
        begin
            if (rnd) begin
                if (u >= 61'sd4095) lsat = 12'sd2047;
                else if (u <= -61'sd4096) lsat = -12'sd2047;
                else begin r = (u + 61'sd1) >>> 1; lsat = r[11:0]; end
            end else begin
                if (u > 61'sd2047) lsat = 12'sd2047;
                else if (u < -61'sd2047) lsat = -12'sd2047;
                else lsat = u[11:0];
            end
        end
    endfunction
    wire [5:0] k3s1 = (k3sh == 6'd0) ? 6'd0 : k3sh - 6'd1;
    reg  [2:0] k4f;
    reg         o_v, o_single, o_sop;
    reg  signed [11:0] o_l0, o_l1;
    always @(posedge clk) begin
        k1v <= 1'b0; k2v <= 1'b0; k3v <= 1'b0; k4v <= 1'b0; k5v <= 1'b0; o_v <= 1'b0;
        if (rst_n) begin
            k1v <= e_v; k1s <= e_single; k1p <= e_sop; k1n0 <= e_n0; k1n1 <= e_n1; k1m <= {1'b0, e_m}; k1sh <= e_sh;
            k2v <= k1v; k2s <= k1s; k2p <= k1p; k2sh <= k1sh;
            k2p0 <= k1n0 * k1m; k2p1 <= k1n1 * k1m;
            k3v <= k2v; k3s <= k2s; k3p <= k2p; k3sh <= k2sh; k3p0 <= k2p0; k3p1 <= k2p1;
            // shift by sh - 1 in two steps (coarse 8, fine 1)
            k4v <= k3v; k4s <= k3s; k4p <= k3p; k4sh <= k3sh; k4f <= k3s1[2:0];
            k4u0 <= k3p0 >>> {k3s1[5:3], 3'd0}; k4u1 <= k3p1 >>> {k3s1[5:3], 3'd0};
            k5v <= k4v; k5s <= k4s; k5p <= k4p; k5sh <= k4sh;
            k5u0 <= k4u0 >>> k4f; k5u1 <= k4u1 >>> k4f;
            o_v <= k5v; o_single <= k5s; o_sop <= k5p;
            o_l0 <= lsat(k5u0, k5sh != 6'd0); o_l1 <= lsat(k5u1, k5sh != 6'd0);
        end
    end
    assign out_ev = o_v;

    // ------------------------------------------------------------------ BPSK pairing, pair FIFO (32)
    reg         hv;                            // a BPSK LLR waiting for its partner
    reg  signed [11:0] hl;
    reg         hsop;
    (* ram_style = "distributed" *) reg [24:0] pm [0:31];
    always @(posedge clk) begin
        if (!rst_n) begin
            hv <= 1'b0; p_wr <= 6'd0; p_rd <= 6'd0;
        end else begin
            if (o_v) begin
                if (!o_single) begin
                    if (o_sop) hv <= 1'b0;
                    pm[p_wr[4:0]] <= {o_sop, o_l0, o_l1}; p_wr <= p_wr + 6'd1;
                end else if (hv && !o_sop) begin
                    pm[p_wr[4:0]] <= {hsop, hl, o_l0}; p_wr <= p_wr + 6'd1; hv <= 1'b0;
                end else begin
                    hv <= 1'b1; hl <= o_l0; hsop <= o_sop;   // a new packet drops a waiting one
                end
            end
            if (lf_pop && p_wr != p_rd) p_rd <= p_rd + 6'd1;
        end
    end
    assign pf_n = p_wr - p_rd;
    wire [24:0] ph = pm[p_rd[4:0]];
    assign lf_valid = (p_wr != p_rd);
    assign lf_sop = ph[24];
    assign lf_l0 = ph[23:12];
    assign lf_l1 = ph[11:0];

    assign busy = (f_n != 5'd0) || v1 || v2 || v3 || v4 || v5 || v6 || v7 || (s_left != 2'd0) ||
                  (infl != 6'd0) || lf_valid || (sc != SC_IDLE);
endmodule

`default_nettype wire
