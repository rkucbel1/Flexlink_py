// tb_pl_soft.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// pl_demap + pl_comb + pl_dec against pl_reference (soft_bits, combine, decode_ldpc, crc24) on
// the packets of make_lq_vectors.py (+dir=, default "."; n.txt, cases.txt). The testbench plays
// pl_chest: the packet record, then the H stream with credits (16), random gaps (+gap=), the
// next packet's record as soon as the last H is out. rc_room toggles at random, the transport-
// block FIFO is read with random gaps. Every decoder input Lq (code order), every code-word
// record and every transport block (bits, crc_ok, iterations) must match, in order.
`timescale 1ns / 1ps

module tb_pl_soft;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg               pk_valid = 1'b0;
    reg  [31:0]       pk_t = 0;
    reg  [53:0]       pk_msg = 0;
    reg  [9:0]        pk_n0 = 0;
    reg  [47:0]       pk_sumn = 0;
    reg               h_valid = 1'b0, h_ref = 1'b0;
    reg  [6:0]        h_sym = 0;
    reg  [9:0]        h_col = 0;
    reg  signed [15:0] h_re = 0, h_im = 0, y_re = 0, y_im = 0;
    wire              h_take;
    wire              lf_valid, lf_sop, lf_pop;
    wire signed [11:0] lf_l0, lf_l1;
    wire [1:0]        cfg_ncode;
    wire [4:0]        cfg_rep;
    wire [9:0]        cfg_ndb;
    wire [7:0]        cfg_cinit;
    wire [31:0]       cfg_t;
    wire [1:0]        cfg_rate;
    wire [3:0]        lq_code;
    wire [9:0]        lq_w;
    wire [31:0]       lq_t;
    wire [11:0]       tb_cnt;
    wire [31:0]       tb_word;
    reg               tb_pop = 1'b0;
    wire              dc_busy;
    wire              dm_busy, cb_busy;
    wire              lq_ready;
    reg               rc_room = 1'b1;
    wire              lq_valid, lq_first, lq_last, rc_valid;
    wire [10:0]       lq_v;
    wire signed [6:0] lq_q0, lq_q1;
    wire [31:0]       rc_t, rc_a, rc_b;
    wire [9:0]        rc_w;

    pl_demap dm (.clk(clk), .rst_n(rst_n), .pk_valid(pk_valid), .pk_t(pk_t), .pk_msg(pk_msg), .pk_n0(pk_n0),
                 .pk_sumn(pk_sumn), .h_valid(h_valid), .h_sym(h_sym), .h_col(h_col), .h_ref(h_ref), .h_re(h_re),
                 .h_im(h_im), .y_re(y_re), .y_im(y_im), .h_take(h_take), .lf_valid(lf_valid), .lf_sop(lf_sop),
                 .lf_l0(lf_l0), .lf_l1(lf_l1), .lf_pop(lf_pop), .cfg_ncode(cfg_ncode), .cfg_rate(cfg_rate), .cfg_rep(cfg_rep),
                 .cfg_ndb(cfg_ndb), .cfg_cinit(cfg_cinit), .cfg_t(cfg_t), .busy(dm_busy));
    pl_comb cb (.clk(clk), .rst_n(rst_n), .lf_valid(lf_valid), .lf_sop(lf_sop), .lf_l0(lf_l0), .lf_l1(lf_l1),
                .lf_pop(lf_pop), .cfg_ncode(cfg_ncode), .cfg_rate(cfg_rate), .cfg_rep(cfg_rep), .cfg_ndb(cfg_ndb), .cfg_cinit(cfg_cinit),
                .cfg_t(cfg_t), .lq_ready(lq_ready), .lq_valid(lq_valid), .lq_first(lq_first), .lq_last(lq_last),
                .lq_v(lq_v), .lq_q0(lq_q0), .lq_q1(lq_q1), .lq_code(lq_code), .lq_w(lq_w), .lq_t(lq_t), .rc_room(rc_room), .rc_valid(rc_valid), .rc_t(rc_t),
                .rc_w(rc_w), .rc_a(rc_a), .rc_b(rc_b), .busy(cb_busy));
    pl_dec dc (.clk(clk), .rst_n(rst_n), .lq_valid(lq_valid), .lq_first(lq_first), .lq_last(lq_last), .lq_q0(lq_q0),
               .lq_q1(lq_q1), .lq_code(lq_code), .lq_w(lq_w), .lq_t(lq_t), .lq_ready(lq_ready), .tb_cnt(tb_cnt),
               .tb_word(tb_word), .tb_pop(tb_pop), .busy(dc_busy));

    integer fn, fc, rv, npk, p, s, col, plen, rsp, gap, seed, errs, cyc, n0v, tv, a, b2, c2, d2;
    integer cred, nblk_tot, nrec, nlq_cw, nlq_v;
    reg [63:0] msg64, sumn64;
    // expected, all packets: per code word (in order) N values, signature, t
    integer xn [0:255];
    integer xnb [0:63];
    reg [31:0] xa [0:255], xb [0:255], xt [0:255];
    integer xw [0:255];
    reg signed [7:0] xq [0:255*1944];
    integer xbase [0:255];
    integer xok [0:255], xit [0:255], xnbt [0:255];
    reg [31:0] xtw [0:255*50];
    integer ntb, tb_k, tb_nw, tb_wait, tb_errs0;
    reg [31:0] tb_hdr, tb_t;
    integer ncw;                               // code words expected in total
    reg [8*256-1:0] dir, fname;
    integer pk_plen [0:63], pk_r [0:63], pk_off [0:63];
    integer pk_n0v [0:63];
    integer pk_tt [0:63];
    reg [63:0] pk_sv [0:63], pk_mv [0:63];
    integer ngrid, xi;
    reg signed [15:0] gbuf_yr [0:800000], gbuf_yi [0:800000], gbuf_hr [0:800000], gbuf_hi [0:800000];
    always @(posedge clk) cyc <= cyc + 1;

    // ------------------------------------------------------------------ consumer: random readiness
    always @(posedge clk) begin
        rc_room <= (gap == 0) || (($random(seed) & 7) != 0);
    end
    // ------------------------------------------------------------------ checker
    always @(posedge clk) begin
        if (lq_valid) begin
            if (nrec >= ncw) begin
                if (errs < 10) $display("ERROR: Lq output beyond the %0d expected code words", ncw);
                errs = errs + 1;
            end else begin
                if (lq_v !== nlq_v || lq_first !== (nlq_v == 0) || lq_last !== (nlq_v + 2 == xn[nrec])) begin
                    if (errs < 10) $display("ERROR cw %0d: lq_v %0d first %b last %b, expected v %0d", nrec, lq_v, lq_first, lq_last, nlq_v);
                    errs = errs + 1;
                end
                if (lq_q0 !== xq[xbase[nrec] + nlq_v][6:0] || lq_q1 !== xq[xbase[nrec] + nlq_v + 1][6:0]) begin
                    if (errs < 10) $display("ERROR cw %0d (w %0d) v %0d: Lq %0d %0d, model %0d %0d", nrec, xw[nrec], nlq_v,
                                            lq_q0, lq_q1, xq[xbase[nrec] + nlq_v], xq[xbase[nrec] + nlq_v + 1]);
                    errs = errs + 1;
                end
                nlq_v = nlq_v + 2;
            end
        end
        if (rc_valid) begin
            if (nrec >= ncw) begin
                if (errs < 10) $display("ERROR: record beyond the expected code words");
                errs = errs + 1;
            end else begin
                if (rc_t !== xt[nrec] || rc_w !== xw[nrec] || rc_a !== xa[nrec] || rc_b !== xb[nrec] || nlq_v != xn[nrec]) begin
                    if (errs < 10) $display("ERROR cw %0d: record t %0d w %0d sig %h %h after %0d Lq, model t %0d w %0d sig %h %h",
                                            nrec, rc_t, rc_w, rc_a, rc_b, nlq_v, xt[nrec], xw[nrec], xa[nrec], xb[nrec]);
                    errs = errs + 1;
                end
            end
            nrec = nrec + 1; nlq_v = 0;
        end
    end
    // transport-block reader: one word, then at least 2 clocks (the FIFO's read latency)
    always @(posedge clk) begin
        tb_pop <= 1'b0;
        if (tb_wait > 0) tb_wait = tb_wait - 1;
        else if (rst_n && tb_cnt != 12'd0 && (gap == 0 || ($random(seed) & 3) == 0)) begin
            tb_pop <= 1'b1; tb_wait = 2;
            if (tb_k == 0) tb_t = tb_word;
            else if (tb_k == 1) begin tb_hdr = tb_word; tb_nw = (tb_word[10:0] + 31) / 32; end
            else if (ntb < ncw && tb_word !== xtw[ntb * 50 + tb_k - 2]) begin
                if (errs < 10) $display("ERROR block %0d word %0d: %h, model %h", ntb, tb_k - 2, tb_word, xtw[ntb * 50 + tb_k - 2]);
                errs = errs + 1;
            end
            tb_k = tb_k + 1;
            if (tb_k >= 2 && tb_k == tb_nw + 2) begin
                if (ntb >= ncw) begin
                    if (errs < 10) $display("ERROR: transport block beyond the expected code words");
                    errs = errs + 1;
                end else if (tb_t !== xt[ntb] || tb_hdr[31] !== xok[ntb][0] || tb_hdr[27:24] !== xit[ntb][3:0] ||
                             tb_hdr[23:14] !== xw[ntb][9:0] || tb_hdr[10:0] !== xnbt[ntb][10:0]) begin
                    if (errs < 10) $display("ERROR block %0d: t %0d hdr crc %b it %0d w %0d nbits %0d, model t %0d crc %0d it %0d w %0d nbits %0d",
                                            ntb, tb_t, tb_hdr[31], tb_hdr[27:24], tb_hdr[23:14], tb_hdr[10:0],
                                            xt[ntb], xok[ntb], xit[ntb], xw[ntb], xnbt[ntb]);
                    errs = errs + 1;
                end
                ntb = ntb + 1; tb_k = 0;
            end
        end
    end
    // credits: pl_chest's side
    always @(posedge clk) if (rst_n && h_take) cred = cred + 1;

    task send_re(input integer idx, input integer sym, input integer c, input integer rf);
        begin
            while (cred == 0 || (($random(seed) & 7) < gap)) @(negedge clk);
            h_valid = 1'b1; h_sym = sym; h_col = c; h_ref = rf;
            y_re = gbuf_yr[idx]; y_im = gbuf_yi[idx]; h_re = gbuf_hr[idx]; h_im = gbuf_hi[idx];
            cred = cred - 1;
            @(negedge clk); h_valid = 1'b0;
        end
    endtask

    initial begin
        errs = 0; cyc = 0; nrec = 0; ntb = 0; tb_k = 0; tb_wait = 0; nlq_v = 0; ncw = 0; cred = 16; ngrid = 0;
        if (!$value$plusargs("dir=%s", dir)) dir = ".";
        if (!$value$plusargs("gap=%d", gap)) gap = 1;
        if (!$value$plusargs("seed=%d", seed)) seed = 1;
        rv = $random(seed);
        $sformat(fname, "%0s/n.txt", dir); fn = $fopen(fname, "r");
        if (fn == 0) begin $display("FAIL lq_units: cannot open %0s", fname); $finish; end
        rv = $fscanf(fn, "%d", npk); $fclose(fn);
        $sformat(fname, "%0s/cases.txt", dir); fc = $fopen(fname, "r");
        // read everything first
        xi = 0;
        for (p = 0; p < npk; p = p + 1) begin
            rv = $fscanf(fc, "%h %d %d %d %d %d", msg64, tv, n0v, sumn64, plen, rsp);
            pk_mv[p] = msg64; pk_sv[p] = sumn64; pk_n0v[p] = n0v; pk_plen[p] = plen; pk_r[p] = rsp; pk_off[p] = ngrid;
            pk_tt[p] = tv;
            for (s = 0; s < plen * 841; s = s + 1) begin
                rv = $fscanf(fc, "%d %d %d %d", a, b2, c2, d2);
                gbuf_yr[ngrid + s] = a; gbuf_yi[ngrid + s] = b2; gbuf_hr[ngrid + s] = c2; gbuf_hi[ngrid + s] = d2;
            end
            ngrid = ngrid + plen * 841;
            if (ngrid > 800000 || ncw > 255) begin $display("FAIL lq_units: the vectors exceed the testbench arrays"); $finish; end
            rv = $fscanf(fc, "%d %d", xnb[p], a);
            for (s = 0; s < xnb[p]; s = s + 1) begin
                xn[ncw] = a; xw[ncw] = s; xt[ncw] = tv; xbase[ncw] = xi;
                for (col = 0; col < a; col = col + 1) begin rv = $fscanf(fc, "%d", b2); xq[xi] = b2; xi = xi + 1; end
                rv = $fscanf(fc, "%d %d", xa[ncw], xb[ncw]);
                rv = $fscanf(fc, "%d %d %d", xok[ncw], xit[ncw], xnbt[ncw]);
                for (col = 0; col < (xnbt[ncw] + 31) / 32; col = col + 1) begin
                    rv = $fscanf(fc, "%h", tb_t); xtw[ncw * 50 + col] = tb_t;
                end
                ncw = ncw + 1;
            end
        end
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        for (p = 0; p < npk; p = p + 1) begin
            @(negedge clk);
            pk_valid = 1'b1; pk_t = pk_tt[p]; pk_msg = pk_mv[p][53:0]; pk_n0 = pk_n0v[p]; pk_sumn = pk_sv[p][47:0];
            @(negedge clk); pk_valid = 1'b0;
            repeat (($random(seed) & 63) * gap + 2) @(negedge clk);
            for (s = 0; s < pk_plen[p]; s = s + 1)
                for (col = 0; col < 841; col = col + 1)
                    send_re(pk_off[p] + s * 841 + col, s, col, (s % pk_r[p]) == 0);
        end
        // drain
        a = 0;
        while ((dm_busy || cb_busy || dc_busy || nrec < ncw || ntb < ncw || tb_cnt != 0) && a < 2000000) begin @(posedge clk); a = a + 1; end
        repeat (50) @(posedge clk);
        if (nrec != ncw) begin $display("ERROR: %0d code-word records, expected %0d", nrec, ncw); errs = errs + 1; end
        if (ntb != ncw) begin $display("ERROR: %0d transport blocks, expected %0d", ntb, ncw); errs = errs + 1; end
        if (dm_busy || cb_busy || dc_busy) begin $display("ERROR: still busy (demap %b, comb %b, dec %b)", dm_busy, cb_busy, dc_busy); errs = errs + 1; end
        if (errs == 0) $display("PASS lq_units: %0d packets, %0d code words, every decoder input, record and transport block equals the model (%0d clocks)", npk, ncw, cyc);
        else $display("FAIL lq_units: %0d errors", errs);
        $finish;
    end
endmodule
