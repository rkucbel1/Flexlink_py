// pl_comb.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Payload combiner (step 12c), bit-true to pl_reference.combine (steps 9-11): the per-copy LLRs
// of pl_demap (pairs, sequence order) are descrambled with the Gold code (C_init, 1600-step
// start, pl_gold_init), summed over the copies of each code word bit (acc = satm(acc + Lf, 16),
// copies in order) and, after the last copy, read out in code order through the inverse
// interleaver as the decoder input Lq = satm(rnd(acc, 3), 7).
//
// Two accumulator banks (1024 x 32 bit, a word = the pair 2p, 2p + 1): one fills while the
// other is read out. Code words w = 0 .. NDB_A - 1 of a packet; the LLRs after the last one, and
// a code word that the packet does not complete, are dropped. lf_sop restarts the count.
//
// Output: Lq two per clock (lq_*: code positions v, v + 1) while lq_ready, and per code word one
// record (rc_*: t_trig, w and the signature of Lq in code order, cf_reference.grid_signature)
// when rc_room. The readout starts only with rc_room.
`timescale 1ns / 1ps
`default_nettype none

module pl_comb (
    input  wire               clk,
    input  wire               rst_n,
    // LLR pairs (pl_demap)
    input  wire               lf_valid,
    input  wire               lf_sop,
    input  wire signed [11:0] lf_l0,
    input  wire signed [11:0] lf_l1,
    output wire               lf_pop,
    input  wire [1:0]         cfg_ncode,
    input  wire [1:0]         cfg_rate,
    input  wire [4:0]         cfg_rep,
    input  wire [9:0]         cfg_ndb,
    input  wire [7:0]         cfg_cinit,
    input  wire [31:0]        cfg_t,
    // decoder input
    input  wire               lq_ready,
    output reg                lq_valid,
    output reg                lq_first,
    output reg                lq_last,
    output reg  [10:0]        lq_v,           // code position of lq_q0 (lq_q1: v + 1)
    output reg  signed [6:0]  lq_q0,
    output reg  signed [6:0]  lq_q1,
    output wire [3:0]         lq_code,        // the code word's code: 4 N code + rate code (pl_ldpc_core)
    output wire [9:0]         lq_w,           // its index in the packet
    output wire [31:0]        lq_t,           // its packet's t_trig
    // per code word record
    input  wire               rc_room,
    output reg                rc_valid,
    output reg  [31:0]        rc_t,
    output reg  [9:0]         rc_w,
    output reg  [31:0]        rc_a,
    output reg  [31:0]        rc_b,
    output wire               busy
);
    function [9:0] half_n;                     // N / 2
        input [1:0] nc;
        half_n = (nc == 2'd0) ? 10'd324 : (nc == 2'd1) ? 10'd648 : 10'd972;
    endfunction
    function [11:0] il_base;
        input [1:0] nc;
        il_base = (nc == 2'd0) ? 12'd0 : (nc == 2'd1) ? 12'd648 : 12'd1944;
    endfunction

    // ------------------------------------------------------------------ Gold code, 2 steps per pair
    wire [30:0] g_x1i, g_x2i;
    pl_gold_init u_gi (.ci(cfg_cinit), .x1(g_x1i), .x2(g_x2i));
    reg  [30:0] gx1, gx2;
    function [30:0] st1;
        input [30:0] r;
        st1 = {r[0] ^ r[3], r[30:1]};
    endfunction
    function [30:0] st2;
        input [30:0] r;
        st2 = {r[0] ^ r[1] ^ r[2] ^ r[3], r[30:1]};
    endfunction
    wire [30:0] ga1 = lf_sop ? g_x1i : gx1;
    wire [30:0] ga2 = lf_sop ? g_x2i : gx2;
    wire [30:0] gb1 = st1(ga1), gb2 = st2(ga2);
    wire        g0 = ga1[0] ^ ga2[0], g1 = gb1[0] ^ gb2[0];

    // ------------------------------------------------------------------ fill side
    reg         fb;                            // bank being filled
    reg  [1:0]  full;                          // bank holds a complete code word
    reg  [9:0]  bk_w [0:1];
    reg  [31:0] bk_t [0:1];
    reg  [1:0]  bk_nc [0:1];
    reg  [1:0]  bk_rt [0:1];
    reg  [9:0]  j2, n2;                        // pair index in the copy, N / 2
    reg  [4:0]  cc, rep;
    reg  [9:0]  w, ndb;
    reg  [1:0]  nc, rt;
    reg  [31:0] tt;
    reg         drop;
    wire        drop_now = lf_sop ? (cfg_ndb == 10'd0) : drop;
    assign lf_pop = lf_valid && (drop_now || !full[fb]);
    wire        take = lf_pop && !drop_now;
    wire [9:0]  j_now = lf_sop ? 10'd0 : j2;
    wire [4:0]  c_now = lf_sop ? 5'd0 : cc;
    wire [9:0]  n2_now = lf_sop ? half_n(cfg_ncode) : n2;
    wire [4:0]  rep_now = lf_sop ? cfg_rep : rep;
    wire        cw_end = (j_now == n2_now - 10'd1) && (c_now == rep_now);
    // RMW pipeline: f1 (RAM address), f2 (RAM data: sum, write registered), f3 (write; the bank
    // is full after a code word's last write)
    reg         f1v, f2v, f3v, f1c0, f2c0, f1e, f2e, f3e, f1b, f2b, f3b;
    reg  [9:0]  f1a, f2a;
    reg  signed [11:0] f1l0, f1l1, f2l0, f2l1;
    reg         wb_we, wb_bank;
    reg  [9:0]  wb_addr;
    reg  [31:0] wb_d;
    wire [31:0] rda [0:1];
    wire [31:0] rdb [0:1];
    function signed [15:0] acc16;              // satm(acc + l, 16)
        input signed [15:0] a;
        input signed [11:0] l;
        reg signed [16:0] s;
        begin
            s = {a[15], a} + {{5{l[11]}}, l};
            acc16 = (s > 17'sd32767) ? 16'sd32767 : (s < -17'sd32767) ? -16'sd32767 : s[15:0];
        end
    endfunction

    always @(posedge clk) begin
        f1v <= 1'b0; wb_we <= 1'b0;
        if (!rst_n) begin
            fb <= 1'b0; drop <= 1'b1; j2 <= 10'd0; cc <= 5'd0; w <= 10'd0;
            f2v <= 1'b0; f3v <= 1'b0;
        end else begin
            if (lf_pop && lf_sop) begin
                n2 <= half_n(cfg_ncode); rep <= cfg_rep; ndb <= cfg_ndb; nc <= cfg_ncode; rt <= cfg_rate; tt <= cfg_t;
                w <= 10'd0; j2 <= 10'd0; cc <= 5'd0; drop <= (cfg_ndb == 10'd0);
            end
            if (take) begin
                gx1 <= st1(gb1); gx2 <= st2(gb2);
                f1v <= 1'b1; f1b <= fb; f1a <= j_now; f1c0 <= (c_now == 5'd0); f1e <= cw_end;
                f1l0 <= g0 ? -lf_l0 : lf_l0;
                f1l1 <= g1 ? -lf_l1 : lf_l1;
                if (j_now == n2_now - 10'd1) begin
                    j2 <= 10'd0;
                    if (c_now == rep_now) begin
                        cc <= 5'd0;
                        bk_w[fb] <= lf_sop ? 10'd0 : w;
                        bk_t[fb] <= lf_sop ? cfg_t : tt;
                        bk_nc[fb] <= lf_sop ? cfg_ncode : nc;
                        bk_rt[fb] <= lf_sop ? cfg_rate : rt;
                        w <= (lf_sop ? 10'd0 : w) + 10'd1;
                        if ((lf_sop ? 10'd0 : w) + 10'd1 == (lf_sop ? cfg_ndb : ndb)) drop <= 1'b1;
                        fb <= !fb;
                    end else cc <= c_now + 5'd1;
                end else begin
                    j2 <= j_now + 10'd1; cc <= c_now;
                end
            end
            f2v <= f1v; f2e <= f1e; f2b <= f1b; f2a <= f1a; f2c0 <= f1c0; f2l0 <= f1l0; f2l1 <= f1l1;
            // f2: the read data are valid: sum (copy 0: the LLR itself), register the write
            if (f2v) begin
                wb_we <= 1'b1; wb_bank <= f2b; wb_addr <= f2a;
                if (f2c0) wb_d <= {{4{f2l1[11]}}, f2l1, {4{f2l0[11]}}, f2l0};
                else wb_d <= {acc16(rda[f2b][31:16], f2l1), acc16(rda[f2b][15:0], f2l0)};
            end
            f3v <= f2v; f3e <= f2e; f3b <= f2b;
        end
    end

    // ------------------------------------------------------------------ readout
    reg         rb;                            // bank to read next
    reg         ro;                            // readout running
    reg  [11:0] rv;                            // next code position
    reg  [11:0] rn;                            // N
    reg  [11:0] rbase;
    reg  [11:0] il_a, il_b;
    wire [10:0] il_da, il_db;
    pl_ilinv_rom u_il (.clk(clk), .addr_a(il_a), .addr_b(il_b), .da(il_da), .db(il_db));
    reg         r1v, r2v, r3v, r1l, r2l, r3l, r1f, r2f, r3f;
    reg  [11:0] r1p, r2p, r3p;
    reg         r3h0, r3h1;
    reg  [31:0] sa, sb;
    function signed [6:0] lq7;                 // satm(rnd(acc, 3), 7)
        input signed [15:0] a;
        reg signed [13:0] r;
        begin
            r = ({a[15], a} + 17'sd4) >>> 3;
            lq7 = (r > 14'sd63) ? 7'sd63 : (r < -14'sd63) ? -7'sd63 : r[6:0];
        end
    endfunction
    wire        r_go = ro && lq_ready && (rv != rn);
    // RAM addresses: fill (port A read, port B write) or readout (both ports read)
    wire [9:0]  a_addr0 = (ro && rb == 1'b0) ? il_da[10:1] : (f1v && f1b == 1'b0) ? f1a : 10'd0;
    wire [9:0]  a_addr1 = (ro && rb == 1'b1) ? il_da[10:1] : (f1v && f1b == 1'b1) ? f1a : 10'd0;
    wire [9:0]  b_addr0 = (ro && rb == 1'b0) ? il_db[10:1] : wb_addr;
    wire [9:0]  b_addr1 = (ro && rb == 1'b1) ? il_db[10:1] : wb_addr;
    cf_tdp_ram #(.W(32), .A(10)) u_b0 (.clk(clk), .we_a(1'b0), .addr_a(a_addr0), .wd_a(32'd0), .rd_a(rda[0]),
                                      .we_b(wb_we && wb_bank == 1'b0), .addr_b(b_addr0), .wd_b(wb_d), .rd_b(rdb[0]));
    cf_tdp_ram #(.W(32), .A(10)) u_b1 (.clk(clk), .we_a(1'b0), .addr_a(a_addr1), .wd_a(32'd0), .rd_a(rda[1]),
                                      .we_b(wb_we && wb_bank == 1'b1), .addr_b(b_addr1), .wd_b(wb_d), .rd_b(rdb[1]));
    reg  signed [6:0] q0, q1;
    reg         q_v, q_l, q_f;
    reg  [11:0] q_p;
    wire [31:0] x0 = {14'd0, {11{q0[6]}}, q0};
    wire [31:0] x1 = {14'd0, {11{q1[6]}}, q1};
    wire [31:0] sa_q = q_f ? 32'd0 : sa;       // signature before this pair (0 at v = 0)
    wire [31:0] sb_q = q_f ? 32'd0 : sb;
    wire [31:0] sa_n = sa_q + x0 + x1;
    wire [31:0] sb_n = sb_q + {sa_q[30:0], 1'b0} + {x0[30:0], 1'b0} + x1;

    always @(posedge clk) begin
        r1v <= 1'b0; q_v <= 1'b0; rc_valid <= 1'b0; lq_valid <= 1'b0;
        if (!rst_n) begin
            rb <= 1'b0; ro <= 1'b0; full <= 2'b00; r2v <= 1'b0; r3v <= 1'b0;
        end else begin
            // a code word complete (its last write done): the bank is full
            if (f3v && f3e) full[f3b] <= 1'b1;
            // start
            if (!ro && full[rb] && rc_room) begin
                ro <= 1'b1; rv <= 12'd0; rn <= {1'b0, half_n(bk_nc[rb]), 1'b0}; rbase <= il_base(bk_nc[rb]);
            end
            // issue: ROM addresses (r1), ROM data / RAM addresses (r2), RAM data (r3), Lq (q)
            if (r_go) begin
                il_a <= rbase + rv; il_b <= rbase + rv + 12'd1;
                r1v <= 1'b1; r1p <= rv; r1f <= (rv == 12'd0); r1l <= (rv + 12'd2 == rn);
                rv <= rv + 12'd2;
            end
            r2v <= r1v; r2p <= r1p; r2f <= r1f; r2l <= r1l;
            r3v <= r2v; r3p <= r2p; r3f <= r2f; r3l <= r2l; r3h0 <= il_da[0]; r3h1 <= il_db[0];
            if (r3v) begin
                q_v <= 1'b1; q_p <= r3p; q_f <= r3f; q_l <= r3l;
                q0 <= lq7(r3h0 ? rda[rb][31:16] : rda[rb][15:0]);
                q1 <= lq7(r3h1 ? rdb[rb][31:16] : rdb[rb][15:0]);
            end
            if (q_v) begin
                lq_valid <= 1'b1; lq_first <= q_f; lq_last <= q_l; lq_v <= q_p[10:0]; lq_q0 <= q0; lq_q1 <= q1;
                sa <= sa_n; sb <= sb_n;
                if (q_l) begin
                    rc_valid <= 1'b1; rc_t <= bk_t[rb]; rc_w <= bk_w[rb]; rc_a <= sa_n; rc_b <= sb_n;
                    full[rb] <= 1'b0; rb <= !rb; ro <= 1'b0;
                end
            end
        end
    end

    assign lq_code = {bk_nc[rb], bk_rt[rb]};
    assign lq_w = bk_w[rb];
    assign lq_t = bk_t[rb];

    assign busy = f1v || f2v || f3v || wb_we || (full != 2'b00) || ro || r1v || r2v || r3v || q_v;
endmodule

`default_nettype wire
