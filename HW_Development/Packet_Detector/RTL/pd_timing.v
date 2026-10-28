// pd_timing.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Preamble B timing (spec 4.5): 1-bit tap correlator, bit-true to pd_reference.timing_records.
// For each detector event that passed the tone check (N = n_cfo, t = t_trig):
//   window    n = lo .. hi, lo = t + N + 1099, hi = t + 6163 (last Preamble B sample)
//   samples   y(n0 .. hi), n0 = lo - 1139: the DC blocker output (unfiltered), same index as t
//   de-rotate phi(n0) = 0, phi += cfo_inc, a = phi[23:14], (c, s) = (COS[a], COS[a - 256]),
//             xr = sat6(rnd((yi c - yq s) >> 21)), xi = sat6(rnd((yi s + yq c) >> 21))
//   correlate Y(n) = sum_j x(n-1139+j) (sI(j) - j sQ(j)), P(n) = |Y(n)|^2; first maximum,
//             sum of P; PEAK_OK = T_pk == 0 or (p_max > 0 and p_max * n_pos * 256 >= T_pk * p_sum);
//             LATE = n_end < t + N + 1131
// One search at a time: an event whose n0 is not after the hi of the last search started
// gets a BUSY record at once; the others queue (in order) and get a result record when their
// window is done. Records of different events can come out of order (BUSY first).
//
// Samples: every DC blocker output goes into a circular buffer of 2^DB samples (block RAM),
// indexed by sample number. A search reads its samples from there, so it can start long after
// they arrived (the event only comes after the tone check). `stall` holds the input back while
// a sample still needed (a waiting trigger, a queued or the running search) would be
// overwritten. MARGIN covers the samples still inside the front end.
//
// Correlator: the 1140 taps are split over NK = 5 phases of NL = 228 lane units, one phase per
// clock, so one output takes 5 clocks: real time for 20 Msps on a 100 MHz clock. Faster input
// (DMA from memory) is held back through `stall`. The 1140 de-rotated samples sit in per-lane
// LUT RAMs (slot q = (m - n0) mod 1140 -> lane q mod 228, depth q / 228, stored as A = xr + xi,
// B = xr - xi); lane u reads depth k in phase k. The tap of slot q for output n is
// j = (q - (n - 1139 - n0)) mod 1140, so the sign registers start as the sign table and rotate
// by one place per output. Each lane forms its term from A, B and the two signs without a
// multiplier: re = sI xr + sQ xi = +-A or +-B, im = sI xi - sQ xr = -+B or +-A; a negation is
// a bit inversion plus 1, and the 1s (599 for re, 576 for im) are added as a constant.
// The next sample is written into the slot of the oldest one in the phase that reads that slot
// (once per output: the write that wraps to the next depth must not trigger a second one);
// the write itself lands one clock later (registered, for the fanout to 228 lane RAMs).
`timescale 1ns / 1ps
`default_nettype none

module pd_timing #(
    parameter integer DB     = 12,       // sample buffer 2^DB samples
    parameter integer MARGIN = 64
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,          // input_stage 1 or 2 (needs the unfiltered stream)
    input  wire [15:0]        n_cfo,
    input  wire [15:0]        t_pk,        // Q8.8, 0 = no peak check
    // DC blocker output (20 Msps index = t_trig index)
    input  wire               y_valid,
    input  wire signed [11:0] y_i,
    input  wire signed [11:0] y_q,
    // detector
    input  wire               trig,
    input  wire [31:0]        t_trig,
    input  wire               ev_valid,
    input  wire [31:0]        ev_t,
    input  wire [31:0]        ev_cfo_inc,
    input  wire               ev_fail,
    // back-pressure towards the sample input
    output reg                stall,
    // timing record
    output reg                tm_valid,
    output reg  [31:0]        tm_t,
    output reg  [31:0]        tm_n_end,
    output reg  [2:0]         tm_status,   // bit0 PEAK_OK, bit1 LATE, bit2 BUSY
    output reg  [15:0]        tm_n_pos,
    output reg  [33:0]        tm_p_max,
    output reg  [47:0]        tm_p_sum,
    output wire               busy,        // work in progress that does not wait for samples
    output reg                err          // start queue overflow (sticky; cannot happen)
);
    localparam integer LEN  = 1140;
    localparam integer NL   = 228;
    localparam integer NK   = 5;
    localparam integer AW   = 7;            // A, B
    localparam integer TW   = AW + 8;       // 256-input tree sum
    localparam integer YW   = 18;           // |Y| <= 1140 * 64
    localparam integer DEPTH = 1 << DB;
    localparam [31:0] LIM   = DEPTH - MARGIN;
    localparam [LEN-1:0] NEG_I = 1140'h0d99ff6a70e5a3f3267c6d3872b7fcddc0cacff253bf698e12b18249ff258fcd275c963e34cfe6a60324c06523c27600dd8f8a58065b004d9ff693fcd4c06cde793303ab3fcd6ff9b300ca601b50f1a9006563c6b603b4c07527f32c7c693ae4b3f9a4ff92638dcc719675cb4ffb5301b13fe56f1c963e24cfc4a70e567fc9b00d99ff6a70e5a3f3267c6d3872b7f;
    localparam [LEN-1:0] NEG_Q = 1140'hb948fc4b3fcd98065200c99fe691f9b4ffda67c6d9fcd4c076980ca47c6ec01933f99b006b628cb78e4277cdb006938396c01b4e1cd981d2c7c4d8f196ffca47e25bff2d8f1933e349c19b387258032dc0c9300c93ee6a71edb1425600d99fcc9803323e2530192e032b3f9b63e249ff2d9f8927f993004b6009b3fcd23f12deb948fc4b3fcd98065200c99fe691f;
    localparam integer C_RE = 599;            // taps with sI = -1
    localparam integer C_IM = 576;            // taps with sQ = +1

    // ------------------------------------------------------------------ sample buffer
    reg  [31:0] wp;                         // samples written (= next sample index)
    wire        y_in = en && y_valid;
    reg  [DB-1:0] b_ra;
    wire [23:0] b_rd;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(24), .A(DB)) u_buf (.clk(clk), .we(y_in), .waddr(wp[DB-1:0]),
        .wdata({y_q, y_i}), .raddr(b_ra), .rdata(b_rd));
    always @(posedge clk) begin
        if (!rst_n) wp <= 32'd0;
        else if (y_in) wp <= wp + 32'd1;
    end

    // ------------------------------------------------------------------ triggers waiting for their event
    reg  [31:0] pt [0:3];
    reg  [1:0]  pt_wr, pt_rd;
    reg  [2:0]  pt_n;
    wire        pt_push = en && trig;
    wire        pt_pop  = en && ev_valid && (pt_n != 3'd0);
    always @(posedge clk) begin
        if (!rst_n) begin
            pt_wr <= 2'd0; pt_rd <= 2'd0; pt_n <= 3'd0;
        end else begin
            if (pt_push) begin pt[pt_wr] <= t_trig; pt_wr <= pt_wr + 2'd1; end
            if (pt_pop) pt_rd <= pt_rd + 2'd1;
            pt_n <= pt_n + {2'd0, pt_push} - {2'd0, pt_pop};
        end
    end

    // ------------------------------------------------------------------ decision per event
    wire [31:0] e_n0 = ev_t + {16'd0, n_cfo} - 32'd40;
    wire [31:0] e_hi = ev_t + 32'd6163;
    reg         last_v;
    reg  [31:0] last_hi;
    wire        e_busy = last_v && ($signed(e_n0 - last_hi) <= 0);
    wire        e_ok   = en && ev_valid && !ev_fail;
    // start queue: t, cfo_inc
    reg  [31:0] sq_t [0:3];
    reg  [23:0] sq_inc [0:3];
    reg  [1:0]  sq_wr, sq_rd;
    reg  [2:0]  sq_n;
    wire        sq_push = e_ok && !e_busy;
    reg         sq_pop;
    // BUSY record waiting for the output
    reg         bz_v;
    reg  [31:0] bz_t;
    always @(posedge clk) begin
        if (!rst_n) begin
            last_v <= 1'b0; last_hi <= 32'd0; sq_wr <= 2'd0; sq_rd <= 2'd0; sq_n <= 3'd0;
            err <= 1'b0;
        end else begin
            if (sq_push) begin
                last_v <= 1'b1; last_hi <= e_hi;
                sq_t[sq_wr] <= ev_t; sq_inc[sq_wr] <= ev_cfo_inc[23:0]; sq_wr <= sq_wr + 2'd1;
                if (sq_n == 3'd4 && !sq_pop) err <= 1'b1;
            end
            if (sq_pop) sq_rd <= sq_rd + 2'd1;
            sq_n <= sq_n + {2'd0, sq_push} - {2'd0, sq_pop};
        end
    end

    // ------------------------------------------------------------------ engine state
    localparam [2:0] S_IDLE = 3'd0, S_FILL = 3'd1, S_CORR = 3'd2, S_DRAIN = 3'd3, S_DEC = 3'd4,
                     S_OUT = 3'd5;
    reg  [2:0]  st;
    reg  [31:0] s_t, s_lo, s_hi, s_n0;
    reg  [23:0] s_inc;
    reg  [15:0] s_npos;
    reg  [31:0] rp;                         // next sample to fetch
    reg  [23:0] ph;
    wire        active = (st == S_FILL) || (st == S_CORR);

    // stall: a sample still needed would be overwritten
    wire [31:0] age_pt = wp - (pt[pt_rd] + {16'd0, n_cfo} - 32'd40);
    wire [31:0] age_sq = wp - (sq_t[sq_rd] + {16'd0, n_cfo} - 32'd40);
    wire [31:0] age_rp = wp - rp;
    always @(posedge clk) begin
        if (!rst_n || !en) stall <= 1'b0;
        else stall <= ((pt_n != 3'd0) && !age_pt[31] && (age_pt >= LIM)) ||
                      ((sq_n != 3'd0) && (age_sq >= LIM)) ||
                      (active && (age_rp >= LIM));
    end

    // ------------------------------------------------------------------ fetch + de-rotate
    // c0 address -> c1 buffer + table -> c2 products -> c3 sums -> c4 saturated x, A, B into the skid FIFO
    reg  [3:0]  skid_n;                     // entries in the skid FIFO
    reg  [2:0]  infl;                       // fetches in the pipeline
    wire        avail = $signed(rp - wp) < 0;
    wire        fetch = active && avail && (rp != s_hi + 32'd1) && ({1'b0, skid_n} + {2'b0, infl} < 5'd6);
    wire signed [15:0] cos_d, sin_d;
    pd_tone_rom u_rom (.clk(clk), .addr_a(ph[23:14]), .addr_b(ph[23:14] - 10'd256),
                       .dout_a(cos_d), .dout_b(sin_d));
    reg  [4:1]  pv;
    reg  signed [27:0] m_ic, m_qs, m_is, m_qc;
    reg  signed [29:0] s_r, s_i;
    reg  signed [5:0]  x_r, x_i;
    function signed [5:0] sat6;
        input signed [29:0] v;              // already shifted
        begin
            if (v > 30'sd31)       sat6 = 6'sd31;
            else if (v < -30'sd32) sat6 = -6'sd32;
            else                   sat6 = v[5:0];
        end
    endfunction
    wire signed [29:0] s_r_sh = s_r >>> 21;
    wire signed [29:0] s_i_sh = s_i >>> 21;
    always @(*) b_ra = rp[DB-1:0];
    always @(posedge clk) begin
        if (!rst_n) pv <= 5'd0;
        else        pv <= {pv[3:1], fetch};
        m_ic <= $signed(b_rd[11:0]) * cos_d;
        m_qs <= $signed(b_rd[23:12]) * sin_d;
        m_is <= $signed(b_rd[11:0]) * sin_d;
        m_qc <= $signed(b_rd[23:12]) * cos_d;
        s_r  <= m_ic - m_qs + 30'sd1048576;
        s_i  <= m_is + m_qc + 30'sd1048576;
        x_r  <= sat6(s_r_sh);
        x_i  <= sat6(s_i_sh);
    end
    // skid FIFO of {A, B}
    reg  [2*AW-1:0] skid [0:7];
    reg  [2:0]      sk_wr, sk_rd;
    wire            sk_push = pv[4];
    wire signed [AW-1:0] a_new = {x_r[5], x_r} + {x_i[5], x_i};
    wire signed [AW-1:0] b_new = {x_r[5], x_r} - {x_i[5], x_i};
    reg             sk_pop;
    wire [2*AW-1:0] sk_head = skid[sk_rd];

    // ------------------------------------------------------------------ lane storage and correlator control
    (* max_fanout = 32 *) reg [2:0] k;      // phase (read address of all 228 lane RAMs)
    reg  [7:0]   wl;                        // next write: lane
    reg  [2:0]   wd;                        //             depth
    reg  [NL-1:0] wsel;                     // one-hot wl
    reg  [10:0]  fill_n;
    reg  [31:0]  n_out;                     // output being computed
    reg  [LEN-1:0] r_ni, r_nq;              // rotating signs (1 = -1)
    reg          w_done;                    // the sample of this output block is written
    wire         need_w = (k == wd) && !w_done && (n_out != s_hi);
    wire         sk_any = (skid_n != 4'd0);
    reg          rd_v, rd_first, rd_last;   // a phase read this clock
    reg          lw;                        // write the skid head this clock
    always @(*) begin
        sk_pop = 1'b0; lw = 1'b0; rd_v = 1'b0;
        if (st == S_FILL && sk_any) begin
            sk_pop = 1'b1; lw = 1'b1;
        end else if (st == S_CORR && !(need_w && !sk_any)) begin
            rd_v = 1'b1;
            if (need_w) begin sk_pop = 1'b1; lw = 1'b1; end
        end
        rd_first = (k == 3'd0);
        rd_last  = (k == 3'd4);
    end

    // lane writes go through one register stage: the 228 RAMs are far apart, so address,
    // data and enable must not come straight from one flip-flop each. A slot written in
    // clock c is next read at least 4 clocks later (the next output block reads the other
    // depths first; at the end of the fill the last slot written has depth 4), so the extra
    // clock changes nothing.
    (* max_fanout = 32 *) reg [2:0]      w_addr_q;
    (* max_fanout = 32 *) reg [2*AW-1:0] w_data_q;
    reg  [NL-1:0] w_en_q;
    always @(posedge clk) begin
        if (!rst_n) w_en_q <= {NL{1'b0}};
        else        w_en_q <= lw ? wsel : {NL{1'b0}};
        w_addr_q <= wd;
        w_data_q <= sk_head;
    end

    // lanes: LUT RAM per lane (8 x 14), read address = phase, terms registered
    reg  signed [AW-1:0] t_re [0:NL-1];
    reg  signed [AW-1:0] t_im [0:NL-1];
    genvar u;
    generate
        for (u = 0; u < NL; u = u + 1) begin : ln
            (* ram_style = "distributed" *) reg [2*AW-1:0] mem [0:7];
            always @(posedge clk) if (w_en_q[u]) mem[w_addr_q] <= w_data_q;
            wire [2*AW-1:0] d  = mem[k];
            wire signed [AW-1:0] da = d[2*AW-1:AW];
            wire signed [AW-1:0] db = d[AW-1:0];
            wire ni = (k == 3'd0) ? r_ni[u] : (k == 3'd1) ? r_ni[u + NL] : (k == 3'd2) ? r_ni[u + 2 * NL] :
                      (k == 3'd3) ? r_ni[u + 3 * NL] : r_ni[u + 4 * NL];
            wire nq = (k == 3'd0) ? r_nq[u] : (k == 3'd1) ? r_nq[u + NL] : (k == 3'd2) ? r_nq[u + 2 * NL] :
                      (k == 3'd3) ? r_nq[u + 3 * NL] : r_nq[u + 4 * NL];
            wire eq = (ni == nq);
            wire signed [AW-1:0] xs = eq ? da : db;
            wire signed [AW-1:0] ys = eq ? db : da;
            always @(posedge clk) begin
                t_re[u] <= xs ^ {AW{ni}};
                t_im[u] <= ys ^ {AW{!nq}};
            end
        end
    endgenerate

    // engine
    reg  [3:0]  dec_i;
    reg  [71:0] lhs, rhs;
    reg  [33:0] p_max;
    reg  [47:0] p_sum;
    reg  [31:0] n_max;
    reg  [15:0] outs;                       // outputs seen by the tracker
    reg         trk_done;
    reg         sq_take;
    always @(*) sq_pop = sq_take;
    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; rp <= 32'd0; ph <= 24'd0; k <= 3'd0; wl <= 8'd0; wd <= 3'd0;
            wsel <= {{(NL-1){1'b0}}, 1'b1}; fill_n <= 11'd0; n_out <= 32'd0; sq_take <= 1'b0;
            skid_n <= 4'd0; sk_wr <= 3'd0; sk_rd <= 3'd0; infl <= 3'd0;
            tm_valid <= 1'b0; bz_v <= 1'b0; dec_i <= 4'd0;
        end else begin
            sq_take  <= 1'b0;
            tm_valid <= 1'b0;
            // fetch pipeline bookkeeping
            if (fetch) begin rp <= rp + 32'd1; ph <= ph + s_inc; end
            infl <= infl + {2'd0, fetch} - {2'd0, pv[4]};
            if (sk_push) begin skid[sk_wr] <= {a_new, b_new}; sk_wr <= sk_wr + 3'd1; end
            if (sk_pop) sk_rd <= sk_rd + 3'd1;
            skid_n <= skid_n + {3'd0, sk_push} - {3'd0, sk_pop};
            if (lw) begin
                if (wl == NL - 1) begin
                    wl <= 8'd0; wsel <= {{(NL-1){1'b0}}, 1'b1};
                    wd <= (wd == NK - 1) ? 3'd0 : wd + 3'd1;
                end else begin
                    wl <= wl + 8'd1; wsel <= {wsel[NL-2:0], 1'b0};
                end
            end
            // BUSY records from the decision, kept until the output is free
            if (e_ok && e_busy) begin bz_v <= 1'b1; bz_t <= ev_t; end
            case (st)
                S_IDLE: begin
                    if (sq_n != 3'd0 && !sq_take) begin
                        sq_take <= 1'b1;
                        s_t   <= sq_t[sq_rd];
                        s_inc <= sq_inc[sq_rd];
                        s_n0  <= sq_t[sq_rd] + {16'd0, n_cfo} - 32'd40;
                        s_lo  <= sq_t[sq_rd] + {16'd0, n_cfo} + 32'd1099;
                        s_hi  <= sq_t[sq_rd] + 32'd6163;
                        s_npos <= 16'd5065 - n_cfo;
                        rp    <= sq_t[sq_rd] + {16'd0, n_cfo} - 32'd40;
                        ph    <= 24'd0;
                        r_ni  <= NEG_I; r_nq <= NEG_Q;
                        k <= 3'd0; wl <= 8'd0; wd <= 3'd0; wsel <= {{(NL-1){1'b0}}, 1'b1};
                        fill_n <= 11'd0;
                        st <= S_FILL;
                    end
                end
                S_FILL: begin
                    if (lw) begin
                        fill_n <= fill_n + 11'd1;
                        if (fill_n == LEN - 1) begin st <= S_CORR; n_out <= s_lo; k <= 3'd0; w_done <= 1'b0; end
                    end
                end
                S_CORR: begin
                    if (lw) w_done <= 1'b1;
                    if (rd_v) begin
                        if (k == NK - 1) begin
                            k <= 3'd0;
                            w_done <= 1'b0;
                            r_ni <= {r_ni[LEN-2:0], r_ni[LEN-1]};
                            r_nq <= {r_nq[LEN-2:0], r_nq[LEN-1]};
                            if (n_out == s_hi) st <= S_DRAIN;
                            else n_out <= n_out + 32'd1;
                        end else
                            k <= k + 3'd1;
                    end
                end
                S_DRAIN: begin
                    if (trk_done) begin
                        st <= S_DEC; dec_i <= 4'd0;
                        lhs <= 72'd0; rhs <= 72'd0;
                    end
                end
                S_DEC: begin                // lhs = p_max * n_pos, rhs = t_pk * p_sum (16 steps)
                    if (s_npos[dec_i]) lhs <= lhs + ({38'd0, p_max} << dec_i);
                    if (t_pk[dec_i])   rhs <= rhs + ({24'd0, p_sum} << dec_i);
                    dec_i <= dec_i + 4'd1;
                    if (dec_i == 4'd15) st <= S_OUT;
                end
                S_OUT: begin
                    if (!bz_v) begin
                        tm_valid  <= 1'b1;
                        tm_t      <= s_t;
                        tm_n_end  <= n_max;
                        tm_status <= {1'b0, $signed(n_max - (s_t + {16'd0, n_cfo} + 32'd1131)) < 0,
                                      (t_pk == 16'd0) || ((p_max != 34'd0) && ((lhs << 8) >= rhs))};
                        tm_n_pos  <= s_npos;
                        tm_p_max  <= p_max;
                        tm_p_sum  <= p_sum;
                        st <= S_IDLE;
                    end
                end
                default: st <= S_IDLE;
            endcase
            if (bz_v && !(e_ok && e_busy)) begin
                tm_valid  <= 1'b1;
                tm_t      <= bz_t;
                tm_n_end  <= 32'd0;
                tm_status <= 3'b100;
                tm_n_pos  <= 16'd0;
                tm_p_max  <= 34'd0;
                tm_p_sum  <= 48'd0;
                bz_v      <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------ adder trees (256 inputs, 8 levels)
    wire signed [TW-1:0] sum_re, sum_im;
    wire [NL*AW-1:0] flat_re, flat_im;
    pd_addtree #(.W(AW), .N(NL)) u_tre (.clk(clk), .in_flat(flat_re), .sum(sum_re));
    pd_addtree #(.W(AW), .N(NL)) u_tim (.clk(clk), .in_flat(flat_im), .sum(sum_im));
    generate
        for (u = 0; u < NL; u = u + 1) begin : fl
            assign flat_re[u*AW +: AW] = t_re[u];
            assign flat_im[u*AW +: AW] = t_im[u];
        end
    endgenerate
    // tags: term stage (1) + tree (8)
    localparam integer TL = 9;
    reg  [TL-1:0] tg_v, tg_f, tg_l;
    always @(posedge clk) begin
        if (!rst_n) begin tg_v <= 0; tg_f <= 0; tg_l <= 0; end
        else begin
            tg_v <= {tg_v[TL-2:0], rd_v};
            tg_f <= {tg_f[TL-2:0], rd_first};
            tg_l <= {tg_l[TL-2:0], rd_last};
        end
    end
    // accumulate the 5 phases, add the negation constants
    reg  signed [YW-1:0] acc_re, acc_im, y_re, y_im;
    reg                  y_v;
    always @(posedge clk) begin
        if (!rst_n) y_v <= 1'b0;
        else begin
            y_v <= 1'b0;
            if (tg_v[TL-1]) begin
                if (tg_l[TL-1]) begin
                    y_re <= acc_re + sum_re + C_RE;
                    y_im <= acc_im + sum_im + C_IM;
                    y_v  <= 1'b1;
                end else if (tg_f[TL-1]) begin
                    acc_re <= sum_re; acc_im <= sum_im;
                end else begin
                    acc_re <= acc_re + sum_re; acc_im <= acc_im + sum_im;
                end
            end
        end
    end
    // power and tracker
    reg  [35:0] q_re, q_im;
    reg         q_v, pw_v;
    reg  [33:0] pw;
    always @(posedge clk) begin
        if (!rst_n) begin q_v <= 1'b0; pw_v <= 1'b0; outs <= 16'd0; trk_done <= 1'b0; end
        else begin
            q_v  <= y_v;
            q_re <= y_re * y_re;
            q_im <= y_im * y_im;
            pw_v <= q_v;
            pw   <= q_re[33:0] + q_im[33:0];
            if (st == S_IDLE) begin outs <= 16'd0; trk_done <= 1'b0; end
            if (pw_v) begin
                if (outs == 16'd0 || pw > p_max) begin p_max <= pw; n_max <= s_lo + {16'd0, outs}; end
                p_sum <= (outs == 16'd0) ? {14'd0, pw} : p_sum + {14'd0, pw};
                outs  <= outs + 16'd1;
                if (outs + 16'd1 == s_npos) trk_done <= 1'b1;
            end
        end
    end

    // busy: something to do that does not wait for new samples
    wire starving = active && !sk_any && (infl == 3'd0) && !avail &&
                    ((st == S_FILL) || need_w);
    assign busy = ((st == S_IDLE) ? (sq_n != 3'd0) : !starving) || bz_v || (tg_v != 0) || y_v || q_v || pw_v;
endmodule

// Pipelined adder tree: N signed W-bit inputs, padded to 256, sum after 8 clocks.
module pd_addtree #(
    parameter integer W = 7,
    parameter integer N = 228
) (
    input  wire                  clk,
    input  wire [N*W-1:0]        in_flat,
    output wire signed [W+7:0]   sum
);
    genvar l, j;
    generate
        for (l = 0; l < 8; l = l + 1) begin : lv
            localparam integer NN = 256 >> (l + 1);
            reg signed [W+l:0] s [0:NN-1];
            for (j = 0; j < NN; j = j + 1) begin : nd
                if (l == 0 && 2 * j + 1 < N) begin : two
                    wire signed [W-1:0] a = in_flat[(2 * j) * W +: W];
                    wire signed [W-1:0] b = in_flat[(2 * j + 1) * W +: W];
                    always @(posedge clk) s[j] <= a + b;
                end else if (l == 0 && 2 * j < N) begin : one
                    wire signed [W-1:0] a = in_flat[(2 * j) * W +: W];
                    always @(posedge clk) s[j] <= a;
                end else if (l == 0) begin : none
                    always @(posedge clk) s[j] <= 0;
                end else begin : next
                    always @(posedge clk) s[j] <= lv[l - 1].s[2 * j] + lv[l - 1].s[2 * j + 1];
                end
            end
        end
    endgenerate
    assign sum = lv[7].s[0];
endmodule

`default_nettype wire
