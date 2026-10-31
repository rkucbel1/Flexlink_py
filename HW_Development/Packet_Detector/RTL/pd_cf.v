// pd_cf.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Control field after a good Preamble B timing record (Docs/Specification_ControlField.md),
// bit-true to cf_reference.control_records: for every timing record with status == PEAK_OK only
// (not LATE, not BUSY), in record order, cf_rx decodes the 4560 samples y(n_end + 1 ...) of the
// DC blocker output with the event's cfo_inc and gives one control record.
//
// Samples: every DC blocker output goes into a circular buffer of 2^DB samples (block RAM),
// indexed by sample number, as in pd_timing. The control field is read from there once its
// timing record is out, so it can start long after the samples arrived. Good timing records
// queue (4 deep; one control field at a time). `stall` holds the input back while a sample still
// needed (the next one of the running control field, the first one of a queued one) would be
// overwritten. Samples of a timing search not finished yet are safe without that: pd_timing keeps
// them within 2^12 - 64 samples of the input, and this buffer is twice as long.
// At 20 Msps (5 clocks per sample) nothing stalls: a control field starts at most ~5000 samples
// behind the input and falls ~700 further behind while it runs (the FFTs stop its input).
//
// Speculative payload FFT (pl_en, i.e. re-arm by the Rx state machine, so packets cannot
// overlap): cf_rx carries on with the payload symbols after the control field (see there); each
// symbol's 841 bins give a signature (cf_reference.grid_signature) that goes into the payload
// FIFO (16 records). Records of symbols started before the control field was decided are
// pending: a good decision keeps the first PacketLength of them, a rejected one drops them all;
// after a good decision new records are kept at once (and symbols >= PacketLength, which can
// only be in flight, are dropped). A payload symbol starts only while the FIFO has room, so
// nothing is lost if the firmware is slow (the sample input is then held by the stall).
// Payload FIFO head: pl_cnt kept records, pl_head = {sig_b, sig_a, 25'b0 sym, t_trig},
// pl_pop removes it.
`timescale 1ns / 1ps
`default_nettype none

module pd_cf #(
    parameter integer DB     = 13,       // sample buffer 2^DB samples
    parameter integer MARGIN = 64
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,          // input_stage 1 or 2
    input  wire               pl_en,       // speculative payload FFT (Params.rearm)
    // DC blocker output
    input  wire               y_valid,
    input  wire signed [11:0] y_i,
    input  wire signed [11:0] y_q,
    // timing records
    input  wire               tm_valid,
    input  wire [31:0]        tm_t,
    input  wire [31:0]        tm_n_end,
    input  wire [2:0]         tm_status,
    input  wire [23:0]        tm_inc,
    // back-pressure towards the sample input
    output reg                stall,
    // control record
    output wire               cf_valid,
    output wire [31:0]        cf_t,
    output wire [31:0]        cf_n_end,
    output wire               cf_crc_ok,
    output wire [53:0]        cf_msg,
    output wire [5:0]         cf_shift,
    output wire signed [15:0] cf_metric,
    output wire               busy,        // work in progress that does not wait for samples
    output reg                err,         // start queue overflow (sticky; cannot happen)
    // payload records
    output wire [4:0]         pl_cnt,
    output wire [127:0]       pl_head,
    input  wire               pl_pop,
    // channel-estimate records (pl_chest, step 12b), as the payload records
    output wire [4:0]         ch_cnt,
    output wire [127:0]       ch_head,
    input  wire               ch_pop,
    output wire               ch_err,      // pl_chest's packet queue overflowed (sticky; cannot happen)
    // decoder-input records (step 12c): per code word {sig_b, sig_a, 22'b0 w, t_trig}
    output wire [4:0]         lq_cnt,
    output wire [127:0]       lq_head,
    input  wire               lq_pop,
    // transport blocks (step 12d): FIFO words of complete records (pl_dec)
    output wire [11:0]        tb_cnt,
    output wire [31:0]        tb_word,
    input  wire               tb_pop
);
    localparam integer DEPTH = 1 << DB;
    localparam [31:0]  LIM   = DEPTH - MARGIN;

    // ------------------------------------------------------------------ sample buffer
    reg  [31:0]   wp;
    wire          y_in = en && y_valid;
    reg  [31:0]   rp;                       // next sample to fetch
    wire [23:0]   b_rd;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(24), .A(DB)) u_buf (.clk(clk), .we(y_in), .waddr(wp[DB-1:0]),
        .wdata({y_q, y_i}), .raddr(rp[DB-1:0]), .rdata(b_rd));
    always @(posedge clk) begin
        if (!rst_n) wp <= 32'd0;
        else if (y_in) wp <= wp + 32'd1;
    end

    // ------------------------------------------------------------------ start queue: t, n_end, cfo_inc
    reg  [31:0] q_t [0:3];
    reg  [31:0] q_e [0:3];
    reg  [31:0] q_s [0:3];                  // n_end + 1, the first sample (for the stall check)
    reg  [23:0] q_inc [0:3];
    reg  [1:0]  q_wr, q_rd;
    reg  [2:0]  q_n;
    wire        q_push = en && tm_valid && (tm_status == 3'b001);
    reg         q_pop;
    always @(posedge clk) begin
        if (!rst_n) begin
            q_wr <= 2'd0; q_rd <= 2'd0; q_n <= 3'd0; err <= 1'b0;
        end else begin
            if (q_push) begin
                q_t[q_wr] <= tm_t; q_e[q_wr] <= tm_n_end; q_inc[q_wr] <= tm_inc; q_s[q_wr] <= tm_n_end + 32'd1;
                q_wr <= q_wr + 2'd1;
                if (q_n == 3'd4 && !q_pop) err <= 1'b1;
            end
            if (q_pop) q_rd <= q_rd + 2'd1;
            q_n <= q_n + {2'd0, q_push} - {2'd0, q_pop};
        end
    end

    // ------------------------------------------------------------------ reader
    reg         active;
    wire        rx_more;                    // cf_rx will still take samples
    wire        rx_busy;
    reg  [1:0]  pv;                         // fetch -> buffer data (one clock) -> FIFO
    reg  [2:0]  infl;
    reg  [23:0] ff [0:3];
    reg  [1:0]  ff_wr, ff_rd;
    reg  [2:0]  ff_n;
    wire        avail = $signed(rp - wp) < 0;
    wire        fetch = active && rx_more && avail && ({1'b0, ff_n} + {1'b0, infl} < 4'd4);
    wire        cf_ready;
    wire        ff_pop = (ff_n != 3'd0) && cf_ready;
    reg         cf_start;
    wire        rec_v;
    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 1'b0; q_pop <= 1'b0; cf_start <= 1'b0; pv <= 2'd0; infl <= 3'd0;
            ff_wr <= 2'd0; ff_rd <= 2'd0; ff_n <= 3'd0; rp <= 32'd0;
        end else begin
            q_pop <= 1'b0; cf_start <= 1'b0;
            if (!active && q_n != 3'd0 && !q_pop && infl == 3'd0 && pv == 2'd0) begin
                // new job: samples prefetched for the last one are dropped
                active <= 1'b1; q_pop <= 1'b1; cf_start <= 1'b1;
                rp <= q_s[q_rd];
                ff_wr <= 2'd0; ff_rd <= 2'd0; ff_n <= 3'd0;
            end else begin
                if (active && !rx_busy) active <= 1'b0;
                if (fetch) rp <= rp + 32'd1;
                if (pv[0]) begin ff[ff_wr] <= b_rd; ff_wr <= ff_wr + 2'd1; end
                if (ff_pop) ff_rd <= ff_rd + 2'd1;
                ff_n <= ff_n + {2'd0, pv[0]} - {2'd0, ff_pop};
            end
            pv   <= {pv[0], fetch};
            infl <= infl + {2'd0, fetch} - {2'd0, pv[0]};
        end
    end
    wire [23:0] ff_head = ff[ff_rd];

    // stall: a sample still needed would be overwritten
    wire [31:0] age_rp = wp - rp;
    wire [31:0] age_q  = wp - q_s[q_rd];
    always @(posedge clk) begin
        if (!rst_n || !en) stall <= 1'b0;
        else stall <= (active && rx_more && !age_rp[31] && (age_rp >= LIM)) ||
                      ((q_n != 3'd0) && !age_q[31] && (age_q >= LIM));
    end

    // ------------------------------------------------------------------ receiver
    wire               pg_valid, pl_dec, pl_ok;
    wire [6:0]         pg_sym, pl_len;
    wire [9:0]         pg_col;
    wire signed [17:0] pg_re, pg_im;
    wire               pl_room, ch_room;
    wire               ch_busy, ch_wait;           // pl_chest (below)
    wire               dm_busy, cb_busy, dc_busy;  // pl_demap, pl_comb, pl_dec (below)
    wire               jb_req, jb_gnt, jb_we, jb_start, jb_done;
    wire [9:0]         jb_idx, jb_oaddr;
    wire signed [17:0] jb_re, jb_im, jb_ore, jb_oim;
    cf_rx u_rx (.clk(clk), .rst_n(rst_n), .start(cf_start), .cfo_inc(q_inc[q_rd]), .t_trig(q_t[q_rd]),
                .n_end(q_e[q_rd]), .busy(rx_busy), .pl_en(pl_en), .pl_room(pl_room), .s_more(rx_more),
                .pg_valid(pg_valid), .pg_sym(pg_sym), .pg_col(pg_col), .pg_re(pg_re), .pg_im(pg_im),
                .pl_dec(pl_dec), .pl_ok(pl_ok), .pl_len(pl_len),
                .job_req(jb_req), .job_gnt(jb_gnt), .job_we(jb_we), .job_idx(jb_idx), .job_re(jb_re),
                .job_im(jb_im), .job_start(jb_start), .job_oaddr(jb_oaddr), .job_done(jb_done),
                .job_ore(jb_ore), .job_oim(jb_oim),
                .s_valid(ff_n != 3'd0), .s_ready(cf_ready), .s_i(ff_head[11:0]), .s_q(ff_head[23:12]),
                .rec_valid(rec_v), .rec_crc_ok(cf_crc_ok), .rec_msg(cf_msg), .rec_shift(cf_shift),
                .rec_metric(cf_metric), .rec_t_trig(cf_t), .rec_n_end(cf_n_end),
                .dbg_grid_we(), .dbg_grid_addr(), .dbg_grid_data(), .dbg_phi(), .dbg_hf_we(), .dbg_hf_k(),
                .dbg_hf(), .dbg_v_we(), .dbg_v_i(), .dbg_v(), .dbg_bits());
    assign cf_valid = rec_v;

    // busy: something to do that does not wait for new samples
    wire starving = active && rx_more && cf_ready && (ff_n == 3'd0) && (infl == 3'd0) && !avail;
    assign busy = (active && !starving) || (!active && q_n != 3'd0) || q_pop || cf_start ||
                  (pl_en && ((ch_busy && !ch_wait) || dm_busy || cb_busy || dc_busy));

    // ------------------------------------------------------------------ payload signatures
    // x = re0, im0, re1, im1, ... (18 bit unsigned each): A += x, B += A per value; two values
    // per bin: A' = A + re + im, B' = B + 2 A + 2 re + im
    // (the bins are registered first: they come straight from the FFT's block RAM)
    reg         gv;
    reg  [9:0]  gc;
    reg  [6:0]  gs;
    reg  [17:0] gr, gi;
    always @(posedge clk) begin
        gv <= pg_valid && rst_n; gc <= pg_col; gs <= pg_sym; gr <= pg_re; gi <= pg_im;
    end
    reg  [31:0] sg_a, sg_b;
    wire [31:0] ur = {14'd0, gr};
    wire [31:0] ui = {14'd0, gi};
    wire        first = (gc == 10'd0);
    wire [31:0] a0 = first ? 32'd0 : sg_a;
    wire [31:0] b0 = first ? 32'd0 : sg_b;
    wire [31:0] a1 = a0 + ur + ui;
    wire [31:0] b1 = b0 + {a0[30:0], 1'b0} + {ur[30:0], 1'b0} + ui;
    reg         sg_w;                       // signature of the symbol complete
    reg  [6:0]  sg_s;
    always @(posedge clk) begin
        sg_w <= 1'b0;
        if (gv) begin
            sg_a <= a1; sg_b <= b1;
            if (gc == 10'd840) begin sg_w <= 1'b1; sg_s <= gs; end
        end
        if (!rst_n) sg_w <= 1'b0;
    end

    // ------------------------------------------------------------------ payload FIFO
    // [rd, cm) kept, [cm, wr) pending (before the decision)
    reg  [127:0] pl_mem [0:15];
    reg  [4:0]   pl_wr, pl_cm, pl_rd;
    reg  [31:0]  j_t;                       // t_trig of the running job
    reg          j_dec, j_ok;
    reg  [6:0]   j_len;
    wire [4:0]   n_pend = pl_wr - pl_cm;
    wire         w_take = sg_w && (!j_dec || (j_ok && sg_s < j_len));   // write this record
    wire [4:0]   n_pw   = n_pend + {4'd0, w_take && !j_dec};             // pending incl. it
    wire [4:0]   keep   = ({5'd0, pl_len} < {2'd0, n_pw}) ? pl_len[4:0] : n_pw;
    always @(posedge clk) begin
        if (w_take) pl_mem[pl_wr[3:0]] <= {sg_b, sg_a, 25'd0, sg_s, j_t};
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            pl_wr <= 5'd0; pl_cm <= 5'd0; pl_rd <= 5'd0; j_dec <= 1'b0;
        end else begin
            if (cf_start) begin j_t <= q_t[q_rd]; j_dec <= 1'b0; end
            if (pl_dec) begin
                j_dec <= 1'b1; j_ok <= pl_ok; j_len <= pl_len;
                if (pl_ok) begin pl_cm <= pl_cm + keep; pl_wr <= pl_cm + keep; end
                else pl_wr <= pl_cm;
            end else if (w_take) begin
                pl_wr <= pl_wr + 5'd1;
                if (j_dec) pl_cm <= pl_cm + 5'd1;
            end
            if (pl_pop && pl_cm != pl_rd) pl_rd <= pl_rd + 5'd1;
        end
    end
    assign pl_room = ((pl_wr - pl_rd) < 5'd16) && (!pl_en || ch_room);
    assign pl_cnt  = pl_cm - pl_rd;
    assign pl_head = pl_mem[pl_rd[3:0]];

    // ------------------------------------------------------------------ channel estimate (12b)
    wire        pk_v, h_v;
    wire [31:0] pk_t, pk_a, pk_b, h_t;
    wire [6:0]  h_s;
    wire [9:0]  h_c;
    wire signed [15:0] h_r, h_i, hy_r, hy_i;
    wire        h_rf, h_tk;
    wire [53:0] pk_m;
    wire [47:0] pk_sn;
    wire        ch_out_room;
    pl_chest u_ch (.clk(clk), .rst_n(rst_n), .pg_valid(pg_valid), .pg_sym(pg_sym), .pg_col(pg_col),
                   .pg_re(pg_re), .pg_im(pg_im), .pl_dec(pl_dec && pl_en), .pl_ok(pl_ok), .pl_msg(cf_msg), .pl_t(j_t),
                   .room(ch_room), .job_req(jb_req), .job_gnt(jb_gnt), .job_we(jb_we), .job_idx(jb_idx),
                   .job_re(jb_re), .job_im(jb_im), .job_start(jb_start), .fft_done(jb_done),
                   .job_oaddr(jb_oaddr), .fft_ore(jb_ore), .fft_oim(jb_oim), .out_room(ch_out_room),
                   .pk_valid(pk_v), .pk_t(pk_t), .pk_a(pk_a), .pk_b(pk_b), .h_valid(h_v), .h_t(h_t),
                   .h_sym(h_s), .h_col(h_c), .h_re(h_r), .h_im(h_i), .y_re(hy_r), .y_im(hy_i),
                   .h_ref(h_rf), .h_take(h_tk), .pk_msg(pk_m), .pk_sumn(pk_sn),
                   .busy(ch_busy), .waiting(ch_wait), .err(ch_err));
    // per-symbol signature of H (as the bins: x = re0, im0, ... 18 bit each)
    reg  [31:0] hs_a, hs_b;
    wire [31:0] hr = {14'd0, h_r[15], h_r[15], h_r};
    wire [31:0] hi = {14'd0, h_i[15], h_i[15], h_i};
    wire        hfirst = (h_c == 10'd0);
    wire [31:0] ha0 = hfirst ? 32'd0 : hs_a;
    wire [31:0] hb0 = hfirst ? 32'd0 : hs_b;
    wire [31:0] ha1 = ha0 + hr + hi;
    wire [31:0] hb1 = hb0 + {ha0[30:0], 1'b0} + {hr[30:0], 1'b0} + hi;
    reg         hs_w;
    reg  [6:0]  hs_s;
    reg  [31:0] hs_t;
    always @(posedge clk) begin
        hs_w <= 1'b0;
        if (h_v) begin
            hs_a <= ha1; hs_b <= hb1;
            if (h_c == 10'd840) begin hs_w <= 1'b1; hs_s <= h_s; hs_t <= h_t; end
        end
        if (!rst_n) hs_w <= 1'b0;
    end
    // channel-record FIFO: packet record {sumN, a, 0x8000, t}, symbol record {sig_b, sig_a, sym, t}
    reg  [127:0] ch_mem [0:15];
    reg  [4:0]   ch_wr, ch_rd;
    always @(posedge clk) begin
        if (pk_v) ch_mem[ch_wr[3:0]] <= {pk_b, pk_a, 32'h0000_8000, pk_t};
        else if (hs_w) ch_mem[ch_wr[3:0]] <= {hs_b, hs_a, 25'd0, hs_s, hs_t};
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            ch_wr <= 5'd0; ch_rd <= 5'd0;
        end else begin
            if (pk_v || hs_w) ch_wr <= ch_wr + 5'd1;
            if (ch_pop && ch_wr != ch_rd) ch_rd <= ch_rd + 5'd1;
        end
    end
    assign ch_out_room = (ch_wr - ch_rd) < 5'd15;
    assign ch_cnt  = ch_wr - ch_rd;
    assign ch_head = ch_mem[ch_rd[3:0]];

    // ------------------------------------------------------------------ demapper, combiner (12c)
    wire        lf_v, lf_s, lf_p;
    wire signed [11:0] lf_0, lf_1;
    wire [1:0]  cg_nc, cg_rt;
    wire [4:0]  cg_rep;
    wire [9:0]  cg_ndb;
    wire [7:0]  cg_ci;
    wire [31:0] cg_t;
    pl_demap u_dm (.clk(clk), .rst_n(rst_n), .pk_valid(pk_v), .pk_t(pk_t), .pk_msg(pk_m), .pk_n0(pk_a[25:16]),
                   .pk_sumn(pk_sn), .h_valid(h_v), .h_sym(h_s), .h_col(h_c), .h_ref(h_rf), .h_re(h_r), .h_im(h_i),
                   .y_re(hy_r), .y_im(hy_i), .h_take(h_tk), .lf_valid(lf_v), .lf_sop(lf_s), .lf_l0(lf_0), .lf_l1(lf_1),
                   .lf_pop(lf_p), .cfg_ncode(cg_nc), .cfg_rate(cg_rt), .cfg_rep(cg_rep), .cfg_ndb(cg_ndb), .cfg_cinit(cg_ci),
                   .cfg_t(cg_t), .busy(dm_busy));
    wire        rc_v;
    wire [31:0] rc_t, rc_a, rc_b;
    wire [9:0]  rc_w;
    wire        lq_room;
    wire        dq_v, dq_f, dq_l, dq_rdy;
    wire signed [6:0] dq_0, dq_1;
    wire [3:0]  dq_code;
    wire [9:0]  dq_w;
    wire [31:0] dq_t;
    pl_comb u_cb (.clk(clk), .rst_n(rst_n), .lf_valid(lf_v), .lf_sop(lf_s), .lf_l0(lf_0), .lf_l1(lf_1),
                  .lf_pop(lf_p), .cfg_ncode(cg_nc), .cfg_rate(cg_rt), .cfg_rep(cg_rep), .cfg_ndb(cg_ndb), .cfg_cinit(cg_ci),
                  .cfg_t(cg_t), .lq_ready(dq_rdy), .lq_valid(dq_v), .lq_first(dq_f), .lq_last(dq_l), .lq_v(), .lq_q0(dq_0),
                  .lq_q1(dq_1), .lq_code(dq_code), .lq_w(dq_w), .lq_t(dq_t),
                  .rc_room(lq_room), .rc_valid(rc_v), .rc_t(rc_t), .rc_w(rc_w), .rc_a(rc_a), .rc_b(rc_b),
                  .busy(cb_busy));
    // LDPC decoder, CRC, transport-block FIFO (12d)
    pl_dec u_dc (.clk(clk), .rst_n(rst_n), .lq_valid(dq_v), .lq_first(dq_f), .lq_last(dq_l), .lq_q0(dq_0), .lq_q1(dq_1),
                 .lq_code(dq_code), .lq_w(dq_w), .lq_t(dq_t), .lq_ready(dq_rdy), .tb_cnt(tb_cnt), .tb_word(tb_word),
                 .tb_pop(tb_pop), .busy(dc_busy));
    // decoder-input record FIFO
    reg  [127:0] lq_mem [0:15];
    reg  [4:0]   lq_wr, lq_rd;
    always @(posedge clk) if (rc_v) lq_mem[lq_wr[3:0]] <= {rc_b, rc_a, 22'd0, rc_w, rc_t};
    always @(posedge clk) begin
        if (!rst_n) begin
            lq_wr <= 5'd0; lq_rd <= 5'd0;
        end else begin
            if (rc_v) lq_wr <= lq_wr + 5'd1;
            if (lq_pop && lq_wr != lq_rd) lq_rd <= lq_rd + 5'd1;
        end
    end
    assign lq_room = (lq_wr - lq_rd) < 5'd15;
    assign lq_cnt  = lq_wr - lq_rd;
    assign lq_head = lq_mem[lq_rd[3:0]];
endmodule

`default_nettype wire
