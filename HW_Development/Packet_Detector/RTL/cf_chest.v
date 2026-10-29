// cf_chest.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Control-field channel estimate, soft values, descrambling, copy combining and Viterbi input
// scaling, bit-true to cf_reference.channel / decode_soft. Reads the LS pilots (pilot RAMs of
// symbols 0 and 3, 280 x {re, im} 19-bit, sign already applied) and the grid (Y, 18-bit,
// address {s, k}) that cf_rx wrote. Phases after start:
//   DRIFT  D = sum_j P3(j) conj(P0(j)) exact (46-bit), phi = CORDIC angle (pd_cordic, IW 46),
//          d = rnd(phi * 21845 >> 16)
//   ANGLE  cos/sin (pd_tone_rom, address = angle[15:6]) of -phi and of s * d, s = 0..3
//   FIR    per tone k = 0..840, taps i = 0..7 (start, set from cf_fir_map_rom):
//            Hf_s(k) = rnd(sum_i c_i P_s(start + i) >> 14), s = 0, 3
//            H3'(k)  = rnd(Hf_3 (cos + j sin)(-phi) >> 15)      -> Hf RAM {Hf0, H3'}
//   SOFT   s = 0..3, k = 0..840, data tones only, in that order (data index i = 0..2199):
//            u = (3 - s) Hf0 + s H3', m = rnd(u * 10923 >> 15), H = rnd(m (cos + j sin)(s d) >> 15)
//            v = Yr Hr + Yi Hi, negated where cf_gold_rom(i) = 1;
//            i < 140: A[i mod 140] = v, i < 2100: A[i mod 140] += v
//   SCALE  sh = max(bitlen(max |A|) - 4, 0); code bit p = A[deint(p)] >>> sh (5-bit) -> q_*
// Then done (sh on shift). About 300 + 40 + 6730 + 3370 + 150 clocks.
// The dbg_* outputs only feed the testbench (unconnected in the design, so removed).
`timescale 1ns / 1ps
`default_nettype none

module cf_chest (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    output wire               busy,
    output reg                done,
    output reg  [5:0]         shift,
    // pilot RAMs (one clock read latency): {re[37:19], im[18:0]}
    output reg  [8:0]         p_addr,
    input  wire [37:0]        p0_data,
    input  wire [37:0]        p3_data,
    // grid RAM (one clock): {re[35:18], im[17:0]} at {s, k}
    output reg  [11:0]        g_addr,
    input  wire [35:0]        g_data,
    // Viterbi soft values
    output reg                q_we,
    output reg  [7:0]         q_addr,
    output reg  signed [4:0]  q_data,
    // testbench only
    output reg  signed [15:0] dbg_phi,
    output wire               dbg_hf_we,
    output wire [9:0]         dbg_hf_k,
    output wire [79:0]        dbg_hf,
    output reg                dbg_v_we,
    output reg  [11:0]        dbg_v_i,
    output reg  signed [45:0] dbg_v
);
    localparam [3:0] S_IDLE = 4'd0, S_DRIFT = 4'd1, S_DWAIT = 4'd2, S_CORD = 4'd3, S_ANG = 4'd4,
                     S_FIR = 4'd5, S_FWAIT = 4'd6, S_SOFT = 4'd7, S_SWAIT = 4'd8, S_SCALE = 4'd9,
                     S_DONE = 4'd10;
    reg [3:0] state;
    reg [9:0] cnt;
    reg [2:0] tap;
    reg [1:0] sym;
    assign busy = (state != S_IDLE);

    // ================================================================ DRIFT
    reg               dv1, dv2;
    reg signed [18:0] p0r, p0i, p3r, p3i;
    reg signed [37:0] m_rr, m_ii, m_ir, m_ri;
    reg signed [45:0] d_re, d_im;
    always @(posedge clk) begin
        dv1 <= (state == S_DRIFT);
        dv2 <= dv1;
        // dv1: pilot data valid
        p0r <= p0_data[37:19]; p0i <= p0_data[18:0];
        p3r <= p3_data[37:19]; p3i <= p3_data[18:0];
        // dv2: operands registered -> products
        m_rr <= p3r * p0r; m_ii <= p3i * p0i; m_ir <= p3i * p0r; m_ri <= p3r * p0i;
    end
    reg dv3;
    always @(posedge clk) begin
        dv3 <= dv2;
        if (state == S_IDLE) begin d_re <= 46'sd0; d_im <= 46'sd0; end
        else if (dv3) begin
            d_re <= d_re + m_rr + m_ii;
            d_im <= d_im + m_ir - m_ri;
        end
    end

    wire               c_done;
    wire signed [15:0] c_angle;
    reg                c_start;
    pd_cordic #(.IW(46)) u_cordic (.clk(clk), .rst_n(rst_n), .start(c_start), .re(d_re), .im(d_im),
                                   .busy(), .done(c_done), .angle(c_angle));

    reg signed [15:0] phi, dd;
    wire signed [31:0] phi_x = phi * 32'sd21845 + 32'sd32768;

    // ================================================================ ANGLE (tone ROM)
    reg  [9:0] t_addr_a, t_addr_b;
    wire signed [15:0] t_cos, t_sin;
    pd_tone_rom u_tone (.clk(clk), .addr_a(t_addr_a), .addr_b(t_addr_b), .dout_a(t_cos), .dout_b(t_sin));
    // angle index 0: -phi, 1..4: s * d, s = 0..3
    wire [1:0]  ang_sym = cnt[1:0] - 2'd1;     // cnt 1..4 -> s = 0..3
    wire signed [17:0] ang_s = $signed({1'b0, ang_sym}) * $signed({{2{dd[15]}}, dd});
    wire signed [16:0] mphi = -$signed({phi[15], phi});
    wire [15:0] ang16 = (cnt == 10'd0) ? mphi[15:0] : ang_s[15:0];
    always @* begin
        t_addr_a = ang16[15:6];
        t_addr_b = ang16[15:6] - 10'd256;       // sin(a) = cos(a - quarter turn)
    end
    reg signed [15:0] cm, sm;                  // -phi
    reg signed [15:0] cs [0:3], ss [0:3];      // s * d
    reg [2:0] ang_k;
    reg       ang_v;
    always @(posedge clk) begin
        ang_v <= (state == S_ANG) && (cnt < 10'd5);
        ang_k <= cnt[2:0];
        if (ang_v) begin
            if (ang_k == 3'd0) begin cm <= t_cos; sm <= t_sin; end
            else begin cs[ang_k - 3'd1] <= t_cos; ss[ang_k - 3'd1] <= t_sin; end
        end
    end

    // ================================================================ FIR
    // stage A (counters cnt = k, tap) -> map ROM; B: pilot / coef addresses; C: data;
    // D: products; E: accumulate; F: round; G: rotate products; H: rotate sums -> Hf RAM
    wire [13:0] map_q;
    cf_fir_map_rom u_map (.clk(clk), .addr(cnt), .q(map_q));
    reg        fa_v, fb_v, fc_v, fd_v, fe_v, ff_v, fg_v;
    reg  [2:0] fa_t, fb_t, fc_t, fd_t;
    reg  [9:0] fa_k, fb_k, fc_k, fd_k, fe_k, ff_k, fg_k;
    always @(posedge clk) begin
        fa_v <= (state == S_FIR); fa_t <= tap; fa_k <= cnt;
        fb_v <= fa_v; fb_t <= fa_t; fb_k <= fa_k;
        fc_v <= fb_v; fc_t <= fb_t; fc_k <= fb_k;
        fd_v <= fc_v; fd_t <= fc_t; fd_k <= fc_k;
        if (!rst_n) begin fa_v <= 1'b0; fb_v <= 1'b0; fc_v <= 1'b0; fd_v <= 1'b0; end
    end
    wire [8:0] fir_paddr = map_q[13:5] + {6'd0, fa_t};
    wire [7:0] coef_addr = {map_q[4:0], fa_t};
    wire [31:0] coef_q;
    cf_fir_coef_rom u_coef (.clk(clk), .addr(coef_addr), .q(coef_q));
    // C: operands
    reg signed [15:0] cr, ci;
    reg signed [18:0] a0r, a0i, a3r, a3i;
    always @(posedge clk) begin
        cr <= coef_q[31:16]; ci <= coef_q[15:0];
        a0r <= p0_data[37:19]; a0i <= p0_data[18:0];
        a3r <= p3_data[37:19]; a3i <= p3_data[18:0];
    end
    // D: products
    reg signed [34:0] q0rr, q0ii, q0ri, q0ir, q3rr, q3ii, q3ri, q3ir;
    always @(posedge clk) begin
        q0rr <= cr * a0r; q0ii <= ci * a0i; q0ri <= cr * a0i; q0ir <= ci * a0r;
        q3rr <= cr * a3r; q3ii <= ci * a3i; q3ri <= cr * a3i; q3ir <= ci * a3r;
    end
    // E: accumulate (tap 0 restarts)
    reg signed [39:0] acc0r, acc0i, acc3r, acc3i;
    wire signed [39:0] s0r = q0rr - q0ii, s0i = q0ri + q0ir, s3r = q3rr - q3ii, s3i = q3ri + q3ir;
    reg  e_last;
    always @(posedge clk) begin
        fe_v <= fd_v && (fd_t == 3'd7); fe_k <= fd_k;
        if (fd_v) begin
            acc0r <= (fd_t == 3'd0) ? s0r : acc0r + s0r;
            acc0i <= (fd_t == 3'd0) ? s0i : acc0i + s0i;
            acc3r <= (fd_t == 3'd0) ? s3r : acc3r + s3r;
            acc3i <= (fd_t == 3'd0) ? s3i : acc3i + s3i;
        end
        if (!rst_n) fe_v <= 1'b0;
    end
    // F: round >> 14
    function signed [19:0] rnd14;
        input signed [39:0] v;
        reg   signed [39:0] r;
        begin
            r = (v + 40'sd8192) >>> 14;
            rnd14 = r[19:0];
        end
    endfunction
    reg signed [19:0] hf0r, hf0i, hf3r, hf3i;
    always @(posedge clk) begin
        ff_v <= fe_v; ff_k <= fe_k;
        if (fe_v) begin
            hf0r <= rnd14(acc0r); hf0i <= rnd14(acc0i);
            hf3r <= rnd14(acc3r); hf3i <= rnd14(acc3i);
        end
        if (!rst_n) ff_v <= 1'b0;
    end
    // G: rotation products by (cm, sm)
    reg signed [35:0] r_rc, r_is, r_rs, r_ic;
    reg signed [19:0] g0r, g0i;
    always @(posedge clk) begin
        fg_v <= ff_v; fg_k <= ff_k;
        r_rc <= hf3r * cm; r_is <= hf3i * sm; r_rs <= hf3r * sm; r_ic <= hf3i * cm;
        g0r <= hf0r; g0i <= hf0i;
        if (!rst_n) fg_v <= 1'b0;
    end
    function signed [19:0] rnd15_36;
        input signed [36:0] v;
        reg   signed [36:0] r;
        begin
            r = (v + 37'sd16384) >>> 15;
            rnd15_36 = r[19:0];
        end
    endfunction
    wire signed [36:0] h3x_r = {r_rc[35], r_rc} - {r_is[35], r_is};
    wire signed [36:0] h3x_i = {r_rs[35], r_rs} + {r_ic[35], r_ic};
    reg        hf_we;
    reg  [9:0] hf_wk;
    reg  [79:0] hf_wd;
    always @(posedge clk) begin
        hf_we <= fg_v; hf_wk <= fg_k;
        hf_wd <= {g0r, g0i, rnd15_36(h3x_r), rnd15_36(h3x_i)};
        if (!rst_n) hf_we <= 1'b0;
    end
    reg  [9:0]  hf_rk;
    wire [79:0] hf_rd;
    pd_sdp_ram #(.W(80), .A(10)) u_hf (.clk(clk), .we(hf_we), .waddr(hf_wk), .wdata(hf_wd),
                                       .raddr(hf_rk), .rdata(hf_rd));

    // ================================================================ SOFT
    // c0: addresses (state S_SOFT, sym, cnt = k); c1: Y, Hf; c2: u; c3: u * 10923;
    // c4: m; c5: rotation products; c6: H; c7: Y H* products (+ gold ROM); c8: v; c9: A update
    wire is_dc_ref = (cnt == 10'd0) || (cnt == 10'd414) || (cnt == 10'd417) || (cnt == 10'd420) ||
                     (cnt == 10'd423) || (cnt == 10'd426) || (cnt == 10'd840);
    reg  [1:0] k_mod3;
    wire is_data = (sym == 2'd0 || sym == 2'd3) ? ((k_mod3 == 2'd0) && !is_dc_ref)
                                                : !((cnt >= 10'd414) && (cnt <= 10'd428));
    reg  [11:0] di;                            // data index of the next data tone
    reg  [7:0]  dj;                            // di mod 140
    reg  [9:0]  sv;                            // valid shift
    reg  [1:0]  s_t  [1:3];
    reg  [11:0] i_t  [1:8];
    reg  [7:0]  j_t  [1:8];
    integer x;
    always @(posedge clk) begin
        sv[0] <= (state == S_SOFT) && is_data;
        i_t[1] <= di; j_t[1] <= dj; s_t[1] <= sym;
        for (x = 1; x < 10; x = x + 1) sv[x] <= sv[x-1];
        for (x = 2; x < 9; x = x + 1) begin i_t[x] <= i_t[x-1]; j_t[x] <= j_t[x-1]; end
        s_t[2] <= s_t[1]; s_t[3] <= s_t[2];
        if (!rst_n) sv <= 10'd0;
    end
    // c1 (sv[0]): Y and Hf valid
    wire signed [19:0] h0r = hf_rd[79:60], h0i = hf_rd[59:40], h3r = hf_rd[39:20], h3i = hf_rd[19:0];
    function signed [21:0] mix;               // (3 - s) a + s b
        input [1:0] s;
        input signed [19:0] a;
        input signed [19:0] b;
        reg signed [21:0] ea, eb;
        begin
            ea = {{2{a[19]}}, a}; eb = {{2{b[19]}}, b};
            case (s)
                2'd0: mix = (ea <<< 1) + ea;
                2'd1: mix = (ea <<< 1) + eb;
                2'd2: mix = ea + (eb <<< 1);
                default: mix = (eb <<< 1) + eb;
            endcase
        end
    endfunction
    reg signed [21:0] u_r, u_i;
    reg signed [17:0] y_r [2:7];
    reg signed [17:0] y_i [2:7];
    always @(posedge clk) begin
        u_r <= mix(s_t[1], h0r, h3r);
        u_i <= mix(s_t[1], h0i, h3i);
        y_r[2] <= g_data[35:18]; y_i[2] <= g_data[17:0];
        for (x = 3; x < 8; x = x + 1) begin y_r[x] <= y_r[x-1]; y_i[x] <= y_i[x-1]; end
    end
    // c2 -> c3: u * 10923
    reg signed [36:0] um_r, um_i;
    always @(posedge clk) begin
        um_r <= u_r * 37'sd10923; um_i <= u_i * 37'sd10923;
    end
    // c3 -> c4: m (21 bits)
    reg signed [20:0] mm_r, mm_i;
    reg signed [15:0] rc, rs;
    always @(posedge clk) begin
        mm_r <= rnd15_36(um_r) ;
        mm_i <= rnd15_36(um_i);
        rc <= cs[s_t[3]]; rs <= ss[s_t[3]];
    end
    // c4 -> c5: rotation products
    reg signed [36:0] w_rc, w_is, w_rs, w_ic;
    always @(posedge clk) begin
        w_rc <= mm_r * rc; w_is <= mm_i * rs; w_rs <= mm_r * rs; w_ic <= mm_i * rc;
    end
    // c5 -> c6: H
    reg signed [20:0] hh_r, hh_i;
    always @(posedge clk) begin
        hh_r <= rnd15_36(w_rc - w_is);
        hh_i <= rnd15_36(w_rs + w_ic);
    end
    // c6 -> c7: Y H* products, gold ROM read (addr i at c6)
    reg signed [38:0] yh_rr, yh_ii;
    wire gold_q;
    cf_gold_rom u_gold (.clk(clk), .addr(i_t[6]), .q(gold_q));
    always @(posedge clk) begin
        yh_rr <= y_r[6] * hh_r; yh_ii <= y_i[6] * hh_i;
    end
    // c7 -> c8: v, descrambled
    reg signed [45:0] vv;
    always @(posedge clk) begin
        vv <= gold_q ? -({{7{yh_rr[38]}}, yh_rr} + {{7{yh_ii[38]}}, yh_ii})
                     :  ({{7{yh_rr[38]}}, yh_rr} + {{7{yh_ii[38]}}, yh_ii});
    end
    // c8: combine
    (* ram_style = "distributed" *) reg signed [45:0] acc_mem [0:255];
    reg [45:0] amax;
    wire        c8_v = sv[7];                  // sv[x] is valid in cycle c(x + 1)
    wire signed [45:0] a_old = acc_mem[j_t[8]];
    wire signed [45:0] a_new = (i_t[8] < 12'd140) ? vv : a_old + vv;
    // c9: |A| of the last copy's (final) values; c10: running maximum
    reg               c9_v, c10_v;
    reg signed [45:0] a9;
    reg        [45:0] a_abs;
    always @(posedge clk) begin
        if (c8_v && i_t[8] < 12'd2100) acc_mem[j_t[8]] <= a_new;
        c9_v  <= c8_v && i_t[8] >= 12'd1960 && i_t[8] < 12'd2100;
        a9    <= a_new;
        c10_v <= c9_v;
        a_abs <= a9[45] ? -a9 : a9;
        if (state == S_FIR) amax <= 46'd0;
        else if (c10_v && a_abs > amax) amax <= a_abs;
        dbg_v_we <= c8_v; dbg_v_i <= i_t[8]; dbg_v <= vv;
        if (!rst_n) begin c9_v <= 1'b0; c10_v <= 1'b0; end
    end

    // ================================================================ SCALE
    function [5:0] bitlen46;
        input [45:0] m;
        integer b;
        begin
            bitlen46 = 6'd0;
            for (b = 0; b < 46; b = b + 1) if (m[b]) bitlen46 = b[5:0] + 6'd1;
        end
    endfunction
    wire [5:0] blen = bitlen46(amax);
    wire [7:0] deint_q;
    cf_deint_rom u_deint (.clk(clk), .addr(cnt[7:0]), .q(deint_q));
    reg        sc_v;
    reg [7:0]  sc_p;
    wire signed [45:0] a_sel = acc_mem[deint_q];
    wire signed [45:0] a_shift = a_sel >>> shift;
    always @(posedge clk) begin
        sc_v <= (state == S_SCALE) && (cnt < 10'd140);
        sc_p <= cnt[7:0];
        q_we <= sc_v; q_addr <= sc_p; q_data <= a_shift[4:0];
        if (!rst_n) begin sc_v <= 1'b0; q_we <= 1'b0; end
    end

    // ================================================================ control
    always @* begin
        p_addr = (state == S_DRIFT) ? cnt[8:0] : fir_paddr;
        g_addr = {sym, cnt};
        hf_rk  = cnt;
    end
    assign dbg_hf_we = hf_we;
    assign dbg_hf_k  = hf_wk;
    assign dbg_hf    = hf_wd;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; c_start <= 1'b0;
        end else begin
            done <= 1'b0; c_start <= 1'b0;
            case (state)
                S_IDLE: if (start) begin cnt <= 10'd0; state <= S_DRIFT; end
                S_DRIFT: if (cnt == 10'd279) begin cnt <= 10'd0; state <= S_DWAIT; end
                         else cnt <= cnt + 10'd1;
                S_DWAIT: if (cnt == 10'd4) begin c_start <= 1'b1; state <= S_CORD; end
                         else cnt <= cnt + 10'd1;
                S_CORD: if (c_done) begin
                            phi <= c_angle; dbg_phi <= c_angle; cnt <= 10'd0; state <= S_ANG;
                        end
                S_ANG: begin                    // cnt 0: d from phi; ROM reads at cnt 1..5
                    if (cnt == 10'd0) dd <= phi_x[31:16];
                    if (cnt == 10'd7) begin cnt <= 10'd0; tap <= 3'd0; state <= S_FIR; end
                    else cnt <= cnt + 10'd1;
                end
                S_FIR: begin
                    tap <= tap + 3'd1;
                    if (tap == 3'd7) begin
                        if (cnt == 10'd840) begin cnt <= 10'd0; state <= S_FWAIT; end
                        else cnt <= cnt + 10'd1;
                    end
                end
                S_FWAIT: if (cnt == 10'd12) begin
                             cnt <= 10'd0; sym <= 2'd0; k_mod3 <= 2'd0; di <= 12'd0; dj <= 8'd0;
                             state <= S_SOFT;
                         end else cnt <= cnt + 10'd1;
                S_SOFT: begin
                    if (is_data) begin
                        di <= di + 12'd1;
                        dj <= (dj == 8'd139) ? 8'd0 : dj + 8'd1;
                    end
                    k_mod3 <= (k_mod3 == 2'd2) ? 2'd0 : k_mod3 + 2'd1;
                    if (cnt == 10'd840) begin
                        cnt <= 10'd0; k_mod3 <= 2'd0;
                        if (sym == 2'd3) state <= S_SWAIT;
                        sym <= sym + 2'd1;
                    end else cnt <= cnt + 10'd1;
                end
                S_SWAIT: if (cnt == 10'd12) begin
                             cnt <= 10'd0;
                             shift <= (blen > 6'd4) ? blen - 6'd4 : 6'd0;
                             state <= S_SCALE;
                         end else cnt <= cnt + 10'd1;
                S_SCALE: if (cnt == 10'd143) state <= S_DONE;
                         else cnt <= cnt + 10'd1;
                S_DONE: begin done <= 1'b1; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
