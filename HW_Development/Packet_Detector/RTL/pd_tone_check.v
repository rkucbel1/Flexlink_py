// pd_tone_check.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Tone-structure check (spec 4.7), bit-true to pd_reference.tone_check and the abort decision
// in pd_reference.detect. After the CFO lane delivers f_hat for the window t .. t+N-1:
//   13 single-bin DFTs over the window, bin b at f_b = (f_hat + m_b * 2^18) mod 2^24 (NCO units,
//   Fs / 2^24; 2^18 = 312.5 kHz), m = -6 -2 2 6 (Preamble A tones), 0 4 -4 1 -1 5 3 -3 -5 (refs):
//     phi(0) = 0, phi(n+1) = phi(n) + f_b (mod 2^24), a = phi[23:14]
//     X_b = sum x(n) (COS[a] - j SIN[a]),  SIN[a] = COS[a - 256]      (exact, 41-bit)
//   common floor shift sh so the largest |Re X|, |Im X| fits 18 bits signed; P_b = Re'^2 + Im'^2
//   num = 9 (sum P_tone - max P_tone), den = sum P_ref,  fail = lin != 0 && (num << 16) < lin * den
// The firmware converts TONE_THRESH (dB, Q8.8) to lin (Q.16) and num / den to ToneCheck in dB.
//
// The window has ended before f_hat is known, so the LPF stream is written into a circular
// buffer of 2^DB samples (indexed by sample number, the t_trig count) and the window is read
// back from it. NU DFT units run in parallel, one sample per clock each, so a check takes
// ceil(13 / NU) passes of N clocks (NU = 4: 4 N + about 40 clocks).
//
// One check runs at a time, plus one CFO result waiting. At the real sample rate (20 Msps on a
// 100 MHz clock: 5 clocks per sample, windows >= N samples = 5 N clocks apart) a check always
// finishes before the next CFO result, so the input is never stalled. When samples come faster
// (DMA from memory, one per clock), `stall` holds the input back while
//   - a CFO result is waiting and a check is running, or
//   - the buffer would overwrite a sample of a window not yet checked (MARGIN covers the
//     samples still inside the front end).
// `stall` is registered; it only depends on this block's state.
// Results are in order: one event per window (ev_valid), carrying the CFO result with it.
`timescale 1ns / 1ps
`default_nettype none

module pd_tone_check #(
    parameter integer NU     = 4,       // DFT units (bins computed in parallel), 1..13
    parameter integer DB     = 13,      // sample buffer 2^DB samples; needs 2^DB - MARGIN > N + 64
    parameter integer MARGIN = 64
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [15:0]        n_cfo,       // window length N, 128..4096
    input  wire [23:0]        tone_lin,    // threshold, linear power ratio Q.16; 0 = check off
    // LPF stream (the detector input)
    input  wire               in_valid,
    input  wire signed [11:0] in_i,
    input  wire signed [11:0] in_q,
    // trigger and CFO result
    input  wire               trig,
    input  wire [31:0]        t_trig,
    input  wire               cfo_valid,
    input  wire [31:0]        cfo_t,
    input  wire signed [31:0] cfo_inc,
    input  wire signed [15:0] ang16,
    input  wire signed [15:0] ang32,
    input  wire signed [15:0] ang128,
    input  wire signed [15:0] ang256,
    // back-pressure request towards the sample input
    output reg                stall,
    // event: one per window
    output reg                ev_valid,
    output reg  [31:0]        ev_t,
    output reg  signed [31:0] ev_cfo_inc,
    output reg  signed [15:0] ev_ang16,
    output reg  signed [15:0] ev_ang32,
    output reg  signed [15:0] ev_ang128,
    output reg  signed [15:0] ev_ang256,
    output reg  [39:0]        ev_num,
    output reg  [39:0]        ev_den,
    output reg                ev_fail,
    // state, for the end of a run
    output wire               busy,        // a check running or waiting, or an event going out
    output wire [3:0]         open_n,      // windows triggered but not checked yet
    output wire [31:0]        open_t       // the oldest of them (valid when open_n != 0)
);
    localparam integer NB    = 13;
    localparam integer NP    = (NB + NU - 1) / NU;       // passes
    localparam integer DEPTH = 1 << DB;
    integer u;

    // bin offsets in units of 2^18 (model order: TONE_OFFSETS, then NOISE_REF_OFFSETS)
    function [5:0] bin_m;
        input integer b;
        begin
            case (b)
                0: bin_m = -6;  1: bin_m = -2;  2: bin_m = 2;   3: bin_m = 6;
                4: bin_m = 0;   5: bin_m = 4;   6: bin_m = -4;  7: bin_m = 1;  8: bin_m = -1;
                9: bin_m = 5;  10: bin_m = 3;  11: bin_m = -3; 12: bin_m = -5;
                default: bin_m = 0;
            endcase
        end
    endfunction

    function [40:0] abs41;
        input signed [40:0] v;
        begin
            abs41 = v[40] ? -v : v;
        end
    endfunction

    // ------------------------------------------------------------------ sample buffer
    // block RAM in its own kept hierarchy: Vivado would otherwise move the read register into
    // the DFT DSPs and fall back to LUT RAM
    reg [31:0]   wcnt;                         // samples written = index of the next sample
    reg [DB-1:0] ra;                           // read address
    wire [23:0]  rdata;
    reg  [23:0]  rdata2;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(24), .A(DB)) u_sbuf (
        .clk(clk), .we(in_valid), .waddr(wcnt[DB-1:0]), .wdata({in_q, in_i}),
        .raddr(ra), .rdata(rdata));
    always @(posedge clk) rdata2 <= rdata;
    always @(posedge clk) begin
        if (!rst_n)        wcnt <= 32'd0;
        else if (in_valid) wcnt <= wcnt + 32'd1;
    end
    wire signed [11:0] x_i = rdata2[11:0];
    wire signed [11:0] x_q = rdata2[23:12];

    // ------------------------------------------------------------------ outstanding windows
    // Every trigger stays in this FIFO until its check is done; the oldest one says which
    // samples the buffer must keep.
    reg  [31:0] tf [0:7];
    reg  [2:0]  tf_wr, tf_rd;
    reg  [3:0]  tf_n;
    reg         job_done;
    wire [31:0] keep = tf[tf_rd];
    wire [31:0] age  = wcnt - keep;
    wire        stall_buf = (tf_n != 4'd0) && (age >= DEPTH - MARGIN);
    always @(posedge clk) begin
        if (!rst_n) begin
            tf_wr <= 3'd0; tf_rd <= 3'd0; tf_n <= 4'd0;
        end else begin
            if (trig) begin tf[tf_wr] <= t_trig; tf_wr <= tf_wr + 3'd1; end
            if (job_done) tf_rd <= tf_rd + 3'd1;
            tf_n <= tf_n + {3'd0, trig} - {3'd0, job_done};
        end
    end

    // ------------------------------------------------------------------ waiting CFO result
    reg               pv;
    reg  [31:0]       p_t;
    reg  signed [31:0] p_inc;
    reg  signed [15:0] p_a16, p_a32, p_a128, p_a256;

    // ------------------------------------------------------------------ engine control
    localparam [3:0] S_IDLE = 4'd0, S_SETUP = 4'd1, S_RUN = 4'd2, S_WAIT = 4'd3, S_SH = 4'd4,
                     S_PWR = 4'd5, S_PWAIT = 4'd6, S_NUM = 4'd7, S_MUL = 4'd8, S_MUL2 = 4'd9,
                     S_CMP = 4'd10;
    reg  [3:0]        st;
    reg  [31:0]       a_t;
    reg  [23:0]       a_f;                     // f_hat mod 2^24
    reg  signed [31:0] a_inc;
    reg  signed [15:0] a_a16, a_a32, a_a128, a_a256;
    reg  [3:0]        pass;
    reg  [12:0]       n;
    wire [12:0]       n_last = n_cfo[12:0] - 13'd1;
    wire              setup = (st == S_SETUP);
    wire              run   = (st == S_RUN);

    // registered (one clock late: MARGIN and the window spacing cover the extra sample)
    always @(posedge clk) begin
        if (!rst_n) stall <= 1'b0;
        else        stall <= (pv && st != S_IDLE) || stall_buf;
    end

    // pipeline flags: stage 1 = buffer / ROM data, 2 = registered data, 3 = products,
    // 4 = sums, 5 = accumulators updated
    reg v1, v2, v3, v4, f1, f2, f3, f4, l1, l2, l3, l4, l5;
    always @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0; l1 <= 1'b0; l2 <= 1'b0; l3 <= 1'b0;
            l4 <= 1'b0; l5 <= 1'b0;
        end else begin
            v1 <= run; f1 <= (n == 13'd0); l1 <= run && (n == n_last);
            v2 <= v1;  f2 <= f1;  l2 <= l1;
            v3 <= v2;  f3 <= f2;  l3 <= l2;
            v4 <= v3;  f4 <= f3;  l4 <= l3;
            l5 <= l4;
        end
    end

    // ------------------------------------------------------------------ DFT units
    wire [NU*41-1:0] acc_re, acc_im;
    genvar g;
    generate
        for (g = 0; g < NU; g = g + 1) begin : dft
            reg  [23:0]        f, phi;
            wire signed [15:0] c1, s1;
            reg  signed [15:0] c2, s2;
            reg  signed [27:0] p_ic, p_qs, p_qc, p_is;
            reg  signed [28:0] sr, si;
            reg  signed [40:0] ar, ai;
            pd_tone_rom u_rom (.clk(clk), .addr_a(phi[23:14]), .addr_b(phi[23:14] - 10'd256),
                               .dout_a(c1), .dout_b(s1));
            always @(posedge clk) begin
                if (setup) begin
                    f   <= a_f + {bin_m(pass * NU + g), 18'd0};
                    phi <= 24'd0;
                end else if (run) begin
                    phi <= phi + f;
                end
                c2 <= c1; s2 <= s1;
                // x(n) (c - j s) = (xi c + xq s) + j (xq c - xi s)
                p_ic <= x_i * c2;  p_qs <= x_q * s2;
                p_qc <= x_q * c2;  p_is <= x_i * s2;
                sr   <= p_ic + p_qs;
                si   <= p_qc - p_is;
                if (v4) begin
                    ar <= (f4 ? 41'sd0 : ar) + sr;
                    ai <= (f4 ? 41'sd0 : ai) + si;
                end
            end
            assign acc_re[g*41 +: 41] = ar;
            assign acc_im[g*41 +: 41] = ai;
        end
    endgenerate

    // ------------------------------------------------------------------ results and powers
    reg  signed [40:0] res_re [0:NB-1];
    reg  signed [40:0] res_im [0:NB-1];
    reg  [40:0]        or_mag;                 // OR of all |Re|, |Im|: bit length = that of the max
    reg  [4:0]         sh;
    reg  [3:0]         k;                      // power stage: bin issued
    reg                q1, q2, q3;
    reg  [3:0]         k1, k2, k3;
    reg  signed [17:0] rr, ri;
    reg  [35:0]        sq_r, sq_i;
    reg  [35:0]        pw;
    reg  [37:0]        tsum;
    reg  [35:0]        tmax;
    reg  [38:0]        rsum;
    reg                pdone;
    reg  [39:0]        num;
    reg  [39:0]        den;
    reg  [63:0]        prod, prod2;
    integer            b, bl, uo;
    reg  [40:0]        or_new;                 // |Re|, |Im| of this pass, ORed
    wire signed [40:0] sel_re = res_re[k];
    wire signed [40:0] sel_im = res_im[k];

    always @(*) begin
        bl = 0;
        for (b = 0; b < 41; b = b + 1)
            if (or_mag[b]) bl = b + 1;
        or_new = 41'd0;
        for (uo = 0; uo < NU; uo = uo + 1)
            if (pass * NU + uo < NB)
                or_new = or_new | abs41(acc_re[uo*41 +: 41]) | abs41(acc_im[uo*41 +: 41]);
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; pv <= 1'b0; ev_valid <= 1'b0; job_done <= 1'b0;
            q1 <= 1'b0; q2 <= 1'b0; q3 <= 1'b0; pdone <= 1'b0;
        end else begin
            ev_valid <= 1'b0;
            job_done <= 1'b0;
            if (cfo_valid) begin
                pv <= 1'b1; p_t <= cfo_t; p_inc <= cfo_inc;
                p_a16 <= ang16; p_a32 <= ang32; p_a128 <= ang128; p_a256 <= ang256;
            end

            // power pipeline: shift, square, sum, accumulate
            q1 <= (st == S_PWR);  k1 <= k;
            rr <= sel_re >>> sh;  ri <= sel_im >>> sh;
            q2 <= q1;  k2 <= k1;
            sq_r <= rr * rr;      sq_i <= ri * ri;
            q3 <= q2;  k3 <= k2;
            pw <= sq_r + sq_i;
            pdone <= 1'b0;
            if (q3) begin
                if (k3 < 4'd4) begin
                    tsum <= tsum + {2'd0, pw};
                    if (pw > tmax) tmax <= pw;
                end else
                    rsum <= rsum + {3'd0, pw};
                pdone <= (k3 == NB - 1);
            end

            case (st)
                S_IDLE:
                    if (pv) begin
                        a_t <= p_t; a_f <= -p_inc[23:0]; a_inc <= p_inc;
                        a_a16 <= p_a16; a_a32 <= p_a32; a_a128 <= p_a128; a_a256 <= p_a256;
                        if (!cfo_valid) pv <= 1'b0;
                        pass <= 4'd0; or_mag <= 41'd0;
                        st <= S_SETUP;
                    end
                S_SETUP: begin
                    ra <= a_t[DB-1:0];
                    n  <= 13'd0;
                    st <= S_RUN;
                end
                S_RUN: begin
                    ra <= ra + 1'b1;
                    n  <= n + 13'd1;
                    if (n == n_last) st <= S_WAIT;
                end
                S_WAIT:
                    if (l5) begin
                        for (u = 0; u < NU; u = u + 1)
                            if (pass * NU + u < NB) begin
                                res_re[pass * NU + u] <= acc_re[u*41 +: 41];
                                res_im[pass * NU + u] <= acc_im[u*41 +: 41];
                            end
                        or_mag <= or_mag | or_new;
                        if (pass == NP - 1) st <= S_SH;
                        else begin pass <= pass + 4'd1; st <= S_SETUP; end
                    end
                S_SH: begin
                    sh   <= (bl > 17) ? bl - 17 : 5'd0;
                    tsum <= 38'd0; tmax <= 36'd0; rsum <= 39'd0;
                    k    <= 4'd0;
                    st   <= S_PWR;
                end
                S_PWR: begin
                    k <= k + 4'd1;
                    if (k == NB - 1) st <= S_PWAIT;
                end
                S_PWAIT:
                    if (pdone) st <= S_NUM;
                S_NUM: begin
                    num <= ({2'd0, tsum - {2'd0, tmax}} << 3) + {2'd0, tsum - {2'd0, tmax}};
                    den <= {1'b0, rsum};
                    st  <= S_MUL;
                end
                S_MUL: begin
                    prod <= tone_lin * den;
                    st   <= S_MUL2;
                end
                S_MUL2: begin
                    prod2 <= prod;
                    st    <= S_CMP;
                end
                default: begin   // S_CMP
                    ev_valid   <= 1'b1;
                    ev_t       <= a_t;
                    ev_cfo_inc <= a_inc;
                    ev_ang16 <= a_a16; ev_ang32 <= a_a32; ev_ang128 <= a_a128; ev_ang256 <= a_a256;
                    ev_num     <= num;
                    ev_den     <= den;
                    ev_fail    <= (tone_lin != 24'd0) && ({8'd0, num, 16'd0} < prod2);
                    job_done   <= 1'b1;
                    st         <= S_IDLE;
                end
            endcase
        end
    end

    assign busy   = (st != S_IDLE) || pv || ev_valid;
    assign open_n = tf_n;
    assign open_t = keep;

`ifdef PD_SIM_CHECKS
    always @(posedge clk) begin
        if (rst_n && cfo_valid && pv && !(st == S_IDLE))
            $display("ERROR pd_tone_check: CFO result for t %0d lost (a check is running and one is waiting)", cfo_t);
        if (rst_n && in_valid && tf_n != 4'd0 && age >= DEPTH)
            $display("ERROR pd_tone_check: sample %0d overwrites window %0d before its check", wcnt, keep);
        if (rst_n && trig && tf_n == 4'd8 && !job_done)
            $display("ERROR pd_tone_check: more than 8 windows outstanding");
    end
`endif
endmodule

`default_nettype wire
