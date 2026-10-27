// pd_cfo_lane.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Coarse CFO lane (spec 4.3), bit-true to pd_reference (cfo_accumulators, cordic_angle,
// cfo_cascade). After a trigger at sample t:
//   A_l = s_l * sum_{n = t+l}^{t+N-1} x(n) x*(n-l),  l = 16 / 32 / 128 / 256, s_16 = -1
//   (both samples of every product inside the window t .. t+N-1; N = n_cfo, 1..4096)
//   angle_l = CORDIC(A_l) (16-bit); integer cascade in NCO units (Fs / 2^24):
//     f = 16 ang_16;  for l = 32, 128, 256:
//       f_l = ang_l * 256 / l;  k = (f - f_l + step/2) >>> log2(step);  f = f_l + k * step
//     (step = 2^24 / l = 2^19, 2^17, 2^16);  cfo_inc = -f
//
// The lane works on the LPF sample stream delayed by LANE_DELAY samples, so the trigger for
// sample t (which the detection core decides about 12 clocks after sample t) is known before
// sample t reaches the lane. Consequence: a window is only complete once LANE_DELAY further
// samples have arrived after it. One window at a time; the detection core guarantees
// t_next >= t + N (holdoff >= n_cfo). Results: cfo_valid pulse with cfo_t (= t_trig), cfo_inc
// (sign-extended to 32 bits), the four angles and, for verification, the four accumulators.
`timescale 1ns / 1ps
`default_nettype none

module pd_cfo_lane #(
    parameter integer LANE_DELAY = 32
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [15:0]        n_cfo,
    // LPF stream (the detection core's input)
    input  wire               in_valid,
    input  wire signed [11:0] in_i,
    input  wire signed [11:0] in_q,
    // trigger from the detection core
    input  wire               trig,
    input  wire [31:0]        t_trig,
    // result
    output reg                cfo_valid,
    output reg  [31:0]        cfo_t,
    output reg  signed [31:0] cfo_inc,
    output reg  signed [15:0] ang16,
    output reg  signed [15:0] ang32,
    output reg  signed [15:0] ang128,
    output reg  signed [15:0] ang256,
    output reg  signed [36:0] a16_re, a16_im, a32_re, a32_im,
    output reg  signed [36:0] a128_re, a128_im, a256_re, a256_im
);
    integer k;

    // ------------------------------------------------------------------ input delay
    // Sample m leaves the delay when sample m + LANE_DELAY arrives. No reset (SRLs): a sample is
    // only used once LANE_DELAY real samples have passed (fill counter).
    reg signed [11:0] dl_i [0:LANE_DELAY-1];
    reg signed [11:0] dl_q [0:LANE_DELAY-1];
    reg [5:0]         fill;
    reg               c_valid;
    reg signed [11:0] c_i, c_q;
    always @(posedge clk) begin
        if (in_valid) begin
            dl_i[0] <= in_i; dl_q[0] <= in_q;
            for (k = 1; k < LANE_DELAY; k = k + 1) begin dl_i[k] <= dl_i[k-1]; dl_q[k] <= dl_q[k-1]; end
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            fill <= 6'd0; c_valid <= 1'b0;
        end else begin
            c_valid <= in_valid && (fill == LANE_DELAY);
            if (in_valid) begin
                c_i <= dl_i[LANE_DELAY-1]; c_q <= dl_q[LANE_DELAY-1];
                if (fill != LANE_DELAY) fill <= fill + 6'd1;
            end
        end
    end

    // ------------------------------------------------------------------ lag taps on the lane stream
    // tap[j] = x(m - 1 - j); taps used: l - 1 for l = 16, 32, 128, 256. No reset: a tap is only
    // read for n >= t + l, where it holds a sample of the window.
    reg signed [11:0] tp_i [0:255];
    reg signed [11:0] tp_q [0:255];
    always @(posedge clk) begin
        if (c_valid) begin
            tp_i[0] <= c_i; tp_q[0] <= c_q;
            for (k = 1; k < 256; k = k + 1) begin tp_i[k] <= tp_i[k-1]; tp_q[k] <= tp_q[k-1]; end
        end
    end

    // ------------------------------------------------------------------ window control
    reg [31:0] m;                // index of the lane sample (counts c_valid)
    reg        pend;             // trigger waiting for its first sample
    reg [31:0] t_pend;
    reg        act;              // inside a window
    reg [31:0] t_act;
    reg [12:0] kw;               // m - t_act of the current sample (0 .. N-1)
    wire       starting = c_valid && !act && pend && (m == t_pend);
    wire       in_win   = c_valid && (act || starting);
    wire [12:0] kc      = starting ? 13'd0 : kw;
    wire [12:0] n_last  = n_cfo[12:0] - 13'd1;
    wire       last     = in_win && (kc == n_last);

    always @(posedge clk) begin
        if (!rst_n) begin
            m <= 32'd0; pend <= 1'b0; act <= 1'b0; kw <= 13'd0; t_pend <= 32'd0; t_act <= 32'd0;
        end else begin
            if (trig) begin pend <= 1'b1; t_pend <= t_trig; end
            if (c_valid) begin
                m <= m + 32'd1;
                if (starting) begin
                    pend  <= trig;         // a trigger in this very clock stays pending
                    t_act <= m;
                end
                if (in_win) begin
                    act <= !last;
                    kw  <= kc + 13'd1;
                end
            end
        end
    end
`ifdef PD_SIM_CHECKS
    always @(posedge clk)
        if (rst_n && c_valid && pend && !act && m > t_pend)
            $display("ERROR pd_cfo_lane: trigger at %0d reached the lane after its first sample (m = %0d)", t_pend, m);
`endif

    // ------------------------------------------------------------------ products (stage P)
    reg               p_v, p_start, p_last;
    reg  [3:0]        p_en;        // lag 16, 32, 128, 256 enabled (kc >= l)
    reg  signed [24:0] pr16, pi16, pr32, pi32, pr128, pi128, pr256, pi256;
    reg  [31:0]       p_t;
    // x(n) x*(n-l): re = xi di + xq dq, im = xq di - xi dq
    always @(posedge clk) begin
        if (!rst_n) begin
            p_v <= 1'b0;
        end else begin
            p_v <= in_win;
            if (in_win) begin
                p_start <= starting;
                p_last  <= last;
                p_t     <= starting ? m : t_act;
                p_en    <= {kc >= 13'd256, kc >= 13'd128, kc >= 13'd32, kc >= 13'd16};
                pr16  <= c_i * tp_i[15]  + c_q * tp_q[15];   pi16  <= c_q * tp_i[15]  - c_i * tp_q[15];
                pr32  <= c_i * tp_i[31]  + c_q * tp_q[31];   pi32  <= c_q * tp_i[31]  - c_i * tp_q[31];
                pr128 <= c_i * tp_i[127] + c_q * tp_q[127];  pi128 <= c_q * tp_i[127] - c_i * tp_q[127];
                pr256 <= c_i * tp_i[255] + c_q * tp_q[255];  pi256 <= c_q * tp_i[255] - c_i * tp_q[255];
            end
        end
    end

    // ------------------------------------------------------------------ accumulators (stage A)
    reg signed [36:0] s16r, s16i, s32r, s32i, s128r, s128i, s256r, s256i;
    wire signed [36:0] z37 = 37'sd0;
    wire signed [36:0] n16r  = (p_start ? z37 : s16r)  + (p_en[0] ? pr16  : 25'sd0);
    wire signed [36:0] n16i  = (p_start ? z37 : s16i)  + (p_en[0] ? pi16  : 25'sd0);
    wire signed [36:0] n32r  = (p_start ? z37 : s32r)  + (p_en[1] ? pr32  : 25'sd0);
    wire signed [36:0] n32i  = (p_start ? z37 : s32i)  + (p_en[1] ? pi32  : 25'sd0);
    wire signed [36:0] n128r = (p_start ? z37 : s128r) + (p_en[2] ? pr128 : 25'sd0);
    wire signed [36:0] n128i = (p_start ? z37 : s128i) + (p_en[2] ? pi128 : 25'sd0);
    wire signed [36:0] n256r = (p_start ? z37 : s256r) + (p_en[3] ? pr256 : 25'sd0);
    wire signed [36:0] n256i = (p_start ? z37 : s256i) + (p_en[3] ? pi256 : 25'sd0);

    // snapshot for the CORDIC sequence (A_16 negated: s_16 = -1)
    reg               snap_v;
    reg  [31:0]       snap_t;
    reg  signed [36:0] q16r, q16i, q32r, q32i, q128r, q128i, q256r, q256i;
    always @(posedge clk) begin
        if (!rst_n) begin
            snap_v <= 1'b0;
        end else begin
            snap_v <= 1'b0;
            if (p_v) begin
                s16r <= n16r;   s16i <= n16i;   s32r <= n32r;   s32i <= n32i;
                s128r <= n128r; s128i <= n128i; s256r <= n256r; s256i <= n256i;
                if (p_last) begin
                    snap_v <= 1'b1;
                    snap_t <= p_t;
                    q16r <= -n16r;  q16i <= -n16i;  q32r <= n32r;   q32i <= n32i;
                    q128r <= n128r; q128i <= n128i; q256r <= n256r; q256i <= n256i;
                end
            end
        end
    end

    // ------------------------------------------------------------------ CORDIC sequence + cascade
    reg  [2:0]         seq;        // 0 idle, 1..4: angle of lag 16 / 32 / 128 / 256, 5: cascade
    reg                c_start;
    reg  signed [36:0] c_re, c_im;
    wire               c_busy, c_done;
    wire signed [15:0] c_ang;
    reg  [31:0]        seq_t;
    reg  signed [36:0] w16r, w16i, w32r, w32i, w128r, w128i, w256r, w256i;
    reg  signed [15:0] g16, g32, g128, g256;
    reg                casc_go;

    pd_cordic u_cordic (.clk(clk), .rst_n(rst_n), .start(c_start), .re(c_re), .im(c_im),
                        .busy(c_busy), .done(c_done), .angle(c_ang));

    // cascade (3-clock pipeline) on the four angles
    wire               casc_v;
    wire signed [31:0] f_hat, inc;
    pd_cfo_cascade u_casc (.clk(clk), .rst_n(rst_n), .in_valid(casc_go),
                           .ang16(g16), .ang32(g32), .ang128(g128), .ang256(g256),
                           .out_valid(casc_v), .f_hat(f_hat), .cfo_inc(inc));

    always @(posedge clk) begin
        if (!rst_n) begin
            seq <= 3'd0; c_start <= 1'b0; cfo_valid <= 1'b0; casc_go <= 1'b0;
        end else begin
            c_start   <= 1'b0;
            cfo_valid <= 1'b0;
            casc_go   <= 1'b0;
            if (snap_v) begin
                // a new window result: start the sequence (the previous one is long finished,
                // windows are >= N samples apart and the sequence takes about 80 clocks)
                w16r <= q16r; w16i <= q16i; w32r <= q32r; w32i <= q32i;
                w128r <= q128r; w128i <= q128i; w256r <= q256r; w256i <= q256i;
                seq_t <= snap_t;
                c_re <= q16r; c_im <= q16i; c_start <= 1'b1; seq <= 3'd1;
            end else if (c_done) begin
                case (seq)
                    3'd1: begin g16  <= c_ang; c_re <= w32r;  c_im <= w32i;  c_start <= 1'b1; seq <= 3'd2; end
                    3'd2: begin g32  <= c_ang; c_re <= w128r; c_im <= w128i; c_start <= 1'b1; seq <= 3'd3; end
                    3'd3: begin g128 <= c_ang; c_re <= w256r; c_im <= w256i; c_start <= 1'b1; seq <= 3'd4; end
                    default: begin g256 <= c_ang; casc_go <= 1'b1; seq <= 3'd5; end
                endcase
            end else if (casc_v && seq == 3'd5) begin
                ang16 <= g16; ang32 <= g32; ang128 <= g128; ang256 <= g256;
                cfo_inc <= inc;
                cfo_t   <= seq_t;
                a16_re <= w16r;   a16_im <= w16i;   a32_re <= w32r;   a32_im <= w32i;
                a128_re <= w128r; a128_im <= w128i; a256_re <= w256r; a256_im <= w256i;
                cfo_valid <= 1'b1;
                seq <= 3'd0;
            end
        end
    end
`ifdef PD_SIM_CHECKS
    always @(posedge clk)
        if (rst_n && snap_v && seq != 3'd0)
            $display("ERROR pd_cfo_lane: new window result while the CORDIC sequence is busy (n_cfo too small)");
`endif
endmodule

`default_nettype wire
