// pd_detect_core.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Detection lane of the packet detector, bit-true to pd_reference.detect (spec 4.1-4.2):
//   c32(n) = x(n) x*(n-32), e(n) = |x(n)|^2                         25-bit
//   P(n) = P(n-1) + c32(n) - c32(n-256), R0 likewise; R_D(n) = R0(n-32) 33-bit, exact
//   comparator: common floor shift s so the largest of |Re P|, |Im P|, R0, R_D fits 18-bit
//     signed; num = (Re'^2 + Im'^2) << 16, den = R0' * R_D'
//     above_hi = num > ETA_HI_SQ * den,  below_lo = num < ETA_LO_SQ * den
//   blanking (n < BLANK_LEN), qualifier (count of above_hi, reset by below_lo, held between),
//   trigger when the count reaches QUAL_LEN, re-arm ARM_HOLDOFF samples after t_trig.
//   dbg_metric = min(65535, floor(num / den)), 0 if den = 0.
//
// Samples are processed in order, one per in_valid (at most one per clock). sample_cnt counts
// processed samples from reset; t_trig is the index of the triggering sample. Samples before
// index 0 count as zero (delay lines reset to zero, the history RAM is masked until full).
// Outputs: trig (one-clock pulse) with t_trig; a debug stream (one record per input sample):
//   dbg_metric, dbg_bits = {11'b0, blank, trig, armed, below_lo, above_hi}.
`timescale 1ns / 1ps
`default_nettype none

module pd_detect_core (
    input  wire               clk,
    input  wire               rst_n,
    // run-time parameters (constant while samples flow)
    input  wire [15:0]        eta_hi_sq,     // Q0.16
    input  wire [15:0]        eta_lo_sq,
    input  wire [15:0]        qual_len,      // 0 is treated as 1
    input  wire [31:0]        blank_len,
    input  wire [31:0]        arm_holdoff,   // 0 is treated as 1
    // samples (LPF output)
    input  wire               in_valid,
    input  wire signed [11:0] in_i,
    input  wire signed [11:0] in_q,
    // trigger
    output reg                trig,
    output reg  [31:0]        t_trig,
    // debug stream
    output wire               dbg_valid,
    output wire [15:0]        dbg_metric,
    output wire [15:0]        dbg_bits
);
    localparam integer L = 256;
    localparam integer D = 32;
    integer k;

    // ------------------------------------------------------------------ stage 1: x(n), x(n-32)
    reg signed [11:0] xd_i [0:D-1];
    reg signed [11:0] xd_q [0:D-1];
    reg signed [11:0] a_i, a_q, b_i, b_q;    // x(n), x(n-32)
    reg               v1;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < D; k = k + 1) begin xd_i[k] <= 12'sd0; xd_q[k] <= 12'sd0; end
            v1 <= 1'b0;
        end else begin
            v1 <= in_valid;
            if (in_valid) begin
                a_i <= in_i;        a_q <= in_q;
                b_i <= xd_i[D-1];   b_q <= xd_q[D-1];
                xd_i[0] <= in_i;    xd_q[0] <= in_q;
                for (k = 1; k < D; k = k + 1) begin xd_i[k] <= xd_i[k-1]; xd_q[k] <= xd_q[k-1]; end
            end
        end
    end

    // ------------------------------------------------------------------ stage 2: products
    reg signed [24:0] c_re, c_im, c_e;
    reg               v2;
    always @(posedge clk) begin
        if (!rst_n) v2 <= 1'b0;
        else begin
            v2 <= v1;
            if (v1) begin
                c_re <= a_i * b_i + a_q * b_q;
                c_im <= a_q * b_i - a_i * b_q;
                c_e  <= a_i * a_i + a_q * a_q;
            end
        end
    end

    // ------------------------------------------------------------------ stage 3: history (c(n-256))
    reg [74:0]        hist [0:L-1];          // BRAM, read-first
    reg [74:0]        old;
    reg [7:0]         wptr;
    reg [8:0]         fill;                  // samples written, saturates at 256
    reg               full3;
    reg signed [24:0] c3_re, c3_im, c3_e;
    reg               v3;
    always @(posedge clk) begin
        if (v2) begin
            old        <= hist[wptr];
            hist[wptr] <= {c_re, c_im, c_e};
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            wptr <= 8'd0; fill <= 9'd0; v3 <= 1'b0; full3 <= 1'b0;
        end else begin
            v3 <= v2;
            if (v2) begin
                wptr  <= wptr + 8'd1;
                full3 <= fill[8];
                if (!fill[8]) fill <= fill + 9'd1;
                c3_re <= c_re; c3_im <= c_im; c3_e <= c_e;
            end
        end
    end
    wire signed [24:0] o_re = full3 ? $signed(old[74:50]) : 25'sd0;
    wire signed [24:0] o_im = full3 ? $signed(old[49:25]) : 25'sd0;
    wire signed [24:0] o_e  = full3 ? $signed(old[24:0])  : 25'sd0;

    // ------------------------------------------------------------------ stage 4: running sums
    reg signed [32:0] P_re, P_im, R0;
    reg               v4;
    always @(posedge clk) begin
        if (!rst_n) begin
            P_re <= 33'sd0; P_im <= 33'sd0; R0 <= 33'sd0; v4 <= 1'b0;
        end else begin
            v4 <= v3;
            if (v3) begin
                P_re <= P_re + c3_re - o_re;
                P_im <= P_im + c3_im - o_im;
                R0   <= R0   + c3_e  - o_e;
            end
        end
    end

    // ------------------------------------------------------------------ stage 5: R_D = R0(n-32)
    reg signed [32:0] rdl [0:D-1];
    reg signed [32:0] s5_pr, s5_pi, s5_r0, s5_rd;
    reg               v5;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < D; k = k + 1) rdl[k] <= 33'sd0;
            v5 <= 1'b0;
        end else begin
            v5 <= v4;
            if (v4) begin
                s5_pr <= P_re; s5_pi <= P_im; s5_r0 <= R0; s5_rd <= rdl[D-1];
                rdl[0] <= R0;
                for (k = 1; k < D; k = k + 1) rdl[k] <= rdl[k-1];
            end
        end
    end

    // ------------------------------------------------------------------ stage 6: magnitudes
    function [31:0] abs33;
        input signed [32:0] v;
        reg   signed [32:0] t;
        begin t = (v < 0) ? -v : v; abs33 = t[31:0]; end
    endfunction
    reg [31:0]        m1, m2;
    reg signed [32:0] s6_pr, s6_pi, s6_r0, s6_rd;
    reg               v6;
    always @(posedge clk) begin
        if (!rst_n) v6 <= 1'b0;
        else begin
            v6 <= v5;
            if (v5) begin
                m1 <= (abs33(s5_pr) > abs33(s5_pi)) ? abs33(s5_pr) : abs33(s5_pi);
                m2 <= (s5_r0[31:0] > s5_rd[31:0]) ? s5_r0[31:0] : s5_rd[31:0];
                s6_pr <= s5_pr; s6_pi <= s5_pi; s6_r0 <= s5_r0; s6_rd <= s5_rd;
            end
        end
    end

    // ------------------------------------------------------------------ stage 7: common shift
    function [4:0] shift_of;                 // max(bit_length(m) - 17, 0)
        input [31:0] m;
        integer b;
        reg [5:0] bl;
        begin
            bl = 6'd0;
            for (b = 0; b < 32; b = b + 1) if (m[b]) bl = b[5:0] + 6'd1;
            shift_of = (bl > 6'd17) ? bl[4:0] - 5'd17 : 5'd0;
        end
    endfunction
    reg [4:0]         s7;
    reg signed [32:0] s7_pr, s7_pi, s7_r0, s7_rd;
    reg               v7;
    always @(posedge clk) begin
        if (!rst_n) v7 <= 1'b0;
        else begin
            v7 <= v6;
            if (v6) begin
                s7 <= shift_of((m1 > m2) ? m1 : m2);
                s7_pr <= s6_pr; s7_pi <= s6_pi; s7_r0 <= s6_r0; s7_rd <= s6_rd;
            end
        end
    end

    // ------------------------------------------------------------------ stage 8: 18-bit values
    wire signed [32:0] sh_pr = s7_pr >>> s7;
    wire signed [32:0] sh_pi = s7_pi >>> s7;
    wire signed [32:0] sh_r0 = s7_r0 >>> s7;
    wire signed [32:0] sh_rd = s7_rd >>> s7;
    reg  signed [17:0] p8_re, p8_im;
    reg         [17:0] r8_0, r8_d;
    reg                v8;
    always @(posedge clk) begin
        if (!rst_n) v8 <= 1'b0;
        else begin
            v8 <= v7;
            if (v7) begin
                p8_re <= sh_pr[17:0]; p8_im <= sh_pi[17:0];
                r8_0  <= sh_r0[17:0]; r8_d  <= sh_rd[17:0];
            end
        end
    end

    // ------------------------------------------------------------------ stage 9: squares, den
    reg [35:0] sq_re, sq_im;
    reg [35:0] den9;
    reg        v9;
    always @(posedge clk) begin
        if (!rst_n) v9 <= 1'b0;
        else begin
            v9 <= v8;
            if (v8) begin
                sq_re <= p8_re * p8_re;
                sq_im <= p8_im * p8_im;
                den9  <= r8_0 * r8_d;
            end
        end
    end

    // ------------------------------------------------------------------ stage 10: num, thresholds
    reg [51:0] num10, thr_hi, thr_lo;
    reg [35:0] den10;
    reg        v10;
    always @(posedge clk) begin
        if (!rst_n) v10 <= 1'b0;
        else begin
            v10 <= v9;
            if (v9) begin
                num10  <= {(sq_re + sq_im), 16'd0};
                thr_hi <= eta_hi_sq * den9;
                thr_lo <= eta_lo_sq * den9;
                den10  <= den9;
            end
        end
    end

    // ------------------------------------------------------------------ stage 11: compare
    reg        above11, below11;
    reg [51:0] num11;
    reg [35:0] den11;
    reg        v11;
    always @(posedge clk) begin
        if (!rst_n) v11 <= 1'b0;
        else begin
            v11 <= v10;
            if (v10) begin
                above11 <= num10 > thr_hi;
                below11 <= num10 < thr_lo;
                num11   <= num10;
                den11   <= den10;
            end
        end
    end

    // ------------------------------------------------------------------ stage 12: trigger FSM
    reg [31:0] n12;                          // index of the sample being processed
    reg [31:0] arm_at;                       // first armed sample index
    reg [15:0] count;
    wire [15:0] q_need  = (qual_len == 16'd0) ? 16'd1 : qual_len;
    wire [31:0] holdoff = (arm_holdoff == 32'd0) ? 32'd1 : arm_holdoff;
    wire        armed   = (n12 >= arm_at);
    wire        blank   = (n12 < blank_len);
    wire [15:0] count_n = below11 ? 16'd0 : (above11 ? count + 16'd1 : count);
    wire        fire    = armed && above11 && !below11 && (count_n >= q_need);

    reg        v12;
    reg [15:0] bits12;
    reg [51:0] num12;
    reg [35:0] den12;
    always @(posedge clk) begin
        if (!rst_n) begin
            n12 <= 32'd0; arm_at <= 32'd0; count <= 16'd0;
            trig <= 1'b0; t_trig <= 32'd0; v12 <= 1'b0;
        end else begin
            v12  <= v11;
            trig <= 1'b0;
            if (n12 == 32'd0 && !v11)
                arm_at <= blank_len;         // follow the register until the first sample
            if (v11) begin
                n12    <= n12 + 32'd1;
                bits12 <= {11'd0, blank, fire, armed, below11, above11};
                num12  <= num11;
                den12  <= den11;
                if (n12 == 32'd0) arm_at <= blank_len;
                if (armed) begin
                    if (fire) begin
                        trig   <= 1'b1;
                        t_trig <= n12;
                        count  <= 16'd0;
                        arm_at <= n12 + holdoff;
                    end else
                        count <= count_n;
                end
            end
        end
    end

    // ------------------------------------------------------------------ debug: metric divider
    pd_div16 u_div (
        .clk(clk), .rst_n(rst_n),
        .in_valid(v12), .num(num12), .den(den12), .side_in(bits12),
        .out_valid(dbg_valid), .quot(dbg_metric), .side_out(dbg_bits));
endmodule

`default_nettype wire
