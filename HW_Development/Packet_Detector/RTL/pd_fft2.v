// pd_fft2.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// 1024-point radix-2 decimation-in-time FFT, TWO butterflies per clock, bit-true to
// cf_reference.fft_fixed (FFT_W 18, TW_FRAC 17) and to cf_fft.v (same interface, same results,
// about half the time): start -> done after 10 x (256 + GAP) clocks (2640).
//
// Every stage computes the same butterflies as cf_fft (see there); only the order changes:
// in clock cnt = 0..255 of stage st it does butterflies b0 = cnt and b1 = cnt + 256.
//   st <= 8: ia1 = ia0 + 512, ib1 = ib0 + 512, the same twiddle
//   st == 9: ia0 = b, ib0 = b + 512, ia1 = b + 256, ib1 = b + 768, twiddles k and k + 256
// Each bank is two true dual-port RAMs of 512 words, "lo" (address bit 9 = 0) and "hi", so the
// four reads and four writes of a clock always use the four ports:
//   st <= 8: lo.p = ia0, lo.q = ib0, hi.p = ia1, hi.q = ib1
//   st == 9: lo.p = ia0, lo.q = ia1, hi.p = ib0, hi.q = ib1
// Stage st reads bank st[0] and writes the other one; the result of stage 9 is in bank 0.
//   load   (idle) in_we writes sample in_idx to bank 0 at the bit-reversed address
//   output (idle) out_addr (natural bin order) -> out_re / out_im one clock later
`timescale 1ns / 1ps
`default_nettype none

module pd_fft2 (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_we,
    input  wire [9:0]         in_idx,
    input  wire signed [17:0] in_re,
    input  wire signed [17:0] in_im,
    input  wire               start,
    output wire               busy,
    output reg                done,
    input  wire [9:0]         out_addr,
    output wire signed [17:0] out_re,
    output wire signed [17:0] out_im
);
    localparam integer LAT = 5;                 // issue -> write
    localparam integer GAP = 8;                 // idle clocks between stages (> LAT)

    function [9:0] bitrev10;
        input [9:0] v;
        integer i;
        begin
            for (i = 0; i < 10; i = i + 1) bitrev10[i] = v[9 - i];
        end
    endfunction

    // ---- control
    reg        run;
    reg  [3:0] st;
    reg  [8:0] cnt;                             // 0..255 issue, 256..256+GAP-1 drain
    wire       issue = run && (cnt < 9'd256);
    wire       s9 = (st == 4'd9);
    wire [8:0] b0 = {1'b0, cnt[7:0]};
    wire [8:0] b1 = {1'b1, cnt[7:0]};
    wire [8:0] hmask = (9'd1 << st) - 9'd1;
    wire [9:0] ia0 = (({1'b0, b0} >> st) << (st + 1)) | {1'b0, b0 & hmask};
    wire [9:0] ib0 = ia0 | (10'd1 << st);
    wire [9:0] ia1 = (({1'b0, b1} >> st) << (st + 1)) | {1'b0, b1 & hmask};
    wire [9:0] ib1 = ia1 | (10'd1 << st);
    wire [8:0] tw_k0 = (b0 & hmask) << (4'd9 - st);
    wire [8:0] tw_k1 = (b1 & hmask) << (4'd9 - st);
    // port addresses inside a 512-word half bank
    wire [8:0] lo_p = ia0[8:0];
    wire [8:0] lo_q = s9 ? ia1[8:0] : ib0[8:0];
    wire [8:0] hi_p = s9 ? ib0[8:0] : ia1[8:0];
    wire [8:0] hi_q = ib1[8:0];
    assign busy = run;

    always @(posedge clk) begin
        if (!rst_n) begin
            run <= 1'b0; st <= 4'd0; cnt <= 9'd0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (!run) begin
                if (start) begin run <= 1'b1; st <= 4'd0; cnt <= 9'd0; end
            end else if (cnt == 9'd256 + GAP - 1) begin
                cnt <= 9'd0;
                if (st == 4'd9) begin run <= 1'b0; done <= 1'b1; end
                else st <= st + 4'd1;
            end else
                cnt <= cnt + 9'd1;
        end
    end

    // ---- write-side pipeline (port addresses, routing and bank delayed LAT clocks)
    reg  [35:0] wa_d [0:LAT-1];                 // {lo_p, lo_q, hi_p, hi_q}
    reg         v_d  [0:LAT-1];
    reg         wb_d [0:LAT-1];                 // bank written: ~st[0]
    reg         s9_d [0:LAT-1];
    integer p;
    always @(posedge clk) begin
        wa_d[0] <= {lo_p, lo_q, hi_p, hi_q}; v_d[0] <= issue; wb_d[0] <= ~st[0]; s9_d[0] <= s9;
        for (p = 1; p < LAT; p = p + 1) begin
            wa_d[p] <= wa_d[p-1]; v_d[p] <= v_d[p-1]; wb_d[p] <= wb_d[p-1]; s9_d[p] <= s9_d[p-1];
        end
        if (!rst_n) for (p = 0; p < LAT; p = p + 1) v_d[p] <= 1'b0;
    end
    wire        wv    = v_d[LAT-1];
    wire        wbank = wb_d[LAT-1];
    wire        ws9   = s9_d[LAT-1];
    wire [8:0]  w_lo_p = wa_d[LAT-1][35:27], w_lo_q = wa_d[LAT-1][26:18];
    wire [8:0]  w_hi_p = wa_d[LAT-1][17:9],  w_hi_q = wa_d[LAT-1][8:0];
    wire [35:0] na0, nb0, na1, nb1;             // butterfly results (a', b') of b0 and b1
    // write data per port: lo.p = a0', lo.q = s9 ? a1' : b0', hi.p = s9 ? b0' : a1', hi.q = b1'
    wire [35:0] wd_lo_p = na0;
    wire [35:0] wd_lo_q = ws9 ? na1 : nb0;
    wire [35:0] wd_hi_p = ws9 ? nb0 : na1;
    wire [35:0] wd_hi_q = nb1;

    // ---- banks: [bank][half] ; half 0 = lo, 1 = hi
    wire        rbank = st[0];
    reg         we_p [0:3], we_q [0:3];         // index {bank, half}
    reg  [8:0]  ad_p [0:3], ad_q [0:3];
    reg  [35:0] wd_p [0:3], wd_q [0:3];
    wire [35:0] rd_p [0:3], rd_q [0:3];
    wire [9:0]  ld_a = bitrev10(in_idx);
    integer j;
    always @* begin
        for (j = 0; j < 4; j = j + 1) begin
            we_p[j] = 1'b0; we_q[j] = 1'b0; ad_p[j] = 9'd0; ad_q[j] = 9'd0;
            wd_p[j] = 36'd0; wd_q[j] = 36'd0;
        end
        if (!run) begin
            // bank 0: load or output on port p of the half selected by address bit 9
            ad_p[0] = in_we ? ld_a[8:0] : out_addr[8:0];
            ad_p[1] = in_we ? ld_a[8:0] : out_addr[8:0];
            we_p[0] = in_we && !ld_a[9];
            we_p[1] = in_we &&  ld_a[9];
            wd_p[0] = {in_re, in_im};
            wd_p[1] = {in_re, in_im};
        end else begin
            ad_p[{rbank, 1'b0}] = lo_p; ad_q[{rbank, 1'b0}] = lo_q;
            ad_p[{rbank, 1'b1}] = hi_p; ad_q[{rbank, 1'b1}] = hi_q;
            if (wv) begin
                we_p[{wbank, 1'b0}] = 1'b1; we_q[{wbank, 1'b0}] = 1'b1;
                we_p[{wbank, 1'b1}] = 1'b1; we_q[{wbank, 1'b1}] = 1'b1;
                ad_p[{wbank, 1'b0}] = w_lo_p; ad_q[{wbank, 1'b0}] = w_lo_q;
                ad_p[{wbank, 1'b1}] = w_hi_p; ad_q[{wbank, 1'b1}] = w_hi_q;
                wd_p[{wbank, 1'b0}] = wd_lo_p; wd_q[{wbank, 1'b0}] = wd_lo_q;
                wd_p[{wbank, 1'b1}] = wd_hi_p; wd_q[{wbank, 1'b1}] = wd_hi_q;
            end
        end
    end
    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_ram
            cf_tdp_ram #(.W(36), .A(9)) u_ram (.clk(clk),
                .we_a(we_p[g]), .addr_a(ad_p[g]), .wd_a(wd_p[g]), .rd_a(rd_p[g]),
                .we_b(we_q[g]), .addr_b(ad_q[g]), .wd_b(wd_q[g]), .rd_b(rd_q[g]));
        end
    endgenerate
    reg o_hi;
    always @(posedge clk) o_hi <= out_addr[9];
    wire [35:0] o_d = o_hi ? rd_p[1] : rd_p[0];
    assign out_re = o_d[35:18];
    assign out_im = o_d[17:0];

    wire [35:0] tw0, tw1;
    cf_twiddle_rom u_tw0 (.clk(clk), .addr(tw_k0), .q(tw0));
    cf_twiddle_rom u_tw1 (.clk(clk), .addr(tw_k1), .q(tw1));

    // ---- read routing (c1: RAM / ROM outputs valid; bank and stage of one clock earlier)
    reg rbank_q, s9_q;
    always @(posedge clk) begin rbank_q <= rbank; s9_q <= s9; end
    wire [35:0] r_lo_p = rbank_q ? rd_p[2] : rd_p[0];
    wire [35:0] r_lo_q = rbank_q ? rd_q[2] : rd_q[0];
    wire [35:0] r_hi_p = rbank_q ? rd_p[3] : rd_p[1];
    wire [35:0] r_hi_q = rbank_q ? rd_q[3] : rd_q[1];
    wire [35:0] ra0 = r_lo_p;
    wire [35:0] rb0 = s9_q ? r_hi_p : r_lo_q;
    wire [35:0] ra1 = s9_q ? r_lo_q : r_hi_p;
    wire [35:0] rb1 = r_hi_q;

    pd_fft_bfly u_bf0 (.clk(clk), .a(ra0), .b(rb0), .w(tw0), .na(na0), .nb(nb0));
    pd_fft_bfly u_bf1 (.clk(clk), .a(ra1), .b(rb1), .w(tw1), .na(na1), .nb(nb1));
endmodule

// One radix-2 butterfly, 4 registered stages (operands, products, sums, result), as cf_fft:
//   t = rnd((b * w) >> 17), a' = sat18(rnd((a + t) >> 1)), b' = sat18(rnd((a - t) >> 1))
module pd_fft_bfly (
    input  wire        clk,
    input  wire [35:0] a,                       // {re, im}, 18 bit each
    input  wire [35:0] b,
    input  wire [35:0] w,
    output reg  [35:0] na,
    output reg  [35:0] nb
);
    reg signed [17:0] a_r1, a_i1, b_r, b_i, w_r, w_i;
    always @(posedge clk) begin
        a_r1 <= a[35:18]; a_i1 <= a[17:0];
        b_r  <= b[35:18]; b_i  <= b[17:0];
        w_r  <= w[35:18]; w_i  <= w[17:0];
    end
    reg signed [35:0] m_rr, m_ii, m_ri, m_ir;
    reg signed [17:0] a_r2, a_i2;
    always @(posedge clk) begin
        m_rr <= b_r * w_r; m_ii <= b_i * w_i; m_ri <= b_r * w_i; m_ir <= b_i * w_r;
        a_r2 <= a_r1; a_i2 <= a_i1;
    end
    reg signed [36:0] s_r, s_i;
    reg signed [17:0] a_r3, a_i3;
    always @(posedge clk) begin
        s_r <= m_rr - m_ii + 37'sd65536;
        s_i <= m_ri + m_ir + 37'sd65536;
        a_r3 <= a_r2; a_i3 <= a_i2;
    end
    wire signed [19:0] t_r = s_r[36:17];
    wire signed [19:0] t_i = s_i[36:17];
    function [17:0] half_sat;
        input signed [20:0] v;
        reg   signed [20:0] h;
        begin
            h = (v + 21'sd1) >>> 1;
            if (h > 21'sd131071) half_sat = 18'h1FFFF;
            else if (h < -21'sd131072) half_sat = 18'h20000;
            else half_sat = h[17:0];
        end
    endfunction
    wire signed [20:0] ax_r = {{3{a_r3[17]}}, a_r3};
    wire signed [20:0] ax_i = {{3{a_i3[17]}}, a_i3};
    wire signed [20:0] tx_r = {t_r[19], t_r};
    wire signed [20:0] tx_i = {t_i[19], t_i};
    always @(posedge clk) begin
        na <= {half_sat(ax_r + tx_r), half_sat(ax_i + tx_i)};
        nb <= {half_sat(ax_r - tx_r), half_sat(ax_i - tx_i)};
    end
endmodule

`default_nettype wire
