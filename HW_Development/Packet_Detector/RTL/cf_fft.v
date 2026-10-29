// cf_fft.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// 1024-point radix-2 decimation-in-time FFT, one butterfly per clock, bit-true to
// cf_reference.fft_fixed (FFT_W 18, TW_FRAC 17):
//   load   in_we writes sample in_idx (natural order, already << FFT_IN_SHIFT, 18-bit) to
//          bank 0 at the bit-reversed address
//   stage  st = 0..9, half = 2^st, butterfly b = 0..511:
//            ia = (b >> st) << (st + 1) | (b & (half - 1)), ib = ia + half,
//            w = W^((b & (half - 1)) << (9 - st))
//            t  = rnd((b * w) >> 17)          per rail: tr = br wr - bi wi, ti = br wi + bi wr
//            a' = sat18(rnd((a + t) >> 1)),  b' = sat18(rnd((a - t) >> 1))
//          stage st reads bank st[0] (both ports) and writes the other bank, so the result
//          of stage 9 is in bank 0
//   output out_addr (natural bin order) -> out_re / out_im one clock later, while idle
// start -> done pulse after 10 x (512 + GAP) clocks (5200).
`timescale 1ns / 1ps
`default_nettype none

module cf_fft (
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
    reg  [9:0] cnt;                             // 0..511 issue, 512..512+GAP-1 drain
    wire       issue = run && (cnt < 10'd512);
    wire [8:0] bf = cnt[8:0];
    wire [8:0] hmask = (9'd1 << st) - 9'd1;
    wire [9:0] ia = (({1'b0, bf} >> st) << (st + 1)) | {1'b0, bf & hmask};
    wire [9:0] ib = ia | (10'd1 << st);
    wire [8:0] tw_k = (bf & hmask) << (4'd9 - st);
    assign busy = run;

    always @(posedge clk) begin
        if (!rst_n) begin
            run <= 1'b0; st <= 4'd0; cnt <= 10'd0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (!run) begin
                if (start) begin run <= 1'b1; st <= 4'd0; cnt <= 10'd0; end
            end else if (cnt == 10'd512 + GAP - 1) begin
                cnt <= 10'd0;
                if (st == 4'd9) begin run <= 1'b0; done <= 1'b1; end
                else st <= st + 4'd1;
            end else
                cnt <= cnt + 10'd1;
        end
    end

    // ---- write-side pipeline (addresses and bank delayed LAT clocks)
    reg  [9:0] ia_d [0:LAT-1];
    reg  [9:0] ib_d [0:LAT-1];
    reg        v_d  [0:LAT-1];
    reg        wb_d [0:LAT-1];                  // bank written: ~st[0]
    integer p;
    always @(posedge clk) begin
        ia_d[0] <= ia; ib_d[0] <= ib; v_d[0] <= issue; wb_d[0] <= ~st[0];
        for (p = 1; p < LAT; p = p + 1) begin
            ia_d[p] <= ia_d[p-1]; ib_d[p] <= ib_d[p-1]; v_d[p] <= v_d[p-1]; wb_d[p] <= wb_d[p-1];
        end
        if (!rst_n) for (p = 0; p < LAT; p = p + 1) v_d[p] <= 1'b0;
    end
    wire       wv = v_d[LAT-1];
    wire       wbank = wb_d[LAT-1];
    wire [9:0] wa = ia_d[LAT-1];
    wire [9:0] wb = ib_d[LAT-1];
    reg  [35:0] wd_a, wd_b;                     // written data (set in the datapath)

    // ---- banks
    wire        rbank = st[0];
    reg         b0_we_a, b0_we_b, b1_we_a, b1_we_b;
    reg  [9:0]  b0_ad_a, b0_ad_b, b1_ad_a, b1_ad_b;
    reg  [35:0] b0_wd_a, b1_wd_a;
    wire [35:0] b0_rd_a, b0_rd_b, b1_rd_a, b1_rd_b;
    always @* begin
        b0_we_a = 1'b0; b0_we_b = 1'b0; b1_we_a = 1'b0; b1_we_b = 1'b0;
        b0_ad_a = out_addr; b0_ad_b = ib; b1_ad_a = ia; b1_ad_b = ib;
        b0_wd_a = wd_a; b1_wd_a = wd_a;
        if (!run) begin
            b0_we_a = in_we;
            b0_ad_a = in_we ? bitrev10(in_idx) : out_addr;
            b0_wd_a = {in_re, in_im};
        end else begin
            if (rbank == 1'b0) begin b0_ad_a = ia; b0_ad_b = ib; end
            else begin b1_ad_a = ia; b1_ad_b = ib; end
            if (wv) begin
                if (wbank == 1'b0) begin b0_we_a = 1'b1; b0_we_b = 1'b1; b0_ad_a = wa; b0_ad_b = wb; end
                else begin b1_we_a = 1'b1; b1_we_b = 1'b1; b1_ad_a = wa; b1_ad_b = wb; end
            end
        end
    end
    cf_tdp_ram #(.W(36), .A(10)) u_b0 (.clk(clk), .we_a(b0_we_a), .addr_a(b0_ad_a), .wd_a(b0_wd_a),
        .rd_a(b0_rd_a), .we_b(b0_we_b), .addr_b(b0_ad_b), .wd_b(wd_b), .rd_b(b0_rd_b));
    cf_tdp_ram #(.W(36), .A(10)) u_b1 (.clk(clk), .we_a(b1_we_a), .addr_a(b1_ad_a), .wd_a(b1_wd_a),
        .rd_a(b1_rd_a), .we_b(b1_we_b), .addr_b(b1_ad_b), .wd_b(wd_b), .rd_b(b1_rd_b));
    assign out_re = b0_rd_a[35:18];
    assign out_im = b0_rd_a[17:0];

    wire [35:0] tw_q;
    cf_twiddle_rom u_tw (.clk(clk), .addr(tw_k), .q(tw_q));

    // ---- datapath
    // c1: RAM / ROM outputs valid (bank selected by the bank read one clock earlier)
    reg rbank_q;
    always @(posedge clk) rbank_q <= rbank;
    wire [35:0] ra = rbank_q ? b1_rd_a : b0_rd_a;
    wire [35:0] rb = rbank_q ? b1_rd_b : b0_rd_b;
    // c1 -> c2: operand registers
    reg signed [17:0] a_r1, a_i1, b_r, b_i, w_r, w_i;
    always @(posedge clk) begin
        a_r1 <= ra[35:18]; a_i1 <= ra[17:0];
        b_r  <= rb[35:18]; b_i  <= rb[17:0];
        w_r  <= tw_q[35:18]; w_i <= tw_q[17:0];
    end
    // c2 -> c3: products
    reg signed [35:0] m_rr, m_ii, m_ri, m_ir;
    reg signed [17:0] a_r2, a_i2;
    always @(posedge clk) begin
        m_rr <= b_r * w_r; m_ii <= b_i * w_i; m_ri <= b_r * w_i; m_ir <= b_i * w_r;
        a_r2 <= a_r1; a_i2 <= a_i1;
    end
    // c3 -> c4: sums with the rounding constant
    reg signed [36:0] s_r, s_i;
    reg signed [17:0] a_r3, a_i3;
    always @(posedge clk) begin
        s_r <= m_rr - m_ii + 37'sd65536;
        s_i <= m_ri + m_ir + 37'sd65536;
        a_r3 <= a_r2; a_i3 <= a_i2;
    end
    // c4 -> c5: butterfly, rounding, saturation
    wire signed [19:0] t_r = s_r[36:17];
    wire signed [19:0] t_i = s_i[36:17];
    function [17:0] half_sat;
        input signed [20:0] v;                  // a +- t
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
    wire signed [20:0] sum_r = ax_r + tx_r, dif_r = ax_r - tx_r;
    wire signed [20:0] sum_i = ax_i + tx_i, dif_i = ax_i - tx_i;
    always @(posedge clk) begin
        wd_a <= {half_sat(sum_r), half_sat(sum_i)};
        wd_b <= {half_sat(dif_r), half_sat(dif_i)};
    end
endmodule

`default_nettype wire
