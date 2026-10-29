// cf_rx.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Control-field receiver, bit-true to cf_reference.receive. After start (cfo_inc of the event,
// t_trig and n_end of the good timing record, both only passed through) it takes exactly
// CF_LEN = 4560 samples y(n_end + 1 ...) of the DC blocker output on s_* (valid / ready) and
// returns one record:
//   1. de-rotator  x(n) = sat12(rnd(y(n) (cos + j sin)(cfo_inc n) >> 15)), phase 0 at n = 0,
//                  pd_tone_rom address = phase[23:14]
//   2. FFT         symbol s: x(1140 s + 104 .. + 1127) << 5 into cf_fft; the input stops
//                  (s_ready low) while the FFT runs and its 841 bins are unloaded:
//                  grid column c = 0..840 <- bin (c + 604) mod 1024, grid RAM {s, c};
//                  symbols 0 and 3: P0 pilots c = 3 j + 2 -> pilot RAM j = Y * ref (19-bit)
//   3. cf_chest    channel estimate, soft values, combining, Viterbi input scaling
//   4. cf_viterbi  70 bits, metric;  5. cf_crc10 over bits 0..63
// rec_valid: rec_crc_ok, rec_msg (bit i = message bit i), rec_shift, rec_metric.
// Clocks per control field: 4560 input + 4 x (5200 FFT + 850 unload) + ~10600 decoding
// + 4760 Viterbi, about 44000 (0.44 ms at 100 MHz).
`timescale 1ns / 1ps
`default_nettype none

module cf_rx (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [23:0]        cfo_inc,
    input  wire [31:0]        t_trig,
    input  wire [31:0]        n_end,
    output wire               busy,
    input  wire               s_valid,
    output wire               s_ready,
    input  wire signed [11:0] s_i,
    input  wire signed [11:0] s_q,
    output reg                rec_valid,
    output reg                rec_crc_ok,
    output reg  [53:0]        rec_msg,
    output reg  [5:0]         rec_shift,
    output reg  signed [15:0] rec_metric,
    output reg  [31:0]        rec_t_trig,
    output reg  [31:0]        rec_n_end,
    // testbench only
    output reg                dbg_grid_we,
    output reg  [11:0]        dbg_grid_addr,
    output reg  [35:0]        dbg_grid_data,
    output wire signed [15:0] dbg_phi,
    output wire               dbg_hf_we,
    output wire [9:0]         dbg_hf_k,
    output wire [79:0]        dbg_hf,
    output wire               dbg_v_we,
    output wire [11:0]        dbg_v_i,
    output wire signed [45:0] dbg_v,
    output wire [69:0]        dbg_bits
);
    localparam [3:0] R_IDLE = 4'd0, R_IN = 4'd1, R_DRAIN = 4'd2, R_FFT = 4'd3, R_FWAIT = 4'd4,
                     R_UNLOAD = 4'd5, R_CE = 4'd6, R_CEW = 4'd7, R_VIT = 4'd8, R_CRC = 4'd9;
    reg [3:0]  state;
    reg [23:0] inc, ph;
    reg [10:0] pos;                             // sample position in the symbol, 0..1139
    reg [1:0]  isym;                            // symbol of the input sample
    reg [1:0]  fsym;                            // symbol in the FFT
    reg [9:0]  cnt;
    assign busy = (state != R_IDLE);
    assign s_ready = (state == R_IN);
    wire acc = s_valid && s_ready;

    // ================================================================ de-rotator
    wire signed [15:0] tc, ts;
    pd_tone_rom u_tone (.clk(clk), .addr_a(ph[23:14]), .addr_b(ph[23:14] - 10'd256),
                        .dout_a(tc), .dout_b(ts));
    reg               v1, v2, v3;
    reg               w1, w2, w3;               // inside the FFT window
    reg  [9:0]        x1, x2, x3;               // FFT input index
    reg signed [11:0] yi1, yq1;
    reg signed [27:0] p_ic, p_qs, p_is, p_qc;
    reg signed [11:0] xr3, xi3;
    wire in_win = (pos >= 11'd104) && (pos < 11'd1128);
    wire [10:0] widx = pos - 11'd104;
    function signed [11:0] rsat12;
        input signed [28:0] v;
        reg   signed [28:0] r;
        begin
            r = (v + 29'sd16384) >>> 15;
            if (r > 29'sd2047) rsat12 = 12'sd2047;
            else if (r < -29'sd2048) rsat12 = -12'sd2048;
            else rsat12 = r[11:0];
        end
    endfunction
    always @(posedge clk) begin
        v1 <= acc; w1 <= in_win; x1 <= widx[9:0]; yi1 <= s_i; yq1 <= s_q;
        v2 <= v1; w2 <= w1; x2 <= x1;
        p_ic <= yi1 * tc; p_qs <= yq1 * ts; p_is <= yi1 * ts; p_qc <= yq1 * tc;
        v3 <= v2; w3 <= w2; x3 <= x2;
        xr3 <= rsat12({p_ic[27], p_ic} - {p_qs[27], p_qs});
        xi3 <= rsat12({p_is[27], p_is} + {p_qc[27], p_qc});
        if (!rst_n) begin v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; end
    end
    // c3 -> FFT load (registered once more, so the FFT sees a clean write)
    reg               f_we;
    reg  [9:0]        f_idx;
    reg signed [17:0] f_re, f_im;
    always @(posedge clk) begin
        f_we <= v3 && w3; f_idx <= x3;
        f_re <= {{1{xr3[11]}}, xr3, 5'd0};
        f_im <= {{1{xi3[11]}}, xi3, 5'd0};
        if (!rst_n) f_we <= 1'b0;
    end

    // ================================================================ FFT
    reg         fft_start;
    wire        fft_done;
    wire [9:0]  fft_oaddr = cnt + 10'd604;
    wire signed [17:0] fft_ore, fft_oim;
    cf_fft u_fft (.clk(clk), .rst_n(rst_n), .in_we(f_we), .in_idx(f_idx), .in_re(f_re), .in_im(f_im),
                  .start(fft_start), .busy(), .done(fft_done), .out_addr(fft_oaddr),
                  .out_re(fft_ore), .out_im(fft_oim));

    // unload: issue column cnt (R_UNLOAD), data one clock later
    reg        u_v;
    reg  [9:0] u_c;
    reg  [1:0] u_m3;                            // cnt mod 3 at issue
    reg  [1:0] c_m3;
    reg  [8:0] u_j, c_j;                        // pilot index of column cnt
    wire       ref_neg;
    cf_ref_rom u_ref (.clk(clk), .addr(c_j), .q(ref_neg));
    always @(posedge clk) begin
        u_v <= (state == R_UNLOAD) && (cnt <= 10'd840);
        u_c <= cnt; u_m3 <= c_m3; u_j <= c_j;
        if (!rst_n) u_v <= 1'b0;
    end
    wire        g_we = u_v;
    wire [11:0] g_waddr = {fsym, u_c};
    wire [35:0] g_wdata = {fft_ore, fft_oim};
    wire        p_we = u_v && (u_m3 == 2'd2);
    wire signed [18:0] pr = ref_neg ? -{fft_ore[17], fft_ore} : {fft_ore[17], fft_ore};
    wire signed [18:0] pq = ref_neg ? -{fft_oim[17], fft_oim} : {fft_oim[17], fft_oim};
    always @(posedge clk) begin
        dbg_grid_we <= g_we; dbg_grid_addr <= g_waddr; dbg_grid_data <= g_wdata;
    end

    // ================================================================ RAMs
    wire [11:0] ce_gaddr;
    wire [35:0] ce_gdata;
    wire [8:0]  ce_paddr;
    wire [37:0] ce_p0, ce_p3;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(36), .A(12)) u_grid (.clk(clk), .we(g_we), .waddr(g_waddr), .wdata(g_wdata),
                                         .raddr(ce_gaddr), .rdata(ce_gdata));
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(38), .A(9)) u_p0 (.clk(clk), .we(p_we && fsym == 2'd0), .waddr(u_j),
                                      .wdata({pr, pq}), .raddr(ce_paddr), .rdata(ce_p0));
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(38), .A(9)) u_p3 (.clk(clk), .we(p_we && fsym == 2'd3), .waddr(u_j),
                                      .wdata({pr, pq}), .raddr(ce_paddr), .rdata(ce_p3));

    // ================================================================ decoding
    reg         ce_start, vit_start, crc_start;
    wire        ce_done, vit_done, crc_done, crc_ok;
    wire [5:0]  ce_shift;
    wire        q_we;
    wire [7:0]  q_addr;
    wire signed [4:0] q_data;
    wire [69:0] vbits;
    wire signed [15:0] vmetric;
    cf_chest u_ce (.clk(clk), .rst_n(rst_n), .start(ce_start), .busy(), .done(ce_done), .shift(ce_shift),
                   .p_addr(ce_paddr), .p0_data(ce_p0), .p3_data(ce_p3), .g_addr(ce_gaddr), .g_data(ce_gdata),
                   .q_we(q_we), .q_addr(q_addr), .q_data(q_data),
                   .dbg_phi(dbg_phi), .dbg_hf_we(dbg_hf_we), .dbg_hf_k(dbg_hf_k), .dbg_hf(dbg_hf),
                   .dbg_v_we(dbg_v_we), .dbg_v_i(dbg_v_i), .dbg_v(dbg_v));
    cf_viterbi u_vit (.clk(clk), .rst_n(rst_n), .q_we(q_we), .q_addr(q_addr), .q_data(q_data),
                      .start(vit_start), .busy(), .done(vit_done), .bits(vbits), .metric(vmetric));
    cf_crc10 #(.N(64)) u_crc (.clk(clk), .rst_n(rst_n), .start(crc_start), .bits(vbits[63:0]),
                              .done(crc_done), .crc(), .ok(crc_ok));
    assign dbg_bits = vbits;

    // ================================================================ control
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= R_IDLE; rec_valid <= 1'b0; fft_start <= 1'b0;
            ce_start <= 1'b0; vit_start <= 1'b0; crc_start <= 1'b0;
        end else begin
            rec_valid <= 1'b0; fft_start <= 1'b0; ce_start <= 1'b0; vit_start <= 1'b0; crc_start <= 1'b0;
            case (state)
                R_IDLE: if (start) begin
                    inc <= cfo_inc; ph <= 24'd0; pos <= 11'd0; isym <= 2'd0; fsym <= 2'd0;
                    rec_t_trig <= t_trig; rec_n_end <= n_end;
                    state <= R_IN;
                end
                R_IN: if (acc) begin
                    ph <= ph + inc;
                    if (pos == 11'd1139) begin pos <= 11'd0; isym <= isym + 2'd1; end
                    else pos <= pos + 11'd1;
                    if (pos == 11'd1127) begin cnt <= 10'd0; state <= R_DRAIN; end
                    else if (isym == 2'd3 && pos == 11'd1139) state <= R_CE;
                end
                R_DRAIN: if (cnt == 10'd6) begin fft_start <= 1'b1; state <= R_FFT; end
                         else cnt <= cnt + 10'd1;
                R_FFT: state <= R_FWAIT;
                R_FWAIT: if (fft_done) begin
                    cnt <= 10'd0; c_m3 <= 2'd0; c_j <= 9'd0; state <= R_UNLOAD;
                end
                R_UNLOAD: begin
                    c_m3 <= (c_m3 == 2'd2) ? 2'd0 : c_m3 + 2'd1;
                    if (c_m3 == 2'd2) c_j <= c_j + 9'd1;
                    if (cnt == 10'd842) begin
                        fsym <= fsym + 2'd1;
                        state <= R_IN;
                    end else cnt <= cnt + 10'd1;
                end
                R_CE: begin ce_start <= 1'b1; state <= R_CEW; end
                R_CEW: if (ce_done) begin vit_start <= 1'b1; rec_shift <= ce_shift; state <= R_VIT; end
                R_VIT: if (vit_done) begin crc_start <= 1'b1; rec_metric <= vmetric; state <= R_CRC; end
                R_CRC: if (crc_done) begin
                    rec_valid <= 1'b1; rec_crc_ok <= crc_ok; rec_msg <= vbits[53:0];
                    state <= R_IDLE;
                end
                default: state <= R_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
