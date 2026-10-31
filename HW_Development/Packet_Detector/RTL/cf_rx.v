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
// Clocks per control field: 4560 input + 4 x (2640 FFT + 850 unload) + ~10600 decoding
// + 4760 Viterbi, about 34000 (0.34 ms at 100 MHz).
//
// The FFT is pd_fft2 (two butterflies per clock), shared with the payload: with pl_en the
// front end (de-rotator, FFT, unload) carries on after the 4 control-field symbols with payload
// symbol p = 0, 1, ... (stream symbol 4 + p, same window and phase ramp) while the back end
// (cf_chest, Viterbi, CRC) decodes the control field; the bins of payload symbols come out on
// pg_* (pg_sym = p, pg_col = 0..840). Before the control field is decided at most PL_SPEC
// payload symbols are started; then pl_dec (one clock after rec_valid) gives pl_ok (CRC ok and
// every field code valid, as pd_arm) and pl_len (PacketLength in symbols), and the front end
// stops at symbol pl_len (or at once if not pl_ok). A payload symbol starts only with pl_room.
// Without pl_en the front end stops after the control field (4560 samples), as before.
// s_more: the front end will still take samples (pd_cf fetches for it while it is set).
// FFT job port: pl_chest's delay profiles run on this FFT (job_req -> job_gnt while idle or after a
// symbol's unload; the sample input waits meanwhile). A start that comes during a job is kept.
// The decision (pl_ok) also needs a valid payload grid and one Tx antenna (cf_reference.fields_valid).
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
    input  wire               pl_en,
    input  wire               pl_room,
    output wire               s_more,
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
    // payload symbols (speculative) and the decision
    output wire               pg_valid,
    output wire [6:0]         pg_sym,
    output wire [9:0]         pg_col,
    output wire signed [17:0] pg_re,
    output wire signed [17:0] pg_im,
    output reg                pl_dec,
    output reg                pl_ok,
    output reg  [6:0]         pl_len,
    // FFT job (pl_chest's delay profiles): granted between two payload symbols or while idle;
    // with job_gnt the FFT's input, start and output address are the job's
    input  wire               job_req,
    output wire               job_gnt,
    input  wire               job_we,
    input  wire [9:0]         job_idx,
    input  wire signed [17:0] job_re,
    input  wire signed [17:0] job_im,
    input  wire               job_start,
    input  wire [9:0]         job_oaddr,
    output wire               job_done,
    output wire signed [17:0] job_ore,
    output wire signed [17:0] job_oim,
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
    localparam integer PL_SPEC = 8;             // payload symbols started before the decision
    // front end
    localparam [2:0] F_IDLE = 3'd0, F_IN = 3'd1, F_DRAIN = 3'd2, F_FFT = 3'd3, F_FWAIT = 3'd4,
                     F_UNLOAD = 3'd5, F_JOB = 3'd6;
    // back end
    localparam [2:0] B_IDLE = 3'd0, B_CE = 3'd1, B_CEW = 3'd2, B_VIT = 3'd3, B_CRC = 3'd4, B_DEC = 3'd5;
    reg [2:0]  fst, bst;
    reg [23:0] inc, ph;
    reg [10:0] pos;                             // sample position in the symbol, 0..1139
    reg [6:0]  isym;                            // symbol of the input sample (0..3 control field)
    reg [6:0]  fsym;                            // symbol in the FFT
    reg [9:0]  cnt;
    reg        pl_on;                           // pl_en latched at start
    reg        decided, d_ok;
    reg [6:0]  d_len;
    reg        be_go;
    reg        st_pend;                         // start seen while a job had the FFT
    reg  [2:0] j_ret;                           // front-end state after the job
    assign busy = (fst != F_IDLE) || (bst != B_IDLE) || be_go || start || st_pend;
    assign job_gnt = (fst == F_JOB);
    // gate at the first sample of every payload symbol
    wire       at_pl  = (pos == 11'd0) && (isym >= 7'd4);
    wire [6:0] pidx   = isym - 7'd4;
    wire       pl_end = !pl_on || (decided && !(d_ok && pidx < d_len));
    wire       pl_go  = !pl_end && pl_room && (decided || pidx < PL_SPEC);
    assign s_ready = (fst == F_IN) && (!at_pl || pl_go);
    assign s_more  = (fst != F_IDLE);
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
    pd_fft2 u_fft (.clk(clk), .rst_n(rst_n), .in_we(job_gnt ? job_we : f_we), .in_idx(job_gnt ? job_idx : f_idx),
                  .in_re(job_gnt ? job_re : f_re), .in_im(job_gnt ? job_im : f_im),
                  .start(job_gnt ? job_start : fft_start), .busy(), .done(fft_done),
                  .out_addr(job_gnt ? job_oaddr : fft_oaddr), .out_re(fft_ore), .out_im(fft_oim));
    assign job_done = fft_done;
    assign job_ore = fft_ore;
    assign job_oim = fft_oim;

    // unload: issue column cnt (R_UNLOAD), data one clock later
    reg        u_v;
    reg  [9:0] u_c;
    reg  [1:0] u_m3;                            // cnt mod 3 at issue
    reg  [1:0] c_m3;
    reg  [8:0] u_j, c_j;                        // pilot index of column cnt
    wire       ref_neg;
    cf_ref_rom u_ref (.clk(clk), .addr(c_j), .q(ref_neg));
    always @(posedge clk) begin
        u_v <= (fst == F_UNLOAD) && (cnt <= 10'd840);
        u_c <= cnt; u_m3 <= c_m3; u_j <= c_j;
        if (!rst_n) u_v <= 1'b0;
    end
    wire        cf_sym = (fsym[6:2] == 5'd0);    // control-field symbol 0..3
    wire        g_we = u_v && cf_sym;
    wire [11:0] g_waddr = {fsym[1:0], u_c};
    wire [35:0] g_wdata = {fft_ore, fft_oim};
    wire        p_we = g_we && (u_m3 == 2'd2);
    assign pg_valid = u_v && !cf_sym;
    assign pg_sym   = fsym - 7'd4;
    assign pg_col   = u_c;
    assign pg_re    = fft_ore;
    assign pg_im    = fft_oim;
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
    pd_sdp_ram #(.W(38), .A(9)) u_p0 (.clk(clk), .we(p_we && fsym == 7'd0), .waddr(u_j),
                                      .wdata({pr, pq}), .raddr(ce_paddr), .rdata(ce_p0));
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(38), .A(9)) u_p3 (.clk(clk), .we(p_we && fsym == 7'd3), .waddr(u_j),
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

    // ================================================================ control-field decision
    // PacketLength, CBS_A_Flag, RM_A_Flag codes (MSB first) as cf_reference.CF_VALID / pd_arm
    wire [2:0] c_pl  = {vbits[4], vbits[5], vbits[6]};
    wire [1:0] c_cbs = {vbits[10], vbits[11]};
    wire [3:0] c_rm  = {vbits[14], vbits[15], vbits[16], vbits[17]};
    reg  [6:0] c_len;
    always @* begin
        case (c_pl)
            3'b001: c_len = 7'd4;
            3'b011: c_len = 7'd37;
            3'b111: c_len = 7'd49;
            3'b100: c_len = 7'd73;
            3'b110: c_len = 7'd97;
            3'b101: c_len = 7'd121;
            default: c_len = 7'd0;
        endcase
    end
    // also: a valid payload grid (PacketLength 4 only with RefSymbolSpacing 1 or 3) and one Tx antenna
    wire f_ok = (c_len != 7'd0) && (c_cbs != 2'b11) && (c_rm <= 4'b1000) &&
                !(c_pl == 3'b001 && vbits[0]) && !vbits[9];

    // ================================================================ front end
    always @(posedge clk) begin
        if (!rst_n) begin
            fst <= F_IDLE; fft_start <= 1'b0; be_go <= 1'b0; st_pend <= 1'b0;
        end else begin
            fft_start <= 1'b0; be_go <= 1'b0;
            if (start && !(fst == F_IDLE && bst == B_IDLE && !job_req)) st_pend <= 1'b1;
            case (fst)
                F_IDLE: if (job_req && !start && !st_pend) begin
                    j_ret <= F_IDLE; fst <= F_JOB;
                end else if ((start || st_pend) && bst == B_IDLE) begin
                    st_pend <= 1'b0;
                    inc <= cfo_inc; ph <= 24'd0; pos <= 11'd0; isym <= 7'd0; fsym <= 7'd0;
                    rec_t_trig <= t_trig; rec_n_end <= n_end; pl_on <= pl_en;
                    fst <= F_IN;
                end
                F_IN: if (at_pl && pl_end) fst <= F_IDLE;
                      else if (acc) begin
                    ph <= ph + inc;
                    if (pos == 11'd1139) begin pos <= 11'd0; isym <= isym + 7'd1; end
                    else pos <= pos + 11'd1;
                    if (pos == 11'd1127) begin cnt <= 10'd0; fst <= F_DRAIN; end
                end
                F_DRAIN: if (cnt == 10'd6) begin fft_start <= 1'b1; fst <= F_FFT; end
                         else cnt <= cnt + 10'd1;
                F_FFT: fst <= F_FWAIT;
                F_FWAIT: if (fft_done) begin
                    cnt <= 10'd0; c_m3 <= 2'd0; c_j <= 9'd0; fst <= F_UNLOAD;
                end
                F_UNLOAD: begin
                    c_m3 <= (c_m3 == 2'd2) ? 2'd0 : c_m3 + 2'd1;
                    if (c_m3 == 2'd2) c_j <= c_j + 9'd1;
                    if (cnt == 10'd842) begin
                        if (fsym == 7'd3) be_go <= 1'b1;
                        fsym <= fsym + 7'd1;
                        if (job_req) begin j_ret <= F_IN; fst <= F_JOB; end
                        else fst <= F_IN;
                    end else cnt <= cnt + 10'd1;
                end
                F_JOB: if (!job_req) fst <= j_ret;
                default: fst <= F_IDLE;
            endcase
        end
    end

    // ================================================================ back end
    always @(posedge clk) begin
        if (!rst_n) begin
            bst <= B_IDLE; rec_valid <= 1'b0; ce_start <= 1'b0; vit_start <= 1'b0; crc_start <= 1'b0;
            pl_dec <= 1'b0; decided <= 1'b0;
        end else begin
            rec_valid <= 1'b0; ce_start <= 1'b0; vit_start <= 1'b0; crc_start <= 1'b0; pl_dec <= 1'b0;
            if (fst == F_IDLE && start && bst == B_IDLE) decided <= 1'b0;
            case (bst)
                B_IDLE: if (be_go) begin ce_start <= 1'b1; bst <= B_CE; end
                B_CE:   bst <= B_CEW;
                B_CEW: if (ce_done) begin vit_start <= 1'b1; rec_shift <= ce_shift; bst <= B_VIT; end
                B_VIT: if (vit_done) begin crc_start <= 1'b1; rec_metric <= vmetric; bst <= B_CRC; end
                B_CRC: if (crc_done) begin
                    rec_valid <= 1'b1; rec_crc_ok <= crc_ok; rec_msg <= vbits[53:0];
                    d_ok <= crc_ok && f_ok; d_len <= c_len;
                    bst <= B_DEC;
                end
                B_DEC: begin
                    pl_dec <= 1'b1; pl_ok <= d_ok; pl_len <= d_len; decided <= 1'b1;
                    bst <= B_IDLE;
                end
                default: bst <= B_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
