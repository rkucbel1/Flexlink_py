// pl_dec.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Payload decoder stage (step 12d, Docs/Specification_PayloadReceiver.md section 4): the decoder
// input of pl_comb (Lq, two per clock in code order) into pl_ldpc_core (27 per word), the hard
// bits back, CRC24 and a transport-block record per code word into a FIFO. Bit-true to
// pl_reference.decode_soft (steps 12, 13): the transport block = the first K - 24 decoded bits,
// crc_ok = the CRC24 (Gen24, zero init) of the first K bits is 0 (the same as comparing with the
// last 24 bits), iterations as decode_ldpc.
//
// Flow: idle -> a code word's first Lq (lq_first) starts the core with lq_code; Lq are gathered
// into 27-wide words (a 40-entry buffer: lq_ready while it holds less than 27 and the code word is
// not complete); after the last word the core decodes; its output words go through the CRC and
// are packed 32 bits per word into the FIFO. A code word starts only with room for a whole
// record (64 words), so the output never waits.
//
// Record (FIFO words, the firmware reads them in order): t_trig; {crc_ok, 3'b0, iterations[3:0],
// w[9:0], 3'b0, nbits[10:0]}; then ceil(nbits / 32) words of transport-block bits, bit j in word
// j / 32, bit j mod 32. tb_cnt counts only the words of complete records.
`timescale 1ns / 1ps
`default_nettype none

module pl_dec (
    input  wire               clk,
    input  wire               rst_n,
    // decoder input (pl_comb)
    input  wire               lq_valid,
    input  wire               lq_first,
    input  wire               lq_last,
    input  wire signed [6:0]  lq_q0,
    input  wire signed [6:0]  lq_q1,
    input  wire [3:0]         lq_code,
    input  wire [9:0]         lq_w,
    input  wire [31:0]        lq_t,
    output wire               lq_ready,
    // transport-block FIFO
    output wire [11:0]        tb_cnt,         // words of complete records
    output wire [31:0]        tb_word,        // the next word (valid two clocks after a pop)
    input  wire               tb_pop,
    output wire               busy
);
    localparam integer CRC_POLY = 24'h864CFB;

    // ------------------------------------------------------------------ decoder core
    reg         c_start;
    reg  [3:0]  c_code;
    wire        c_busy, c_in_ready, c_out_valid, c_out_last;
    wire [26:0] c_out_bits;
    wire [3:0]  c_iters;
    reg         c_in_valid;
    reg  [27*7-1:0] c_in_llr;
    pl_ldpc_core #(.MAXIT(10)) u_core (.clk(clk), .rst_n(rst_n), .start(c_start), .code(c_code), .busy(c_busy),
                                       .in_valid(c_in_valid), .in_ready(c_in_ready), .in_llr(c_in_llr),
                                       .out_valid(c_out_valid), .out_ready(1'b1), .out_bits(c_out_bits),
                                       .out_last(c_out_last), .iters(c_iters), .cycles());

    // ------------------------------------------------------------------ FIFO (2048 x 32)
    reg  [11:0] f_wr, f_cm, f_rd, f_rs;        // write, commit, read, record start
    reg         f_we;
    reg  [10:0] f_wa;
    reg  [31:0] f_wd;
    wire [31:0] f_rdata;
    pd_sdp_ram #(.W(32), .A(11)) u_fifo (.clk(clk), .we(f_we), .waddr(f_wa), .wdata(f_wd),
                                         .raddr(f_rd[10:0]), .rdata(f_rdata));
    assign tb_cnt = f_cm - f_rd;
    assign tb_word = f_rdata;
    wire [11:0] f_used = f_wr - f_rd;
    wire        f_room = f_used <= 12'd1984;  // 64 words free

    // ------------------------------------------------------------------ state
    localparam [2:0] S_IDLE = 0, S_LOAD = 1, S_RUN = 2, S_FLUSH = 3, S_H0 = 4, S_H1 = 5, S_CM = 6, S_FL0 = 7;
    reg  [2:0]  st;
    reg         got_last;
    reg  [6:0]  nw;                            // input / output words of the code word (24 G)
    reg  [6:0]  wi;                            // words fed / received
    reg  [10:0] nb;                            // K - 24
    reg  [6:0]  kw;                            // K / 27: output words through the CRC
    reg  [10:0] rem;                           // transport-block bits still to take
    reg  [4:0]  tk;                            // bits to take from the next word: min(27, rem)
    reg  [26:0] msk;                           //   as a mask
    reg         obv;                           // packing stage: a masked word
    reg  [26:0] ob;
    reg  [4:0]  otk;
    reg  [31:0] r_t;
    reg  [9:0]  r_w;
    reg  [23:0] crc;
    reg  [3:0]  r_it;
    reg  [63:0] pk;                            // packing: pk_n bits in pk[pk_n - 1 : 0]
    reg  [5:0]  pk_n;

    // gather buffer: 40 x 7 bit, g_n entries, oldest at 0
    reg  [6:0]  gb [0:39];
    reg  [5:0]  g_n;
    wire        g_pop = c_in_valid && c_in_ready;
    assign lq_ready = (g_n < 6'd27) && ((st == S_IDLE && !c_busy && f_room) || (st == S_LOAD && !got_last));

    function [23:0] crc27;                     // 27 bits, bit 0 first
        input [23:0] c;
        input [26:0] d;
        integer i;
        reg [23:0] x;
        begin
            x = c;
            for (i = 0; i < 27; i = i + 1)
                x = {x[22:0], 1'b0} ^ ((x[23] ^ d[i]) ? CRC_POLY[23:0] : 24'd0);
            crc27 = x;
        end
    endfunction

    integer i;
    reg [5:0] base;
    reg [10:0] rem_n;
    reg [63:0] pk_new;
    reg [6:0]  pk_cnt;
    always @(posedge clk) begin
        c_start <= 1'b0; f_we <= 1'b0;
        if (!rst_n) begin
            st <= S_IDLE; g_n <= 6'd0; c_in_valid <= 1'b0;
            f_wr <= 12'd0; f_cm <= 12'd0; f_rd <= 12'd0; f_rs <= 12'd0;
        end else begin
            // ---------------- gather (pop 27 to the core, push 2 from the combiner)
            base = g_pop ? g_n - 6'd27 : g_n;
            if (g_pop)
                for (i = 0; i < 13; i = i + 1) gb[i] <= gb[i + 27];
            if (lq_valid) begin
                gb[base] <= lq_q0; gb[base + 6'd1] <= lq_q1;
            end
            g_n <= base + (lq_valid ? 6'd2 : 6'd0);
            // core input word: the 27 oldest (registered; at most one word in flight)
            if (g_pop) c_in_valid <= 1'b0;
            else if (st == S_LOAD && !c_in_valid && g_n >= 6'd27 && wi != nw) c_in_valid <= 1'b1;
            if (!c_in_valid || g_pop)
                for (i = 0; i < 27; i = i + 1) c_in_llr[i*7 +: 7] <= gb[i];
            // ---------------- control
            case (st)
                S_IDLE: if (lq_valid && lq_first) begin
                    c_start <= 1'b1; c_code <= lq_code;
                    r_t <= lq_t; r_w <= lq_w;
                    nw <= (lq_code[3:2] == 2'd0) ? 7'd24 : (lq_code[3:2] == 2'd1) ? 7'd48 : 7'd72;
                    // K - 24 = N rate - 24
                    case (lq_code)
                        4'd0: kw <= 7'd12; 4'd1: kw <= 7'd16; 4'd2: kw <= 7'd18; 4'd3: kw <= 7'd20;
                        4'd4: kw <= 7'd24; 4'd5: kw <= 7'd32; 4'd6: kw <= 7'd36; 4'd7: kw <= 7'd40;
                        4'd8: kw <= 7'd36; 4'd9: kw <= 7'd48; 4'd10: kw <= 7'd54; default: kw <= 7'd60;
                    endcase
                    case (lq_code)
                        4'd0: nb <= 11'd300;  4'd1: nb <= 11'd408;  4'd2: nb <= 11'd462;  4'd3: nb <= 11'd516;
                        4'd4: nb <= 11'd624;  4'd5: nb <= 11'd840;  4'd6: nb <= 11'd948;  4'd7: nb <= 11'd1056;
                        4'd8: nb <= 11'd948;  4'd9: nb <= 11'd1272; 4'd10: nb <= 11'd1434; default: nb <= 11'd1596;
                    endcase
                    wi <= 7'd0; got_last <= lq_last;
                    st <= S_LOAD;
                end
                S_LOAD: begin
                    if (lq_valid && lq_last) got_last <= 1'b1;
                    if (g_pop) begin
                        wi <= wi + 7'd1;
                        if (wi + 7'd1 == nw) begin
                            st <= S_RUN; wi <= 7'd0; crc <= 24'd0; pk_n <= 6'd0; pk <= 64'd0;
                            rem <= nb; tk <= 5'd27; msk <= {27{1'b1}};
                            f_wr <= f_rs + 12'd2;
                        end
                    end
                end
                // output words: CRC over the first K / 27, the first nb bits masked (stage A); packed
                // into 32-bit words one clock later (stage B, below)
                S_RUN: if (c_out_valid) begin
                    if (wi == 7'd0) r_it <= c_iters;
                    wi <= wi + 7'd1;
                    if (wi < kw) crc <= crc27(crc, c_out_bits);
                    ob <= c_out_bits & msk; otk <= tk;
                    rem_n = rem - {6'd0, tk};
                    rem <= rem_n;
                    tk <= (rem_n > 11'd27) ? 5'd27 : rem_n[4:0];
                    msk <= (rem_n > 11'd27) ? {27{1'b1}} : ((27'd1 << rem_n[4:0]) - 27'd1);
                    if (c_out_last) st <= S_FL0;
                end
                S_FL0: st <= S_FLUSH;               // the last word through stage B
                S_FLUSH: begin
                    if (pk_n != 6'd0) begin f_we <= 1'b1; f_wa <= f_wr[10:0]; f_wd <= pk[31:0]; f_wr <= f_wr + 12'd1; end
                    st <= S_H0;
                end
                S_H0: begin f_we <= 1'b1; f_wa <= f_rs[10:0]; f_wd <= r_t; st <= S_H1; end
                S_H1: begin
                    f_we <= 1'b1; f_wa <= f_rs[10:0] + 11'd1;
                    f_wd <= {crc == 24'd0, 3'd0, r_it, r_w, 3'd0, nb};
                    st <= S_CM;
                end
                // commit once the header is in the RAM
                S_CM: begin f_cm <= f_wr; f_rs <= f_wr; st <= S_IDLE; end
                default: st <= S_IDLE;
            endcase
            // stage B: pack
            obv <= (st == S_RUN) && c_out_valid;
            if (obv) begin
                pk_new = pk | ({37'd0, ob} << pk_n);
                pk_cnt = {1'b0, pk_n} + {2'd0, otk};
                if (pk_cnt >= 7'd32) begin
                    f_we <= 1'b1; f_wa <= f_wr[10:0]; f_wd <= pk_new[31:0]; f_wr <= f_wr + 12'd1;
                    pk <= {32'd0, pk_new[63:32]}; pk_n <= pk_cnt - 7'd32;
                end else begin
                    pk <= pk_new; pk_n <= pk_cnt[5:0];
                end
            end
            if (tb_pop && f_cm != f_rd) f_rd <= f_rd + 12'd1;
        end
    end

    assign busy = (st != S_IDLE) || c_busy || (g_n != 6'd0);
endmodule

`default_nettype wire
