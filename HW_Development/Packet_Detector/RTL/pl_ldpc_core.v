// pl_ldpc_core.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// LDPC decoder core of the payload receiver: one 802.11n code word at a time (N = 648 / 1296 /
// 1944, every rate), 27 lanes. Bit-true to pl_reference.decode_ldpc (PC/pl_reference.py, step 12;
// Docs/Specification_PayloadReceiver.md section 4):
//   layered normalised min-sum, layers in prototype row order, APP 10 bit (+-511), messages 7 bit,
//   alpha = m - m >> 2, minimum = first minimum in column order, at most MAXIT iterations, stop
//   after the first iteration whose hard decisions (APP < 0) meet every parity check.
//
// Interface
//   start (one clock, while idle) with code = 4 n_sel + rate_sel (pl_ldpc_rom.v); then
//   24 G words on in_* (valid / ready): word m = code-word bits 27 m .. 27 m + 26, lane b = bits
//   [7 b + 6 : 7 b], signed 7-bit LLR (ln P0/P1, positive = 0); then 24 G words on out_*: hard
//   bits of the same positions (bit b = code-word bit 27 m + b), out_last on the last word; the
//   iteration count is on iters from the first out word until the next start. cycles counts the
//   clocks of the decoding (testbench).
//
// Architecture (27 lanes = one row group of 27 rows of a layer; G = Z / 27 row groups):
//   APP     27 banks (one block RAM each, 128 x 10): variable v of block column c is in bank
//           v mod 27 at address {c, v div 27}. Row group g of a circulant with shift
//           sh = 27 sh27 + s27 needs v = (27 g + i + sh) mod Z in lane i: bank (i + s27) mod 27,
//           address {c, w0} (w0 = (g + sh27) mod G) or {c, w0 + 1 mod G} for banks below s27.
//           Lanes = banks rotated by s27; written back the same way (inverse rotation).
//   P1      per item (edge e of the layer, row group g; edges outer, g inner): q = sat(L - R_old),
//           R_old from the compressed messages of the previous iteration (state RAM: per row
//           mn1', mn2', idx; sign RAM: one sign per edge and row), |q| capped at 63, running
//           first / second minimum, index, sign product per row (two banks of G sets, by layer
//           parity); q into a FIFO. A column is read only after the previous layer has written
//           it back (scoreboard pend[parity][col]).
//   P2      (pipeline T1 issue, T2 minima select, T2b r and L, T3 rotation, T4 write, T5 release)
//           per item of a finished layer: r = sign * (e == idx ? mn2' : mn1'), L = sat(q + r),
//           written back with its sign bit and the new compressed messages.
//   Syndrome after each iteration (except the last): every check row from the hard-bit copies
//           (2 LUT RAM copies, 2 edges per clock), while P1 already starts the next iteration's
//           first layer (P1 writes nothing that a stop would leave behind).
`timescale 1ns / 1ps
`default_nettype none

module pl_ldpc_core #(
    parameter integer MAXIT = 10
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [3:0]         code,
    output wire               busy,
    input  wire               in_valid,
    output wire               in_ready,
    input  wire [27*7-1:0]    in_llr,
    output reg                out_valid,
    input  wire               out_ready,
    output reg  [26:0]        out_bits,
    output reg                out_last,
    output reg  [3:0]         iters,
    output reg  [15:0]        cycles
);
    localparam integer NL = 27;
    localparam integer AMAX = 511;

    // ------------------------------------------------------------------ helpers
    function [NL*10-1:0] rot10;            // y[i] = x[(i + s) mod 27]
        input [NL*10-1:0] x;
        input [4:0] s;
        reg [NL*10-1:0] a, b;
        integer k, i, r;
        begin
            a = x;
            for (k = 0; k < 5; k = k + 1) begin
                r = (1 << k) % NL;
                for (i = 0; i < NL; i = i + 1)
                    b[i*10 +: 10] = a[((i + r) % NL)*10 +: 10];
                if (s[k]) a = b;
            end
            rot10 = a;
        end
    endfunction

    function [NL-1:0] rot1;
        input [NL-1:0] x;
        input [4:0] s;
        reg [NL-1:0] a, b;
        integer k, i, r;
        begin
            a = x;
            for (k = 0; k < 5; k = k + 1) begin
                r = (1 << k) % NL;
                for (i = 0; i < NL; i = i + 1)
                    b[i] = a[(i + r) % NL];
                if (s[k]) a = b;
            end
            rot1 = a;
        end
    endfunction

    function [4:0] inv27;                   // (27 - s) mod 27
        input [4:0] s;
        inv27 = (s == 5'd0) ? 5'd0 : 5'd27 - s;
    endfunction

    function signed [9:0] sat511;
        input signed [11:0] v;
        sat511 = (v > 12'sd511) ? 10'sd511 : (v < -12'sd511) ? -10'sd511 : v[9:0];
    endfunction

    function [1:0] modg;                    // v mod G for v < 2G
        input [2:0] v;
        input [1:0] g;                      // G
        modg = (v >= {1'b0, g}) ? v - g : v;
    endfunction

    // ------------------------------------------------------------------ code parameters
    wire [28:0] hdr;
    reg  [3:0]  code_r;
    pl_ldpc_code_rom u_hdr (.code(code_r), .d(hdr));
    wire [10:0] c_base = hdr[28:18];
    wire [6:0]  c_nent = hdr[17:11];
    wire [3:0]  c_nl   = hdr[5:2];
    wire [1:0]  c_G    = hdr[1:0] + 2'd1;

    // ------------------------------------------------------------------ top state
    localparam [1:0] S_IDLE = 2'd0, S_LOAD = 2'd1, S_DEC = 2'd2, S_OUT = 2'd3;
    reg [1:0] st;
    assign busy = (st != S_IDLE);
    assign in_ready = (st == S_LOAD);

    reg [4:0] ld_c;                         // load / output position: column, word
    reg [1:0] ld_w;
    wire ld_last = (ld_c == 5'd23) && (ld_w == c_G - 2'd1);

    // ------------------------------------------------------------------ memories
    // APP banks
    reg  [NL-1:0]       app_we_b;
    wire [NL*7-1:0]     app_addr_a;
    reg  [NL*7-1:0]     app_addr_b;
    reg  [NL*10-1:0]    app_wd_b;
    wire [NL*10-1:0]    app_rd_a;
    genvar gb;
    generate
        for (gb = 0; gb < NL; gb = gb + 1) begin : g_app
            cf_tdp_ram #(.W(10), .A(7)) u_bank (
                .clk(clk),
                .we_a(1'b0), .addr_a(app_addr_a[gb*7 +: 7]), .wd_a(10'd0), .rd_a(app_rd_a[gb*10 +: 10]),
                .we_b(app_we_b[gb]), .addr_b(app_addr_b[gb*7 +: 7]), .wd_b(app_wd_b[gb*10 +: 10]), .rd_b());
        end
    endgenerate

    // hard-bit copies (LUT RAM, asynchronous read): copy 0 -> syndrome edge a and the output,
    // copy 1 -> syndrome edge b; written together with the APP banks
    reg  [NL*7-1:0]     hb_raddr0, hb_raddr1;
    reg  [NL*7-1:0]     hb_ra0;                 // copy 0 read address (output or syndrome)
    wire [NL-1:0]       hb_rd0, hb_rd1;
    reg  [NL-1:0]       hb_wd;
    generate
        for (gb = 0; gb < NL; gb = gb + 1) begin : g_hb
            pl_lutram1 u_h0 (.clk(clk), .we(app_we_b[gb]), .waddr(app_addr_b[gb*7 +: 7]), .wd(hb_wd[gb]),
                             .raddr(hb_ra0[gb*7 +: 7]), .rd(hb_rd0[gb]));
            pl_lutram1 u_h1 (.clk(clk), .we(app_we_b[gb]), .waddr(app_addr_b[gb*7 +: 7]), .wd(hb_wd[gb]),
                             .raddr(hb_raddr1[gb*7 +: 7]), .rd(hb_rd1[gb]));
        end
    endgenerate

    // compressed messages: per (layer, g) and lane {idx[4:0], mn2'[5:0], mn1'[5:0]}
    reg               st_we;
    reg  [5:0]        st_waddr;
    wire [5:0]        st_raddr;
    reg  [NL*17-1:0]  st_wd;
    wire [NL*17-1:0]  st_rd;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(NL*17), .A(6)) u_state (.clk(clk), .we(st_we), .waddr(st_waddr), .wdata(st_wd),
                                           .raddr(st_raddr), .rdata(st_rd));
    // message signs: per (edge, g) and lane
    reg               sg_we;
    reg  [8:0]        sg_waddr;
    wire [8:0]        sg_raddr;
    reg  [NL-1:0]     sg_wd;
    wire [NL-1:0]     sg_rd;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(NL), .A(9)) u_sign (.clk(clk), .we(sg_we), .waddr(sg_waddr), .wdata(sg_wd),
                                       .raddr(sg_raddr), .rdata(sg_rd));
    // q FIFO between P1 and P2
    reg               qf_we;
    reg  [7:0]        qf_wp, qf_rp;
    reg  [NL*10-1:0]  qf_wd;
    wire [NL*10-1:0]  qf_rd;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(NL*10), .A(8)) u_qfifo (.clk(clk), .we(qf_we), .waddr(qf_wp), .wdata(qf_wd),
                                           .raddr(qf_rp), .rdata(qf_rd));

    // edge ROMs: P1 (a) / P2 (b), syndrome (a / b)
    reg  [10:0] e1_next, e2_next, ya_next, yb_next;
    wire [17:0] e1_d, e2_d, ya_d, yb_d;
    pl_ldpc_edge_rom u_rom12 (.clk(clk), .addr_a(e1_next), .d_a(e1_d), .addr_b(e2_next), .d_b(e2_d));
    pl_ldpc_edge_rom u_romsy (.clk(clk), .addr_a(ya_next), .d_a(ya_d), .addr_b(yb_next), .d_b(yb_d));

    // ------------------------------------------------------------------ layer bookkeeping
    // global layer numbers q = it * nl + layer
    reg  [7:0] p1_q, p1_fin, p2_q, p2_fin;     // P1 / P2 current layer, layers finished
    reg  [3:0] p1_lay, p2_lay;                 // layer within the iteration
    reg  [3:0] p1_it, p2_it;
    reg        p1_run, p2_run;                 // inside a layer
    reg  [10:0] e1, e2;                        // ROM address of the current edge
    reg  [1:0]  g1, g2;
    reg  [23:0] pend0, pend1;
    reg         syn_busy, syn_ok_q, syn_done;  // syndrome of the iteration just finished
    reg  [3:0]  syn_it;                        // iteration whose syndrome is (being) checked
    reg         stop;

    wire [17:0] e1c = e1_d;                    // current P1 edge (ROM output follows e1)
    wire [4:0]  e1_col = e1c[4:0];
    wire [4:0]  e1_s27 = e1c[9:5];
    wire [1:0]  e1_sh27 = e1c[11:10];
    wire [4:0]  e1_eo = e1c[16:12];
    wire        e1_last = e1c[17];
    wire        p1_par = p1_q[0];
    wire        haz1 = p1_par ? pend0[e1_col] : pend1[e1_col];     // previous layer still writing
    // P1 may begin layer p1_q once P2 has finished p1_q - 2, before the last iteration ends
    wire        p1_may = (p2_fin + 8'd1 >= p1_q) && (p1_it < MAXIT) && !stop;
    wire        p1_issue = (st == S_DEC) && p1_run && !haz1;
    wire        p1_item_last = e1_last && (g1 == c_G - 2'd1);

    // P2 may begin layer p2_q once P1 has finished it; layer 0 of a later iteration waits for
    // the syndrome of the previous one
    wire        p2_first_of_it = (p2_lay == 4'd0) && (p2_q != 8'd0);
    wire        p2_may = (p1_fin > p2_q) && !stop && (!p2_first_of_it || (syn_done && !syn_ok_q && syn_it + 4'd1 == p2_it));
    wire [17:0] e2c = e2_d;
    wire        e2_last = e2c[17];
    wire        p2_issue = (st == S_DEC) && p2_run;
    wire        p2_item_last = e2_last && (g2 == c_G - 2'd1);

    // ROM addresses: the value e1 / e2 take next clock, so the ROM output always belongs to the
    // current edge
    wire load_done = (st == S_LOAD) && in_valid && ld_last;
    always @* begin
        if (load_done) e1_next = c_base;
        else if (p1_issue && g1 == c_G - 2'd1)
            e1_next = (e1_last && p1_lay == c_nl - 4'd1) ? c_base : e1 + 11'd1;
        else e1_next = e1;
        if (load_done) e2_next = c_base;
        else if (p2_issue && g2 == c_G - 2'd1)
            e2_next = (e2_last && p2_lay == c_nl - 4'd1) ? c_base : e2 + 11'd1;
        else e2_next = e2;
    end

    // ------------------------------------------------------------------ P1 pipeline
    // stage 1 = issue (reads), 2 = data (rotate, R_old), 3 = q, 4 = min update + FIFO write
    reg         s2_v, s3_v, s4_v;
    reg  [4:0]  s2_s27;
    reg  [4:0]  s2_eo, s3_eo, s4_eo;
    reg  [1:0]  s2_g, s3_g, s4_g;
    reg         s2_par, s3_par, s4_par;
    reg         s2_it0;
    reg         s2_end, s3_end, s4_end;       // last item of the layer
    reg  [NL*10-1:0] s3_l;
    reg  [NL*7-1:0]  s3_r;                    // R_old, signed 7 bit
    reg  [NL*10-1:0] s4_q;
    reg  [NL*6-1:0]  s4_mag;                  // min(|q|, 63)
    reg  [NL-1:0]    s4_neg;                  // q < 0

    // per lane min state: [bank][g]
    reg  [6:0] mn1 [0:1][0:2][0:NL-1];
    reg  [6:0] mn2 [0:1][0:2][0:NL-1];
    reg  [4:0] mi1 [0:1][0:2][0:NL-1];
    reg        mpr [0:1][0:2][0:NL-1];
    reg        fresh [0:1][0:2];

    integer i;
    reg [NL*10-1:0] rot_tmp;
    reg [5:0]  t_m1, t_m2, t_mag;
    reg [4:0]  t_i1;
    reg signed [6:0] t_r;
    reg signed [11:0] t_sum;
    reg [6:0]  t_abs;
    reg signed [9:0] t_a10;
    reg signed [9:0] t_q;

    // ------------------------------------------------------------------ P2 pipeline
    // T1 = issue (FIFO read), T2 = r and L, T3 = inverse rotation, T4 = write, T5 = clear pend
    reg         t2_v, t3_v, t4_v, t5_v;
    reg  [4:0]  t2_col, t3_col, t4_col, t5_col;
    reg  [4:0]  t2_s27, t3_s27, t4_s27;
    reg  [1:0]  t2_sh27, t3_sh27, t4_sh27;
    reg  [4:0]  t2_eo;
    reg  [1:0]  t2_g, t3_g, t4_g, t5_g;
    reg         t2_par, t3_par, t4_par, t5_par;
    reg         t2_first;                     // first edge of the layer: write the state RAM
    reg  [8:0]  t2_sga;
    reg  [3:0]  t2_lay;
    reg         t2_end, t3_end, t4_end, t5_end;
    reg  [NL*10-1:0] t3_L, t4_L;
    reg  [NL-1:0]    t3_h, t4_h;
    reg         e2_first;                     // current P2 edge is the first of its layer
    reg [5:0]  u_m1, u_m2, u_mag;
    // T2b stage registers
    reg         tb_v, tb_first, tb_par, tb_end;
    reg  [NL*6-1:0]  tb_m1, tb_m2;
    reg  [NL*5-1:0]  tb_i1;
    reg  [NL-1:0]    tb_pr;
    reg  [NL*10-1:0] tb_q;
    reg  [4:0]  tb_eo, tb_col, tb_s27;
    reg  [1:0]  tb_sh27, tb_g;
    reg  [8:0]  tb_sga;
    reg  [3:0]  tb_lay;
    reg signed [11:0] u_sum;
    reg signed [9:0] u_L;
    reg        u_sg;

    // ------------------------------------------------------------------ syndrome
    reg  [10:0] ya, y_lstart;                 // current pair (ya, ya + 1)
    reg  [3:0]  y_lay;
    reg  [1:0]  y_g;
    reg         y_run;
    reg         y1_v, y1_b, y1_end, y1_done;   // stage 1: read hard bits
    reg  [4:0]  y1_sa, y1_sb;
    reg  [NL-1:0] y1_ha, y1_hb;
    reg  [NL-1:0] y_acc;
    reg         y2_v, y2_end, y2_done;         // stage 2: accumulate
    reg  [NL-1:0] y2_x;
    reg         y_fail;
    wire [17:0] yac = ya_d, ybc = yb_d;
    wire        ya_last = yac[17];
    wire        y_pair_end = ya_last || ybc[17];  // the pair ends the layer's edge list
    // syndrome stage 2 input: this pair's parity contribution (registered in stage 1)
    wire [NL-1:0] y_row = y_acc ^ y2_x;
    // the iteration p2_it - 1 is completely written back and its syndrome not yet started
    wire it_done = (st == S_DEC) && !stop && !syn_busy && !y_run && !y1_v && !y2_v && p2_fin != 8'd0 && p2_fin == p2_q &&
                   !p2_run && !t2_v && !tb_v && !t3_v && !t4_v && !t5_v && p2_lay == 4'd0 &&
                   !(syn_done && syn_it + 4'd1 == p2_it);
    always @* begin
        if (it_done) ya_next = c_base;
        else if (y_run) begin
            if (y_pair_end) begin
                if (y_g != c_G - 2'd1) ya_next = y_lstart;
                else ya_next = ya_last ? ya + 11'd1 : ya + 11'd2;
            end else ya_next = ya + 11'd2;
        end else ya_next = ya;
        yb_next = ya_next + 11'd1;
    end

    // per-bank addresses
    function [NL*7-1:0] bank_addr;
        input [4:0] col;
        input [4:0] s27;
        input [1:0] sh27;
        input [1:0] g;
        input [1:0] G;
        reg [1:0] w0, w1;
        integer b;
        begin
            w0 = modg({1'b0, g} + {1'b0, sh27}, G);
            w1 = modg({1'b0, w0} + 3'd1, G);
            for (b = 0; b < NL; b = b + 1)
                bank_addr[b*7 +: 7] = {col, (b < s27) ? w1 : w0};
        end
    endfunction

    // P1 reads of the current item (block RAM: the data come one clock later)
    wire [10:0] e1_loc = e1 - c_base;
    assign app_addr_a = bank_addr(e1_col, e1_s27, e1_sh27, g1, c_G);
    assign st_raddr = {p1_lay, g1};
    assign sg_raddr = {e1_loc[6:0], g1};

    // copy-0 read address: the output position, or the syndrome
    always @* begin : out_addr
        integer bb;
        for (bb = 0; bb < NL; bb = bb + 1)
            hb_ra0[bb*7 +: 7] = (st == S_OUT) ? {ld_c, ld_w} : hb_raddr0[bb*7 +: 7];
    end

    // ------------------------------------------------------------------ main sequential logic
    integer b, gs, bk;
    always @(posedge clk) begin
        // defaults
        e1 <= e1_next;
        e2 <= e2_next;
        ya <= ya_next;
        app_we_b <= {NL{1'b0}};
        st_we <= 1'b0;
        sg_we <= 1'b0;
        qf_we <= 1'b0;
        if (!rst_n) begin
            st <= S_IDLE;
            out_valid <= 1'b0;
            out_last <= 1'b0;
            iters <= 4'd0;
            cycles <= 16'd0;
            s2_v <= 1'b0; s3_v <= 1'b0; s4_v <= 1'b0;
            t2_v <= 1'b0; tb_v <= 1'b0; t3_v <= 1'b0; t4_v <= 1'b0; t5_v <= 1'b0;
            y_run <= 1'b0; y1_v <= 1'b0; y2_v <= 1'b0;
            p1_run <= 1'b0; p2_run <= 1'b0;
            syn_busy <= 1'b0; syn_done <= 1'b0; syn_ok_q <= 1'b0;
            stop <= 1'b0;
            code_r <= 4'd0;
        end else begin
            case (st)
            // ---------------------------------------------------------- idle / load
            S_IDLE: begin
                out_valid <= 1'b0;
                if (start) begin
                    code_r <= code;
                    st <= S_LOAD;
                    ld_c <= 5'd0;
                    ld_w <= 2'd0;
                end
            end
            S_LOAD: begin
                if (in_valid) begin
                    for (b = 0; b < NL; b = b + 1) begin
                        app_addr_b[b*7 +: 7] <= {ld_c, ld_w};
                        app_wd_b[b*10 +: 10] <= {{3{in_llr[b*7 + 6]}}, in_llr[b*7 +: 7]};
                        hb_wd[b] <= in_llr[b*7 + 6];
                    end
                    app_we_b <= {NL{1'b1}};
                    if (ld_w == c_G - 2'd1) begin ld_w <= 2'd0; ld_c <= ld_c + 5'd1; end
                    else ld_w <= ld_w + 2'd1;
                    if (ld_last) begin
                        st <= S_DEC;
                        cycles <= 16'd0;
                        // decoder state for a new code word
                        p1_q <= 8'd0; p1_fin <= 8'd0; p2_q <= 8'd0; p2_fin <= 8'd0;
                        p1_lay <= 4'd0; p2_lay <= 4'd0; p1_it <= 4'd0; p2_it <= 4'd0;
                        p1_run <= 1'b0; p2_run <= 1'b0; g1 <= 2'd0; g2 <= 2'd0;
                        pend0 <= 24'd0; pend1 <= 24'd0;
                        qf_wp <= 8'd0; qf_rp <= 8'd0;
                        syn_busy <= 1'b0; syn_done <= 1'b0; syn_ok_q <= 1'b0; syn_it <= 4'd0;
                        stop <= 1'b0;
                        e2_first <= 1'b1;
                    end
                end
            end
            // ---------------------------------------------------------- output
            S_OUT: begin
                if (!out_valid || out_ready) begin
                    if (out_valid && out_last) begin
                        out_valid <= 1'b0;
                        out_last <= 1'b0;
                        st <= S_IDLE;
                    end else begin
                        out_valid <= 1'b1;
                        out_bits <= hb_rd0;
                        out_last <= ld_last;
                        if (ld_w == c_G - 2'd1) begin ld_w <= 2'd0; ld_c <= ld_c + 5'd1; end
                        else ld_w <= ld_w + 2'd1;
                    end
                end
            end
            // ---------------------------------------------------------- decoding
            S_DEC: begin
                cycles <= cycles + 16'd1;

                // ----- P1: layer start / issue
                if (!p1_run && p1_may) begin
                    p1_run <= 1'b1;
                    g1 <= 2'd0;
                    fresh[p1_par][0] <= 1'b1; fresh[p1_par][1] <= 1'b1; fresh[p1_par][2] <= 1'b1;
                end
                if (p1_issue) begin
                    if (g1 == 2'd0) begin
                        if (p1_par) pend1[e1_col] <= 1'b1; else pend0[e1_col] <= 1'b1;
                    end
                    s2_s27 <= e1_s27; s2_eo <= e1_eo; s2_g <= g1; s2_par <= p1_par;
                    s2_it0 <= (p1_it == 4'd0);
                    s2_end <= p1_item_last;
                    if (g1 == c_G - 2'd1) begin
                        g1 <= 2'd0;
                        if (e1_last) begin
                            p1_run <= 1'b0;
                            p1_q <= p1_q + 8'd1;
                            if (p1_lay == c_nl - 4'd1) begin p1_lay <= 4'd0; p1_it <= p1_it + 4'd1; end
                            else p1_lay <= p1_lay + 4'd1;
                        end
                    end else g1 <= g1 + 2'd1;
                end
                s2_v <= p1_issue;

                // stage 2: rotate the APP word, R_old from the compressed messages
                s3_v <= s2_v;
                if (s2_v) begin
                    s3_l <= rot10(app_rd_a, s2_s27);
                    for (i = 0; i < NL; i = i + 1) begin
                        t_m1 = st_rd[i*17 +: 6];
                        t_m2 = st_rd[i*17 + 6 +: 6];
                        t_i1 = st_rd[i*17 + 12 +: 5];
                        t_mag = (s2_eo == t_i1) ? t_m2 : t_m1;
                        t_r = s2_it0 ? 7'sd0 : (sg_rd[i] ? -$signed({1'b0, t_mag}) : $signed({1'b0, t_mag}));
                        s3_r[i*7 +: 7] <= t_r;
                    end
                    s3_eo <= s2_eo; s3_g <= s2_g; s3_par <= s2_par; s3_end <= s2_end;
                end

                // stage 3: q = sat(L - R_old)
                s4_v <= s3_v;
                if (s3_v) begin
                    for (i = 0; i < NL; i = i + 1) begin
                        t_sum = $signed(s3_l[i*10 +: 10]) - $signed(s3_r[i*7 +: 7]);
                        s4_q[i*10 +: 10] <= sat511(t_sum);
                        // min(|q|, 63) and the sign straight from the difference (saturating
                        // to +-511 changes neither)
                        s4_mag[i*6 +: 6] <= (t_sum > 12'sd63 || t_sum < -12'sd63) ? 6'd63 :
                                            ((t_sum < 0) ? -t_sum[5:0] : t_sum[5:0]);
                        s4_neg[i] <= (t_sum < 0);
                    end
                    s4_eo <= s3_eo; s4_g <= s3_g; s4_par <= s3_par; s4_end <= s3_end;
                end

                // stage 4: min update, FIFO write
                if (s4_v) begin
                    qf_we <= 1'b1;
                    qf_wd <= s4_q;
                    for (i = 0; i < NL; i = i + 1) begin
                        t_abs = {1'b0, s4_mag[i*6 +: 6]};
                        if (fresh[s4_par][s4_g]) begin
                            mn1[s4_par][s4_g][i] <= t_abs;
                            mn2[s4_par][s4_g][i] <= 7'd64;
                            mi1[s4_par][s4_g][i] <= s4_eo;
                            mpr[s4_par][s4_g][i] <= s4_neg[i];
                        end else begin
                            if (t_abs < mn1[s4_par][s4_g][i] ||
                                (t_abs == mn1[s4_par][s4_g][i] && s4_eo < mi1[s4_par][s4_g][i])) begin
                                mn2[s4_par][s4_g][i] <= mn1[s4_par][s4_g][i];
                                mn1[s4_par][s4_g][i] <= t_abs;
                                mi1[s4_par][s4_g][i] <= s4_eo;
                            end else if (t_abs < mn2[s4_par][s4_g][i])
                                mn2[s4_par][s4_g][i] <= t_abs;
                            mpr[s4_par][s4_g][i] <= mpr[s4_par][s4_g][i] ^ s4_neg[i];
                        end
                    end
                    fresh[s4_par][s4_g] <= 1'b0;
                    if (s4_end) p1_fin <= p1_fin + 8'd1;
                end
                if (qf_we) qf_wp <= qf_wp + 8'd1;

                // ----- P2: layer start / issue
                if (!p2_run && p2_may) begin
                    p2_run <= 1'b1;
                    g2 <= 2'd0;
                end
                t2_v <= p2_issue;
                if (p2_issue) begin
                    t2_col <= e2c[4:0]; t2_s27 <= e2c[9:5]; t2_sh27 <= e2c[11:10]; t2_eo <= e2c[16:12];
                    t2_g <= g2; t2_par <= p2_q[0]; t2_first <= e2_first; t2_lay <= p2_lay;
                    t2_sga <= {e2 - c_base, g2};
                    t2_end <= p2_item_last;
                    qf_rp <= qf_rp + 8'd1;
                    if (g2 == c_G - 2'd1) begin
                        g2 <= 2'd0;
                        e2_first <= e2_last;
                        if (e2_last) begin
                            p2_run <= 1'b0;
                            p2_q <= p2_q + 8'd1;
                            if (p2_lay == c_nl - 4'd1) begin p2_lay <= 4'd0; p2_it <= p2_it + 4'd1; end
                            else p2_lay <= p2_lay + 4'd1;
                        end
                    end else g2 <= g2 + 2'd1;
                end

                // T2: select this row group's minima (one register stage: the select by layer
                // parity and g, the scaling and the add would not fit one clock at 100 MHz)
                tb_v <= t2_v;
                if (t2_v) begin
                    for (i = 0; i < NL; i = i + 1) begin
                        u_m1 = mn1[t2_par][t2_g][i] - (mn1[t2_par][t2_g][i] >> 2);
                        u_m2 = mn2[t2_par][t2_g][i] - (mn2[t2_par][t2_g][i] >> 2);
                        tb_m1[i*6 +: 6] <= u_m1;
                        tb_m2[i*6 +: 6] <= u_m2;
                        tb_i1[i*5 +: 5] <= mi1[t2_par][t2_g][i];
                        tb_pr[i] <= mpr[t2_par][t2_g][i];
                    end
                    tb_q <= qf_rd;
                    tb_eo <= t2_eo; tb_sga <= t2_sga; tb_first <= t2_first; tb_lay <= t2_lay;
                    tb_col <= t2_col; tb_s27 <= t2_s27; tb_sh27 <= t2_sh27; tb_g <= t2_g;
                    tb_par <= t2_par; tb_end <= t2_end;
                end

                // T2b: r, new L, signs, compressed messages
                t3_v <= tb_v;
                if (tb_v) begin
                    for (i = 0; i < NL; i = i + 1) begin
                        t_q = tb_q[i*10 +: 10];
                        u_m1 = tb_m1[i*6 +: 6];
                        u_m2 = tb_m2[i*6 +: 6];
                        u_mag = (tb_eo == tb_i1[i*5 +: 5]) ? u_m2 : u_m1;
                        u_sg = tb_pr[i] ^ (t_q < 0);
                        u_sum = t_q + (u_sg ? -$signed({1'b0, u_mag}) : $signed({1'b0, u_mag}));
                        u_L = sat511(u_sum);
                        t3_L[i*10 +: 10] <= u_L;
                        t3_h[i] <= u_L[9];
                        sg_wd[i] <= u_sg;
                        st_wd[i*17 +: 17] <= {tb_i1[i*5 +: 5], u_m2, u_m1};
                    end
                    sg_we <= 1'b1;
                    sg_waddr <= tb_sga;
                    st_we <= tb_first;
                    st_waddr <= {tb_lay, tb_g};
                    t3_col <= tb_col; t3_s27 <= tb_s27; t3_sh27 <= tb_sh27; t3_g <= tb_g;
                    t3_par <= tb_par; t3_end <= tb_end;
                end

                // T3: back to bank order
                t4_v <= t3_v;
                if (t3_v) begin
                    t4_L <= rot10(t3_L, inv27(t3_s27));
                    t4_h <= rot1(t3_h, inv27(t3_s27));
                    t4_col <= t3_col; t4_s27 <= t3_s27; t4_sh27 <= t3_sh27; t4_g <= t3_g;
                    t4_par <= t3_par; t4_end <= t3_end;
                end

                // T4: write
                t5_v <= t4_v;
                if (t4_v) begin
                    app_we_b <= {NL{1'b1}};
                    app_addr_b <= bank_addr(t4_col, t4_s27, t4_sh27, t4_g, c_G);
                    app_wd_b <= t4_L;
                    hb_wd <= t4_h;
                    t5_col <= t4_col; t5_g <= t4_g; t5_par <= t4_par; t5_end <= t4_end;
                end

                // T5: the write is in; release the column, count the layer
                if (t5_v) begin
                    if (t5_g == c_G - 2'd1) begin
                        if (t5_par) pend1[t5_col] <= 1'b0; else pend0[t5_col] <= 1'b0;
                    end
                    if (t5_end) p2_fin <= p2_fin + 8'd1;
                end

                // ----- syndrome after an iteration (not after the last one)
                if (it_done) begin
                    // P2 has written every layer of iteration p2_it - 1
                    if (p2_it == MAXIT) begin
                        stop <= 1'b1;
                        iters <= p2_it;
                    end else begin
                        syn_busy <= 1'b1;
                        syn_done <= 1'b0;
                        syn_it <= p2_it - 4'd1;
                        y_run <= 1'b1;
                        y_lstart <= c_base;
                        y_lay <= 4'd0;
                        y_g <= 2'd0;
                        y_acc <= {NL{1'b0}};
                        y_fail <= 1'b0;
                    end
                end
                // syndrome stage 0: pair (ya, ya + 1) from the ROM -> read the hard bits
                y1_v <= y_run;
                if (y_run) begin
                    hb_raddr0 <= bank_addr(yac[4:0], yac[9:5], yac[11:10], y_g, c_G);
                    hb_raddr1 <= bank_addr(ybc[4:0], ybc[9:5], ybc[11:10], y_g, c_G);
                    y1_sa <= yac[9:5];
                    y1_sb <= ybc[9:5];
                    y1_b <= !ya_last;
                    y1_end <= y_pair_end;
                    y1_done <= y_pair_end && (y_g == c_G - 2'd1) && (y_lay == c_nl - 4'd1);
                    if (y_pair_end) begin
                        if (y_g != c_G - 2'd1) y_g <= y_g + 2'd1;
                        else begin
                            y_g <= 2'd0;
                            y_lay <= y_lay + 4'd1;
                            y_lstart <= ya_next;
                            if (y_lay == c_nl - 4'd1) y_run <= 1'b0;
                        end
                    end
                end
                // syndrome stage 1: rotate the two edges' hard bits, combine
                y2_v <= y1_v;
                if (y1_v) begin
                    y2_x <= rot1(hb_rd0, y1_sa) ^ (y1_b ? rot1(hb_rd1, y1_sb) : {NL{1'b0}});
                    y2_end <= y1_end;
                    y2_done <= y1_done;
                end
                // syndrome stage 2: accumulate the row parities of (layer, g)
                if (y2_v) begin
                    y_acc <= y2_end ? {NL{1'b0}} : y_row;
                    if (y2_end && |y_row) y_fail <= 1'b1;
                    if (y2_done) begin
                        syn_busy <= 1'b0;
                        syn_done <= 1'b1;
                        syn_ok_q <= !(y_fail || |y_row);
                        if (!(y_fail || |y_row)) begin
                            stop <= 1'b1;
                            iters <= syn_it + 4'd1;
                        end
                    end
                end

                // ----- finished: output the hard bits
                if (stop) begin
                    st <= S_OUT;
                    ld_c <= 5'd0;
                    ld_w <= 2'd0;
                    out_valid <= 1'b0;
                    p1_run <= 1'b0; p2_run <= 1'b0;
                    s2_v <= 1'b0; s3_v <= 1'b0; s4_v <= 1'b0;
                    t2_v <= 1'b0; tb_v <= 1'b0; t3_v <= 1'b0; t4_v <= 1'b0; t5_v <= 1'b0;
                    y_run <= 1'b0; y1_v <= 1'b0; y2_v <= 1'b0;
                end
            end
            endcase
        end
    end

endmodule

`default_nettype wire
