// pl_chest.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Payload channel estimate (step 12b, Docs/Specification_PayloadReceiver.md section 4), bit-true
// to pl_reference.channel (steps 1-7 of PC/pl_reference.py): packet shift k, pilots, noise sum,
// delay class from the delay profile of reference rows 0 and 1 (on the shared FFT, through the
// FFT job port of cf_rx), SNR mode, 8-tap FIR per tone, drift (pd_cordic), aligned rows,
// smoothing, linear time interpolation and rotation. Output per payload symbol: H and Y' of all
// 841 tones in column order (h_*), and one packet record before symbol 0 (pk_*).
//
// Input: the payload bins of cf_rx (pg_*), speculative, and the decision (pl_dec / pl_ok with
// the control message and t_trig). Bins go into a 16-slot grid ring (one block RAM per symbol,
// slot = symbol count mod 16); room = a slot is free (cf_rx starts a payload symbol only with
// room). An accepted packet is queued (2 deep); a rejected one drops its symbols, also one still
// in the FFT.
//
// One sequential engine does the work. Start-up, once symbols 0 and R are in: k, rows 0 and 1
// (pilots, noise, pilot power), their delay profiles (two FFT jobs), class, SNR mode, packet
// record. Then, in this priority (pl_reference.chest_needs_row):
//   1. H(s) of the next symbol, once Sm(j) and Sm(j + 1) exist, j = min(s / R, nref - 2)
//   2. the next smoothed row Sm(m), once the aligned rows up to min(nref - 1, m + T / 2) exist
//   3. the next reference row r, once its symbol is in: pilots (r >= 2), drift r - 1 -> r, FIR and
//      rotation by -cum(r) into the row store (10 rows)
// Clocks: per reference row about 5400, per symbol about 860 (RefSymbolSpacing 1: about 6600 per
// symbol, a little slower than 20 Msps; the FFT then waits for room).
//
// Every loop issues one memory read per clock (cycle C0); the data are valid two clocks later
// (C2: the address register, then the registered read) and registered again; processing runs in
// stages 3 .. 6 and the shared rotation unit, never more than one add or multiply deep per clock
// (100 MHz). The issue tags (tg*) carry the loop type through the pipeline. Counters keep
// nxt / R, mod R, mod 3 and mod 10 running, so the engine has no division or modulo.
`timescale 1ns / 1ps
`default_nettype none

module pl_chest (
    input  wire               clk,
    input  wire               rst_n,
    // bins from cf_rx
    input  wire               pg_valid,
    input  wire [6:0]         pg_sym,
    input  wire [9:0]         pg_col,
    input  wire signed [17:0] pg_re,
    input  wire signed [17:0] pg_im,
    input  wire               pl_dec,
    input  wire               pl_ok,
    input  wire [53:0]        pl_msg,
    input  wire [31:0]        pl_t,
    output wire               room,
    // FFT job (cf_rx): with job_gnt the FFT's ports are ours
    output reg                job_req,
    input  wire               job_gnt,
    output reg                job_we,
    output reg  [9:0]         job_idx,
    output reg  signed [17:0] job_re,
    output reg  signed [17:0] job_im,
    output reg                job_start,
    input  wire               fft_done,
    output reg  [9:0]         job_oaddr,
    input  wire signed [17:0] fft_ore,
    input  wire signed [17:0] fft_oim,
    // output
    input  wire               out_room,      // the consumer takes one more packet record / symbol
    output reg                pk_valid,
    output reg  [31:0]        pk_t,
    output reg  [31:0]        pk_a,          // k | class << 8 | hi << 12 | n0 << 16
    output reg  [31:0]        pk_b,          // sumN mod 2^32
    output reg                h_valid,
    output reg  [31:0]        h_t,
    output reg  [6:0]         h_sym,
    output reg  [9:0]         h_col,
    output reg  signed [15:0] h_re,
    output reg  signed [15:0] h_im,
    output reg  signed [15:0] y_re,
    output reg  signed [15:0] y_im,
    output wire               busy,
    output wire               waiting,       // busy only waiting for symbols or the FFT
    output reg                err            // an accepted packet while the queue is full (sticky)
);
`include "pl_coef_base.vh"

    // ------------------------------------------------------------------ grid ring and packet queue
    reg  [7:0]  wr_ptr, fr_ptr, w_base;
    reg         w_drop, w_on;
    wire [7:0]  w_slot = w_base + {1'b0, pg_sym};
    wire [13:0] g_waddr = {w_slot[3:0], pg_col};
    reg  [13:0] g_raddr;
    wire [35:0] g_rd;
    reg         g_we_q;
    reg  [13:0] g_waddr_q;
    reg  [35:0] g_wd_q;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(36), .A(14)) u_grid (.clk(clk), .we(g_we_q), .waddr(g_waddr_q), .wdata(g_wd_q),
                                        .raddr(g_raddr), .rdata(g_rd));
    wire signed [17:0] g_yr = g_rd[35:18];
    wire signed [17:0] g_yi = g_rd[17:0];
    assign room = ((wr_ptr - fr_ptr) < 8'd16);

    reg  [1:0]  q_n;                           // descriptors queued, head at 0
    reg  [7:0]  q_base [0:1];
    reg  [53:0] q_msg [0:1];
    reg  [31:0] q_t [0:1];
    reg         q_pop;
    wire        new_pkt = pg_valid && pg_sym == 7'd0 && pg_col == 10'd0;
    wire [1:0]  q_wi = q_pop ? q_n - 2'd1 : q_n;

    always @(posedge clk) begin
        g_we_q <= 1'b0;
        if (!rst_n) begin
            wr_ptr <= 8'd0; w_base <= 8'd0; w_drop <= 1'b0; w_on <= 1'b0; q_n <= 2'd0; err <= 1'b0;
        end else begin
            // a packet's first bin starts it at the current write position
            g_we_q <= pg_valid && (new_pkt || (w_on && !w_drop));
            g_waddr_q <= new_pkt ? {wr_ptr[3:0], 10'd0} : g_waddr;
            g_wd_q <= {pg_re, pg_im};
            if (new_pkt) begin
                w_base <= wr_ptr; w_drop <= 1'b0; w_on <= 1'b1;
            end else if (pg_valid && w_on && !w_drop && pg_col == 10'd840)
                wr_ptr <= w_slot + 8'd1;
            if (pl_dec) begin
                if (pl_ok) begin
                    if (q_n == 2'd2 && !q_pop) err <= 1'b1;
                    else begin
                        q_base[q_wi] <= w_base; q_msg[q_wi] <= pl_msg; q_t[q_wi] <= pl_t;
                    end
                end else begin
                    w_drop <= 1'b1;
                    wr_ptr <= w_base;
                end
            end
            if (q_pop && !(pl_dec && pl_ok && q_wi == 2'd0)) begin
                q_base[0] <= q_base[1]; q_msg[0] <= q_msg[1]; q_t[0] <= q_t[1];
            end
            q_n <= q_n + ((pl_dec && pl_ok && !(q_n == 2'd2 && !q_pop)) ? 2'd1 : 2'd0) - (q_pop ? 2'd1 : 2'd0);
        end
    end

    // ------------------------------------------------------------------ packet settings
    reg  [7:0]  base;
    reg  [31:0] cur_t;
    reg  [1:0]  mod;                           // 0 BPSK, 1 QPSK, 2 16QAM, 3 64QAM
    reg  [1:0]  scode;                         // RefSignalSpacing code
    reg  [3:0]  R;                             // RefSymbolSpacing
    reg  [6:0]  plen, nref;
    reg  [4:0]  S;
    reg  [8:0]  Np;
    reg  [9:0]  n0;                            // noise tones of rows 0 and 1
    reg  [36:0] W2;
    reg  [1:0]  ncls;                          // number of classes - 1
    reg  [7:0]  nmax;                          // delay profile bins n = 4 .. nmax - 1
    reg  [2:0]  half;                          // smoothing half length (0 in the high-SNR mode)
    reg  [14:0] RSr;                           // round(2^15 / R)
    wire [7:0]  avail_raw = wr_ptr - base;
    wire [7:0]  avail = (avail_raw > {1'b0, plen}) ? {1'b0, plen} : avail_raw;
    wire        qam = (mod != 2'd0);

    // ------------------------------------------------------------------ memories
    // pilots: 4 rows (r mod 4) x 512, {Hp_r, Hp_i} 17 + 17 bit; port A write or read, port B read
    reg         p_we;
    reg  [10:0] p_waddr, p_ra, p_rb;
    reg  [33:0] p_wd;
    wire [33:0] p_da, p_db;
    cf_tdp_ram #(.W(34), .A(11)) u_pil (.clk(clk), .we_a(p_we), .addr_a(p_we ? p_waddr : p_ra), .wd_a(p_wd),
                                       .rd_a(p_da), .we_b(1'b0), .addr_b(p_rb), .wd_b(34'd0), .rd_b(p_db));
    wire signed [16:0] pa_r = p_da[33:17], pa_i = p_da[16:0];
    wire signed [16:0] pb_r = p_db[33:17], pb_i = p_db[16:0];
    // ROMs
    reg  [11:0] tr_addr;
    wire [17:0] tr_d;
    pl_tone_rom u_trom (.clk(clk), .addr(tr_addr), .d(tr_d));
    reg  [10:0] pr_addr;
    wire [15:0] pr_d;
    pl_pil_rom u_prom (.clk(clk), .addr(pr_addr), .d(pr_d));
    reg  [12:0] cr_addr;
    wire [63:0] cr_d;
    pl_coef_rom u_crom (.clk(clk), .addr(cr_addr), .d(cr_d));
    reg  [9:0]  cs_addr;
    wire signed [15:0] cs_cos, cs_sin;
    pd_tone_rom u_cs (.clk(clk), .addr_a(cs_addr), .addr_b(cs_addr - 10'd256), .dout_a(cs_cos), .dout_b(cs_sin));
    // aligned rows A (10 slots), smoothed rows Sm (3 slots)
    reg         a_we;
    reg  [3:0]  a_wslot;
    reg  [9:0]  a_waddr, a_raddr;
    reg  [31:0] a_wd;
    wire [10*32-1:0] a_rd;
    genvar ga;
    generate
        for (ga = 0; ga < 10; ga = ga + 1) begin : g_a
            (* keep_hierarchy = "yes" *)
            pd_sdp_ram #(.W(32), .A(10)) u_a (.clk(clk), .we(a_we && a_wslot == ga), .waddr(a_waddr), .wdata(a_wd),
                                             .raddr(a_raddr), .rdata(a_rd[ga*32 +: 32]));
        end
    endgenerate
    reg         s_we;
    reg  [11:0] s_waddr, s_ra, s_rb;
    reg  [31:0] s_wd;
    wire [31:0] s_da, s_db;
    cf_tdp_ram #(.W(32), .A(12)) u_sm (.clk(clk), .we_a(s_we), .addr_a(s_we ? s_waddr : s_ra), .wd_a(s_wd),
                                      .rd_a(s_da), .we_b(1'b0), .addr_b(s_rb), .wd_b(32'd0), .rd_b(s_db));
    // delay profile (n - 4, LUT RAM)
    (* ram_style = "distributed" *) reg [37:0] pdp [0:127];
    // per reference row, r mod 16: drift phi(r) (r -> r + 1) and cum(r)
    reg  signed [15:0] phi [0:15];
    reg  signed [15:0] cum [0:15];

    // ------------------------------------------------------------------ per-packet arithmetic
    reg         ms_go;
    reg  [47:0] ms_a, ms_b;
    wire        ms_done;
    wire [95:0] ms_p;
    pl_mulseq #(.WA(48), .WB(48)) u_ms (.clk(clk), .rst_n(rst_n), .start(ms_go), .a(ms_a), .b(ms_b),
                                       .done(ms_done), .p(ms_p));
    reg         cd_go;
    reg  signed [45:0] cd_re, cd_im;
    wire        cd_done;
    wire signed [15:0] cd_ang;
    pd_cordic #(.IW(46)) u_cd (.clk(clk), .rst_n(rst_n), .start(cd_go), .re(cd_re), .im(cd_im), .busy(),
                               .done(cd_done), .angle(cd_ang));

    // ------------------------------------------------------------------ engine state
    localparam [5:0]
        E_IDLE = 0, E_CFG = 1, E_WAIT0 = 2, E_KSCAN = 3, E_K = 4, E_RSCAN = 5, E_JREQ = 6, E_JZERO = 7,
        E_JPIL = 8, E_JFFT = 9, E_JWAIT = 10, E_JREAD = 11, E_JNEXT = 12, E_A44W = 13, E_M1W = 14,
        E_M2W = 15, E_CMAX = 16, E_CL1 = 17, E_CL2 = 18, E_CL3 = 19, E_PKT = 20, E_MAIN = 21,
        E_DRIFT = 22, E_CORD = 23, E_CORDW = 24, E_FSET = 25, E_FSET2 = 26, E_FSET3 = 27, E_FIR = 28,
        E_SSET = 29, E_SMOOTH = 30, E_HSET = 31, E_HSET2 = 32, E_HSET3 = 33, E_HOUT = 34, E_DRAIN = 35,
        E_DONE = 36, E_DRIFTI = 37, E_A44 = 38, E_MAIN0 = 39, E_SSET2 = 40, E_SSET3 = 41, E_HSET4 = 42,
        E_HSET5 = 43, E_HSET6 = 44, E_CL2B = 45;
    localparam [3:0] T_NONE = 0, T_K = 1, T_SCAN = 2, T_JPIL = 3, T_JRD = 4, T_DRIFT = 5, T_FIR = 6,
                     T_SM = 7, T_H = 8;
    reg  [5:0]  es, e_ret;
    assign busy = (es != E_IDLE) || (q_n != 2'd0);
    reg  [9:0]  cnt;                           // loop counter (column, pilot, bin, tone)
    reg  [8:0]  pcnt;                          // pilot index during a row scan
    reg  [4:0]  pmod;                          // column mod S
    reg  [9:0]  kcol;                          // pilot column during the k scan
    reg  [1:0]  pair;
    reg  [4:0]  dcnt;                          // drain clocks
    reg  [6:0]  row;                           // row being scanned / processed
    reg  [7:0]  row_slot;                      // grid slot of row (base + row R)
    reg         init;                          // start-up (rows 0, 1)
    reg         j1;                            // second delay profile job
    reg  [6:0]  nxt_row, nxt_sm, nxt_sym, hsym;
    // running values kept with the counters (no divisions / modulos in the loop)
    reg  [7:0]  row_sym;                       // nxt_row * R
    reg  [3:0]  row10;                         // nxt_row mod 10
    reg  [3:0]  sm10;                          // nxt_sm mod 10
    reg  [1:0]  sm3;                           // nxt_sm mod 3
    reg  [6:0]  sym_j;                         // nxt_sym / R
    reg  [3:0]  sym_t;                         // nxt_sym mod R
    reg  [4:0]  k;
    reg  [6:0]  rk;                            // rounding constant of the shift k
    reg  [19:0] mx;
    reg  [47:0] pw, sumn;
    reg  [39:0] a44;
    reg  [57:0] m_lhs;
    reg         hi;
    reg  [1:0]  cls;
    reg  [12:0] cbase;                         // pl_coef_base of the packet
    reg  [37:0] pmax, pv;
    reg  [47:0] pvn;
    reg  [28:0] pvl, pvh;
    reg  [6:0]  last;
    reg  signed [47:0] dr, di;
    reg  [1:0]  wv [0:9];                      // smoothing weight per A slot
    reg  [16:0] recip;
    reg  [3:0]  ht;                            // t of the symbol
    reg  [6:0]  hj;                            // j of the symbol
    reg  [1:0]  hj_m3, hj1_m3;                 // Sm slots of rows j, j + 1
    reg  [1:0]  hmode;                         // 0: Sm_j, 1: Sm_j+1, 2: interpolate
    reg  signed [15:0] th;                     // rotation angle of the row / symbol
    reg  signed [15:0] h_phi, h_c0, h_c1;
    reg  signed [20:0] h_p1;
    reg  signed [35:0] h_p2;
    reg  [3:0]  aslot;                         // r mod 10
    reg  [1:0]  sslot;                         // m mod 3
    reg  [3:0]  sb;                            // smoothing setup: window position
    reg  [3:0]  sl;                            //   its A slot
    reg  signed [7:0] sm_m;                    //   its row
    reg  [3:0]  wsum;
    reg  [9:0]  jcol;                          // pilot column during the job load
    reg  [8:0]  t_start, t_start_n;
    reg  [7:0]  t_set, t_set_n;
    reg  signed [15:0] rc, rs;                 // cos / sin for the rotation unit
    reg         hfree;
    wire [6:0]  hj_now = (sym_j > nref - 7'd2) ? nref - 7'd2 : sym_j;
    // main loop choices: registered in E_MAIN0, used in E_MAIN (nothing they depend on changes
    // in between except avail and out_room, which only grow)
    reg         go_h, go_s, go_r, go_end;
    wire [6:0]  s_need = (nxt_sm + {4'd0, half} >= nref) ? nref - 7'd1 : nxt_sm + {4'd0, half};
    wire        c_h = (nxt_sm > hj_now + 7'd1);
    wire        c_s = (nxt_sm < nref) && (nxt_sm <= hj_now + 7'd2) && (nxt_row > s_need);
    wire        c_r = (nxt_row < nref) && (avail > row_sym) && (nxt_row + {4'd0, half} < nxt_sm + 7'd10);
    // waiting for symbols or for the FFT (not for output room: the firmware drains that)
    assign waiting = (es == E_IDLE && q_n == 2'd0) || (es == E_WAIT0) || (es == E_JREQ && !job_gnt) ||
                     ((es == E_MAIN || es == E_MAIN0) && nxt_sym != plen && !c_h && !c_s && !c_r);

    // pipeline tags (stage 1 = the clock after the issue, stage 2 = memory data, stage 3 = data
    // registered, then processing)
    reg  [3:0]  tg1, tg2, tg3, tg4, tg5, tg6;
    reg  [9:0]  c1, c2, c3, c4, c5, c6;
    reg  [8:0]  i1, i2, i3, i4, i5;
    reg         f1, f2, f3, f4, f5;
    reg  [1:0]  pp1, pp2, pp3, pp4, pp5;

    // smoothing weights of position b of the window
    function [1:0] hw_of;
        input [3:0] bb;
        hw_of = ((R == 4'd3 && bb >= 4'd1 && bb <= 4'd3) || (R == 4'd6 && bb == 4'd1)) ? 2'd2 : 2'd1;
    endfunction
    function [16:0] recip_of;
        input [3:0] w;
        case (w)
            4'd1: recip_of = 17'd32768; 4'd2: recip_of = 17'd16384; 4'd3: recip_of = 17'd10923;
            4'd4: recip_of = 17'd8192;  4'd5: recip_of = 17'd6554;  4'd6: recip_of = 17'd5461;
            4'd7: recip_of = 17'd4681;  4'd8: recip_of = 17'd4096;  default: recip_of = 17'd3641;
        endcase
    endfunction
    function [1:0] cls_of;                     // first class with CLASS_END >= last + 4
        input [6:0] l;
        reg [7:0] e;
        begin
            e = {1'b0, l} + 8'd4;
            cls_of = (e <= 8'd24) ? 2'd0 : (e <= 8'd40 && ncls >= 2'd1) ? 2'd1 :
                     (e <= 8'd64 && ncls >= 2'd2) ? 2'd2 : ncls;
        end
    endfunction
    function [1:0] mod3;
        input [6:0] v;
        mod3 = v % 7'd3;
    endfunction
    function [6:0] nref_of;                    // (PacketLength - 1) / R + 1
        input [2:0] pc;                        // PacketLength code bits {m4, m5, m6}
        input [1:0] rcode;
        reg [27:0] t;                          // per R code 1, 3, 6, 12
        begin
            case (pc)
                3'b001: t = {7'd1, 7'd1, 7'd2, 7'd4};
                3'b011: t = {7'd4, 7'd7, 7'd13, 7'd37};
                3'b111: t = {7'd5, 7'd9, 7'd17, 7'd49};
                3'b100: t = {7'd7, 7'd13, 7'd25, 7'd73};
                3'b110: t = {7'd9, 7'd17, 7'd33, 7'd97};
                default: t = {7'd11, 7'd21, 7'd41, 7'd121};
            endcase
            nref_of = t[rcode * 7 +: 7];
        end
    endfunction
    // Y' = sat16(rnd(Y, k)): 19-bit add and shift (k <= 7, rk = 2^(k-1) or 0)
    function signed [15:0] yshift;
        input signed [17:0] y;
        input [2:0] kk;
        input [6:0] rr;
        reg signed [18:0] v, s;
        begin
            v = {y[17], y} + $signed({12'd0, rr});
            s = v >>> kk;
            yshift = (s > 19'sd32767) ? 16'sd32767 : (s < -19'sd32768) ? -16'sd32768 : s[15:0];
        end
    endfunction

    integer b;

    // ------------------------------------------------------------------ engine
    always @(posedge clk) begin
        q_pop <= 1'b0;
        ms_go <= 1'b0; cd_go <= 1'b0;
        job_start <= 1'b0;
        tg1 <= T_NONE; f1 <= 1'b0;
        if (!rst_n) begin
            es <= E_IDLE; fr_ptr <= 8'd0; job_req <= 1'b0; hfree <= 1'b0;
        end else begin
            case (es)
            // ========================================================== packet start
            E_IDLE: if (q_n != 2'd0 && !q_pop) begin
                base <= q_base[0]; cur_t <= q_t[0];
                mod <= {q_msg[0][7], q_msg[0][8]};
                scode <= {q_msg[0][2], q_msg[0][3]};
                case ({q_msg[0][0], q_msg[0][1]})
                    2'b00: begin R <= 4'd1;  RSr <= 15'd32767; end
                    2'b01: begin R <= 4'd3;  RSr <= 15'd10923; end
                    2'b10: begin R <= 4'd6;  RSr <= 15'd5461; end
                    default: begin R <= 4'd12; RSr <= 15'd2731; end
                endcase
                case ({q_msg[0][4], q_msg[0][5], q_msg[0][6]})
                    3'b001: plen <= 7'd4;  3'b011: plen <= 7'd37; 3'b111: plen <= 7'd49;
                    3'b100: plen <= 7'd73; 3'b110: plen <= 7'd97; default: plen <= 7'd121;
                endcase
                nref <= nref_of({q_msg[0][4], q_msg[0][5], q_msg[0][6]}, {q_msg[0][0], q_msg[0][1]});
                es <= E_CFG;
            end
            E_CFG: begin
                case (scode)
                    2'd0: begin S <= 5'd3;  Np <= 9'd280; n0 <= 10'd570; W2 <= 37'd112736174412; ncls <= 2'd3; nmax <= 8'd128; end
                    2'd1: begin S <= 5'd6;  Np <= 9'd140; n0 <= 10'd310; W2 <= 37'd56367865518;  ncls <= 2'd3; nmax <= 8'd128; end
                    2'd2: begin S <= 5'd12; Np <= 9'd70;  n0 <= 10'd198; W2 <= 37'd28184154291;  ncls <= 2'd2; nmax <= 8'd85;  end
                    default: begin S <= 5'd24; Np <= 9'd35; n0 <= 10'd156; W2 <= 37'd14091855115; ncls <= 2'd0; nmax <= 8'd42; end
                endcase
                cum[0] <= 16'sd0;
                j1 <= 1'b0;
                es <= E_WAIT0;
            end
            E_WAIT0: if (avail > {4'd0, R}) begin cnt <= 10'd0; kcol <= {5'd0, S} - 10'd1; es <= E_KSCAN; end
            // ---------------------------------------------------------- k: max |Hp| of row 0 (raw Y)
            E_KSCAN: begin
                g_raddr <= {base[3:0], kcol};
                pr_addr <= {scode, cnt[8:0]};
                tg1 <= T_K;
                kcol <= kcol + {5'd0, S};
                if (cnt == Np - 9'd1) begin dcnt <= 5'd8; e_ret <= E_K; es <= E_DRAIN; end
                cnt <= cnt + 10'd1;
            end
            E_K: begin
                k <= mx[19] ? 5'd7 : mx[18] ? 5'd6 : mx[17] ? 5'd5 : mx[16] ? 5'd4 : mx[15] ? 5'd3 :
                     mx[14] ? 5'd2 : mx[13] ? 5'd1 : 5'd0;
                rk <= mx[19] ? 7'd64 : mx[18] ? 7'd32 : mx[17] ? 7'd16 : mx[16] ? 7'd8 : mx[15] ? 7'd4 :
                      mx[14] ? 7'd2 : mx[13] ? 7'd1 : 7'd0;
                row <= 7'd0; row_slot <= base; init <= 1'b1;
                cnt <= 10'd0; pcnt <= 9'd0; pmod <= 5'd0;
                es <= E_RSCAN;
            end
            // ---------------------------------------------------------- row scan: pilots (+ noise)
            E_RSCAN: begin
                g_raddr <= {row_slot[3:0], cnt};
                tr_addr <= {scode, cnt};
                pr_addr <= {scode, pcnt};
                tg1 <= T_SCAN; c1 <= cnt; i1 <= pcnt; f1 <= (pmod == S - 5'd1);
                if (pmod == S - 5'd1) begin pmod <= 5'd0; pcnt <= pcnt + 9'd1; end
                else pmod <= pmod + 5'd1;
                if (cnt == 10'd840) begin
                    dcnt <= 5'd10;
                    e_ret <= init ? ((row == 7'd0) ? E_RSCAN : E_JREQ) : E_DRIFTI;
                    es <= E_DRAIN;
                end
                cnt <= cnt + 10'd1;
            end
            // ---------------------------------------------------------- delay profile job (rows 0, 1)
            E_JREQ: begin
                job_req <= 1'b1;
                if (job_req && job_gnt) begin cnt <= 10'd0; es <= E_JZERO; end
            end
            E_JZERO: if (cnt == 10'd1023) begin cnt <= 10'd0; jcol <= {5'd0, S} - 10'd1; es <= E_JPIL; end
                     else cnt <= cnt + 10'd1;
            E_JPIL: begin
                p_ra <= {1'b0, j1, cnt[8:0]};
                pr_addr <= {scode, cnt[8:0]};
                tg1 <= T_JPIL; c1 <= jcol - 10'd420;            // FFT index (mod 1024)
                jcol <= jcol + S;
                if (cnt == Np - 9'd1) begin dcnt <= 5'd10; e_ret <= E_JFFT; es <= E_DRAIN; end
                cnt <= cnt + 10'd1;
            end
            E_JFFT: begin job_start <= 1'b1; es <= E_JWAIT; end
            E_JWAIT: if (fft_done) begin cnt <= 10'd4; es <= E_JREAD; end
            E_JREAD: begin
                job_oaddr <= cnt;
                tg1 <= T_JRD; c1 <= cnt;
                if (cnt == {2'd0, nmax} - 10'd1) begin dcnt <= 5'd10; e_ret <= E_JNEXT; es <= E_DRAIN; end
                cnt <= cnt + 10'd1;
            end
            E_JNEXT: begin
                job_req <= 1'b0;
                if (!j1) begin j1 <= 1'b1; es <= E_JREQ; end
                else es <= E_A44;
            end
            // ---------------------------------------------------------- class threshold, SNR mode
            E_A44: begin ms_a <= {11'd0, W2}; ms_b <= sumn; ms_go <= 1'b1; es <= E_A44W; end
            E_A44W: if (ms_done) begin
                // (PDP_T 8 * NR 2 * cg * W2 * sumN) >> 44
                a44 <= qam ? ms_p[78:39] : ms_p[79:40];
                ms_a <= pw; ms_b <= {38'd0, n0}; ms_go <= 1'b1; es <= E_M1W;
            end
            E_M1W: if (ms_done) begin
                m_lhs <= ms_p[57:0];
                // HI_RATIO 16 * NR 2 * Np * cg * sumN
                ms_a <= sumn; ms_b <= qam ? {33'd0, Np, 6'd0} : {34'd0, Np, 5'd0};
                ms_go <= 1'b1; es <= E_M2W;
            end
            E_M2W: if (ms_done) begin
                hi <= ({38'd0, m_lhs} >= ms_p);
                cnt <= 10'd4; pmax <= 38'd0; es <= E_CMAX;
            end
            E_CMAX: begin
                if (pdp[cnt - 10'd4] > pmax) pmax <= pdp[cnt - 10'd4];
                if (cnt == {2'd0, nmax} - 10'd1) begin cnt <= 10'd4; last <= 7'd0; es <= E_CL1; end
                else cnt <= cnt + 10'd1;
            end
            E_CL1: begin pv <= pdp[cnt - 10'd4]; es <= E_CL2; end
            E_CL2: begin pvl <= pv[18:0] * n0; pvh <= pv[37:19] * n0; es <= E_CL2B; end
            E_CL2B: begin pvn <= {pvh, 19'd0} + {19'd0, pvl}; es <= E_CL3; end
            E_CL3: begin
                // above: p > pmax >> 10 and p * n0 > a44 (p > (A // (n0 << 44)))
                if (pv > (pmax >> 10) && pvn > {8'd0, a44}) last <= cnt[6:0];
                if (cnt == {2'd0, nmax} - 10'd1) es <= E_PKT;
                else begin cnt <= cnt + 10'd1; es <= E_CL1; end
            end
            E_PKT: if (out_room) begin
                cls <= cls_of(last);
                cbase <= pl_coef_base(scode, cls_of(last), hi);
                pk_t <= cur_t;
                pk_a <= {6'd0, n0, 3'd0, hi, 2'd0, cls_of(last), 3'd0, k};
                pk_b <= sumn[31:0];
                half <= hi ? 3'd0 : (R == 4'd1) ? 3'd4 : (R == 4'd3) ? 3'd2 : (R == 4'd6) ? 3'd1 : 3'd0;
                nxt_row <= 7'd0; nxt_sm <= 7'd0; nxt_sym <= 7'd0;
                row_sym <= 8'd0; row10 <= 4'd0; sm10 <= 4'd0; sm3 <= 2'd0; sym_j <= 7'd0; sym_t <= 4'd0;
                init <= 1'b0;
                es <= E_MAIN0;
            end
            // ========================================================== main loop
            E_MAIN0: begin
                go_end <= (nxt_sym == plen);
                go_h <= c_h && out_room;
                go_s <= c_s;
                go_r <= c_r;
                es <= E_MAIN;
            end
            E_MAIN: begin
                es <= E_MAIN0;
                if (go_end) es <= E_DONE;
                else if (go_h) begin
                    hj <= hj_now; hsym <= nxt_sym;
                    ht <= (sym_j > nref - 7'd2) ? R : sym_t;
                    es <= E_HSET;
                end else if (go_s)
                    es <= E_SSET;
                else if (go_r) begin
                    row <= nxt_row; row_slot <= base + row_sym;
                    aslot <= row10;
                    cnt <= 10'd0; pcnt <= 9'd0; pmod <= 5'd0;
                    es <= (nxt_row >= 7'd2) ? E_RSCAN : (nxt_row == 7'd1) ? E_DRIFTI : E_FSET;
                end
                go_h <= 1'b0; go_s <= 1'b0; go_r <= 1'b0;
            end
            // ---------------------------------------------------------- drift row - 1 -> row
            E_DRIFTI: begin cnt <= 10'd0; es <= E_DRIFT; end
            E_DRIFT: begin
                p_ra <= {row[1:0], cnt[8:0]}; p_rb <= {row[1:0] - 2'd1, cnt[8:0]};
                tg1 <= T_DRIFT;
                if (cnt == Np - 9'd1) begin dcnt <= 5'd10; e_ret <= E_CORD; es <= E_DRAIN; end
                cnt <= cnt + 10'd1;
            end
            E_CORD: begin cd_re <= dr[45:0]; cd_im <= di[45:0]; cd_go <= 1'b1; es <= E_CORDW; end
            E_CORDW: if (cd_done) begin
                phi[(row - 7'd1) & 7'd15] <= cd_ang;
                cum[row & 7'd15] <= cum[(row - 7'd1) & 7'd15] + cd_ang;
                es <= E_FSET;
            end
            // ---------------------------------------------------------- FIR + rotation of row
            E_FSET: begin
                th = -cum[row & 7'd15];
                cs_addr <= th[15:6];
                tr_addr <= {scode, 10'd0};
                es <= E_FSET2;
            end
            E_FSET2: es <= E_FSET3;
            E_FSET3: begin
                rc <= cs_cos; rs <= cs_sin;
                t_start <= tr_d[8:0]; t_set <= tr_d[16:9];
                cnt <= 10'd0; pair <= 2'd0;
                es <= E_FIR;
            end
            E_FIR: begin
                if (pair == 2'd0) tr_addr <= {scode, cnt + 10'd1};
                if (pair == 2'd2) begin t_start_n <= tr_d[8:0]; t_set_n <= tr_d[16:9]; end
                p_ra <= {row[1:0], t_start + {6'd0, pair, 1'b0}};
                p_rb <= {row[1:0], t_start + {6'd0, pair, 1'b1}};
                cr_addr <= cbase + {3'd0, t_set, 2'b00} + {11'd0, pair};
                tg1 <= T_FIR; c1 <= cnt; pp1 <= pair; f1 <= (pair == 2'd3);
                if (pair == 2'd3) begin
                    pair <= 2'd0;
                    t_start <= t_start_n;
                    t_set <= t_set_n;
                    if (cnt == 10'd840) begin
                        dcnt <= 5'd14; e_ret <= E_MAIN0; es <= E_DRAIN;
                        nxt_row <= nxt_row + 7'd1; row_sym <= row_sym + {4'd0, R};
                        row10 <= (row10 == 4'd9) ? 4'd0 : row10 + 4'd1;
                    end
                    cnt <= cnt + 10'd1;
                end else pair <= pair + 2'd1;
            end
            // ---------------------------------------------------------- smoothing of row nxt_sm
            // window position b = 0 .. 8 (row nxt_sm + b - half), one per clock
            E_SSET: begin
                for (b = 0; b < 10; b = b + 1) wv[b] <= 2'd0;
                wsum <= 4'd0; sb <= 4'd0;
                sm_m <= $signed({1'b0, nxt_sm}) - $signed({5'd0, half});
                sl <= (sm10 >= {1'b0, half}) ? sm10 - {1'b0, half} : sm10 + 4'd10 - {1'b0, half};
                sslot <= sm3;
                es <= E_SSET2;
            end
            E_SSET2: begin
                if ({1'b0, sb} <= {1'b0, half, 1'b0} && !sm_m[7] && sm_m[6:0] < nref) begin
                    wv[sl] <= hw_of(sb);
                    wsum <= wsum + {2'd0, hw_of(sb)};
                end
                sb <= sb + 4'd1; sm_m <= sm_m + 8'sd1;
                sl <= (sl == 4'd9) ? 4'd0 : sl + 4'd1;
                if (sb == 4'd8) es <= E_SSET3;
            end
            E_SSET3: begin recip <= recip_of(wsum); cnt <= 10'd0; es <= E_SMOOTH; end
            E_SMOOTH: begin
                a_raddr <= cnt;
                tg1 <= T_SM; c1 <= cnt;
                if (cnt == 10'd840) begin
                    dcnt <= 5'd10; e_ret <= E_MAIN0; es <= E_DRAIN;
                    nxt_sm <= nxt_sm + 7'd1;
                    sm10 <= (sm10 == 4'd9) ? 4'd0 : sm10 + 4'd1;
                    sm3 <= (sm3 == 2'd2) ? 2'd0 : sm3 + 2'd1;
                end
                cnt <= cnt + 10'd1;
            end
            // ---------------------------------------------------------- H of symbol hsym
            // angle: cum(j) (t = 0), cum(j + 1) (t = R), else cum(j) + rnd(phi(j) t RSr, 15)
            E_HSET: begin
                hmode <= (ht == 4'd0) ? 2'd0 : (ht == R) ? 2'd1 : 2'd2;
                h_phi <= phi[hj & 7'd15]; h_c0 <= cum[hj & 7'd15]; h_c1 <= cum[(hj + 7'd1) & 7'd15];
                hj_m3 <= mod3(hj); hj1_m3 <= mod3(hj + 7'd1);
                es <= E_HSET2;
            end
            E_HSET2: begin h_p1 <= h_phi * $signed({1'b0, ht}); es <= E_HSET3; end
            E_HSET3: begin h_p2 <= h_p1 * $signed({1'b0, RSr}); es <= E_HSET4; end
            E_HSET4: begin
                th = (hmode == 2'd0) ? h_c0 : (hmode == 2'd1) ? h_c1 : h_c0 + h_p2[30:15] + {15'd0, h_p2[14]};
                cs_addr <= th[15:6];
                es <= E_HSET5;
            end
            E_HSET5: es <= E_HSET6;
            E_HSET6: begin rc <= cs_cos; rs <= cs_sin; cnt <= 10'd0; es <= E_HOUT; end
            E_HOUT: begin
                s_ra <= {hj_m3, cnt}; s_rb <= {hj1_m3, cnt};
                g_raddr <= {base[3:0] + hsym[3:0], cnt};
                tg1 <= T_H; c1 <= cnt;
                if (cnt == 10'd840) begin
                    dcnt <= 5'd14; e_ret <= E_MAIN0; es <= E_DRAIN;
                    nxt_sym <= nxt_sym + 7'd1; hfree <= 1'b1;
                    if (sym_t == R - 4'd1) begin sym_t <= 4'd0; sym_j <= sym_j + 7'd1; end
                    else sym_t <= sym_t + 4'd1;
                end
                cnt <= cnt + 10'd1;
            end
            // ---------------------------------------------------------- end of a packet, drain
            E_DONE: begin q_pop <= 1'b1; es <= E_IDLE; end
            E_DRAIN: begin
                if (dcnt == 5'd0) begin
                    es <= e_ret;
                    if (e_ret == E_RSCAN) begin
                        row <= 7'd1; row_slot <= base + {4'd0, R}; cnt <= 10'd0; pcnt <= 9'd0; pmod <= 5'd0;
                    end
                    if (hfree) begin fr_ptr <= base + {1'b0, nxt_sym}; hfree <= 1'b0; end
                end else dcnt <= dcnt - 5'd1;
            end
            default: es <= E_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------ data pipeline
    // stage 2: memory outputs valid, registered here; stage 3 on: processing from registers
    reg  [35:0] d_g;
    reg  [15:0] d_pr;
    reg  [17:0] d_tr;
    reg  [33:0] d_pa, d_pb;
    reg  [63:0] d_cr;
    reg  [10*32-1:0] d_a;
    reg  [31:0] d_sa, d_sb;
    reg  signed [17:0] d_fr, d_fi;
    wire signed [17:0] dg_r = d_g[35:18], dg_i = d_g[17:0];
    wire signed [16:0] da_r = d_pa[33:17], da_i = d_pa[16:0];
    wire signed [16:0] db_r = d_pb[33:17], db_i = d_pb[16:0];

    reg  signed [15:0] y4r, y4i, y5r, y5i, y6r, y6i;
    reg  signed [16:0] h5r, h5i;
    reg         sg4, nz4, nz5;
    reg  [19:0] km1, km2;
    reg  signed [33:0] q4a, q4b, q5a, q5b;     // squares
    reg  signed [33:0] fm [0:7];               // FIR products
    reg  signed [39:0] fsr, fsi;               // FIR partial (pair) sums
    reg  signed [39:0] far, fai;               // FIR accumulators
    reg  signed [39:0] fin_r, fin_i;           // FIR result of the tone
    reg         fv6;
    reg  [9:0]  fc6;
    reg  signed [33:0] dp1, dp2, dp3, dp4;     // drift products
    reg  signed [31:0] wp_r, wp_i;             // window products
    reg  signed [18:0] psr [0:4], psi [0:4];   // smoothing pair sums
    reg  signed [21:0] sms_r, sms_i;           // smoothing sums
    reg  signed [38:0] smp_r, smp_i;
    reg  signed [15:0] m4r, m4i, m5r, m5i;
    reg  signed [20:0] ip_r, ip_i;
    reg  signed [36:0] ipp_r, ipp_i;
    // shared rotation unit
    reg         ro_v, ro_v2, ro_dst, ro_dst2;
    reg  [9:0]  ro_c, ro_c2;
    reg  signed [15:0] ro_hr, ro_hi, ro_yr, ro_yi, ro_yr2, ro_yi2;
    reg  signed [31:0] rp1, rp2, rp3, rp4;
    integer ib;

    // a weighted A value (weight 0, 1 or 2; slots of weight 0 may never have been written)
    function signed [17:0] wsel;
        input [15:0] v;
        input [1:0]  w;
        wsel = (w == 2'd0) ? 18'sd0 : (w == 2'd1) ? {{2{v[15]}}, v} : {v[15], v, 1'b0};
    endfunction
    function [19:0] absd;                      // |v|
        input signed [18:0] v;
        absd = v[18] ? -{v[18], v} : {v[18], v};
    endfunction
    // sat16(rnd(v, n)) for the narrow cases
    function signed [15:0] sat16_33;           // v 33 bit, n = 15
        input signed [32:0] v;
        reg signed [33:0] t;
        reg signed [18:0] s;
        begin
            t = {v[32], v} + 34'sd16384;
            s = t[33:15];
            sat16_33 = (s > 19'sd32767) ? 16'sd32767 : (s < -19'sd32768) ? -16'sd32768 : s[15:0];
        end
    endfunction
    function signed [15:0] sat16_40;           // v 40 bit, n = 14
        input signed [39:0] v;
        reg signed [40:0] t;
        reg signed [26:0] s;
        begin
            t = {v[39], v} + 41'sd8192;
            s = t[40:14];
            sat16_40 = (s > 27'sd32767) ? 16'sd32767 : (s < -27'sd32768) ? -16'sd32768 : s[15:0];
        end
    endfunction
    function signed [15:0] sat16_39;           // v 39 bit, n = 15
        input signed [38:0] v;
        reg signed [39:0] t;
        reg signed [24:0] s;
        begin
            t = {v[38], v} + 40'sd16384;
            s = t[39:15];
            sat16_39 = (s > 25'sd32767) ? 16'sd32767 : (s < -25'sd32768) ? -16'sd32768 : s[15:0];
        end
    endfunction
    function signed [15:0] sat16_37;           // v 37 bit, n = 15
        input signed [36:0] v;
        reg signed [37:0] t;
        reg signed [22:0] s;
        begin
            t = {v[36], v} + 38'sd16384;
            s = t[37:15];
            sat16_37 = (s > 23'sd32767) ? 16'sd32767 : (s < -23'sd32768) ? -16'sd32768 : s[15:0];
        end
    endfunction
    function signed [17:0] job_val;            // sat12(rnd(v, 17)) << 5, 18 bit
        input signed [31:0] v;
        reg signed [32:0] t;
        reg signed [15:0] s;
        begin
            t = {v[31], v} + 33'sd65536;
            s = t[32:17];
            job_val = (s > 16'sd2047) ? {13'sd2047, 5'd0} : (s < -16'sd2048) ? {-13'sd2048, 5'd0} : {s[12:0], 5'd0};
        end
    endfunction

    always @(posedge clk) begin
        p_we <= 1'b0; a_we <= 1'b0; s_we <= 1'b0; job_we <= 1'b0;
        pk_valid <= 1'b0; h_valid <= 1'b0;
        ro_v <= 1'b0; fv6 <= 1'b0;
        tg2 <= tg1; c2 <= c1; i2 <= i1; f2 <= f1; pp2 <= pp1;
        tg3 <= tg2; c3 <= c2; i3 <= i2; f3 <= f2; pp3 <= pp2;
        tg4 <= tg3; c4 <= c3; i4 <= i3; f4 <= f3; pp4 <= pp3;
        tg5 <= tg4; c5 <= c4; i5 <= i4; f5 <= f4; pp5 <= pp4;
        tg6 <= tg5; c6 <= c5;
        // stage 2: register the memory outputs
        d_g <= g_rd; d_pr <= pr_d; d_tr <= tr_d; d_pa <= p_da; d_pb <= p_db; d_cr <= cr_d;
        d_a <= a_rd; d_sa <= s_da; d_sb <= s_db; d_fr <= fft_ore; d_fi <= fft_oim;
        if (!rst_n) begin
            tg2 <= T_NONE; tg3 <= T_NONE; tg4 <= T_NONE; tg5 <= T_NONE; tg6 <= T_NONE; ro_v2 <= 1'b0;
        end else begin
            // the job's zero fill and the packet record come from the engine directly
            if (es == E_JZERO) begin job_we <= 1'b1; job_idx <= cnt; job_re <= 18'sd0; job_im <= 18'sd0; end
            if (es == E_PKT && out_room) pk_valid <= 1'b1;
            if (es == E_CFG) begin mx <= 20'd0; pw <= 48'd0; sumn <= 48'd0; end
            if (es == E_DRIFTI) begin dr <= 48'sd0; di <= 48'sd0; end

            // ---------------- stage 3
            case (tg3)
            T_K: begin
                km1 <= absd(qam ? {dg_r[17], dg_r} - {dg_i[17], dg_i} : {dg_r[17], dg_r});
                km2 <= absd(qam ? {dg_r[17], dg_r} + {dg_i[17], dg_i} : {dg_i[17], dg_i});
            end
            T_SCAN, T_H: begin
                y4r <= yshift(dg_r, k[2:0], rk); y4i <= yshift(dg_i, k[2:0], rk);
                sg4 <= d_pr[15]; nz4 <= d_tr[17];
                m4r <= (hmode == 2'd1) ? $signed(d_sb[31:16]) : $signed(d_sa[31:16]);
                m4i <= (hmode == 2'd1) ? $signed(d_sb[15:0]) : $signed(d_sa[15:0]);
                ip_r <= $signed({1'b0, R - ht}) * $signed(d_sa[31:16]) + $signed({1'b0, ht}) * $signed(d_sb[31:16]);
                ip_i <= $signed({1'b0, R - ht}) * $signed(d_sa[15:0]) + $signed({1'b0, ht}) * $signed(d_sb[15:0]);
            end
            T_JPIL: begin
                wp_r <= da_r * $signed({1'b0, d_pr[14:0]});
                wp_i <= -da_i * $signed({1'b0, d_pr[14:0]});
            end
            T_JRD: begin
                q4a <= d_fr * d_fr;
                q4b <= d_fi * d_fi;
            end
            T_DRIFT: begin
                dp1 <= da_r * db_r; dp2 <= da_i * db_i;     // Re: Hp1 r Hp0 r + Hp1 i Hp0 i
                dp3 <= da_i * db_r; dp4 <= da_r * db_i;     // Im: Hp1 i Hp0 r - Hp1 r Hp0 i
            end
            T_FIR: begin
                fm[0] <= $signed(d_cr[15:0])  * da_r; fm[1] <= $signed(d_cr[31:16]) * da_i;
                fm[2] <= $signed(d_cr[15:0])  * da_i; fm[3] <= $signed(d_cr[31:16]) * da_r;
                fm[4] <= $signed(d_cr[47:32]) * db_r; fm[5] <= $signed(d_cr[63:48]) * db_i;
                fm[6] <= $signed(d_cr[47:32]) * db_i; fm[7] <= $signed(d_cr[63:48]) * db_r;
            end
            T_SM: begin
                for (ib = 0; ib < 5; ib = ib + 1) begin
                    psr[ib] <= wsel(d_a[(2*ib)*32 + 16 +: 16], wv[2*ib]) + wsel(d_a[(2*ib+1)*32 + 16 +: 16], wv[2*ib+1]);
                    psi[ib] <= wsel(d_a[(2*ib)*32 +: 16], wv[2*ib]) + wsel(d_a[(2*ib+1)*32 +: 16], wv[2*ib+1]);
                end
            end
            default: ;
            endcase

            // ---------------- stage 4
            case (tg4)
            T_K: begin
                if (km1 >= km2 && km1 > mx) mx <= km1;
                else if (km2 > km1 && km2 > mx) mx <= km2;
            end
            T_SCAN: begin
                y5r <= y4r; y5i <= y4i; nz5 <= nz4;
                if (!qam) begin
                    h5r <= sg4 ? -{y4r[15], y4r} : {y4r[15], y4r};
                    h5i <= sg4 ? -{y4i[15], y4i} : {y4i[15], y4i};
                end else begin
                    h5r <= sg4 ? ({y4r[15], y4r} - {y4i[15], y4i}) : ({y4i[15], y4i} - {y4r[15], y4r});
                    h5i <= sg4 ? ({y4r[15], y4r} + {y4i[15], y4i}) : -({y4r[15], y4r} + {y4i[15], y4i});
                end
            end
            T_JPIL: begin
                job_we <= 1'b1; job_idx <= c4;
                job_re <= job_val(wp_r);
                job_im <= job_val(wp_i);
            end
            T_JRD: begin
                pdp[c4 - 10'd4] <= (j1 ? pdp[c4 - 10'd4] : 38'd0) + {4'd0, q4a} + {4'd0, q4b};
            end
            T_DRIFT: begin
                dr <= dr + dp1 + dp2;
                di <= di + dp3 - dp4;
            end
            T_FIR: begin
                fsr <= fm[0] - fm[1] + fm[4] - fm[5];
                fsi <= fm[2] + fm[3] + fm[6] + fm[7];
            end
            T_SM: begin
                sms_r <= psr[0] + psr[1] + psr[2] + psr[3] + psr[4];
                sms_i <= psi[0] + psi[1] + psi[2] + psi[3] + psi[4];
            end
            T_H: begin
                y5r <= y4r; y5i <= y4i; m5r <= m4r; m5i <= m4i;
                ipp_r <= ip_r * $signed({1'b0, RSr});
                ipp_i <= ip_i * $signed({1'b0, RSr});
            end
            default: ;
            endcase

            // ---------------- stage 5
            case (tg5)
            T_SCAN: begin
                if (f5) begin
                    p_we <= 1'b1; p_waddr <= {row[1:0], i5}; p_wd <= {h5r, h5i};
                end
                q5a <= (init && f5) ? h5r * h5r + h5i * h5i : 34'sd0;
                q5b <= (init && nz5) ? y5r * y5r + y5i * y5i : 34'sd0;
            end
            T_FIR: begin
                if (pp5 == 2'd0) begin far <= fsr; fai <= fsi; end
                else begin far <= far + fsr; fai <= fai + fsi; end
                if (f5) begin
                    fv6 <= 1'b1; fc6 <= c5;
                    fin_r <= far + fsr; fin_i <= fai + fsi;
                end
            end
            T_SM: begin
                smp_r <= sms_r * $signed({1'b0, recip});
                smp_i <= sms_i * $signed({1'b0, recip});
            end
            T_H: begin
                ro_v <= 1'b1; ro_dst <= 1'b1; ro_c <= c5; ro_yr <= y5r; ro_yi <= y5i;
                ro_hr <= (hmode == 2'd2) ? sat16_37(ipp_r) : m5r;
                ro_hi <= (hmode == 2'd2) ? sat16_37(ipp_i) : m5i;
            end
            default: ;
            endcase

            // ---------------- stage 6
            case (tg6)
            T_SCAN: begin
                pw <= pw + q5a;
                sumn <= sumn + q5b;
            end
            T_SM: begin
                s_we <= 1'b1; s_waddr <= {sslot, c6};
                s_wd <= {sat16_39(smp_r), sat16_39(smp_i)};
            end
            default: ;
            endcase
            if (fv6) begin
                ro_v <= 1'b1; ro_dst <= 1'b0; ro_c <= fc6;
                ro_hr <= sat16_40(fin_r);
                ro_hi <= sat16_40(fin_i);
            end

            // ---------------- rotation unit: rnd(H (c + j s) >> 15), sat16
            ro_v2 <= ro_v;
            if (ro_v) begin
                rp1 <= ro_hr * rc; rp2 <= ro_hi * rs; rp3 <= ro_hr * rs; rp4 <= ro_hi * rc;
                ro_c2 <= ro_c; ro_dst2 <= ro_dst; ro_yr2 <= ro_yr; ro_yi2 <= ro_yi;
            end
            if (ro_v2) begin
                if (!ro_dst2) begin
                    a_we <= 1'b1; a_wslot <= aslot; a_waddr <= ro_c2;
                    a_wd <= {sat16_33({rp1[31], rp1} - {rp2[31], rp2}), sat16_33({rp3[31], rp3} + {rp4[31], rp4})};
                end else begin
                    h_valid <= 1'b1; h_t <= cur_t; h_sym <= hsym; h_col <= ro_c2;
                    h_re <= sat16_33({rp1[31], rp1} - {rp2[31], rp2});
                    h_im <= sat16_33({rp3[31], rp3} + {rp4[31], rp4});
                    y_re <= ro_yr2; y_im <= ro_yi2;
                end
            end
        end
    end
endmodule

`default_nettype wire
