// cf_viterbi.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Soft Viterbi decoder for the control field, bit-true to cf_reference.viterbi:
// K = 7, generators 133 / 171 (octal), 70 trellis steps, start and end in state 0, maximum
// correlation. State = the last 6 input bits, newest in the MSB; next state
// ns = {b, p[5:1]} for input b from state p, so the two predecessors of ns are
// p0 = {ns[4:0], 0} and p1 = {ns[4:0], 1}. Code bits of a transition (b, p):
// oa = parity({b, p} & 133o), ob = parity({b, p} & 171o).
//   branch metric = (oa ? qa : -qa) + (ob ? qb : -qb), q = 5-bit soft value (positive = 1)
//   m'(ns) = max(m(p0) + bm0, m(p1) + bm1); decision 1 only if the p1 path is strictly larger
//   traceback from state 0: bit t = state[5], state = {state[4:0], decision_t(state)}
// One state per clock (two clocks of pipeline, 2 idle clocks per step): 70 x 66 + 140 clocks.
// Unreachable states start at -4096; reachable metrics stay within +-2240, so 16 bits never
// overflow and an unreachable path never wins.
// q_we/q_addr/q_data load the 140 soft values (code bit order) before start.
`timescale 1ns / 1ps
`default_nettype none

module cf_viterbi (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               q_we,
    input  wire [7:0]         q_addr,
    input  wire signed [4:0]  q_data,
    input  wire               start,
    output wire               busy,
    output reg                done,
    output reg  [69:0]        bits,
    output reg  signed [15:0] metric
);
    localparam signed [15:0] NEG = -16'sd4096;

    reg signed [4:0] qmem [0:139];
    always @(posedge clk) if (q_we) qmem[q_addr] <= q_data;

    // metrics: bank x {even, odd} x 32, distributed RAM
    reg signed [15:0] m_even [0:63];            // index {bank, state[5:1]} for even states
    reg signed [15:0] m_odd  [0:63];
    reg [63:0] dec [0:69];

    localparam [2:0] S_IDLE = 3'd0, S_Q = 3'd1, S_ACS = 3'd2, S_TB = 3'd3, S_MET = 3'd4;
    reg  [2:0] state;
    reg  [6:0] t;                               // trellis step
    reg  [6:0] ns;                              // 0..63 issue, 64..65 drain
    reg        bank;                            // bank read in this step
    reg signed [4:0] qa, qb;
    reg  [63:0] dword;

    function par;
        input [6:0] v;
        par = ^v;
    endfunction
    function signed [6:0] bm;
        input [6:0] bp;                         // {b, p}
        input signed [4:0] a;
        input signed [4:0] b;
        reg signed [6:0] ea, eb;
        begin
            ea = {{2{a[4]}}, a};
            eb = {{2{b[4]}}, b};
            bm = (par(bp & 7'o133) ? ea : -ea) + (par(bp & 7'o171) ? eb : -eb);
        end
    endfunction

    // stage 1: read predecessors, add branch metrics
    wire [5:0]  s_ns = ns[5:0];
    wire [4:0]  pidx = s_ns[4:0];               // p0 = {pidx, 0}, p1 = {pidx, 1}
    wire        b_in = s_ns[5];
    wire signed [15:0] mp0 = (t == 7'd0) ? ((pidx == 5'd0) ? 16'sd0 : NEG) : m_even[{bank, pidx}];
    wire signed [15:0] mp1 = (t == 7'd0) ? NEG : m_odd[{bank, pidx}];
    wire signed [6:0]  bm0 = bm({b_in, pidx, 1'b0}, qa, qb);
    wire signed [6:0]  bm1 = bm({b_in, pidx, 1'b1}, qa, qb);
    reg  signed [15:0] c0, c1;
    reg  [5:0] ns1;
    reg        v1;
    always @(posedge clk) begin
        c0  <= mp0 + {{9{bm0[6]}}, bm0};
        c1  <= mp1 + {{9{bm1[6]}}, bm1};
        ns1 <= s_ns;
        v1  <= (state == S_ACS) && (ns < 7'd64);
    end
    // stage 2: compare, select, write the other bank
    wire x = (c1 > c0);
    wire signed [15:0] mnew = x ? c1 : c0;
    always @(posedge clk) begin
        if (v1) begin
            if (ns1[0]) m_odd[{~bank, ns1[5:1]}] <= mnew;
            else        m_even[{~bank, ns1[5:1]}] <= mnew;
            dword[ns1] <= x;
        end
    end

    assign busy = (state != S_IDLE);
    reg [5:0] tb_state;
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: if (start) begin t <= 7'd0; bank <= 1'b0; state <= S_Q; end
                S_Q: begin                      // soft values of step t
                    qa <= qmem[{t, 1'b0}];
                    qb <= qmem[{t, 1'b1}];
                    ns <= 7'd0;
                    state <= S_ACS;
                end
                S_ACS: begin
                    if (ns == 7'd65) begin      // last write done at ns = 65
                        dec[t] <= dword;
                        bank <= ~bank;
                        if (t == 7'd69) begin
                            tb_state <= 6'd0; state <= S_TB;
                        end else begin
                            t <= t + 7'd1; state <= S_Q;
                        end
                    end else
                        ns <= ns + 7'd1;
                end
                S_TB: begin                     // t = 69 .. 0
                    bits[t] <= tb_state[5];
                    tb_state <= {tb_state[4:0], dec[t][tb_state]};
                    if (t == 7'd0) state <= S_MET;
                    else t <= t - 7'd1;
                end
                S_MET: begin
                    metric <= m_even[{bank, 5'd0}];  // state 0 of the last step's bank
                    done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
