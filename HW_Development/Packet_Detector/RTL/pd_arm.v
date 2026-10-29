// pd_arm.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Rx state machine re-arm (Docs/Specification_PayloadInput.md section 4), bit-true to
// pd_reference.rearm_point. With rearm = 1 the detection core stays disarmed after a trigger
// until this block knows the packet's outcome and tells it the re-arm sample R (arm_set,
// arm_val); N = n_cfo:
//   tone check failed                        R = t + 2N + 256
//   input stage 0, tone check passed         R = t + max(arm_holdoff, N, 1, 2N + 256)
//   timing record not good                   R = t + 2N + 6656
//   control field CRC failed, or a field code with no meaning (PacketLength 000 / 010,
//   CBS_A_Flag 11, RM_A_Flag > 1000)          R = n_end + 2N + 12753
//   control field good                       R = max(n_end + 4561 + 1140 L, n_end + 2N + 12753)
// While a decision is pending, `stall` holds the input back once the detector input (LPF output)
// has reached sample R_min - 64, R_min = the smallest R still possible. Every decision only needs samples
// before that, so it always comes; at 20 Msps it comes before sample R_min - 64 arrives, so
// nothing stalls. With rearm = 0 this block does nothing (the core uses its fixed holdoff).
// One packet at a time: the core cannot trigger again before arm_set.
`timescale 1ns / 1ps
`default_nettype none

module pd_arm (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rearm,
    input  wire               fe_on,       // input stage 1 or 2 (timing and control field run)
    input  wire [15:0]        n_cfo,
    input  wire [31:0]        arm_holdoff,
    input  wire               x_valid,     // detector input (LPF output; sample index = t_trig index)
    input  wire               trig,
    input  wire [31:0]        t_trig,
    input  wire               ev_valid,
    input  wire [31:0]        ev_t,
    input  wire               ev_fail,
    input  wire               tm_valid,
    input  wire [31:0]        tm_t,
    input  wire [31:0]        tm_n_end,
    input  wire [2:0]         tm_status,
    input  wire               cf_valid,
    input  wire [31:0]        cf_t,
    input  wire               cf_crc_ok,
    input  wire [53:0]        cf_msg,
    output reg                arm_set,
    output reg  [31:0]        arm_val,
    output reg                stall,
    output wire               pending
);
    localparam [1:0] S_IDLE = 2'd0, S_TONE = 2'd1, S_TIMING = 2'd2, S_CF = 2'd3;
    reg  [1:0]  st;
    reg  [31:0] t, ne, r_min;
    reg  [31:0] nd;                            // detector input samples so far
    assign pending = (st != S_IDLE);

    // offsets from the parameters (constant while samples flow), registered so that a re-arm
    // point is one addition
    wire [31:0] n2      = {15'd0, n_cfo, 1'b0};
    wire [31:0] n_cfo32 = {16'd0, n_cfo};
    wire [31:0] hold_a  = (arm_holdoff > n_cfo32) ? arm_holdoff : n_cfo32;
    wire [31:0] holdoff = (hold_a == 32'd0) ? 32'd1 : hold_a;
    reg  [31:0] o_tone, o_s0, o_tm, o_cf;   // 2N + 256, max(H, 2N + 256), 2N + 6656, 2N + 12753
    always @(posedge clk) begin
        o_tone <= n2 + 32'd256;
        o_s0   <= (holdoff > n2 + 32'd256) ? holdoff : n2 + 32'd256;
        o_tm   <= n2 + 32'd6656;
        o_cf   <= n2 + 32'd12753;
    end

    // control-field codes (message bit 0 first; each field MSB first)
    wire [2:0] c_pl  = {cf_msg[4], cf_msg[5], cf_msg[6]};
    wire [1:0] c_cbs = {cf_msg[10], cf_msg[11]};
    wire [3:0] c_rm  = {cf_msg[14], cf_msg[15], cf_msg[16], cf_msg[17]};
    reg  [17:0] pl_len;                        // 1140 x PacketLength
    reg         pl_ok;
    always @* begin
        pl_ok = 1'b1;
        case (c_pl)
            3'b001: pl_len = 18'd4560;         //   4
            3'b011: pl_len = 18'd42180;        //  37
            3'b111: pl_len = 18'd55860;        //  49
            3'b100: pl_len = 18'd83220;        //  73
            3'b110: pl_len = 18'd110580;       //  97
            3'b101: pl_len = 18'd137940;       // 121
            default: begin pl_len = 18'd0; pl_ok = 1'b0; end
        endcase
    end
    wire        f_ok  = pl_ok && (c_cbs != 2'b11) && (c_rm <= 4'b1000);
    wire [31:0] r_cf  = ne + o_cf;
    wire [31:0] r_end = ne + 32'd4561 + {14'd0, pl_len};

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; arm_set <= 1'b0; stall <= 1'b0; nd <= 32'd0; r_min <= 32'd0;
        end else begin
            arm_set <= 1'b0;
            if (x_valid) nd <= nd + 32'd1;
            stall <= rearm && (st != S_IDLE) && ($signed(nd + 32'd64 - r_min) >= 0);
            case (st)
                S_IDLE:
                    if (rearm && trig) begin
                        st <= S_TONE; t <= t_trig; r_min <= t_trig + o_tone;
                    end
                S_TONE:
                    if (ev_valid && ev_t == t) begin
                        if (ev_fail) begin
                            arm_set <= 1'b1; arm_val <= t + o_tone; st <= S_IDLE;
                        end else if (!fe_on) begin
                            arm_set <= 1'b1; arm_val <= t + o_s0; st <= S_IDLE;
                        end else begin
                            st <= S_TIMING; r_min <= t + o_tm;
                        end
                    end
                S_TIMING:
                    if (tm_valid && tm_t == t) begin
                        if (tm_status == 3'b001) begin
                            st <= S_CF; ne <= tm_n_end; r_min <= tm_n_end + o_cf;
                        end else begin
                            arm_set <= 1'b1; arm_val <= t + o_tm; st <= S_IDLE;
                        end
                    end
                S_CF:
                    if (cf_valid && cf_t == t) begin
                        arm_set <= 1'b1;
                        arm_val <= (cf_crc_ok && f_ok && ($signed(r_end - r_cf) > 0)) ? r_end : r_cf;
                        st <= S_IDLE;
                    end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
