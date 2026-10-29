// pd_cf.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Control field after a good Preamble B timing record (Docs/Specification_ControlField.md),
// bit-true to cf_reference.control_records: for every timing record with status == PEAK_OK only
// (not LATE, not BUSY), in record order, cf_rx decodes the 4560 samples y(n_end + 1 ...) of the
// DC blocker output with the event's cfo_inc and gives one control record.
//
// Samples: every DC blocker output goes into a circular buffer of 2^DB samples (block RAM),
// indexed by sample number, as in pd_timing. The control field is read from there once its
// timing record is out, so it can start long after the samples arrived. Good timing records
// queue (4 deep; one control field at a time). `stall` holds the input back while a sample still
// needed (the next one of the running control field, the first one of a queued one) would be
// overwritten. Samples of a timing search not finished yet are safe without that: pd_timing keeps
// them within 2^12 - 64 samples of the input, and this buffer is twice as long.
// At 20 Msps (5 clocks per sample) nothing stalls: a control field starts at most ~5000 samples
// behind the input and falls ~1300 further behind while it runs (the FFTs stop its input).
`timescale 1ns / 1ps
`default_nettype none

module pd_cf #(
    parameter integer DB     = 13,       // sample buffer 2^DB samples
    parameter integer MARGIN = 64
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,          // input_stage 1 or 2
    // DC blocker output
    input  wire               y_valid,
    input  wire signed [11:0] y_i,
    input  wire signed [11:0] y_q,
    // timing records
    input  wire               tm_valid,
    input  wire [31:0]        tm_t,
    input  wire [31:0]        tm_n_end,
    input  wire [2:0]         tm_status,
    input  wire [23:0]        tm_inc,
    // back-pressure towards the sample input
    output reg                stall,
    // control record
    output wire               cf_valid,
    output wire [31:0]        cf_t,
    output wire [31:0]        cf_n_end,
    output wire               cf_crc_ok,
    output wire [53:0]        cf_msg,
    output wire [5:0]         cf_shift,
    output wire signed [15:0] cf_metric,
    output wire               busy,        // work in progress that does not wait for samples
    output reg                err          // start queue overflow (sticky; cannot happen)
);
    localparam integer LEN   = 4560;
    localparam integer DEPTH = 1 << DB;
    localparam [31:0]  LIM   = DEPTH - MARGIN;

    // ------------------------------------------------------------------ sample buffer
    reg  [31:0]   wp;
    wire          y_in = en && y_valid;
    reg  [31:0]   rp;                       // next sample to fetch
    wire [23:0]   b_rd;
    (* keep_hierarchy = "yes" *)
    pd_sdp_ram #(.W(24), .A(DB)) u_buf (.clk(clk), .we(y_in), .waddr(wp[DB-1:0]),
        .wdata({y_q, y_i}), .raddr(rp[DB-1:0]), .rdata(b_rd));
    always @(posedge clk) begin
        if (!rst_n) wp <= 32'd0;
        else if (y_in) wp <= wp + 32'd1;
    end

    // ------------------------------------------------------------------ start queue: t, n_end, cfo_inc
    reg  [31:0] q_t [0:3];
    reg  [31:0] q_e [0:3];
    reg  [31:0] q_s [0:3];                  // n_end + 1, the first sample (for the stall check)
    reg  [23:0] q_inc [0:3];
    reg  [1:0]  q_wr, q_rd;
    reg  [2:0]  q_n;
    wire        q_push = en && tm_valid && (tm_status == 3'b001);
    reg         q_pop;
    always @(posedge clk) begin
        if (!rst_n) begin
            q_wr <= 2'd0; q_rd <= 2'd0; q_n <= 3'd0; err <= 1'b0;
        end else begin
            if (q_push) begin
                q_t[q_wr] <= tm_t; q_e[q_wr] <= tm_n_end; q_inc[q_wr] <= tm_inc; q_s[q_wr] <= tm_n_end + 32'd1;
                q_wr <= q_wr + 2'd1;
                if (q_n == 3'd4 && !q_pop) err <= 1'b1;
            end
            if (q_pop) q_rd <= q_rd + 2'd1;
            q_n <= q_n + {2'd0, q_push} - {2'd0, q_pop};
        end
    end

    // ------------------------------------------------------------------ reader
    reg         active;
    reg  [12:0] fetched;                    // samples fetched for the running control field
    reg  [1:0]  pv;                         // fetch -> buffer data (one clock) -> FIFO
    reg  [2:0]  infl;
    reg  [23:0] ff [0:3];
    reg  [1:0]  ff_wr, ff_rd;
    reg  [2:0]  ff_n;
    wire        avail = $signed(rp - wp) < 0;
    wire        fetch = active && avail && (fetched != LEN) && ({1'b0, ff_n} + {1'b0, infl} < 4'd4);
    wire        cf_ready;
    wire        ff_pop = (ff_n != 3'd0) && cf_ready;
    reg         cf_start;
    wire        rec_v;
    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 1'b0; q_pop <= 1'b0; cf_start <= 1'b0; pv <= 2'd0; infl <= 3'd0;
            ff_wr <= 2'd0; ff_rd <= 2'd0; ff_n <= 3'd0; rp <= 32'd0; fetched <= 13'd0;
        end else begin
            q_pop <= 1'b0; cf_start <= 1'b0;
            if (!active && q_n != 3'd0 && !q_pop) begin
                active <= 1'b1; q_pop <= 1'b1; cf_start <= 1'b1;
                rp <= q_s[q_rd]; fetched <= 13'd0;
            end else if (active && rec_v)
                active <= 1'b0;
            if (fetch) begin rp <= rp + 32'd1; fetched <= fetched + 13'd1; end
            pv   <= {pv[0], fetch};
            infl <= infl + {2'd0, fetch} - {2'd0, pv[0]};
            if (pv[0]) begin ff[ff_wr] <= b_rd; ff_wr <= ff_wr + 2'd1; end
            if (ff_pop) ff_rd <= ff_rd + 2'd1;
            ff_n <= ff_n + {2'd0, pv[0]} - {2'd0, ff_pop};
        end
    end
    wire [23:0] ff_head = ff[ff_rd];

    // stall: a sample still needed would be overwritten
    wire [31:0] age_rp = wp - rp;
    wire [31:0] age_q  = wp - q_s[q_rd];
    always @(posedge clk) begin
        if (!rst_n || !en) stall <= 1'b0;
        else stall <= (active && (fetched != LEN) && !age_rp[31] && (age_rp >= LIM)) ||
                      ((q_n != 3'd0) && !age_q[31] && (age_q >= LIM));
    end

    // ------------------------------------------------------------------ receiver
    wire cf_busy;
    cf_rx u_rx (.clk(clk), .rst_n(rst_n), .start(cf_start), .cfo_inc(q_inc[q_rd]), .t_trig(q_t[q_rd]),
                .n_end(q_e[q_rd]), .busy(cf_busy),
                .s_valid(ff_n != 3'd0), .s_ready(cf_ready), .s_i(ff_head[11:0]), .s_q(ff_head[23:12]),
                .rec_valid(rec_v), .rec_crc_ok(cf_crc_ok), .rec_msg(cf_msg), .rec_shift(cf_shift),
                .rec_metric(cf_metric), .rec_t_trig(cf_t), .rec_n_end(cf_n_end),
                .dbg_grid_we(), .dbg_grid_addr(), .dbg_grid_data(), .dbg_phi(), .dbg_hf_we(), .dbg_hf_k(),
                .dbg_hf(), .dbg_v_we(), .dbg_v_i(), .dbg_v(), .dbg_bits());
    assign cf_valid = rec_v;

    // busy: something to do that does not wait for new samples
    wire starving = active && cf_ready && (ff_n == 3'd0) && (infl == 3'd0) && !avail && (fetched != LEN);
    assign busy = (active && !starving) || (!active && q_n != 3'd0) || q_pop || cf_start;
endmodule

`default_nettype wire
