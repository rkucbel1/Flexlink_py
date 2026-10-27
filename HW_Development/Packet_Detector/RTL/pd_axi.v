// pd_axi.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// AXI wrapper of the packet detector for the Zedboard block design (replaces the phase-1
// loopback FIFO "pl_core"):
//   S_AXIS  samples from the AXI DMA MM2S (one complex sample per beat, I [15:0], Q [31:16])
//   M_AXIS  debug records to the AXI DMA S2MM, one per detector sample (20 Msps; with input
//           stage 2 one per input pair): {dbg_bits, dbg_metric}; TLAST on the record of each
//           input beat that had TLAST, so every S2MM transfer ends with its MM2S transfer
//           (stage 2: transfers must hold whole pairs, i.e. an even number of samples)
//   S_AXI   AXI4-Lite registers: control, status, parameters, event FIFO
//
// Register map (byte offsets; RW = read/write, RO = read only):
//   0x00 ID          RO  0x50440200 ("PD", version 2.0; 1.0 had no input stage 2)
//   0x04 CTRL        RW  bit0 RESET (1 = detector held in reset; 1 after power-up)
//                        bit1 DEBUG (send debug records on M_AXIS; change only when not BUSY)
//                        bit2 LOOPBACK (S_AXIS -> M_AXIS unchanged, detector not fed)
//   0x08 STATUS      RO  bit0 BUSY (samples, CFO or tone results, events or debug records in
//                        flight), bit1 OPEN (a window still waits for samples), bits 7:4 number
//                        of open windows, bit8 EV_OVF (event lost; sticky until RESET),
//                        bit9 in reset, bit10 loopback
//   0x0C SAMPLES     RO  input samples taken since RESET (stage 2: 40 Msps samples, twice the
//                        detector's sample_cnt)
//   0x10 OPEN_T      RO  t_trig of the first open window (valid when OPEN)
//   0x14 EV_LEVEL    RO  words in the event FIFO (8 per event)
//   0x18 EV_DATA     RO  next event word (reading removes it)
//   0x1C DBG_SENT    RO  debug records sent since RESET
//   0x20 ETA_HI_SQ   RW  \
//   0x24 ETA_LO_SQ   RW   |
//   0x28 QUAL_LEN    RW   |
//   0x2C N_CFO       RW   |  detector parameters (pd_protocol.Params); write them while RESET
//   0x30 BLANK_LEN   RW   |  is 1. Power-up values are the Params defaults.
//   0x34 ARM_HOLDOFF RW   |
//   0x38 INPUT_STAGE RW   |  0 LPF output, 1 20 Msps, 2 40 Msps (half-band in the PL)
//   0x3C DC_SHIFT    RW   |
//   0x40 TONE_LIN    RW  /   threshold as a linear ratio, Q.16 (firmware: tone_thresh_lin)
//
// Event (8 words, in this order): t_trig, cfo_inc, {ang32, ang16}, {ang256, ang128},
//   num[31:0], {fail, 23'b0, num[39:32]}, den[31:0], {24'b0, den[39:32]}
//
// Flow control: the sample input is held back (S_AXIS tready = 0) while the tone check is
// catching up, while the event FIFO is nearly full (the firmware reads it during the DMA), and
// with DEBUG while the debug FIFO and the records in flight would exceed its 64 entries. Nothing
// is ever dropped, and none of this changes the results.
`timescale 1ns / 1ps
`default_nettype none

module pd_axi (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI:S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    // AXI4-Lite slave
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12" *)
    input  wire [11:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
    input  wire [2:0]  s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire        s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire        s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output wire [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output reg         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [11:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
    input  wire [2:0]  s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire        s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output reg  [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire        s_axi_rready,

    // samples in
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "TDATA_NUM_BYTES 4, HAS_TLAST 1, HAS_TKEEP 1" *)
    input  wire [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *)
    input  wire [3:0]  s_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,

    // debug records (or samples, in loopback) out
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "TDATA_NUM_BYTES 4, HAS_TLAST 1, HAS_TKEEP 1" *)
    output wire [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *)
    output wire [3:0]  m_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire        m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready
);
    localparam [31:0] ID = 32'h5044_0200;     // "PD", version 2.0 (2.0: input stage 2)
    localparam integer EV_DEPTH = 256;           // words (32 events)
    localparam integer EV_LIMIT = EV_DEPTH - 96; // hold the input above this (>= 11 events room)
    localparam integer DBG_BITS = 6;             // debug FIFO 64 records
    localparam integer DBG_LIMIT = (1 << DBG_BITS) - 4;

    // ------------------------------------------------------------------ registers
    reg        ctrl_reset, ctrl_debug, ctrl_loop;
    reg [15:0] r_eta_hi, r_eta_lo, r_qual, r_ncfo;
    reg [31:0] r_blank, r_hold;
    reg [1:0]  r_stage;
    reg [3:0]  r_shift;
    reg [23:0] r_lin;

    // detector reset: registered, from the bus reset and CTRL.RESET
    reg det_rst_n;
    always @(posedge aclk) det_rst_n <= aresetn && !ctrl_reset;

    // ------------------------------------------------------------------ detector
    wire        det_tready, det_dbg_valid, det_ev_valid, det_ev_fail, det_busy;
    wire [15:0] det_dbg_metric, det_dbg_bits;
    wire [31:0] det_ev_t, det_ev_inc;
    wire [15:0] det_ev_a16, det_ev_a32, det_ev_a128, det_ev_a256;
    wire [39:0] det_ev_num, det_ev_den;
    wire [3:0]  det_open_n;
    wire [31:0] det_open_t;
    reg         gate;                             // input allowed (registered)
    wire        det_tvalid = s_axis_tvalid && gate;
    wire        accept     = det_tvalid && det_tready;

    pd_detect_top u_det (
        .clk(aclk), .rst_n(det_rst_n),
        .input_stage(r_stage), .dc_shift(r_shift),
        .eta_hi_sq(r_eta_hi), .eta_lo_sq(r_eta_lo), .qual_len(r_qual),
        .blank_len(r_blank), .arm_holdoff(r_hold), .n_cfo(r_ncfo), .tone_lin(r_lin),
        .s_iq_tvalid(det_tvalid), .s_iq_tdata(s_axis_tdata), .s_iq_tready(det_tready),
        .lpf_valid(), .lpf_i(), .lpf_q(), .trig(), .t_trig(),
        .dbg_valid(det_dbg_valid), .dbg_metric(det_dbg_metric), .dbg_bits(det_dbg_bits),
        .cfo_valid(), .cfo_t(), .cfo_inc(), .ang16(), .ang32(), .ang128(), .ang256(),
        .a16_re(), .a16_im(), .a32_re(), .a32_im(), .a128_re(), .a128_im(), .a256_re(), .a256_im(),
        .ev_valid(det_ev_valid), .ev_t(det_ev_t), .ev_cfo_inc(det_ev_inc),
        .ev_ang16(det_ev_a16), .ev_ang32(det_ev_a32), .ev_ang128(det_ev_a128), .ev_ang256(det_ev_a256),
        .ev_tone_num(det_ev_num), .ev_tone_den(det_ev_den), .ev_tone_fail(det_ev_fail),
        .busy(det_busy), .open_n(det_open_n), .open_t(det_open_t));

    wire       stage2 = (r_stage == 2'd2);
    reg [31:0] samp_cnt;                          // accepted samples since reset
    // a debug record is due with this beat (stage 2: with the second sample of each pair)
    wire       rec_due = accept && (!stage2 || samp_cnt[0]);
    wire [31:0] rec_of_beat = stage2 ? {1'b0, samp_cnt[31:1]} : samp_cnt;
    always @(posedge aclk) begin
        if (!det_rst_n)  samp_cnt <= 32'd0;
        else if (accept) samp_cnt <= samp_cnt + 32'd1;
    end

    // ------------------------------------------------------------------ event FIFO
    reg  [31:0] ev_mem [0:EV_DEPTH-1];
    reg  [7:0]  ev_wr, ev_rd;
    reg  [8:0]  ev_cnt;
    reg  [255:0] ser;                             // event being written, word 0 in bits 31:0
    reg  [3:0]  ser_n;                            // words left to write
    reg         ev_ovf;
    wire        ev_push = (ser_n != 4'd0) && (ev_cnt != EV_DEPTH);
    reg         ev_pop;                           // from the register read below
    always @(posedge aclk) begin
        if (ev_push) ev_mem[ev_wr] <= ser[31:0];
    end
    always @(posedge aclk) begin
        if (!det_rst_n) begin
            ev_wr <= 8'd0; ev_rd <= 8'd0; ev_cnt <= 9'd0; ser_n <= 4'd0; ev_ovf <= 1'b0;
        end else begin
            if (det_ev_valid) begin
                ser <= {24'd0, det_ev_den[39:32], det_ev_den[31:0],
                        det_ev_fail, 23'd0, det_ev_num[39:32], det_ev_num[31:0],
                        det_ev_a256, det_ev_a128, det_ev_a32, det_ev_a16, det_ev_inc, det_ev_t};
                ser_n <= 4'd8;
                if (ser_n != 4'd0) ev_ovf <= 1'b1;
            end else if (ser_n != 4'd0) begin
                if (ev_push) begin
                    ser   <= {32'd0, ser[255:32]};
                    ser_n <= ser_n - 4'd1;
                end else
                    ev_ovf <= 1'b1;               // FIFO full: cannot happen with the input gate
            end
            if (ev_push) ev_wr <= ev_wr + 8'd1;
            if (ev_pop)  ev_rd <= ev_rd + 8'd1;
            ev_cnt <= ev_cnt + {8'd0, ev_push} - {8'd0, ev_pop};
        end
    end

    // ------------------------------------------------------------------ debug FIFO
    // res = records reserved: accepted samples whose record has not left the FIFO (or been
    // dropped with DEBUG = 0). The input is held while res is near the FIFO size.
    localparam integer DD = 1 << DBG_BITS;
    reg  [32:0]         dbg_mem [0:DD-1];
    reg  [DBG_BITS-1:0] dbg_wr, dbg_rd;
    reg  [DBG_BITS:0]   dbg_cnt;
    reg  [7:0]          res;
    reg  [31:0]         rec_n, dbg_sent;
    // input beats with TLAST (sample index), to mark their records
    reg  [31:0]         tl [0:3];
    reg  [1:0]          tl_wr, tl_rd;
    reg  [2:0]          tl_n;
    wire                tl_hit   = (tl_n != 3'd0) && (rec_n == tl[tl_rd]);
    wire                dbg_push = det_dbg_valid && ctrl_debug;
    wire                dbg_drop = det_dbg_valid && !ctrl_debug;
    wire                dbg_pop  = !ctrl_loop && (dbg_cnt != 0) && m_axis_tready;
    always @(posedge aclk) begin
        if (dbg_push) dbg_mem[dbg_wr] <= {tl_hit, det_dbg_bits, det_dbg_metric};
    end
    always @(posedge aclk) begin
        if (!det_rst_n) begin
            dbg_wr <= 0; dbg_rd <= 0; dbg_cnt <= 0; res <= 8'd0; rec_n <= 32'd0; dbg_sent <= 32'd0;
            tl_wr <= 2'd0; tl_rd <= 2'd0; tl_n <= 3'd0;
        end else begin
            if (dbg_push) dbg_wr <= dbg_wr + 1'b1;
            if (dbg_pop) begin dbg_rd <= dbg_rd + 1'b1; dbg_sent <= dbg_sent + 32'd1; end
            dbg_cnt <= dbg_cnt + {{DBG_BITS{1'b0}}, dbg_push} - {{DBG_BITS{1'b0}}, dbg_pop};
            res     <= res + {7'd0, rec_due} - {7'd0, dbg_pop} - {7'd0, dbg_drop};
            if (det_dbg_valid) rec_n <= rec_n + 32'd1;
            if (rec_due && s_axis_tlast && ctrl_debug) begin
                tl[tl_wr] <= rec_of_beat; tl_wr <= tl_wr + 2'd1;
            end
            tl_n <= tl_n + {2'd0, rec_due && s_axis_tlast && ctrl_debug} - {2'd0, det_dbg_valid && tl_hit};
            if (det_dbg_valid && tl_hit) tl_rd <= tl_rd + 2'd1;
        end
    end
    wire [32:0] dbg_head = dbg_mem[dbg_rd];

    // ------------------------------------------------------------------ input gate
    always @(posedge aclk) begin
        if (!det_rst_n) gate <= 1'b0;
        else gate <= !ctrl_loop && (ev_cnt < EV_LIMIT) && (!ctrl_debug || res < DBG_LIMIT) &&
                     (tl_n < 3'd3);
    end

    // ------------------------------------------------------------------ streams
    assign s_axis_tready = ctrl_loop ? m_axis_tready : (gate && det_tready);
    assign m_axis_tvalid = ctrl_loop ? s_axis_tvalid : (dbg_cnt != 0);
    assign m_axis_tdata  = ctrl_loop ? s_axis_tdata  : dbg_head[31:0];
    assign m_axis_tlast  = ctrl_loop ? s_axis_tlast  : dbg_head[32];
    assign m_axis_tkeep  = 4'hF;

    // ------------------------------------------------------------------ AXI4-Lite
    wire busy = det_busy || (ser_n != 4'd0) || (dbg_cnt != 0);
    wire [31:0] status = {21'd0, ctrl_loop, !det_rst_n, ev_ovf, det_open_n, 2'd0,
                          det_open_n != 4'd0, busy};
    reg  awready_r, wready_r, arready_r;
    reg  [11:0] wa;
    assign s_axi_awready = awready_r;
    assign s_axi_wready  = wready_r;
    assign s_axi_arready = arready_r;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_rresp   = 2'b00;

    always @(posedge aclk) begin
        if (!aresetn) begin
            awready_r <= 1'b0; wready_r <= 1'b0; s_axi_bvalid <= 1'b0;
            ctrl_reset <= 1'b1; ctrl_debug <= 1'b0; ctrl_loop <= 1'b0;
            r_eta_hi <= 16'd8028; r_eta_lo <= 16'd2621; r_qual <= 16'd128; r_ncfo <= 16'd2048;
            r_blank <= 32'd288; r_hold <= 32'd131072; r_stage <= 2'd0; r_shift <= 4'd10;
            r_lin <= 24'd1307615;
        end else begin
            awready_r <= 1'b0;
            wready_r  <= 1'b0;
            // accept a write when address and data are both there; respond after the handshake
            if (!awready_r && !s_axi_bvalid && s_axi_awvalid && s_axi_wvalid) begin
                awready_r <= 1'b1;
                wready_r  <= 1'b1;
                case (s_axi_awaddr[7:2])
                    6'h01: begin ctrl_reset <= s_axi_wdata[0]; ctrl_debug <= s_axi_wdata[1];
                                 ctrl_loop <= s_axi_wdata[2]; end
                    6'h08: r_eta_hi <= s_axi_wdata[15:0];
                    6'h09: r_eta_lo <= s_axi_wdata[15:0];
                    6'h0A: r_qual   <= s_axi_wdata[15:0];
                    6'h0B: r_ncfo   <= s_axi_wdata[15:0];
                    6'h0C: r_blank  <= s_axi_wdata;
                    6'h0D: r_hold   <= s_axi_wdata;
                    6'h0E: r_stage  <= s_axi_wdata[1:0];
                    6'h0F: r_shift  <= s_axi_wdata[3:0];
                    6'h10: r_lin    <= s_axi_wdata[23:0];
                    default: ;
                endcase
            end
            if (awready_r)                        s_axi_bvalid <= 1'b1;
            else if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end

    always @(posedge aclk) begin
        if (!aresetn) begin
            arready_r <= 1'b0; s_axi_rvalid <= 1'b0; ev_pop <= 1'b0;
        end else begin
            arready_r <= 1'b0;
            ev_pop    <= 1'b0;
            if (!arready_r && !s_axi_rvalid && s_axi_arvalid) begin
                arready_r <= 1'b1;
                case (s_axi_araddr[7:2])
                    6'h00: s_axi_rdata <= ID;
                    6'h01: s_axi_rdata <= {29'd0, ctrl_loop, ctrl_debug, ctrl_reset};
                    6'h02: s_axi_rdata <= status;
                    6'h03: s_axi_rdata <= samp_cnt;
                    6'h04: s_axi_rdata <= det_open_t;
                    6'h05: s_axi_rdata <= {23'd0, ev_cnt};
                    6'h06: begin
                        s_axi_rdata <= (ev_cnt != 0) ? ev_mem[ev_rd] : 32'd0;
                        ev_pop      <= (ev_cnt != 0) && det_rst_n;
                    end
                    6'h07: s_axi_rdata <= dbg_sent;
                    6'h08: s_axi_rdata <= {16'd0, r_eta_hi};
                    6'h09: s_axi_rdata <= {16'd0, r_eta_lo};
                    6'h0A: s_axi_rdata <= {16'd0, r_qual};
                    6'h0B: s_axi_rdata <= {16'd0, r_ncfo};
                    6'h0C: s_axi_rdata <= r_blank;
                    6'h0D: s_axi_rdata <= r_hold;
                    6'h0E: s_axi_rdata <= {30'd0, r_stage};
                    6'h0F: s_axi_rdata <= {28'd0, r_shift};
                    6'h10: s_axi_rdata <= {8'd0, r_lin};
                    default: s_axi_rdata <= 32'd0;
                endcase
            end
            if (arready_r)                         s_axi_rvalid <= 1'b1;
            else if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
        end
    end
endmodule

`default_nettype wire
