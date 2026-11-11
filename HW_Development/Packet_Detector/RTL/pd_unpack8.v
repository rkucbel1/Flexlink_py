// pd_unpack8.v  (Flexlink_py/HW_Development/Packet_Detector/RTL)
//
// Optional int8 sample unpacker in front of the detector (pd_axi CTRL bit 3, PACK8; bitstream
// ID 9.1). The CoreBoard (ESP32-S31, ESP-SDR) delivers 8-bit I/Q; with PACK8 the firmware only
// copies those bytes into DDR and the AXI DMA carries two samples per 32-bit word:
//
//     s_tdata = {Q1, I1, Q0, I0}  (bytes 3..0, little-endian: the order of the ESR2 payload)
//
// Each word becomes two beats in the detector's format, sample 0 first:
//
//     m_tdata = {Q << 4, I << 4}  (12-bit samples in int16, I in [15:0], Q in [31:16])
//
// i.e. exactly what the firmware's CPU conversion (pd_server.c udp_store) produced. TLAST of a
// word goes with its second sample. Transfers must hold whole words (an even sample count).
//
// With en = 0 the module is a wire: m = s, s_tready = m_tready (no register in the path), so
// the existing 1-sample-per-word format (LOAD / RUN, pd_hw_test.py) is unchanged.
//
// The only state is the phase bit (which half of the word is next). It is cleared while en = 0
// and by the bus reset; it returns to 0 after every whole word.
`timescale 1ns / 1ps
module pd_unpack8 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire [31:0] s_tdata,
    input  wire        s_tvalid,
    input  wire        s_tlast,
    output wire        s_tready,
    output wire [31:0] m_tdata,
    output wire        m_tvalid,
    output wire        m_tlast,
    input  wire        m_tready
);
    reg phase;                                   // 0: bytes 1:0 (sample 0) next, 1: bytes 3:2

    wire [7:0]  i8 = phase ? s_tdata[23:16] : s_tdata[7:0];
    wire [7:0]  q8 = phase ? s_tdata[31:24] : s_tdata[15:8];
    wire [31:0] up = {{4{q8[7]}}, q8, 4'b0000, {4{i8[7]}}, i8, 4'b0000};

    assign m_tdata  = en ? up : s_tdata;
    assign m_tvalid = s_tvalid;
    assign m_tlast  = en ? (s_tlast && phase) : s_tlast;
    assign s_tready = en ? (m_tready && phase) : m_tready;   // the word is consumed with sample 1

    always @(posedge clk) begin
        if (!rst_n || !en)
            phase <= 1'b0;
        else if (s_tvalid && m_tready)
            phase <= ~phase;
    end
endmodule
