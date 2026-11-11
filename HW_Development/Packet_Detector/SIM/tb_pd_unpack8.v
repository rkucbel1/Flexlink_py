// tb_pd_unpack8.v  (Flexlink_py/HW_Development/Packet_Detector/SIM)
//
// Self-checking testbench of RTL/pd_unpack8.v with random tvalid / tready gaps:
//   en = 1: 4000 random words -> 8000 samples, each {Q << 4, I << 4} of the right byte pair,
//           in order, TLAST only on the second sample of a word with TLAST
//   en = 0: words pass unchanged, one beat per word
// Run with Vivado's simulator (from this folder):
//   xvlog ../RTL/pd_unpack8.v tb_pd_unpack8.v && xelab tb_pd_unpack8 -s tb_unpack && xsim tb_unpack -R
// Expect "PASS" at the end; any mismatch prints the beat and stops with "FAIL".
`timescale 1ns / 1ps
module tb_pd_unpack8;
    reg         clk = 0, rst_n = 0, en = 0;
    reg  [31:0] s_tdata;
    reg         s_tvalid = 0, s_tlast = 0, m_tready = 0;
    wire        s_tready, m_tvalid, m_tlast;
    wire [31:0] m_tdata;

    pd_unpack8 dut (.clk(clk), .rst_n(rst_n), .en(en),
                    .s_tdata(s_tdata), .s_tvalid(s_tvalid), .s_tlast(s_tlast), .s_tready(s_tready),
                    .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tlast(m_tlast), .m_tready(m_tready));
    always #5 clk = ~clk;

    localparam integer NW = 4000;
    reg  [31:0] words [0:NW-1];
    reg         lasts [0:NW-1];
    integer     sent, got, errors, i, mode;

    function [31:0] expect_up(input [31:0] w, input integer half);
        reg [7:0] i8, q8;
        begin
            i8 = half ? w[23:16] : w[7:0];
            q8 = half ? w[31:24] : w[15:8];
            expect_up = {{4{q8[7]}}, q8, 4'b0, {4{i8[7]}}, i8, 4'b0};
        end
    endfunction

    // source: random gaps, holds data until accepted (AXI-Stream rules)
    always @(posedge clk) begin
        if (rst_n) begin
            if (s_tvalid && s_tready) sent = sent + 1;
            if (!s_tvalid || s_tready) begin
                if (sent < NW && ($random & 3) != 0) begin
                    s_tvalid <= 1; s_tdata <= words[sent]; s_tlast <= lasts[sent];
                end else begin
                    s_tvalid <= 0;
                end
            end
            m_tready <= ($random & 3) != 0;
        end
    end

    // sink: check every beat
    always @(posedge clk) begin
        if (rst_n && m_tvalid && m_tready) begin
            if (mode == 1) begin
                if (m_tdata !== expect_up(words[got / 2], got % 2) ||
                    m_tlast !== (lasts[got / 2] && (got % 2 == 1))) begin
                    $display("FAIL: en=1 sample %0d: got %h last %b, expected %h last %b", got, m_tdata, m_tlast,
                             expect_up(words[got / 2], got % 2), lasts[got / 2] && (got % 2 == 1));
                    errors = errors + 1;
                end
            end else begin
                if (m_tdata !== words[got] || m_tlast !== lasts[got]) begin
                    $display("FAIL: en=0 beat %0d: got %h, expected %h", got, m_tdata, words[got]);
                    errors = errors + 1;
                end
            end
            got = got + 1;
        end
    end

    task run_mode(input integer m);
        integer want;
        begin
            mode = m; sent = 0; got = 0;
            for (i = 0; i < NW; i = i + 1) begin
                words[i] = $random;
                lasts[i] = (($random & 15) == 0);
            end
            want = m ? 2 * NW : NW;
            @(negedge clk); en = m; rst_n = 1;
            for (i = 0; i < 200000 && got < want && errors == 0; i = i + 1) @(posedge clk);
            @(negedge clk); rst_n = 0; s_tvalid = 0; en = 0;
            repeat (3) @(posedge clk);
            if (got != want) begin
                $display("FAIL: en=%0d: %0d beats out, expected %0d", m, got, want);
                errors = errors + 1;
            end else
                $display("en=%0d: %0d words -> %0d beats checked", m, NW, got);
        end
    endtask

    initial begin
        errors = 0;
        repeat (3) @(posedge clk);
        run_mode(1);
        run_mode(0);
        run_mode(1);                    // again after en = 0 (phase restarts at 0)
        if (errors == 0) $display("PASS"); else $display("FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
