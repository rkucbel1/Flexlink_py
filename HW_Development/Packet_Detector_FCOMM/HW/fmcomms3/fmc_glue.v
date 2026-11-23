// fmc_glue.v  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/HW/fmcomms3)
//
// Small glue for the FMCOMMS3 build variant (build_hw_fmcomms3.tcl), used as a module reference
// so the block design needs no xlslice / xlconcat / xlconstant cells (those IPs are being replaced
// by inline_hdl versions in recent Vivado releases).
//
//  - PS EMIO GPIO (64 bits) -> AD9361 control pins, with the same bit numbers as ADI's fmcomms2
//    reference design, so no-OS's default GPIO numbers work unchanged (Zynq GPIO = 54 + bit):
//       bit 48 up_txnrx  (-> axi_ad9361, which drives the TXNRX pin)        Zynq GPIO 102
//       bit 47 up_enable (-> axi_ad9361, which drives the ENABLE pin)       Zynq GPIO 101
//       bit 46 gpio_resetb                                                  Zynq GPIO 100
//       bit 45 gpio_sync                                                    Zynq GPIO  99
//       bit 44 gpio_en_agc                                                  Zynq GPIO  98
//       bits 43:40 gpio_ctl[3:0]                                            Zynq GPIO 97:94
//       bits 39:32 gpio_status[7:0] (inputs)                                Zynq GPIO 93:86
//    All other bits read back what the PS drives, as in ADI's design.
//  - PS SPI0 on EMIO: tie the unused slave-side inputs (SS_I high = no other master).
//  - One Rx channel (1R1T): {Q0, I0} -> one 32-bit word, [15:0] = I, [31:16] = Q, the same
//    layout as the pd_0 stream word.
//  - dma_wr_sync = 1 for axi_dmac's fifo_wr_sync, which exists only with SYNC_TRANSFER_START = 1.
//  - pd_rec_tready = 1 for pd_0's debug-record output (M_AXIS): this variant has no AXI DMA S2MM,
//    so records (sent only with CTRL.DEBUG or LOOPBACK) are dropped instead of stalling pd_0.
//  - X_INTERFACE_IGNORE on every port: without it Vivado infers ADI fifo_wr interfaces (one per bit)
//    from the port names and warns about each one. Plain wires are wanted here.

`timescale 1ns / 1ps

module fmc_glue (
    // PS7 EMIO GPIO
    (* X_INTERFACE_IGNORE = "true" *) input  wire [63:0] gpio_o,
    (* X_INTERFACE_IGNORE = "true" *) output wire [63:0] gpio_i,

    // AD9361 control (FMC pins, except up_* which go to axi_ad9361)
    (* X_INTERFACE_IGNORE = "true" *) input  wire [7:0]  gpio_status,
    (* X_INTERFACE_IGNORE = "true" *) output wire [3:0]  gpio_ctl,
    (* X_INTERFACE_IGNORE = "true" *) output wire        gpio_en_agc,
    (* X_INTERFACE_IGNORE = "true" *) output wire        gpio_sync,
    (* X_INTERFACE_IGNORE = "true" *) output wire        gpio_resetb,
    (* X_INTERFACE_IGNORE = "true" *) output wire        up_enable,
    (* X_INTERFACE_IGNORE = "true" *) output wire        up_txnrx,

    // PS7 SPI0 slave-side inputs
    (* X_INTERFACE_IGNORE = "true" *) output wire        spi0_ss_i,
    (* X_INTERFACE_IGNORE = "true" *) output wire        spi0_sclk_i,
    (* X_INTERFACE_IGNORE = "true" *) output wire        spi0_mosi_i,

    // axi_ad9361 Rx channel 0 -> axi_dmac FIFO write data
    (* X_INTERFACE_IGNORE = "true" *) input  wire [15:0] adc_data_i0,
    (* X_INTERFACE_IGNORE = "true" *) input  wire [15:0] adc_data_q0,
    (* X_INTERFACE_IGNORE = "true" *) output wire [31:0] adc_data_iq,
    (* X_INTERFACE_IGNORE = "true" *) output wire        dma_wr_sync,     // axi_dmac fifo_wr_sync: always 1 (free-running capture)
    (* X_INTERFACE_IGNORE = "true" *) output wire        pd_rec_tready    // pd_0 m_axis_tready: always 1 (no S2MM, see below)
);

    assign pd_rec_tready = 1'b1;

    assign dma_wr_sync = 1'b1;

    assign up_txnrx    = gpio_o[48];
    assign up_enable   = gpio_o[47];
    assign gpio_resetb = gpio_o[46];
    assign gpio_sync   = gpio_o[45];
    assign gpio_en_agc = gpio_o[44];
    assign gpio_ctl    = gpio_o[43:40];

    assign gpio_i = {gpio_o[63:40], gpio_status, gpio_o[31:0]};

    assign spi0_ss_i   = 1'b1;
    assign spi0_sclk_i = 1'b0;
    assign spi0_mosi_i = 1'b0;

    assign adc_data_iq = {adc_data_q0, adc_data_i0};

endmodule
