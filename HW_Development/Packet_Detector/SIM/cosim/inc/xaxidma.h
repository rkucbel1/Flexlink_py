#pragma once
#include "xil_types.h"
typedef struct { UINTPTR RegBase; } XAxiDma; typedef struct {int x;} XAxiDma_Config;
XAxiDma_Config *XAxiDma_LookupConfig(UINTPTR); int XAxiDma_CfgInitialize(XAxiDma*,XAxiDma_Config*);
int XAxiDma_HasSg(XAxiDma*); void XAxiDma_Reset(XAxiDma*); int XAxiDma_ResetIsDone(XAxiDma*);
void XAxiDma_IntrDisable(XAxiDma*,u32,int); int XAxiDma_SimpleTransfer(XAxiDma*,UINTPTR,u32,int);
int XAxiDma_Busy(XAxiDma*,int); u32 XAxiDma_ReadReg(UINTPTR,u32);
#define XAXIDMA_IRQ_ALL_MASK 0x7000 
#define XAXIDMA_DEVICE_TO_DMA 1
#define XAXIDMA_DMA_TO_DEVICE 0
#define XAXIDMA_TX_OFFSET 0
#define XAXIDMA_RX_OFFSET 0x30
#define XAXIDMA_SR_OFFSET 4
#define XAXIDMA_BUFFLEN_OFFSET 0x28
#define XAXIDMA_ERR_ALL_MASK 0x770
