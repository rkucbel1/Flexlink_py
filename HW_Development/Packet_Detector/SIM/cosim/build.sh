#!/bin/bash
# Builds fw_cosim: the board firmware (FW/pd_server.c) with fake lwIP (harness_tcp.c, a real TCP
# socket) and the PL as Verilated RTL (pl_model.cpp: pd_axi + an AXI DMA model). Needs Verilator 5.
#   ./build.sh && ./obj/fw_cosim 5101          # then: python ../../PC/pd_hw_test.py <vector> --host 127.0.0.1 --port 5101
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
R="$HERE/../../RTL"
gcc -O2 -Wall -Wno-unused-function -I inc -c ../../FW/pd_server.c -o pd_server.o
gcc -O2 -I inc -c harness_tcp.c -o harness_tcp.o
verilator --cc --exe --build -O2 -j 4 -Wno-fatal -Wno-WIDTH -DPD_SIM_CHECKS --top-module pd_axi -Mdir obj \
  -CFLAGS "-I$HERE/inc -O2" -LDFLAGS "$HERE/pd_server.o $HERE/harness_tcp.o" -o fw_cosim \
  "$R"/pd_halfband.v "$R"/pd_dc_blocker.v "$R"/pd_lpf.v "$R"/pd_div16.v "$R"/pd_detect_core.v "$R"/pd_cordic.v \
  "$R"/pd_cfo_cascade.v "$R"/pd_cfo_lane.v "$R"/pd_tone_rom.v "$R"/pd_sdp_ram.v "$R"/pd_tone_check.v "$R"/pd_timing.v \
  "$R"/pd_detect_top.v "$R"/pd_axi.v "$HERE"/pl_model.cpp > build.log 2>&1 || { tail -30 build.log; exit 1; }
echo "built obj/fw_cosim"
