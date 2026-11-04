#!/usr/bin/env bash
# run_sim.sh - Packet_Transmitter RTL tests (Verilator 5 --binary, or Vivado xsim with SIM=xsim)
#
#   ./run_sim.sh ifft          # ptx_ifft against ptx_reference.ifft_fixed
#   ./run_sim.sh core [case]   # ptx_core against ptx_reference (make_core_vectors.py: json, mix, stress, big, sweep)
#   ./run_sim.sh pluto [case]  # ptx_pluto: two clocks, DMA and DAC sides, stock path
#   ./run_sim.sh all           # everything
#   SIM=xsim ./run_sim.sh ifft
#
# Vectors are (re)built by the make_*_vectors.py scripts in this folder (Python 3 + numpy).
set -euo pipefail
cd "$(dirname "$0")"
RTL=../RTL
SIM=${SIM:-verilator}
what=${1:-ifft}
shift || true

build_run() {   # top, sources...
  local top=$1; shift
  if [ "$SIM" = xsim ]; then
    mkdir -p work/xsim_$top
    xvlog --sv -d XSIM "$@" > work/xsim_$top/xvlog.log
    xelab -debug off -timescale 1ns/1ps "$top" -s "$top" > work/xsim_$top/xelab.log
    xsim "$top" -R | grep -E "PASS|FAIL|error|Error" || true
  else
    local obj=work/obj_$top
    verilator --binary --timing -O2 -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      --top-module "$top" -Mdir "$obj" "$@" > work/verilator_$top.log 2>&1 || { tail -30 work/verilator_$top.log; exit 1; }
    "$obj/V$top"
  fi
}

mkdir -p work
case "$what" in
  ifft)
    python3 make_ifft_vectors.py "$@"
    build_run tb_ptx_ifft tb_ptx_ifft.v $RTL/ptx_ifft.v $RTL/ptx_roms.v
    ;;
  core)
    python3 make_core_vectors.py "$@"
    build_run tb_ptx_core tb_ptx_core.v $RTL/ptx_core.v $RTL/ptx_enc.v $RTL/ptx_ifft.v $RTL/ptx_roms.v
    ;;
  corefile)   # vectors already in work/core (FW/test_fw.py --rtl)
    build_run tb_ptx_core tb_ptx_core.v $RTL/ptx_core.v $RTL/ptx_enc.v $RTL/ptx_ifft.v $RTL/ptx_roms.v
    ;;
  pluto)
    python3 make_core_vectors.py "$@"
    build_run tb_ptx_pluto tb_ptx_pluto.v $RTL/ptx_pluto.v $RTL/ptx_core.v $RTL/ptx_enc.v $RTL/ptx_ifft.v $RTL/ptx_roms.v
    ;;
  all)
    "$0" ifft 1 3 && "$0" core json 1 3 1 && "$0" core sim 1 3 1 && "$0" core mix 2 3 1 && "$0" core stress 3 0 1 && \
      "$0" core big 4 2 1 && "$0" core sweep 5 3 1 && "$0" core sweep 6 0 0 && "$0" pluto mix 7
    ;;
  *) echo "unknown test $what"; exit 2 ;;
esac
