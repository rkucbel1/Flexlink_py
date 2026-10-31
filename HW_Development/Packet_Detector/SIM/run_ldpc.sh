#!/usr/bin/env bash
# run_ldpc.sh  (Flexlink_py/HW_Development/Packet_Detector/SIM)
#
# LDPC decoder core (RTL/pl_ldpc_core.v) against the bit-true model (PC/pl_reference.decode_ldpc),
# with xsim (Vivado, default when xvlog is on the PATH) or Verilator (SIM=verilator):
#   1. make_ldpc_vectors.py writes work/ldpc_units/ (every code, noisy and extreme code words)
#   2. tb_pl_ldpc: every hard bit and every iteration count, input with random gaps and output
#      with random back-pressure, twice (gap 0 and gap 3)
#
#   ./run_ldpc.sh                 # 540 code words
#   NOVEC=1 ./run_ldpc.sh         # keep the existing vectors
#   SIM=verilator ./run_ldpc.sh   # Verilator instead of xsim
#   python ldpc_mut.py            # fault injection (Verilator): every mutant must be caught
# Exit status 0 only if every run passes.
set -euo pipefail
cd "$(dirname "$0")"
VENV_PY="$(cd ../../.. && pwd)/.venv/bin/python"
if [[ -z "${PYTHON:-}" ]]; then
  if [[ -x "$VENV_PY" ]]; then PYTHON="$VENV_PY"; else PYTHON=python3; fi
fi
[[ -n "${NOVEC:-}" ]] || "$PYTHON" make_ldpc_vectors.py --seed 7 --per-snr 8
RTL=(../RTL/pl_ldpc_core.v ../RTL/pl_ldpc_rom.v ../RTL/pl_lutram1.v ../RTL/cf_tdp_ram.v ../RTL/pd_sdp_ram.v)
if [[ -z "${SIM:-}" ]]; then
  if command -v xvlog > /dev/null; then SIM=xsim; else SIM=verilator; fi
fi
case "$SIM" in
  verilator)
    rm -rf work/obj_tb_pl_ldpc
    verilator --binary --timing -Wno-fatal -Wno-WIDTH -Wno-INITIALDLY --x-assign unique --x-initial unique \
        --top-module tb_pl_ldpc --Mdir work/obj_tb_pl_ldpc -o tb_pl_ldpc -j 2 "${RTL[@]}" tb_pl_ldpc.v \
        > work/verilator_ldpc.log 2>&1 || { echo "Verilator build failed - log: SIM/work/verilator_ldpc.log" >&2; exit 1; }
    run() { ../obj_tb_pl_ldpc/tb_pl_ldpc "+dir=." "+gap=$1" "+seed=$2" +verilator+rand+reset+2 "+verilator+seed+$2"; } ;;
  xsim)
    mkdir -p work/xsim_ldpc
    if ! (cd work/xsim_ldpc && unset LIBRARY_PATH &&
          xvlog --nolog "${RTL[@]/#/../../}" ../../tb_pl_ldpc.v &&
          xelab --nolog --debug off tb_pl_ldpc -s tbl) > work/xsim_ldpc_build.log 2>&1; then
      grep -E "ERROR|Error" work/xsim_ldpc_build.log | head -20 >&2
      echo "xsim build failed - full log: SIM/work/xsim_ldpc_build.log" >&2
      exit 1
    fi
    run() { (cd ../xsim_ldpc && xsim tbl --nolog -R -testplusarg "dir=../ldpc_units" -testplusarg "gap=$1" \
                 -testplusarg "seed=$2"); } ;;
  *) echo "SIM must be xsim or verilator" >&2; exit 2 ;;
esac
: > ldpc_sim.log
for gs in "0 1" "3 2"; do
  set -- $gs
  echo "== ldpc_units gap $1 seed $2" | tee -a ldpc_sim.log
  (cd work/ldpc_units && unset LIBRARY_PATH && run "$1" "$2") | grep -E "^(PASS|FAIL|case|code)" | tee -a ldpc_sim.log
done
! grep -q "^FAIL" ldpc_sim.log && [[ $(grep -c "^PASS" ldpc_sim.log) -eq 2 ]]
