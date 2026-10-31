#!/usr/bin/env bash
# run_soft.sh  (Flexlink_py/HW_Development/Packet_Detector/SIM)
#
# Payload demapper and combiner (RTL/pl_demap.v, pl_comb.v) against the bit-true model
# (PC/pl_reference.soft_bits, combine), with xsim (Vivado, default when xvlog is on the PATH) or
# Verilator (SIM=verilator):
#   1. make_lq_vectors.py writes work/lq_units/ (16 packets: every modulation, all 12 codes, 2 to 32
#      copies, NDB_A limits, sumN = 0, saturating LLRs and sums)
#   2. tb_pl_soft: every decoder input Lq (code order) and every code-word record, with random
#      gaps and consumer back-pressure; three runs (gap 0, 1, 3)
#
#   ./run_soft.sh                 # vectors + three runs
#   NOVEC=1 ./run_soft.sh         # keep the existing vectors
#   SIM=verilator ./run_soft.sh   # Verilator instead of xsim
#   python soft_mut.py            # fault injection (Verilator): every mutant must be caught
# Exit status 0 only if every run passes.
set -euo pipefail
cd "$(dirname "$0")"
VENV_PY="$(cd ../../.. && pwd)/.venv/bin/python"
if [[ -z "${PYTHON:-}" ]]; then
  if [[ -x "$VENV_PY" ]]; then PYTHON="$VENV_PY"; else PYTHON=python3; fi
fi
[[ -n "${NOVEC:-}" ]] || "$PYTHON" make_lq_vectors.py
RTL=(../RTL/pl_demap.v ../RTL/pl_comb.v ../RTL/pl_soft_roms.v ../RTL/cf_tdp_ram.v ../RTL/pl_dec.v
     ../RTL/pl_ldpc_core.v ../RTL/pl_ldpc_rom.v ../RTL/pl_lutram1.v ../RTL/pd_sdp_ram.v)
if [[ -z "${SIM:-}" ]]; then
  if command -v xvlog > /dev/null; then SIM=xsim; else SIM=verilator; fi
fi
case "$SIM" in
  verilator)
    rm -rf work/obj_tb_pl_soft
    verilator --binary --timing -Wno-fatal -DPD_SIM_CHECKS -Wno-WIDTH -Wno-INITIALDLY -I../RTL \
        --top-module tb_pl_soft --Mdir work/obj_tb_pl_soft -o tb_pl_soft -j 2 "${RTL[@]}" tb_pl_soft.v \
        > work/verilator_soft.log 2>&1 || { echo "Verilator build failed - log: SIM/work/verilator_soft.log" >&2; exit 1; }
    run() { ../obj_tb_pl_soft/tb_pl_soft "+dir=." "+gap=$1" "+seed=$2"; } ;;
  xsim)
    mkdir -p work/xsim_soft
    if ! (cd work/xsim_soft && unset LIBRARY_PATH &&
          xvlog --nolog -d PD_SIM_CHECKS -i ../../../RTL "${RTL[@]/#/../../}" ../../tb_pl_soft.v &&
          xelab --nolog --debug off tb_pl_soft -s tbs) > work/xsim_soft_build.log 2>&1; then
      grep -E "ERROR|Error" work/xsim_soft_build.log | head -20 >&2
      echo "xsim build failed - full log: SIM/work/xsim_soft_build.log" >&2
      exit 1
    fi
    run() { (cd ../xsim_soft && xsim tbs --nolog -R -testplusarg "dir=../lq_units" -testplusarg "gap=$1" \
                 -testplusarg "seed=$2"); } ;;
  *) echo "SIM must be xsim or verilator" >&2; exit 2 ;;
esac
: > soft_sim.log
for gs in "0 1" "1 2" "3 3"; do
  set -- $gs
  echo "== lq_units gap $1 seed $2" | tee -a soft_sim.log
  (cd work/lq_units && unset LIBRARY_PATH && run "$1" "$2") | grep -E "^(PASS|FAIL|case)" | tee -a soft_sim.log
done
! grep -q "^FAIL" soft_sim.log && [[ $(grep -c "^PASS" soft_sim.log) -eq 3 ]]
