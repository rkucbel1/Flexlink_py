#!/usr/bin/env bash
# run_chest.sh  (Flexlink_py/HW_Development/Packet_Detector/SIM)
#
# Payload channel estimate (RTL/pl_chest.v) against the bit-true model (PC/pl_reference.channel),
# with xsim (Vivado, default when xvlog is on the PATH) or Verilator (SIM=verilator):
#   1. make_chest_vectors.py writes work/chest_units/ (15 packets: every modulation, pilot
#      pattern, smoothing length, both SNR modes, packet shifts k > 0, PacketLength 4 to 73)
#   2. tb_pl_chest: every packet record, every H and every Y', with rejected packets in between,
#      the decision after a random symbol, random gaps and output back-pressure; three runs
#      (gap 0, 1, 3)
#
#   ./run_chest.sh                # vectors + three runs
#   NOVEC=1 ./run_chest.sh        # keep the existing vectors
#   SIM=verilator ./run_chest.sh  # Verilator instead of xsim
#   python chest_mut.py           # fault injection (Verilator): every mutant must be caught
# Exit status 0 only if every run passes.
set -euo pipefail
cd "$(dirname "$0")"
VENV_PY="$(cd ../../.. && pwd)/.venv/bin/python"
if [[ -z "${PYTHON:-}" ]]; then
  if [[ -x "$VENV_PY" ]]; then PYTHON="$VENV_PY"; else PYTHON=python3; fi
fi
[[ -n "${NOVEC:-}" ]] || "$PYTHON" make_chest_vectors.py
RTL=(../RTL/pl_chest.v ../RTL/pl_roms.v ../RTL/pl_mulseq.v ../RTL/cf_tdp_ram.v ../RTL/pd_sdp_ram.v
     ../RTL/pd_tone_rom.v ../RTL/pd_cordic.v ../RTL/pd_fft2.v ../RTL/cf_roms.v)
if [[ -z "${SIM:-}" ]]; then
  if command -v xvlog > /dev/null; then SIM=xsim; else SIM=verilator; fi
fi
case "$SIM" in
  verilator)
    rm -rf work/obj_tb_pl_chest
    verilator --binary --timing -Wno-fatal -Wno-WIDTH -Wno-INITIALDLY -I../RTL \
        --top-module tb_pl_chest --Mdir work/obj_tb_pl_chest -o tb_pl_chest -j 2 "${RTL[@]}" tb_pl_chest.v \
        > work/verilator_chest.log 2>&1 || { echo "Verilator build failed - log: SIM/work/verilator_chest.log" >&2; exit 1; }
    run() { ../obj_tb_pl_chest/tb_pl_chest "+dir=." "+gap=$1" "+seed=$2"; } ;;
  xsim)
    mkdir -p work/xsim_chest
    if ! (cd work/xsim_chest && unset LIBRARY_PATH &&
          xvlog --nolog -i ../../../RTL "${RTL[@]/#/../../}" ../../tb_pl_chest.v &&
          xelab --nolog --debug off tb_pl_chest -s tbc) > work/xsim_chest_build.log 2>&1; then
      grep -E "ERROR|Error" work/xsim_chest_build.log | head -20 >&2
      echo "xsim build failed - full log: SIM/work/xsim_chest_build.log" >&2
      exit 1
    fi
    run() { (cd ../xsim_chest && xsim tbc --nolog -R -testplusarg "dir=../chest_units" -testplusarg "gap=$1" \
                 -testplusarg "seed=$2"); } ;;
  *) echo "SIM must be xsim or verilator" >&2; exit 2 ;;
esac
: > chest_sim.log
for gs in "0 1" "1 2" "3 3"; do
  set -- $gs
  echo "== chest_units gap $1 seed $2" | tee -a chest_sim.log
  (cd work/chest_units && unset LIBRARY_PATH && run "$1" "$2") | grep -E "^(PASS|FAIL|case)" | tee -a chest_sim.log
done
! grep -q "^FAIL" chest_sim.log && [[ $(grep -c "^PASS" chest_sim.log) -eq 3 ]]
