#!/usr/bin/env bash
# run_cf.sh  (Flexlink_py/HW_Development/Packet_Detector/SIM)
#
# Control-field receiver RTL against the bit-true model (PC/cf_reference.py), with xsim (Vivado,
# default when xvlog is on the PATH) or Verilator (SIM=verilator):
#   1. make_cf_vectors.py writes work/cf_<case>/ (needs the venv, and TestVectors/rep3)
#   2. tb_cf_viterbi: cf_viterbi + cf_crc10 on 400 unit cases; tb_fft2: pd_fft2 (two butterflies
#      per clock, the receiver's FFT) and cf_fft (one) on 21 cases (make_fft_vectors.py)
#   3. tb_cf_rx: the whole receiver (cf_rx) on every work/cf_<case>, input with random gaps
#
#   ./run_cf.sh                 # all cases, gaps up to 3 clocks
#   GAP=0 ./run_cf.sh           # one sample per clock;  SEED=7 ./run_cf.sh  other gap pattern
#   NOVEC=1 ./run_cf.sh         # keep the existing vectors
#   SIM=verilator ./run_cf.sh   # Verilator instead of xsim
# Exit status 0 only if every case passes.
set -euo pipefail
cd "$(dirname "$0")"
VENV_PY="$(cd ../../.. && pwd)/.venv/bin/python"
if [[ -z "${PYTHON:-}" ]]; then
  if [[ -x "$VENV_PY" ]]; then PYTHON="$VENV_PY"; else PYTHON=python3; fi
fi
[[ -n "${NOVEC:-}" ]] || { "$PYTHON" make_cf_vectors.py && "$PYTHON" make_fft_vectors.py; }
RTL=(../RTL/cf_rx.v ../RTL/pd_fft2.v ../RTL/cf_tdp_ram.v ../RTL/cf_chest.v ../RTL/cf_viterbi.v ../RTL/cf_crc10.v
     ../RTL/cf_roms.v ../RTL/pd_cordic.v ../RTL/pd_tone_rom.v ../RTL/pd_sdp_ram.v)
if [[ -z "${SIM:-}" ]]; then
  if command -v xvlog > /dev/null; then SIM=xsim; else SIM=verilator; fi
fi
: > cf_sim.log
case "$SIM" in
  verilator)
    rm -rf work/obj_tb_cf_rx work/obj_tb_cf_viterbi work/obj_tb_fft2
    trap 'echo "Verilator build failed - log: SIM/work/verilator_cf.log" >&2' ERR
    verilator --binary --timing -Wno-fatal -Wno-WIDTH -Wno-INITIALDLY --top-module tb_cf_viterbi \
        --Mdir work/obj_tb_cf_viterbi -o tb_cf_viterbi -j 2 ../RTL/cf_viterbi.v ../RTL/cf_crc10.v tb_cf_viterbi.v \
        > work/verilator_cf.log 2>&1
    verilator --binary --timing -Wno-fatal -Wno-WIDTH -Wno-INITIALDLY --x-assign unique --x-initial unique \
        --top-module tb_fft2 --Mdir work/obj_tb_fft2 -o tb_fft2 -j 2 ../RTL/pd_fft2.v ../RTL/cf_fft.v \
        ../RTL/cf_tdp_ram.v ../RTL/cf_roms.v tb_fft2.v >> work/verilator_cf.log 2>&1
    verilator --binary --timing -Wno-fatal -Wno-WIDTH -Wno-INITIALDLY --x-assign unique --x-initial unique \
        --top-module tb_cf_rx \
        --Mdir work/obj_tb_cf_rx -o tb_cf_rx -j 2 "${RTL[@]}" tb_cf_rx.v >> work/verilator_cf.log 2>&1
    run_vit() { ../obj_tb_cf_viterbi/tb_cf_viterbi "+dir=."; }
    run_fft() { ../obj_tb_fft2/tb_fft2 "+dir=." +verilator+rand+reset+2; }
    # registers and RAMs start with random values (+verilator+rand+reset+2)
    run_rx()  { ../obj_tb_cf_rx/tb_cf_rx "+dir=." "+gap=${GAP:-3}" "+seed=${SEED:-1}" +verilator+rand+reset+2 \
                    "+verilator+seed+${SEED:-1}"; } ;;
  xsim)
    mkdir -p work/xsim_cf
    if ! (cd work/xsim_cf && unset LIBRARY_PATH &&
          xvlog --nolog ../../../RTL/cf_viterbi.v ../../../RTL/cf_crc10.v ../../tb_cf_viterbi.v &&
          xelab --nolog --debug off tb_cf_viterbi -s tbv &&
          xvlog --nolog ../../../RTL/pd_fft2.v ../../../RTL/cf_fft.v ../../../RTL/cf_tdp_ram.v \
                ../../../RTL/cf_roms.v ../../tb_fft2.v &&
          xelab --nolog --debug off tb_fft2 -s tbf &&
          xvlog --nolog "${RTL[@]/#/../../}" ../../tb_cf_rx.v &&
          xelab --nolog --debug off tb_cf_rx -s tbr) > work/xsim_cf_build.log 2>&1; then
      grep -E "ERROR|Error" work/xsim_cf_build.log | head -20 >&2
      echo "xsim build failed - full log: SIM/work/xsim_cf_build.log" >&2
      exit 1
    fi
    run_vit() { (cd ../xsim_cf && xsim tbv --nolog -R -testplusarg "dir=../$1"); }
    run_fft() { (cd ../xsim_cf && xsim tbf --nolog -R -testplusarg "dir=../fft_units"); }
    run_rx()  { (cd ../xsim_cf && xsim tbr --nolog -R -testplusarg "dir=../$1" -testplusarg "gap=${GAP:-3}" \
                    -testplusarg "seed=${SEED:-1}"); } ;;
  *) echo "SIM must be xsim or verilator" >&2; exit 2 ;;
esac
trap - ERR
echo "== cf_units" | tee -a cf_sim.log
(cd work/cf_units && unset LIBRARY_PATH && run_vit cf_units) | grep -E "^(PASS|FAIL|case)" | tee -a cf_sim.log
echo "== fft_units" | tee -a cf_sim.log
(cd work/fft_units && unset LIBRARY_PATH && run_fft) | grep -E "^(PASS|FAIL|case|clocks)" | tee -a cf_sim.log
n=2
for d in work/cf_*/; do
  [[ "$(basename "$d")" == cf_units ]] && continue
  echo "== $(basename "$d")" | tee -a cf_sim.log
  (cd "$d" && unset LIBRARY_PATH && run_rx "$(basename "$d")") | grep -E "^(PASS|FAIL|grid|hf|v i|phi|shift|bits|     model|metric|crc|msg|counts|tags)" |
      tee -a cf_sim.log
  n=$((n + 1))
done
! grep -q "^FAIL" cf_sim.log && [[ $(grep -c "^PASS" cf_sim.log) -eq $n ]]
