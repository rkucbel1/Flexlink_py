#!/usr/bin/env bash
# run_sim.sh  (Flexlink_py/HW_Development/Packet_Detector/SIM)
#
# Runs the RTL testbench against the bit-true model:
#   1. make_tb_vectors.py exports the vector and the model's expected outputs (needs the venv)
#   2. compile and run tb_pd_detect with xsim (Vivado, default) or Verilator
#
#   ./run_sim.sh                                   # rep3, input_stage 1, gaps up to 3 clocks, xsim
#   ./run_sim.sh ../TestVectors/rep3.json 0        # input_stage 0 (LPF output in)
#   ./run_sim.sh cfo 2                             # input_stage 2 (40 Msps in, half-band)
#   ./run_sim.sh highsnr 2                         # packets at +40..+65 dB, near-silent idle
#   GAP=0 ./run_sim.sh                             # one sample per clock
#   FIXGAP=1 GAP=4 ./run_sim.sh stress 1           # exactly 5 clocks per sample (20 Msps): no stalls
#   SIM=verilator ./run_sim.sh                     # Verilator instead of xsim
#   MAX=20000 ./run_sim.sh                         # first 20000 samples only
#   HOLDOFF=300 ./run_sim.sh                       # short re-arm holdoff (effective >= N_CFO)
#   NCFO=4096 ./run_sim.sh cfo 1                   # other CFO window length (128..4096)
#   ./run_sim.sh stress 1 / ./run_sim.sh cfo 1     # synthetic vectors (full scale / CFO sweep)
#   ./run_sim.sh units                             # CORDIC, cascade, tone-check unit tests
#
# Python: uses the project venv (Flexlink_py/.venv) if it exists, else python3 (or PYTHON=...).
# xsim:   source the Vivado settings script first (any terminal; the venv need not be active).
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"

VEC=${1:-../TestVectors/rep3.json}
STAGE=${2:-1}
GAP=${GAP:-3}
SEED=${SEED:-1}
FIXGAP=${FIXGAP:-0}
SIM=${SIM:-xsim}
VENV_PY="$(cd ../../.. && pwd)/.venv/bin/python"
if [[ -z "${PYTHON:-}" ]]; then
  if [[ -x "$VENV_PY" ]]; then PYTHON="$VENV_PY"; else PYTHON=python3; fi
fi
if [[ "$SIM" == xsim ]] && ! command -v xvlog > /dev/null; then
  echo "xvlog not found: source the Vivado settings first, e.g." >&2
  echo "  source /tools/Xilinx/2025.2/Vivado/settings64.sh" >&2
  exit 2
fi

if [[ "$VEC" == units ]]; then
  "$PYTHON" make_tb_vectors.py units
  DIR=work/units; TOP=tb_pd_units; TB=tb_pd_units.v
  RTL=(../RTL/pd_cordic.v ../RTL/pd_cfo_cascade.v ../RTL/pd_tone_rom.v ../RTL/pd_sdp_ram.v ../RTL/pd_tone_check.v)
else
  TOP=tb_pd_detect; TB=tb_pd_detect.v
  RTL=(../RTL/pd_halfband.v ../RTL/pd_dc_blocker.v ../RTL/pd_lpf.v ../RTL/pd_div16.v ../RTL/pd_detect_core.v
       ../RTL/pd_cordic.v ../RTL/pd_cfo_cascade.v ../RTL/pd_cfo_lane.v ../RTL/pd_tone_rom.v
       ../RTL/pd_sdp_ram.v ../RTL/pd_tone_check.v ../RTL/pd_detect_top.v)
args=("$VEC" --input-stage "$STAGE")
[[ -n "${MAX:-}" ]] && args+=(--max-samples "$MAX")
[[ -n "${HOLDOFF:-}" ]] && args+=(--holdoff "$HOLDOFF")
[[ -n "${NCFO:-}" ]] && args+=(--n-cfo "$NCFO")
"$PYTHON" make_tb_vectors.py "${args[@]}"
DIR="work/$(basename "${VEC%.json}")_s${STAGE}"
fi

case "$SIM" in
  xsim)
    # compile and run inside the vector folder: the testbench opens cfg.txt, in.txt, ... there
    cd "$DIR"
    # xelab compiles the design to C with its own compiler; LIBRARY_PATH from the environment can
    # make that compiler pick up the wrong system libraries (xelab warns, and may crash).
    unset LIBRARY_PATH
    xvlog --nolog -d PD_SIM_CHECKS "${RTL[@]/#/../../}" "../../$TB"
    # extra options, e.g. XELAB_OPTS="-mt off -v 1" (single-threaded, verbose) to diagnose a crash
    # shellcheck disable=SC2086
    xelab --nolog --debug off ${XELAB_OPTS:-} "$TOP" -s "${TOP}_sim"
    xsim "${TOP}_sim" --nolog -R \
        -testplusarg "gap=$GAP" -testplusarg "seed=$SEED" -testplusarg "fixgap=$FIXGAP" | tee ../../sim.log
    ;;
  verilator)
    verilator --binary --timing -Wno-fatal -DPD_SIM_CHECKS --top-module "$TOP" \
        --Mdir "work/obj_$TOP" -o "$TOP" -j 2 "${RTL[@]}" "$TB" > work/verilator_build.log
    (cd "$DIR" && "../obj_$TOP/$TOP" "+gap=$GAP" "+seed=$SEED" "+fixgap=$FIXGAP") | tee sim.log
    ;;
  *)
    echo "SIM must be xsim or verilator" >&2; exit 2 ;;
esac

grep -q "^PASS" "$HERE/sim.log"
