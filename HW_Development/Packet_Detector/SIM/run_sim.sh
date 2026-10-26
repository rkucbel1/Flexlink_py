#!/usr/bin/env bash
# run_sim.sh  (Flexlink_py/HW_Development/Packet_Detector/SIM)
#
# Runs the RTL testbench against the bit-true model:
#   1. make_tb_vectors.py exports the vector and the model's expected outputs (needs the venv)
#   2. compile and run tb_pd_detect with xsim (Vivado, default) or Verilator
#
#   ./run_sim.sh                                   # rep3, input_stage 1, gaps up to 3 clocks, xsim
#   ./run_sim.sh ../TestVectors/rep3.json 0        # input_stage 0 (LPF output in)
#   GAP=0 ./run_sim.sh                             # one sample per clock
#   SIM=verilator ./run_sim.sh                     # Verilator instead of xsim
#   MAX=20000 ./run_sim.sh                         # first 20000 samples only
#   HOLDOFF=300 ./run_sim.sh                       # short re-arm holdoff: many triggers
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

args=("$VEC" --input-stage "$STAGE")
[[ -n "${MAX:-}" ]] && args+=(--max-samples "$MAX")
[[ -n "${HOLDOFF:-}" ]] && args+=(--holdoff "$HOLDOFF")
"$PYTHON" make_tb_vectors.py "${args[@]}"
DIR="work/$(basename "${VEC%.json}")_s${STAGE}"

RTL=(../RTL/pd_dc_blocker.v ../RTL/pd_lpf.v ../RTL/pd_div16.v ../RTL/pd_detect_core.v ../RTL/pd_detect_top.v)

case "$SIM" in
  xsim)
    # compile and run inside the vector folder: the testbench opens cfg.txt, in.txt, ... there
    cd "$DIR"
    # xelab compiles the design to C with its own compiler; LIBRARY_PATH from the environment can
    # make that compiler pick up the wrong system libraries (xelab warns, and may crash).
    unset LIBRARY_PATH
    xvlog --nolog -d PD_SIM_CHECKS "${RTL[@]/#/../../}" ../../tb_pd_detect.v
    # extra options, e.g. XELAB_OPTS="-mt off -v 1" (single-threaded, verbose) to diagnose a crash
    # shellcheck disable=SC2086
    xelab --nolog --debug off ${XELAB_OPTS:-} tb_pd_detect -s tb_pd_detect_sim
    xsim tb_pd_detect_sim --nolog -R \
        -testplusarg "gap=$GAP" -testplusarg "seed=$SEED" | tee ../../sim.log
    ;;
  verilator)
    verilator --binary --timing -Wno-fatal -DPD_SIM_CHECKS --top-module tb_pd_detect \
        --Mdir work/obj_dir -o tb_pd_detect -j 2 "${RTL[@]}" tb_pd_detect.v > work/verilator_build.log
    (cd "$DIR" && ../obj_dir/tb_pd_detect "+gap=$GAP" "+seed=$SEED") | tee sim.log
    ;;
  *)
    echo "SIM must be xsim or verilator" >&2; exit 2 ;;
esac

grep -q "^PASS" "$HERE/sim.log"
