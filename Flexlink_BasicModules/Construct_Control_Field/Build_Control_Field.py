#!/usr/bin/env python3
"""
Build_Control_Field.py  (lives in Flexlink_BasicModules/Construct_Control_Field)

Rebuild the whole control field, from the payload parameters to the waveform that
Construct_Payload_Waveform.py inserts between the preamble and the payload.
Tx/Initialize_Tx.py calls this at the end of every run, so the control field is
recalculated exactly when the Tx parameters change.

Steps (each script reads the file the one before it wrote, all in Tx/ControlSignals):
    1  Initialize_Control_Field.py      grid, Calculated values, 54-bit ControlMessage
                                        (from Tx/Parameters_Tx_payload.json)
    2  Construct_Encoded_Control_Tx.py  CRC, tail bits, BCC, interleave, repeat, scramble
    3  Construct_ControlSigs_QAM.py     BPSK mapping
    4  Construct_Control_Grid.py        fill the 4-symbol grid
    5  Create_ControlField_Waveform.py  OFDM waveform -> Waveform_Control_Tx.h5

Every step's output is written to Tx/ControlSignals/Build_Control_Field.log (the
Simulate tab of App/Flexlinksimulation.py shows it under Control Field Results).

Usage:
    python Flexlink_BasicModules/Construct_Control_Field/Build_Control_Field.py
    python .../Build_Control_Field.py --payload-params other.json -v

    # or from Python
    from Build_Control_Field import build_control_field
    ok, log_lines = build_control_field(payload_params_path)

Exit status: 0 every step ran, 1 a step stopped with an error (its output is printed).
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for _parent in HERE.parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX_CONTROL_SIGNALS  # noqa: E402

LOG_FILE = TX_CONTROL_SIGNALS / "Build_Control_Field.log"

STEPS = (
    "Initialize_Control_Field.py",
    "Construct_Encoded_Control_Tx.py",
    "Construct_ControlSigs_QAM.py",
    "Construct_Control_Grid.py",
    "Create_ControlField_Waveform.py",
)


def build_control_field(payload_params: Path | None = None,
                        verbose: bool = False) -> tuple[bool, list[str]]:
    """Run the five control field steps in order.

    payload_params  Parameters_Tx_payload.json to build the ControlMessage from
                    (default: Tx/Parameters_Tx_payload.json)
    Returns (True if every step ran, the output of every step that ran).
    """
    out_lines: list[str] = []
    for i, script in enumerate(STEPS, 1):
        cmd = [sys.executable, str(HERE / script)]
        if script == "Initialize_Control_Field.py" and payload_params is not None:
            cmd += ["--payload-params", str(Path(payload_params).resolve())]
        p = subprocess.run(cmd, capture_output=True, text=True, cwd=HERE)
        text = (p.stdout + p.stderr).rstrip()
        out_lines.append(f"===== Control field step {i}: {script} (exit {p.returncode})")
        out_lines.extend(text.splitlines())
        if verbose:
            print("\n".join(out_lines[-(len(text.splitlines()) + 1):]))
        if p.returncode != 0:
            break
    else:
        _write_log(out_lines)
        return True, out_lines
    _write_log(out_lines)
    return False, out_lines


def _write_log(lines: list[str]) -> None:
    try:
        LOG_FILE.write_text("\n".join(lines) + "\n", encoding="utf-8")
    except OSError:
        pass


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Rebuild the control field waveform.")
    ap.add_argument("--payload-params", type=Path,
                    help="Parameters_Tx_payload.json (default: Tx/Parameters_Tx_payload.json)")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every step's output")
    a = ap.parse_args(argv)
    ok, lines = build_control_field(a.payload_params, a.verbose)
    if not ok and not a.verbose:
        print("\n".join(lines))
    last = next((ln for ln in reversed(lines) if ln.startswith("Wrote")), "")
    print("Control field rebuilt" + (f": {last[6:]}" if last else "") if ok
          else "Control field build stopped with an error")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
