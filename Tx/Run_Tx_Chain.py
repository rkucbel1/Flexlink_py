#!/usr/bin/env python3
"""
Run_Tx_Chain.py  (lives in Flexlink_py/Tx)

Runs the payload Tx chain, steps 3 to 8, in order. Each step reads the file the
step before it wrote (see Tx/Docs/Flexlink_Payload _Tx_Chain.md):

    3   Generate_RandomPacketOfData/Generate_TransportBlock_Bitstream.py   random transport blocks
    4   Construct_Encoded_Bitstream_Tx.py                                 CRC, LDPC, interleave, repeat, scramble
    5   Construct_Datasigs_QAM.py                                         QAM mapping
    6   Construct_Payload_Grid.py                                         fill the resource grid
    7   Construct_Payload_Waveform.py                                     OFDM waveform (+ preamble)
    8   Defect_Model.py                                                   only when Channel_Defects.json has a
                                                                          Mode set to 1 (new channel every run)

Run Initialize_Tx.py first after any change to Defined (it writes Calculated and
Grid_Resource_Tx.h5). Every file goes to its usual place under its usual name, exactly
as running the scripts by hand. Nothing is written to the JSON files.

Usage
-----
    python Tx/Run_Tx_Chain.py
    python Tx/Run_Tx_Chain.py --seed 1234            # repeatable transport blocks
    python Tx/Run_Tx_Chain.py --channel-seed 7       # repeatable channel/noise (Defect_Model --seed)
    python Tx/Run_Tx_Chain.py --no-defects           # stop after step 7
    python Tx/Run_Tx_Chain.py --keep-blocks          # skip step 3, encode the transport blocks
                                                     # already in Bitstream_TransportBlks_Tx.h5
                                                     # (e.g. a file packet from
                                                     # Generate_FileData/Generate_File_Bitstream.py)
    python Tx/Run_Tx_Chain.py --dry-run              # print the commands only

Exit status: 0 every step ran, 3 a step stopped with an error.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

TX = Path(__file__).resolve().parent
ROOT = TX.parent

PARAMS_FILE = TX / "Parameters_Tx_payload.json"
DEFECTS_FILE = ROOT / "Flexlink_BasicModules" / "Channel_Defects" / "Channel_Defects.json"

# (step label, script, what to pick out of its output for the summary)
STEPS = [
    ("3 Generate", TX / "Generate_RandomPacketOfData" / "Generate_TransportBlock_Bitstream.py",
     r"(Wrote \d+ bits)"),
    ("4 Encode", TX / "Construct_Encoded_Bitstream_Tx.py", None),
    ("5 QAM", TX / "Construct_Datasigs_QAM.py", None),
    ("6 Grid", TX / "Construct_Payload_Grid.py", r"(Port \d+, RefSignalSpacing \d+, Mod \w+)"),
    ("7 Waveform", TX / "Construct_Payload_Waveform.py",
     r"((?:Waveform: preamble \+ payload = \d+ samples)|(?:Payload: .*samples))"),
    ("8 Defect_Model", TX / "Defect_Model.py", r"(Defects applied: .*)"),
]


def defects_on(defects_file: Path) -> list[str]:
    cfg = json.loads(Path(defects_file).read_text())
    return [k for k, v in cfg.get("Mode", {}).items() if int(v)]


class Chain:
    def __init__(self, params_path: Path, dry: bool = False, verbose: bool = False):
        self.params_path = Path(params_path)
        self.dry, self.verbose = dry, verbose
        self.py = sys.executable
        self.log: list[str] = []

    def run(self, step: str, cmd: list[Any]) -> tuple[int, str]:
        cmd = [str(c) for c in cmd]
        if self.dry:
            print("   ", " ".join(cmd))
            return 0, ""
        p = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
        out = p.stdout + p.stderr
        self.log.append(f"===== {step} (exit {p.returncode})\n{out}")
        if self.verbose:
            print(f"--- {step} (exit {p.returncode})\n{out}", end="")
        return p.returncode, out

    def run_all(self, defects_file: Path, seed: int | None = None, channel_seed: int | None = None,
                model: bool = True, generate: bool = True) -> dict[str, Any]:
        prm = ["--params", self.params_path]
        extra = {
            "3 Generate": ["--json", self.params_path] + (["--seed", seed] if seed is not None else []),
            "8 Defect_Model": [*prm, "--defects", defects_file]
                              + (["--seed", channel_seed] if channel_seed is not None else []),
        }
        res: dict[str, Any] = {"steps": [], "status": 0}
        for label, script, pattern in STEPS:
            if (label == "8 Defect_Model" and not model) or (label == "3 Generate" and not generate):
                res["steps"].append((label, "skipped", ""))
                continue
            rc, out = self.run(label, [self.py, script, *extra.get(label, prm)])
            m = re.search(pattern, out) if pattern else None
            res["steps"].append((label, "ok" if rc == 0 else f"exit {rc}", m.group(1) if m else ""))
            if rc:
                res.update(status=3, error=label)
                break
        return res


def summary_lines(res: dict[str, Any]) -> list[str]:
    lines = [f"  {label:<15} {state:<8} {note}".rstrip() for label, state, note in res.get("steps", [])]
    if res.get("error"):
        lines.append(f"  stopped at {res['error']}")
    return lines


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Run Tx steps 3 to 8.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--defects-file", type=Path, default=DEFECTS_FILE,
                    help="Channel_Defects.json whose Mode flags decide whether step 8 runs")
    ap.add_argument("--seed", type=int, help="random seed for the transport blocks (step 3)")
    ap.add_argument("--channel-seed", type=int,
                    help="random seed for Defect_Model (default: Settings/RandomSeed, else random)")
    ap.add_argument("--no-defects", action="store_true", help="never run Defect_Model.py (step 8)")
    ap.add_argument("--keep-blocks", action="store_true",
                    help="skip step 3 and encode the transport blocks already in "
                         "Generate_RandomPacketOfData/Bitstream_TransportBlks_Tx.h5 (file transfer)")
    ap.add_argument("--log", type=Path, help="write every step's output to this file")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every step's output")
    ap.add_argument("--dry-run", action="store_true", help="print the commands without running them")
    a = ap.parse_args(argv)

    on = defects_on(a.defects_file)
    model = bool(on) and not a.no_defects
    if on:
        print(f"{a.defects_file.name}: {', '.join(on)}"
              + ("" if model else " (--no-defects: Defect_Model not run)"))
    else:
        print(f"{a.defects_file.name}: no defects (Defect_Model not run)")

    chain = Chain(a.params, a.dry_run, a.verbose)
    res = chain.run_all(a.defects_file, a.seed, a.channel_seed, model, generate=not a.keep_blocks)
    if a.log:
        a.log.write_text("\n".join(chain.log))
    if not a.dry_run:
        print("Tx chain")
        for line in summary_lines(res):
            print(line)
        if res["status"] == 3 and not a.verbose:
            print("A step stopped with an error - rerun with -v or --log to see its output")
    return res["status"]


if __name__ == "__main__":
    sys.exit(main())
