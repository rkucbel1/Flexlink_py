#!/usr/bin/env python3
"""pd_make_coreboard_vector.py -- a CoreBoard capture as an RTL test vector (input stage 2).

Writes TestVectors/<name>.json + <name>_40.bin: the int8 I/Q of a capture (App/Rx_CoreBoard/Captures/*.cs8,
40 Msps) as 12-bit samples (x 16, exactly what pd_unpack8.v gives the receiver), with the
detector parameters of TestVectors/rep3.json, dc_shift 6 and input stage 2. SIM/run_sim.sh then
runs the RTL on it against the bit-true model, e.g. with the streaming DMA's pauses:

    python pd_make_coreboard_vector.py                      # EmuSource.cs8, 100 ms from 2.6 s
    cd ../SIM
    BURST=43008 BURSTGAP=200000 ./run_sim.sh ../TestVectors/coreboard.json 2

(the default stretch holds the 7 packets in which the board's receiver hung when fed in
64-datagram pieces; xsim needs some time for it: about an hour for 4 M samples)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TV = HERE.parent / "TestVectors"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="CoreBoard capture -> RTL test vector (input stage 2)")
    ap.add_argument("--file", type=Path, default=ROOT / "App" / "Rx_CoreBoard" / "Captures" / "EmuSource.cs8")
    ap.add_argument("--start-seconds", type=float, default=2.6)
    ap.add_argument("--seconds", type=float, default=0.1)
    ap.add_argument("--dc-shift", type=int, default=6)
    ap.add_argument("--name", default="coreboard")
    a = ap.parse_args(argv)

    raw = np.memmap(a.file, dtype=np.int8, mode="r")
    first = int(a.start_seconds * 40e6)
    n = int(a.seconds * 40e6) // 2 * 2
    iq8 = np.asarray(raw[2 * first:2 * (first + n)]).reshape(-1, 2)
    iq12 = iq8.astype("<i2") * 16
    TV.mkdir(exist_ok=True)
    bin_name = f"{a.name}_40.bin"
    iq12.tofile(TV / bin_name)

    params = json.loads((TV / "rep3.json").read_text())["params"]
    params.update(dc_shift=a.dc_shift, input_stage=2)
    meta = {
        "format": "int16 I,Q interleaved, little-endian, 12-bit values (CoreBoard int8 x 16), 40 Msps",
        "files": {"input40": bin_name},
        "samples_40": int(iq12.shape[0]),
        "sample_rate": 40000000.0,
        "source": f"{a.file.name} from {a.start_seconds} s, {a.seconds} s (ESP32-S31 CoreBoard capture)",
        "params": params,
        "note": "made by PC/pd_make_coreboard_vector.py; no truth / expected_events (real capture)",
    }
    (TV / f"{a.name}.json").write_text(json.dumps(meta, indent=1))
    print(f"Wrote {TV / (a.name + '.json')} and {bin_name}: {iq12.shape[0]} samples "
          f"({iq12.shape[0] / 40e3:.1f} ms) at 40 Msps, peak {int(np.abs(iq12).max())}, dc_shift {a.dc_shift}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
