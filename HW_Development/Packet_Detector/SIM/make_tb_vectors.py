#!/usr/bin/env python3
"""
make_tb_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Exports a test vector (TestVectors/<name>.json + .bin / _dec.bin) as text files for the RTL
testbench, with the bit-true model's expected outputs for every sample:

  work/<name>_s<stage>/cfg.txt       input_stage dc_shift eta_hi_sq eta_lo_sq qual_len
                                     blank_len arm_holdoff n_samples n_trig
  work/<name>_s<stage>/in.txt        input samples, "iiii qqqq" (16-bit hex, two's complement)
  work/<name>_s<stage>/exp_lpf.txt   detector input (LPF output), "iii qqq" (12-bit hex)
  work/<name>_s<stage>/exp_dbg.txt   per sample "mmmm bbbb": dbg_metric, dbg_bits (hex)
  work/<name>_s<stage>/exp_trig.txt  t_trig of each trigger (decimal)

    python make_tb_vectors.py ../TestVectors/rep3.json --input-stage 1
    python make_tb_vectors.py ../TestVectors/rep3.json --input-stage 0 --max-samples 50000
    python make_tb_vectors.py stress --input-stage 1      # synthetic full-scale stress vector
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
import pd_protocol as pr  # noqa: E402
import pd_reference as ref  # noqa: E402


def stress_vector(seed: int = 7) -> np.ndarray:
    """Synthetic 12-bit input that drives every word length to its limit."""
    rng = np.random.default_rng(seed)
    n = 16000
    t = np.arange(n)
    seg = [
        np.full((n, 2), -2048),                                            # full-scale DC (both rails)
        np.where((t // 32) % 2 == 0, 2047, -2048)[:, None].repeat(2, 1),  # square wave, period 64
        rng.integers(-2048, 2048, (n, 2)),                                 # random full scale
        np.zeros((n, 2), np.int64),                                        # silence (den = 0, settling)
        np.stack([np.clip(np.round(3000 * (np.cos(2 * np.pi * t / 32 + np.pi / 4)
                                           + np.cos(2 * np.pi * 3 * t / 32 + 3 * np.pi / 4))), -2048, 2047),
                  np.clip(np.round(3000 * (np.sin(2 * np.pi * t / 32 + np.pi / 4)
                                           + np.sin(2 * np.pi * 3 * t / 32 + 3 * np.pi / 4))), -2048, 2047)],
                 axis=1),                                                  # clipped Preamble-A-like tones
        np.stack([np.full(n, 2047), np.full(n, -2048)], axis=1),          # opposite DC on I and Q
        np.where(t % 2 == 0, 2047, -2048)[:, None] * np.array([[1, -1]]),  # Nyquist, full scale
        rng.integers(-40, 41, (n, 2)),                                     # small noise
    ]
    return np.clip(np.concatenate(seg), -2048, 2047).astype(np.int64)


def export(vector: Path, stage: int, max_samples: int | None, out_root: Path, holdoff: int | None) -> Path:
    if str(vector) == "stress":
        p = pr.Params(arm_holdoff=500)
        p.input_stage = stage
        iq_dec = stress_vector()
        # input_stage 0 feeds the same pattern straight into the detector
        iq = iq_dec
        vector = Path("stress.json")
    else:
        meta = json.loads(vector.with_suffix(".json").read_text())
        known = {f.name for f in fields(pr.Params)}
        p = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
        p.input_stage = stage
        name = meta["files"]["decimated"] if stage == 1 else meta["files"]["lpf_out"]
        iq = np.fromfile(vector.with_name(name), dtype="<i2").reshape(-1, 2).astype(np.int64)
    if holdoff is not None:
        p.arm_holdoff = holdoff
    if max_samples:
        iq = iq[:max_samples]

    lpf = ref.front_end(iq, p.dc_shift) if stage == 1 else iq
    res = ref.detect(iq, p, debug=True)
    trig = [e.t_trig for e in res.events]

    out = out_root / f"{vector.stem}_s{stage}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "cfg.txt").write_text(
        f"{p.input_stage} {p.dc_shift} {p.eta_hi_sq} {p.eta_lo_sq} {p.qual_len} {p.blank_len} "
        f"{p.arm_holdoff} {iq.shape[0]} {len(trig)}\n")
    u16 = (iq & 0xFFFF)
    np.savetxt(out / "in.txt", u16, fmt="%04x")
    np.savetxt(out / "exp_lpf.txt", lpf & 0xFFF, fmt="%03x")
    np.savetxt(out / "exp_dbg.txt", np.stack([res.metric, res.bits], axis=1).astype(np.int64), fmt="%04x")
    (out / "exp_trig.txt").write_text("".join(f"{t}\n" for t in trig))
    print(f"{out}: {iq.shape[0]} samples, input_stage {stage}, {len(trig)} trigger(s) at {trig}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Export a test vector for the RTL testbench")
    ap.add_argument("vector", type=Path, help="TestVectors/<name>.json, or 'stress' for the synthetic vector")
    ap.add_argument("--input-stage", type=int, choices=(0, 1), default=1)
    ap.add_argument("--max-samples", type=int)
    ap.add_argument("--holdoff", type=int, help="override arm_holdoff (e.g. small, to get many triggers)")
    ap.add_argument("--out", type=Path, default=HERE / "work")
    a = ap.parse_args(argv)
    export(a.vector, a.input_stage, a.max_samples, a.out, a.holdoff)
    return 0


if __name__ == "__main__":
    sys.exit(main())
