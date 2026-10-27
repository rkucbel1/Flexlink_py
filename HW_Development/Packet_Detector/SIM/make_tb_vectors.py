#!/usr/bin/env python3
"""
make_tb_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Exports a test vector (TestVectors/<name>.json + .bin / _dec.bin) as text files for the RTL
testbench, with the bit-true model's expected outputs for every sample:

  work/<name>_s<stage>/cfg.txt       input_stage dc_shift eta_hi_sq eta_lo_sq qual_len
                                     blank_len arm_holdoff n_samples n_trig n_cfo n_cfo_events
  work/<name>_s<stage>/in.txt        input samples, "iiii qqqq" (16-bit hex, two's complement)
  work/<name>_s<stage>/exp_lpf.txt   detector input (LPF output), "iii qqq" (12-bit hex)
  work/<name>_s<stage>/exp_dbg.txt   per sample "mmmm bbbb": dbg_metric, dbg_bits (hex)
  work/<name>_s<stage>/exp_trig.txt  t_trig of each trigger (decimal)
  work/<name>_s<stage>/exp_cfo.txt   per completed CFO window (hex): t, A16 re im, A32 re im,
                                     A128 re im, A256 re im (37-bit), ang16 ang32 ang128 ang256,
                                     cfo_inc (32-bit). Windows ending in the last LANE_DELAY
                                     samples are left out (the RTL lane needs those samples).

    python make_tb_vectors.py ../TestVectors/rep3.json --input-stage 1
    python make_tb_vectors.py ../TestVectors/rep3.json --input-stage 0 --max-samples 50000
    python make_tb_vectors.py stress --input-stage 1      # synthetic full-scale stress vector
    python make_tb_vectors.py cfo --input-stage 1         # 40 real packets, CFO +-600 kHz, SNR -6..30 dB
    python make_tb_vectors.py units                       # CORDIC + cascade unit-test vectors
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


def cfo_sweep_vector(seed: int = 11, n_packets: int = 40) -> np.ndarray:
    """Real packets (Tx/Waveform_Payload_Tx.h5) back to back, each with its own CFO (uniform
    +-600 kHz) and SNR (uniform -6..30 dB), decimated and quantised as in pd_bittrue_check."""
    import pd_bittrue_check as chk
    rng = np.random.default_rng(seed)
    tx = chk.load_tx(chk.find_root(HERE))
    recs = []
    for _ in range(n_packets):
        x20, _, _ = chk.make_trials(tx, float(rng.uniform(-6, 30)), 1, rng, cfo_max=600e3)
        recs.append(chk.quantise(x20)[:, :, 0])
    return np.concatenate(recs).astype(np.int64)


def units_vectors(out_root: Path, seed: int = 3) -> Path:
    """CORDIC vectors (random + edge cases) and cascade angle sets (random + exact rounding
    boundaries and one LSB either side) with the model's results."""
    rng = np.random.default_rng(seed)
    m37, lo, hi = (1 << 37) - 1, -(1 << 36), (1 << 36) - 1
    vec = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (-5, 0), (-5, -1), (-5, 1), (1, 1), (-1, -1),
           (hi, 0), (lo, 0), (0, hi), (0, lo), (hi, hi), (lo, lo), (lo, hi), (hi, lo), (lo, -1), (lo, 1),
           (3, -4), ((1 << 22) - 1, 1), (1 << 22, -1), ((1 << 21), (1 << 21)), (-(1 << 23), 7)]
    for _ in range(20000):
        mag = 10 ** rng.uniform(0, 10.8)
        a = rng.uniform(-np.pi, np.pi)
        vec.append((int(np.clip(round(mag * np.cos(a)), lo, hi)), int(np.clip(round(mag * np.sin(a)), lo, hi))))
    for _ in range(2000):                                     # small integers: exact angle edges
        vec.append((int(rng.integers(-40, 41)), int(rng.integers(-40, 41))))
    out = out_root / "units"
    out.mkdir(parents=True, exist_ok=True)
    (out / "cordic.txt").write_text("".join(
        f"{r & m37:010x} {i & m37:010x} {ref.cordic_angle(r, i) & 0xFFFF:04x}\n" for r, i in vec))

    def wrap16(v):
        v &= 0xFFFF
        return v - 0x10000 if v >= 0x8000 else v

    sets = [tuple(int(a) for a in rng.integers(-32768, 32768, 4)) for _ in range(20000)]
    for _ in range(3000):                                     # exact rounding boundaries
        a16 = int(rng.integers(-32768, 32768))
        f16 = 16 * a16
        for d32 in (-1, 0, 1):
            a32 = wrap16(2 * a16 + 32768 + d32)               # 8 a32 = f16 + 2^18 (mod 2^19)
            f32 = 8 * a32 + ((16 * a16 - 8 * a32 + (1 << 18)) >> 19 << 19)
            for d128 in (-1, 0, 1):
                a128 = wrap16((f32 + (1 << 16)) // 2 + d128)  # 2 a128 = f32 + 2^16 (mod 2^17)
                f128 = 2 * a128 + ((f32 - 2 * a128 + (1 << 16)) >> 17 << 17)
                for d256 in (-1, 0, 1):
                    sets.append((a16, a32, a128, wrap16(f128 + (1 << 15) + d256)))
    (out / "cascade.txt").write_text("".join(
        f"{a & 0xFFFF:04x} {b & 0xFFFF:04x} {c & 0xFFFF:04x} {d & 0xFFFF:04x} "
        f"{-ref.cfo_cascade({16: a, 32: b, 128: c, 256: d}) & 0xFFFFFFFF:08x}\n" for a, b, c, d in sets))
    print(f"{out}: {len(vec)} CORDIC vectors, {len(sets)} cascade angle sets")
    return out


def export(vector: Path, stage: int, max_samples: int | None, out_root: Path, holdoff: int | None) -> Path:
    if str(vector) == "units":
        return units_vectors(out_root)
    if str(vector) == "cfo":
        p = pr.Params(arm_holdoff=4000)
        p.input_stage = stage
        iq_dec = cfo_sweep_vector()
        iq = iq_dec if stage == 1 else ref.front_end(iq_dec, p.dc_shift)
        vector = Path("cfo.json")
    elif str(vector) == "stress":
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
    n = iq.shape[0]
    xi, xq = lpf[:, 0].astype(np.int64), lpf[:, 1].astype(np.int64)
    cfo_lines = []
    for e in res.events:
        if not e.status & pr.EV_CFO_VALID or e.t_trig + p.n_cfo + ref.LANE_DELAY > n:
            continue
        acc = ref.cfo_accumulators(xi, xq, e.t_trig, p.n_cfo)
        m37 = (1 << 37) - 1
        vals = [f"{e.t_trig & 0xFFFFFFFF:08x}"]
        for lag in (16, 32, 128, 256):
            vals += [f"{acc[lag][0] & m37:010x}", f"{acc[lag][1] & m37:010x}"]
        vals += [f"{a & 0xFFFF:04x}" for a in (e.ang16, e.ang32, e.ang128, e.ang256)]
        vals.append(f"{e.cfo_inc & 0xFFFFFFFF:08x}")
        cfo_lines.append(" ".join(vals) + "\n")

    out = out_root / f"{vector.stem}_s{stage}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "cfg.txt").write_text(
        f"{p.input_stage} {p.dc_shift} {p.eta_hi_sq} {p.eta_lo_sq} {p.qual_len} {p.blank_len} "
        f"{p.arm_holdoff} {iq.shape[0]} {len(trig)} {p.n_cfo} {len(cfo_lines)}\n")
    u16 = (iq & 0xFFFF)
    np.savetxt(out / "in.txt", u16, fmt="%04x")
    np.savetxt(out / "exp_lpf.txt", lpf & 0xFFF, fmt="%03x")
    np.savetxt(out / "exp_dbg.txt", np.stack([res.metric, res.bits], axis=1).astype(np.int64), fmt="%04x")
    (out / "exp_trig.txt").write_text("".join(f"{t}\n" for t in trig))
    (out / "exp_cfo.txt").write_text("".join(cfo_lines))
    shown = trig if len(trig) <= 12 else trig[:12] + ["..."]
    print(f"{out}: {iq.shape[0]} samples, input_stage {stage}, {len(trig)} trigger(s) at {shown}, "
          f"{len(cfo_lines)} CFO result(s)")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Export a test vector for the RTL testbench")
    ap.add_argument("vector", type=Path,
                    help="TestVectors/<name>.json, or 'stress', 'cfo' (synthetic vectors), 'units' (unit tests)")
    ap.add_argument("--input-stage", type=int, choices=(0, 1), default=1)
    ap.add_argument("--max-samples", type=int)
    ap.add_argument("--holdoff", type=int, help="override arm_holdoff (e.g. small, to get many triggers)")
    ap.add_argument("--out", type=Path, default=HERE / "work")
    a = ap.parse_args(argv)
    export(a.vector, a.input_stage, a.max_samples, a.out, a.holdoff)
    return 0


if __name__ == "__main__":
    sys.exit(main())
