#!/usr/bin/env python3
"""
pd_noise_rate.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

False-trigger rate of the detector on the Zedboard, measured on a long stretch of noise
(spec 3.3: < 0.2 triggers/s at eta_hi 0.35; the bit-true study measured about 1/s). The board
runs the samples as one continuous stream (RESET once, then RUN after RUN, state carried over),
which is what the receiver sees between packets. arm_holdoff is 1 by default, so the detector
re-arms after the N_CFO window, as the receiver does when the tone check aborts. The board's results are bit-true to the model
(pd_board_regression.py), so the samples are not re-run on the PC: only the board counts.

Noise sources (generated block by block on the PC, 4M samples = 0.2 s each):
  sim     complex white noise at 40 Msps, decimated as the simulator does (resample_poly 1/2),
          scaled to -12 dBFS rms and quantised to 12 bits: the noise of pd_bittrue_check.py;
          input stage 1 (the board runs the DC blocker and LPF)               [default]
  white   complex white noise at 20 Msps, -12 dBFS rms, input stage 1
  FILE    int16 I/Q pairs (.bin, like TestVectors/*_dec.bin), e.g. noise from your channel
          model, decimated to 20 Msps; used once from start to end (--seconds is ignored)

    python pd_noise_rate.py                          # 100 s of 'sim' noise at eta_hi 0.35
    python pd_noise_rate.py --seconds 20 --eta 0.33,0.35,0.40
    python pd_noise_rate.py --source my_noise_dec.bin --input-stage 1

For each eta_hi: number of triggers, rate with its 95% confidence interval (Poisson), how many
passed the tone check (those would cost a control-field decode in the receiver), and the
highest ToneCheck seen. Report: TestVectors/noise_rate_result.json.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
import pd_hw_test as hw  # noqa: E402

BLOCK = 4 * 1024 * 1024            # samples per LOAD + RUN
X_MAX = 2047


def noise_blocks(source: str, seconds: float, seed: int):
    """Yield int16 (n, 2) blocks of noise."""
    if source not in ("sim", "white"):
        iq = np.fromfile(source, "<i2").reshape(-1, 2)
        for i in range(0, iq.shape[0], BLOCK):
            yield iq[i:i + BLOCK]
        return
    rng = np.random.default_rng(seed)
    total = int(round(seconds * pr.FS))
    scale = X_MAX * 10 ** (-12 / 20)                  # rms of |x| = -12 dBFS (as pd_bittrue_check.quantise)
    for i in range(0, total, BLOCK):
        n = min(BLOCK, total - i)
        if source == "sim":
            from scipy import signal
            w = (rng.standard_normal(2 * n) + 1j * rng.standard_normal(2 * n)) / math.sqrt(2)
            x = signal.resample_poly(w, 1, 2) * math.sqrt(2)
        else:
            x = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) / math.sqrt(2)
        x = x * (scale / math.sqrt(np.mean(np.abs(x) ** 2)))
        iq = np.stack([np.round(x.real), np.round(x.imag)], axis=1)
        yield np.clip(iq, -2048, X_MAX).astype(np.int16)


def poisson_ci(k: int, t: float) -> tuple[float, float]:
    from scipy.stats import chi2
    lo = 0.0 if k == 0 else chi2.ppf(0.025, 2 * k) / 2
    hi = chi2.ppf(0.975, 2 * k + 2) / 2
    return lo / t, hi / t


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="False-trigger rate on noise, measured on the board")
    ap.add_argument("--host", default=hw.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--source", default="sim", help="sim, white, or an int16 I/Q .bin file")
    ap.add_argument("--seconds", type=float, default=100.0, help="noise duration at 20 Msps (sim / white)")
    ap.add_argument("--eta", default="0.35", help="eta_hi values, comma-separated")
    ap.add_argument("--input-stage", type=int, choices=(0, 1), default=1)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--holdoff", type=int, default=1,
                    help="arm_holdoff (default 1: re-arm after the N_CFO window, as the receiver does when "
                         "the tone check aborts; the default Params value stands in for a whole packet)")
    a = ap.parse_args(argv)
    etas = [float(e) for e in a.eta.split(",")]

    try:
        b = hw.Board(a.host, a.port, a.timeout)
        info = b.ping()
    except OSError as exc:
        print(f"Communication error: {exc}")
        return 2
    if info.flags & pr.BOARD_NO_DETECTOR:
        print("This bitstream has no detector")
        return 2
    kind = "emulator" if info.flags & pr.BOARD_EMULATOR else "board"
    print(f"{kind} at {a.host}:{a.port}, build {info.build}; noise '{a.source}', input stage {a.input_stage}")

    results = []
    for eta in etas:
        p = pr.Params(eta_hi_sq=int(round(eta ** 2 * 65536)), input_stage=a.input_stage, arm_holdoff=a.holdoff)
        b.call(pr.RESET)
        b.call(pr.SET_PARAMS, p.pack())
        n_tot, events, t_load, t_run, t0 = 0, [], 0.0, 0.0, time.perf_counter()
        print(f"\neta_hi {eta:.2f}: {p.describe()}")
        for blk in noise_blocks(a.source, a.seconds, a.seed):
            t1 = time.perf_counter()
            b.load(blk)
            t2 = time.perf_counter()
            reply = b.call(pr.RUN, struct.pack("<II", blk.shape[0], 0))
            t_run += time.perf_counter() - t2
            t_load += t2 - t1
            n_ev, done = struct.unpack_from("<II", reply)
            for e in pr.EventRecord.unpack_many(reply[8:], n_ev):
                events[:] = [x for x in events if x.t_trig != e.t_trig]   # truncated -> final
                events.append(e)
            n_tot += done
            secs = n_tot / pr.FS
            print(f"\r  {secs:7.1f} s of noise, {len(events)} trigger(s); "
                  f"{8 * 4 * n_tot / 1e6 / max(t_load, 1e-9):.0f} Mbit/s load, "
                  f"{time.perf_counter() - t0:.0f} s elapsed", end="", flush=True)
        print()
        secs = n_tot / pr.FS
        final = [e for e in events if e.status & pr.EV_CFO_VALID]
        passed = [e for e in final if not e.status & pr.EV_TONE_FAIL]
        lo, hi = poisson_ci(len(events), secs)
        best = max((e.tone_db for e in final), default=None)
        print(f"  {len(events)} false trigger(s) in {secs:.1f} s = {len(events) / secs:.3f}/s "
              f"(95% interval {lo:.3f} .. {hi:.3f}/s); {len(passed)} passed the tone check; "
              + (f"highest ToneCheck {best:.1f} dB" if best is not None else "no ToneCheck values")
              + f" (threshold {p.tone_thresh / 256:.0f} dB)")
        print(f"  board: {t_run:.1f} s detector runs for {secs:.1f} s of signal "
              f"({secs / max(t_run, 1e-9):.1f}x real time incl. transfer)")
        results.append({"eta_hi": eta, "seconds": secs, "triggers": len(events),
                        "rate_per_s": len(events) / secs, "rate_ci95": [lo, hi],
                        "passed_tone_check": len(passed), "max_tone_db": best,
                        "t_trig": [e.t_trig for e in events],
                        "tone_db": [round(e.tone_db, 2) for e in final]})

    print("\nSummary (spec 3.3: < 0.2 false triggers/s at eta_hi 0.35; each costs one 2048-sample "
          "CFO window unless it passes the tone check)")
    for r in results:
        print(f"  eta_hi {r['eta_hi']:.2f}: {r['rate_per_s']:.3f}/s "
              f"[{r['rate_ci95'][0]:.3f}, {r['rate_ci95'][1]:.3f}], {r['passed_tone_check']} passed the tone check")
    out = ROOT / "TestVectors" / "noise_rate_result.json"
    out.write_text(json.dumps({"target": kind, "source": a.source, "input_stage": a.input_stage,
                               "seed": a.seed, "results": results}, indent=2))
    print(f"Report {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
