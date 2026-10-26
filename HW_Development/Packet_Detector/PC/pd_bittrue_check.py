#!/usr/bin/env python3
"""
pd_bittrue_check.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Checks the bit-true model (pd_reference.py) against the floating-point simulator and picks
its word lengths:

  1. Word-length sweeps, each block against its floating-point version:
     DC blocker fractional bits, LPF coefficient bits, CORDIC iterations / angle width,
     tone-check table size and amplitude.
  2. Monte Carlo with real packets (Tx/Waveform_Payload_Tx.h5): AWGN, CFO uniform +-500 kHz,
     SNR = preamble power / noise power over the 40 MHz band (as in the spec). Both chains
     see the same samples: float = simulator modules on the decimated signal; bit-true =
     the same signal quantised to 12 bits (rms -12 dBFS) through pd_reference.
     Reported: Pd, latency, CFO error (rms / p99 / max), ToneCheck, agreement.
  3. Noise only: false triggers per second at eta_hi 0.33 and 0.35.
  4. DC offset and CW interferer: trigger and tone-check outcome.

    python pd_bittrue_check.py                 # everything (~5 minutes)
    python pd_bittrue_check.py --quick         # fewer trials (~1 minute)
    python pd_bittrue_check.py --only sweeps   # or: mc, noise, interferers

Results: printed, and written to ../BITTRUE_results.json.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy import signal

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_reference as R  # noqa: E402
from pd_protocol import FS, Params  # noqa: E402

IDLE = 1500                       # noise samples before the packet, 20 Msps
PA_OFFSET = 100                   # AGC burst
L_A = 5000
BACKOFF_DB = 12.0
PRE = 8192                        # front-end pre-roll for steady-state interferer tests (8 x 2^10)


# --------------------------------------------------------------------------------------- #
# Simulator modules and test signal                                                         #
# --------------------------------------------------------------------------------------- #
def find_root(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "flexlink_paths.py").is_file():
            return p
    raise FileNotFoundError("Flexlink_py root not found above this script")


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[path.stem] = mod
    spec.loader.exec_module(mod)
    return mod


def load_sim(root: Path):
    pre = root / "Flexlink_BasicModules" / "Preamble"
    return {"lpf": load_module(pre / "Receive_LP_Filter.py"),
            "det": load_module(pre / "Receive_Packet_Detector.py"),
            "freq": load_module(pre / "Receive_Get_Frequency_Offset.py")}


def load_tx(root: Path) -> np.ndarray:
    import h5py
    with h5py.File(root / "Tx" / "Waveform_Payload_Tx.h5", "r") as f:
        x = np.asarray(f["data"][()])
    if x.dtype.names:
        x = x["r"] + 1j * x["i"] if "r" in x.dtype.names else x["real"] + 1j * x["imag"]
    return np.asarray(x, np.complex128).ravel()


def make_trials(tx: np.ndarray, snr_db: float, n: int, rng, cfo_max=500e3, payload40=2000,
                extra=None, packet=True, idle=IDLE):
    """(n, N20) complex decimated signals, CFOs, Preamble A start in the LPF domain.
    packet=False: noise (at the level SNR snr_db would give) plus `extra` only."""
    pkt = tx[:12480 + payload40] * (1.0 if packet else 0.0)
    p_pre = float(np.mean(np.abs(tx[:12480]) ** 2))
    n40 = 2 * idle + pkt.size
    t = np.arange(n40) / 40e6
    cfo = rng.uniform(-cfo_max, cfo_max, n)
    sig = np.zeros((n, n40), complex)
    sig[:, 2 * idle:] = pkt[None, :]
    sig *= np.exp(2j * np.pi * cfo[:, None] * t[None, :])
    pn = p_pre / 10 ** (snr_db / 10)
    sig += np.sqrt(pn / 2) * (rng.standard_normal(sig.shape) + 1j * rng.standard_normal(sig.shape))
    if extra is not None:
        sig += extra(t)
    x20 = signal.resample_poly(sig, 1, 2, axis=1) * np.sqrt(2)   # Decimate_Payload_Rx "filter"
    return x20, cfo, idle + PA_OFFSET + R.LPF_DELAY


def quantise(x20: np.ndarray, backoff_db=BACKOFF_DB) -> np.ndarray:
    """Per record: scale to rms backoff dB below 12-bit full scale, round, clip -> (N, 2, n)."""
    rms = np.sqrt(np.mean(np.abs(x20) ** 2, axis=1, keepdims=True))
    xs = x20 * (R.X_MAX * 10 ** (-backoff_db / 20) / rms)
    iq = np.stack([np.round(xs.real), np.round(xs.imag)], axis=1)     # (n, 2, N)
    return np.clip(iq, R.X_MIN, R.X_MAX).astype(np.int64).transpose(2, 1, 0)


def params(eta_hi=0.35, holdoff=6240) -> Params:
    return Params(eta_hi_sq=int(round(eta_hi ** 2 * 65536)), arm_holdoff=holdoff)


# --------------------------------------------------------------------------------------- #
# 1. Word-length sweeps                                                                     #
# --------------------------------------------------------------------------------------- #
def sweep_dc(rng) -> list[dict]:
    """DC blocker: error vs the float filter, residual DC, accumulator size, limit cycles."""
    n = 60000
    sigma = R.X_MAX * 10 ** (-BACKOFF_DB / 20) / math.sqrt(2)
    x = np.clip(np.round(rng.normal(0, sigma, (n, 2)) + [sigma, -0.5 * sigma]), R.X_MIN, R.X_MAX).astype(np.int64)
    yf = signal.lfilter([1, -1], [1, -(1 - 2 ** -10)], x.astype(float), axis=0)
    step = np.zeros((20000, 2), np.int64)
    step[:200] = [1500, -1500]                      # DC burst then silence: limit-cycle test
    out = []
    for frac in (0, 2, 4, 6, 8, 10, 12):
        y, acc = R.dc_blocker(x, 10, frac, return_acc=True)
        e = (y - yf)[10000:]
        ys = R.dc_blocker(step, 10, frac)
        tail = ys[15000:]
        out.append({"frac": frac, "err_rms_lsb": float(e.std()), "err_mean_lsb": float(e.mean()),
                    "residual_dc_lsb": float(np.abs(y[10000:].mean(axis=0)).max()),
                    "acc_bits": int(np.abs(acc).max()).bit_length() + 1,
                    "limit_cycle_lsb": int(np.abs(tail).max())})
    return out


def sweep_lpf(rng) -> list[dict]:
    n = 40000
    sigma = R.X_MAX * 10 ** (-BACKOFF_DB / 20) / math.sqrt(2)
    y = np.clip(np.round(rng.normal(0, sigma, (n, 2))), R.X_MIN, R.X_MAX).astype(np.int64)
    zf = signal.lfilter(np.array(R.H_LP), [1], y.astype(float), axis=0)
    out = []
    for frac in (10, 12, 14, 15, 16, 17):
        coef = tuple(int(round(h * (1 << frac))) for h in R.H_LP)
        z = R.lpf(y, coef, frac)
        e = z - zf
        w, h = signal.freqz(np.array(coef) / (1 << frac), worN=4096, fs=FS)
        _, h0 = signal.freqz(np.array(R.H_LP), worN=4096, fs=FS)
        stop = np.abs(w) > 4e6
        out.append({"frac": frac, "coef_bits": max(abs(c) for c in coef).bit_length() + 1,
                    "err_rms_lsb": float(e.std()), "err_max_lsb": float(np.abs(e).max()),
                    "resp_dev_db": float(np.max(np.abs(20 * np.log10(np.abs(h[~stop]) / np.abs(h0[~stop]))))),
                    "stopband_db": float(20 * np.log10(np.abs(h[stop]).max()))})
    return out


def sweep_cordic(rng) -> list[dict]:
    """Max angle error (16-bit LSB) against exact atan2 on CFO-accumulator-sized vectors."""
    vec = []
    for _ in range(4000):
        mag = 10 ** rng.uniform(4, 10.5)             # |A| over -14..+30 dB at 12-bit, 2048 terms
        a = rng.uniform(-math.pi, math.pi)
        vec.append((int(mag * math.cos(a)), int(mag * math.sin(a))))
    vec += [(-5, 0), (-5, -1), (-5, 1), (0, 7), (0, -7), (1, 0), (-(2 ** 36), 1)]
    exact = np.array([math.atan2(i, r) / math.pi * 32768 for r, i in vec])
    out = []
    for iters in (10, 12, 14, 15, 16, 18):
        for zb in (18, 20, 22, 24):
            got = np.array([R.cordic_angle(r, i, iters, R.CORDIC_IN_BITS, zb) for r, i in vec])
            e = (got - exact + 32768) % 65536 - 32768
            out.append({"iters": iters, "z_bits": zb, "err_max_lsb": float(np.abs(e).max()),
                        "err_rms_lsb": float(e.std())})
    return out


# --------------------------------------------------------------------------------------- #
# 2. Monte Carlo                                                                            #
# --------------------------------------------------------------------------------------- #
def float_chain(x20: np.ndarray, sim, eta_hi=0.35):
    """Simulator chain on one float record: (t_trig or None, f_hat, ToneCheck dB)."""
    lf, det, fq = sim["lpf"], sim["det"], sim["freq"]
    x = lf.lp_filter_flexlink(lf.dc_blocker(x20))
    flag, _, _, _ = det.packet_detector_flexlink(x, R.L, 0.2, eta_hi, normaliser="Bounded", qualifier=128)
    hits = np.flatnonzero(flag)
    if hits.size == 0:
        return None, None, None
    t = int(hits[0])
    if t + 2048 > x.size:
        return t, None, None
    c, f, _ = fq.detect_frequency_offset_cascade(flag, x)
    tone, _, _ = fq.tone_structure_check(x, t, c + f)
    return t, float(c + f), float(tone)


def bittrue_chain(x_lpf_iq: np.ndarray, p: Params):
    res = R.detect(x_lpf_iq, p)
    if not res.events:
        return None, None, None, None
    e, x = res.events[0], res.extra[0]
    if "f_hat_hz" not in x:
        return e.t_trig, None, None, None
    return e.t_trig, x["f_hat_hz"], x["tone_db"], bool(e.status & 2)


def stats(err: list[float]) -> dict:
    if not err:
        return {}
    a = np.abs(np.array(err))
    return {"rms": float(np.sqrt(np.mean(a ** 2))), "p99": float(np.percentile(a, 99)), "max": float(a.max())}


def monte_carlo(tx, sim, snrs, n_trials, rng) -> list[dict]:
    p = params()
    out = []
    for snr in snrs:
        t0 = time.time()
        x20, cfo, pa0 = make_trials(tx, snr, n_trials, rng)
        iq = quantise(x20)                                   # (N, 2, n)
        x_lpf = R.front_end(iq, 10)                          # bit-true front end, all trials at once
        r = {"snr_db": snr, "trials": n_trials}
        rows = {"float": [], "bit": []}
        same_t, dt, dtone, dec_agree, dec_n = 0, [], [], 0, 0
        for k in range(n_trials):
            tf, ff, tonef = float_chain(x20[k], sim)
            tb, fb, toneb, failb = bittrue_chain(x_lpf[:, :, k], p)
            for name, (t, f, tone) in (("float", (tf, ff, tonef)), ("bit", (tb, fb, toneb))):
                rows[name].append((t, f, tone))
            if tf is not None and tb is not None:
                same_t += tf == tb
                dt.append(tb - tf)
                if tonef is not None and toneb is not None:
                    dtone.append(toneb - tonef)
                    dec_n += 1
                    dec_agree += (tonef < 13) == failb
        for name, rr in rows.items():
            det_ok = [(t, f, tone, c) for (t, f, tone), c in zip(rr, cfo)
                      if t is not None and pa0 - 300 <= t < pa0 + L_A]
            false = sum(1 for t, _, _ in rr if t is not None and t < pa0 - 300)
            lat = [t - pa0 for t, _, _, _ in det_ok]
            valid = [(t, f, tone, c) for t, f, tone, c in det_ok if f is not None and t - pa0 + 2048 <= L_A]
            ferr = [f - c for _, f, _, c in valid]
            tones = [tone for _, _, tone, _ in valid]
            r[name] = {"pd": len(det_ok) / n_trials, "false_early": false,
                       "latency_max": int(max(lat)) if lat else None,
                       "late": len(det_ok) - len(valid),
                       "cfo_err_hz": stats(ferr),
                       "cfo_over_5khz": int(sum(abs(e) > 5000 for e in ferr)),
                       "tone_min_db": float(min(tones)) if tones else None,
                       "tone_below_13": int(sum(t < 13 for t in tones))}
        r["same_t_trig"] = same_t
        r["dt_max"] = int(np.abs(dt).max()) if dt else None
        r["dtone_max_db"] = float(np.abs(dtone).max()) if dtone else None
        r["tone_decision_agree"] = f"{dec_agree}/{dec_n}"
        r["seconds"] = round(time.time() - t0, 1)
        out.append(r)
        b, f = r["bit"], r["float"]
        print(f"  {snr:+5.0f} dB  Pd float {f['pd']:.3f} bit {b['pd']:.3f} | CFO rms float "
              f"{f['cfo_err_hz'].get('rms', float('nan')):7.0f} bit {b['cfo_err_hz'].get('rms', float('nan')):7.0f} Hz"
              f" | same t_trig {same_t}/{len(dt)}, max dt {r['dt_max']} | dTone max {r['dtone_max_db']}"
              f" | {r['seconds']} s", flush=True)
    return out


def tone_sweep(tx, sim, rng, n_trials) -> list[dict]:
    """ToneCheck error of the integer DFT (LUT bits / amplitude / power width) vs float."""
    x20, cfo, _ = make_trials(tx, -12, n_trials, rng)
    iq = quantise(x20)
    x_lpf = R.front_end(iq, 10)
    p = params()
    cases = []
    for k in range(n_trials):
        res = R.detect(x_lpf[:, :, k], p)
        if res.events and "f_hat_hz" in res.extra[0]:
            e = res.events[0]
            xi, xq = x_lpf[e.t_trig:e.t_trig + 2048, 0, k], x_lpf[e.t_trig:e.t_trig + 2048, 1, k]
            cases.append((xi, xq, -e.cfo_inc))
    # CW cases (single tone near the noise level): the check has to fail these
    for lvl in (0.0, 10.0):
        def cw(t, lvl=lvl):
            return np.sqrt(0.36 * 10 ** (lvl / 10)) * np.exp(2j * np.pi * 1.1e6 * t)
        x20c, _, _ = make_trials(tx, 0.0, 4, rng, extra=cw, packet=False)
        xc = R.front_end(quantise(x20c), 10)
        for k in range(xc.shape[2]):
            res = R.detect(xc[:, :, k], p)
            if res.events and "f_hat_hz" in res.extra[0]:
                e = res.events[0]
                cases.append((xc[e.t_trig:e.t_trig + 2048, 0, k], xc[e.t_trig:e.t_trig + 2048, 1, k], -e.cfo_inc))
    n = np.arange(2048)
    ref_db = []
    for xi, xq, f in cases:
        x = xi + 1j * xq
        fh = f * FS / (1 << 24)

        def pw(fr):
            return abs(np.sum(x * np.exp(-2j * np.pi * fr * n / FS))) ** 2
        tones = [pw(fh + o) for o in (-1.875e6, -0.625e6, 0.625e6, 1.875e6)]
        refs = [pw(fh + o) for o in (0, 1.25e6, -1.25e6, 0.3125e6, -0.3125e6, 1.5625e6, 0.9375e6, -0.9375e6, -1.5625e6)]
        ref_db.append(10 * math.log10(max(sum(tones) - max(tones), 1e-300) / (sum(refs) / 9)))
    out = []
    for lut in (6, 8, 10, 12):
        for amp in (10, 12, 16):
            for pwr in (12, 18):
                err = [R.tone_db(*R.tone_check(xi, xq, f, lut, amp, pwr)[:2]) - rd
                       for (xi, xq, f), rd in zip(cases, ref_db)]
                out.append({"lut_bits": lut, "amp_bits": amp, "pwr_bits": pwr, "cases": len(cases),
                            "err_max_db": float(np.abs(err).max()), "err_rms_db": float(np.std(err))})
    return out


# --------------------------------------------------------------------------------------- #
# 3. Noise only and 4. interferers                                                          #
# --------------------------------------------------------------------------------------- #
def noise_only(sim, seconds, rng) -> list[dict]:
    """Noise generated at 40 Msps and decimated (as the packets), in records of 100k samples
    at 20 Msps, each starting from reset (288 samples of blanking per record)."""
    rec, batch = 100000, 50
    n_rec = int(round(seconds * FS / rec))
    etas = (0.33, 0.35)
    bit = {e: 0 for e in etas}
    passed = {e: 0 for e in etas}                     # bit-true triggers that pass the tone check
    flt = {e: 0 for e in etas}
    lf, det = sim["lpf"], sim["det"]
    for b0 in range(0, n_rec, batch):
        nb = min(batch, n_rec - b0)
        w = (rng.standard_normal((nb, 2 * rec)) + 1j * rng.standard_normal((nb, 2 * rec))) / math.sqrt(2)
        x20 = signal.resample_poly(w, 1, 2, axis=1) * math.sqrt(2)
        x_lpf = R.front_end(quantise(x20), 10)
        for e in etas:
            p = params(e, holdoff=6240)
            for k in range(nb):
                evs = R.detect(x_lpf[:, :, k], p).events
                bit[e] += len(evs)
                passed[e] += sum(1 for ev in evs if ev.status & 1 and not ev.status & 2)
            for k in range(nb):
                x = lf.lp_filter_flexlink(lf.dc_blocker(x20[k]))
                flag, _, _, _ = det.packet_detector_flexlink(x, R.L, 0.2, e, normaliser="Bounded", qualifier=128)
                flt[e] += int(np.sum(np.diff(np.concatenate([[0], flag.astype(int)])) == 1))
    out = []
    for e in etas:
        out.append({"eta_hi": e, "seconds": n_rec * rec / FS, "bit_triggers": bit[e], "float_triggers": flt[e],
                    "bit_pass_tone_check": passed[e]})
        print(f"  eta_hi {e}: {n_rec * rec / FS:.1f} s of noise, triggers bit-true {bit[e]}, float {flt[e]};"
              f" passing the tone check {passed[e]}", flush=True)
    return out


def interferers(tx, sim, rng, n=10) -> list[dict]:
    """Noise + DC or CW, no packet: trigger? tone check fails? (bit-true and float)."""
    p = params()
    out = []
    for kind, levels in (("DC", (-6, 0, 10, 20, 30)), ("CW 1.1 MHz", (-10, 0, 10, 20, 30))):
        for lvl in levels:
            def extra(t, lvl=lvl, kind=kind):
                amp = math.sqrt(0.36 * 10 ** (lvl / 10))         # relative to the noise power at SNR 0
                ph = rng.uniform(0, 2 * math.pi)
                return amp * np.exp(1j * ph) * (np.ones_like(t) if kind == "DC" else np.exp(2j * np.pi * 1.1e6 * t))
            # PRE samples of pre-roll: the front end has been running (steady state) when the
            # detector starts; "after reset" runs the front end from zero state as well.
            x20, _, _ = make_trials(tx, 0.0, n, rng, extra=extra, packet=False, idle=IDLE + PRE)
            fe = R.front_end(quantise(x20), 10)
            steady, x20s = fe[PRE:], x20[:, PRE:]
            reset = R.front_end(quantise(x20s), 10)
            row = {"kind": kind, "level_db": lvl, "records": n}
            for name, xl in (("bit", steady), ("bit_after_reset", reset)):
                trig = rej = 0
                for k in range(n):
                    tb, _, _, failb = bittrue_chain(xl[:, :, k], p)
                    trig += tb is not None
                    rej += bool(failb)
                row[name] = {"triggers": trig, "rejected": rej}
            ft = ff = 0
            for k in range(n):
                tf, _, tonef = float_chain(x20s[k], sim)
                ft += tf is not None
                ff += tf is not None and tonef is not None and tonef < 13
            row["float"] = {"triggers": ft, "rejected": ff}
            out.append(row)
            print(f"  {kind:10s} {lvl:+3d} dB: triggers/rejected  bit-true {row['bit']['triggers']}/"
                  f"{row['bit']['rejected']}  after reset {row['bit_after_reset']['triggers']}/"
                  f"{row['bit_after_reset']['rejected']}  float {ft}/{ff}", flush=True)
    return out


# --------------------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check the bit-true packet-detector model")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", choices=("sweeps", "mc", "noise", "interferers"))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", type=Path, default=HERE.parent / "BITTRUE_results.json")
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    root = find_root(HERE)
    sim = load_sim(root)
    tx = load_tx(root)
    res = {"seed": a.seed, "quick": a.quick,
           "word_lengths": {"DC_FRAC": R.DC_FRAC, "LPF_FRAC": R.LPF_FRAC, "CMP_BITS": R.CMP_BITS,
                            "CORDIC_ITER": R.CORDIC_ITER, "CORDIC_IN_BITS": R.CORDIC_IN_BITS,
                            "CORDIC_Z_BITS": R.CORDIC_Z_BITS, "TONE_LUT_BITS": R.TONE_LUT_BITS,
                            "TONE_AMP_BITS": R.TONE_AMP_BITS, "TONE_PWR_BITS": R.TONE_PWR_BITS}}
    if a.out.is_file():
        try:
            res = {**json.loads(a.out.read_text()), **res}
        except json.JSONDecodeError:
            pass
    nt = 40 if a.quick else 200
    if a.only in (None, "sweeps"):
        print("1. Word-length sweeps")
        res["sweep_dc"] = sweep_dc(rng)
        res["sweep_lpf"] = sweep_lpf(rng)
        res["sweep_cordic"] = sweep_cordic(rng)
        res["sweep_tone"] = tone_sweep(tx, sim, rng, 20 if a.quick else 60)
        for k in ("sweep_dc", "sweep_lpf"):
            for row in res[k]:
                print("  ", k, row)
        print("   CORDIC (iters, z_bits, max err LSB):",
              [(r["iters"], r["z_bits"], round(r["err_max_lsb"], 2)) for r in res["sweep_cordic"]])
        print("   tone (lut, amp, pwr, max err dB):",
              [(r["lut_bits"], r["amp_bits"], r["pwr_bits"], round(r["err_max_db"], 3)) for r in res["sweep_tone"]])
    if a.only in (None, "mc"):
        print("2. Monte Carlo, bit-true vs float")
        res["monte_carlo"] = monte_carlo(tx, sim, (-14, -13, -12, -10, -6, 0, 10, 30), nt, rng)
    if a.only in (None, "noise"):
        print("3. Noise only")
        res["noise_only"] = noise_only(sim, 0.5 if a.quick else 5.0, rng)
    if a.only in (None, "interferers"):
        print("4. DC offset and CW interferer (no packet)")
        res["interferers"] = interferers(tx, sim, rng, 5 if a.quick else 10)
    a.out.write_text(json.dumps(res, indent=1))
    print(f"Wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
