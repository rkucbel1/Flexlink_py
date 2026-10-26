"""
pd_reference.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Golden reference for the FPGA packet detector (Specification_PacketDetector.md, section 4).
Input: the 12-bit I/Q samples exactly as the board receives them. Output: the event
records the board must return, bit for bit.

What is exact and what is provisional
    Exact (integer, matches the HDL by construction):
      - products c32 and e, running sums P and R0 over L = 256, R_D = R0 delayed by 32
      - the 18-bit comparator (5.3): a per-sample common shift s so the largest of
        |Re P|, |Im P|, R0, R_D fits 18-bit signed, each truncated by an arithmetic right
        shift (floor), then  (Re'^2 + Im'^2) * 2^16  >  ETA_SQ * R0' * R_D'
      - blanking, qualifier with hysteresis, holdoff/re-arm, t_trig
      - the lag 16/32/128/256 accumulators and the integer cascade, given the 16-bit angles
    Provisional (to be pinned down in the bit-true model, spec 7):
      - CORDIC angle: here round(atan2 / pi * 32768); a real CORDIC differs by an LSB or so
      - tone-structure check: computed in floating point here
    pd_hw_test.py therefore compares t_trig exactly and cfo_inc / angles / ToneCheck with a
    small tolerance until the CORDIC and DFT word lengths are fixed.

Conventions
    sample_cnt counts samples from RESET (0 = first sample). Samples before 0 are zero.
    Blanking: samples n < blank_len never count towards the qualifier.
    Qualifier: count += 1 on above_hi, count = 0 on below_lo, held otherwise; the trigger
      fires on the sample where count reaches qual_len: t_trig = that sample's index.
    Holdoff: the trigger is disabled from t_trig + 1 and re-arms at t_trig + arm_holdoff,
      count restarting from 0 (stands in for the Rx FSM's arm pulse).
    CFO window: n = t_trig .. t_trig + N_CFO - 1, using x(n - lag) from the delay line
      (it may reach back before t_trig), A_lag = s_lag * sum x(n) conj(x(n - lag)),
      s_16 = -1. cfo_valid rises at sample_cnt = t_trig + N_CFO.
    Cascade in NCO units (LSB = Fs / 2^24): f_lag = ang16(lag) * 256 / lag, step = 2^24 / lag,
      f <- f_lag + floor((f - f_lag + step/2) / step) * step, starting from f_16.
      cfo_inc = -f. All integer, no multiplier: 256/lag and the step are powers of two.
    Tone check on x(t_trig .. t_trig + N_CFO - 1) with f_hat = f * Fs / 2^24, as in
      Receive_Get_Frequency_Offset.tone_structure_check.
    dbg_metric = min(65535, floor(2^16 * |P'|^2 / (R0' R_D'))), 0 when R0' R_D' = 0.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pd_protocol import (DBG_ABOVE_HI, DBG_ARMED, DBG_BELOW_LO, DBG_BLANK, DBG_TRIG, EV_CFO_VALID,
                         EV_TONE_FAIL, EV_TRUNCATED, FS, NCO_BITS, EventRecord, Params)

L = 256                                   # running-sum length (fixed at synthesis)
D = 32                                    # detection lag (fixed at synthesis)
CMP_BITS = 18                             # comparator input width (spec 5.3)
CASCADE = ((16, -1), (32, 1), (128, 1), (256, 1))
TONE_OFFSETS = (-1.875e6, -0.625e6, 0.625e6, 1.875e6)
NOISE_REF_OFFSETS = (0.0, 1.25e6, -1.25e6, 0.3125e6, -0.3125e6, 1.5625e6, 0.9375e6, -0.9375e6, -1.5625e6)


@dataclass
class Result:
    events: list[EventRecord]
    metric: np.ndarray | None = None      # uint16 per sample (debug)
    bits: np.ndarray | None = None        # uint16 per sample (debug)
    extra: list[dict] = field(default_factory=list)   # float-domain values per event, for reports


def _delay(a: np.ndarray, d: int) -> np.ndarray:
    out = np.zeros_like(a)
    if d < a.size:
        out[d:] = a[:a.size - d]
    return out


def _running_sum(a: np.ndarray, n: int) -> np.ndarray:
    cs = np.cumsum(a)
    out = cs.copy()
    out[n:] -= cs[:-n]
    return out


def _bit_length(m: np.ndarray) -> np.ndarray:
    """Bit length of non-negative int64 values < 2^53 (0 -> 0)."""
    _, e = np.frexp(m.astype(np.float64))
    return np.where(m > 0, e, 0).astype(np.int64)


def comparator_inputs(xi: np.ndarray, xq: np.ndarray):
    """Return (Pr', Pi', R0', RD') after the common shift, plus the full-precision sums."""
    xi_d, xq_d = _delay(xi, D), _delay(xq, D)
    c_re = xi * xi_d + xq * xq_d                  # x(n) conj(x(n-32))
    c_im = xq * xi_d - xi * xq_d
    e = xi * xi + xq * xq
    pr, pi, r0 = _running_sum(c_re, L), _running_sum(c_im, L), _running_sum(e, L)
    rd = _delay(r0, D)
    m = np.maximum.reduce([np.abs(pr), np.abs(pi), r0, rd])
    s = np.maximum(_bit_length(m) - (CMP_BITS - 1), 0)
    return (pr >> s, pi >> s, r0 >> s, rd >> s), (pr, pi, r0, rd)


def quantise_angle(z: complex) -> int:
    a = int(round(np.angle(z) / np.pi * 32768))
    return -32768 if a >= 32768 else a


def cfo_cascade(angles: dict[int, int]) -> int:
    """Integer cascade; returns f_hat in NCO units (LSB Fs / 2^24)."""
    f = angles[16] * (256 // 16)
    for lag in (32, 128, 256):
        f_lag = angles[lag] * (256 // lag)
        step = (1 << NCO_BITS) // lag
        f = f_lag + ((f - f_lag + step // 2) // step) * step
    return f


def tone_check_db(seg: np.ndarray, f_hat_hz: float) -> float:
    n = np.arange(seg.size)

    def power(f):
        return abs(np.sum(seg * np.exp(-2j * np.pi * f * n / FS))) ** 2

    tones = np.array([power(f_hat_hz + f) for f in TONE_OFFSETS])
    noise = float(np.mean([power(f_hat_hz + f) for f in NOISE_REF_OFFSETS]))
    weaker = tones.sum() - tones.max()
    if noise <= 0:
        return 127.0 if weaker > 0 else -128.0
    return float(10 * np.log10(max(weaker, 1e-300) / noise))


def find_triggers(above: np.ndarray, below: np.ndarray, p: Params) -> list[int]:
    """Qualifier with hysteresis, blanking and holdoff (vectorised)."""
    n = above.size
    cum_a = np.concatenate([[0], np.cumsum(above, dtype=np.int64)])   # cum_a[k] = above in [0, k)
    below_idx = np.where(below, np.arange(n), -1)
    trig = []
    arm = p.blank_len
    q = max(int(p.qual_len), 1)
    while arm < n:
        seg = slice(arm, n)
        # last count reset at or before each sample: a below_lo sample, or the arming point
        last_reset = np.maximum.accumulate(np.maximum(below_idx[seg], arm - 1))
        count = cum_a[arm + 1:n + 1] - cum_a[last_reset + 1]
        hit = np.flatnonzero(count >= q)
        if hit.size == 0:
            break
        t = arm + int(hit[0])
        trig.append(t)
        arm = t + max(int(p.arm_holdoff), 1)
    return trig


def detect(iq: np.ndarray, p: Params, debug: bool = False) -> Result:
    """Run the detector over int16 I/Q samples (N, 2), starting from reset."""
    iq = np.asarray(iq)
    xi, xq = iq[:, 0].astype(np.int64), iq[:, 1].astype(np.int64)
    n = xi.size
    (pr, pi, r0, rd), _ = comparator_inputs(xi, xq)
    num = (pr * pr + pi * pi) << 16
    den = r0 * rd
    above = num > p.eta_hi_sq * den
    below = num < p.eta_lo_sq * den
    trig = find_triggers(above, below, p)

    events, extra = [], []
    x = xi + 1j * xq
    for t in trig:
        N = int(p.n_cfo)
        if t + N > n:
            events.append(EventRecord(t, 0, t + N, 0, EV_TRUNCATED))
            extra.append({"t_trig": t})
            continue
        win = np.arange(t, t + N)
        angles, acc = {}, {}
        for lag, sign in CASCADE:
            ii, qq = xi[win], xq[win]
            idx = win - lag
            ok = idx >= 0
            di = np.where(ok, xi[np.maximum(idx, 0)], 0)
            dq = np.where(ok, xq[np.maximum(idx, 0)], 0)
            a = complex(sign * int(np.sum(ii * di + qq * dq)), sign * int(np.sum(qq * di - ii * dq)))
            acc[lag] = a
            angles[lag] = quantise_angle(a)
        f = cfo_cascade(angles)
        f_hz = f * FS / 2 ** NCO_BITS
        tone = tone_check_db(x[t:t + N], f_hz)
        tone_q8 = int(np.clip(round(tone * 256), -32768, 32767))
        status = EV_CFO_VALID
        if p.tone_thresh > 0 and tone_q8 < p.tone_thresh:
            status |= EV_TONE_FAIL
        events.append(EventRecord(t, -f, t + N, tone_q8, status,
                                  angles[16], angles[32], angles[128], angles[256]))
        extra.append({"t_trig": t, "f_hat_hz": f_hz, "tone_db": tone,
                      "lag_estimates_hz": {lag: angles[lag] * FS / (65536 * lag) for lag in angles}})

    res = Result(events, extra=extra)
    if debug:
        with np.errstate(divide="ignore", invalid="ignore"):
            met = np.where(den > 0, num // np.maximum(den, 1), 0)
        res.metric = np.minimum(met, 65535).astype(np.uint16)
        bits = np.zeros(n, np.uint16)
        bits[above] |= DBG_ABOVE_HI
        bits[below] |= DBG_BELOW_LO
        bits[:min(p.blank_len, n)] |= DBG_BLANK
        armed = np.zeros(n, bool)
        start = p.blank_len
        for t in trig:
            armed[start:t + 1] = True
            start = t + max(int(p.arm_holdoff), 1)
        armed[start:] = True
        bits[armed] |= DBG_ARMED
        bits[trig] |= DBG_TRIG
        res.bits = bits
    return res
