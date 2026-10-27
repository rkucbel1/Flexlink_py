"""
pd_reference.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Bit-true model of the FPGA packet detector (Specification_PacketDetector.md, sections 4-5).
Every operation is integer and fully defined (widths, rounding, saturation, table contents),
so an HDL implementation that follows this file reproduces its outputs bit for bit. The
word lengths were chosen with pd_bittrue_check.py; the results are in ../BITTRUE.md.

Datapath (one complex sample per step, I and Q are independent rails where it applies)

  x_dec  12-bit signed I/Q, 20 Msps (decimated, AGC-normalised radio samples)
    |
    |  front_end()      input_stage 1 only; with input_stage 0 the PC does this and the
    v                    board receives x_lpf directly
  DC blocker            acc(n) = acc(n-1) - rnd(acc(n-1) >> k) + ((x(n) - x(n-1)) << DC_FRAC)
                         y(n)   = sat12(rnd(acc(n) >> DC_FRAC)),  k = dc_shift (0 = bypass)
  LPF, 17 taps          z(n) = sat12(rnd(sum_k H_LP_Q[k] * y(n-k) >> LPF_FRAC))
    |
  x_lpf 12-bit -------------------------------------------------------------------------
    |
  detect()
    running sums         P = sum x(n) x*(n-32), R0 = sum |x(n)|^2 over L = 256 (exact)
    comparator           common shift s: largest of |Re P|, |Im P|, R0, R_D fits 18-bit
                         signed; P', R0', RD' = value >> s (floor);
                         above_hi: (Re'^2 + Im'^2) << 16 >  ETA_HI_SQ * R0' * RD'
                         below_lo: (Re'^2 + Im'^2) << 16 <  ETA_LO_SQ * R0' * RD'
    blanking / qualifier / holdoff  -> t_trig
    CFO accumulators     A_lag = s_lag * sum_{n=t+lag..t+N-1} x(n) x*(n-lag), lag 16/32/128/256
    CORDIC (vectoring)   16-bit angle of each A_lag
    cascade              integer, NCO units (Fs / 2^24) -> f_hat, cfo_inc = -f_hat
    tone check           13 single-bin DFTs of x_lpf(t..t+N-1) with a 24-bit phase
                         accumulator and a cos/sin table; integer threshold decision

Rounding conventions
    rnd(v >> k)   = (v + 2^(k-1)) >> k      (round half up; arithmetic shift)
    v >> k        = floor(v / 2^k)          (arithmetic shift, where "floor" is stated)
    sat12(v)      = clip(v, -2048, 2047)

Detector conventions
    sample_cnt counts samples from RESET (0 = first sample). Samples before 0 are zero, and
    the DC blocker and LPF start from zero state.
    Blanking: samples n < blank_len never count towards the qualifier.
    Qualifier: count += 1 on above_hi, count = 0 on below_lo, held otherwise; the trigger
      fires on the sample where count reaches qual_len: t_trig = that sample's index.
    Holdoff: the trigger is disabled from t_trig + 1 and re-arms at t_trig + H, count
      restarting from 0 (stands in for the Rx FSM's arm pulse), H = holdoff(p) =
      max(arm_holdoff, n_cfo, 1): the detector cannot re-arm before the CFO window is over
      (one CFO lane; spec 4.2: "ignores further triggers until it is re-armed").
    CFO window: samples t_trig .. t_trig + N_CFO - 1 only. The lag-l accumulator adds
      x(n) x*(n - l) for n = t_trig + l .. t_trig + N_CFO - 1 (N_CFO - l terms), so both
      samples of every product lie in the window, as in the simulator. (Reaching back before
      t_trig pairs Preamble A with the AGC burst: about 40 Hz bias at high SNR.)
      cfo_valid rises at sample_cnt = t_trig + N_CFO.
    dbg_metric = min(65535, floor(2^16 * |P'|^2 / (R0' R_D'))), 0 when R0' R_D' = 0.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from pd_protocol import (DBG_ABOVE_HI, DBG_ARMED, DBG_BELOW_LO, DBG_BLANK, DBG_TRIG, EV_CFO_VALID,
                         EV_TONE_FAIL, EV_TRUNCATED, FS, NCO_BITS, EventRecord, Params)

# --------------------------------------------------------------------------------------- #
# Word lengths and constants (fixed at synthesis)                                          #
# --------------------------------------------------------------------------------------- #
X_BITS = 12                               # samples, all stages
X_MIN, X_MAX = -(1 << (X_BITS - 1)), (1 << (X_BITS - 1)) - 1

DC_FRAC = 10                              # DC blocker: fractional bits of the accumulator
DC_ACC_BITS = 24                          # signed accumulator: |y| <= 2 * 2048 (L1 norm 2), so |acc| <= 2^22 + rounding

H_LP = (-0.0015, -0.0107, -0.0271, -0.0388, -0.0216, 0.0431, 0.1452, 0.2418, 0.2817,
        0.2418, 0.1452, 0.0431, -0.0216, -0.0388, -0.0271, -0.0107, -0.0015)   # Receive_LP_Filter
LPF_FRAC = 17                             # coefficients: signed 18-bit, value / 2^17
H_LP_Q = tuple(int(round(h * (1 << LPF_FRAC))) for h in H_LP)
LPF_DELAY = (len(H_LP) - 1) // 2          # 8 samples
LANE_DELAY = 32                           # RTL CFO lane runs this many samples behind the input:
#                                           a window is complete LANE_DELAY samples after its end

L = 256                                   # running-sum length
D = 32                                    # detection lag
CMP_BITS = 18                             # comparator input width (spec 5.3)

CASCADE = ((16, -1), (32, 1), (128, 1), (256, 1))
ANGLE_BITS = 16                           # CORDIC output: -32768..32767 = -pi..pi

CORDIC_ITER = 16                          # micro-rotations
CORDIC_IN_BITS = 24                       # x, y after normalisation: |x|,|y| < 2^(IN_BITS-2)
CORDIC_Z_BITS = 22                        # angle accumulator: 2^Z_BITS = one full turn
CORDIC_ATAN = tuple(int(round(math.atan(2.0 ** -i) / (2 * math.pi) * (1 << CORDIC_Z_BITS)))
                    for i in range(CORDIC_ITER))

TONE_LUT_BITS = 10                        # cos/sin table address bits (top bits of the phase)
TONE_AMP_BITS = 16                        # table values: signed 16-bit, amplitude 2^15 - 1
TONE_PWR_BITS = 18                        # DFT results shifted (common) to 18-bit before squaring
TONE_LIN_FRAC = 16                        # threshold as a linear power ratio, Q.16
TONE_LIN_BITS = 24                        # unsigned; limits TONE_THRESH to 24 dB, so lin * den < 2^63
TONE_THRESH_MAX_Q8 = 24 * 256
_A = (1 << (TONE_AMP_BITS - 1)) - 1
TONE_COS = tuple(int(round(_A * math.cos(2 * math.pi * i / (1 << TONE_LUT_BITS))))
                 for i in range(1 << TONE_LUT_BITS))
TONE_SIN = tuple(int(round(_A * math.sin(2 * math.pi * i / (1 << TONE_LUT_BITS))))
                 for i in range(1 << TONE_LUT_BITS))
_NCO = 1 << NCO_BITS
TONE_OFFSETS = tuple(int(round(f * _NCO / FS)) for f in (-1.875e6, -0.625e6, 0.625e6, 1.875e6))
NOISE_REF_OFFSETS = tuple(int(round(f * _NCO / FS)) for f in
                          (0.0, 1.25e6, -1.25e6, 0.3125e6, -0.3125e6, 1.5625e6, 0.9375e6,
                           -0.9375e6, -1.5625e6))


@dataclass
class Result:
    events: list[EventRecord]
    metric: np.ndarray | None = None      # uint16 per sample (debug)
    bits: np.ndarray | None = None        # uint16 per sample (debug)
    extra: list[dict] = field(default_factory=list)   # values per event, for reports


# --------------------------------------------------------------------------------------- #
# Helpers                                                                                  #
# --------------------------------------------------------------------------------------- #
def _rnd_shift(v, k: int):
    """(v + 2^(k-1)) >> k for integer numpy arrays or ints, k >= 0."""
    return v if k == 0 else (v + (1 << (k - 1))) >> k


def _sat(v, lo: int = X_MIN, hi: int = X_MAX):
    return np.clip(v, lo, hi)


def _delay(a: np.ndarray, d: int) -> np.ndarray:
    out = np.zeros_like(a)
    if d < a.shape[0]:
        out[d:] = a[:a.shape[0] - d]
    return out


def _running_sum(a: np.ndarray, n: int) -> np.ndarray:
    cs = np.cumsum(a, axis=0)
    out = cs.copy()
    out[n:] -= cs[:-n]
    return out


def _bit_length(m: np.ndarray) -> np.ndarray:
    """Bit length of non-negative int64 values < 2^53 (0 -> 0)."""
    _, e = np.frexp(m.astype(np.float64))
    return np.where(m > 0, e, 0).astype(np.int64)


def as_iq(x) -> np.ndarray:
    """Complex or (N, 2) input -> (N, 2) int64."""
    x = np.asarray(x)
    if np.iscomplexobj(x):
        return np.stack([x.real, x.imag], axis=1).astype(np.int64)
    return x.astype(np.int64)


# --------------------------------------------------------------------------------------- #
# Front end: DC blocker + LPF                                                              #
# --------------------------------------------------------------------------------------- #
def dc_blocker(x: np.ndarray, shift: int = 10, frac: int = DC_FRAC, return_acc: bool = False):
    """Integer DC blocker on (N, ...) int arrays, each trailing element an independent rail.

    acc(n) = acc(n-1) - rnd(acc(n-1) >> shift) + ((x(n) - x(n-1)) << frac)
    y(n)   = sat12(rnd(acc(n) >> frac))
    shift = 0 bypasses the blocker (y = x).
    """
    x = np.asarray(x, dtype=np.int64)
    if shift == 0:
        return (x.copy(), np.zeros_like(x)) if return_acc else x.copy()
    n = x.shape[0]
    d = np.diff(x, axis=0, prepend=np.zeros((1,) + x.shape[1:], np.int64)) << frac
    acc = np.zeros(x.shape[1:], np.int64)
    accs = np.empty_like(x)
    half = 1 << (shift - 1)
    for i in range(n):                      # nonlinear (rounded) recursion: sample by sample
        acc = acc - ((acc + half) >> shift) + d[i]
        accs[i] = acc
    y = _sat(_rnd_shift(accs, frac))
    return (y, accs) if return_acc else y


def lpf(y: np.ndarray, coef=H_LP_Q, frac: int = LPF_FRAC) -> np.ndarray:
    """17-tap causal FIR: z(n) = sat12(rnd(sum_k h[k] y(n-k) >> frac)), y(n<0) = 0."""
    y = np.asarray(y, dtype=np.int64)
    acc = np.zeros_like(y)
    for k, h in enumerate(coef):
        if h:
            acc += h * _delay(y, k)
    return _sat(_rnd_shift(acc, frac))


def front_end(x_dec, dc_shift: int = 10) -> np.ndarray:
    """Decimated 12-bit I/Q -> LPF output 12-bit I/Q (input_stage 1 path)."""
    return lpf(dc_blocker(as_iq(x_dec), dc_shift))


# --------------------------------------------------------------------------------------- #
# Detector                                                                                 #
# --------------------------------------------------------------------------------------- #
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


def holdoff(p: Params) -> int:
    """Effective re-arm holdoff: max(arm_holdoff, n_cfo, 1)."""
    return max(int(p.arm_holdoff), int(p.n_cfo), 1)


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
        arm = t + holdoff(p)
    return trig


# --------------------------------------------------------------------------------------- #
# CORDIC and frequency cascade                                                             #
# --------------------------------------------------------------------------------------- #
def cordic_angle(re: int, im: int, iters: int = CORDIC_ITER, in_bits: int = CORDIC_IN_BITS,
                 z_bits: int = CORDIC_Z_BITS) -> int:
    """Vectoring CORDIC: 16-bit angle of re + j im (-32768..32767 = -pi..pi).

    1. Normalise: common shift (left or right) so that max(|re|, |im|) has exactly
       in_bits - 2 bits; right shifts are arithmetic (floor). (0, 0) -> angle 0.
    2. Left half plane: x, y = -x, -y and z = half a turn.
    3. For i = 0 .. iters-1: if y >= 0: x, y, z = x + (y >> i), y - (x >> i), z + ATAN[i]
                             else:      x, y, z = x - (y >> i), y + (x >> i), z - ATAN[i]
       (ATAN[i] = round(atan(2^-i) / 2pi * 2^z_bits); shifts arithmetic)
    4. angle = rnd(z >> (z_bits - 16)), wrapped to signed 16 bits.
    """
    re, im = int(re), int(im)
    m = max(abs(re), abs(im))
    if m == 0:
        return 0
    s = m.bit_length() - (in_bits - 2)
    x, y = (re >> s, im >> s) if s > 0 else (re << -s, im << -s)
    if iters == CORDIC_ITER and z_bits == CORDIC_Z_BITS:
        atan = CORDIC_ATAN
    else:
        atan = [int(round(math.atan(2.0 ** -i) / (2 * math.pi) * (1 << z_bits))) for i in range(iters)]
    z = 0
    if x < 0:
        x, y, z = -x, -y, 1 << (z_bits - 1)
    for i in range(iters):
        if y >= 0:
            x, y, z = x + (y >> i), y - (x >> i), z + atan[i]
        else:
            x, y, z = x - (y >> i), y + (x >> i), z - atan[i]
    a = _rnd_shift(z, z_bits - ANGLE_BITS) & 0xFFFF
    return a - 0x10000 if a >= 0x8000 else a


def cfo_cascade(angles: dict[int, int]) -> int:
    """Integer cascade; returns f_hat in NCO units (LSB Fs / 2^24).

    f_lag = ang(lag) * (256 / lag); step = 2^24 / lag;
    f <- f_lag + floor((f - f_lag + step/2) / step) * step, from f = f_16.
    """
    f = angles[16] * (256 // 16)
    for lag in (32, 128, 256):
        f_lag = angles[lag] * (256 // lag)
        step = (1 << NCO_BITS) // lag
        f = f_lag + ((f - f_lag + step // 2) // step) * step
    return f


def cfo_accumulators(xi: np.ndarray, xq: np.ndarray, t: int, n_cfo: int) -> dict[int, tuple[int, int]]:
    """A_lag = s_lag * sum_{n=t+lag}^{t+N-1} x(n) conj(x(n-lag)) (exact integers)."""
    acc = {}
    for lag, sign in CASCADE:
        ii, qq = xi[t + lag:t + n_cfo], xq[t + lag:t + n_cfo]
        di, dq = xi[t:t + n_cfo - lag], xq[t:t + n_cfo - lag]
        acc[lag] = (sign * int(np.sum(ii * di + qq * dq)), sign * int(np.sum(qq * di - ii * dq)))
    return acc


# --------------------------------------------------------------------------------------- #
# Tone-structure check                                                                     #
# --------------------------------------------------------------------------------------- #
def tone_thresh_lin(tone_thresh_q8: int) -> int:
    """ToneCheck threshold (dB, Q8.8) -> linear power ratio in Q.16, as the firmware computes it:
    floor(10^(T/10) * 2^16 + 0.5), with T = tone_thresh_q8 / 256 (double precision)."""
    return int(math.floor(10.0 ** (tone_thresh_q8 / 2560.0) * (1 << TONE_LIN_FRAC) + 0.5))


def tone_check(xi: np.ndarray, xq: np.ndarray, f_hat: int, lut_bits: int = TONE_LUT_BITS,
               amp_bits: int = TONE_AMP_BITS, pwr_bits: int = TONE_PWR_BITS):
    """13 single-bin DFTs of the window xi/xq (already cut to N_CFO samples).

    Bin k at frequency f_k = (f_hat + offset_k) mod 2^24 (NCO units). Phase accumulator
    phi(0) = 0, phi(n+1) = (phi(n) + f_k) mod 2^24; address a = phi >> (24 - lut_bits);
    X_k = sum x(n) (COS[a] - j SIN[a]) = sum (xi c + xq s) + j (xq c - xi s)   (exact).
    Common shift s: largest |Re X|, |Im X| over all 13 bins fits pwr_bits signed (floor shift);
    P_k = Re'^2 + Im'^2.
    Returns (num, den, tone_powers, ref_powers) with num = 9 (sum P_tone - max P_tone),
    den = sum P_ref. ToneCheck = num / den (linear); fails when num << 16 < lin * den.
    """
    if lut_bits == TONE_LUT_BITS and amp_bits == TONE_AMP_BITS:
        cos_t, sin_t = np.array(TONE_COS, np.int64), np.array(TONE_SIN, np.int64)
    else:
        a = (1 << (amp_bits - 1)) - 1
        ang = 2 * np.pi * np.arange(1 << lut_bits) / (1 << lut_bits)
        cos_t, sin_t = np.round(a * np.cos(ang)).astype(np.int64), np.round(a * np.sin(ang)).astype(np.int64)
    n = np.arange(xi.size, dtype=np.int64)
    res = []
    for off in TONE_OFFSETS + NOISE_REF_OFFSETS:
        f = (f_hat + off) % _NCO
        addr = ((f * n) % _NCO) >> (NCO_BITS - lut_bits)
        c, s = cos_t[addr], sin_t[addr]
        res.append((int(np.sum(xi * c + xq * s)), int(np.sum(xq * c - xi * s))))
    m = max(max(abs(r), abs(i)) for r, i in res)
    sh = max(m.bit_length() - (pwr_bits - 1), 0)
    pw = [(r >> sh) ** 2 + (i >> sh) ** 2 for r, i in res]
    tones, refs = pw[:4], pw[4:]
    num = 9 * (sum(tones) - max(tones))
    den = sum(refs)
    return num, den, tones, refs


def tone_db(num: int, den: int) -> float:
    """ToneCheck in dB for reporting (firmware: double-precision log10 of the two integers)."""
    if den <= 0:
        return 127.0 if num > 0 else -128.0
    if num <= 0:
        return -128.0
    return 10.0 * math.log10(num / den)


# --------------------------------------------------------------------------------------- #
# Top level                                                                                #
# --------------------------------------------------------------------------------------- #
def detect(iq: np.ndarray, p: Params, debug: bool = False) -> Result:
    """Run the detector over 12-bit I/Q samples (N, 2), starting from reset.

    With p.input_stage == 1 the samples are the decimated stream and go through front_end()
    first; with 0 they are already the LPF output.
    """
    iq = as_iq(iq)
    if p.input_stage == 1:
        iq = front_end(iq, p.dc_shift)
    xi, xq = iq[:, 0], iq[:, 1]
    n = xi.size
    (pr, pi, r0, rd), _ = comparator_inputs(xi, xq)
    num = (pr * pr + pi * pi) << 16
    den = r0 * rd
    above = num > p.eta_hi_sq * den
    below = num < p.eta_lo_sq * den
    trig = find_triggers(above, below, p)

    if not 128 <= p.n_cfo <= 4096:
        raise ValueError(f"n_cfo {p.n_cfo} outside 128..4096 (accumulator widths; the hardware's "
                         "CORDIC sequence needs about 80 clocks between windows)")
    if not 0 <= p.tone_thresh <= TONE_THRESH_MAX_Q8:
        raise ValueError(f"tone_thresh {p.tone_thresh / 256:g} dB outside 0..24 dB")
    lin = tone_thresh_lin(p.tone_thresh) if p.tone_thresh > 0 else 0
    events, extra = [], []
    for t in trig:
        N = int(p.n_cfo)
        if t + N > n:
            events.append(EventRecord(t, 0, t + N, 0, EV_TRUNCATED))
            extra.append({"t_trig": t})
            continue
        acc = cfo_accumulators(xi, xq, t, N)
        angles = {lag: cordic_angle(*acc[lag]) for lag in acc}
        f = cfo_cascade(angles)
        tn, td, tones, refs = tone_check(xi[t:t + N], xq[t:t + N], f)
        tdb = tone_db(tn, td)
        status = EV_CFO_VALID
        if lin and (tn << TONE_LIN_FRAC) < lin * td:
            status |= EV_TONE_FAIL
        tone_q8 = int(np.clip(math.floor(tdb * 256 + 0.5), -32768, 32767))
        events.append(EventRecord(t, -f, t + N, tone_q8, status,
                                  angles[16], angles[32], angles[128], angles[256]))
        extra.append({"t_trig": t, "f_hat_hz": f * FS / _NCO, "tone_db": tdb,
                      "tone_num": tn, "tone_den": td,
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
            start = t + holdoff(p)
        armed[start:] = True
        bits[armed] |= DBG_ARMED
        bits[trig] |= DBG_TRIG
        res.bits = bits
    return res
