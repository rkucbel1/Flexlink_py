"""
pl_reference.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Bit-true model of the payload receiver (Docs/Specification_PayloadInput.md, sections 5b and 6a).
Integer everywhere, every rounding and width stated, so the RTL can follow it bit for bit. Tables:
pl_tables.npz (gen_pl_tables.py). Single Tx antenna (port 0).

Input: the payload grid of one packet, PacketLength x 841 integer bins (Yr, Yi) from the shared
FFT (cf_reference.grid_symbol: symbol 4 + s of the de-rotated stream, FFT_StartTime ramp kept),
and the payload settings from the control field (PLConfig.from_message).

 Notation: rnd(v, k) = (v + 2^(k-1)) >> k (k > 0; arithmetic shift, so halves round up),
 sat(v, b) = clip to -(2^(b-1)) .. 2^(b-1) - 1, satm(v, b) = clip to +-(2^(b-1) - 1).
 rot(H, a) = rnd(H (COS[a'] + j SIN[a']) >> 15), a' = (a >> 6) mod 1024 (the tone-check table).

  1. Scale     reference row 0 (symbol 0): Pr = pilot values of step 2 computed on Y;
               k = max(bitlen(max |Pr re|, |Pr im| over the pilots) - HP_BITS, 0)  (once per packet)
               Y' = sat(rnd(Y, k), Y_BITS) for every bin of the packet
  2. Pilots    reference rows j (symbols 0, R, 2R, ..; R = RefSymbolSpacing), Ref_P0 tones:
               BPSK Hp = s Y',  s = 1 - 2 b          (b = pilot bit, pilot +1 for b = 0)
               QAM  Hp = s Y' (1 + j), s = 2 b - 1   (pilot s * corner (A - jA), A the outer level)
               so Hp = g H with g = 1 (BPSK) or 2A / norm (QAM): a known gain, folded into step 8
  3. Window    the delay profile of reference rows 0 and 1 (NR = NR_EST = 2): per row in_i = sat(rnd(conj(Hp_i) win_i, 17), 12) (win = Hann
               table, 15 bit; the shift also brings Hp to the FFT's 12-bit input) on FFT bins
               (col - 420) mod 1024, 0 on the rest; X = fft_fixed(in) (the shared FFT);
               p(n) = sum over the NR rows of Xr^2 + Xi^2, n = 4 .. n_max - 1,
               n_max = min(1024 / S rounded down, 128) (n = delay in the hardware frame).
               thr = max(max_n p >> 10, (PDP_T NR cg W2 sumN) // (n0 << 44))  (cg = 1 BPSK, 2 QAM;
               W2 = sum win_i^2; sumN, n0 from step 7; PDP_T = 8: 8x the noise floor of p)
               last = largest n with p(n) > thr (none: 0); E = last + 4;
               class c = first with CLASS_END[c] >= E, at most n_classes(S) - 1
     SNR mode  hi = (sum over the pilots of rows 0, 1 of |Hp|^2) * n0 >= HI_RATIO * NR * Np * cg * sumN (pilot power
               incl. noise at least HI_RATIO x the pilot noise; sumN, n0 from step 7). The mode
               picks the weight set (v = 1: design noise 0.05, else 0.5) and turns the
               smoothing of step 6 off (hi: T = 1).
  4. Frequency Hf(j, k) = sat(rnd(sum_i (w_k,i Hp(j, start_k + i)) >> FIR_FRAC), H_BITS), i = 0..7,
               weights of (S, c, v) from the tables
  5. Drift     D_j = sum over pilots Hp(j + 1) conj(Hp(j)) (exact), phi_j = cordic_angle(D_j);
               cum_0 = 0, cum_j+1 = cum_j + phi_j (16-bit wrap)
               aligned rows A_j = sat(rot(Hf(j), -cum_j), H_BITS)
  6. Smoothing T = 9, 5, 3, 1 rows for R = 1, 3, 6, 12, weights h = (1 x 9), (1 2 2 2 1), (1 2 1),
               (1); centred, rows outside the packet dropped:
               Sm_j = sat(rnd(sum_o h_o A_j+o * RECIP[wsum], 15), H_BITS), RECIP[w] = round(2^15 / w)
     Time      symbol s between reference rows j and j + 1 (s0 = j R, t = s - s0, 0 <= t <= R):
               t = 0:      M = Sm_j,   theta = cum_j
               t = R:      M = Sm_j+1, theta = cum_j+1
               otherwise:  M = sat(rnd(((R - t) Sm_j + t Sm_j+1) * RS_R, 15), H_BITS),
                           theta = cum_j + rnd(phi_j * t * RS_R, 15), RS_R = round(2^15 / R)
               H(s) = sat(rot(M, theta), H_BITS)
  7. Noise     reference rows 0 and 1, tones with type Ref_P1 or in-band DC (cols 1..839):
               sumN = sum |Y'|^2, n0 = their number
  8. LLR       z = Y' conj(H) (exact), e = |H|^2 (exact), on every Data RE in grid order (row-major)
               per bit b of the RE (axis Re for the first half of the bits, Im for the rest, Gray
               levels m = +-1, +-3, .. of QAM_Mapper): D(m) = m^2 e - C1 m z_axis,
               N_b = min over m with bit 1 of D(m) - min over m with bit 0 of D(m)
               (= LLR ln P0/P1 * C2 sigma^2, C1 = 2, C2 = 1 for BPSK, C1 = 4A, C2 = 4A^2 for QAM)
               scale: st = 2 (LLR step 1/4, every mode); num = n0 << (F + st),
               den = C2 sumN; sh = smallest >= 0 with (num << sh) // den >= 2^15,
               m = min((num << sh) // den, 2^16 - 1) (den = 0: m = 2^16 - 1, sh = 0)
               Lf = satm(rnd(N_b * m, sh), LF_BITS)           (LLR in units of step / 2^F)
  9. Descramble Lf = -Lf where the Gold code (C_init of the control field, the whole sequence) is 1
 10. Combine   code word w, copy c, bit j at sequence index w (Rep+1) N + c N + j:
               acc = 0; acc = satm(acc + Lf_c, ACC_BITS) for c = 0 .. Rep (in order)
               Lq = satm(rnd(acc, F), 7)
 11. Deinterleave code[il[i]] = Lq[i]
 12. LDPC      layered normalised min-sum (ldpc_study frozen decoder): APP 10 bit, messages 7 bit,
               alpha = m - m >> 2, layers in prototype row order, max 10 iterations, a code word
               stops after the first iteration whose hard decisions (APP < 0) meet every check
 13. CRC       the first K decoded bits (systematic code): CRC24 (Gen24, zero init) of the
               K - 24 transport block bits equals the last 24

Result: PLResult(blocks = [(transport block bits, crc_ok, iterations)], window class, k, ...).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import cf_reference as C
import pd_reference as R

HERE = Path(__file__).resolve().parent
_T = np.load(HERE / "pl_tables.npz")

HP_BITS = 13              # row-0 pilot peak after the packet shift k
Y_BITS = 16               # Y' (grid after the shift)
H_BITS = 16               # Hf, A, Sm, M, H
F = 3                     # extra fraction bits of the per-copy LLR
LF_BITS = 12              # per-copy LLR (step / 2^F)
ACC_BITS = 16             # copy accumulator
LLR_BITS = 7              # decoder input
APP_BITS = 10
MSG_BITS = 7
ITERS = 10
FIR_FRAC = int(_T["FIR_FRAC"])
CLASS_END = tuple(int(v) for v in _T["CLASS_END"])
SMOOTH = {1: (1,) * 9, 3: (1, 2, 2, 2, 1), 6: (1, 2, 1), 12: (1,)}
NR_EST = 2                # reference rows 0, 1 for the delay profile, the noise and the SNR mode
PDP_T = 8                 # delay-profile noise threshold: 8 x the noise floor of the row sum
HI_RATIO = 16             # high-SNR mode: pilot power (incl. noise) >= 16 x noise (pilot SNR about 12 dB)
MODE_FORCE = None         # experiments: "lo" / "hi" overrides the SNR mode
RECIP = {w: int(round((1 << 15) / w)) for w in range(1, 10)}
RS = {r: int(round((1 << 15) / r)) for r in (1, 3, 6, 12)}
REF_P0, REF_P1, DC, DATA = 10, 20, 30, 40
BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}
OUTER = {"BPSK": 1, "QPSK": 1, "16QAM": 3, "64QAM": 7}
MODS = ("BPSK", "QPSK", "16QAM", "64QAM")
RATES = ("12", "23", "34", "56")
CRC24_POLY = 0x864CFB     # Gen24 without the leading term


# --------------------------------------------------------------------------------------------
# settings and layout

@dataclass(frozen=True)
class PLConfig:
    mod: str
    ref_sym: int          # RefSymbolSpacing
    ref_sig: int          # RefSignalSpacing
    plen: int             # PacketLength (payload OFDM symbols)
    n: int                # EncoderBlockSize (CBS)
    rate: str             # "12", "23", "34", "56"
    rep: int              # Repetition (copies = rep + 1)
    ndb: int              # NDB_A: transport blocks (code words) in the packet
    c_init: int           # payload scrambler id

    @staticmethod
    def from_message(message: int) -> "PLConfig":
        """Settings from a control-field message whose fields are valid (cf_reference)."""
        c = C.field_codes(message)
        return PLConfig(MODS[c["PayloadFieldMod"]], (1, 3, 6, 12)[c["RefSymbolSpacing"]],
                        (3, 6, 12, 24)[c["RefSignalSpacing"]], C.packet_length(message),
                        C.CF_VALID["CBS_A_Flag"][c["CBS_A_Flag"]], RATES[c["FEC_A_Flag"]],
                        C.CF_VALID["RM_A_Flag"][c["RM_A_Flag"]], c["NDB_A"], c["C_init"])

    @property
    def code(self) -> str:
        return f"{self.n}_{self.rate}"

    @property
    def k(self) -> int:                    # message bits per code word
        return self.n * {"12": 1, "23": 2, "34": 3, "56": 5}[self.rate] // {"12": 2, "23": 3, "34": 4, "56": 6}[self.rate]

    @property
    def st(self) -> int:                   # LLR step 2^-st: 1/4 in every mode (section 6b: the
        return 2                           # 64QAM rate 1/2 step 1 of section 5b loses up to 1 dB)

    def valid_grid(self) -> bool:
        return (self.plen - 1) % self.ref_sym == 0


def signal_types(cfg: PLConfig) -> np.ndarray:
    ref, dat = _T[f"ref_row_{cfg.ref_sig}"], _T["data_row"]
    return np.array([ref if s % cfg.ref_sym == 0 else dat for s in range(cfg.plen)])


def data_bits(cfg: PLConfig) -> int:
    return int((signal_types(cfg) == DATA).sum()) * BITS[cfg.mod]


def available_blocks(cfg: PLConfig) -> int:
    """Whole code words that fit (Initialize_Tx: AvailableDataBlocks)."""
    return data_bits(cfg) // (cfg.n * (cfg.rep + 1))


# --------------------------------------------------------------------------------------------
# helpers

def _rnd(v, k):
    return (v + (1 << (k - 1))) >> k if k > 0 else v


def _sat(v, b):
    return np.clip(v, -(1 << (b - 1)), (1 << (b - 1)) - 1)


def _satm(v, b):
    m = (1 << (b - 1)) - 1
    return np.clip(v, -m, m)


def _rot(hr, hi, a16):
    a = (int(a16) >> 6) & 1023
    c, s = R.TONE_COS[a], R.TONE_SIN[a]
    return _rnd(hr * c - hi * s, 15), _rnd(hr * s + hi * c, 15)


def _wrap16(a):
    a &= 0xFFFF
    return a - 0x10000 if a >= 0x8000 else a


def _levels(mod):
    """Per bit: (axis, levels with bit 0, levels with bit 1), odd-integer Gray levels of QAM_Mapper
    (802.11, first bit = MSB; the first half of the bits on Re)."""
    nb = BITS[mod]
    if mod == "BPSK":
        return [(0, (-1,), (1,))]                    # QAM_Mapper: bit 0 -> -1, 1 -> +1
    half = nb // 2
    pam = {1: {0: -1, 1: 1},
           2: {0b00: -3, 0b01: -1, 0b11: 1, 0b10: 3},
           3: {0b000: -7, 0b001: -5, 0b011: -3, 0b010: -1, 0b110: 1, 0b111: 3, 0b101: 5, 0b100: 7}}[half]
    out = []
    for k in range(nb):
        axis, kk = (0, k) if k < half else (1, k - half)
        lv0 = tuple(sorted(v for lab, v in pam.items() if not (lab >> (half - 1 - kk)) & 1))
        lv1 = tuple(sorted(v for lab, v in pam.items() if (lab >> (half - 1 - kk)) & 1))
        out.append((axis, lv0, lv1))
    return out


LEVELS = {m: _levels(m) for m in MODS}


def crc24(bits) -> int:
    crc = 0
    for b in np.asarray(bits, np.int64):
        fb = ((crc >> 23) & 1) ^ int(b)
        crc = (crc << 1) & 0xFFFFFF
        if fb:
            crc ^= CRC24_POLY
    return crc


class Code:
    """802.11n LDPC code from the prototype matrix (pl_tables): layers = prototype rows."""

    def __init__(self, t: str):
        P, Z = _T[f"P_{t}"], int(_T[f"Z_{t}"])
        self.Z, self.t = Z, t
        self.N, self.M = P.shape[1] * Z, P.shape[0] * Z
        self.K = self.N - self.M
        z = np.arange(Z)
        self.layers = [np.stack([c * Z + (z + int(P[r, c])) % Z for c in np.flatnonzero(P[r] >= 0)])
                       for r in range(P.shape[0])]

    def syndrome_ok(self, hard):
        ok = np.ones(hard.shape[0], bool)
        for idx in self.layers:
            ok &= (hard[:, idx].sum(1) % 2 == 0).all(1)
        return ok


_CODES: dict = {}


def code_of(t: str) -> Code:
    if t not in _CODES:
        _CODES[t] = Code(t)
    return _CODES[t]


def decode_ldpc(code: Code, llr_q, iters=ITERS, app_bits=APP_BITS, msg_bits=MSG_BITS):
    """Step 12 for B code words (B, N) at once. Returns (hard bits (B, N), iterations (B,))."""
    B = llr_q.shape[0]
    amax = (1 << (app_bits - 1)) - 1
    mmax = (1 << (msg_bits - 1)) - 1
    L = np.clip(np.asarray(llr_q, np.int64), -amax, amax)
    Rm = [np.zeros((B,) + idx.shape, np.int64) for idx in code.layers]
    active = np.ones(B, bool)
    used = np.zeros(B, np.int64)
    for _ in range(iters):
        a_idx = np.flatnonzero(active)
        if a_idx.size == 0:
            break
        La = L[a_idx]
        for k, idx in enumerate(code.layers):
            q = np.clip(La[:, idx] - Rm[k][a_idx], -amax, amax)
            mag = np.minimum(np.abs(q), mmax)
            s = np.where(q < 0, -1, 1)
            o = np.argsort(mag, axis=1, kind="stable")
            mn1 = np.take_along_axis(mag, o[:, :1], 1)
            mn2 = np.take_along_axis(mag, o[:, 1:2], 1)
            mn1, mn2 = mn1 - (mn1 >> 2), mn2 - (mn2 >> 2)
            r = np.prod(s, 1, keepdims=True) * s * np.where(np.arange(idx.shape[0])[None, :, None] == o[:, :1], mn2, mn1)
            La[:, idx] = np.clip(q + r, -amax, amax)
            Rm[k][a_idx] = r
        L[a_idx] = La
        used[a_idx] += 1
        ok = code.syndrome_ok((La < 0).astype(np.int64))
        active[a_idx[ok]] = False
    return (L < 0).astype(np.int64), used


# --------------------------------------------------------------------------------------------
# the receiver

@dataclass
class PLResult:
    cfg: PLConfig
    blocks: list = field(default_factory=list)   # (transport block bits, crc_ok, iterations)
    window: int = 0                              # delay class (step 3)
    hi: bool = False                             # high-SNR mode
    k: int = 0                                   # packet shift (step 1)
    sum_n: int = 0                               # noise power sum (step 7) and its tone count
    n0: int = 0
    m: int = 0                                   # LLR scale m / 2^sh (step 8)
    sh: int = 0
    phi: list = field(default_factory=list)      # drift per reference pair (16-bit)
    debug: dict = field(default_factory=dict)

    @property
    def crc_ok(self) -> list:
        return [b[1] for b in self.blocks]


def _pilots(Yr, Yi, cols, bits, mod):
    """Step 2 on rows (n_rows, 841) -> (Hp_r, Hp_i) (n_rows, Np)."""
    a, b = Yr[:, cols], Yi[:, cols]
    if mod == "BPSK":
        s = 1 - 2 * bits
        return s * a, s * b
    s = 2 * bits - 1
    return s * (a - b), s * (a + b)


def delay_class(hpr, hpi, cfg: PLConfig, sum_n: int, n0: int, debug: dict | None = None) -> int:
    """Step 3 from the pilots of the first NR_EST reference rows (hpr, hpi: (rows, Np))."""
    S = cfg.ref_sig
    cols = np.arange(S - 1, 841, S)
    bins = (cols - 420) % 1024
    win = _T[f"win_{S}"]
    n_max = min(1024 // S, 128)
    p = np.zeros(n_max - 4, np.int64)
    for j in range(hpr.shape[0]):
        xr = np.zeros(1024, np.int64)
        xi = np.zeros(1024, np.int64)
        xr[bins], xi[bins] = _sat(_rnd(hpr[j] * win, 17), 12), _sat(_rnd(-hpi[j] * win, 17), 12)
        fr, fi = C.fft_fixed(xr, xi)
        p += fr[4:n_max] ** 2 + fi[4:n_max] ** 2
    cg = 1 if cfg.mod == "BPSK" else 2
    w2 = int((win * win).sum())
    thr = max(int(p.max()) >> 10, (PDP_T * hpr.shape[0] * cg * w2 * sum_n) // max(n0 << 44, 1))
    above = np.flatnonzero(p > thr)
    last = int(above[-1]) + 4 if above.size else 0           # delay n of the last bin above
    e = last + 4
    nc = int(_T[f"n_classes_{S}"])
    c = next((i for i in range(nc) if CLASS_END[i] >= e), nc - 1)
    if debug is not None:
        debug.update(pdp=p, thr=thr, last=last)
    return c


def llr_scale(sum_n: int, n0: int, cfg: PLConfig) -> tuple[int, int]:
    """Step 8: (m, sh)."""
    c2 = 1 if cfg.mod == "BPSK" else 4 * OUTER[cfg.mod] ** 2
    num, den = n0 << (F + cfg.st), c2 * sum_n
    if den == 0:
        return (1 << 16) - 1, 0
    sh = 0
    while (num << sh) // den < (1 << 15):
        sh += 1
    return min((num << sh) // den, (1 << 16) - 1), sh


def channel(Yr, Yi, cfg: PLConfig, res: PLResult | None = None):
    """Steps 1-7: (Y'r, Y'i, Hr, Hi, sum_n, n0) on the whole grid."""
    S, Rs, mod = cfg.ref_sig, cfg.ref_sym, cfg.mod
    Yr = np.asarray(Yr, np.int64)
    Yi = np.asarray(Yi, np.int64)
    cols = np.arange(S - 1, 841, S)
    bits = _T[f"pilot_bits_{S}"]
    refs = np.arange(0, cfg.plen, Rs)
    # 1. scale
    p0r, p0i = _pilots(Yr[:1], Yi[:1], cols, bits, mod)
    mx = int(max(np.abs(p0r).max(), np.abs(p0i).max()))
    k = max(mx.bit_length() - HP_BITS, 0)
    Ypr, Ypi = _sat(_rnd(Yr, k), Y_BITS), _sat(_rnd(Yi, k), Y_BITS)
    # 2. pilots of every reference row
    hpr, hpi = _pilots(Ypr[refs], Ypi[refs], cols, bits, mod)
    # 7. noise (needed by step 3)
    t0 = _T[f"ref_row_{S}"]
    noise = (t0 == REF_P1) | ((t0 == DC) & (np.arange(841) >= 1) & (np.arange(841) <= 839))
    nr = min(NR_EST, refs.size)
    sum_n = int(sum((Ypr[refs[j], noise] ** 2 + Ypi[refs[j], noise] ** 2).sum() for j in range(nr)))
    n0 = int(noise.sum()) * nr
    # 3. delay class
    dbg = {} if res is not None else None
    c = delay_class(hpr[:nr], hpi[:nr], cfg, sum_n, n0, dbg)
    #    SNR mode
    cg = 1 if mod == "BPSK" else 2
    pw = int((hpr[:nr] ** 2 + hpi[:nr] ** 2).sum())
    hi = pw * n0 >= HI_RATIO * nr * cols.size * cg * sum_n
    if MODE_FORCE is not None:
        hi = MODE_FORCE == "hi"
    v = int(hi)
    # 4. frequency
    st, wr, wi = _T[f"fir_start_{S}_{c}"], _T[f"fir_re_{S}_{c}_{v}"], _T[f"fir_im_{S}_{c}_{v}"]
    idx = st[:, None] + np.arange(wr.shape[1])[None, :]
    a, b = hpr[:, idx], hpi[:, idx]                                     # (nref, 841, 8)
    hfr = _sat(_rnd((wr * a - wi * b).sum(2), FIR_FRAC), H_BITS)
    hfi = _sat(_rnd((wr * b + wi * a).sum(2), FIR_FRAC), H_BITS)
    # 5. drift, aligned rows
    nref = refs.size
    phi = []
    for j in range(nref - 1):
        dr = int(np.sum(hpr[j + 1] * hpr[j] + hpi[j + 1] * hpi[j]))
        di = int(np.sum(hpi[j + 1] * hpr[j] - hpr[j + 1] * hpi[j]))
        phi.append(R.cordic_angle(dr, di))
    cum = [0]
    for v in phi:
        cum.append(_wrap16(cum[-1] + v))
    Ar = np.zeros_like(hfr)
    Ai = np.zeros_like(hfi)
    for j in range(nref):
        r_, i_ = _rot(hfr[j], hfi[j], -cum[j])
        Ar[j], Ai[j] = _sat(r_, H_BITS), _sat(i_, H_BITS)
    # 6. smoothing
    h = (1,) if hi else SMOOTH[Rs]
    half = len(h) // 2
    Smr = np.zeros_like(Ar)
    Smi = np.zeros_like(Ai)
    for j in range(nref):
        accr = np.zeros(841, np.int64)
        acci = np.zeros(841, np.int64)
        w = 0
        for o, hv in enumerate(h):
            jj = j + o - half
            if 0 <= jj < nref:
                accr += hv * Ar[jj]
                acci += hv * Ai[jj]
                w += hv
        Smr[j] = _sat(_rnd(accr * RECIP[w], 15), H_BITS)
        Smi[j] = _sat(_rnd(acci * RECIP[w], 15), H_BITS)
    #    time
    Hr = np.zeros((cfg.plen, 841), np.int64)
    Hi = np.zeros((cfg.plen, 841), np.int64)
    for s in range(cfg.plen):
        j = min(s // Rs, nref - 2) if nref > 1 else 0
        t = s - j * Rs
        if nref == 1 or t == 0:
            mr, mi, th = Smr[j], Smi[j], cum[j]
        elif t == Rs:
            mr, mi, th = Smr[j + 1], Smi[j + 1], cum[j + 1]
        else:
            mr = _sat(_rnd(((Rs - t) * Smr[j] + t * Smr[j + 1]) * RS[Rs], 15), H_BITS)
            mi = _sat(_rnd(((Rs - t) * Smi[j] + t * Smi[j + 1]) * RS[Rs], 15), H_BITS)
            th = _wrap16(cum[j] + _rnd(phi[j] * t * RS[Rs], 15))
        r_, i_ = _rot(mr, mi, th)
        Hr[s], Hi[s] = _sat(r_, H_BITS), _sat(i_, H_BITS)
    if res is not None:
        res.window, res.k, res.sum_n, res.n0, res.phi, res.hi = c, k, sum_n, n0, phi, hi
        res.debug.update(dbg, hf=(hfr, hfi), sm=(Smr, Smi), cum=cum)
    return Ypr, Ypi, Hr, Hi, sum_n, n0


def llr_numerators(zr, zi, e, mod):
    """Step 8 numerators N (n_RE, nb)."""
    a = OUTER[mod]
    c1 = 2 if mod == "BPSK" else 4 * a
    out = []
    for axis, lv0, lv1 in LEVELS[mod]:
        z = zr if axis == 0 else zi
        d0 = np.min(np.stack([m * m * e - c1 * m * z for m in lv0]), 0)
        d1 = np.min(np.stack([m * m * e - c1 * m * z for m in lv1]), 0)
        out.append(d1 - d0)
    return np.stack(out, 1)


def soft_bits(Yr, Yi, cfg: PLConfig, res: PLResult | None = None):
    """Steps 1-8: per-copy LLRs Lf of the whole (scrambled) sequence, in sequence order."""
    Ypr, Ypi, Hr, Hi, sum_n, n0 = channel(Yr, Yi, cfg, res)
    mask = signal_types(cfg) == DATA
    yr, yi, hr, hi = Ypr[mask], Ypi[mask], Hr[mask], Hi[mask]
    zr = yr * hr + yi * hi
    zi = yi * hr - yr * hi
    e = hr * hr + hi * hi
    N = llr_numerators(zr, zi, e, cfg.mod).ravel()
    m, sh = llr_scale(sum_n, n0, cfg)
    if res is not None:
        res.m, res.sh = m, sh
    return _satm(_rnd(N * m, sh), LF_BITS)


def decode_soft(lf, cfg: PLConfig, res: PLResult):
    """Steps 9-13 on the per-copy LLRs."""
    nbits = lf.size
    g = C.gold(cfg.c_init, nbits)
    lf = np.where(g == 1, -lf, lf)
    N, cw = cfg.n, cfg.n * (cfg.rep + 1)
    nblk = min(cfg.ndb, nbits // cw)
    il = _T[f"il_{N}"]
    code = code_of(cfg.code)
    llr = np.zeros((nblk, N), np.int64)
    amax = (1 << (ACC_BITS - 1)) - 1
    for w in range(nblk):
        acc = np.zeros(N, np.int64)
        for c in range(cfg.rep + 1):
            acc = np.clip(acc + lf[w * cw + c * N: w * cw + (c + 1) * N], -amax, amax)
        q = _satm(_rnd(acc, F), LLR_BITS)
        llr[w, il] = q                                   # de-interleave: code[il[i]] = q[i]
    res.debug["llr_q"] = llr
    if nblk:
        hard, used = decode_ldpc(code, llr)
    else:
        hard, used = np.zeros((0, N), np.int64), np.zeros(0, np.int64)
    K = code.K
    for w in range(nblk):
        msg = hard[w, :K]
        ok = crc24(msg[:K - 24]) == sum(int(b) << (23 - i) for i, b in enumerate(msg[K - 24:]))
        res.blocks.append((msg[:K - 24].copy(), bool(ok), int(used[w])))
    return res


def receive_grid(Yr, Yi, cfg: PLConfig) -> PLResult:
    """The whole payload chain on one packet's grid (PacketLength x 841 integer bins)."""
    res = PLResult(cfg)
    lf = soft_bits(Yr, Yi, cfg, res)
    return decode_soft(lf, cfg, res)


# --------------------------------------------------------------------------------------------
# from the detector stream (pd_reference.detect with rearm)

def grid_from_stream(y: np.ndarray, n_end: int, cfo_inc: int, plen: int):
    """(Yr, Yi) of the payload symbols after a control record (cf_reference.payload_records), or
    None if the stream ends before the last symbol."""
    n0 = n_end + 1
    m = (C.CF_SYMS + plen) * C.CF_SYM
    if n0 + m > y.shape[0]:
        return None
    xr, xi = R.derotate(y[n0:n0 + m, 0], y[n0:n0 + m, 1], cfo_inc & 0xFFFFFF, C.X_BITS_CF)
    Yr = np.zeros((plen, 841), np.int64)
    Yi = np.zeros((plen, 841), np.int64)
    for s in range(plen):
        Yr[s], Yi[s] = C.grid_symbol(xr, xi, C.CF_SYMS + s)
    return Yr, Yi


def payload_results(y: np.ndarray, events, timing, control) -> list:
    """PLResult for every accepted packet (control record CRC ok, every field valid, valid grid)."""
    inc = {e.t_trig: e.cfo_inc for e in events}
    out = []
    for c in control:
        if not (c.crc_ok and C.fields_valid(c.message)):
            continue
        cfg = PLConfig.from_message(c.message)
        if not cfg.valid_grid():
            continue
        g = grid_from_stream(y, c.n_end, inc[c.t_trig], cfg.plen)
        if g is None:
            break
        r = receive_grid(*g, cfg)
        r.debug["t_trig"] = c.t_trig
        out.append(r)
    return out


# --------------------------------------------------------------------------------------------
# Channel-estimate records (step 12b of Docs/Specification_PayloadReceiver.md): what the board
# reports before the demapper and the decoder exist. For every accepted packet: one packet record
# (sym = CH_PKT; sig_a = k | class << 8 | hi << 12 | n0 << 16, sig_b = sumN mod 2^32), then per
# payload symbol s the signature (cf_reference.grid_signature) of its channel estimate H(s), all
# 841 tones, 16 bit each. H(s) is out once the smoothed rows j and j + 1 exist
# (j = min(s // R, nref - 2)), i.e. once reference row min(nref - 1, j + 1 + T // 2) is in (T = 1 in
# the high-SNR mode); the packet record once rows 0 and 1 are in. Symbols are in the stream when
# their FFT window is (cf_reference.payload_records).

CH_PKT = 0x8000


def chest_needs_row(cfg: PLConfig, s: int, hi: bool) -> int:
    """Last reference row that H(s) waits for."""
    nref = (cfg.plen - 1) // cfg.ref_sym + 1
    half = 0 if hi else len(SMOOTH[cfg.ref_sym]) // 2
    j = min(s // cfg.ref_sym, nref - 2)
    return min(nref - 1, j + 1 + half)


def chest_records(y: np.ndarray, events, timing, control) -> list:
    """Channel-estimate records of every accepted packet (control record CRC ok, every field
    valid), in order; only what the stream y allows (see above)."""
    from pd_protocol import PayloadRecord
    inc = {e.t_trig: e.cfo_inc for e in events}
    out = []
    for c in control:
        if not (c.crc_ok and C.fields_valid(c.message)):
            continue
        cfg = PLConfig.from_message(c.message)
        n0s = c.n_end + 1
        m = min(y.shape[0], n0s + (C.CF_SYMS + cfg.plen) * C.CF_SYM) - n0s
        xr, xi = R.derotate(y[n0s:n0s + m, 0], y[n0s:n0s + m, 1], inc[c.t_trig] & 0xFFFFFF, C.X_BITS_CF)
        Yr = np.zeros((cfg.plen, 841), np.int64)
        Yi = np.zeros((cfg.plen, 841), np.int64)
        n_av = 0
        for s in range(cfg.plen):
            if (C.CF_SYMS + s) * C.CF_SYM + C.CF_CP + C.FFT_START + C.FFT_N > m:
                break
            Yr[s], Yi[s] = C.grid_symbol(xr, xi, C.CF_SYMS + s)
            n_av = s + 1
        if n_av <= cfg.ref_sym:
            continue                                   # rows 0 and 1 not both in
        res = PLResult(cfg)
        _, _, Hr, Hi, sum_n, n0 = channel(Yr, Yi, cfg, res)
        out.append(PayloadRecord(c.t_trig, CH_PKT, res.k | (res.window << 8) | (int(res.hi) << 12) | (n0 << 16),
                                 sum_n & 0xFFFFFFFF))
        for s in range(cfg.plen):
            if chest_needs_row(cfg, s, res.hi) * cfg.ref_sym >= n_av:
                break
            a, b = C.grid_signature(Hr[s], Hi[s])
            out.append(PayloadRecord(c.t_trig, s, a, b))
    return out
