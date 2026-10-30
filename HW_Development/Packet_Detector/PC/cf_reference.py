"""
cf_reference.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Bit-true model of the control-field receiver (Docs/Specification_ControlField.md). Integer
everywhere, every rounding and width stated, so the RTL can follow it bit for bit. Input: the
DC blocker output y of the detector (pd_reference.dc_blocker, 12-bit, 20 Msps), and per packet
the good timing record (n_end) and its event's cfo_inc.

  1. De-rotator   phase 0 at n0 = n_end + 1, step cfo_inc (24 bit), tone-check table,
                  x = sat12(rnd((y * (c + j s)) >> 15)), 4560 samples
  2. FFT          symbol s window x(n0 + 1140 s + 104 .. + 1023) (CP end - 12),
                  1024-point radix-2 decimation in time, in place, bit-reversed input:
                    input  a = x << FFT_IN_SHIFT                       (FFT_W-bit signed)
                    stage  t = rnd((b * w) >> TW_FRAC) per rail, w = W^k (TW_FRAC + 1 bits)
                           a' = rnd((a + t) >> 1), b' = rnd((a - t) >> 1)
                  output X(m) = FFT(x)(m) << FFT_IN_SHIFT >> 10 (plus rounding); grid column
                  c = 0..840 <- bin (c - 420) mod 1024
  3. Pilots       Hp(s) = Y(s, P0 tones) * ref, ref = +-1, s = 0, 3     (280 each)
  4. Frequency    Hf(k) = rnd(sum_i w_k,i Hp(start_k + i) >> CF_FIR_FRAC), i = 0..7 (cf_tables)
  5. Drift        D = sum Hp(3) conj(Hp(0)) (exact); phi = cordic_angle(D) (16-bit angle);
                  d = rnd(phi * 21845 >> 16) (phi / 3)
                  rot(H, a) = rnd(H (COS[a'] + j SIN[a']) >> 15), a' = a >> 6 (10-bit table address)
                  H3' = rot(Hf3, -phi); for s = 0..3:
                  H(s) = rot(rnd(((3 - s) Hf0 + s H3') * 10923 >> 15), s d)   (10923 = 2^15 / 3)
  6. Soft         v = Re(Y conj(H)) = Yr Hr + Yi Hi (exact) on the 2200 data tones, in order
  7. Decode       descramble (negate where gold(130) = 1); A[j] = sum of v over the 15 copies
                  (i < 2100, j = i mod 140); code[IDX[j]] = A[j]; shift sh = max(bitlen(max |A|)
                  - (VIT_BITS - 1), 0); q = code >> sh (floor)
  8. Viterbi      K = 7 [133, 171], maximum correlation, branch metric = sum of +-q (+ for an
                  output bit 1), start and end in state 0, ties choose the predecessor with input
                  bit 0 of the older stage (as Convolutionalencoder.ViterbiDecoder)
  9. CRC10        over the 64 decoded bits (Gen10, zero init); message = first 54 bits

Result: CFRecord(t_trig, n_end, crc_ok, message (54 bits as int, bit 0 = first), sh, metric).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import cf_tables as T
import pd_reference as R

CF_SYMS = 4
CF_SYM = 1140
CF_CP = 116
CF_LEN = CF_SYMS * CF_SYM                    # 4560 samples at 20 Msps
FFT_N = 1024
FFT_START = -12                              # window starts this many samples before the CP end
FFT_W = 18
FFT_IN_SHIFT = 5
TW_FRAC = 17
VIT_BITS = 5
X_BITS_CF = 12                               # de-rotated samples
CF_MSG_BITS = 54
CRC10_POLY = 0b1111011001                    # Gen10 without the leading term
GOLD_C_INIT = 130

# grid layout (Construct_Resource_Grid: 4 symbols Ref Data Data Ref, profile 3, RefSignalSpacing 3)
P0, P1, DC, DATA = 10, 20, 30, 40
_REF_DC = (0, 414, 417, 420, 423, 426, 840)


def _layout():
    ref = np.full(841, DATA, np.int8)
    p1 = np.arange(1, 840, 3)
    ref[p1] = P1
    ref[p1 + 1] = P0
    ref[list(_REF_DC)] = DC
    dat = np.full(841, DATA, np.int8)
    dat[414:429] = DC
    return np.array([ref, dat, dat, ref])


CF_TYPES = _layout()
CF_P0_K = np.flatnonzero(CF_TYPES[0] == P0)                   # 280 columns
CF_DATA_MASK = CF_TYPES == DATA                               # 2200 data tones
CF_REF = np.array([-1 if (T.CF_REF_NEG >> j) & 1 else 1 for j in range(280)], np.int64)
CF_IDX = np.array(T.CF_IDX, np.int64)
CF_FIR_START = np.array(T.CF_FIR_START, np.int64)
CF_FIR_RE = np.array(T.CF_FIR_RE, np.int64)
CF_FIR_IM = np.array(T.CF_FIR_IM, np.int64)
assert CF_DATA_MASK.sum() == 2200 and CF_P0_K.size == 280


def gold(c_init: int = GOLD_C_INIT, n: int = 2200, shift: int = 1600) -> np.ndarray:
    """LTE Gold code (Scrambler_Gold_Code_Generator.py)."""
    r1, r2 = 1, c_init & 0x7FFFFFFF
    out = np.zeros(n + shift, np.int64)
    for i in range(n + shift):
        out[i] = (r1 ^ r2) & 1
        r1 = (r1 >> 1) | (((r1 ^ (r1 >> 3)) & 1) << 30)
        r2 = (r2 >> 1) | (((r2 ^ (r2 >> 1) ^ (r2 >> 2) ^ (r2 >> 3)) & 1) << 30)
    return out[shift:]


CF_GOLD = gold()


def _rnd(v, k):
    return (v + (1 << (k - 1))) >> k if k > 0 else v


def _sat(v, bits):
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return np.clip(v, lo, hi)


def _bitrev(n_bits):
    i = np.arange(1 << n_bits)
    r = np.zeros_like(i)
    for b in range(n_bits):
        r |= ((i >> b) & 1) << (n_bits - 1 - b)
    return r


_BITREV = _bitrev(10)


def twiddles(frac: int = TW_FRAC):
    """W^k = exp(-j 2 pi k / 1024), k = 0..511, rounded to frac fractional bits, capped at
    2^frac - 1 (so they fit frac + 1 bits signed)."""
    k = np.arange(FFT_N // 2)
    cap = (1 << frac) - 1
    wr = np.clip(np.round(np.cos(2 * np.pi * k / FFT_N) * (1 << frac)), -cap, cap).astype(np.int64)
    wi = np.clip(np.round(-np.sin(2 * np.pi * k / FFT_N) * (1 << frac)), -cap, cap).astype(np.int64)
    return wr, wi


TW_RE, TW_IM = twiddles()


def fft_fixed(xr, xi, w=FFT_W, in_shift=FFT_IN_SHIFT, tw_frac=TW_FRAC):
    """Fixed-point radix-2 DIT FFT of 1024 integer samples (see the module text)."""
    wr, wi = (TW_RE, TW_IM) if tw_frac == TW_FRAC else twiddles(tw_frac)
    ar = (np.asarray(xr, np.int64) << in_shift)[_BITREV]
    ai = (np.asarray(xi, np.int64) << in_shift)[_BITREV]
    half = 1
    while half < FFT_N:
        step = FFT_N // (2 * half)
        ar = ar.reshape(-1, 2 * half)
        ai = ai.reshape(-1, 2 * half)
        k = np.arange(half) * step
        br, bi = ar[:, half:], ai[:, half:]
        tr = _rnd(br * wr[k] - bi * wi[k], tw_frac)
        ti = _rnd(br * wi[k] + bi * wr[k], tw_frac)
        a0r, a0i = ar[:, :half], ai[:, :half]
        nr = np.concatenate([_rnd(a0r + tr, 1), _rnd(a0r - tr, 1)], axis=1)
        ni = np.concatenate([_rnd(a0i + ti, 1), _rnd(a0i - ti, 1)], axis=1)
        ar, ai = _sat(nr, w).ravel(), _sat(ni, w).ravel()
        half *= 2
    return ar, ai


def demodulate(xr, xi, n0: int, **kw):
    """(4, 841) integer grid from de-rotated samples x (n0 = first control-field sample)."""
    cols = (np.arange(841) - 420) % FFT_N
    Yr = np.zeros((4, 841), np.int64)
    Yi = np.zeros((4, 841), np.int64)
    for s in range(CF_SYMS):
        a = n0 + s * CF_SYM + CF_CP + FFT_START
        fr, fi = fft_fixed(xr[a:a + FFT_N], xi[a:a + FFT_N], **kw)
        Yr[s], Yi[s] = fr[cols], fi[cols]
    return Yr, Yi


def _rot(hr, hi, a16):
    """Rotate by the 16-bit angle a16 with the tone-check table (10-bit address)."""
    a = (int(a16) >> 6) & 1023
    c, s = R.TONE_COS[a], R.TONE_SIN[a]
    return _rnd(hr * c - hi * s, 15), _rnd(hr * s + hi * c, 15)


def channel_parts(Yr, Yi, fir_frac: int | None = None):
    """Frequency filter outputs Hf0 and H3' = rot(Hf3, -phi) per tone, phi and d = phi / 3."""
    pr = np.stack([Yr[0, CF_P0_K], Yr[3, CF_P0_K]]) * CF_REF
    pi = np.stack([Yi[0, CF_P0_K], Yi[3, CF_P0_K]]) * CF_REF
    cre, cim = CF_FIR_RE, CF_FIR_IM
    frac = T.CF_FIR_FRAC
    if fir_frac is not None and fir_frac < frac:
        cre, cim, frac = _rnd(cre, frac - fir_frac), _rnd(cim, frac - fir_frac), fir_frac
    idx = CF_FIR_START[:, None] + np.arange(T.CF_FIR_TAPS)[None, :]
    hf_r, hf_i = [], []
    for s in range(2):
        a, b = pr[s][idx], pi[s][idx]                        # (841, 8)
        hf_r.append(_rnd((cre * a - cim * b).sum(1), frac))
        hf_i.append(_rnd((cre * b + cim * a).sum(1), frac))
    dr = int(np.sum(pr[1] * pr[0] + pi[1] * pi[0]))         # sum Hp3 conj(Hp0)
    di = int(np.sum(pi[1] * pr[0] - pr[1] * pi[0]))
    phi = R.cordic_angle(dr, di)
    d = _rnd(phi * 21845, 16)
    h3r, h3i = _rot(hf_r[1], hf_i[1], -phi)
    return hf_r[0], hf_i[0], h3r, h3i, phi, d


def channel(Yr, Yi, fir_frac: int | None = None):
    """(4, 841) channel estimate (Hr, Hi) and the drift angle phi (16-bit)."""
    h0r, h0i, h3r, h3i, phi, d = channel_parts(Yr, Yi, fir_frac)
    Hr = np.zeros((4, 841), np.int64)
    Hi = np.zeros((4, 841), np.int64)
    for s in range(4):
        mr = _rnd(((3 - s) * h0r + s * h3r) * 10923, 15)
        mi = _rnd(((3 - s) * h0i + s * h3i) * 10923, 15)
        Hr[s], Hi[s] = _rot(mr, mi, s * d)
    return Hr, Hi, phi


def crc10(bits) -> int:
    crc = 0
    for b in bits:
        fb = ((crc >> 9) & 1) ^ int(b)
        crc = (crc << 1) & 0x3FF
        if fb:
            crc ^= CRC10_POLY
    return crc


# trellis (Convolutionalencoder: state = last 6 inputs, newest in the MSB)
_G = (0o133, 0o171)


def _parity(v):
    return bin(v).count("1") & 1


_OUT = [[(_parity(((b << 6) | s) & _G[0]), _parity(((b << 6) | s) & _G[1])) for b in (0, 1)] for s in range(64)]


def viterbi(q) -> tuple[list[int], int]:
    """q: 140 integer soft values (code bits A0 B0 A1 B1 ...; positive = bit 1).
    Returns the 70 decoded bits and the final metric of state 0."""
    NEG = -(1 << 40)
    m = [0] + [NEG] * 63
    dec_tab = []
    for t in range(70):
        sa, sb = int(q[2 * t]), int(q[2 * t + 1])
        new, ch = [0] * 64, [0] * 64
        for ns in range(64):
            b = ns >> 5
            best = None
            for x in (0, 1):
                p = ((ns & 31) << 1) | x
                oa, ob = _OUT[p][b]
                c = m[p] + (sa if oa else -sa) + (sb if ob else -sb)
                if best is None or c > best:
                    best, bx = c, x
            new[ns], ch[ns] = best, bx
        m = new
        dec_tab.append(ch)
    state, bits = 0, [0] * 70
    for t in range(69, -1, -1):
        bits[t] = state >> 5
        state = ((state & 31) << 1) | dec_tab[t][state]
    return bits, m[0]


@dataclass
class CFRecord:
    t_trig: int
    n_end: int
    crc_ok: int
    message: int          # 54 bits, bit i = message bit i
    shift: int            # Viterbi input scaling (sh)
    metric: int           # final path metric of state 0

    def message_bits(self) -> np.ndarray:
        return np.array([(self.message >> i) & 1 for i in range(CF_MSG_BITS)], np.int64)


def viterbi_input(v, vit_bits: int = VIT_BITS):
    """2200 integer soft values -> (descrambled v, Viterbi inputs q (140), shift)."""
    v = np.where(CF_GOLD == 1, -v, v)
    A = v[:2100].reshape(15, 140).sum(0)
    code = np.empty(140, np.int64)
    code[CF_IDX] = A                                      # de-interleave: code bit IDX[j] = A[j]
    m = int(np.abs(code).max())
    sh = max(m.bit_length() - (vit_bits - 1), 0)
    return v, code >> sh, sh


def decode_soft(v, vit_bits: int = VIT_BITS):
    """2200 integer soft values -> (bits70, crc_ok, shift, metric)."""
    _, q, sh = viterbi_input(v, vit_bits)
    bits, metric = viterbi(q)
    ok = crc10(bits[:64]) == 0
    return bits, ok, sh, metric


def receive(y: np.ndarray, t_trig: int, n_end: int, cfo_inc: int, vit_bits: int = VIT_BITS,
            fir_frac: int | None = None, **fft_kw) -> CFRecord | None:
    """Control field after a good timing record; None if y ends before it."""
    n0 = n_end + 1
    if n0 + CF_LEN > y.shape[0]:
        return None
    xr, xi = R.derotate(y[n0:n0 + CF_LEN, 0], y[n0:n0 + CF_LEN, 1], cfo_inc & 0xFFFFFF, X_BITS_CF)
    Yr, Yi = demodulate(xr, xi, 0, **fft_kw)
    Hr, Hi, _ = channel(Yr, Yi, fir_frac)
    v = (Yr * Hr + Yi * Hi)[CF_DATA_MASK]
    bits, ok, sh, metric = decode_soft(v, vit_bits)
    msg = sum(int(b) << i for i, b in enumerate(bits[:CF_MSG_BITS]))
    return CFRecord(t_trig, n_end, int(ok), msg, sh, metric)


def receive_debug(y_cf: np.ndarray, cfo_inc: int) -> dict:
    """Every intermediate of receive() for the CF_LEN samples y_cf (the RTL testbench)."""
    xr, xi = R.derotate(y_cf[:, 0], y_cf[:, 1], cfo_inc & 0xFFFFFF, X_BITS_CF)
    Yr, Yi = demodulate(xr, xi, 0)
    h0r, h0i, h3r, h3i, phi, d = channel_parts(Yr, Yi)
    Hr, Hi, _ = channel(Yr, Yi)
    v = (Yr * Hr + Yi * Hi)[CF_DATA_MASK]
    vd, q, sh = viterbi_input(v)
    bits, metric = viterbi(q)
    return dict(Yr=Yr, Yi=Yi, hf=(h0r, h0i, h3r, h3i), phi=phi, d=d, v=vd, q=q, shift=sh,
                bits=bits, metric=metric, crc_ok=int(crc10(bits[:64]) == 0))


# Control-field layout (Tx/ControlSignals/Parameters_Control_Field.json "ControlField", MSB first)
# and the codes that mean something (Initialize_Control_Field.CONTROL_FIELD_CODING); fields without
# a table (NDB_A, Tx_Ref_Clk_Cnt, C_init, Reserved) take any value.
CF_FIELDS = (("RefSymbolSpacing", 2), ("RefSignalSpacing", 2), ("PacketLength", 3), ("PayloadFieldMod", 2),
             ("NoOfTxAntenna", 1), ("CBS_A_Flag", 2), ("FEC_A_Flag", 2), ("RM_A_Flag", 4), ("NDB_A", 10),
             ("ClientFlag", 1), ("Tx_Ref_Clk_Cnt", 14), ("C_init", 8), ("Reserved", 3))
CF_VALID = {"PacketLength": {0b001: 4, 0b011: 37, 0b111: 49, 0b100: 73, 0b110: 97, 0b101: 121},
            "CBS_A_Flag": {0b00: 648, 0b01: 1296, 0b10: 1944},
            "RM_A_Flag": {0b0000: 1, 0b0001: 3, 0b0010: 5, 0b0011: 7, 0b0100: 11, 0b0101: 15, 0b0110: 19,
                          0b0111: 23, 0b1000: 31}}


def field_codes(message: int) -> dict[str, int]:
    """54-bit message (bit i = message bit i) -> {field: code}."""
    out, pos = {}, 0
    for name, w in CF_FIELDS:
        code = 0
        for i in range(w):
            code = (code << 1) | ((message >> (pos + i)) & 1)
        out[name] = code
        pos += w
    return out


def fields_valid(message: int) -> bool:
    c = field_codes(message)
    return all(c[k] in v for k, v in CF_VALID.items())


def packet_length(message: int) -> int:
    """Payload OFDM symbols (only for a message with valid fields)."""
    return CF_VALID["PacketLength"][field_codes(message)["PacketLength"]]


def control_records(y: np.ndarray, events, timing) -> list:
    """The control records of the detector (RTL/pd_cf.v): one per timing record with status ==
    TM_GOOD, in record order, with the event's cfo_inc; none if y ends before the control field
    does (the hardware then still waits for samples)."""
    from pd_protocol import CF_CRC_OK, TM_GOOD, ControlRecord
    inc = {e.t_trig: e.cfo_inc for e in events}
    out = []
    for tm in timing:
        if tm.status != TM_GOOD:
            continue
        r = receive(y, tm.t_trig, tm.n_end, inc[tm.t_trig])
        if r is None:
            break                                       # later ones end later still
        out.append(ControlRecord(tm.t_trig, tm.n_end, CF_CRC_OK if r.crc_ok else 0, r.shift, r.metric,
                                 r.message))
    return out


# ---------------------------------------------------------------------------------------------
# Speculative payload FFT (RTL/cf_rx.v, Docs/Specification_PayloadInput.md section 5): after the
# control field the same de-rotator and FFT carry on with the payload symbols, phase continuous
# from n_end + 1. Payload symbol s (0..PacketLength - 1) is symbol 4 + s of the stream, window
# 1140 (4 + s) + 104 .. + 1127; its 841 bins are reduced to a 64-bit signature so the board can
# be checked without the rest of the payload chain. Kept only for a control record with CRC ok
# and every field code valid, and only with Params.rearm (packets cannot overlap then).

PL_SPEC = 8                                     # payload symbols the front end runs ahead of the
#                                                 control-field decision (no effect on results)


def grid_symbol(xr, xi, s: int):
    """841 bins (Yr, Yi) of symbol s of the de-rotated stream x (x[0] = sample n_end + 1)."""
    a = s * CF_SYM + CF_CP + FFT_START
    fr, fi = fft_fixed(xr[a:a + FFT_N], xi[a:a + FFT_N])
    cols = (np.arange(841) - 420) % FFT_N
    return fr[cols], fi[cols]


def grid_signature(Yr, Yi) -> tuple[int, int]:
    """Fletcher-style signature of the bins: x = re0, im0, re1, im1, ... (each mod 2^18),
    A = sum x, B = sum of the running sums A_k, both mod 2^32."""
    x = np.empty(2 * len(Yr), np.int64)
    x[0::2] = np.asarray(Yr, np.int64) & 0x3FFFF
    x[1::2] = np.asarray(Yi, np.int64) & 0x3FFFF
    n = x.size
    a = int(x.sum()) & 0xFFFFFFFF
    b = int((x * (n - np.arange(n))).sum()) & 0xFFFFFFFF
    return a, b


def payload_records(y: np.ndarray, events, timing, control) -> list:
    """PayloadRecord per payload symbol of every accepted packet (control record CRC ok, every
    field valid), in order; a symbol is there only if its FFT window lies inside y."""
    from pd_protocol import PayloadRecord
    inc = {e.t_trig: e.cfo_inc for e in events}
    out = []
    for c in control:
        if not (c.crc_ok and fields_valid(c.message)):
            continue
        n0 = c.n_end + 1
        L = packet_length(c.message)
        m = min(y.shape[0], n0 + (CF_SYMS + L) * CF_SYM) - n0
        xr, xi = R.derotate(y[n0:n0 + m, 0], y[n0:n0 + m, 1], inc[c.t_trig] & 0xFFFFFF, X_BITS_CF)
        for s in range(L):
            if (CF_SYMS + s) * CF_SYM + CF_CP + FFT_START + FFT_N > m:
                break
            a, b = grid_signature(*grid_symbol(xr, xi, CF_SYMS + s))
            out.append(PayloadRecord(c.t_trig, s, a, b))
    return out
