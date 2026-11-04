#!/usr/bin/env python3
"""
ptx_reference.py  (Flexlink_py/HW_Development/Packet_Transmitter/PC)

Bit-true model of the Flexlink packet transmitter for the ADALM-Pluto PL (Zynq-7010). The RTL in
../RTL must give the same integers at every point listed below; ptx_check.py compares this model
with the simulator (Tx/), and pushes its output through the Zedboard receiver's bit-true model.

Packet at 40 Msps, 12-bit I/Q (the AD9363 DAC; the PL drives dac_data = {sample, 4'b0}):

    [ AGC 200 | Preamble A 10000 | Preamble B 2280 | control field 4 x 2280 | payload PacketLength x 2280 ]

Front end (bits)                                        model function
    transport block (TransportBlockSize bits)
    + CRC24 (Gen24)                         -> K        crc24()
    LDPC 802.11n, systematic, from the       -> N        ldpc_encode()  (H-based, prototype matrices;
      prototype matrices (no G matrix)                    equals the simulator's G encoding)
    interleave out[i] = cw[il[i]]            -> N        payload_bits()
    Repetition + 1 copies, next block, ...
    ZeroPadding zeros, Gold scrambler C_init_Dat
Mapper (one OFDM symbol at a time, k = 0 .. 840)
    Data RE: QAM_Mapper levels (odd integers, first bits on I, MSB first)      grid_levels()
    Ref RE : Ref_P<port> pilot bit -> corner (BPSK 0 -> +1, 1 -> -1;
             QAM 0 -> (-L, +L), 1 -> (+L, -L), L = 1, 3, 7)
    DC / other port's Ref: 0
    IFFT input X = level * AMP[mod], AMP = round(2^13 / sqrt(norm)), on bins 0..420 <- k 420..840,
    bins 1628..2047 <- k 0..419, others 0                                         ifft_input()
Back end
    2048-point radix-2^2 SDF IFFT, decimation in frequency, 18-bit data, 18-bit twiddles (Q16),
    1/2 with rounding after stages SHIFTS, saturation after every stage            ifft_fixed()
    cyclic prefix 232 (last 232 samples first)
    output stage: y = sat12(rnd(x * gain, 16)), one gain per segment               output_stage()
Preamble: ROMs (ptx_tables.npz) through the same output stage with gain_pre.
Control field: the 2200 scrambled bits (cf_bits(), from the 54-bit message) are mapped to BPSK
with the Ref_P0 pilots of RefSignalSpacing 3 and go through the same IFFT, with gain_cf.

The three gains make the average power of preamble, control field and payload equal, as
Construct_Payload_Waveform.py does (it measures the payload power; the hardware uses the expected
power for the settings, gains()).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, replace
from fractions import Fraction
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
_T = np.load(HERE / "ptx_tables.npz")

N_FFT = 2048
CP = 232
SYM = N_FFT + CP
W = 18                                   # IFFT data width
TW_FRAC = int(_T["TW_FRAC"])             # 16
A_UNIT = 1 << 13                         # IFFT input amplitude of a unit-power constellation point
SHIFTS = (0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1)   # 1/2 after these of the 11 stages (total 1/32)
OUT_BITS = 12
GAIN_FRAC = 16
GAIN_BITS = 16                           # unsigned gain register
FULL_SCALE = (1 << (OUT_BITS - 1)) - 1   # 2047
BACKOFF_DB = 13.0                        # default rms below full scale (complex rms); the control
#                                          field peaks 12.3 dB above its rms for the default settings
PRE_A_LEN, PRE_B_LEN, AGC_LEN = 10000, 2280, 200
PRE_LEN = AGC_LEN + PRE_A_LEN + PRE_B_LEN          # 12480
CF_SYMS = 4
CF_LEN = CF_SYMS * SYM                              # 9120

P0, P1, DC, DATA = 10, 20, 30, 40
BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}
NORM = {"BPSK": 1, "QPSK": 2, "16QAM": 10, "64QAM": 42}
OUTER = {"BPSK": 1, "QPSK": 1, "16QAM": 3, "64QAM": 7}
AMP = {m: int(np.round(A_UNIT / np.sqrt(NORM[m]))) for m in BITS}
RATES = {"1/2": "12", "2/3": "23", "3/4": "34", "5/6": "56"}
RATE_FRAC = {"12": Fraction(1, 2), "23": Fraction(2, 3), "34": Fraction(3, 4), "56": Fraction(5, 6)}
CRC24_POLY = 0x864CFB                    # Gen24 without the leading term
CRC10_POLY = 0b1111011001                # Gen10 [1 1 1 1 1 0 1 1 0 0 1] without the leading term
CF_C_INIT = 130
REPETITIONS = (1, 3, 5, 7, 11, 15, 19, 23, 31)
PACKET_LENGTHS = (37, 49, 73, 97, 121)

# 802.11 Gray PAM per axis: bit label (MSB first) -> level
_PAM = {1: {0: -1, 1: 1},
        2: {0b00: -3, 0b01: -1, 0b11: 1, 0b10: 3},
        3: {0b000: -7, 0b001: -5, 0b011: -3, 0b010: -1, 0b110: 1, 0b111: 3, 0b101: 5, 0b100: 7}}

# control field message layout (Tx/ControlSignals/Parameters_Control_Field.json, MSB first)
CF_FIELDS = (("RefSymbolSpacing", 2), ("RefSignalSpacing", 2), ("PacketLength", 3), ("PayloadFieldMod", 2),
             ("NoOfTxAntenna", 1), ("CBS_A_Flag", 2), ("FEC_A_Flag", 2), ("RM_A_Flag", 4), ("NDB_A", 10),
             ("ClientFlag", 1), ("Tx_Ref_Clk_Cnt", 14), ("C_init", 8), ("Reserved", 3))


# --------------------------------------------------------------------------------------------
# settings

@dataclass(frozen=True)
class TxConfig:
    mod: str = "QPSK"
    ref_sym: int = 6          # RefSymbolSpacing
    ref_sig: int = 3          # RefSignalSpacing
    plen: int = 73            # PacketLength
    n: int = 1296             # EncoderBlockSize
    rate: str = "56"          # "12", "23", "34", "56"
    rep: int = 15             # Repetition (copies = rep + 1)
    port: int = 0
    c_init: int = 129         # C_init_Dat (payload scrambler)
    client: int = 0           # ClientFlag: 0 Service, 1 Client
    antennas: int = 1         # NumberOfAntennas (control field NoOfTxAntenna)
    id1: int = 129            # C_init_ID1: Gold code of the Ref_P0 pilots (payload and control field)
    id2: int = 128            # C_init_ID2: Gold code of the Ref_P1 pilots
    use_pre: int = 1          # Use_Preamble
    use_cf: int = 1           # Use_ControlField
    clk_cnt: int = 0          # Tx_Ref_Clk_Cnt (control field; the simulator sends 0)
    crc: str = "Gen24"
    fs: int = 40_000_000      # SampleRate (the hardware runs 40 Msps only)
    cf_id1: int = 129         # Parameters_Control_Field.json C_init_ID1: control-field Ref_P0 pilots
    cf_cdat: int = 130        # Parameters_Control_Field.json C_init_Dat: control-field scrambler

    @staticmethod
    def from_params(path: Path, cf_path: Path | None = None) -> "TxConfig":
        """The Defined sections of Tx/Parameters_Tx_payload.json and (default: next to it in
        ControlSignals/) Parameters_Control_Field.json. The Calculated sections are derived."""
        path = Path(path)
        cf = Path(cf_path) if cf_path else path.parent / "ControlSignals" / "Parameters_Control_Field.json"
        cfd = json.loads(cf.read_text())["Defined"] if cf.is_file() else None
        return TxConfig.from_defined(json.loads(path.read_text())["Defined"], cfd)

    @staticmethod
    def from_defined(d: dict, cfd: dict | None = None) -> "TxConfig":
        cf_id1, cf_cdat = 129, 130
        if cfd is not None:
            check_cf_defined(cfd)
            cf_id1, cf_cdat = int(cfd.get("C_init_ID1", 129)), int(cfd.get("C_init_Dat", 130))
        return TxConfig(cf_id1=cf_id1, cf_cdat=cf_cdat, mod=str(d["Mod"]), ref_sym=int(d["RefSymbolSpacing"]),
                        ref_sig=int(d["RefSignalSpacing"]), plen=int(d["PacketLength"]),
                        n=int(d["EncoderBlockSize"]), rate=RATES[str(d["EncoderCodeRate"])],
                        rep=int(d["Repetition"]), port=int(d.get("Port", 0)),
                        c_init=int(d["C_init_Dat"]),
                        client=0 if str(d.get("ClientFlag", "Service")) == "Service" else 1,
                        antennas=int(d.get("NumberOfAntennas", 1)),
                        id1=int(d.get("C_init_ID1", 129)), id2=int(d.get("C_init_ID2", 128)),
                        use_pre=int(d.get("Use_Preamble", 1)), use_cf=int(d.get("Use_ControlField", 1)),
                        clk_cnt=int(d.get("Tx_Ref_Clk_Cnt", 0)), crc=str(d.get("CRC", "Gen24")),
                        fs=int(float(d.get("SampleRate", 40e6))))

    def defined(self) -> dict:
        """The Defined section of Parameters_Tx_payload.json for these settings."""
        return {"EncoderCodeRate": {v: k for k, v in RATES.items()}[self.rate], "EncoderBlockSize": self.n,
                "Repetition": self.rep, "CRC": self.crc, "Mod": self.mod, "PacketLength": self.plen,
                "RefSignalSpacing": self.ref_sig, "Port": self.port, "NumberOfAntennas": self.antennas,
                "ClientFlag": "Client" if self.client else "Service", "RefSymbolSpacing": self.ref_sym,
                "C_init_ID1": self.id1, "C_init_ID2": self.id2, "C_init_Dat": self.c_init, "SampleRate": self.fs,
                "Use_Preamble": self.use_pre, "Use_ControlField": self.use_cf, "Tx_Ref_Clk_Cnt": self.clk_cnt}

    def cf_defined(self) -> dict:
        """The Defined section of Parameters_Control_Field.json for these settings."""
        return {**CF_FIXED, "C_init_ID1": self.cf_id1, "C_init_Dat": self.cf_cdat}

    def check(self) -> None:
        assert self.mod in BITS and self.ref_sig in (3, 6, 12, 24) and self.ref_sym in (1, 3, 6, 12)
        assert self.plen in PACKET_LENGTHS and (self.plen - 1) % self.ref_sym == 0
        assert self.n in (648, 1296, 1944) and self.rate in RATE_FRAC and self.rep in REPETITIONS
        assert self.port in (0, 1) and 0 <= self.c_init <= 255 and self.antennas in (1, 2)
        assert 0 <= self.id1 < 1 << 31 and 0 <= self.id2 < 1 << 31 and 0 <= self.clk_cnt < 1 << 14
        assert self.use_pre in (0, 1) and self.use_cf in (0, 1) and self.client in (0, 1)
        assert self.crc == "Gen24", "the hardware has CRC Gen24 only"
        assert self.fs == 40_000_000, "the hardware runs at 40 Msps only"
        assert 1 <= self.ndb <= 1023, "no code word fits in the packet (or NDB_A over 10 bits)"
        assert 0 <= self.cf_id1 < 1 << 31 and 0 <= self.cf_cdat < 1 << 31

    @property
    def code(self) -> str:
        return f"{self.n}_{self.rate}"

    @property
    def k(self) -> int:
        return int(self.n * RATE_FRAC[self.rate])

    @property
    def tb_size(self) -> int:
        return self.k - 24

    @property
    def nb(self) -> int:
        return BITS[self.mod]

    def types(self) -> np.ndarray:
        ref, dat = _T[f"ref_row_{self.ref_sig}"], _T["data_row"]
        return np.array([ref if s % self.ref_sym == 0 else dat for s in range(self.plen)])

    @property
    def data_res(self) -> int:
        return int((self.types() == DATA).sum())

    @property
    def data_bits(self) -> int:                       # NumberOfDataSigs
        return self.data_res * self.nb

    @property
    def cw_size(self) -> int:                         # CodeWordSize
        return self.n * (self.rep + 1)

    @property
    def ndb(self) -> int:                             # AvailableDataBlocks
        return self.data_bits // self.cw_size

    @property
    def zero_pad(self) -> int:
        return self.data_bits - self.ndb * self.cw_size

    @property
    def samples(self) -> int:
        return self.use_pre * PRE_LEN + self.use_cf * CF_LEN + self.plen * SYM


# the control field's Defined values the hardware supports (Parameters_Control_Field.json)
CF_FIXED = {"ControlMessageSize": 54, "CRC": "Gen10", "Encoder": "BCC", "ConstraintLength": 7,
            "Polynomials": [133, 171], "EncoderCodeRate": "1/2", "TailBiting": 0, "Mod": "BPSK",
            "PacketLength": 4, "RefSignalSpacing": 3, "RefSymbolSpacing": 3, "Port": 0,
            "SampleRate": 40000000}


def check_cf_defined(cfd: dict) -> None:
    """Every fixed control-field setting must have the one value the hardware (and the Zedboard)
    supports; C_init_ID1 and C_init_Dat may be anything (31 bits)."""
    for k, v in CF_FIXED.items():
        if k not in cfd:
            continue
        got = cfd[k]
        same = [int(x) for x in got] == v if isinstance(v, list) else (
            float(got) == float(v) if isinstance(v, int) else str(got) == v)
        if not same:
            raise ValueError(f"Parameters_Control_Field.json {k} = {got!r}: the hardware supports {v!r} only")


# --------------------------------------------------------------------------------------------
# helpers

def _rnd(v, k):
    return (v + (1 << (k - 1))) >> k if k > 0 else v


def _sat(v, b):
    return np.clip(v, -(1 << (b - 1)), (1 << (b - 1)) - 1)


def gold(c_init: int, n: int, shift: int = 1600) -> np.ndarray:
    """LTE Gold code (Scrambler_Gold_Code_Generator.py), n bits after the 1600-bit shift."""
    r1, r2 = 1, c_init & 0x7FFFFFFF
    for _ in range(shift):
        r1 = (r1 >> 1) | (((r1 ^ (r1 >> 3)) & 1) << 30)
        r2 = (r2 >> 1) | (((r2 ^ (r2 >> 1) ^ (r2 >> 2) ^ (r2 >> 3)) & 1) << 30)
    return gold_from_state(r1, r2, n)


def gold_from_state(r1: int, r2: int, n: int) -> np.ndarray:
    out = np.zeros(n, np.int64)
    for i in range(n):
        out[i] = (r1 ^ r2) & 1
        r1 = (r1 >> 1) | (((r1 ^ (r1 >> 3)) & 1) << 30)
        r2 = (r2 >> 1) | (((r2 ^ (r2 >> 1) ^ (r2 >> 2) ^ (r2 >> 3)) & 1) << 30)
    return out


def gold_start(c_init: int) -> tuple[int, int]:
    """(x1, x2) register values after the 1600-bit shift: what the RTL loads at a packet start."""
    r1, r2 = 1, c_init & 0x7FFFFFFF
    for _ in range(1600):
        r1 = (r1 >> 1) | (((r1 ^ (r1 >> 3)) & 1) << 30)
        r2 = (r2 >> 1) | (((r2 ^ (r2 >> 1) ^ (r2 >> 2) ^ (r2 >> 3)) & 1) << 30)
    return r1, r2


def crc(bits, poly: int, width: int) -> np.ndarray:
    """GenerateCRC.py: zero start, MSB first, no reflection, no final XOR. Returns width bits."""
    c, top, mask = 0, width - 1, (1 << width) - 1
    for b in np.asarray(bits, np.int64):
        fb = ((c >> top) & 1) ^ int(b)
        c = (c << 1) & mask
        if fb:
            c ^= poly
    return np.array([(c >> (width - 1 - i)) & 1 for i in range(width)], np.int64)


def crc24(bits) -> np.ndarray:
    return crc(bits, CRC24_POLY, 24)


def bitrev(n_bits: int) -> np.ndarray:
    i = np.arange(1 << n_bits)
    r = np.zeros_like(i)
    for b in range(n_bits):
        r |= ((i >> b) & 1) << (n_bits - 1 - b)
    return r


# --------------------------------------------------------------------------------------------
# LDPC encoder (802.11n, from the prototype matrix)

class Code:
    """Row r, lane z of block column c connects code bit c*Z + (z + P[r, c]) % Z (pl_reference.Code).
    Parity: block columns kb .. kb+m-1; column kb has the shifts h (rows 0, x, m-1), the others the
    dual diagonal with shift 0."""

    def __init__(self, t: str):
        self.P, self.Z, self.t = _T[f"P_{t}"].astype(np.int64), int(_T[f"Z_{t}"]), t
        self.m, nb = self.P.shape
        self.kb = nb - self.m
        self.N, self.K = nb * self.Z, self.kb * self.Z
        h = self.P[:, self.kb]
        self.h = [(r, int(h[r])) for r in range(self.m) if h[r] >= 0]
        for j in range(1, self.m):          # dual diagonal: column kb+j has 0 at rows j-1 and j
            col = self.P[:, self.kb + j]
            assert col[j - 1] == 0 and col[j] == 0 and (col >= 0).sum() == 2, (t, j)
        # p0 = sum of lambda: needs the h shifts to add up to the identity (two equal, one 0)
        s = sorted(v for _, v in self.h)
        assert len(s) == 3 and (s[0] == s[1] or s[1] == s[2]), (t, s)
        self.h_x = [v for v in s if s.count(v) == 1][0]

    def rot(self, v: np.ndarray, s: int) -> np.ndarray:
        """Contribution of a block-column vector to the check rows: out[z] = v[(z + s) % Z]."""
        return np.roll(v, -s)

    def lambdas(self, msg: np.ndarray) -> np.ndarray:
        blocks = msg.reshape(self.kb, self.Z)
        lam = np.zeros((self.m, self.Z), np.int64)
        for r in range(self.m):
            for c in range(self.kb):
                if self.P[r, c] >= 0:
                    lam[r] ^= self.rot(blocks[c], int(self.P[r, c]))
        return lam

    def encode(self, msg) -> np.ndarray:
        msg = np.asarray(msg, np.int64)
        assert msg.size == self.K
        lam = self.lambdas(msg)
        # sum over rows: sum_r rot(p0, h_r) = sum_r lambda_r. With h = (a, b, a): rot(p0, b).
        tot = np.bitwise_xor.reduce(lam, axis=0)
        p0 = np.roll(tot, self.h_x)                  # rot(p0, h_x) = tot  ->  p0 = roll(tot, +h_x)
        hs = dict(self.h)
        p = [p0]
        prev = None
        for r in range(self.m - 1):                  # row r: lambda_r + rot(p0,h_r) + p_r' + p_{r+1}' = 0
            acc = lam[r].copy()
            if r in hs:
                acc ^= self.rot(p0, hs[r])
            if r > 0:
                acc ^= prev
            prev = acc                               # p_{r+1}
            p.append(acc)
        cw = np.concatenate([msg, *p])
        return cw

    def syndrome_ok(self, cw) -> bool:
        cw = np.asarray(cw, np.int64).reshape(-1, self.Z)
        for r in range(self.m):
            acc = np.zeros(self.Z, np.int64)
            for c in range(self.P.shape[1]):
                if self.P[r, c] >= 0:
                    acc ^= self.rot(cw[c], int(self.P[r, c]))
            if acc.any():
                return False
        return True


_CODES: dict = {}


def code_of(t: str) -> Code:
    if t not in _CODES:
        _CODES[t] = Code(t)
    return _CODES[t]


def ldpc_encode(cfg: TxConfig, msg) -> np.ndarray:
    return code_of(cfg.code).encode(msg)


# --------------------------------------------------------------------------------------------
# payload bits

@dataclass
class PayloadBits:
    codewords: np.ndarray       # (ndb, N) before interleaving
    interleaved: np.ndarray     # (ndb, N)
    unscrambled: np.ndarray     # (data_bits,)
    scrambled: np.ndarray       # (data_bits,)


def payload_bits(cfg: TxConfig, tbs) -> PayloadBits:
    """tbs: (ndb, TransportBlockSize) bits -> the scrambled sequence that fills the Data REs."""
    tbs = np.asarray(tbs, np.int64).reshape(cfg.ndb, cfg.tb_size)
    il = _T[f"il_{cfg.n}"]
    cws = np.array([ldpc_encode(cfg, np.concatenate([tb, crc24(tb)])) for tb in tbs])
    inter = cws[:, il]
    seq = np.concatenate([np.tile(w, cfg.rep + 1) for w in inter] + [np.zeros(cfg.zero_pad, np.int64)])
    assert seq.size == cfg.data_bits
    return PayloadBits(cws, inter, seq, seq ^ gold(cfg.c_init, seq.size))


# --------------------------------------------------------------------------------------------
# mapper

def qam_levels(bits, mod: str) -> tuple[np.ndarray, np.ndarray]:
    """QAM_Mapper.py as integer levels (I, Q): odd integers, first half of each group on I."""
    nb = BITS[mod]
    b = np.asarray(bits, np.int64).reshape(-1, nb)
    if mod == "BPSK":
        return 2 * b[:, 0] - 1, np.zeros(b.shape[0], np.int64)
    half = nb // 2
    wts = 1 << np.arange(half - 1, -1, -1)
    lut = np.array([_PAM[half][v] for v in range(1 << half)], np.int64)
    return lut[b[:, :half] @ wts], lut[b[:, half:] @ wts]


def pilot_levels(bits, mod: str) -> tuple[np.ndarray, np.ndarray]:
    """Generate_Reference_Signal_Vectors.map_bits as levels: BPSK 1 - 2b; QAM 1 -> (L, -L), 0 -> (-L, L)."""
    b = np.asarray(bits, np.int64)
    if mod == "BPSK":
        return 1 - 2 * b, np.zeros(b.size, np.int64)
    L = OUTER[mod]
    s = 2 * b - 1
    return s * L, -s * L


def pilot_bits(c_init_id: int, ref_sig: int) -> np.ndarray:
    """Ref_Pz bits of one reference symbol: 1, 0, 1, ... scrambled with the Gold code c_init_id
    (Generate_Reference_Signal_Vectors.py; C_init_ID1 for Ref_P0, C_init_ID2 for Ref_P1)."""
    n = 840 // ref_sig
    return (np.arange(n) % 2 == 0).astype(np.int64) ^ gold(c_init_id, n)


def grid_levels(types: np.ndarray, data_bits, mod: str, port: int, ref_sig: int,
                pilot_mod: str | None = None, c_init_id: int | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(symbols, 841) integer levels. Data REs in order symbol by symbol, k = 0 .. 840.
    c_init_id: Gold code of the port's pilots (default 129 for port 0, 128 for port 1)."""
    gi = np.zeros(types.shape, np.int64)
    gq = np.zeros(types.shape, np.int64)
    di, dq = qam_levels(data_bits, mod)
    m = types == DATA
    assert di.size == m.sum(), (di.size, m.sum())
    gi[m], gq[m] = di, dq
    code = P0 if port == 0 else P1
    cid = c_init_id if c_init_id is not None else (129 if port == 0 else 128)
    pi, pq = pilot_levels(pilot_bits(cid, ref_sig), pilot_mod or mod)
    for s in range(types.shape[0]):
        k = np.flatnonzero(types[s] == code)
        if k.size:
            gi[s, k], gq[s, k] = pi, pq
    return gi, gq


def ifft_input(gi: np.ndarray, gq: np.ndarray, amp: int) -> tuple[np.ndarray, np.ndarray]:
    """(symbols, 2048) IFFT input bins: bins 0..420 <- k 420..840, bins 1628..2047 <- k 0..419."""
    xr = np.zeros((gi.shape[0], N_FFT), np.int64)
    xi = np.zeros_like(xr)
    xr[:, :421], xi[:, :421] = gi[:, 420:] * amp, gq[:, 420:] * amp
    xr[:, 1628:], xi[:, 1628:] = gi[:, :420] * amp, gq[:, :420] * amp
    return xr, xi


# --------------------------------------------------------------------------------------------
# IFFT (radix-2^2 SDF, decimation in frequency)

TW_RE, TW_IM = _T["tw_re"].astype(np.int64), _T["tw_im"].astype(np.int64)


def _stage_out(vr, vi, stage):
    sh = SHIFTS[stage]
    return _sat(_rnd(vr, sh), W), _sat(_rnd(vi, sh), W)


def ifft_fixed(xr, xi, debug: dict | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(symbols, 2048) integer bins in natural order -> (symbols, 2048) time samples in natural order.
    Computes sum_k X[k] exp(+j 2 pi k n / 2048) / 32, with the hardware's rounding: stage s output
    = sat18(rnd(butterfly, SHIFTS[s])); twiddle products (a*w - b*v + 2^15) >> 16, sat18. The SDF
    hardware produces the samples in bit-reversed order (index bitrev(n) at output slot n)."""
    xr = _sat(np.asarray(xr, np.int64), W).copy()
    xi = _sat(np.asarray(xi, np.int64), W).copy()
    S = xr.shape[0]
    stage = 0
    L = N_FFT
    while L >= 8:
        q = L // 4
        ar = xr.reshape(S, -1, 4, q)
        ai = xi.reshape(S, -1, 4, q)
        a, b, c, d = (ar[:, :, i] for i in range(4))
        ia, ib, ic, id_ = (ai[:, :, i] for i in range(4))
        # BF2I (distance L/2)
        u0r, u0i = _stage_out(a + c, ia + ic, stage)
        u1r, u1i = _stage_out(b + d, ib + id_, stage)
        u2r, u2i = _stage_out(a - c, ia - ic, stage)
        u3r, u3i = _stage_out(b - d, ib - id_, stage)
        stage += 1
        # BF2II (distance L/4); the k1 = 1 half takes +j * u3 (IFFT)
        v0r, v0i = _stage_out(u0r + u1r, u0i + u1i, stage)
        v1r, v1i = _stage_out(u0r - u1r, u0i - u1i, stage)
        v2r, v2i = _stage_out(u2r - u3i, u2i + u3r, stage)
        v3r, v3i = _stage_out(u2r + u3i, u2i - u3r, stage)
        stage += 1
        # twiddles W_L^(n3 * e), e = 0, 2, 1, 3 for the four sub-blocks
        n3 = np.arange(q)
        outr = np.empty_like(ar)
        outi = np.empty_like(ai)
        for j, (vr, vi, e) in enumerate(((v0r, v0i, 0), (v1r, v1i, 2), (v2r, v2i, 1), (v3r, v3i, 3))):
            m = n3 * e * (N_FFT // L)
            wr, wi = TW_RE[m], TW_IM[m]
            outr[:, :, j] = _sat(_rnd(vr * wr - vi * wi, TW_FRAC), W)
            outi[:, :, j] = _sat(_rnd(vr * wi + vi * wr, TW_FRAC), W)
        xr = outr.reshape(S, N_FFT)
        xi = outi.reshape(S, N_FFT)
        if debug is not None:
            debug[f"L{L}"] = (xr.copy(), xi.copy())
        L = q
    # final radix-2 stage (L = 2)
    ar = xr.reshape(S, -1, 2)
    ai = xi.reshape(S, -1, 2)
    yr0, yi0 = _stage_out(ar[:, :, 0] + ar[:, :, 1], ai[:, :, 0] + ai[:, :, 1], stage)
    yr1, yi1 = _stage_out(ar[:, :, 0] - ar[:, :, 1], ai[:, :, 0] - ai[:, :, 1], stage)
    br = np.stack([yr0, yr1], axis=2).reshape(S, N_FFT)     # bit-reversed order
    bi = np.stack([yi0, yi1], axis=2).reshape(S, N_FFT)
    rev = bitrev(11)
    yr = np.empty_like(br)
    yi = np.empty_like(bi)
    yr[:, rev] = br
    yi[:, rev] = bi
    return yr, yi


def add_cp(yr, yi) -> tuple[np.ndarray, np.ndarray]:
    return (np.concatenate([yr[:, N_FFT - CP:], yr], axis=1).ravel(),
            np.concatenate([yi[:, N_FFT - CP:], yi], axis=1).ravel())


def output_stage(xr, xi, gain: int) -> tuple[np.ndarray, np.ndarray]:
    assert 0 <= gain < 1 << GAIN_BITS
    return (_sat(_rnd(np.asarray(xr, np.int64) * gain, GAIN_FRAC), OUT_BITS),
            _sat(_rnd(np.asarray(xi, np.int64) * gain, GAIN_FRAC), OUT_BITS))


# --------------------------------------------------------------------------------------------
# preamble

def preamble_rom() -> tuple[np.ndarray, np.ndarray]:
    """The 12480 preamble samples as the ROM player outputs them (before the output stage)."""
    ar, ai = _T["pre_a_re"], _T["pre_a_im"]
    br, bi = _T["pre_b_re"], _T["pre_b_im"]
    ia = np.arange(PRE_A_LEN) % 64
    ib = np.concatenate([np.arange(N_FFT - CP, N_FFT), np.arange(N_FFT)])
    r = np.concatenate([br[:AGC_LEN], ar[ia], br[ib]]).astype(np.int64)
    i = np.concatenate([bi[:AGC_LEN], ai[ia], bi[ib]]).astype(np.int64)
    return r, i


# --------------------------------------------------------------------------------------------
# control field

CF_TYPES = np.array([_T["ref_row_3"], _T["data_row"], _T["data_row"], _T["ref_row_3"]])

_CF_CODES = {
    "RefSymbolSpacing": {1: 0b00, 3: 0b01, 6: 0b10, 12: 0b11},
    "RefSignalSpacing": {3: 0b00, 6: 0b01, 12: 0b10, 24: 0b11},
    "PacketLength": {4: 0b001, 37: 0b011, 49: 0b111, 73: 0b100, 97: 0b110, 121: 0b101},
    "PayloadFieldMod": {"BPSK": 0b00, "QPSK": 0b01, "16QAM": 0b10, "64QAM": 0b11},
    "CBS_A_Flag": {648: 0b00, 1296: 0b01, 1944: 0b10},
    "FEC_A_Flag": {"12": 0b00, "23": 0b01, "34": 0b10, "56": 0b11},
    "RM_A_Flag": {r: i for i, r in enumerate(REPETITIONS)},
}


def cf_message(cfg: TxConfig, tx_ref_clk_cnt: int | None = None) -> np.ndarray:
    """The 54-bit ControlMessage (Initialize_Control_Field.build_control_message), MSB first."""
    codes = {"RefSymbolSpacing": _CF_CODES["RefSymbolSpacing"][cfg.ref_sym],
             "RefSignalSpacing": _CF_CODES["RefSignalSpacing"][cfg.ref_sig],
             "PacketLength": _CF_CODES["PacketLength"][cfg.plen],
             "PayloadFieldMod": _CF_CODES["PayloadFieldMod"][cfg.mod],
             "NoOfTxAntenna": cfg.antennas - 1,
             "CBS_A_Flag": _CF_CODES["CBS_A_Flag"][cfg.n],
             "FEC_A_Flag": _CF_CODES["FEC_A_Flag"][cfg.rate],
             "RM_A_Flag": _CF_CODES["RM_A_Flag"][cfg.rep],
             "NDB_A": cfg.ndb, "ClientFlag": cfg.client,
             "Tx_Ref_Clk_Cnt": cfg.clk_cnt if tx_ref_clk_cnt is None else tx_ref_clk_cnt,
             "C_init": cfg.c_init, "Reserved": 0}
    bits = []
    for name, w in CF_FIELDS:
        assert 0 <= codes[name] < 1 << w, name
        bits += [(codes[name] >> (w - 1 - i)) & 1 for i in range(w)]
    return np.array(bits, np.int64)


def bcc_encode(bits) -> np.ndarray:
    """K = 7, 133/171, zero start, output A0 B0 A1 B1 ... (Convolutionalencoder.py, cf_reference)."""
    g = (0o133, 0o171)
    s, out = 0, []
    for b in np.asarray(bits, np.int64):
        v = (int(b) << 6) | s
        out += [bin(v & g[0]).count("1") & 1, bin(v & g[1]).count("1") & 1]
        s = v >> 1
    return np.array(out, np.int64)


def cf_bits(message, c_init: int = CF_C_INIT) -> np.ndarray:
    """54 message bits -> 2200 scrambled bits (Construct_Encoded_Control_Tx.py)."""
    msg = np.asarray(message, np.int64)
    blk = np.concatenate([msg, crc(msg, CRC10_POLY, 10), np.zeros(6, np.int64)])     # 70
    code = bcc_encode(blk)                                                          # 140
    inter = code[_T["bcc_il"]]
    seq = np.concatenate([np.tile(inter, 15), np.zeros(100, np.int64)])             # 2200
    return seq ^ gold(c_init, seq.size)


# --------------------------------------------------------------------------------------------
# power and gains

def _ifft_power(sum_abs2_per_symbol: float) -> float:
    # y[n] = sum_k X[k] e^(...) / 2^S  ->  mean |y|^2 = sum_k |X[k]|^2 / 2^(2S)
    return sum_abs2_per_symbol / (1 << (2 * sum(SHIFTS)))


def payload_power_expected(cfg: TxConfig) -> float:
    t = cfg.types()
    nd = int((t == DATA).sum())
    npil = int((t == (P0 if cfg.port == 0 else P1)).sum())
    amp = AMP[cfg.mod]
    pil = (1 if cfg.mod == "BPSK" else 2 * OUTER[cfg.mod] ** 2) * amp ** 2
    return _ifft_power((nd * NORM[cfg.mod] * amp ** 2 + npil * pil) / cfg.plen)


def cf_power_expected() -> float:
    n = int(np.isin(CF_TYPES, (DATA, P0)).sum())        # every Data and Ref_P0 RE is +-AMP_BPSK
    return _ifft_power(n * AMP["BPSK"] ** 2 / CF_SYMS)


def preamble_power() -> float:
    r, i = preamble_rom()
    return float(np.mean(r.astype(float) ** 2 + i.astype(float) ** 2))


def gains(cfg: TxConfig, backoff_db: float = BACKOFF_DB) -> dict[str, int]:
    """Output-stage gains: every segment at rms FULL_SCALE * 10^(-backoff/20) (complex rms)."""
    target = FULL_SCALE * 10 ** (-backoff_db / 20)
    out = {}
    for name, p in (("pre", preamble_power()), ("cf", cf_power_expected()),
                    ("pl", payload_power_expected(cfg))):
        g = int(round(target / np.sqrt(p) * (1 << GAIN_FRAC)))
        assert 0 < g < 1 << GAIN_BITS, (name, g)
        out[name] = g
    return out


# --------------------------------------------------------------------------------------------
# whole packet

@dataclass
class Packet:
    cfg: TxConfig
    i: np.ndarray               # (samples,) 12-bit I
    q: np.ndarray               # (samples,) 12-bit Q
    gains: dict
    bits: PayloadBits
    cf_scrambled: np.ndarray
    grid: tuple                 # payload levels (gi, gq)
    cf_grid: tuple
    ifft_pl: tuple              # payload IFFT output (symbols, 2048) natural order
    ifft_cf: tuple

    @property
    def iq(self) -> np.ndarray:
        return np.stack([self.i, self.q], axis=1).astype(np.int16)


def packet(cfg: TxConfig, tbs, backoff_db: float = BACKOFF_DB, gain: dict | None = None) -> Packet:
    """One packet. gain: per-segment overrides ({"pre", "cf", "pl"}; missing or 0 = gains())."""
    cfg.check()
    g = dict(gains(cfg, backoff_db))
    for k, v in (gain or {}).items():
        if v:
            g[k] = int(v)
    pb = payload_bits(cfg, tbs)
    pid = cfg.id1 if cfg.port == 0 else cfg.id2
    gi, gq = grid_levels(cfg.types(), pb.scrambled, cfg.mod, cfg.port, cfg.ref_sig, c_init_id=pid)
    pr_, pi_ = ifft_fixed(*ifft_input(gi, gq, AMP[cfg.mod]))
    cfb = cf_bits(cf_message(cfg), cfg.cf_cdat)
    ci, cq = grid_levels(CF_TYPES, cfb, "BPSK", 0, 3, c_init_id=cfg.cf_id1)
    cr_, ci_ = ifft_fixed(*ifft_input(ci, cq, AMP["BPSK"]))
    segs = ([output_stage(*preamble_rom(), g["pre"])] if cfg.use_pre else []) + \
           ([output_stage(*add_cp(cr_, ci_), g["cf"])] if cfg.use_cf else []) + \
           [output_stage(*add_cp(pr_, pi_), g["pl"])]
    i = np.concatenate([s[0] for s in segs])
    q = np.concatenate([s[1] for s in segs])
    assert i.size == cfg.samples
    return Packet(cfg, i, q, g, pb, cfb, (gi, gq), (ci, cq), (pr_, pi_), (cr_, ci_))


def random_tbs(cfg: TxConfig, rng: np.random.Generator) -> np.ndarray:
    return rng.integers(0, 2, size=(cfg.ndb, cfg.tb_size)).astype(np.int64)


if __name__ == "__main__":
    c = TxConfig()
    p = packet(c, random_tbs(c, np.random.default_rng(1)))
    rms = np.sqrt(np.mean(p.i.astype(float) ** 2 + p.q.astype(float) ** 2))
    print(f"{c}: {p.i.size} samples, rms {rms:.1f}, peak {max(np.abs(p.i).max(), np.abs(p.q).max())}, "
          f"gains {p.gains}")


# --------------------------------------------------------------------------------------------
# stream for the PL (RTL/ptx_core.v): what the host writes into the TX DMA buffer
#
# A record = parameter header (6 words: the Defined sections of Tx/Parameters_Tx_payload.json and
# Tx/ControlSignals/Parameters_Control_Field.json)
# followed by n_packets x AvailableDataBlocks transport blocks, each TransportBlockSize bits from
# a new 64-bit word, bit j of the block in word j // 64 at bit j % 64. The PL derives the
# Calculated values, the control-field message and the gains itself. Records follow each other;
# a record with 0 packets only sets the parameters (for the packets of later records, which carry
# their own header anyway).

MAGIC = 0x50584C46                     # "FLXP"
VERSION = 2
MOD_CODE = {"BPSK": 0, "QPSK": 1, "16QAM": 2, "64QAM": 3}
RATE_CODE = {"12": 0, "23": 1, "34": 2, "56": 3}
CBS_CODE = {648: 0, 1296: 1, 1944: 2}
HDR_WORDS = 6


def pack_bits(bits) -> np.ndarray:
    """Bits -> 64-bit words, bit j in word j // 64 at position j % 64 (last word zero-padded)."""
    b = np.asarray(bits, np.uint64)
    n = (b.size + 63) // 64
    pad = np.zeros(n * 64, np.uint64)
    pad[:b.size] = b
    return (pad.reshape(n, 64) << np.arange(64, dtype=np.uint64)).sum(axis=1, dtype=np.uint64)


def param_header(cfg: TxConfig, n_packets: int, gap: int = 4096, gain: dict | None = None) -> np.ndarray:
    """The 6 header words (field layout in RTL/ptx_core.v and the README).
    gap: idle samples after each packet (rounded up to 16); gain: optional output-stage overrides
    ({"pre", "cf", "pl"}, 0 = the PL's own value for 13 dB backoff)."""
    cfg.check()
    g = gain or {}
    g16 = (int(gap) + 15) // 16
    assert 0 <= g16 < 1 << 16 and 0 <= n_packets < 1 << 32
    w0 = MAGIC | VERSION << 32
    w1 = (MOD_CODE[cfg.mod] | cfg.ref_sym << 2 | cfg.ref_sig << 6 | cfg.plen << 11 | cfg.port << 18
          | (cfg.antennas - 1) << 19 | cfg.client << 20 | cfg.use_pre << 21 | cfg.use_cf << 22
          | RATE_CODE[cfg.rate] << 23 | CBS_CODE[cfg.n] << 25 | cfg.rep << 27 | cfg.c_init << 32
          | cfg.clk_cnt << 40 | 0 << 54)
    w2 = cfg.id1 | cfg.id2 << 31
    w3 = int(g.get("pre", 0)) | int(g.get("cf", 0)) << 16 | int(g.get("pl", 0)) << 32 | g16 << 48
    w4 = n_packets | (cfg.fs // 1000) << 32
    w5 = cfg.cf_id1 | cfg.cf_cdat << 31
    return np.array([w0, w1, w2, w3, w4, w5], np.uint64)


def tb_words(cfg: TxConfig, tbs) -> np.ndarray:
    """Transport blocks (k, TransportBlockSize) -> words, each block from a new word."""
    tbs = np.asarray(tbs).reshape(-1, cfg.tb_size)
    return np.concatenate([pack_bits(t) for t in tbs]) if len(tbs) else np.zeros(0, np.uint64)


def record_words(cfg: TxConfig, packets_tbs, gap: int = 4096, gain: dict | None = None) -> np.ndarray:
    """Header + the blocks of every packet (packets_tbs: list of (ndb, TransportBlockSize) arrays)."""
    return np.concatenate([param_header(cfg, len(packets_tbs), gap, gain),
                           *[tb_words(cfg, t) for t in packets_tbs]])


def record_packets(cfg: TxConfig, packets_tbs, gain: dict | None = None) -> list[Packet]:
    return [packet(cfg, t, gain=gain) for t in packets_tbs]


def stream_samples(p: Packet, gap: int = 4096) -> np.ndarray:
    """What ptx_core outputs for one packet and its gap: (samples, 3) = I, Q, active."""
    g = (int(gap) + 15) // 16 * 16
    a = np.ones(p.i.size, np.int64)
    out = np.stack([p.i, p.q, a], axis=1)
    return np.concatenate([out, np.zeros((g, 3), np.int64)])


def json_record(params_path: Path, packets_tbs, gap: int = 4096, gain: dict | None = None) -> np.ndarray:
    """Record straight from Tx/Parameters_Tx_payload.json."""
    return record_words(TxConfig.from_params(params_path), packets_tbs, gap, gain)
