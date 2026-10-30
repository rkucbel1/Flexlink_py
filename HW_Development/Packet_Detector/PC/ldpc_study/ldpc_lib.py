"""LDPC study helpers: 802.11n codes from the prototype tables, channel + demapper as the
simulator, decoders: simulator (flooding min-sum, averaged probabilities), float layered,
fixed-point layered."""
import sys, numpy as np
import os, pathlib
U = os.environ.get("FLEXLINK_MODULES", str(pathlib.Path(__file__).resolve().parents[4] / "Flexlink_BasicModules"))
sys.path.insert(0, U + "/LDPC_Encoder/MatrixGeneration_H"); sys.path.insert(0, U + "/QAM_Mapper")
from get_p_ldpc import get_p_ldpc
from QAM_Mapper import qam_map
from QAM_Demapper import QAM_Demapper
BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}


class Code:
    def __init__(self, t):
        P, Z = get_p_ldpc(t); P = np.asarray(P); self.Z = Z; self.t = t
        self.nb, self.kb = P.shape[1], P.shape[1] - P.shape[0]
        self.N = self.nb * Z; self.K = self.kb * Z
        z = np.arange(Z)
        self.layers = []
        for r in range(P.shape[0]):
            cols = np.flatnonzero(P[r] >= 0)
            idx = np.stack([c * Z + (z + int(P[r, c])) % Z for c in cols])     # (deg, Z): row z of layer r
            self.layers.append(idx)
        self.edges = np.concatenate([l.ravel() for l in self.layers])

    def syndrome_ok(self, hard):                # hard (B, N) 0/1
        ok = np.ones(hard.shape[0], bool)
        for idx in self.layers:
            ok &= (hard[:, idx].sum(1) % 2 == 0).all(1)
        return ok


def channel_llr(code, mod, snr_db, copies, B, rng):
    """(copies, B, N) max-log LLRs (ln P0/P1) of the all-zero code word sent through a random
    scrambler (symmetrised), descrambled. AWGN, unit-power constellation."""
    nb = BITS[mod]; N = code.N
    nsym = -(-N // nb)
    nv = 10 ** (-snr_db / 10)
    out = np.empty((copies, B, N))
    for c in range(copies):
        scr = rng.integers(0, 2, (B, nsym * nb))
        sym = np.asarray(qam_map(scr.ravel(), mod)).reshape(B, nsym)
        y = sym + np.sqrt(nv / 2) * (rng.standard_normal(sym.shape) + 1j * rng.standard_normal(sym.shape))
        llr = np.asarray(QAM_Demapper(y.ravel(), mod, "maxlog", nv), float).reshape(B, nsym * nb)
        llr = np.where(scr == 1, -llr, llr)                   # descramble -> LLR of a 0 bit
        out[c] = llr[:, :N]
    return out


def beliefs_sim(llr_copies):
    """Simulator: p1 = 1/(1+exp(LLR)), averaged over copies, belief 2p-1 (positive = bit 1)."""
    p1 = 1.0 / (1.0 + np.exp(np.clip(llr_copies, -50, 50)))
    return 2.0 * p1.mean(0) - 1.0


def decode_flood(code, belief, iters=15):
    """Simulator's flooding min-sum (beliefs: positive = 1). Returns hard bits (B, N)."""
    B = belief.shape[0]
    E = [np.zeros((B,) + idx.shape) for idx in code.layers]
    L = belief.copy()
    for _ in range(iters):
        tot = np.zeros_like(belief)
        newE = []
        for idx, e in zip(code.layers, E):
            m = L[:, idx] - e                                 # (B, deg, Z)
            s = np.sign(m); s[s == 0] = 1; a = np.abs(m)
            # sim: Sign_Other = -sign(prod(-r_other)) -> for positive=1 beliefs: parity per check
            d = idx.shape[0]
            prod = np.prod(s, axis=1, keepdims=True)
            so = prod * s * (-1.0) ** (d - 1) * (-1.0)          # -(-1)^(d-1) prod_others
            o = np.argsort(a, axis=1)
            mn1 = np.take_along_axis(a, o[:, :1], 1); mn2 = np.take_along_axis(a, o[:, 1:2], 1)
            mag = np.where(np.arange(d)[None, :, None] == o[:, :1], mn2, mn1)
            en = so * mag
            newE.append(en)
            np.add.at(tot, (slice(None), idx.ravel()), en.reshape(B, -1))
        E = newE
        L = belief + tot
    return (L > 0).astype(np.int64)


def decode_layered(code, llr, iters=8, alpha=0.75, early=True):
    """Float layered normalised min-sum. llr: (B, N), ln P0/P1 (positive = 0). Hard bits."""
    B = llr.shape[0]
    L = llr.copy()
    R = [np.zeros((B,) + idx.shape) for idx in code.layers]
    done = np.zeros(B, bool)
    for it in range(iters):
        for k, idx in enumerate(code.layers):
            q = L[:, idx] - R[k]
            s = np.where(q < 0, -1.0, 1.0); a = np.abs(q)
            o = np.argsort(a, axis=1)
            mn1 = np.take_along_axis(a, o[:, :1], 1); mn2 = np.take_along_axis(a, o[:, 1:2], 1)
            mag = np.where(np.arange(idx.shape[0])[None, :, None] == o[:, :1], mn2, mn1)
            r = alpha * np.prod(s, 1, keepdims=True) * s * mag
            L[:, idx] = q + r
            R[k] = r
        if early:
            hard = (L < 0).astype(np.int64)
            done |= code.syndrome_ok(hard)
            if done.all():
                break
    return (L < 0).astype(np.int64), it + 1


def decode_layered_fixed(code, llr_q, iters=8, app_bits=8, msg_bits=6, early=True):
    """Fixed-point layered normalised min-sum (alpha 0.75 = m - m >> 2, floor), integer LLRs.
    APP saturated to app_bits, messages to msg_bits (signed)."""
    B = llr_q.shape[0]
    amax = (1 << (app_bits - 1)) - 1; mmax = (1 << (msg_bits - 1)) - 1
    L = np.clip(llr_q.astype(np.int64), -amax, amax)
    R = [np.zeros((B,) + idx.shape, np.int64) for idx in code.layers]
    done = np.zeros(B, bool); it = 0
    for it in range(iters):
        for k, idx in enumerate(code.layers):
            q = np.clip(L[:, idx] - R[k], -amax, amax)
            a = np.minimum(np.abs(q), mmax)
            s = np.where(q < 0, -1, 1)
            o = np.argsort(a, axis=1, kind="stable")
            mn1 = np.take_along_axis(a, o[:, :1], 1); mn2 = np.take_along_axis(a, o[:, 1:2], 1)
            mn1 = mn1 - (mn1 >> 2); mn2 = mn2 - (mn2 >> 2)
            mag = np.where(np.arange(idx.shape[0])[None, :, None] == o[:, :1], mn2, mn1)
            r = np.prod(s, 1, keepdims=True) * s * mag
            L[:, idx] = np.clip(q + r, -amax, amax)
            R[k] = r
        if early:
            done |= code.syndrome_ok((L < 0).astype(np.int64))
            if done.all():
                break
    return (L < 0).astype(np.int64), it + 1


def quantise(llr, scale, bits):
    m = (1 << (bits - 1)) - 1
    return np.clip(np.round(llr * scale), -m, m).astype(np.int64)
