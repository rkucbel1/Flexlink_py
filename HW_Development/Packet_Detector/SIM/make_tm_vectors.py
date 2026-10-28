#!/usr/bin/env python3
"""
make_tm_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Directed vectors for the Preamble B timing block alone (tb_pd_timing.v -> RTL/pd_timing.v),
with the bit-true model's records (pd_reference.timing_records). The detector is not in the
loop: the testbench feeds DC-blocker samples, pulses `trig` at t and the event (t, cfo_inc,
fail) N + 60 samples later. Each case aims at the exact boundaries that real vectors almost
never hit:

  samples on the de-rotator's rounding boundary  yi c - yq s (or yi s + yq c) = 2^20 mod 2^21
  late trigger exactly at the limit               n_end = t + N + 1130 (late) / + 1131 (not)
  one search at a time exactly at the limit       n0 = hi of the running search (BUSY) / hi + 1
  full-scale samples (saturation), no Preamble B (peak check fails), failed tone checks,
  a silent window (all P equal: the first position wins)
  N_CFO 128 and 4096 (window lengths 4937 and 969)

  work/tm_<case>/cfg.txt      n_cfo t_pk n_samples n_events n_records
  work/tm_<case>/in.txt       y samples, "iii qqq" (12-bit hex)
  work/tm_<case>/ev.txt       per event "t cfo_inc fail" (decimal, cfo_inc 24-bit)
  work/tm_<case>/exp_tm.txt   expected records, as make_tb_vectors.py

    python make_tm_vectors.py          # all cases (about 1 minute: the boundary search)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
import pd_protocol as pr  # noqa: E402
import pd_reference as ref  # noqa: E402

M21 = (1 << 21) - 1
HALF = 1 << 20
LIM = (1 << (ref.TIM_BITS - 1)) - 1          # 31: stay below saturation


def boundary_samples(rail: str, n_a: int = 64) -> dict[int, tuple[int, int]]:
    """Table address a -> (yi, yq) whose rotation lands exactly on rnd's half for `rail`
    ('re': yi c - yq s, 'im': yi s + yq c), without saturating."""
    c_t, s_t = np.array(ref.TONE_COS, np.int64), np.array(ref.TONE_SIN, np.int64)
    y = np.arange(-2048, 2048, dtype=np.int64)
    out = {}
    for a in range(3, 1024, 1024 // n_a):
        c, s = c_t[a], s_t[a]
        v = (y * c)[:, None] - (y * s)[None, :] if rail == "re" else (y * s)[:, None] + (y * c)[None, :]
        r = (v + HALF) >> 21
        ok = ((v & M21) == HALF) & (np.abs(r) < LIM)
        w = np.argwhere(ok)
        if len(w):
            out[a] = (int(y[w[0, 0]]), int(y[w[0, 1]]))
    return out


class Case:
    def __init__(self, n: int, n_cfo: int, t_pk: int, seed: int):
        self.rng = np.random.default_rng(seed)
        self.n_cfo, self.t_pk = n_cfo, t_pk
        self.y = np.round(self.rng.normal(0, 300, (n, 2)))
        self.events: list[tuple[int, int, int]] = []      # t, cfo_inc, fail

    def preamble(self, end: int, t: int, inc: int, amp: float = 1500.0):
        """Preamble B ending at sample `end`, rotated so that the event (t, inc) de-rotates it."""
        pb = np.load(HERE / "pb20.npy") if (HERE / "pb20.npy").exists() else None
        if pb is None:
            pb = ref.PB_SI + 1j * ref.PB_SQ                   # the signs alone correlate just as well
        lo, _ = ref.timing_window(t, self.n_cfo)
        n0 = lo - (ref.PB_LEN - 1)
        n = np.arange(end - ref.PB_LEN + 1, end + 1)
        ph = 2 * np.pi * ((inc * (n - n0)) % (1 << 24)) / (1 << 24)
        z = amp * pb / np.sqrt(np.mean(np.abs(pb) ** 2)) * np.exp(-1j * ph)
        self.y[n, 0] += z.real
        self.y[n, 1] += z.imag

    def put_boundary(self, t: int, inc: int, table: dict[int, tuple[int, int]], every: int = 3):
        """Overwrite samples of this event's window whose table address has a boundary pair."""
        lo, hi = ref.timing_window(t, self.n_cfo)
        n0 = lo - (ref.PB_LEN - 1)
        hits = 0
        for n in range(n0, hi + 1, every):
            a = ((inc * (n - n0)) % (1 << 24)) >> 14
            if a in table:
                self.y[n] = table[a]
                hits += 1
        return hits

    def event(self, t: int, inc: int, fail: int = 0):
        self.events.append((t, inc & 0xFFFFFF, fail))

    def write(self, name: str):
        y = np.clip(self.y, -2048, 2047).astype(np.int64)
        evs = [pr.EventRecord(t, inc, t + self.n_cfo, 0, pr.EV_CFO_VALID | (pr.EV_TONE_FAIL if f else 0))
               for t, inc, f in self.events]
        recs = ref.timing_records(y, evs, self.n_cfo, self.t_pk)
        out = HERE / "work" / f"tm_{name}"
        out.mkdir(parents=True, exist_ok=True)
        np.savetxt(out / "in.txt", y & 0xFFF, fmt="%03x")
        (out / "ev.txt").write_text("".join(f"{t} {inc} {f}\n" for t, inc, f in self.events))
        (out / "exp_tm.txt").write_text("".join(
            f"{r.t_trig:08x} {r.n_end:08x} {r.status:x} {r.n_pos:04x} {r.p_max:09x} {r.p_sum:012x}\n"
            for r in recs))
        (out / "cfg.txt").write_text(f"{self.n_cfo} {self.t_pk} {y.shape[0]} {len(self.events)} {len(recs)}\n")
        kinds = {"good": 0, "late": 0, "nopeak": 0, "busy": 0}
        for r in recs:
            k = "busy" if r.status & pr.TM_BUSY else "late" if r.status & pr.TM_LATE else \
                "good" if r.status & pr.TM_PEAK_OK else "nopeak"
            kinds[k] += 1
        print(f"{out}: {y.shape[0]} samples, {len(self.events)} events, {len(recs)} records {kinds}")
        return recs


def main() -> int:
    tab_re, tab_im = boundary_samples("re"), boundary_samples("im")
    N = 2048
    c = Case(80000, N, 18 * 256, seed=1)
    inc = 1 << 14                                       # table address steps by 1 per sample
    t = 1000
    # 1, 2: rounding boundaries (re, im), Preamble B where a real latency would put it
    for tab in (tab_re, tab_im):
        c.preamble(t + 4100, t, inc)
        c.put_boundary(t, inc, tab)
        c.event(t, inc)
        t += 7000
    # 3, 4: late trigger exactly at the limit, and one sample inside
    for off in (1130, 1131):
        c.preamble(t + N + off, t, -12345)
        c.event(t, -12345)
        t += 7000
    # 5, 6, 7: one search at a time: n0 = hi of the running search -> BUSY; n0 = hi + 1 -> runs;
    # a failed tone check inside that window gets no record and holds nothing
    t5 = t
    c.preamble(t5 + 4000, t5, 98765)
    c.event(t5, 98765)
    hi5 = ref.timing_window(t5, N)[1]
    t6 = hi5 - (N - 40)                                   # n0 = hi5 -> BUSY
    c.event(t6, 5555)
    t7 = t6 + 1                                           # n0 = hi5 + 1 -> runs
    c.preamble(t7 + 3600, t7, -2222)
    c.event(t7, -2222)
    c.event(t7 + 2500, 777, fail=1)
    t = t7 + 7000
    # 8: full scale (saturation), with a strong Preamble B
    c.y[t + 2000:t + 6200] = c.rng.choice([-2048, 2047], (4200, 2))
    c.preamble(t + 4500, t, 1 << 20, amp=20000)
    c.event(t, 1 << 20)
    t += 7000
    # 9: no Preamble B: the peak check fails
    c.event(t, 31337)
    t += 7000
    # 10: silence over the whole window: every P is 0, so the first position must win
    lo10, hi10 = ref.timing_window(t, N)
    c.y[lo10 - (ref.PB_LEN - 1) - 10:hi10 + 10] = 0
    c.event(t, 4242)
    t += 7000
    c.y = c.y[:t + 100]
    c.write("bounds")

    # N_CFO 128 and 4096, T_PK off and high
    for n_cfo, t_pk, name in ((128, 18 * 256, "n128"), (4096, 18 * 256, "n4096"), (2048, 0, "tpk0"),
                              (2048, 60 * 256, "tpk60")):
        c = Case(48000, n_cfo, t_pk, seed=n_cfo + t_pk)
        t = 500
        for k in range(4):
            lo, hi = ref.timing_window(t, n_cfo)
            inc = int(c.rng.integers(-(1 << 19), 1 << 19))
            c.preamble(int(c.rng.integers(lo + 40, hi - 40)), t, inc,
                       amp=float(c.rng.uniform(60, 160) if name == "tpk60" else c.rng.uniform(300, 3000)))
            c.event(t, inc)
            t += 8000
        if t_pk == 0:                                   # check off: even a silent window passes
            lo, hi = ref.timing_window(t, n_cfo)
            c.y[lo - (ref.PB_LEN - 1) - 10:hi + 10] = 0
            c.event(t, 99)
            t += 8000
        c.y = c.y[:t]
        c.write(name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
