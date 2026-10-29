#!/usr/bin/env python3
"""
make_cf_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Vectors for the control-field receiver alone (tb_cf_rx.v -> RTL/cf_rx.v) with every
intermediate of the bit-true model (PC/cf_reference.receive_debug). Each case is the 4560
samples y(n_end + 1 ...) of the DC blocker output and a cfo_inc:

  rep3_p1..p3   the three packets of TestVectors/rep3 (ETU, -8 dB, CW and DC defects), with
                n_end and cfo_inc from the bit-true detector (input stage 1)
  noisy_p1..p3  the same with noise added (0.7, 1.4, 2 x the signal power: CRC passes and fails)
  loud          rep3 packet 1 x 8, clipped to 12 bit (de-rotator saturation, large FFT values)
  zeros         all zero: phi 0, shift 0 and a Viterbi made of ties only
  fullscale     uniform random full-scale samples, random cfo_inc
  tone          full-scale complex tone on one bin (largest FFT output)
  cfo_max       rep3 packet 2 with cfo_inc = 0x800000 (half the sample rate)
  max_first / max_last   synthetic control fields (flat channel, made from a chosen grid) whose
                soft values are large only for code-bit accumulator 0 / 139, so the largest
                |A| (and the Viterbi scaling) comes from the first / last accumulator

  work/cf_<case>/cfg.txt       cfo_inc phi shift metric crc_ok
  work/cf_<case>/in.txt        4560 lines "iii qqq" (12-bit hex)
  work/cf_<case>/exp_grid.txt  4 x 841 lines "rrrrr iiiii" (18-bit hex), symbol by symbol
  work/cf_<case>/exp_hf.txt    841 lines Hf0 re, im, H3' re, im (20-bit hex)
  work/cf_<case>/exp_v.txt     2200 lines: descrambled soft value (46-bit hex)
  work/cf_<case>/exp_bits.txt  the 70 decoded bits, bit 0 first

  work/cf_units/vit.txt        Viterbi + CRC unit cases (tb_cf_viterbi.v): random soft values
                               (small ones: mostly ties), encoded messages with noise, all zero

    python make_cf_vectors.py
"""
from __future__ import annotations

import json
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
import cf_reference as cf  # noqa: E402
import pd_protocol as pr  # noqa: E402
import pd_reference as ref  # noqa: E402

TV = HERE.parent / "TestVectors"


def hx(v, bits):
    return f"{int(v) & ((1 << bits) - 1):0{(bits + 3) // 4}x}"


def write_case(name: str, y: np.ndarray, cfo_inc: int) -> dict:
    y = np.asarray(y, np.int64)
    assert y.shape == (cf.CF_LEN, 2) and np.abs(y).max() <= 2048 and y.min() >= -2048 and y.max() <= 2047
    d = cf.receive_debug(y, cfo_inc)
    w = HERE / "work" / f"cf_{name}"
    w.mkdir(parents=True, exist_ok=True)
    (w / "cfg.txt").write_text(f"{cfo_inc & 0xFFFFFF} {d['phi']} {d['shift']} {d['metric']} {d['crc_ok']}\n")
    (w / "in.txt").write_text("".join(f"{hx(a, 12)} {hx(b, 12)}\n" for a, b in y))
    (w / "exp_grid.txt").write_text("".join(f"{hx(d['Yr'][s, c], 18)} {hx(d['Yi'][s, c], 18)}\n"
                                            for s in range(4) for c in range(841)))
    h = d["hf"]
    (w / "exp_hf.txt").write_text("".join(" ".join(hx(h[i][k], 20) for i in range(4)) + "\n" for k in range(841)))
    (w / "exp_v.txt").write_text("".join(f"{hx(v, 46)}\n" for v in d["v"]))
    (w / "exp_bits.txt").write_text("".join(str(b) for b in d["bits"]) + "\n")
    for arr, bits in ((d["Yr"], 18), (d["Yi"], 18), (np.concatenate(h), 20), (d["v"], 46)):
        assert np.abs(arr).max() < (1 << (bits - 1)), name
    print(f"cf_{name}: cfo_inc {cfo_inc & 0xFFFFFF:06x}, phi {d['phi']}, shift {d['shift']}, "
          f"metric {d['metric']}, crc {'ok' if d['crc_ok'] else 'FAIL'}")
    return d


def rep3():
    meta = json.loads((TV / "rep3.json").read_text())
    known = {f.name for f in fields(pr.Params)}
    p = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
    p.input_stage = 1
    iq = np.fromfile(TV / meta["files"]["decimated"], "<i2").reshape(-1, 2).astype(np.int64)
    r = ref.detect(iq, p)
    y = ref.dc_blocker(iq, p.dc_shift)
    out = []
    for tm in r.timing:
        if tm.status == pr.TM_GOOD:
            ev = [e for e in r.events if e.t_trig == tm.t_trig][0]
            out.append((y[tm.n_end + 1:tm.n_end + 1 + cf.CF_LEN], ev.cfo_inc))
    return out


def conv_encode(bits):
    """K = 7 [133, 171] encoder (state = last 6 inputs, newest in the MSB), code bits A0 B0 ..."""
    st, out = 0, []
    for b in bits:
        oa, ob = cf._OUT[st][b]
        out += [oa, ob]
        st = (b << 5) | (st >> 1)
    return out


def viterbi_units(rng, n=400):
    lines = []
    for k in range(n):
        kind = k % 4
        if kind == 0:
            q = rng.integers(-16, 16, 140)
        elif kind == 1:
            q = rng.integers(-1, 2, 140)                         # tiny: ties everywhere
        elif kind == 2:                                        # a valid code word with noise
            msg = list(rng.integers(0, 2, 54))
            crc = cf.crc10(msg)
            data = msg + [(crc >> (9 - i)) & 1 for i in range(10)]
            assert cf.crc10(data) == 0
            c = np.array(conv_encode(data + [0] * 6))
            q = np.clip(np.round((2 * c - 1) * 6 + rng.standard_normal(140) * 6), -16, 15).astype(np.int64)
        else:
            q = np.zeros(140, np.int64) if k == 3 else rng.integers(-16, 16, 140) * (rng.random(140) < 0.2)
        bits, metric = cf.viterbi(q)
        ok = int(cf.crc10(bits[:64]) == 0)
        lines.append(" ".join(hx(v, 5) for v in q) + f" {''.join(map(str, bits))} {metric} {ok}")
    w = HERE / "work" / "cf_units"
    w.mkdir(parents=True, exist_ok=True)
    (w / "vit.txt").write_text(f"{n}\n" + "\n".join(lines) + "\n")
    print(f"cf_units: {n} Viterbi cases")


def synthetic(j_big: int, rng) -> np.ndarray:
    """4560 samples from a grid: pilots = ref, data tone i = +-1 (after descrambling) times 1 for
    i mod 140 == j_big, 1/16 otherwise; CP 116; about -12 dBFS."""
    grid = np.zeros((4, 841), complex)
    grid[0, cf.CF_P0_K] = cf.CF_REF
    grid[3, cf.CF_P0_K] = cf.CF_REF
    i = np.arange(2200)
    amp = np.where(i % 140 == j_big, 1.0, 1 / 16) * rng.choice([-1, 1], 2200) ** (i % 140 != j_big)
    amp = amp * np.where(cf.CF_GOLD == 1, -1, 1)
    grid[cf.CF_DATA_MASK] = amp
    x = []
    for s_ in range(4):
        spec = np.zeros(1024, complex)
        spec[(np.arange(841) - 420) % 1024] = grid[s_]
        t = np.fft.ifft(spec)
        x.append(np.r_[t[-cf.CF_CP:], t])
    x = np.concatenate(x)
    x *= 2047 * 10 ** (-12 / 20) / np.sqrt(np.mean(np.abs(x) ** 2))
    return np.clip(np.round(np.stack([x.real, x.imag], 1)), -2048, 2047).astype(np.int64)


def main() -> int:
    rng = np.random.default_rng(20261029)
    viterbi_units(rng)
    pk = rep3()
    if len(pk) != 3:
        raise SystemExit(f"rep3: {len(pk)} good timing records, expected 3")
    for i, (y, inc) in enumerate(pk):
        write_case(f"rep3_p{i + 1}", y, inc)
    for i, ((y, inc), g) in enumerate(zip(pk, (0.7, 1.4, 2.0))):
        sig = float(np.mean(y.astype(float) ** 2))
        n = np.round(rng.standard_normal(y.shape) * np.sqrt(sig * g)).astype(np.int64)
        write_case(f"noisy_p{i + 1}", np.clip(y + n, -2048, 2047), inc)
    write_case("loud", np.clip(pk[0][0] * 8, -2048, 2047), pk[0][1])
    write_case("zeros", np.zeros((cf.CF_LEN, 2), np.int64), 0)
    write_case("fullscale", rng.integers(-2048, 2048, (cf.CF_LEN, 2)), int(rng.integers(0, 1 << 24)))
    n = np.arange(cf.CF_LEN)
    ph = 2 * np.pi * 37 * n / 1024
    write_case("tone", np.stack([np.round(2047 * np.cos(ph)), np.round(2047 * np.sin(ph))], 1).astype(np.int64), 0)
    write_case("cfo_max", pk[1][0], 0x800000)
    for name, j in (("max_first", 0), ("max_last", 139)):
        d = write_case(name, synthetic(j, rng), 0)
        A = d["v"][:2100].reshape(15, 140).sum(0)
        others = np.delete(np.abs(A), j).max()
        assert int(abs(A[j])).bit_length() > int(others).bit_length(), name
    return 0


if __name__ == "__main__":
    sys.exit(main())
