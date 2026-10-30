#!/usr/bin/env python3
"""Unit vectors for the FFT engines (RTL/pd_fft2.v, RTL/cf_fft.v) against cf_reference.fft_fixed.

work/fft_units/in.txt : per case 1024 lines "re im" (18-bit, already << FFT_IN_SHIFT)
work/fft_units/exp.txt: per case 1024 lines "re im" (natural bin order)
Cases: impulses, a tone, full-scale random (saturation in the last stages), small random,
12-bit de-rotator style input, alternating extremes."""
import pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "PC"))
import cf_reference as C

def cases(rng):
    n = C.FFT_N
    lim = 1 << 17
    out = []
    for k in (0, 1, 511, 1023):
        x = np.zeros(n, np.int64); x[k] = lim - 1; out.append((x, np.zeros(n, np.int64)))
    t = np.arange(n)
    out.append((np.round(60000 * np.cos(2 * np.pi * 37 * t / n)).astype(np.int64),
                np.round(60000 * np.sin(2 * np.pi * 37 * t / n)).astype(np.int64)))
    for _ in range(6):
        out.append((rng.integers(-lim, lim, n), rng.integers(-lim, lim, n)))
    for _ in range(4):
        out.append((rng.integers(-300, 300, n), rng.integers(-300, 300, n)))
    for _ in range(4):
        out.append((rng.integers(-2048, 2048, n) << 5, rng.integers(-2048, 2048, n) << 5))
    a = np.where(t % 2 == 0, lim - 1, -lim); out.append((a, np.clip(-a, -lim, lim - 1)))
    out.append((np.full(n, -lim), np.full(n, -lim)))
    return out

def main():
    d = pathlib.Path(__file__).resolve().parent / "work" / "fft_units"
    d.mkdir(parents=True, exist_ok=True)
    cs = cases(np.random.default_rng(5))
    with open(d / "in.txt", "w") as fi, open(d / "exp.txt", "w") as fe:
        for xr, xi in cs:
            yr, yi = C.fft_fixed(xr, xi, in_shift=0)
            for a, b in zip(xr, xi): fi.write(f"{a} {b}\n")
            for a, b in zip(yr, yi): fe.write(f"{a} {b}\n")
    (d / "n.txt").write_text(f"{len(cs)}\n")
    print(f"fft_units: {len(cs)} cases -> {d}")

if __name__ == "__main__":
    main()
