#!/usr/bin/env python3
"""
make_chest_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Test vectors for tb_pl_chest.v (the payload channel estimate pl_chest.v) from the bit-true model
pl_reference.channel. Packets with real payloads (PC/pl_check.make_packet) of several
modulations, pilot patterns, channels and SNRs go through the channel model, 12-bit quantisation
and the hardware FFT (pl_check.hw_grid); the grid is the RTL input. Writes work/chest_units/:
  n.txt      number of cases
  cases.txt  per case: message (54 bits as hex, bit i = message bit i), PacketLength; the grid
             (PacketLength x 841 lines "Yr Yi"); the expected packet record "pk_a pk_b" (as
             pl_reference.chest_records); per symbol and tone "Hr Hi Y'r Y'i"

    python make_chest_vectors.py [--quick]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
import cf_reference as C                                            # noqa: E402
import pl_check as K                                                # noqa: E402
import pl_reference as P                                            # noqa: E402

# (mod, code, rep, RefSymbolSpacing, RefSignalSpacing, channel, SNR dB, PacketLength)
CASES = [
    ("QPSK", "648_12", 1, 3, 3, "AWGN", 0.0, 37),        # low SNR: smoothing T 5
    ("QPSK", "648_12", 1, 3, 3, "EVA-70", 25.0, 37),     # high SNR mode
    ("QPSK", "648_12", 1, 1, 3, "ETU-300", 3.0, 37),     # pilot every symbol, T 9
    ("BPSK", "648_12", 3, 6, 6, "EPA-5", -2.0, 37),      # T 3
    ("16QAM", "1296_23", 1, 12, 12, "EVA-70", 8.0, 37),  # R 12, S 12
    ("64QAM", "1944_34", 1, 3, 24, "AWGN", 30.0, 37),    # S 24
    ("16QAM", "648_12", 1, 6, 3, "ETU-300", 12.0, 49),
    ("QPSK", "648_12", 1, 3, 6, "EVA-70", 4.0, 4),       # PacketLength 4
    ("QPSK", "648_12", 1, 1, 12, "AWGN", 6.0, 37),
    ("64QAM", "648_56", 1, 12, 3, "ETU-300", 30.0, 73),
    ("16QAM", "648_12", 1, 3, 3, "EVA-70", 20.0, 37),    # grid x 12 below: packet shift k > 0
    ("QPSK", "648_12", 1, 3, 3, "EPA-5", 20.0, 37),      # grid x 10: k = 1
    ("QPSK", "648_12", 1, 3, 3, "EPA-5", 40.0, 37),      # high SNR: the -30 dB peak threshold decides
    ("QPSK", "648_12", 1, 3, 6, "EPA-5", 45.0, 37),
    ("64QAM", "648_12", 1, 3, 3, "EVA-70", 40.0, 37),
    ("QPSK", "648_12", 1, 3, 6, "TAP2-5", 50.0, 4),      # 2nd path at -28.5 dB: the -30 dB peak threshold sets class 2
]
# two paths, the second 2 us late at -28.5 dB (between the -30 dB peak threshold and -27 dB)
K.PROFILES["TAP2"] = ([0.0, 2.0e-6], [0.0, -28.5])
GRID_SCALE = {10: 12, 11: 10}                                    # case -> grid multiplier (clipped to 18 bit)
CASE_SEED = {15: 502}                                            # case -> random seed (default 100 + case)


def message(cfg: P.PLConfig) -> int:
    codes = {"RefSymbolSpacing": (1, 3, 6, 12).index(cfg.ref_sym),
             "RefSignalSpacing": (3, 6, 12, 24).index(cfg.ref_sig),
             "PacketLength": {v: k for k, v in C.CF_VALID["PacketLength"].items()}[cfg.plen],
             "PayloadFieldMod": P.MODS.index(cfg.mod), "NoOfTxAntenna": 0,
             "CBS_A_Flag": {v: k for k, v in C.CF_VALID["CBS_A_Flag"].items()}[cfg.n],
             "FEC_A_Flag": P.RATES.index(cfg.rate),
             "RM_A_Flag": {v: k for k, v in C.CF_VALID["RM_A_Flag"].items()}[cfg.rep],
             "NDB_A": cfg.ndb, "ClientFlag": 0, "Tx_Ref_Clk_Cnt": 0, "C_init": cfg.c_init, "Reserved": 0}
    m, pos = 0, 0
    for name, w in C.CF_FIELDS:
        for i in range(w):
            m |= ((codes[name] >> (w - 1 - i)) & 1) << (pos + i)
        pos += w
    assert C.field_codes(m) == codes and C.fields_valid(m)
    return m


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="the first 3 cases")
    a = ap.parse_args(argv)
    out = HERE / "work" / "chest_units"
    out.mkdir(parents=True, exist_ok=True)
    lines = []
    cases = CASES[:3] if a.quick else CASES
    for ci, (mod, code, rep, rs, rf, ch, snr, plen) in enumerate(cases):
        cfg = K.cfg_of(mod, code, rep, rs, rf, plen=plen)
        rng = np.random.default_rng(CASE_SEED.get(ci, 100 + ci))
        wave, _, _ = K.make_packet(cfg, rng)
        r, d = K.through_channel(wave, ch, snr, rng)
        xr, xi = K.quantise12(r)
        Yr, Yi = K.hw_grid(xr, xi, cfg.plen, d)
        if ci in GRID_SCALE:
            # plus a small random offset, so the low bits are not all zero (Y' rounding is seen)
            m = GRID_SCALE[ci]
            Yr = np.clip(Yr * m + rng.integers(0, m, Yr.shape), -131072, 131071)
            Yi = np.clip(Yi * m + rng.integers(0, m, Yi.shape), -131072, 131071)
        res = P.PLResult(cfg)
        Ypr, Ypi, Hr, Hi, sum_n, n0 = P.channel(Yr, Yi, cfg, res)
        msg = message(cfg)
        pk_a = res.k | (res.window << 8) | (int(res.hi) << 12) | (n0 << 16)
        lines.append(f"{msg:014x} {plen}\n")
        lines += [f"{int(Yr[s, c])} {int(Yi[s, c])}\n" for s in range(plen) for c in range(841)]
        lines.append(f"{pk_a} {sum_n & 0xFFFFFFFF}\n")
        lines += [f"{int(Hr[s, c])} {int(Hi[s, c])} {int(Ypr[s, c])} {int(Ypi[s, c])}\n"
                  for s in range(plen) for c in range(841)]
        print(f"case {ci}: {mod} {code} R {rs} S {rf} {ch} {snr:+.0f} dB PacketLength {plen}: "
              f"k {res.k}, class {res.window}, {'high' if res.hi else 'low'}-SNR mode")
    (out / "cases.txt").write_text("".join(lines))
    (out / "n.txt").write_text(f"{len(cases)}\n")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
