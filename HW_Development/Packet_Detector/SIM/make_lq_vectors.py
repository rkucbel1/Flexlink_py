#!/usr/bin/env python3
"""
make_lq_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Test vectors for tb_pl_soft.v (the payload demapper pl_demap.v and combiner pl_comb.v) from the
bit-true model (pl_reference.channel, soft_bits, combine). Real packets (PC/pl_check generator)
of every modulation, code size, copy count, pilot pattern and SNR, through the channel model,
12-bit quantisation and the hardware FFT; plus a packet whose noise tones are zero (sumN = 0) and
one with NDB_A = 0. Writes work/lq_units/:
  n.txt      number of packets
  cases.txt  per packet: message (hex) t_trig n0 sumN PacketLength RefSymbolSpacing; then per
             symbol and tone "Y'r Y'i Hr Hi" (what pl_chest sends); then "nblk N" and per code word
             its N decoder inputs Lq (code order) on one line, then "sig_a sig_b", then the
             transport block (pl_reference.decode_ldpc, crc24): "crc_ok iterations nbits" and
             ceil(nbits / 32) words (hex, bit j in word j / 32)

    python make_lq_vectors.py [--quick]
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
sys.path.insert(0, str(HERE))
import cf_reference as C                                            # noqa: E402
import pl_check as K                                                # noqa: E402
import pl_reference as P                                            # noqa: E402
from make_chest_vectors import message                              # noqa: E402

# (mod, code, rep, RefSymbolSpacing, RefSignalSpacing, channel, SNR dB, PacketLength, NDB_A or None
#  = what fits, C_init)
CASES = [
    ("BPSK", "648_12", 1, 3, 3, "EPA-5", 3.0, 37, 4, 129),
    ("QPSK", "1296_23", 3, 6, 6, "EVA-70", 2.0, 37, None, 7),
    ("16QAM", "1944_34", 1, 12, 12, "ETU-300", 8.0, 49, 5, 255),
    ("64QAM", "648_56", 1, 3, 24, "AWGN", 16.0, 37, 6, 0),
    ("64QAM", "1944_12", 1, 1, 3, "EVA-70", 15.0, 4, 10, 77),    # NDB_A more than fits
    ("QPSK", "648_12", 31, 3, 3, "AWGN", -6.0, 73, 2, 200),      # 32 copies, low SNR: saturation
    ("16QAM", "648_12", 3, 3, 6, "EPA-5", -3.0, 37, None, 33),   # LLRs and sums saturate
    ("BPSK", "1296_12", 5, 1, 12, "AWGN", 40.0, 37, None, 1),    # high SNR, R 1
    ("64QAM", "1296_34", 1, 6, 3, "EPA-5", 18.0, 49, None, 128),
    ("QPSK", "648_12", 1, 3, 3, "AWGN", 10.0, 37, 0, 5),         # NDB_A 0: no code word
    ("16QAM", "1296_56", 1, 3, 3, "AWGN", 20.0, 37, 3, 99),      # weak, noise tones zeroed: sumN 0
    ("QPSK", "648_12", 31, 3, 3, "AWGN", 12.0, 73, 2, 9),        # 32 copies, high SNR: sums saturate
    ("64QAM", "1944_23", 1, 3, 3, "EPA-5", 16.0, 37, 4, 41),     # the codes not used above
    ("16QAM", "648_34", 1, 6, 12, "AWGN", 9.0, 37, 6, 42),
    ("QPSK", "1944_56", 1, 3, 6, "EVA-70", 9.0, 37, 3, 43),
    ("BPSK", "648_23", 3, 3, 24, "AWGN", 4.0, 37, None, 44),
]
ZERO_NOISE = {10}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="the first 3 packets")
    a = ap.parse_args(argv)
    out = HERE / "work" / "lq_units"
    out.mkdir(parents=True, exist_ok=True)
    lines = []
    cases = CASES[:3] if a.quick else CASES
    for ci, (mod, code, rep, rs, rf, ch, snr, plen, ndb, cinit) in enumerate(cases):
        cfg = K.cfg_of(mod, code, rep, rs, rf, plen=plen, c_init=cinit)
        rng = np.random.default_rng(300 + ci)
        wave, _, _ = K.make_packet(cfg, rng)
        if ndb is not None:
            cfg = dataclasses.replace(cfg, ndb=ndb)
        r, d = K.through_channel(wave, ch, snr, rng)
        xr, xi = K.quantise12(r)
        Yr, Yi = K.hw_grid(xr, xi, cfg.plen, d)
        if ci in ZERO_NOISE:                       # a weak grid, noise tones of rows 0 and 1 exactly zero
            Yr, Yi = Yr // 512, Yi // 512
            t0 = P._T[f"ref_row_{rf}"]
            nz = (t0 == P.REF_P1) | ((t0 == P.DC) & (np.arange(841) >= 1) & (np.arange(841) <= 839))
            for row in (0, rs):
                Yr[row, nz] = 0
                Yi[row, nz] = 0
        res = P.PLResult(cfg)
        lf = P.soft_bits(Yr, Yi, cfg, res)
        Ypr, Ypi, Hr, Hi, sum_n, n0 = P.channel(Yr, Yi, cfg)
        llr = P.combine(lf, cfg)
        ldc = P.code_of(cfg.code)
        hard, used = P.decode_ldpc(ldc, llr) if llr.shape[0] else (None, None)
        n_ok = 0
        msg = message(cfg)
        t = 5000 + ci
        lines.append(f"{msg:014x} {t} {n0} {sum_n} {plen} {rs}\n")
        lines += [f"{int(Ypr[s, c])} {int(Ypi[s, c])} {int(Hr[s, c])} {int(Hi[s, c])}\n"
                  for s in range(plen) for c in range(841)]
        lines.append(f"{llr.shape[0]} {cfg.n}\n")
        for w, q in enumerate(llr):
            lines.append(" ".join(str(int(v)) for v in q) + "\n")
            sa, sb = C.grid_signature(q[0::2], q[1::2])
            lines.append(f"{sa} {sb}\n")
            kk = ldc.K
            msg = hard[w, :kk]
            ok = P.crc24(msg[:kk - 24]) == sum(int(b) << (23 - i) for i, b in enumerate(msg[kk - 24:]))
            n_ok += ok
            nb = kk - 24
            words = [0] * ((nb + 31) // 32)
            for j in range(nb):
                words[j // 32] |= int(msg[j]) << (j % 32)
            lines.append(f"{int(ok)} {int(used[w])} {nb} " + " ".join(f"{x:08x}" for x in words) + "\n")
        nbits = lf.size
        print(f"packet {ci}: {mod} {code} copies {rep + 1} R {rs} S {rf} {ch} {snr:+.0f} dB PacketLength "
              f"{plen}: {nbits} LLRs, m {res.m}, sh {res.sh}, {llr.shape[0]} code word(s) ({n_ok} CRC ok), "
              f"Lq saturated {np.mean(np.abs(llr) == 63) * 100 if llr.size else 0:.0f} %")
    (out / "cases.txt").write_text("".join(lines))
    (out / "n.txt").write_text(f"{len(cases)}\n")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
