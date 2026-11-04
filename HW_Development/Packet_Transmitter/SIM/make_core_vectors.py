#!/usr/bin/env python3
"""make_core_vectors.py - ptx_core test vectors from ptx_reference (SIM/work/core/).

  words.txt  the input stream, one 64-bit word per line (hex): records = parameter header + blocks
  exp.txt    every output sample: "i q active" (packets and their gaps, in order)
  cfg.txt    words, samples, seed, input-gap and output-ready settings, packets, header errors

Cases (argument 1):
  json     Tx/Parameters_Tx_payload.json as it is (the simulator's settings), 2 packets
  sim      Tx/Parameters_Tx_payload.json with the simulator's own transport blocks
           (Tx/Generate_RandomPacketOfData/Bitstream_TransportBlks_Tx.h5), the stream ptx_stream.py
           writes for them
  mix      two garbage words (one with the magic but a wrong version), then records: BPSK 648 1/2;
           64QAM 1944 5/6 port 1 with other Gold IDs (payload and control field), ClientFlag, Tx_Ref_Clk_Cnt and gain
           overrides; a parameters-only record; a rejected header (Repetition 2); 16QAM without
           the preamble; QPSK without the control field
  stress   the encoder's tightest cases: 64QAM with 2 copies at 648 5/6 and 1944 5/6
  big      64QAM 1944 1/2, 121 symbols, RefSymbolSpacing 1 (the most data REs)
  sweep    one record per code (all 12 EncoderBlockSize x EncoderCodeRate), random other settings
           (Mod, spacings, PacketLength 37 / 49, Repetition, port, IDs, flags), short gaps
"""
import sys
from dataclasses import replace
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
ROOT = next(p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file())
sys.path.insert(0, str(HERE.parent / "PC"))
import ptx_reference as R

C = R.TxConfig
CASES = {
    "json": [(None, 2, 4096, None)],
    "mix": [(C(mod="BPSK", ref_sym=3, ref_sig=6, plen=37, n=648, rate="12", rep=3, c_init=7), 2, 32, None),
            (C(mod="64QAM", ref_sym=12, ref_sig=24, plen=37, n=1944, rate="56", rep=1, c_init=0, port=1,
               id1=0x1234567, id2=77, client=1, clk_cnt=0x3FFF,
               cf_id1=0x55AA55, cf_cdat=7), 1, 0, {"pre": 3000, "cf": 0, "pl": 3900}),
            (C(mod="QPSK"), 0, 0, None),
            ("bad", 0, 0, None),
            (C(mod="16QAM", ref_sym=1, ref_sig=12, plen=37, n=1296, rate="34", rep=5, c_init=200, use_pre=0), 1, 100, None),
            (C(mod="QPSK", ref_sym=6, ref_sig=3, plen=49, n=1296, rate="23", rep=7, c_init=129, use_cf=0), 1, 16, None)],
    "stress": [(C(mod="64QAM", ref_sym=12, ref_sig=24, plen=37, n=648, rate="56", rep=1, c_init=3), 1, 0, None),
               (C(mod="64QAM", ref_sym=12, ref_sig=24, plen=37, n=1944, rate="56", rep=1, c_init=4), 1, 0, None)],
    "big": [(C(mod="64QAM", ref_sym=1, ref_sig=3, plen=121, n=1944, rate="12", rep=1, c_init=77), 1, 64, None)],
}

def sweep(rng):
    out = []
    for n in (648, 1296, 1944):
        for rate in ("12", "23", "34", "56"):
            while True:
                c = C(mod=str(rng.choice(list(R.BITS))), ref_sym=int(rng.choice([1, 3, 6, 12])),
                      ref_sig=int(rng.choice([3, 6, 12, 24])), plen=int(rng.choice([37, 49])), n=n, rate=rate,
                      rep=int(rng.choice(R.REPETITIONS)), port=int(rng.integers(0, 2)),
                      c_init=int(rng.integers(0, 256)), id1=int(rng.integers(0, 1 << 31)),
                      id2=int(rng.integers(0, 1 << 31)), client=int(rng.integers(0, 2)),
                      clk_cnt=int(rng.integers(0, 1 << 14)), use_pre=int(rng.random() < 0.8),
                      cf_id1=int(rng.integers(0, 1 << 31)), cf_cdat=int(rng.integers(0, 1 << 31)),
                      use_cf=int(rng.random() < 0.8))
                if 1 <= c.ndb <= 1023 and c.ndb * c.tb_size < 60000:
                    break
            out.append((c, 1, int(rng.integers(0, 64)), None))
    return out


def main(case="mix", seed=1, in_gap=3, out_mode=1):
    rng = np.random.default_rng(int(seed))
    if case == "sweep":
        CASES["sweep"] = sweep(rng)
    if case == "sim":
        import ptx_stream
        cfg = R.TxConfig.from_params(ROOT / "Tx" / "Parameters_Tx_payload.json")
        blocks = ptx_stream.packets_from_bits(cfg, ptx_stream.load_blocks(
            ROOT / "Tx" / "Generate_RandomPacketOfData" / "Bitstream_TransportBlks_Tx.h5"))
        CASES["sim"] = [(cfg, blocks, 20000, None)]
    words, samples, n_pk, n_err = [], [], 0, 0
    if case == "mix":
        words.append(np.array([0x1234, R.MAGIC | (1 << 32)], np.uint64))   # version 1: rejected
        n_err += 2
    for cfg, k, gap, gain in CASES[case]:
        if cfg is None:
            cfg = R.TxConfig.from_params(ROOT / "Tx" / "Parameters_Tx_payload.json")
        if isinstance(cfg, str):                          # header with Repetition 2: rejected
            h = R.param_header(R.TxConfig(), 0)
            h[1] = (int(h[1]) & ~(0x1F << 27)) | (2 << 27)
            words.append(h)
            n_err += 1
            continue
        tbs = k if isinstance(k, list) else [R.random_tbs(cfg, rng) for _ in range(k)]
        k = len(tbs)
        words.append(R.record_words(cfg, tbs, gap, gain))
        for p in R.record_packets(cfg, tbs, gain):
            samples.append(R.stream_samples(p, gap))
        n_pk += k
    w = np.concatenate(words)
    s = np.concatenate(samples)
    out = HERE / "work" / "core"
    out.mkdir(parents=True, exist_ok=True)
    (out / "words.txt").write_text("\n".join(f"{int(x):016x}" for x in w) + "\n")
    np.savetxt(out / "exp.txt", s, fmt="%d")
    (out / "cfg.txt").write_text(f"{w.size} {s.shape[0]} {int(seed)} {int(in_gap)} {int(out_mode)} {n_pk} {n_err}\n")
    print(f"{out}: case {case}, {n_pk} packet(s), {w.size} words, {s.shape[0]} samples, {n_err} header errors")

if __name__ == "__main__":
    main(*sys.argv[1:])
