#!/usr/bin/env python3
"""
ptx_check.py  (Flexlink_py/HW_Development/Packet_Transmitter/PC)

Checks the transmitter model (ptx_reference.py) three ways:

  sim     against the simulator's last Tx run (Tx/*.h5 written by Tx/Run_Tx_Chain.py with
          Tx/Parameters_Tx_payload.json): the encoded and scrambled payload bits exactly, the
          payload grid and the control-field grid exactly (as constellation levels), and the
          12-bit waveform against the simulator's float waveform per segment (SQNR after a
          least-squares gain, which removes the power normalisation).
  refs    the pilots the hardware generates against all 32 ReferenceSignals/Ref_P*_*.h5 files
  ifft    the fixed-point IFFT against numpy's float IFFT on random symbols of every modulation
          and on the control field (SQNR, saturation count).
  rx      end to end: model packets for many settings (every modulation, code, rate, a range of
          repetitions, spacings and packet lengths) through the Zedboard receiver's bit-true model
          (Packet_Detector/PC/pd_reference.detect, input_stage 2 = 40 Msps, the way the Pluto
          delivers samples); every transport block must come back with CRC ok and equal to the
          one sent, and the control-field message must equal cf_message().

    python ptx_check.py                 # all three
    python ptx_check.py --only sim
    python ptx_check.py --only rx --cases 40 --seed 3

Exit status 0 when everything passes.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
PD_PC = ROOT / "HW_Development" / "Packet_Detector" / "PC"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PD_PC))
import ptx_reference as R  # noqa: E402

TX = ROOT / "Tx"


# --------------------------------------------------------------------------------------------
# reading the simulator's .h5 files (h5py, or the repository's minimal reader without it)

def h5_read(path: Path, name: str) -> np.ndarray:
    try:
        import h5py
        with h5py.File(path, "r") as f:
            return np.asarray(f[name][()])
    except ImportError:
        sys.path.insert(0, str(ROOT / "Flexlink_BasicModules" / "Equalizer" / "pilot_ce"))
        from minih5 import MiniH5
        h = MiniH5(str(path))
        return h.dataset(h.children(h.root_ohdr)[name])


def to_complex(x: np.ndarray) -> np.ndarray:
    if x.dtype.names:
        n = {k.lower(): k for k in x.dtype.names}
        return x[n.get("real") or n["r"]].astype(float) + 1j * x[n.get("imag") or n["i"]].astype(float)
    return x.astype(complex)


def sqnr_db(ref: np.ndarray, got: np.ndarray) -> tuple[float, complex]:
    """SQNR of got against ref after the least-squares complex gain g (got ~ g * ref)."""
    g = np.vdot(ref, got) / np.vdot(ref, ref)
    e = got - g * ref
    return 10 * np.log10(np.vdot(g * ref, g * ref).real / max(np.vdot(e, e).real, 1e-30)), g


def levels_of(grid: np.ndarray, mod: str) -> tuple[np.ndarray, np.ndarray]:
    s = np.sqrt(R.NORM[mod])
    li, lq = np.round(grid.real * s).astype(np.int64), np.round(grid.imag * s).astype(np.int64)
    assert np.abs(grid.real * s - li).max() < 1e-6 and np.abs(grid.imag * s - lq).max() < 1e-6
    return li, lq


# --------------------------------------------------------------------------------------------

def check_sim(lines: list[str]) -> bool:
    params = json.loads((TX / "Parameters_Tx_payload.json").read_text())
    cfg = R.TxConfig.from_params(TX / "Parameters_Tx_payload.json")
    cfg.check()
    calc = params["Calculated"]
    ok = True
    lines.append(f"Settings from Tx/Parameters_Tx_payload.json: {cfg}")
    for key, mine in (("TransportBlockSize", cfg.tb_size), ("AvailableDataBlocks", cfg.ndb),
                      ("ZeroPadding", cfg.zero_pad), ("NumberOfDataSigs", cfg.data_bits)):
        if int(calc[key]) != mine:
            lines.append(f"  {key}: simulator {calc[key]}, model {mine}  MISMATCH")
            ok = False
    tbs = h5_read(TX / "Generate_RandomPacketOfData" / "Bitstream_TransportBlks_Tx.h5",
                  "TransportBlock_Bitstream").ravel().astype(np.int64)
    enc = TX / "Bitstream_Encoded_Tx.h5"
    sim_scr = h5_read(enc, "data").ravel().astype(np.int64)
    sim_uns = h5_read(enc, "unscrambled").ravel().astype(np.int64)
    pb = R.payload_bits(cfg, tbs.reshape(cfg.ndb, cfg.tb_size))
    for name, a, b in (("unscrambled sequence", sim_uns, pb.unscrambled), ("scrambled sequence", sim_scr, pb.scrambled)):
        same = a.size == b.size and np.array_equal(a, b)
        lines.append(f"  {name}: {b.size} bits {'equal' if same else 'DIFFERENT'}")
        ok &= same
    code = R.code_of(cfg.code)
    synd = all(code.syndrome_ok(cw) for cw in pb.codewords)
    lines.append(f"  LDPC code words (H-based encoder): syndrome {'0' if synd else 'NOT 0'}")
    ok &= synd

    g = to_complex(h5_read(TX / "Grid_Payload_Tx.h5", "data"))
    si, sq = levels_of(g, cfg.mod)
    gi, gq = R.grid_levels(cfg.types(), pb.scrambled, cfg.mod, cfg.port, cfg.ref_sig)
    same = np.array_equal(si, gi) and np.array_equal(sq, gq)
    lines.append(f"  payload grid {g.shape}: levels {'equal' if same else 'DIFFERENT'}")
    ok &= same

    msg = R.cf_message(cfg)
    cfb = R.cf_bits(msg)
    cg = to_complex(h5_read(TX / "ControlSignals" / "Grid_DataControl_Tx.h5", "data"))
    ci, cq = levels_of(cg, "BPSK")
    mi, mq = R.grid_levels(R.CF_TYPES, cfb, "BPSK", 0, 3)
    same = np.array_equal(ci, mi) and np.array_equal(cq, mq)
    lines.append(f"  control field grid {cg.shape}: levels {'equal' if same else 'DIFFERENT'} "
                 f"(message {''.join(map(str, msg))})")
    ok &= same

    wav = to_complex(h5_read(TX / "Waveform_Payload_Tx.h5", "data")).ravel()
    p = R.packet(cfg, tbs.reshape(cfg.ndb, cfg.tb_size))
    x = p.i + 1j * p.q
    if wav.size != x.size:
        lines.append(f"  waveform: simulator {wav.size} samples, model {x.size}  MISMATCH "
                     "(Use_Preamble / Use_ControlField must be 1)")
        return False
    seg = {"preamble": slice(0, R.PRE_LEN), "control field": slice(R.PRE_LEN, R.PRE_LEN + R.CF_LEN),
           "payload": slice(R.PRE_LEN + R.CF_LEN, None)}
    gains = {}
    for name, sl in seg.items():
        s, gg = sqnr_db(wav[sl], x[sl])
        gains[name] = gg
        good = s > (40 if name == "preamble" else 50)
        lines.append(f"  waveform {name:<13}: SQNR {s:5.1f} dB vs the simulator's float samples "
                     f"(gain {abs(gg):.2f}, phase {np.degrees(np.angle(gg)):+.3f} deg){'' if good else '  LOW'}")
        ok &= good
    rel = {k: 20 * np.log10(abs(v) / abs(gains["payload"])) for k, v in gains.items()}
    lines.append(f"  segment levels relative to the payload: preamble {rel['preamble']:+.3f} dB, "
                 f"control field {rel['control field']:+.3f} dB (simulator: equal power, measured)")
    ok &= abs(rel["preamble"]) < 0.1 and abs(rel["control field"]) < 0.1
    clipped = int(np.count_nonzero((np.abs(p.i) >= 2048) | (np.abs(p.q) >= 2048) | (p.i == 2047) | (p.q == 2047)))
    lines.append(f"  12-bit output: rms {np.sqrt(np.mean(np.abs(x) ** 2)):.1f}, gains {p.gains}, "
                 f"{clipped} samples at full scale")
    return ok


def check_refs(lines: list[str]) -> bool:
    """The pilots the hardware generates (Gold(C_init_ID) and the corner rule) against every
    ReferenceSignals/Ref_Pz_X_Y.h5 the simulator uses."""
    cfg = R.TxConfig.from_params(TX / "Parameters_Tx_payload.json")
    ok, n = True, 0
    mods = {1: "BPSK", 2: "QPSK", 4: "16QAM", 6: "64QAM"}
    for z, cid in ((0, cfg.id1), (1, cfg.id2)):
        for S in (3, 6, 12, 24):
            for Y, mod in mods.items():
                f = ROOT / "ReferenceSignals" / f"Ref_P{z}_{S}_{Y}.h5"
                ref = to_complex(h5_read(f, "data")).ravel()
                li, lq = R.pilot_levels(R.pilot_bits(cid, S), mod)
                mine = (li + 1j * lq) / np.sqrt(R.NORM[mod])
                same = ref.size == mine.size and np.abs(ref - mine).max() < 1e-9
                ok &= same
                n += 1
                if not same:
                    lines.append(f"  {f.name}: DIFFERENT")
    lines.append(f"  {n} reference-signal files (C_init_ID1 {cfg.id1}, C_init_ID2 {cfg.id2}): "
                 f"{'all equal to the generated pilots' if ok else 'MISMATCH'}")
    return ok


def check_ifft(lines: list[str], rng) -> bool:
    ok = True
    for mod in ("BPSK", "QPSK", "16QAM", "64QAM"):
        cfg = R.TxConfig(mod=mod, ref_sym=1, ref_sig=3, plen=37, n=648, rate="12", rep=1)
        bits = rng.integers(0, 2, cfg.data_bits)
        gi, gq = R.grid_levels(cfg.types(), bits, mod, 0, 3)
        xr, xi = R.ifft_input(gi, gq, R.AMP[mod])
        yr, yi = R.ifft_fixed(xr, xi)
        ref = np.fft.ifft(xr + 1j * xi, axis=1) * R.N_FFT / (1 << sum(R.SHIFTS))
        y = yr + 1j * yi
        s, g = sqnr_db(ref.ravel(), y.ravel())
        sat = int(np.count_nonzero(np.abs(np.concatenate([yr.ravel(), yi.ravel()])) >= (1 << (R.W - 1)) - 1))
        peak = 20 * np.log10(np.abs(np.concatenate([yr.ravel(), yi.ravel()])).max() / (1 << (R.W - 1)))
        good = s > 70 and abs(abs(g) - 1) < 1e-3 and sat == 0
        lines.append(f"  {mod:<5} 37 symbols: SQNR {s:5.1f} dB, gain {abs(g):.5f}, peak {peak:+.1f} dBFS (18 bit), "
                     f"{sat} saturated{'' if good else '  FAIL'}")
        ok &= good
    return ok


def rx_cases(n: int, rng) -> list[R.TxConfig]:
    fixed = [R.TxConfig(),
             R.TxConfig(mod="BPSK", ref_sym=3, ref_sig=6, plen=37, n=648, rate="12", rep=3, c_init=7),
             R.TxConfig(mod="16QAM", ref_sym=6, ref_sig=12, plen=49, n=1944, rate="34", rep=1, c_init=200),
             R.TxConfig(mod="64QAM", ref_sym=3, ref_sig=3, plen=37, n=1944, rate="56", rep=1, c_init=0),
             R.TxConfig(mod="64QAM", ref_sym=12, ref_sig=24, plen=121, n=1296, rate="23", rep=1, c_init=255),
             R.TxConfig(mod="QPSK", ref_sym=1, ref_sig=3, plen=97, n=648, rate="34", rep=31, c_init=33)]
    out = list(fixed)
    while len(out) < max(n, len(fixed)):
        c = R.TxConfig(mod=str(rng.choice(list(R.BITS))), ref_sym=int(rng.choice([1, 3, 6, 12])),
                       ref_sig=int(rng.choice([3, 6, 12, 24])), plen=int(rng.choice(R.PACKET_LENGTHS)),
                       n=int(rng.choice([648, 1296, 1944])), rate=str(rng.choice(list(R.RATE_FRAC))),
                       rep=int(rng.choice(R.REPETITIONS)), c_init=int(rng.integers(0, 256)))
        if (c.plen - 1) % c.ref_sym == 0 and c.ndb >= 1 and c.ndb < 1024:
            out.append(c)
    return out[:max(n, len(fixed))]


def check_rx(lines: list[str], rng, n_cases: int) -> bool:
    import pd_protocol as pr
    import pd_reference as ref
    ok = True
    params = pr.Params(input_stage=2)
    for cfg in rx_cases(n_cases, rng):
        tbs = R.random_tbs(cfg, rng)
        p = R.packet(cfg, tbs)
        noise = lambda k: np.round(rng.normal(0, 1.5, (k, 2))).astype(np.int64)   # noqa: E731
        iq = np.concatenate([noise(6000), p.iq.astype(np.int64), noise(6000)])
        iq = iq[:iq.shape[0] // 2 * 2]
        res = ref.detect(iq, params)
        msg = int(sum(int(b) << i for i, b in enumerate(R.cf_message(cfg))))
        cf_ok = len(res.control) == 1 and res.control[0].status == 1 and res.control[0].message == msg
        got = {r.w: r for r in res.tb}
        good = sum(1 for w in range(cfg.ndb) if w in got and got[w].crc_ok
                   and np.array_equal(np.array(got[w].bits), tbs[w]))
        passed = cf_ok and good == cfg.ndb and len(got) == cfg.ndb
        lines.append(f"  {cfg.mod:<5} N {cfg.n:4d} R {cfg.rate} rep {cfg.rep:2d} sym {cfg.ref_sym:2d} sig {cfg.ref_sig:2d} "
                     f"len {cfg.plen:3d} C_init {cfg.c_init:3d}: control field {'ok' if cf_ok else 'FAIL'}, "
                     f"{good}/{cfg.ndb} blocks equal{'' if passed else '  FAIL'}")
        ok &= passed
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check the transmitter model")
    ap.add_argument("--only", choices=("sim", "refs", "ifft", "rx"))
    ap.add_argument("--cases", type=int, default=24, help="rx: number of settings (6 fixed + random)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    ok = True
    for name, fn in (("sim", lambda l: check_sim(l)), ("refs", lambda l: check_refs(l)),
                     ("ifft", lambda l: check_ifft(l, rng)),
                     ("rx", lambda l: check_rx(l, rng, a.cases))):
        if a.only and a.only != name:
            continue
        lines: list[str] = []
        try:
            res = fn(lines)
        except FileNotFoundError as exc:
            lines.append(f"  skipped: {exc}")
            res = name != "sim"
        print(f"[{name}] {'PASS' if res else 'FAIL'}")
        print("\n".join(lines))
        ok &= res
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
