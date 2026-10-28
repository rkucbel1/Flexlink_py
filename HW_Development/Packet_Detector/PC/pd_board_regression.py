#!/usr/bin/env python3
"""
pd_board_regression.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Regression of the detector on the Zedboard: a set of test cases, each sent to the board with
the normal protocol (RESET, SET_PARAMS, LOAD_SAMPLES, RUN in one or more pieces, GET_DEBUG),
and the same command sequence run through pd_board_emulator's Board in this process (the
bit-true model with the board's RUN semantics). Every RUN reply must match: the events
(t_trig, status, cfo_inc, cfo_valid_cnt, angles exact; ToneCheck within --tone-tol, the abort
decision exact), the Preamble B timing records (every field exact; firmware build 10 and
bitstream 3.0, input stages 1 and 2) and, for cases with debug, every debug record.

Cases (the RTL simulation's coverage, on the board):
  rep3 / defects     real packets from TestVectors/, all three input stages (s2 = 40 Msps,
                     the board's half-band; needs vectors rebuilt with pd_make_vector.py), debug
  cfo                40 real packets, CFO +-600 kHz, SNR -6..30 dB, at 20 and 40 Msps (needs
                     h5py and the simulator project, as pd_bittrue_check.py)
  highsnr            real packets at +40, +55, +60, +65 dB and noise-free: near-silent idle,
                     where the power floor acts (20 and 40 Msps)
  stress             synthetic full-scale vector: saturation, den = 0, many triggers;
                     holdoff 1, N_CFO 128 / 4096, tone check off / 24 dB, DC blocker off / short
  pieces             rep3 in uneven RUNs, with windows split across RUNs (truncated events)
  noise_k            noise only, 5 x 0.2 s: false-trigger rate (they should all fail the tone check)

    python pd_board_regression.py                         # board at 192.168.50.10 (33 cases)
    python pd_board_regression.py --only stress,pieces    # some cases (substring match)
    python pd_board_regression.py --quick                 # one short noise record
    python pd_board_regression.py --list

Exit status 0 = all PASS, 1 = a mismatch, 2 = communication error.
Report: TestVectors/regression_result.json
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from dataclasses import astuple
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "SIM"))
import pd_protocol as pr  # noqa: E402
import pd_reference as ref  # noqa: E402
import pd_board_emulator as emu  # noqa: E402
import pd_hw_test as hw  # noqa: E402

DEFAULT_HOST = hw.DEFAULT_HOST


# --------------------------------------------------------------------------------------- #
# Test cases                                                                               #
# --------------------------------------------------------------------------------------- #
def vector_from_json(name: str, stage: int):
    meta = json.loads((ROOT / "TestVectors" / f"{name}.json").read_text())
    key = {0: "lpf_out", 1: "decimated", 2: "input40"}[stage]
    if key not in meta["files"]:
        raise FileNotFoundError(f"{name}.json has no {key} file - rebuild it with pd_make_vector.py")
    f = meta["files"][key]
    iq = np.fromfile(ROOT / "TestVectors" / f, "<i2").reshape(-1, 2)
    known = set(pr.Params.__dataclass_fields__)
    p = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
    p.input_stage = stage
    return iq, p


def noise_vector(n: int, seed: int = 5) -> np.ndarray:
    """Complex white noise at -12 dBFS rms per rail, through the LPF (input stage 0)."""
    rng = np.random.default_rng(seed)
    x = np.clip(np.round(rng.normal(0, 512, (n, 2))), -2048, 2047).astype(np.int64)
    return ref.lpf(x).astype(np.int16)


def cases(quick: bool):
    """Yield (name, description, iq, params, run sizes, debug). Built lazily."""
    import make_tb_vectors as mk

    def p_of(**kw):
        p = pr.Params()
        for k, v in kw.items():
            setattr(p, k, v)
        return p

    for stage in (0, 1, 2):
        yield (f"rep3_s{stage}", "3 real packets" + (" at 40 Msps" if stage == 2 else ""),
               lambda s=stage: vector_from_json("rep3", s), None, True)
    yield ("defects_s1", "Tx defects file", lambda: vector_from_json("Waveform_Payload_DefectsModeled_Tx", 1), None, True)
    yield ("defects_s2", "Tx defects file at 40 Msps", lambda: vector_from_json("Waveform_Payload_DefectsModeled_Tx", 2), None, True)

    def cfo(stage):
        iq = mk.cfo_sweep_vector(rate40=stage == 2)
        if stage == 0:
            iq = ref.front_end(iq, 10)
        return iq.astype(np.int16), p_of(arm_holdoff=4000, input_stage=stage)
    yield ("cfo_s1", "40 packets, CFO +-600 kHz, SNR -6..30 dB", lambda: cfo(1), None, True)
    yield ("cfo_s2", "the same at 40 Msps", lambda: cfo(2), None, True)
    yield ("cfo_s0_n4096", "same, N_CFO 4096", lambda: (cfo(0)[0], p_of(arm_holdoff=4000, n_cfo=4096)), None, False)
    yield ("cfo_s1_n4096", "same at 20 Msps, N_CFO 4096 (timing window 969 positions)",
           lambda: (cfo(1)[0], p_of(arm_holdoff=4000, n_cfo=4096, input_stage=1)), None, False)
    yield ("cfo_s1_n128", "same, N_CFO 128 (timing window 4937 positions)",
           lambda: (cfo(1)[0], p_of(arm_holdoff=4000, n_cfo=128, input_stage=1)), None, False)
    yield ("cfo_s2_tpk0", "40 Msps, timing peak check off (T_pk 0)",
           lambda: (cfo(2)[0], p_of(arm_holdoff=4000, input_stage=2, t_pk=0)), None, False)
    yield ("cfo_s1_tpk100", "timing peak check 100 (aborts the weaker packets)",
           lambda: (cfo(1)[0], p_of(arm_holdoff=4000, input_stage=1, t_pk=100 * 256)), None, False)

    def high(stage):
        iq = mk.high_snr_vector(rate40=stage == 2)
        if stage == 0:
            iq = ref.front_end(iq, 10)
        return iq.astype(np.int16), p_of(arm_holdoff=4000, input_stage=stage)
    yield ("highsnr_s1", "30 packets at +40..+65 dB and noise-free", lambda: high(1), None, True)
    yield ("highsnr_s2", "the same at 40 Msps", lambda: high(2), None, True)

    st = lambda: mk.stress_vector().astype(np.int16)                      # noqa: E731
    yield ("stress_s1", "full-scale stress, holdoff 500", lambda: (st(), p_of(arm_holdoff=500, input_stage=1)), None, True)
    yield ("stress_s0_hold1", "stress into the detector, holdoff 1", lambda: (st(), p_of(arm_holdoff=1)), None, True)
    yield ("stress_s1_n128", "stress, N_CFO 128 (many events)", lambda: (st(), p_of(arm_holdoff=1, n_cfo=128, input_stage=1)), None, False)
    yield ("stress_s0_n4096", "stress, N_CFO 4096", lambda: (st(), p_of(arm_holdoff=1, n_cfo=4096)), None, False)
    yield ("stress_s1_tone_off", "stress, tone check off", lambda: (st(), p_of(arm_holdoff=1, tone_thresh=0, input_stage=1)), None, False)
    yield ("stress_s1_tone24", "stress, tone threshold 24 dB", lambda: (st(), p_of(arm_holdoff=1, tone_thresh=24 * 256, input_stage=1)), None, False)
    yield ("stress_s1_dc0", "stress, DC blocker off", lambda: (st(), p_of(arm_holdoff=1, dc_shift=0, input_stage=1)), None, True)
    yield ("stress_s1_dc4", "stress, DC blocker shift 4", lambda: (st(), p_of(arm_holdoff=1, dc_shift=4, input_stage=1)), None, True)
    yield ("stress_s2_hold1", "stress at 40 Msps, holdoff 1", lambda: (st(), p_of(arm_holdoff=1, input_stage=2)), None, True)
    yield ("stress_s2_n128", "stress at 40 Msps, N_CFO 128", lambda: (st(), p_of(arm_holdoff=1, n_cfo=128, input_stage=2)), None, False)
    yield ("rep3_s1_hold300", "rep3, re-arm after N_CFO", lambda: (vector_from_json("rep3", 1)[0], p_of(arm_holdoff=300, input_stage=1)), None, False)

    def pieces():
        iq, p = vector_from_json("rep3", 1)
        return iq, p
    yield ("pieces_s1", "rep3 in 8 uneven RUNs, windows split", pieces,
           [3401, 2000, 46, 1, 100000, 98000, 2000, None], True)
    yield ("pieces_s2", "rep3 at 40 Msps in 8 uneven (even) RUNs", lambda: vector_from_json("rep3", 2),
           [6802, 4000, 92, 2, 200000, 196000, 4000, None], True)
    yield ("pieces_s1_end", "rep3 cut inside a window (truncated)", lambda: (vector_from_json("rep3", 1)[0][:5000], vector_from_json("rep3", 1)[1]),
           None, True)
    # noise: independent 4M-sample records (0.2 s each; the model needs ~1 GB per 4M samples)
    for k in range(1 if quick else 5):
        n_noise = 1_000_000 if quick else 4_000_000
        yield (f"noise_{k}", f"noise only, {n_noise / 20e6:.2f} s, seed {k}",
               lambda k=k, n=n_noise: (noise_vector(n, seed=100 + k), p_of()), None, False)


# --------------------------------------------------------------------------------------- #
# Sessions                                                                                 #
# --------------------------------------------------------------------------------------- #
class EmuTarget:
    """pd_board_emulator.Board in this process, called with the same payloads."""
    def __init__(self):
        self.b = emu.Board(64 * 1024 * 1024)

    def call(self, cmd: int, payload: bytes = b"") -> bytes:
        return self.b.handle(cmd, payload)


def session(target, iq: np.ndarray, p: pr.Params, sizes: list[int], debug: bool, board: bool):
    """RESET, SET_PARAMS, then per piece LOAD + RUN (+ GET_DEBUG). Returns per-RUN results."""
    target.call(pr.RESET)
    target.call(pr.SET_PARAMS, p.pack())
    out, i = [], 0
    for n in sizes:
        blk = iq[i:i + n]
        if board:
            target.load(blk)
        else:
            target.call(pr.LOAD_SAMPLES, struct.pack("<I", 0) + pr.iq_to_bytes(blk))
        reply = target.call(pr.RUN, struct.pack("<II", n, pr.RUN_FLAG_DEBUG if debug else 0))
        n_ev, done = struct.unpack_from("<II", reply)
        evs = pr.EventRecord.unpack_many(reply[8:], n_ev)
        off = 8 + pr.EventRecord.FMT.size * n_ev
        tms = []
        if len(reply) >= off + 4:
            (n_tm,) = struct.unpack_from("<I", reply, off)
            tms = sorted(pr.TimingRecord.unpack_many(reply[off + 4:], n_tm), key=lambda r: r.t_trig)
        dbg = None
        if debug:
            if board:
                dbg = target.get_debug(done // (2 if p.input_stage == 2 else 1))
            else:
                dbg = target.b.debug.copy()       # the whole capture (GET_DEBUG is limited to 1 MB)
        out.append((evs, done, dbg, tms))
        i += n
    return out


def compare(got, exp, tone_tol_q8: int, timing: bool) -> list[str]:
    errs = []
    for k, ((ge, gd, gdbg, gt), (ee, ed, edbg, et)) in enumerate(zip(got, exp)):
        if timing and gt != et:
            bad = [(g, e) for g, e in zip(gt, et) if g != e]
            errs.append(f"RUN {k}: timing records differ ({len(gt)} board, {len(et)} model)"
                        + (f", first: board {astuple(bad[0][0])} model {astuple(bad[0][1])}" if bad else ""))
        if gd != ed:
            errs.append(f"RUN {k}: {gd} samples processed, expected {ed}")
        if len(ge) != len(ee):
            errs.append(f"RUN {k}: {len(ge)} events, expected {len(ee)} "
                        f"(board t {[e.t_trig for e in ge][:6]}, model {[e.t_trig for e in ee][:6]})")
        for g, e in zip(ge, ee):
            a, b = astuple(g), astuple(e)
            if a[:3] + a[4:] != b[:3] + b[4:] or abs(a[3] - b[3]) > tone_tol_q8:
                errs.append(f"RUN {k}: event board {a} model {b}")
        if gdbg is not None:
            bad = np.flatnonzero(gdbg != edbg) if gdbg.size == edbg.size else [0]
            if len(bad):
                i = int(bad[0])
                errs.append(f"RUN {k}: {len(bad)} debug records differ, first at {i}"
                            + (f": board {gdbg[i]} model {edbg[i]}" if gdbg.size == edbg.size else
                               f" ({gdbg.size} records, expected {edbg.size})"))
    return errs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Detector regression on the Zedboard against the bit-true model")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--only", help="comma-separated substrings of case names")
    ap.add_argument("--quick", action="store_true", help="one short noise record (0.05 s)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--tone-tol", type=float, default=0.01, help="ToneCheck dB tolerance (decision exact)")
    a = ap.parse_args(argv)

    todo = list(cases(a.quick))
    if a.only:
        keys = a.only.split(",")
        todo = [c for c in todo if any(k in c[0] for k in keys)]
    if a.list:
        for c in todo:
            print(f"{c[0]:22s} {c[1]}")
        return 0
    try:
        board = hw.Board(a.host, a.port, a.timeout)
        info = board.ping()
    except OSError as exc:
        print(f"Communication error: {exc}")
        return 2
    kind = "emulator" if info.flags & pr.BOARD_EMULATOR else "board"
    print(f"{kind} at {a.host}:{a.port}, build {info.build}; {len(todo)} case(s)\n")
    if info.flags & pr.BOARD_NO_DETECTOR:
        print("This bitstream has no detector")
        return 2
    tol = int(round(a.tone_tol * 256))
    timing = bool(info.flags & pr.BOARD_TIMING)
    if not timing:
        print("Note: no Preamble B timing on this board (bitstream before 3.0 or firmware before build 10): "
              "timing records not compared\n")
    report, n_fail = [], 0
    print(f"{'case':22s} {'samples':>9s} {'RUNs':>4s} {'events':>6s} {'fail':>4s} {'trunc':>5s} {'timing':>11s} "
          f"{'debug':>9s} {'board s':>7s}  result")
    for name, desc, build, plan, debug in todo:
        try:
            iq, p = build()
        except Exception as exc:                                       # noqa: BLE001
            print(f"{name:22s} skipped: {exc}")
            report.append({"case": name, "result": "skipped", "reason": str(exc)})
            continue
        n = iq.shape[0]
        sizes = [n] if plan is None else [s if s is not None else n - sum(x for x in plan if x) for s in plan]
        try:
            t0 = time.perf_counter()
            got = session(board, iq, p, sizes, debug, True)
            t_board = time.perf_counter() - t0
        except (OSError, pr.ProtocolError) as exc:
            print(f"{name:22s} communication error: {exc}")
            return 2
        exp = session(EmuTarget(), iq, p, sizes, debug, False)
        errs = compare(got, exp, tol, timing)
        evs = [e for r in got for e in r[0]]
        n_tf = sum(1 for e in evs if e.status & pr.EV_TONE_FAIL)
        n_tr = sum(1 for e in evs if e.status & pr.EV_TRUNCATED)
        n_dbg = sum(r[2].size for r in got if r[2] is not None)
        tms = [t for r in (got if timing else exp) for t in r[3]]
        n_good = sum(t.status == pr.TM_GOOD for t in tms)
        tm_col = f"{len(tms)}/{n_good}" if p.input_stage in (1, 2) else "-"
        ok = not errs
        n_fail += not ok
        print(f"{name:22s} {n:9d} {len(sizes):4d} {len(evs):6d} {n_tf:4d} {n_tr:5d} {tm_col:>11s} "
              f"{(str(n_dbg) if debug else '-'):>9s} {t_board:7.1f}  {'PASS' if ok else 'FAIL'}")
        for e in errs[:5]:
            print(f"    {e}")
        rec = {"case": name, "description": desc, "samples": n, "runs": len(sizes), "events": len(evs),
               "tone_fail": n_tf, "truncated": n_tr, "timing_records": len(tms), "timing_good": n_good,
               "debug_records": n_dbg, "board_s": round(t_board, 2),
               "result": "PASS" if ok else "FAIL", "errors": errs[:20], "params": p.as_dict()}
        report.append(rec)
    noise = [r for r in report if r["case"].startswith("noise") and r["result"] != "skipped"]
    if noise:
        secs = sum(r["samples"] for r in noise) / pr.FS
        trig = sum(r["events"] for r in noise)
        passing = trig - sum(r["tone_fail"] for r in noise)
        print(f"\nFalse triggers on noise (eta_hi {np.sqrt(pr.Params().eta_hi_sq / 65536):.2f}): {trig} in "
              f"{secs:.2f} s = {trig / secs:.2f}/s; {passing} passed the tone check (spec 3.3: < 0.2/s)")
    out = ROOT / "TestVectors" / "regression_result.json"
    out.write_text(json.dumps({"target": kind, "host": a.host, "build": info.build, "cases": report}, indent=2))
    print(f"\n{'ALL PASS' if n_fail == 0 else f'{n_fail} case(s) FAILED'}  (report {out})")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
