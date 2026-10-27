#!/usr/bin/env python3
"""
pd_hw_test.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

PC side of the Zedboard packet-detector test: sends a test vector from pd_make_vector.py
to the board (or to pd_board_emulator.py), runs the detector, reads the results back and
compares them with the expected events in the vector's .json.

    python pd_hw_test.py TestVectors/Waveform_Payload_DefectsModeled_Tx.json            # board at 192.168.50.10
    python pd_hw_test.py TestVectors/<v>.json --host 127.0.0.1                          # emulator
    python pd_hw_test.py TestVectors/<v>.json --loopback-only                           # Ethernet + DDR
    python pd_hw_test.py TestVectors/<v>.json --pl-loopback                             # + DMA + PL
    python pd_hw_test.py TestVectors/<v>.json --chunk 50000                             # run in pieces
    python pd_hw_test.py TestVectors/<v>.json --debug --plot                            # metric + flags

Steps: PING, RESET, SET_PARAMS, LOAD_SAMPLES (64k-sample messages), READ_SAMPLES back and
compare (loopback), RUN (all at once or in --chunk pieces, state carries over), optional
GET_DEBUG, then compare with the bit-true model's expected events:
    t_trig, status, cfo_inc, lag angles     exact (--cfo-tol / --angle-tol to relax)
    ToneCheck dB                            within --tone-tol (0.01 dB): the decision is exact
                                            integer; the dB value is the firmware's log10
    debug records (--debug)                 every dbg_metric and flag word, exact
--input-stage 0 sends the LPF output (<vector>.bin, PC did the front end); 1 sends the
decimated input (<vector>_dec.bin) and the board runs the DC blocker and LPF itself.
Exit status 0 = PASS, 1 = FAIL, 2 = communication error.
Results are saved to <vector>_result.json (and <vector>_debug.png with --plot).
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time
from dataclasses import fields
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pd_protocol as pr  # noqa: E402

DEFAULT_HOST = "192.168.50.10"    # FW/build_firmware.py BOARD_IP


class Board:
    def __init__(self, host: str, port: int, timeout: float):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.seq = 0

    def call(self, cmd: int, payload: bytes = b"") -> bytes:
        self.seq += 1
        pr.send_msg(self.sock, cmd, payload, self.seq)
        rcmd, status, rseq, reply = pr.recv_msg(self.sock)
        if rcmd != cmd or rseq != self.seq:
            raise pr.ProtocolError(f"reply cmd {rcmd} seq {rseq} for request cmd {cmd} seq {self.seq}")
        if status != pr.ST_OK:
            raise pr.ProtocolError(f"{pr.CMD_NAMES.get(cmd, cmd)} failed (status {status}): "
                                   f"{reply.decode(errors='replace')}")
        return reply

    def ping(self) -> pr.BoardInfo:
        return pr.BoardInfo.unpack(self.call(pr.PING))

    def load(self, iq: np.ndarray, offset: int = 0) -> int:
        stored = 0
        for i in range(0, iq.shape[0], pr.MAX_LOAD_SAMPLES):
            blk = iq[i:i + pr.MAX_LOAD_SAMPLES]
            (stored,) = struct.unpack("<I", self.call(pr.LOAD_SAMPLES, struct.pack("<I", offset + i) + pr.iq_to_bytes(blk)))
        return stored

    def read(self, offset: int, count: int) -> np.ndarray:
        out, step = [], pr.MAX_PAYLOAD // 4
        for i in range(offset, offset + count, step):
            n = min(step, offset + count - i)
            out.append(pr.bytes_to_iq(self.call(pr.READ_SAMPLES, struct.pack("<II", i, n))))
        return np.concatenate(out) if out else np.zeros((0, 2), np.int16)

    def run(self, count: int, debug: bool, flags: int = 0) -> tuple[list[pr.EventRecord], int]:
        flags |= pr.RUN_FLAG_DEBUG if debug else 0
        reply = self.call(pr.RUN, struct.pack("<II", count, flags))
        n_ev, done = struct.unpack_from("<II", reply)
        return pr.EventRecord.unpack_many(reply[8:], n_ev), done

    def get_debug(self, count: int) -> np.ndarray:
        out, step = [], pr.MAX_PAYLOAD // pr.DEBUG_DTYPE.itemsize
        for i in range(0, count, step):
            n = min(step, count - i)
            out.append(np.frombuffer(self.call(pr.GET_DEBUG, struct.pack("<II", i, n)), pr.DEBUG_DTYPE))
        return np.concatenate(out) if out else np.zeros(0, pr.DEBUG_DTYPE)


def compare(got: list[pr.EventRecord], exp: list[dict], a) -> tuple[bool, list[str]]:
    lines, ok = [], True
    if len(got) != len(exp):
        ok = False
        lines.append(f"  event count: board {len(got)}, expected {len(exp)}")
    by_t = {e["t_trig"]: e for e in exp}
    for g in got:
        e = by_t.pop(g.t_trig, None)
        if e is None:
            ok = False
            lines.append(f"  EXTRA   t_trig {g.t_trig}, cfo_inc {g.cfo_inc}, status 0x{g.status:x}")
            continue
        errs = []
        if g.status != e["status"]:
            errs.append(f"status 0x{g.status:x} vs 0x{e['status']:x}")
        if g.status & pr.EV_CFO_VALID:
            if abs(g.cfo_inc - e["cfo_inc"]) > a.cfo_tol:
                errs.append(f"cfo_inc {g.cfo_inc} vs {e['cfo_inc']}")
            for k in ("ang16", "ang32", "ang128", "ang256"):
                d = (getattr(g, k) - e[k] + 32768) % 65536 - 32768
                if abs(d) > a.angle_tol:
                    errs.append(f"{k} {getattr(g, k)} vs {e[k]}")
            if abs(g.tone_db_q8 - e["tone_db_q8"]) / 256 > a.tone_tol:
                errs.append(f"ToneCheck {g.tone_db:.2f} vs {e['tone_db_q8'] / 256:.2f} dB")
        if g.cfo_valid_cnt != e["cfo_valid_cnt"]:
            errs.append(f"cfo_valid_cnt {g.cfo_valid_cnt} vs {e['cfo_valid_cnt']}")
        tag = "ok     " if not errs else "MISMATCH"
        ok &= not errs
        desc = (f"f_hat {g.cfo_hz:9.1f} Hz, ToneCheck {g.tone_db:5.1f} dB" if g.status & pr.EV_CFO_VALID
                else "no CFO (truncated)")
        if g.status & pr.EV_TONE_FAIL:
            desc += " (tone check FAIL -> abort)"
        lines.append(f"  {tag} t_trig {g.t_trig:8d}  {desc}" + (f"   [{'; '.join(errs)}]" if errs else ""))
    for t, e in by_t.items():
        ok = False
        lines.append(f"  MISSING t_trig {t}, cfo_inc {e['cfo_inc']}")
    return ok, lines


def check_debug(dbg: np.ndarray, iq: np.ndarray, params: pr.Params, lines: list[str]) -> bool:
    """Compare the board's debug records with the bit-true model, sample by sample."""
    import pd_reference as ref
    res = ref.detect(iq, params, debug=True)
    n = min(dbg.size, res.metric.size)
    bad = np.flatnonzero((dbg["metric"][:n] != res.metric[:n]) | (dbg["bits"][:n] != res.bits[:n]))
    if dbg.size != res.metric.size:
        lines.append(f"  DEBUG   {dbg.size} records, expected {res.metric.size}")
        return False
    if bad.size:
        i = int(bad[0])
        lines.append(f"  DEBUG   {bad.size} of {n} records differ, first at sample {i}: metric "
                     f"{dbg['metric'][i]} bits 0x{dbg['bits'][i]:x}, expected {res.metric[i]} "
                     f"0x{res.bits[i]:x}")
        return False
    lines.append(f"  ok      debug: all {n} records (metric and flags) match the bit-true model")
    return True


def plot_debug(dbg: np.ndarray, meta: dict, events: list[pr.EventRecord], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    p = meta["params"]
    ratio = np.sqrt(dbg["metric"] / 65536)
    n = np.arange(ratio.size)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ax1.plot(n, ratio, lw=0.6, label="|P| / sqrt(R0 R_D) (dbg_metric)")
    ax1.axhline(np.sqrt(p["eta_hi_sq"] / 65536), color="C3", ls="--", lw=1, label="eta_hi")
    ax1.axhline(np.sqrt(p["eta_lo_sq"] / 65536), color="C2", ls="--", lw=1, label="eta_lo")
    for t in meta.get("truth", []):
        ax1.axvspan(t["preamble_a_start"], t["preamble_a_start"] + 5000, color="0.85", lw=0)
    for e in events:
        ax1.axvline(e.t_trig, color="k", lw=1)
        ax1.annotate(f"t_trig {e.t_trig}\n{e.cfo_hz:.0f} Hz", (e.t_trig, 1.0), fontsize=8,
                     xytext=(4, -4), textcoords="offset points", va="top")
    ax1.set_ylim(0, 1.05)
    ax1.set_ylabel("ratio")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title(f"{meta['source']}: detector metric from the board (grey = Preamble A)")
    for bit, name, y in ((pr.DBG_ABOVE_HI, "above_hi", 3), (pr.DBG_BELOW_LO, "below_lo", 2),
                         (pr.DBG_ARMED, "armed", 1), (pr.DBG_BLANK, "blank", 0)):
        ax2.fill_between(n, y, y + 0.8 * ((dbg["bits"] & bit) > 0), step="post", lw=0)
    ax2.set_yticks([0.4, 1.4, 2.4, 3.4], ["blank", "armed", "below_lo", "above_hi"])
    ax2.set_xlabel("sample_cnt (20 Msps)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Run a packet-detector test vector on the Zedboard (or emulator)")
    ap.add_argument("vector", type=Path, help="vector .json from pd_make_vector.py (the .bin sits next to it)")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--chunk", type=int, default=0, help="run the vector in pieces of this many samples")
    ap.add_argument("--loopback-only", action="store_true", help="load and read back only, no detector run")
    ap.add_argument("--no-loopback", action="store_true", help="skip the read-back check")
    ap.add_argument("--pl-loopback", action="store_true",
                    help="stream the samples through the PL and back (DMA path check, no detector)")
    ap.add_argument("--debug", action="store_true", help="capture dbg_metric and flag bits")
    ap.add_argument("--plot", action="store_true", help="plot the debug capture (implies --debug)")
    ap.add_argument("--input-stage", type=int, choices=(0, 1), default=0,
                    help="0: send the LPF output (default); 1: send the decimated input, board runs the front end")
    ap.add_argument("--cfo-tol", type=int, default=0, help="cfo_inc tolerance, LSB (bit-true: 0)")
    ap.add_argument("--angle-tol", type=int, default=0, help="CORDIC angle tolerance, LSB (bit-true: 0)")
    ap.add_argument("--tone-tol", type=float, default=0.01, help="ToneCheck dB tolerance")
    a = ap.parse_args(argv)
    a.debug |= a.plot

    meta = json.loads(a.vector.with_suffix(".json").read_text())
    if a.input_stage == 1:
        name = meta.get("files", {}).get("decimated")
        if not name:
            print("Error: this vector has no decimated input - rebuild it with pd_make_vector.py")
            return 2
        bin_path = a.vector.with_name(name)
    else:
        bin_path = a.vector.with_suffix(".bin")
    iq = np.fromfile(bin_path, dtype=pr.SAMPLE_DTYPE).reshape(-1, 2)
    if iq.shape[0] != meta["samples"]:
        print(f"Error: {bin_path.name} has {iq.shape[0]} samples, json says {meta['samples']}")
        return 2
    known = {f.name for f in fields(pr.Params)}
    params = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
    params.input_stage = a.input_stage
    if "files" not in meta:
        print("Note: vector built before the bit-true model - rebuild it with pd_make_vector.py "
              "for exact comparisons")
    print(f"Vector {a.vector.stem} ({bin_path.name}, input_stage {a.input_stage}): {iq.shape[0]} samples, "
          f"{len(meta['expected_events'])} expected event(s)")
    print(f"Params {params.describe()}")

    try:
        b = Board(a.host, a.port, a.timeout)
        info = b.ping()
        kind = "emulator" if info.flags & pr.BOARD_EMULATOR else "board"
        print(f"Connected to {kind} at {a.host}:{a.port}: protocol v{info.version}, build {info.build}, "
              f"buffer {info.max_samples} samples")
        if info.version != pr.PROTOCOL_VERSION:
            print(f"Warning: protocol v{info.version}, this script speaks v{pr.PROTOCOL_VERSION}")
        if info.flags & pr.BOARD_NO_DETECTOR and not (a.loopback_only or a.pl_loopback):
            print("This bitstream has no detector yet (phase 1): use --loopback-only or --pl-loopback")
            return 2
        chunk = a.chunk or iq.shape[0]
        if chunk > info.max_samples:
            chunk = info.max_samples
            print(f"Vector larger than the board buffer - running in pieces of {chunk}")
        b.call(pr.RESET)
        b.call(pr.SET_PARAMS, params.pack())

        result = {"vector": a.vector.stem, "target": kind, "host": a.host, "chunk": chunk}
        events: dict[int, pr.EventRecord] = {}
        dbg_parts = []
        t_load = t_run = 0.0
        loop_ok = pl_ok = True
        for i in range(0, iq.shape[0], chunk):
            blk = iq[i:i + chunk]
            t0 = time.perf_counter()
            b.load(blk)
            t_load += time.perf_counter() - t0
            if not a.no_loopback:
                back = b.read(0, blk.shape[0])
                if not np.array_equal(back, blk):
                    bad = np.flatnonzero(np.any(back != blk, axis=1))
                    print(f"LOOPBACK FAIL: {bad.size} samples differ, first at {i + bad[0]}: "
                          f"sent {blk[bad[0]].tolist()}, read {back[bad[0]].tolist()}")
                    loop_ok = False
            if a.loopback_only:
                continue
            if a.pl_loopback:
                t0 = time.perf_counter()
                _, done = b.run(blk.shape[0], True, pr.RUN_FLAG_PL_LOOPBACK)
                t_run += time.perf_counter() - t0
                echo = b.get_debug(done).view(pr.SAMPLE_DTYPE).reshape(-1, 2)
                if done != blk.shape[0] or not np.array_equal(echo, blk):
                    bad = np.flatnonzero(np.any(echo != blk[:echo.shape[0]], axis=1)) if done else [0]
                    first = int(bad[0]) if len(bad) else done
                    print(f"PL LOOPBACK FAIL: {done} of {blk.shape[0]} samples returned, "
                          f"{len(bad)} differ, first at {i + first}")
                    pl_ok = False
                continue
            t0 = time.perf_counter()
            evs, done = b.run(blk.shape[0], a.debug)
            t_run += time.perf_counter() - t0
            for e in evs:                      # a truncated event is re-reported once complete
                events[e.t_trig] = e
            if a.debug:
                dbg_parts.append(b.get_debug(done))
        mb = iq.nbytes / 1e6
        print(f"Loaded {mb:.2f} MB in {t_load:.2f} s ({8 * mb / max(t_load, 1e-9):.0f} Mbit/s)"
              + ("" if a.no_loopback else f", loopback {'OK' if loop_ok else 'FAIL'}"))
        result["loopback_ok"] = loop_ok
        if a.pl_loopback:
            print(f"PL loopback (DDR -> DMA -> PL -> DMA -> DDR): {'OK' if pl_ok else 'FAIL'}, "
                  f"{t_run:.2f} s")
            result["pl_loopback_ok"] = pl_ok
        if a.loopback_only or a.pl_loopback:
            a.vector.with_name(a.vector.stem + "_result.json").write_text(json.dumps(result, indent=2))
            ok = loop_ok and pl_ok
            print("PASS" if ok else "FAIL")
            return 0 if ok else 1
        print(f"Detector run: {t_run:.2f} s including transfer")
    except (OSError, pr.ProtocolError) as exc:
        print(f"Communication error: {exc}")
        return 2

    got = sorted(events.values(), key=lambda e: e.t_trig)
    ok, lines = compare(got, meta["expected_events"], a)
    ok &= loop_ok
    if a.debug and dbg_parts:
        ok &= check_debug(np.concatenate(dbg_parts), iq, params, lines)
    print("Events (board vs reference):")
    print("\n".join(lines) if lines else "  none")
    truth = meta.get("truth", [])
    for e in got:
        hit = [t for t in truth if t["preamble_a_start"] - 300 <= e.t_trig <= t["packet_end"]]
        if hit:
            print(f"  t_trig {e.t_trig}: {e.t_trig - hit[0]['preamble_a_start']} samples into Preamble A"
                  + (f", CFO error {e.cfo_hz - meta['defects']['FrequencyOffset']:+.0f} Hz"
                     if "FrequencyOffset" in meta.get("defects", {}) and e.status & pr.EV_CFO_VALID else ""))
        else:
            print(f"  t_trig {e.t_trig}: not on a packet (false trigger)")
    print("PASS" if ok else "FAIL")

    result.update({"pass": ok, "events": [vars(e) for e in got], "report": lines})
    a.vector.with_name(a.vector.stem + "_result.json").write_text(json.dumps(result, indent=2))
    if a.plot and dbg_parts:
        png = a.vector.with_name(a.vector.stem + "_debug.png")
        plot_debug(np.concatenate(dbg_parts), meta, got, png)
        print(f"Plot {png}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
