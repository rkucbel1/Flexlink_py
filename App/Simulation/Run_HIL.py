#!/usr/bin/env python3
"""
Run_HIL.py  (lives in Flexlink_py/App)

Hardware in the loop: sends the waveform of the last Tx chain run (Tx/Run_Tx_Chain.py) through
the Zedboard FPGA receiver instead of the Python Rx chain, and checks the decoded transport
blocks against the ones the Tx sent.

    1. Waveform: Tx/Waveform_Payload_DefectsModeled_Tx.h5 when Channel_Defects.json has a defect
       on, else Tx/Waveform_Payload_Tx.h5. The FPGA has one receive path, so the file must hold
       one waveform (Number of Rays 1).
    2. Board samples (HW_Development/Packet_Detector/PC/pd_make_vector.py): idle noise before and
       after the packet, 12-bit quantisation; 20 Msps (decimated on the PC, the FPGA runs the DC
       blocker and LPF) or 40 Msps (the FPGA also runs its half-band decimator). Detector
       thresholds from Rx/Parameters_Rx_payload.json.
    3. Board run (pd_protocol over TCP, bitstream 9.0 / firmware build 16): detection, CFO,
       tone check, Preamble B timing, control field, channel estimate, demapper, combiner, LDPC
       decoder and CRC24 all in the PL.
    4. Check: every transport block the Tx sent (Tx/Generate_RandomPacketOfData/
       Bitstream_TransportBlks_Tx.h5) against the one the board decoded, and the control-field
       message against Tx/ControlSignals.

Writes App/Simulation/HIL/: HIL_result.json (this run), HIL_results.csv (one row per run, appended),
HIL_vector* (the samples sent).

Usage
    python App/Simulation/Run_HIL.py                                  # settings from App/Simulation/Parameters_HIL.json
    python App/Simulation/Run_HIL.py --host 127.0.0.1                 # pd_board_emulator.py instead of the board
    python App/Simulation/Run_HIL.py --input-rate 40
    python App/Simulation/Run_HIL.py --check-model                    # also compare with the bit-true model (slow)

Exit status: 0 every transport block received and equal to the Tx, 1 a block missing, failed or
different, 2 no packet detected or the packet was aborted (tone check, timing), 4 control field
CRC failed, 5 board not reachable or too old, 6 setup the FPGA cannot take (more than one ray, no
preamble or control field).
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import sys
import time
from pathlib import Path

import numpy as np

for _p in Path(__file__).resolve().parents:      # put the project root (flexlink_paths.py) on sys.path
    if (_p / "flexlink_paths.py").is_file():
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))
        break
from flexlink_paths import APP, APP_SIMULATION, CHANNEL_DEFECTS, PACKET_DETECTOR_PC, ROOT, RX, TX  # noqa: E402

PD_PC = PACKET_DETECTOR_PC
HIL_DIR = APP_SIMULATION / "HIL"
HIL_PARAMS = APP_SIMULATION / "Parameters_HIL.json"
DEFECTS_FILE = CHANNEL_DEFECTS / "Channel_Defects.json"
TX_PARAMS = TX / "Parameters_Tx_payload.json"
RX_PARAMS = RX / "Parameters_Rx_payload.json"
TX_BLOCKS = TX / "Generate_RandomPacketOfData" / "Bitstream_TransportBlks_Tx.h5"

DEFAULTS = {"RxPath": "Simulation", "BoardHost": "192.168.50.10", "BoardPort": 5001, "InputRate": 20}

EXIT_OK, EXIT_BLOCKS, EXIT_NO_PACKET, EXIT_CF, EXIT_BOARD, EXIT_SETUP = 0, 1, 2, 4, 5, 6
STATUS_TEXT = {EXIT_OK: "every transport block received and equal to the Tx",
               EXIT_BLOCKS: "a transport block missing, failed or different",
               EXIT_NO_PACKET: "no packet detected or packet aborted",
               EXIT_CF: "bad packet, control field CRC failed",
               EXIT_BOARD: "board not reachable or bitstream / firmware too old",
               EXIT_SETUP: "setup the FPGA cannot take"}


def hil_settings() -> dict:
    try:
        return {**DEFAULTS, **json.loads(HIL_PARAMS.read_text()).get("HIL", {})}
    except (OSError, json.JSONDecodeError):
        return dict(DEFAULTS)


def defects_on() -> list[str]:
    cfg = json.loads(DEFECTS_FILE.read_text())
    return [k for k, v in cfg.get("Mode", {}).items() if int(v)]


def waveform_file() -> Path:
    """The waveform the Rx chain would get: the channel output if a defect is on."""
    tx = json.loads(TX_PARAMS.read_text()).get("File", {})
    if defects_on():
        return TX / tx.get("Waveform_Payload_DefectsModeled_Tx", "Waveform_Payload_DefectsModeled_Tx.h5")
    return TX / tx.get("Waveform_Payload_Tx", "Waveform_Payload_Tx.h5")


def tx_blocks() -> list[np.ndarray]:
    """The transport blocks the Tx sent, in order."""
    import h5py
    calc = json.loads(TX_PARAMS.read_text()).get("Calculated", {})
    n, size = int(calc["AvailableDataBlocks"]), int(calc["TransportBlockSize"])
    with h5py.File(TX_BLOCKS, "r") as f:
        bits = np.asarray(f["TransportBlock_Bitstream"][()]).ravel().astype(np.uint8)
    if bits.size != n * size:
        raise ValueError(f"{TX_BLOCKS.name} has {bits.size} bits, Parameters_Tx_payload.json says "
                         f"{n} x {size} - run Initialize_Tx and Run_Tx_Chain again")
    return [bits[i * size:(i + 1) * size] for i in range(n)]


def setup_problems(wave: Path) -> list[str]:
    """Settings the FPGA receiver cannot take."""
    out = []
    d = json.loads(TX_PARAMS.read_text()).get("Defined", {})
    if int(d.get("Use_Preamble", 1)) != 1:
        out.append("Use_Preamble is 0: the FPGA needs the preamble to find the packet")
    if int(d.get("Use_ControlField", 1)) != 1:
        out.append("Use_ControlField is 0: the FPGA takes the packet settings from the control field")
    if int(d.get("NumberOfAntennas", 1)) != 1:
        out.append("NumberOfAntennas (Tx) must be 1")
    if not wave.is_file():
        out.append(f"{wave.name} not found - run the Tx chain first")
    else:
        import h5py
        with h5py.File(wave, "r") as f:
            shape = np.shape(f["data"][()])
        if len(shape) == 2 and shape[0] > 1:
            out.append(f"{wave.name} holds {shape[0]} waveforms (rays); the FPGA has one receive path: "
                       "set Number of Rays to 1 on Set Defects")
    return out


def build_vector(wave: Path, rate: int, seed: int | None) -> tuple[np.ndarray, dict, str]:
    """Board samples and the vector metadata (pd_make_vector.py); also returns its printout."""
    import pd_make_vector
    out = HIL_DIR / "HIL_vector"
    argv = ["--project-root", str(ROOT), "--input", str(wave), "--out", str(out), "--no-crosscheck"]
    if seed is not None:
        argv += ["--seed", str(seed)]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        pd_make_vector.main(argv)
    meta = json.loads(out.with_suffix(".json").read_text())
    key = "input40" if rate == 40 else "decimated"
    name = meta["files"].get(key)
    if name is None:
        raise ValueError(f"no {rate} Msps samples: the waveform is not at 40 Msps")
    iq = np.fromfile(out.with_name(name), dtype=np.int16).reshape(-1, 2)
    return iq, meta, buf.getvalue()


def run_board(iq: np.ndarray, params, host: str, port: int, timeout: float):
    """Load and run the samples; returns (board info, events, timing, control, transport blocks)."""
    import pd_hw_test
    import pd_protocol as pr
    b = pd_hw_test.Board(host, port, timeout)
    info = b.ping()
    if not info.flags & pr.BOARD_TB:
        raise RuntimeError(f"the board (firmware build {info.build}) has no LDPC decoder: it needs "
                           "bitstream 9.0 and firmware build 16")
    b.call(pr.RESET)
    b.call(pr.SET_PARAMS, params.pack())
    chunk = min(iq.shape[0], info.max_samples)
    chunk -= chunk % 2
    events, timing, control, tbs = {}, {}, {}, []
    for i in range(0, iq.shape[0], chunk):
        blk = iq[i:i + chunk]
        b.load(blk)
        evs, _ = b.run(blk.shape[0], False)
        for e in evs:
            events[e.t_trig] = e
        for r in b.timing:
            timing[r.t_trig] = r
        for r in b.control:
            control[r.t_trig] = r
        tbs += b.tb
    b.sock.close()
    return info, sorted(events.values(), key=lambda e: e.t_trig), timing, control, tbs


def check(events, timing, control, tbs, sent, meta, rate):
    """Report lines (control field, payload), per-block results and the exit status."""
    import pd_protocol as pr
    import cf_fields
    cl, pl, blocks = [], [], []
    truth = meta.get("truth", [])
    lag = 10 if rate == 40 else 0              # half-band delay at 20 Msps
    true_cfo = meta.get("defects", {}).get("FrequencyOffset")
    on_pkt = [e for e in events if truth and truth[0]["preamble_a_start"] - 300 <= e.t_trig - lag
              <= truth[0]["packet_end"]]
    false = len(events) - len(on_pkt)
    status = EXIT_NO_PACKET
    pkt, cf_fail = None, False
    if not events:
        cl.append("No packet detected")
    for e in events:
        if e not in on_pkt:
            cl.append(f"t_trig {e.t_trig}: false trigger (not on the packet)")
            continue
        line = f"Packet detected: t_trig {e.t_trig} ({e.t_trig - lag - truth[0]['preamble_a_start']} samples into Preamble A)"
        if e.status & pr.EV_CFO_VALID:
            line += f", CFO estimate {e.cfo_hz:.0f} Hz" + (f" (true {float(true_cfo):.0f} Hz)" if true_cfo is not None else "")
        cl.append(line)
        cl.append(f"  Tone check {e.tone_db:.1f} dB: {'FAIL, packet aborted' if e.status & pr.EV_TONE_FAIL else 'pass'}")
        if e.status & pr.EV_TONE_FAIL:
            continue
        r = timing.get(e.t_trig)
        if r is None:
            cl.append("  Preamble B timing: no record")
            continue
        if r.status != pr.TM_GOOD:
            why = "late trigger" if r.status & pr.TM_LATE else "busy" if r.status & pr.TM_BUSY else "peak too weak"
            cl.append(f"  Preamble B timing: not found ({why}, peak/mean {r.peak_to_mean:.1f}), packet aborted")
            continue
        cl.append(f"  Preamble B timing: ends at sample {r.n_end}, peak/mean {r.peak_to_mean:.1f}")
        c = control.get(e.t_trig)
        if c is None:
            cl.append("  Control field: no record")
            continue
        if not c.crc_ok:
            cl.append("  Control field: CRC FAIL, packet dropped")
            cf_fail = True
            continue
        tx = cf_fields.tx_message()
        cl.append("  Control field: CRC ok" + ("" if tx is None else
                                                 ", same as the Tx message" if c.message == tx else
                                                 ", DIFFERENT from the Tx message"))
        cl.append("    " + cf_fields.describe(c.message))
        if pkt is None:
            pkt = e.t_trig
    if false:
        cl.append(f"{false} false trigger(s) outside the packet")

    got = {r.w: r for r in tbs if r.t_trig == pkt} if pkt is not None else {}
    n_ok = n_match = 0
    if pkt is not None:
        pl.append(f"{'Block':>5} {'CRC':>5} {'Iter':>4} {'Bits':>5} {'Bit errors':>10}  Result")
        for w, s in enumerate(sent):
            r = got.get(w)
            if r is None:
                blocks.append({"block": w, "received": False})
                pl.append(f"{w:>5} {'-':>5} {'-':>4} {s.size:>5} {'-':>10}  not received")
                continue
            rb = np.array(r.bits, np.uint8)
            errs = int(np.count_nonzero(rb != s)) if rb.size == s.size else -1
            match = errs == 0
            n_ok += r.crc_ok
            n_match += match
            blocks.append({"block": w, "received": True, "crc_ok": bool(r.crc_ok), "iterations": r.iters,
                           "bits": r.nbits, "bit_errors": errs, "match": match})
            res = ("ok, equal to Tx" if match and r.crc_ok else
                   "CRC ok but DIFFERENT from Tx" if r.crc_ok else
                   "CRC FAIL" + (" (bits equal to Tx)" if match else ""))
            pl.append(f"{w:>5} {'ok' if r.crc_ok else 'FAIL':>5} {r.iters:>4} {r.nbits:>5} "
                      f"{errs if errs >= 0 else 'size?':>10}  {res}")
        extra = sorted(set(got) - set(range(len(sent))))
        if extra:
            pl.append(f"Extra block(s) from the board: {extra}")
        pl.append("")
        pl.append(f"Transport blocks: {len(sent)} sent, {len(got)} received, {n_ok} CRC ok, "
                  f"{n_match} equal to the Tx")
        status = EXIT_OK if n_match == len(sent) and n_ok == len(sent) and not extra else EXIT_BLOCKS
    else:
        status = EXIT_CF if cf_fail else EXIT_NO_PACKET
        pl.append(f"No payload decoded ({len(sent)} transport block(s) sent): "
                  + ("the control field CRC failed" if cf_fail else "no packet got through"))
    return cl, pl, blocks, status, pkt


def append_csv(row: dict) -> None:
    path = HIL_DIR / "HIL_results.csv"
    new = not path.is_file()
    with path.open("a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)


def main(argv=None) -> int:
    s = hil_settings()
    ap = argparse.ArgumentParser(description="Run the last Tx waveform through the Zedboard FPGA receiver")
    ap.add_argument("--host", default=s["BoardHost"])
    ap.add_argument("--port", type=int, default=int(s["BoardPort"]))
    ap.add_argument("--input-rate", type=int, choices=(20, 40), default=int(s["InputRate"]),
                    help="20: the PC decimates; 40: the FPGA's half-band decimates")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--seed", type=int, help="seed of the idle noise around the packet")
    ap.add_argument("--check-model", action="store_true",
                    help="also run the bit-true model on the same samples and compare (slow)")
    ap.add_argument("--log", type=Path, help="write the full report to this file")
    a = ap.parse_args(argv)
    sys.path.insert(0, str(PD_PC))
    HIL_DIR.mkdir(exist_ok=True)
    report: list[str] = []

    def out(line=""):
        print(line)
        report.append(line)

    def finish(status, cl=(), pl=(), extra=None):
        result = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "status": status,
                  "status_text": STATUS_TEXT[status], "host": a.host, "input_rate_msps": a.input_rate,
                  "control_lines": list(cl), "payload_lines": list(pl), **(extra or {})}
        (HIL_DIR / "HIL_result.json").write_text(json.dumps(result, indent=2))
        out(f"HIL result: {STATUS_TEXT[status]}")
        if a.log:
            a.log.write_text("\n".join(report) + "\n")
        return status

    wave = waveform_file()
    problems = setup_problems(wave)
    if problems:
        for p in problems:
            out(f"Setup: {p}")
        return finish(EXIT_SETUP, problems)
    try:
        sent = tx_blocks()
        iq, meta, vec_log = build_vector(wave, a.input_rate, a.seed)
    except (OSError, ValueError, KeyError) as e:
        out(f"Setup: {e}")
        return finish(EXIT_SETUP, [str(e)])
    report += ["===== Vector (pd_make_vector.py)", vec_log.rstrip()]

    import pd_protocol as pr
    rx = json.loads(RX_PARAMS.read_text()).get("Defined_Rx", {})
    known = set(pr.Params.__dataclass_fields__)
    params = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
    params.input_stage = 2 if a.input_rate == 40 else 1
    params.rearm = 1
    params.t_pk = pr.q8(float(rx.get("TimingPeakThresh", 18)))
    out(f"Waveform {wave.name}: {iq.shape[0]} samples at {a.input_rate} Msps to the board at {a.host}:{a.port}")
    out(f"Detector: {params.describe()}")
    try:
        t0 = time.perf_counter()
        info, events, timing, control, tbs = run_board(iq, params, a.host, a.port, a.timeout)
        dt = time.perf_counter() - t0
    except (OSError, RuntimeError, pr.ProtocolError) as e:
        msg = f"Board: {e}"
        out(msg)
        return finish(EXIT_BOARD, [msg, f"Is the board on, programmed (xsdb run_board.tcl) and at {a.host}?"])
    kind = "emulator" if info.flags & pr.BOARD_EMULATOR else "board"
    out(f"{kind.capitalize()} firmware build {info.build}: run took {dt:.1f} s including transfer")

    cl, pl, blocks, status, pkt = check(events, timing, control, tbs, sent, meta, a.input_rate)
    cl.insert(0, f"Zedboard FPGA ({kind}, firmware build {info.build}), {a.input_rate} Msps input:")
    if a.check_model:
        import pd_hw_test
        import pd_reference as ref
        res = ref.detect(iq, params, control=True)
        mok, mlines = pd_hw_test.compare_tb(tbs, res.tb)
        pl += ["", "Bit-true model: " + ("board identical" if mok else "board DIFFERENT")] + mlines
    out("===== Control field (HIL)")
    for ln in cl:
        out(ln)
    out("===== Payload (HIL)")
    for ln in pl:
        out(ln)

    d = json.loads(TX_PARAMS.read_text()).get("Defined", {})
    df = meta.get("defects", {})
    append_csv({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "mod": d.get("Mod"),
                "code": f"{d.get('EncoderBlockSize')}_{d.get('EncoderCodeRate')}",
                "repetition": d.get("Repetition"), "packet_length": d.get("PacketLength"),
                "defects": "+".join(defects_on()) or "none", "snr_db": df.get("SNR_dB", ""),
                "cfo_hz": df.get("FrequencyOffset", ""), "input_msps": a.input_rate,
                "blocks_sent": len(sent), "blocks_received": sum(b["received"] for b in blocks),
                "crc_ok": sum(b.get("crc_ok", False) for b in blocks),
                "equal_to_tx": sum(b.get("match", False) for b in blocks),
                "result": "PASS" if status == EXIT_OK else STATUS_TEXT[status]})
    return finish(status, cl, pl, {"t_trig": pkt, "blocks": blocks, "board_build": info.build,
                                   "target": kind, "waveform": wave.name})


if __name__ == "__main__":
    sys.exit(main())
