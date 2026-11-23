#!/usr/bin/env python3
"""fmc_stream_test.py  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/PC)

Step 4 test: the AD9361 (FMCOMMS3) through the Zedboard's FPGA receiver pd_0, live, with
FW/fmc_pd_server.c on the board and the Pluto transmitting (FlexlinkTxWithPluto.py on PC2).
The FMCOMMS3 counterpart of ../../Packet_Detector/PC/pd_stream_test.py, whose pieces it uses:

  1. RADIO_CONFIG (LO, rate, bandwidth, gain) and SET_PARAMS (TestVectors/rep3.json,
     input_stage 2 at 40 Msps / 1 at 20 Msps, dc_shift 6: with 0, narrow spurs near 0 Hz that
     the AD9361's DC tracking leaves cause false triggers, and some real packets are missed
     (2026-11-23: 6051 triggers for 5764 packets, 1 % missed; with 6 every packet, none false))
  2. STREAM_START: axi_dmac fills the DDR ring, the AXI DMA feeds pd_0
  3. POLL_RESULTS (or the board's STREAM_PUSH with --push), printed once a second: ring slots
     written, overflows (samples lost), ring level, packets, transport blocks CRC ok
  4. STREAM_STOP, summary (BLER)

    python fmc_stream_test.py                         # 10 s, 2420 MHz, 50 dB, polled
    python fmc_stream_test.py --seconds 60 --push     # results pushed by the board
    python fmc_stream_test.py --gain-mode slow_attack

Expected with the Pluto sending random packets (Gap 20000): one packet per 5.203 ms, 192 per
second, every transport block CRC ok, no overflows.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from dataclasses import fields
from pathlib import Path

HERE = Path(__file__).resolve().parent
PD_PC = HERE.parents[1] / "Packet_Detector" / "PC"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PD_PC))
import fmc_protocol as fp  # noqa: E402  (also puts Packet_Detector/PC on the path)
import pd_protocol as pr  # noqa: E402
from pd_hw_test import Board  # noqa: E402
from pd_stream_test import Tally  # noqa: E402

DEFAULT_PARAMS = HERE.parents[1] / "Packet_Detector" / "TestVectors" / "rep3.json"
SLOT_SAMPLES = 32768                   # FW/fmc_pd_server.c FMC_SLOT_SAMPLES (input samples per ring slot)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="10.42.0.101")
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--lo", type=float, default=2420e6)
    ap.add_argument("--rate", type=float, default=40e6, choices=(40e6, 20e6))
    ap.add_argument("--bw", type=float, default=18e6)
    ap.add_argument("--gain-mode", choices=fp.GAIN_MODES, default="manual")
    ap.add_argument("--gain", type=int, default=50)
    ap.add_argument("--dc-shift", type=int, default=6, help="receiver DC blocker (default 6; 0 = off gives false triggers)")
    ap.add_argument("--params-from", type=Path, default=DEFAULT_PARAMS)
    ap.add_argument("--push", action="store_true", help="results pushed by the board (STREAM_PUSH)")
    ap.add_argument("--push-ms", type=float, default=1.0)
    ap.add_argument("--poll", type=float, default=0.05, help="polling interval without --push, s")
    ap.add_argument("--keep-pl", action="store_true", help="also return payload / channel / decoder-input records")
    a = ap.parse_args(argv)

    b = None
    st = None
    tally = Tally()
    try:
        b = Board(a.host, a.port, 60.0)
        info = b.ping()
        if not info.flags & fp.BOARD_FMCOMMS3 or info.flags & pr.BOARD_NO_DETECTOR:
            print(f"Error: board flags 0x{info.flags:X}: needs fmc_pd_server.elf on the step-4 bitstream "
                  f"(FMCOMMS3 and pd_0)", file=sys.stderr)
            return 2
        print(f"Zedboard at {a.host}: fmc_pd_server build {info.build}, flags 0x{info.flags:X}")

        cfg = fp.RadioConfig(lo_hz=int(a.lo), rate_hz=int(a.rate), bw_hz=int(a.bw),
                             gain_mode=fp.GAIN_MODES[a.gain_mode], gain_db=a.gain)
        rs = fp.RadioStatus.unpack(b.call(fp.RADIO_CONFIG, cfg.pack()))
        print(rs.describe())

        params = pr.Params()
        if a.params_from and a.params_from.is_file():
            meta = json.loads(a.params_from.read_text())
            known = {f.name for f in fields(pr.Params)}
            params = pr.Params(**{k: v for k, v in meta.get("params", {}).items() if k in known})
        params.input_stage = 2 if a.rate == 40e6 else 1
        params.dc_shift = a.dc_shift
        b.call(pr.SET_PARAMS, params.pack())

        push_units = min(max(round(a.push_ms * 4), 1), 31) if a.push else 0
        flags = ((pr.STREAM_KEEP_PL if a.keep_pl else 0)
                 | ((pr.STREAM_FLAG_PUSH | (push_units << 2)) if a.push else 0))
        b.call(pr.STREAM_START, struct.pack("<HHIHH", 0, flags, 0, 0, 0))
        print(f"Streaming: input stage {params.input_stage}, dc_shift {a.dc_shift}, "
              + (f"results pushed at most every {push_units / 4:g} ms" if a.push else f"polled every {a.poll:g} s")
              + "\n")
        print("   time    Msps  overflows  ring full  ring max  packets  per s   TB CRC ok    stalls")

        t0 = time.monotonic()
        t_print, prev_slots, prev_pk, prev_t = t0 + 1.0, 0, 0, t0

        def show(s):
            nonlocal t_print, prev_slots, prev_pk, prev_t
            if s.lost_mask:
                print("  WARNING: record buffers overflowed (" + ", ".join(
                    v for k, v in pr.STREAM_LOST_NAMES.items() if s.lost_mask & k) + ")")
            now = time.monotonic()
            if now >= t_print:
                dt = now - prev_t
                tot, ok = tally.blocks
                npk = len(tally.packets)
                print(f"  {now - t0:5.1f}  {(s.datagrams - prev_slots) * SLOT_SAMPLES / dt / 1e6:6.2f}  "
                      f"{s.lost:9d}  {s.ring_full:9d}  {s.ring_max * SLOT_SAMPLES / a.rate * 1e3:6.1f} ms  "
                      f"{npk:7d}  {(npk - prev_pk) / dt:5.0f}  {ok:6d}/{tot:<6d}  {s.stalls:6d}", flush=True)
                prev_slots, prev_pk, prev_t, t_print = s.datagrams, npk, now, now + 1.0

        if a.push:
            b.sock.settimeout(5.0)
            while time.monotonic() - t0 < a.seconds:
                cmd, status, _, raw = pr.recv_msg(b.sock)
                if cmd != pr.STREAM_PUSH:
                    raise pr.ProtocolError(f"unexpected message {pr.CMD_NAMES.get(cmd, cmd)} while streaming")
                _, st_new, recs = pr.parse_push(raw)
                tally.add(recs)
                if st_new is not None:
                    st = st_new
                    show(st)
            b.seq += 1
            pr.send_msg(b.sock, pr.STREAM_STOP, struct.pack("<H", 0), b.seq)
            while True:
                cmd, status, seq, raw = pr.recv_msg(b.sock)
                if cmd == pr.STREAM_PUSH:
                    tally.add(pr.parse_push(raw)[2])
                    continue
                if cmd != pr.STREAM_STOP or seq != b.seq or status != pr.ST_OK:
                    raise pr.ProtocolError(f"STREAM_STOP reply: cmd {cmd} seq {seq} status {status}")
                break
            st, recs = pr.parse_poll(raw)
        else:
            while time.monotonic() - t0 < a.seconds:
                time.sleep(a.poll)
                st, recs = pr.parse_poll(b.call(pr.POLL_RESULTS))
                tally.add(recs)
                show(st)
            st, recs = pr.parse_poll(b.call(pr.STREAM_STOP, struct.pack("<H", 0)))
        tally.add(recs)
    except KeyboardInterrupt:
        try:
            b.call(pr.STREAM_STOP, struct.pack("<H", 0))
        except Exception:  # noqa: BLE001
            pass
        print("\nInterrupted")
        return 3
    except (OSError, pr.ProtocolError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    tot, ok = tally.blocks
    npk = len(tally.packets)
    secs = st.elapsed_ms / 1e3 if st else a.seconds
    samples = st.datagrams * SLOT_SAMPLES
    print(f"\nStream: {st.datagrams} slots = {samples} samples in {secs:.1f} s ({samples / max(secs, 1e-9) / 1e6:.2f} Msps), "
          f"{st.lost} overflows (samples lost), {st.ring_full} ring full, ring max {st.ring_max} slots "
          f"({st.ring_max * SLOT_SAMPLES / a.rate * 1e3:.1f} ms), {st.dma_chunks} DMA chunks, {st.dma_errors} DMA errors, "
          f"{st.stalls} stalls")
    took = st.det_samples + ((st.fed_samples - st.det_samples) // (1 << 32)) * (1 << 32) \
        if st.fed_samples >= st.det_samples else st.det_samples
    print(f"  fed {st.fed_samples} samples to the receiver, it took {took}")
    print(f"  triggers {tally.triggers}, timing ok {tally.timing_ok}, control fields {tally.cf_ok}/{tally.cf} ok")
    if tot:
        print(f"  packets {npk} ({npk / max(secs, 1e-9):.1f}/s), transport blocks {ok}/{tot} CRC ok, "
              f"BLER {(tot - ok) / tot:.2e}, {tally.bits_ok / max(secs, 1e-9) / 1e6:.3f} Mbit/s decoded")
    else:
        print("  no transport blocks: is the Pluto sending at the same LO? (fmc_capture.py shows the signal)")
    return 0 if tot and ok == tot and st.lost == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
