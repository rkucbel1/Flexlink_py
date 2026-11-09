#!/usr/bin/env python3
"""FlexlinkCaptureWithCoreBoard.py -- capture a few seconds of IQ with an ESP32-S31 Function-CoreBoard-1.

The CoreBoard runs ESP-SDR's streaming firmware ("ESP32-S31 Function-CoreBoard . Ethernet / USB for
SoapyESPSDR" profile in the ESP-SDR web flasher). This script talks to that firmware directly over
Gigabit Ethernet, no SoapySDR needed:

    HTTP  POST http://<board>/api/rx   {"op": "status" | "configure" | "start" | "stop", ...}
    UDP   one datagram per packet, sent back to this PC:
              32-byte header ("ESR2", version, bits, epoch, rate, sample position, time_us)
              + 672 complex samples, 8-bit signed, interleaved I, Q

It starts the stream, records CaptureSeconds (default 8 s) at 40 Msps, stops the stream and writes
(always overwriting):

    App/CoreBoard/CoreBoard_Capture.cs8    int8 I,Q,I,Q,... at 40 Msps (lost samples filled with 0)
    App/CoreBoard/CoreBoard_Capture.json   settings, levels, lost samples, board counters

Start the transmitter (App/FlexlinkTxWithPluto.py on the other PC) within a second or two of
starting this script. Then decode with App/FlexlinkProcessCoreBoardCapturedWaveform.py.

Settings come from App/Parameters_CoreBoard.json (made with defaults on the first run; the carrier
defaults to TxLO_MHz of App/Parameters_Pluto.json). Command-line options override them for one
run; --save writes them back.

Usage
-----
    python App/FlexlinkCaptureWithCoreBoard.py                       # 8 s with the saved settings
    python App/FlexlinkCaptureWithCoreBoard.py --seconds 10 --gain 30
    python App/FlexlinkCaptureWithCoreBoard.py --host 192.168.1.50 --save
    python App/FlexlinkCaptureWithCoreBoard.py --status              # board status only, no change
    python App/FlexlinkCaptureWithCoreBoard.py --configure           # apply the settings, no capture

Linux: 40 Msps is ~60,000 UDP datagrams/s (~690 Mbit/s). Raise the socket buffer limit once per
boot or the PC drops datagrams:   sudo sysctl -w net.core.rmem_max=268435456
The whole path (NIC, cable, switch) must be gigabit, and the firewall must let UDP in from the board.

Exit status: 0 capture written, 1 written but samples were lost, 3 error (nothing written).
"""
from __future__ import annotations

import argparse
import gc
import json
import os
import socket
import struct
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR.parent))
from flexlink_paths import APP  # noqa: E402

SETTINGS_JSON = APP / "Parameters_CoreBoard.json"
PLUTO_JSON = APP / "Parameters_Pluto.json"
OUT_DIR = APP / "CoreBoard"
CAPTURE = OUT_DIR / "CoreBoard_Capture.cs8"
CAPTURE_META = OUT_DIR / "CoreBoard_Capture.json"

# ESR2 wire format (ESP-SDR esp32s31 streaming firmware, protocol 2)
MAGIC = b"ESR2"
HEADER = struct.Struct("<4sHHIIQQ")          # magic, version, bits, epoch, rate, sample, time_us
HDR_LEN = HEADER.size                         # 32
PAYLOAD = 1344
SAMPLES_PER_PKT = PAYLOAD // 2                # 672 complex samples
PKT_LEN = HDR_LEN + PAYLOAD                   # 1376
RATES = (4_000_000, 8_000_000, 16_000_000, 20_000_000, 40_000_000)

DEFAULTS = {"Address": "192.168.1.100", "HttpPort": 80, "Frequency_MHz": 2400, "SampleRate": 40_000_000,
            "AGC": 0, "Gain": 40, "Bandwidth_MHz": 0, "DC_Correction": 1, "CaptureSeconds": 8.0,
            "UdpPort": 0, "SocketBuffer_MB": 256}
COMMENTS = {
    "Address_comment": "IP address of the CoreBoard (DHCP; printed on its UART log and shown on http://<board>/)",
    "HttpPort_comment": "HTTP port of the streaming firmware",
    "Frequency_MHz_comment": "receive carrier, whole MHz, 2150 to 2800 (match TxLO_MHz of the Pluto)",
    "SampleRate_comment": "40000000 for Flexlink (40 Msps needs Gigabit Ethernet); 4/8/16/20 Msps also exist",
    "AGC_comment": "1: the chip's Wi-Fi AGC; 0: fixed Gain (recommended, the AGC is tuned for Wi-Fi preambles)",
    "Gain_comment": "manual gain: PHY gain-table index (not dB), 0 to gain_max (reported by the board)",
    "Bandwidth_MHz_comment": "analog filter, 13 to 54 MHz; 0 = follow the sample rate",
    "DC_Correction_comment": "1: the firmware removes the residual DC offset (manual gain only)",
    "CaptureSeconds_comment": "seconds recorded (start the transmitter in the first second or two)",
    "UdpPort_comment": "UDP port on this PC for the samples; 0 = any free port",
    "SocketBuffer_MB_comment": "UDP receive buffer asked for (Linux caps it at net.core.rmem_max)",
}


# ---------- settings ----------
def load_settings() -> dict:
    try:
        data = json.loads(SETTINGS_JSON.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except json.JSONDecodeError as e:
        raise SystemExit(f"Invalid JSON in {SETTINGS_JSON}: line {e.lineno}: {e.msg}")
    sec = data.setdefault("CoreBoard", {})
    if "Frequency_MHz" not in sec:                       # first run: follow the Pluto's carrier
        try:
            sec["Frequency_MHz"] = int(round(json.loads(PLUTO_JSON.read_text())["Pluto"]["TxLO_MHz"]))
        except (OSError, ValueError, KeyError):
            pass
    for k, v in DEFAULTS.items():
        sec.setdefault(k, v)
        if f"{k}_comment" in COMMENTS:
            sec.setdefault(f"{k}_comment", COMMENTS[f"{k}_comment"])
    return data


def write_json_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2)
            fh.write("\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# ---------- board control ----------
class Board:
    def __init__(self, host: str, port: int = 80, timeout: float = 5.0):
        self.url = f"http://{host}:{port}/api/rx"
        self.host, self.timeout = host, timeout

    def request(self, op: str, **fields) -> dict:
        body = json.dumps({"op": op, **fields}).encode()
        req = urllib.request.Request(self.url, data=body, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                reply = json.loads(r.read().decode())
        except urllib.error.HTTPError as e:            # 400 carries {"error": ...}
            try:
                reply = json.loads(e.read().decode())
            except Exception:  # noqa: BLE001
                raise RuntimeError(f"board answered HTTP {e.code} to '{op}'") from e
        except (urllib.error.URLError, OSError) as e:
            raise RuntimeError(f"no answer from the CoreBoard at {self.url} ({getattr(e, 'reason', e)}). "
                               f"Is it powered, on Ethernet, running the SoapyESPSDR streaming firmware, "
                               f"and is Address in {SETTINGS_JSON.name} right?") from e
        if "error" in reply:
            raise RuntimeError(f"board refused '{op}' {fields}: {reply['error']}")
        return reply

    def status(self) -> dict:
        return self.request("status")


def status_lines(s: dict) -> list[str]:
    eth = s.get("ethernet_mbps", 0)
    return [f"{s.get('product', '?')} protocol {s.get('protocol', '?')} at {s.get('ip', '?')}: "
            f"Ethernet {str(eth) + ' Mbit/s' if eth else 'down'}, transport {s.get('transport')}",
            f"  frequency {s.get('frequency', 0) / 1e6:g} MHz, rate {s.get('rate', 0) / 1e6:g} Msps, "
            f"{'AGC' if s.get('agc') else 'gain ' + str(s.get('gain'))} (gain_max {s.get('gain_max')}), "
            f"bandwidth {s.get('effective_bandwidth', 0) / 1e6:g} MHz, DC correction {s.get('dc_correction')}",
            f"  counters since boot: capture drops {s.get('capture_drops')}, transport drops "
            f"{s.get('transport_drops')}"]


# ---------- capture ----------
def open_socket(port: int, buf_mb: int) -> tuple[socket.socket, int]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    want = int(buf_mb) << 20
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, want)
    got = s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    s.bind(("", int(port)))
    s.settimeout(0.5)
    return s, got


def receive(sock: socket.socket, n_pkts: int, progress) -> tuple[np.ndarray, int, int]:
    """Receive up to n_pkts datagrams into a (n_pkts, 1376) array. Returns (array, datagrams
    received, datagrams of the wrong size). Stops early if the stream stops for 2 s."""
    buf = np.empty((n_pkts, PKT_LEN), np.uint8)
    mv = memoryview(buf).cast("B")
    recv_into = sock.recv_into
    i = bad = 0
    t_begin = time.monotonic()
    t_first = t_last = None
    t_next = 0.0
    gc_was = gc.isenabled()
    gc.disable()                      # no collector pauses while 60k datagrams/s arrive
    try:
        while i < n_pkts:
            off = i * PKT_LEN
            try:
                n = recv_into(mv[off:off + PKT_LEN], PKT_LEN)
            except socket.timeout:
                now = time.monotonic()
                if t_first is None and now - t_begin > 3.0:
                    raise RuntimeError("no samples arrived within 3 s of 'start'. Check the PC firewall "
                                       "(UDP from the board must be allowed, e.g. sudo ufw allow from "
                                       "<board IP>) and that the board and this PC are on the same network.")
                if t_last is not None and now - t_last > 2.0:
                    break                     # the stream stopped
                continue
            if n != PKT_LEN:
                bad += 1
                continue
            i += 1
            if (i & 0x3FFF) == 0:             # every ~0.27 s at 40 Msps
                t_last = time.monotonic()
                if t_last >= t_next:
                    progress(t_last - t_first, i)
                    t_next = t_last + 1.0
            elif t_first is None:
                t_first = t_last = time.monotonic()
    finally:
        if gc_was:
            gc.enable()
    return buf, i, bad


def assemble(buf: np.ndarray, n: int, epoch_hint: int | None) -> tuple[dict, np.ndarray, np.ndarray]:
    """Check the headers; returns (info, sample positions, row order) for the packets of the newest
    epoch, sorted by position."""
    hdr = buf[:n, :HDR_LEN]
    magic_ok = (hdr[:, :4] == np.frombuffer(MAGIC, np.uint8)).all(axis=1)
    h = np.ascontiguousarray(hdr).view(np.dtype([("magic", "S4"), ("version", "<u2"), ("bits", "<u2"),
                                                  ("epoch", "<u4"), ("rate", "<u4"), ("sample", "<u8"),
                                                  ("time_us", "<u8")])).ravel()
    rows = np.flatnonzero(magic_ok)
    if rows.size == 0:
        raise RuntimeError("no ESR2 packets received (wrong firmware or protocol version?)")
    epoch = int(epoch_hint) if epoch_hint is not None and (h["epoch"][rows] == epoch_hint).any() \
        else int(h["epoch"][rows].max())
    rows = rows[h["epoch"][rows] == epoch]
    pos = h["sample"][rows].astype(np.int64)
    order = np.argsort(pos, kind="stable")
    rows, pos = rows[order], pos[order]
    keep = np.ones(rows.size, bool)
    keep[1:] = np.diff(pos) != 0                      # duplicates
    rows, pos = rows[keep], pos[keep]
    info = {"epoch": epoch, "version": int(h["version"][rows[0]]), "bits": int(h["bits"][rows[0]]),
            "rate": int(h["rate"][rows[0]]), "first_time_us": int(h["time_us"][rows[0]]),
            "bad_magic": int((~magic_ok).sum()), "other_epoch": int(magic_ok.sum() - order.size),
            "duplicates": int((~keep).sum())}
    return info, pos, rows


def write_capture(path: Path, buf: np.ndarray, rows: np.ndarray, pos: np.ndarray) -> dict:
    """Payloads in sample order into path (int8 I/Q), zeros where packets are missing.
    Returns sample counts and level statistics."""
    first = int(pos[0])
    total = int(pos[-1] - first) + SAMPLES_PER_PKT
    rel = pos - first
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    peak = 0
    sumsq = 0.0
    clip = 0
    s_i = s_q = 0.0
    chunk = 8192
    with open(tmp, "wb") as fh:
        written = 0                                   # complex samples written so far
        for a in range(0, rows.size, chunk):
            r = rows[a:a + chunk]
            p = rel[a:a + chunk]
            block_start = int(p[0])
            if block_start > written:                 # gap before this chunk
                np.zeros(2 * (block_start - written), np.int8).tofile(fh)
                written = block_start
            span = int(p[-1] - block_start) + SAMPLES_PER_PKT
            out = np.zeros((span, 2), np.int8)        # zeros where datagrams are missing
            payload = buf[r, HDR_LEN:].view(np.int8).reshape(r.size, SAMPLES_PER_PKT, 2)
            idx = (p - block_start)[:, None] + np.arange(SAMPLES_PER_PKT)[None, :]
            out[idx.ravel()] = payload.reshape(-1, 2)
            out[max(0, written - block_start):].tofile(fh)   # (no overlap for aligned positions)
            written = block_start + span
            v = payload.astype(np.int16)
            peak = max(peak, int(np.abs(v).max()))
            clip += int((np.abs(v) >= 127).sum())
            sumsq += float((v.astype(np.float64) ** 2).sum())
            s_i += float(v[..., 0].sum())
            s_q += float(v[..., 1].sum())
    os.replace(tmp, path)
    got = rows.size * SAMPLES_PER_PKT
    rms = np.sqrt(sumsq / max(1, 2 * got))           # per component
    return {"samples": total, "received_samples": got, "lost_samples": total - got,
            "lost_packets": total // SAMPLES_PER_PKT - rows.size,
            "peak": peak, "rms": float(rms), "clip_fraction": clip / max(1, 2 * got),
            "dc_i": s_i / max(1, got), "dc_q": s_q / max(1, got)}


def dbfs(v: float) -> float:
    return 20 * np.log10(max(v, 1e-9) / 128.0)


# ---------- main ----------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Capture IQ samples with an ESP32-S31 CoreBoard over Ethernet.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--host", help="board IP address (Address), or IP:PORT (e.g. the emulator at 127.0.0.1:8080)")
    ap.add_argument("--freq-mhz", type=int, help="receive carrier, whole MHz (Frequency_MHz)")
    ap.add_argument("--seconds", type=float, help="capture length (CaptureSeconds)")
    ap.add_argument("--gain", type=int, help="manual gain index; turns AGC off")
    ap.add_argument("--agc", action="store_true", help="use the chip's AGC instead of a fixed gain")
    ap.add_argument("--bandwidth-mhz", type=int, help="analog bandwidth 13..54 MHz, 0 = follow the rate")
    ap.add_argument("--rate", type=int, choices=RATES, help="sample rate (SampleRate), Flexlink needs 40000000")
    ap.add_argument("--save", action="store_true", help="write the settings used to Parameters_CoreBoard.json")
    ap.add_argument("--status", action="store_true",
                    help="print the board's status and the settings a capture would apply, change nothing")
    ap.add_argument("--configure", action="store_true",
                    help="apply the settings to the board (no capture) and print its status")
    ap.add_argument("--out", type=Path, default=CAPTURE, help="capture file (default App/CoreBoard/...cs8)")
    a = ap.parse_args(argv)

    data = load_settings()
    cb = data["CoreBoard"]
    if a.host: cb["Address"] = a.host
    if a.freq_mhz is not None: cb["Frequency_MHz"] = a.freq_mhz
    if a.seconds is not None: cb["CaptureSeconds"] = a.seconds
    if a.gain is not None: cb["Gain"], cb["AGC"] = a.gain, 0
    if a.agc: cb["AGC"] = 1
    if a.bandwidth_mhz is not None: cb["Bandwidth_MHz"] = a.bandwidth_mhz
    if a.rate is not None: cb["SampleRate"] = a.rate
    if not SETTINGS_JSON.is_file() or a.save:
        write_json_atomic(SETTINGS_JSON, data)
        print(f"Settings written to {SETTINGS_JSON}")

    host, http_port = str(cb["Address"]), int(cb["HttpPort"])
    if host.count(":") == 1:                             # --host 127.0.0.1:8080 (e.g. the emulator)
        host, _, p = host.partition(":")
        http_port = int(p)
    board = Board(host, http_port)
    try:
        st = board.status()
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 3
    print("\n".join(status_lines(st)))

    rate = int(cb["SampleRate"])
    seconds = float(cb["CaptureSeconds"])
    if st.get("ethernet_mbps") != 1000 and rate > 20_000_000:
        print(f"Error: the board's Ethernet link is {st.get('ethernet_mbps')} Mbit/s; {rate / 1e6:g} Msps "
              f"needs 1000 Mbit/s (check the cable, switch and PC network card)", file=sys.stderr)
        return 3
    cfg = {"frequency": int(cb["Frequency_MHz"]) * 1_000_000, "rate": rate, "agc": int(cb["AGC"]),
           "bandwidth": int(cb["Bandwidth_MHz"]) * 1_000_000, "dc_correction": int(cb["DC_Correction"])}
    if not cfg["agc"]:
        gmax = int(st.get("gain_max", 127))
        if not 0 <= int(cb["Gain"]) <= gmax:
            print(f"Error: Gain {cb['Gain']} is outside 0..{gmax} (gain_max of this board)", file=sys.stderr)
            return 3
        cfg["gain"] = int(cb["Gain"])
    applies = (f"{cfg['frequency'] / 1e6:g} MHz, {rate / 1e6:g} Msps, "
               f"{'AGC' if cfg['agc'] else 'gain ' + str(cfg['gain'])}, bandwidth "
               f"{(cfg['bandwidth'] or rate) / 1e6:g} MHz, DC correction {cfg['dc_correction']}")
    if a.status:
        print(f"A capture applies: {applies} (from {SETTINGS_JSON.name} and the options given)")
        return 0
    if a.configure:
        try:
            board.request("configure", **cfg)
            print(f"Applied: {applies}")
            print("\n".join(status_lines(board.status())))
        except RuntimeError as e:
            print(f"Error: {e}", file=sys.stderr)
            return 3
        return 0

    n_pkts = int(np.ceil(seconds * rate / SAMPLES_PER_PKT))
    mem_gb = n_pkts * PKT_LEN / 1e9
    sock, got_buf = open_socket(int(cb["UdpPort"]), int(cb["SocketBuffer_MB"]))
    port = sock.getsockname()[1]
    if got_buf < (16 << 20):
        print(f"Warning: the UDP receive buffer is only {got_buf >> 10} KiB ({got_buf / (rate * 2.05) * 1e3:.1f} ms "
              f"at {rate / 1e6:g} Msps); datagrams are likely to be lost. Raise the limit (once per boot):\n"
              f"    sudo sysctl -w net.core.rmem_max=268435456")
    stats0 = {k: st.get(k, 0) for k in ("capture_drops", "transport_drops", "capture_overruns", "ring_drops")}

    print(f"Capture: {seconds:g} s at {rate / 1e6:g} Msps ({n_pkts} datagrams, {mem_gb:.2f} GB in memory), "
          f"carrier {cb['Frequency_MHz']} MHz, {'AGC' if cfg['agc'] else 'gain ' + str(cfg['gain'])}, "
          f"UDP port {port}, socket buffer {got_buf >> 20} MiB")
    started = False
    info = None
    try:
        reply = board.request("start", port=port, **cfg)   # applies cfg, then streams to this PC
        started = True
        t_start = time.monotonic()
        print(">>> Capturing now - start the transmitter. <<<", flush=True)

        def progress(t, i):
            print(f"  {t:4.1f} s  {i * SAMPLES_PER_PKT / 1e6:7.1f} Msamples", flush=True)

        buf, n, bad = receive(sock, n_pkts, progress)
        elapsed = time.monotonic() - t_start
    except KeyboardInterrupt:
        print("Interrupted - nothing written")
        return 3
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 3
    finally:
        if started:
            try:
                board.request("stop")
            except RuntimeError as e:
                print(f"Warning: stop failed ({e}); the board may still be streaming", file=sys.stderr)
        sock.close()

    try:
        st1 = board.status()
    except RuntimeError:
        st1 = {}
    if n == 0:
        print("Error: no samples received", file=sys.stderr)
        return 3
    try:
        info, pos, rows = assemble(buf, n, reply.get("epoch"))
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 3
    if info["rate"] != rate:
        print(f"Warning: the packets say {info['rate']} sps, {rate} was asked for")
    lv = write_capture(a.out, buf, rows, pos)
    del buf
    board_drops = {k: int(st1.get(k, 0)) - int(stats0[k]) for k in stats0} if st1 else {}

    meta = {"File": a.out.name, "Format": "int8 interleaved I,Q (cs8)", "SampleRate": info["rate"],
            "CarrierHz": cfg["frequency"], "Samples": lv["samples"], "Seconds": lv["samples"] / info["rate"],
            "CapturedAt": time.strftime("%Y-%m-%d %H:%M:%S"), "Board": cb["Address"],
            "AGC": cfg["agc"], "Gain": cfg.get("gain"), "GainMax": st.get("gain_max"),
            "Bandwidth": st1.get("effective_bandwidth", cfg["bandwidth"]), "DC_Correction": cfg["dc_correction"],
            "Epoch": info["epoch"], "FirstSamplePosition": int(pos[0]), "FirstTime_us": info["first_time_us"],
            "LostSamples": lv["lost_samples"], "LostPackets": lv["lost_packets"],
            "BadDatagrams": bad + info["bad_magic"], "BoardCounterChange": board_drops,
            "Peak": lv["peak"], "RmsPerComponent": lv["rms"], "ClipFraction": lv["clip_fraction"],
            "DC": [lv["dc_i"], lv["dc_q"]], "ReceiveSeconds": elapsed}
    write_json_atomic(CAPTURE_META, meta)

    lost_pct = 100 * lv["lost_samples"] / max(1, lv["samples"])
    print(f"Wrote {a.out} ({lv['samples']} samples, {lv['samples'] / info['rate']:.2f} s, "
          f"{a.out.stat().st_size / 1e6:.0f} MB) and {CAPTURE_META.name}")
    print(f"  lost: {lv['lost_samples']} samples in {lv['lost_packets']} datagram(s) ({lost_pct:.3f} %)"
          + (f"; board counters {board_drops}" if any(board_drops.values()) else ""))
    hint = ""
    if lv["clip_fraction"] > 1e-4:
        hint = "  CLIPPING: lower the gain (--gain) or the Pluto's Tx gain"
    elif lv["peak"] < 24:
        hint = "  weak: raise the gain (--gain) or move the boards closer"
    print(f"  level: peak {lv['peak']} of 127 ({dbfs(lv['peak']):.1f} dBFS), rms {lv['rms']:.1f} "
          f"({dbfs(lv['rms']):.1f} dBFS), clipped {100 * lv['clip_fraction']:.3f} %, "
          f"DC {lv['dc_i']:+.2f}/{lv['dc_q']:+.2f}" + hint)
    print("Next: python App/FlexlinkProcessCoreBoardCapturedWaveform.py")
    return 1 if lv["lost_samples"] else 0


if __name__ == "__main__":
    sys.exit(main())
