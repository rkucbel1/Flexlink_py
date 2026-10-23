#!/usr/bin/env python3
"""
Run_SNR_Sweep.py  (lives in Flexlink_py/App)

Monte Carlo SNR sweep of the whole link: for every SNR point, packets are sent through
Tx/Run_Tx_Chain.py and Rx/Run_Rx_Chain.py until enough block errors are counted or a
packet limit is reached, and BER, BLER, PER and EVM are recorded.

    once         Tx/Initialize_Tx.py, Rx/Initialize_Rx.py      (parameters do not change)
    per point    a copy of Channel_Defects.json with SNR1..4_dB = the point's SNR and
                 ThermalNoise = 1, written to the sweep folder. Your own
                 Channel_Defects.json is never changed; every other defect setting is used
                 as it is.
    per packet   Run_Tx_Chain.py --seed s --channel-seed s --defects-file <copy>
                 Run_Rx_Chain.py --no-model --defects-file <copy>
                 (new random bits, channel and noise for every packet; the seeds make
                 the sweep repeatable with --seed)

Stopping rules for a point: --target-errors block errors, or --max-packets packets,
whichever comes first. The sweep ends early after a point with no block error at all
(higher SNRs would not have any either), unless --no-early-stop.

Counting
    BLER  failed CRCs / blocks sent; a packet that is not detected counts all its blocks
          as failed
    PER   packets with any failed block (or not detected) / packets sent
    BER   bit errors / bits, over the packets that were decoded (missed packets are
          counted in 'missed' and in BLER/PER, not in BER)
    EVM   ray 1 after equalization vs the transmitted symbols (Flexlink_Plots.compute_evm),
          averaged over the decoded packets as mean error power, then in % and dB
    Measured SNR: the equalizer's SNR_dB of ray 1, averaged in linear power
    Data rate: transport block bits per packet (TransportBlockSize * AvailableDataBlocks)
          / duration of the whole packet, preamble included (Flexlink_Plots.data_rate)
    Goodput: data rate * (1 - BLER)

Output: App/Sweeps/Sweep_<date>_<time>/  (or --out-dir)
    results.csv     one row per SNR point, written after every point
    settings.json   sweep arguments, seed, and copies of the three parameter files
    Channel_Defects_sweep.json   the copy used for the current point
    errors.log      output of any step that stopped with an error
    sweep.png       BER/BLER and EVM against SNR (at the end)

Progress lines on stdout (read by the GUI):
    PROGRESS <text>
    POINT <json of the finished row>
    DONE <out dir>
Create a file named STOP in the sweep folder to stop after the current packet
(the GUI's Stop button does this); the finished points are kept.

Usage
-----
    python App/Run_SNR_Sweep.py                                  # 0..20 dB in 2 dB steps
    python App/Run_SNR_Sweep.py --start -4 --stop 10 --step 1 --max-packets 200
    python App/Run_SNR_Sweep.py --seed 7                         # repeatable sweep

Exit status: 0 finished (or stopped), 3 a Tx step or the initialization failed.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

APP = Path(__file__).resolve().parent
sys.path.insert(0, str(APP))
try:
    from flexlink_paths import CHANNEL_DEFECTS, ROOT, RX, TX
except ImportError:
    sys.path.insert(0, str(APP.parent))
    from flexlink_paths import CHANNEL_DEFECTS, ROOT, RX, TX

DEFECTS_FILE = CHANNEL_DEFECTS / "Channel_Defects.json"
TX_PARAMS = TX / "Parameters_Tx_payload.json"
RX_PARAMS = RX / "Parameters_Rx_payload.json"
SWEEPS = APP / "Sweeps"
MAX_RAYS = 4

COLUMNS = ["snr_db", "packets", "missed", "blocks", "block_errors", "bler", "packet_errors",
           "per", "bits", "bit_errors", "ber", "evm_pct", "evm_db", "measured_snr_db",
           "data_rate_mbps", "goodput_mbps", "seconds"]


def say(kind: str, text: str) -> None:
    print(f"{kind} {text}", flush=True)


def run(cmd: list) -> tuple[int, str]:
    p = subprocess.run([sys.executable, *map(str, cmd)], capture_output=True, text=True, cwd=ROOT)
    return p.returncode, p.stdout + p.stderr


def parse_rx(out: str) -> dict:
    """Final CRC and bit-error result from Run_Rx_Chain.py's summary."""
    m = re.findall(r"CRC (\d+)/(\d+)\s+blocks [^\n]*?bit errors (\S+)", out)
    if not m:
        return {}
    ok, total, bit_err = m[-1]
    return {"crc_ok": int(ok), "blocks": int(total),
            "bit_errors": int(bit_err) if bit_err.isdigit() else None}


def packet_evm(since: float) -> tuple[float | None, float | None]:
    """(error power ratio, measured SNR linear) for ray 1 of this packet, or Nones."""
    try:
        import Flexlink_Plots as P
        import h5py
        rx_params = json.loads(RX_PARAMS.read_text())
        mask = P.load_data_mask(rx_params)
        evm = P.load_evm(rx_params, mask, since)
        ratio = 10 ** (evm["db"] / 10)
        snr = None
        eq = P.grid_path(rx_params, *P.CONSTELLATIONS["after"][:2])
        with h5py.File(eq, "r") as f:
            if "SNR_dB" in f.attrs:
                snr = 10 ** (float(f.attrs["SNR_dB"]) / 10)
        return ratio, snr
    except Exception:
        return None, None


def write_defects_copy(base: dict, snr_db: float, path: Path) -> None:
    d = json.loads(json.dumps(base))
    d.setdefault("Mode", {})["ThermalNoise"] = 1
    s = d.setdefault("Settings", {})
    for k in range(1, MAX_RAYS + 1):
        s[f"SNR{k}_dB"] = snr_db
    path.write_text(json.dumps(d, indent=2) + "\n")


def snr_points(start: float, stop: float, step: float) -> list[float]:
    n = int(math.floor((stop - start) / step + 1e-9)) + 1
    return [round(start + i * step, 6) for i in range(max(n, 0))]


def tx_data_rate(since: float) -> float:
    """Transport block bits per packet / whole-packet duration, in Mbps (NaN if unknown)."""
    try:
        import Flexlink_Plots as P
        return P.data_rate(since)["mbps"]
    except Exception:
        return float("nan")


def finish_row(snr, acc, t0, rate_mbps=float("nan")) -> dict:
    blocks, bits = acc["blocks"], acc["bits"]
    row = {
        "snr_db": snr, "packets": acc["packets"], "missed": acc["missed"],
        "blocks": blocks, "block_errors": acc["block_errors"],
        "bler": acc["block_errors"] / blocks if blocks else float("nan"),
        "packet_errors": acc["packet_errors"],
        "per": acc["packet_errors"] / acc["packets"] if acc["packets"] else float("nan"),
        "bits": bits, "bit_errors": acc["bit_errors"],
        "ber": acc["bit_errors"] / bits if bits else float("nan"),
        "evm_pct": float("nan"), "evm_db": float("nan"), "measured_snr_db": float("nan"),
        "seconds": round(time.time() - t0, 1),
    }
    # goodput: the transport block rate that actually gets through (missed packets included)
    row["data_rate_mbps"] = rate_mbps
    row["goodput_mbps"] = rate_mbps * (1 - row["bler"]) if blocks else float("nan")
    if acc["evm"]:
        r = sum(acc["evm"]) / len(acc["evm"])
        row["evm_db"], row["evm_pct"] = 10 * math.log10(r), 100 * math.sqrt(r)
    if acc["snr"]:
        row["measured_snr_db"] = 10 * math.log10(sum(acc["snr"]) / len(acc["snr"]))
    return row


def save_plot(rows: list[dict], out_dir: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import Flexlink_Plots as P
        fig = plt.figure(figsize=(10, 4.5))
        P.draw_sweep(fig, rows, title=out_dir.name)
        fig.savefig(out_dir / "sweep.png", dpi=110)
        plt.close(fig)
    except Exception as e:
        say("PROGRESS", f"sweep.png not written: {e}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="SNR sweep: BER, BLER, PER and EVM against SNR.")
    ap.add_argument("--start", type=float, default=0.0, help="first SNR (dB), default 0")
    ap.add_argument("--stop", type=float, default=20.0, help="last SNR (dB), default 20")
    ap.add_argument("--step", type=float, default=2.0, help="SNR step (dB), default 2")
    ap.add_argument("--max-packets", type=int, default=50, help="packets per point at most (default 50)")
    ap.add_argument("--target-errors", type=int, default=100,
                    help="stop a point after this many block errors (default 100)")
    ap.add_argument("--no-early-stop", action="store_true",
                    help="keep going after a point with no block errors")
    ap.add_argument("--seed", type=int, help="base random seed (default: random, saved in settings.json)")
    ap.add_argument("--out-dir", type=Path, help="sweep folder (default: App/Sweeps/Sweep_<date>_<time>)")
    a = ap.parse_args(argv)
    if a.step <= 0 or a.stop < a.start or a.max_packets < 1 or a.target_errors < 1:
        ap.error("need step > 0, stop >= start, max-packets >= 1, target-errors >= 1")

    # absolute: the step scripts run with the project root as working directory
    out = (a.out_dir or SWEEPS / datetime.now().strftime("Sweep_%Y%m%d_%H%M%S")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    stop_file = out / "STOP"
    stop_file.unlink(missing_ok=True)
    seed = a.seed if a.seed is not None else random.randrange(1, 2**30)
    defects_base = json.loads(DEFECTS_FILE.read_text())
    defects_copy = out / "Channel_Defects_sweep.json"
    points = snr_points(a.start, a.stop, a.step)
    errors_log = out / "errors.log"

    say("PROGRESS", f"Initializing (sweep folder {out})")
    for label, script in (("Initialize_Tx", TX / "Initialize_Tx.py"), ("Initialize_Rx", RX / "Initialize_Rx.py")):
        rc, txt = run([script])
        if rc:
            errors_log.write_text(f"===== {label} (exit {rc})\n{txt}")
            say("PROGRESS", f"{label} failed (exit {rc}), see {errors_log}")
            return 3

    tx_params = json.loads(TX_PARAMS.read_text())
    calc = tx_params.get("Calculated", {})
    bits_per_packet = int(calc.get("AvailableDataBlocks", 0)) * int(calc.get("TransportBlockSize", 0))
    blocks_per_packet = int(calc.get("AvailableDataBlocks", 0))
    (out / "settings.json").write_text(json.dumps({
        "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(a).items()},
        "seed": seed, "snr_points_db": points, "started": datetime.now().isoformat(timespec="seconds"),
        "Parameters_Tx_payload": tx_params,
        "Parameters_Rx_payload": json.loads(RX_PARAMS.read_text()),
        "Channel_Defects": defects_base,
    }, indent=2) + "\n")

    rows: list[dict] = []
    rate_mbps = float("nan")
    with (out / "results.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        stopped = False
        for i, snr in enumerate(points):
            write_defects_copy(defects_base, snr, defects_copy)
            acc = {"packets": 0, "missed": 0, "blocks": 0, "block_errors": 0, "packet_errors": 0,
                   "bits": 0, "bit_errors": 0, "evm": [], "snr": []}
            t0 = time.time()
            while acc["packets"] < a.max_packets and acc["block_errors"] < a.target_errors:
                if stop_file.exists():
                    stopped = True
                    break
                s = seed + i * 100_000 + acc["packets"]
                say("PROGRESS", f"SNR {snr:g} dB ({i + 1}/{len(points)}), packet "
                                f"{acc['packets'] + 1}/{a.max_packets}, {acc['block_errors']} block errors")
                since = time.time()
                rc, txt = run([TX / "Run_Tx_Chain.py", "--seed", s, "--channel-seed", s,
                               "--defects-file", defects_copy])
                if rc:
                    errors_log.write_text(f"===== Run_Tx_Chain SNR {snr} seed {s} (exit {rc})\n{txt}")
                    say("PROGRESS", f"Run_Tx_Chain failed (exit {rc}), see {errors_log}")
                    return 3
                if rate_mbps != rate_mbps:  # NaN: not known yet (same for every packet)
                    rate_mbps = tx_data_rate(since)
                rc, txt = run([RX / "Run_Rx_Chain.py", "--no-model", "--defects-file", defects_copy])
                res = parse_rx(txt) if rc in (0, 1) else {}
                acc["packets"] += 1
                acc["blocks"] += blocks_per_packet
                if not res:  # not detected (exit 2) or a step stopped (exit 3)
                    acc["missed"] += 1
                    acc["block_errors"] += blocks_per_packet
                    acc["packet_errors"] += 1
                    if rc not in (0, 1, 2):
                        with errors_log.open("a") as fh_err:
                            fh_err.write(f"===== Run_Rx_Chain SNR {snr} seed {s} (exit {rc})\n{txt}\n")
                    continue
                failed = res["blocks"] - res["crc_ok"]
                acc["block_errors"] += failed
                acc["packet_errors"] += 1 if failed or res["bit_errors"] else 0
                if res["bit_errors"] is not None:
                    acc["bits"] += bits_per_packet
                    acc["bit_errors"] += res["bit_errors"]
                ratio, snr_lin = packet_evm(since)
                if ratio is not None:
                    acc["evm"].append(ratio)
                if snr_lin is not None:
                    acc["snr"].append(snr_lin)
            if acc["packets"]:
                row = finish_row(snr, acc, t0, rate_mbps)
                rows.append(row)
                w.writerow(row)
                fh.flush()
                say("POINT", json.dumps(row))
            if stopped:
                say("PROGRESS", "Stopped")
                break
            if acc["packets"] and acc["block_errors"] == 0 and not a.no_early_stop:
                say("PROGRESS", f"No block errors at {snr:g} dB: sweep ends here")
                break
    stop_file.unlink(missing_ok=True)
    save_plot(rows, out)
    say("DONE", str(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
