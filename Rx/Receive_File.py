#!/usr/bin/env python3
"""
Receive_File.py  (lives in Flexlink_py/Rx)

After step 9: takes the decoded transport blocks of one packet (TransportBlock_Bitstream and
CRC_Pass, written by Get_Decoded_Bitstream.py) and adds the blocks whose CRC passed to the
file being received. Blocks are collected in Rx/FileTransfer_State/ across runs, so packets
can be received one Rx run at a time, in any order, and resent blocks fill the gaps. When
every block of a file is in, the file is written to Rx/Received_Files/ under its own name,
after its size and SHA-256 are checked against the ones the Tx sent.

The block format is described in Flexlink_BasicModules/File_Transfer/File_Framing.py.

Usage
-----
    python Rx/Receive_File.py                          # the blocks of the last Rx run
    python Rx/Receive_File.py --blocks Rx/Bitstream_TransportBlks_Rx_Ray2.h5
    python Rx/Receive_File.py --status                 # what has been received so far
    python Rx/Receive_File.py --reset                  # forget every partly received file

Exit status: 0 a file was completed by this packet (or --status/--reset), 1 blocks still
missing, 2 the blocks file is older than the last decimated waveform (the Rx chain did not
decode this packet), 3 error.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

RX = Path(__file__).resolve().parent
sys.path.insert(0, str(RX))
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import BASIC_MODULES  # noqa: E402
from Rx_Antennas import antennas_in_run, target_name  # noqa: E402

sys.path.insert(0, str(BASIC_MODULES / "File_Transfer"))
from File_Framing import FileReceiver  # noqa: E402

PARAMS_FILE = RX / "Parameters_Rx_payload.json"
STATE_DIR = RX / "FileTransfer_State"
OUT_DIR = RX / "Received_Files"
BLOCKS_KEY = "Bitstream_TransportBlks_Rx"
DEC_KEY = "Waveform_Payload_Dec_Rx"
EXIT_DONE, EXIT_MISSING, EXIT_STALE, EXIT_ERROR = 0, 1, 2, 3


def default_blocks_file(params_path: Path = PARAMS_FILE) -> Path:
    """The file step 9 of the last Rx run wrote (_Combined or _Ray1)."""
    params = json.loads(Path(params_path).read_text())
    name = params.get("File_Rx", {}).get(BLOCKS_KEY) or "Bitstream_TransportBlks_Rx.h5"
    return RX / target_name(name, None, antennas_in_run(params))


def is_stale(blocks: Path, params_path: Path = PARAMS_FILE) -> bool:
    """True if the blocks file is older than the decimated waveform of the last Rx run."""
    params = json.loads(Path(params_path).read_text())
    dec = RX / (params.get("File_Rx", {}).get(DEC_KEY) or "Waveform_Payload_Dec_Rx.h5")
    return dec.is_file() and Path(blocks).stat().st_mtime < dec.stat().st_mtime


def load_blocks(path: Path) -> tuple[np.ndarray, np.ndarray]:
    import h5py
    if not Path(path).is_file():
        raise FileNotFoundError(f"blocks file not found: {path} - run Rx/Run_Rx_Chain.py first")
    with h5py.File(path, "r") as f:
        if "TransportBlock_Bitstream" not in f or "CRC_Pass" not in f:
            raise ValueError(f"{path} needs TransportBlock_Bitstream and CRC_Pass (Get_Decoded_Bitstream.py)")
        bits = np.asarray(f["TransportBlock_Bitstream"][()]).ravel().astype(np.uint8)
        crc = np.asarray(f["CRC_Pass"][()]).ravel()
    return bits, crc


def receive(bits: np.ndarray, crc: np.ndarray, state_dir: Path = STATE_DIR, out_dir: Path = OUT_DIR):
    """Add one packet. Returns (report, {file_id: status}, [files written])."""
    rx = FileReceiver(state_dir)
    rep = rx.add_packet(bits, crc)
    status, written = {}, []
    for fid in rep.file_ids:
        st = rx.status(fid)
        if st["complete"] and not st["done"]:
            try:
                written.append(rx.assemble(fid, out_dir))
                st = rx.status(fid)
            except ValueError as exc:
                # A block passed its CRC but was wrong; there is no telling which one,
                # so start this file again (its blocks are all resent).
                rx.reset(fid)
                st = {**st, "received": 0, "complete": False, "missing": list(range(st["total"])),
                      "error": f"{exc} - received blocks discarded"}
        status[fid] = st
    return rep, status, written


def describe(rep) -> str:
    parts = [f"{rep.blocks} block(s): {rep.new} new", f"{rep.duplicate} already had"]
    if rep.crc_fail:
        parts.append(f"{rep.crc_fail} CRC failed")
    if rep.foreign:
        parts.append(f"{rep.foreign} not file data")
    if rep.rejected:
        parts.append(f"{rep.rejected} did not fit the file")
    return ", ".join(parts)


def ranges(idx: list[int], limit: int = 12) -> str:
    out, i = [], 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and idx[j + 1] == idx[j] + 1:
            j += 1
        out.append(str(idx[i]) if i == j else f"{idx[i]}-{idx[j]}")
        i = j + 1
    return ", ".join(out[:limit]) + (", ..." if len(out) > limit else "")


def print_status(rx: FileReceiver) -> None:
    ids = rx.file_ids()
    if not ids:
        print("No file blocks received yet")
    for fid in ids:
        st = rx.status(fid)
        line = f"File ID {fid}: {st['received']} of {st['total']} block(s) from {st['packets']} packet(s)"
        if st["done"]:
            line += f", written to {st['done']}"
        elif st["missing"]:
            line += f", missing {ranges(st['missing'])}"
        print(line)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Add the decoded blocks of one packet to the file being received.")
    ap.add_argument("--blocks", type=Path, help="decoded blocks (default: what the last Rx run wrote)")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--state-dir", type=Path, default=STATE_DIR, help="where partly received files are kept")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR, help="where finished files are written")
    ap.add_argument("--force", action="store_true", help="use the blocks file even if it looks stale")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--status", action="store_true", help="show what has been received and stop")
    g.add_argument("--reset", action="store_true", help="forget every partly received file and stop")
    a = ap.parse_args(argv)

    try:
        if a.status or a.reset:
            rx = FileReceiver(a.state_dir)
            if a.reset:
                n = len(rx.file_ids())
                rx.reset()
                print(f"Forgot {n} file(s) in {a.state_dir}")
            else:
                print_status(rx)
            return EXIT_DONE

        path = a.blocks or default_blocks_file(a.params)
        if a.blocks is None and not a.force and path.is_file() and is_stale(path, a.params):
            print(f"{path.name} is older than the last decimated waveform: the Rx chain did not decode "
                  f"this packet (no packet, or the control field failed). Nothing added (--force to add anyway).")
            return EXIT_STALE
        bits, crc = load_blocks(path)
        rep, status, written = receive(bits, crc, a.state_dir, a.out_dir)
        print(f"{path.name}: {describe(rep)}")
        for fid, st in status.items():
            line = f"File ID {fid}: {st['received']} of {st['total']} block(s)"
            if st.get("error"):
                line += f" - {st['error']}"
            elif st["missing"]:
                line += f", missing {ranges(st['missing'])}"
            print(line)
        for w in written:
            print(f"File complete, size and SHA-256 match: wrote {w}")
        return EXIT_DONE if written else EXIT_MISSING
    except (FileNotFoundError, ValueError, KeyError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
