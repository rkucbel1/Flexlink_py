#!/usr/bin/env python3
"""
Generate_File_Bitstream.py  (lives in Flexlink_py/Tx/Generate_FileData)

Step 3 for file transfer: instead of random bits, puts one packet's worth of a file into the
transport blocks. Reads Calculated.AvailableDataBlocks and Calculated.TransportBlockSize from
Tx/Parameters_Tx_payload.json and writes the same file, with the same dataset, as
Generate_TransportBlock_Bitstream.py (File.Bitstream_TransportBlks_Tx in
Tx/Generate_RandomPacketOfData), so steps 4 to 8 and the Rx compare run unchanged.

The file is cut into blocks as described in
Flexlink_BasicModules/File_Transfer/File_Framing.py: every block carries a 72-bit header
(file ID, block number, number of blocks), and the first blocks also carry the file name,
size and SHA-256. The file ID defaults to a value derived from the file name and contents,
so calling this script once per packet always gives the same ID.

Usage
-----
    python Tx/Generate_FileData/Generate_File_Bitstream.py --file photo.jpg --info
    python Tx/Generate_FileData/Generate_File_Bitstream.py --file photo.jpg --packet 1
    python Tx/Generate_FileData/Generate_File_Bitstream.py --file photo.jpg --blocks 7,12,40

Then run Tx/Run_Tx_Chain.py --keep-blocks, Rx/Run_Rx_Chain.py, and Rx/Receive_File.py.
App/Run_File_Transfer.py does all of that for every packet.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import BASIC_MODULES, TX  # noqa: E402

sys.path.insert(0, str(BASIC_MODULES / "File_Transfer"))
from File_Framing import FileSender  # noqa: E402

DEFAULT_JSON = TX / "Parameters_Tx_payload.json"
OUT_DIR = TX / "Generate_RandomPacketOfData"    # where step 4 and the Rx compare look
OUT_KEY = "Bitstream_TransportBlks_Tx"
DATASET = "TransportBlock_Bitstream"


def default_file_id(path: Path) -> int:
    h = hashlib.sha256(Path(path).name.encode("utf-8") + Path(path).read_bytes()).digest()
    return int.from_bytes(h[:2], "big")


def load_parameters(json_path: Path) -> tuple[int, int, Path]:
    """(AvailableDataBlocks, TransportBlockSize, output path) from the Tx json."""
    if not Path(json_path).is_file():
        raise FileNotFoundError(f"parameter file not found: {json_path}")
    params = json.loads(Path(json_path).read_text())
    try:
        n_blocks = int(params["Calculated"]["AvailableDataBlocks"])
        tb_size = int(params["Calculated"]["TransportBlockSize"])
    except KeyError as e:
        raise ValueError(f"missing key {e} in {json_path} - run Initialize_Tx.py") from None
    if n_blocks <= 0 or tb_size <= 0:
        raise ValueError("AvailableDataBlocks and TransportBlockSize must be > 0")
    name = (params.get("File") or {}).get(OUT_KEY)
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File.{OUT_KEY} not set in {json_path}")
    return n_blocks, tb_size, OUT_DIR / name.strip()


def make_sender(file: Path, json_path: Path = DEFAULT_JSON, file_id: int | None = None) -> FileSender:
    n_blocks, tb_size, _ = load_parameters(json_path)
    fid = default_file_id(file) if file_id is None else file_id
    return FileSender.from_file(file, tb_size, n_blocks, fid)


def write_packet(out_path: Path, sender: FileSender, blocks: list[int], packet: int | None = None,
                 packets: int | None = None) -> np.ndarray:
    """Write one packet's transport blocks in the format step 4 reads."""
    import h5py
    bits = sender.packet_bits(blocks)
    with h5py.File(out_path, "w") as f:
        d = f.create_dataset(DATASET, data=bits)
        d.attrs["AvailableDataBlocks"] = sender.blocks_per_packet
        d.attrs["TransportBlockSize"] = sender.tb_bits
        d.attrs["Length"] = bits.size
        d.attrs["Source"] = "File"
        d.attrs["FileName"] = sender.name
        d.attrs["FileSize"] = sender.size
        d.attrs["FileID"] = sender.file_id
        d.attrs["FileBlocks"] = sender.total
        d.attrs["BlockIndices"] = np.asarray(blocks, dtype=np.int64)
        if packet is not None:
            d.attrs["Packet"] = packet
        if packets is not None:
            d.attrs["Packets"] = packets
    return bits


def parse_blocks(text: str) -> list[int]:
    out: list[int] = []
    for part in text.replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Put one packet of a file into the Tx transport blocks.")
    ap.add_argument("--file", type=Path, required=True, help="file to send")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--packet", type=int, help="packet number, 1 .. packets (see --info)")
    g.add_argument("--blocks", help="resend these block numbers, e.g. 7,12,40-44 "
                                    "(one packet's worth, at most AvailableDataBlocks)")
    g.add_argument("--info", action="store_true", help="print how the file is split and stop")
    ap.add_argument("--file-id", type=int, help="0..65535 (default: from the file name and contents)")
    ap.add_argument("--json", type=Path, default=DEFAULT_JSON, help="Tx parameter file")
    ap.add_argument("--out", type=Path, help=f"output .h5 (default: File.{OUT_KEY} in {OUT_DIR})")
    a = ap.parse_args(argv)

    try:
        if not a.file.is_file():
            raise FileNotFoundError(f"file not found: {a.file}")
        sender = make_sender(a.file, a.json, a.file_id)
        plan = sender.packet_plan()
        print(f"{sender.name}: {sender.size} bytes, file ID {sender.file_id}, {sender.total} block(s) of "
              f"{sender.chunk} bytes, {len(plan)} packet(s) of {sender.blocks_per_packet} block(s) "
              f"(TransportBlockSize {sender.tb_bits})")
        if a.info:
            return 0
        if a.blocks:
            blocks = parse_blocks(a.blocks)
            if not 1 <= len(blocks) <= sender.blocks_per_packet:
                raise ValueError(f"--blocks takes 1 to {sender.blocks_per_packet} block numbers")
            blocks = sender.packet_plan(blocks)[0]
            packet = None
        else:
            packet = a.packet or 1
            if not 1 <= packet <= len(plan):
                raise ValueError(f"--packet must be 1 to {len(plan)}")
            blocks = plan[packet - 1]
        out = a.out or load_parameters(a.json)[2]
        bits = write_packet(out, sender, blocks, packet, len(plan))
        what = f"packet {packet} of {len(plan)}" if packet else "resend"
        print(f"{what}: blocks {', '.join(map(str, blocks))}")
        print(f"Wrote {bits.size} bits to {out}")
        print("Next: Tx/Run_Tx_Chain.py --keep-blocks, Rx/Run_Rx_Chain.py, Rx/Receive_File.py")
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
