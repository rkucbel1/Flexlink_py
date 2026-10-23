#!/usr/bin/env python3
"""
File_Framing.py  (lives in Flexlink_py/Flexlink_BasicModules/File_Transfer)

Carries an arbitrary file in the payload transport blocks and puts it back together at
the receiver. Plain NumPy and the standard library (no h5py), so it can be tested on its own.

Byte stream
-----------
The file is wrapped in a small manifest before it is cut into blocks:

    "FLXF"  4 bytes   magic
    version 1 byte    (1)
    size    8 bytes   file size in bytes, big endian
    sha256  32 bytes  of the file
    namelen 2 bytes   big endian
    name    namelen   file name (UTF-8, no folders)
    data    size      the file

Transport blocks
----------------
Every transport block (TransportBlockSize bits, the CRC is added later by the Tx chain) is

    magic    8 bits   0xF1
    file_id 16 bits   tells files apart
    index   24 bits   block number in the file, 0 .. total-1
    total   24 bits   blocks in the file
    chunk   8 * ChunkBytes bits of the byte stream, ChunkBytes = (TransportBlockSize - 72) // 8
    zeros   the rest (TransportBlockSize - 72) % 8 bits

All fields MSB first. Every block says where it belongs, so blocks can arrive in any order,
more than once, or in different packets: the receiver keeps every block whose CRC passed and
whose header makes sense, and the file is done when it has all `total` of them. A packet
holds AvailableDataBlocks blocks; the last packet of a round is filled up with copies of the
first blocks of that round (free extra copies, ignored if already received).

At the end the receiver checks the size and the SHA-256 from the manifest, so a block that
passed its CRC by chance cannot slip a wrong file through.
"""
from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

STREAM_MAGIC = b"FLXF"
STREAM_VERSION = 1
BLOCK_MAGIC = 0xF1
HEADER_BITS = 72          # 8 + 16 + 24 + 24
MAX_BLOCKS = (1 << 24) - 1
_MANIFEST_FIXED = struct.Struct(">4sBQ32sH")   # magic, version, size, sha256, name length


# --------------------------------------------------------------------------- #
# Bits <-> bytes
# --------------------------------------------------------------------------- #
def bytes_to_bits(data: bytes) -> np.ndarray:
    return np.unpackbits(np.frombuffer(data, dtype=np.uint8))


def bits_to_bytes(bits: np.ndarray) -> bytes:
    bits = np.asarray(bits, dtype=np.uint8).ravel()
    if bits.size % 8:
        raise ValueError(f"{bits.size} bits is not a whole number of bytes")
    return np.packbits(bits).tobytes()


def chunk_bytes(tb_bits: int) -> int:
    """File bytes one transport block of tb_bits carries."""
    n = (int(tb_bits) - HEADER_BITS) // 8
    if n < 1:
        raise ValueError(f"TransportBlockSize {tb_bits} is too small for the {HEADER_BITS}-bit block header")
    return n


# --------------------------------------------------------------------------- #
# Manifest
# --------------------------------------------------------------------------- #
def build_stream(data: bytes, name: str) -> bytes:
    """Manifest + file bytes."""
    nm = Path(name).name.encode("utf-8")[:1024]
    head = _MANIFEST_FIXED.pack(STREAM_MAGIC, STREAM_VERSION, len(data),
                                hashlib.sha256(data).digest(), len(nm))
    return head + nm + data


@dataclass
class Manifest:
    name: str
    size: int
    sha256: bytes
    offset: int           # where the file data starts in the stream


def parse_manifest(stream: bytes) -> Manifest:
    if len(stream) < _MANIFEST_FIXED.size:
        raise ValueError("stream shorter than the manifest")
    magic, ver, size, sha, nlen = _MANIFEST_FIXED.unpack_from(stream)
    if magic != STREAM_MAGIC:
        raise ValueError(f"no file manifest (magic {magic!r})")
    if ver != STREAM_VERSION:
        raise ValueError(f"manifest version {ver} not supported")
    off = _MANIFEST_FIXED.size
    name = stream[off:off + nlen].decode("utf-8", errors="replace")
    name = Path(name.replace("\\", "/")).name or "received_file"
    return Manifest(name=name, size=size, sha256=sha, offset=off + nlen)


# --------------------------------------------------------------------------- #
# Block header
# --------------------------------------------------------------------------- #
def pack_header(file_id: int, index: int, total: int) -> np.ndarray:
    word = (BLOCK_MAGIC << 64) | ((file_id & 0xFFFF) << 48) | ((index & 0xFFFFFF) << 24) | (total & 0xFFFFFF)
    return bytes_to_bits(word.to_bytes(9, "big"))


def unpack_header(bits: np.ndarray) -> tuple[int, int, int, int]:
    """(magic, file_id, index, total) from the first 72 bits of a block."""
    word = int.from_bytes(bits_to_bytes(bits[:HEADER_BITS]), "big")
    return word >> 64, (word >> 48) & 0xFFFF, (word >> 24) & 0xFFFFFF, word & 0xFFFFFF


# --------------------------------------------------------------------------- #
# Transmitter
# --------------------------------------------------------------------------- #
class FileSender:
    """Cuts one file into transport blocks and groups them into packets."""

    def __init__(self, data: bytes, name: str, tb_bits: int, blocks_per_packet: int, file_id: int):
        if not 0 <= file_id <= 0xFFFF:
            raise ValueError("file_id must be 0 to 65535")
        if blocks_per_packet < 1:
            raise ValueError("AvailableDataBlocks must be at least 1")
        self.name = Path(name).name
        self.size = len(data)
        self.tb_bits = int(tb_bits)
        self.blocks_per_packet = int(blocks_per_packet)
        self.file_id = int(file_id)
        self.chunk = chunk_bytes(self.tb_bits)
        stream = build_stream(data, self.name)
        self.total = -(-len(stream) // self.chunk)
        if self.total > MAX_BLOCKS:
            raise ValueError(f"file needs {self.total} blocks, at most {MAX_BLOCKS} fit the header")
        stream += bytes(self.total * self.chunk - len(stream))
        self._stream = np.frombuffer(stream, dtype=np.uint8).reshape(self.total, self.chunk)

    @classmethod
    def from_file(cls, path: Path, tb_bits: int, blocks_per_packet: int, file_id: int) -> "FileSender":
        path = Path(path)
        return cls(path.read_bytes(), path.name, tb_bits, blocks_per_packet, file_id)

    def block(self, index: int) -> np.ndarray:
        if not 0 <= index < self.total:
            raise IndexError(f"block {index} outside 0..{self.total - 1}")
        bits = np.zeros(self.tb_bits, dtype=np.uint8)
        bits[:HEADER_BITS] = pack_header(self.file_id, index, self.total)
        bits[HEADER_BITS:HEADER_BITS + 8 * self.chunk] = np.unpackbits(self._stream[index])
        return bits

    def packet_plan(self, indices=None) -> list[list[int]]:
        """Block numbers in each packet. The last packet is filled with copies of the first blocks."""
        idx = list(range(self.total)) if indices is None else [int(i) for i in indices]
        if not idx:
            return []
        n = self.blocks_per_packet
        plan = [idx[k:k + n] for k in range(0, len(idx), n)]
        last = plan[-1]
        k = 0
        while len(last) < n:
            last.append(idx[k % len(idx)])
            k += 1
        return plan

    def packet_bits(self, block_indices: list[int]) -> np.ndarray:
        if len(block_indices) != self.blocks_per_packet:
            raise ValueError(f"a packet holds {self.blocks_per_packet} blocks, got {len(block_indices)}")
        return np.concatenate([self.block(i) for i in block_indices])

    def packets(self, indices=None):
        """(block numbers, bits) for each packet."""
        for blk in self.packet_plan(indices):
            yield blk, self.packet_bits(blk)


# --------------------------------------------------------------------------- #
# Receiver
# --------------------------------------------------------------------------- #
@dataclass
class PacketReport:
    blocks: int = 0            # blocks in the packet
    crc_fail: int = 0          # dropped: CRC failed
    foreign: int = 0           # dropped: CRC passed but not a block of a file transfer
    rejected: int = 0          # dropped: header did not fit the file already being received
    new: int = 0               # blocks the file did not have yet
    duplicate: int = 0         # already had them
    file_ids: tuple = ()


class FileReceiver:
    """Collects blocks of one or more files; state is kept in state_dir between runs."""

    def __init__(self, state_dir: Path):
        self.dir = Path(state_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._files: dict[int, dict] = {}

    # ---- state on disk, written in place so a packet only costs its own blocks ----
    #   FileRx_<id>.chunks.npy  (total, chunk_bytes) uint8, memory-mapped
    #   FileRx_<id>.have.npy    (total,) bool, memory-mapped
    #   FileRx_<id>.json        total, chunk_bytes, packets, done, ...
    def _paths(self, fid: int) -> tuple[Path, Path, Path]:
        base = f"FileRx_{fid:05d}"
        return self.dir / f"{base}.chunks.npy", self.dir / f"{base}.have.npy", self.dir / f"{base}.json"

    def _load(self, fid: int) -> dict | None:
        if fid in self._files:
            return self._files[fid]
        chunks, have, js = self._paths(fid)
        if not (chunks.is_file() and have.is_file() and js.is_file()):
            return None
        meta = json.loads(js.read_text())
        st = {"chunks": np.load(chunks, mmap_mode="r+"), "have": np.load(have, mmap_mode="r+"), **meta}
        self._files[fid] = st
        return st

    def _save(self, fid: int) -> None:
        st = self._files[fid]
        st["chunks"].flush()
        st["have"].flush()
        js = self._paths(fid)[2]
        meta = {k: v for k, v in st.items() if k not in ("chunks", "have")}
        tmp = js.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(meta, indent=2))
        tmp.replace(js)

    def _new(self, fid: int, total: int, chunk: int) -> dict:
        self._drop(fid)
        chunks, have, _ = self._paths(fid)
        st = {"file_id": fid, "total": total, "chunk_bytes": chunk,
              "chunks": np.lib.format.open_memmap(chunks, mode="w+", dtype=np.uint8, shape=(total, chunk)),
              "have": np.lib.format.open_memmap(have, mode="w+", dtype=bool, shape=(total,)),
              "packets": 0, "done": None}
        self._files[fid] = st
        return st

    def _drop(self, fid: int) -> None:
        st = self._files.pop(fid, None)
        if st is not None:
            del st["chunks"], st["have"]        # close the memory maps before deleting the files
        base = f"FileRx_{fid:05d}"
        for p in (*self._paths(fid), self.dir / f"{base}.npz"):   # .npz: the first, slower format
            p.unlink(missing_ok=True)

    def file_ids(self) -> list[int]:
        ids = {int(p.name.split("_")[1].split(".")[0]) for p in self.dir.glob("FileRx_*.json")}
        ids |= {int(p.name.split("_")[1].split(".")[0]) for p in self.dir.glob("FileRx_*.npz")}
        return sorted(ids | set(self._files))

    def reset(self, fid: int | None = None) -> None:
        for f in (self.file_ids() if fid is None else [fid]):
            self._drop(f)

    # ---- receiving ----
    def add_packet(self, bits: np.ndarray, crc_pass: np.ndarray) -> PacketReport:
        """Feed one decoded packet: TransportBlock_Bitstream and CRC_Pass from Get_Decoded_Bitstream.py."""
        bits = np.asarray(bits, dtype=np.uint8).ravel()
        crc_pass = np.asarray(crc_pass).ravel().astype(bool)
        n = crc_pass.size
        if n == 0 or bits.size % n:
            raise ValueError(f"{bits.size} bits cannot be split into {n} blocks")
        tb_bits = bits.size // n
        rep = PacketReport(blocks=n)
        if tb_bits <= HEADER_BITS:
            rep.foreign = n
            return rep
        chunk = chunk_bytes(tb_bits)
        touched = set()
        for k, blk in enumerate(bits.reshape(n, tb_bits)):
            if not crc_pass[k]:
                rep.crc_fail += 1
                continue
            magic, fid, idx, total = unpack_header(blk)
            if magic != BLOCK_MAGIC or total == 0 or idx >= total:
                rep.foreign += 1
                continue
            st = self._load(fid)
            if st is None or st["done"]:
                if st is not None and st["done"] and (st["total"] != total or st["chunk_bytes"] != chunk):
                    st = None      # a finished file with this id: a new file reuses the id
                if st is None:
                    st = self._new(fid, total, chunk)
            if st["total"] != total or st["chunk_bytes"] != chunk:
                rep.rejected += 1
                continue
            touched.add(fid)
            if st["have"][idx]:
                rep.duplicate += 1
                continue
            st["chunks"][idx] = np.packbits(blk[HEADER_BITS:HEADER_BITS + 8 * chunk])
            st["have"][idx] = True
            rep.new += 1
        for fid in touched:
            self._files[fid]["packets"] += 1
            self._save(fid)
        rep.file_ids = tuple(sorted(touched))
        return rep

    def status(self, fid: int) -> dict:
        st = self._load(fid)
        if st is None:
            raise KeyError(f"no blocks of file {fid} received")
        have = st["have"]
        return {"file_id": fid, "total": int(st["total"]), "received": int(have.sum()),
                "missing": [int(i) for i in np.flatnonzero(~have)], "complete": bool(have.all()),
                "packets": int(st["packets"]), "done": st["done"]}

    def counts(self, fid: int) -> tuple[int, int]:
        """(received, total) without listing the missing blocks."""
        st = self._load(fid)
        if st is None:
            raise KeyError(f"no blocks of file {fid} received")
        return int(np.count_nonzero(st["have"])), int(st["total"])

    def missing(self, fid: int) -> list[int]:
        return self.status(fid)["missing"]

    def assemble(self, fid: int, out_dir: Path, overwrite: bool = False) -> Path:
        """Write the file once every block is in; checks size and SHA-256. Returns its path."""
        st = self._load(fid)
        if st is None:
            raise KeyError(f"no blocks of file {fid} received")
        if not st["have"].all():
            raise ValueError(f"file {fid}: {int((~st['have']).sum())} of {st['total']} blocks still missing")
        stream = st["chunks"].tobytes()
        man = parse_manifest(stream)
        data = stream[man.offset:man.offset + man.size]
        if len(data) != man.size:
            raise ValueError(f"file {fid}: manifest says {man.size} bytes, stream holds {len(data)}")
        if hashlib.sha256(data).digest() != man.sha256:
            raise ValueError(f"file {fid}: SHA-256 does not match - a corrupted block passed its CRC")
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / man.name
        if out.exists() and not overwrite:
            stem, suf, k = out.stem, out.suffix, 1
            while out.exists():
                out = out_dir / f"{stem}_{k}{suf}"
                k += 1
        out.write_bytes(data)
        st["done"] = str(out)
        st["name"], st["size"], st["sha256"] = man.name, man.size, man.sha256.hex()
        self._save(fid)
        return out
