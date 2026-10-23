#!/usr/bin/env python3
"""
Construct_Encoded_Bitstream_Tx.py - Encode a transport-block bitstream into the Tx data sequence.

Steps:
  1. Read Parameters_Tx_payload.json from the Tx directory (flexlink_paths.TX).
  2. Load the bitstream from the file named in File/Bitstream_TransportBlks_Tx
     (e.g. Bitstream_TransportBlks_Tx.h5, in Tx/Generate_RandomPacketOfData, written by
     Generate_RandomPacketOfData/Generate_TransportBlock_Bitstream.py).
  3. Split it into TransportBlockSize chunks. For each chunk, run the same steps
     as the Tx stage scripts in Tx/Encode/, using their own functions:
         Attach_CRC        -> TransportBlockSize + CRClen  (= DataBlockSize)
         FEC_LDPC_Tx       -> EncoderBlockSize per code block
         Interleaver_Tx    -> same length, permuted
         Rate_Matching_Tx  -> x (Repetition + 1)          (= CodeWordSize)
     and append the result to Sequence.
  4. Append ZeroPadding zeros to Sequence.
  5. Check len(Sequence) == NumberOfDataSigs.
  6. Scramble the whole Sequence (padding included) with scrambler_gc() from
     Scrambler/Scrambler_GC.py, using Defined/C_init_Dat as the Gold code id.
     The Gold code is generated on every run and not kept on disk.
  7. Save the scrambled Sequence to the file named in File/Bitstream_Encoded_Tx
     (e.g. Bitstream_Encoded_Tx.h5) in the Tx directory (or --out).

Nothing is written to the JSON or to the stage test-vector files.

Usage:
    python Construct_Encoded_Bitstream_Tx.py
    python Construct_Encoded_Bitstream_Tx.py --out My_Sequence.h5   # relative names go in Tx/
    python Construct_Encoded_Bitstream_Tx.py --input other.h5 --dataset name

The output .h5 file contains:
    data         (NumberOfDataSigs,) uint8   scrambled Sequence (encoded blocks + zero padding)
    unscrambled  (NumberOfDataSigs,) uint8   the same Sequence before scrambling
    attrs        NumberOfDataSigs, C_init_Dat, SourceFile
"""

from __future__ import annotations

import argparse
import importlib.util
import logging
import sys
import tempfile
from pathlib import Path
from types import ModuleType
from typing import Any, Callable

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX, CRC_GEN, LDPC_ENCODER, INTERLEAVER, SCRAMBLER  # noqa: E402

PARAMS_FILENAME = "Parameters_Tx_payload.json"
C_INIT_MAX = 255     # C_init_Dat is sent in the 8-bit control field C_init
INPUT_DIR = TX / "Generate_RandomPacketOfData"
INPUT_KEY = "Bitstream_TransportBlks_Tx"        # File/<key> = input name (in INPUT_DIR)
OUTPUT_KEY = "Bitstream_Encoded_Tx"             # File/<key> = output name (in the Tx directory)
DEFAULT_DATASET = "TransportBlock_Bitstream"
SCRAMBLER_SCRIPT = "Scrambler_GC.py"
ENCODE_DIR = TX / "Encode"     # Attach_CRC, FEC_LDPC_Tx, Interleaver_Tx, Rate_Matching_Tx

log = logging.getLogger("Construct_Encoded_Bitstream_Tx")


# --------------------------------------------------------------------------- #
# Tx scripts, loaded by file path so their helper functions can be reused.
# The four stage scripts live in Tx/Encode/.
# --------------------------------------------------------------------------- #
def load_tx_module(name: str, folder: Path = ENCODE_DIR) -> ModuleType:
    path = folder / f"{name}.py"
    if not path.is_file():
        raise FileNotFoundError(f"{path.name} not found in {folder}")
    spec = importlib.util.spec_from_file_location(f"_tx_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Encoder:
    """The four Tx stages, set up once from the JSON and applied per transport block."""

    def __init__(self, params: dict[str, Any]):
        crc_mod = load_tx_module("Attach_CRC")
        ldpc_mod = load_tx_module("FEC_LDPC_Tx")
        il_mod = load_tx_module("Interleaver_Tx")
        rm_mod = load_tx_module("Rate_Matching_Tx")

        # CRC
        self.crc_gen = crc_mod.crc_gen_name(params, None)
        self._generate_crc = crc_mod.load_generate_crc(CRC_GEN)
        self._attach_crc = crc_mod.attach_crc

        # LDPC
        self.n = ldpc_mod.get_positive_int(params, "EncoderBlockSize")
        rate_str, rate = ldpc_mod.get_code_rate(params)
        k_frac = self.n * rate
        if k_frac.denominator != 1:
            raise ValueError(f"EncoderBlockSize {self.n} x EncoderCodeRate {rate_str} "
                             "is not a whole number of bits")
        self.k = int(k_frac)
        get_g, self._ldpc_encode = ldpc_mod.load_encoder(LDPC_ENCODER)
        self.g_type = ldpc_mod.g_type(self.n, rate_str)
        self.G = ldpc_mod.load_g(get_g, self.g_type)
        if self.G.shape != (self.k, self.n):
            raise ValueError(f"LDPC_Get_G('{self.g_type}') returned G of shape {self.G.shape}, "
                             f"expected ({self.k}, {self.n})")

        # Interleaver
        self.cbs = il_mod.get_block_size(params)
        if self.cbs != self.n:
            raise ValueError(f"Interleaver block size {self.cbs} != EncoderBlockSize {self.n}")
        self._interleave = il_mod.load_interleaver(INTERLEAVER)

        # Rate matching
        self.repetition = rm_mod.get_repetition(params)
        self._rate_match = rm_mod.load_rate_match(rm_mod.rate_match_dirs(None))

        log.info("CRC %s | LDPC %s (K=%d, N=%d) | interleaver %d | repetition %d",
                 self.crc_gen, self.g_type, self.k, self.n, self.cbs, self.repetition)

    def __call__(self, tb: np.ndarray) -> np.ndarray:
        # 1. Attach_CRC
        data_blk, _ = self._attach_crc(tb, self.crc_gen, self._generate_crc)
        if data_blk.size % self.k:
            raise ValueError(f"Data block of {data_blk.size} bits is not a multiple of K = {self.k}")
        m = data_blk.size // self.k

        # 2. FEC_LDPC_Tx
        code = np.asarray(self._ldpc_encode(data_blk.reshape(m, self.k), self.G),
                          dtype=np.uint8).reshape(m, self.n)
        if not np.array_equal(code[:, :self.k], data_blk.reshape(m, self.k)):
            log.warning("Codewords do not start with the message bits - is G systematic?")

        # 3. Interleaver_Tx
        inv = np.asarray(self._interleave(code, self.cbs)).astype(np.uint8)
        if inv.shape != (m, self.cbs) or not np.array_equal(inv.sum(axis=1), code.sum(axis=1)):
            raise ValueError("Interleave_LDPC did not return a permutation of each code block")

        # 4. Rate_Matching_Tx
        inv = inv.ravel()
        word = np.asarray(self._rate_match(inv, self.repetition)).ravel().astype(np.uint8)
        if word.size != inv.size * (self.repetition + 1):
            raise ValueError(f"rate_match returned {word.size} bits, expected "
                             f"{inv.size * (self.repetition + 1)}")
        return word


# --------------------------------------------------------------------------- #
# Scrambler
# --------------------------------------------------------------------------- #
def load_scrambler() -> Callable:
    """scrambler_gc() from Scrambler/Scrambler_GC.py."""
    script = SCRAMBLER / SCRAMBLER_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(f"{SCRAMBLER_SCRIPT} not found in {SCRAMBLER}")
    # Scrambler_GC imports Scrambler_Gold_Code_Generator from its own folder
    if str(SCRAMBLER) not in sys.path:
        sys.path.insert(0, str(SCRAMBLER))
    spec = importlib.util.spec_from_file_location("Scrambler_GC", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    func = getattr(module, "scrambler_gc", None)
    if not callable(func):
        raise ValueError(f"{script} has no scrambler_gc function")
    return func


def scramble(sequence: np.ndarray, c_init: int) -> np.ndarray:
    scrambler_gc = load_scrambler()
    # scrambler_gc always writes its Gold code to an .h5 file; give it a
    # temporary one that is deleted straight away, so nothing is kept in Tx/.
    with tempfile.TemporaryDirectory() as tmp:
        scrambled = np.asarray(scrambler_gc(c_init, sequence,
                                            h5_file=str(Path(tmp) / "GC_Scrambler_Sequence.h5")))
    scrambled = scrambled.ravel().astype(np.uint8)
    if scrambled.size != sequence.size:
        raise ValueError(f"scrambler_gc returned {scrambled.size} bits, expected {sequence.size}")
    return scrambled


# --------------------------------------------------------------------------- #
# Input / output
# --------------------------------------------------------------------------- #
def load_bitstream(path: Path, dataset: str | None) -> np.ndarray:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Input file not found: {path} - "
                                "run Generate_TransportBlock_Bitstream.py first")
    with h5py.File(path, "r") as f:
        if dataset is None:
            dataset = DEFAULT_DATASET if DEFAULT_DATASET in f else None
        if dataset is None:
            names = [n for n in f if isinstance(f[n], h5py.Dataset)]
            if len(names) != 1:
                raise ValueError(f"{path.name} has datasets {names} - pass --dataset")
            dataset = names[0]
        if dataset not in f:
            raise ValueError(f"{path.name} has no dataset '{dataset}'")
        bits = np.asarray(f[dataset][()]).ravel().astype(np.uint8)
    if bits.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path} contains values other than 0 and 1")
    return bits


def save_sequence(path: Path, scrambled: np.ndarray, unscrambled: np.ndarray,
                  attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=scrambled)
        f.create_dataset("unscrambled", data=unscrambled)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def encode(input_path: Path, dataset: str | None = None,
           params_file: Path | None = None) -> tuple[np.ndarray, np.ndarray, int]:
    """Return (ScrambledSequence, Sequence, C_init_Dat).

    Sequence is every encoded transport block followed by ZeroPadding zeros;
    ScrambledSequence is Sequence scrambled with the Gold code for C_init_Dat.
    """
    params_mod = load_tx_module("FEC_LDPC_Tx")       # shared JSON helpers
    params_file = params_file or TX / PARAMS_FILENAME
    params = params_mod.load_params(params_file)

    tb_size = params_mod.get_positive_int(params, "TransportBlockSize")
    n_data_sigs = params_mod.get_positive_int(params, "NumberOfDataSigs")
    _, zero_padding = params_mod._find_key(params, "ZeroPadding")
    if isinstance(zero_padding, bool) or not isinstance(zero_padding, int) or zero_padding < 0:
        raise ValueError(f"ZeroPadding is {zero_padding!r} in {PARAMS_FILENAME}, "
                         "expected a non-negative integer")

    bitstream = load_bitstream(input_path, dataset)
    if bitstream.size % tb_size:
        raise ValueError(f"{input_path.name} has {bitstream.size} bits, not a multiple of "
                         f"TransportBlockSize {tb_size} ({bitstream.size % tb_size} left over)")
    n_blocks = bitstream.size // tb_size
    log.info("Read %d bits from %s -> %d transport block(s) of %d bits",
             bitstream.size, input_path, n_blocks, tb_size)

    encoder = Encoder(params)

    Sequence = np.zeros(0, dtype=np.uint8)
    for i in range(n_blocks):
        word = encoder(bitstream[i * tb_size:(i + 1) * tb_size])
        Sequence = np.concatenate([Sequence, word])
        log.debug("Block %d: %d -> %d bits", i, tb_size, word.size)
    log.info("Encoded %d block(s) -> %d bits", n_blocks, Sequence.size)

    Sequence = np.concatenate([Sequence, np.zeros(zero_padding, dtype=np.uint8)])

    if Sequence.size != n_data_sigs:
        raise ValueError(f"len(Sequence) = {Sequence.size} ({Sequence.size - zero_padding} encoded "
                         f"+ {zero_padding} ZeroPadding) but NumberOfDataSigs = {n_data_sigs}")
    log.info("len(Sequence) = %d = NumberOfDataSigs  (incl. %d ZeroPadding)",
             Sequence.size, zero_padding)

    _, scrambled_size = params_mod._find_key(params, "ScrambledSequenceSize")
    if isinstance(scrambled_size, int) and scrambled_size != Sequence.size:
        raise ValueError(f"ScrambledSequenceSize = {scrambled_size} but len(Sequence) = {Sequence.size}")

    parent, c_init = params_mod._find_key(params, "C_init_Dat")
    if parent is None:
        raise ValueError(f"C_init_Dat not found in {PARAMS_FILENAME}")
    if isinstance(c_init, bool) or not isinstance(c_init, int) or not 0 <= c_init <= C_INIT_MAX:
        raise ValueError(f"C_init_Dat is {c_init!r} in {PARAMS_FILENAME}, expected 0 to {C_INIT_MAX} "
                         "(it is sent in the 8-bit control field C_init)")
    ScrambledSequence = scramble(Sequence, c_init)
    log.info("Scrambled %d bits with C_init_Dat = %d (%d bits changed)",
             ScrambledSequence.size, c_init, int(np.count_nonzero(ScrambledSequence != Sequence)))
    return ScrambledSequence, Sequence, c_init


def file_name(files: Any, key: str) -> str:
    """File name stored under File/<key> in the JSON."""
    name = files.get(key) if isinstance(files, dict) else None
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File/{key} not found in {PARAMS_FILENAME}")
    return name.strip()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Encode a transport-block bitstream into the Tx data sequence.")
    ap.add_argument("--input", type=Path,
                    help=f"bitstream .h5 (default: File/{INPUT_KEY} in {INPUT_DIR})")
    ap.add_argument("--dataset", help=f"dataset in the input file (default: {DEFAULT_DATASET})")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Tx directory)")
    ap.add_argument("--out", type=Path,
                    help=f"output .h5 file; a relative name is put in the Tx directory "
                         f"(default: File/{OUTPUT_KEY})")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading .h5 files needs h5py - install it with: pip install h5py")

        files = load_tx_module("FEC_LDPC_Tx").load_params(args.params or TX / PARAMS_FILENAME).get("File", {})
        in_path = args.input or INPUT_DIR / file_name(files, INPUT_KEY)
        out_name = args.out or Path(file_name(files, OUTPUT_KEY))
        out_path = out_name if out_name.is_absolute() else TX / out_name

        ScrambledSequence, Sequence, c_init = encode(in_path, args.dataset, args.params)

        save_sequence(out_path, ScrambledSequence, Sequence,
                      {"NumberOfDataSigs": Sequence.size, "C_init_Dat": c_init,
                       "SourceFile": in_path.name})
        log.info("Scrambled Sequence saved to %s", out_path)
    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Encoding failed")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
