#!/usr/bin/env python3
"""
Construct_Encoded_Control_Tx.py  (lives in Flexlink_BasicModules/Construct_Control_Field)

Encode the 54-bit ControlMessage into the scrambled bit sequence that fills the
control field (2200 BPSK data signals).

Steps (sizes for the current Parameters_Control_Field.json):
  1. Read Tx/ControlSignals/Parameters_Control_Field.json and the ControlMessage
     from File/Bitstream_ControlBlks_Tx (written by Initialize_Control_Field.py).
  2. Attach the CRC (CRC_Gen/GenerateCRC.py, Defined/CRC)    54 + 10       = 64  DataBlockSize
  3. Append TailBits zeros (encoder ends in state 0)          64 + 6        = 70  EncoderInputSize
  4. BCC encode (BCC_Encoder/Convolutionalencoder.py,
     ConstraintLength 7, Polynomials 133/171, no tail-biting) 70 x 2        = 140 CodeBlockSize
     The code block is Viterbi decoded straight back as a check.
  5. Interleave (Interleaver/Interleave_BCC.py)                              140
  6. Repeat: the code block plus Repetition copies           140 x (14 + 1) = 2100 CodeWordSize
     (Rate_Matching/Rate_Match.py)
  7. Append ZeroPadding zeros                                 2100 + 100    = 2200 NumberOfDataSigs
  8. Scramble the whole sequence with the Gold code for C_init_Dat (130, the
     same for every Flexlink control field), using Scrambler/Scrambler_GC.py.
     The Gold code is generated on every run and not kept on disk.
  9. Save to File/Bitstream_EncodedControl_Tx in Tx/ControlSignals.

Every size is checked against the Calculated section, so run
Initialize_Control_Field.py first after any change. Nothing is written to the JSON.

Usage:
    python Flexlink_BasicModules/Construct_Control_Field/Construct_Encoded_Control_Tx.py
    python .../Construct_Encoded_Control_Tx.py -v          # also log every stage's bits

The output .h5 file contains:
    data            (2200,) uint8   scrambled sequence (what goes to BPSK mapping)
    unscrambled     (2200,) uint8   the same sequence before scrambling
    DataBlock       (64,)   uint8   ControlMessage + CRC
    CodeBlock       (140,)  uint8   BCC output, before interleaving
    InterleavedBlock(140,)  uint8   after interleaving (repeated to make the sequence)
    attrs           NumberOfDataSigs, CodeBlockSize, Repetition, ZeroPadding, C_init_Dat,
                    CRC, SourceFile
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import (TX_CONTROL_SIGNALS, BCC_ENCODER, CRC_GEN, INTERLEAVER,  # noqa: E402
                            RATE_MATCHING, SCRAMBLER)

for _d in (BCC_ENCODER, CRC_GEN, INTERLEAVER, RATE_MATCHING, SCRAMBLER):
    if str(_d) not in sys.path:
        sys.path.insert(0, str(_d))

PARAMS_FILENAME = "Parameters_Control_Field.json"
INPUT_KEY = "Bitstream_ControlBlks_Tx"
INPUT_DATASET = "ControlMessage"
OUTPUT_KEY = "Bitstream_EncodedControl_Tx"

log = logging.getLogger("Construct_Encoded_Control_Tx")


def bits_str(b: np.ndarray) -> str:
    return "".join(map(str, np.asarray(b).astype(int)))


def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    try:
        params = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path.name}: line {exc.lineno}: {exc.msg}") from exc
    for section in ("Defined", "Calculated", "File"):
        if not isinstance(params.get(section), dict):
            raise ValueError(f"{path.name} has no '{section}' section - run Initialize_Control_Field.py")
    return params


def need(section: dict[str, Any], key: str, name: str) -> Any:
    if key not in section:
        raise ValueError(f"{name}/{key} not found in {PARAMS_FILENAME} - run Initialize_Control_Field.py")
    return section[key]


def file_path(params: dict[str, Any], key: str) -> Path:
    name = params["File"].get(key)
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File/{key} not found in {PARAMS_FILENAME}")
    return TX_CONTROL_SIGNALS / name.strip()


def load_message(path: Path, size: int) -> np.ndarray:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found - run Initialize_Control_Field.py first")
    with h5py.File(path, "r") as f:
        if INPUT_DATASET not in f:
            raise ValueError(f"{path.name} has no dataset '{INPUT_DATASET}'")
        bits = np.asarray(f[INPUT_DATASET][()]).ravel().astype(np.uint8)
    if bits.size != size:
        raise ValueError(f"{path.name} holds {bits.size} bits, ControlMessageSize is {size}")
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path.name} contains values other than 0 and 1")
    return bits


def check_size(stage: str, bits: np.ndarray, expected: int, key: str) -> None:
    if bits.size != expected:
        raise ValueError(f"{stage} gave {bits.size} bits but {key} = {expected} - "
                         "run Initialize_Control_Field.py")


def encode(message: np.ndarray, params: dict[str, Any]) -> dict[str, np.ndarray]:
    """Run steps 2 to 8 and return every stage's bits."""
    from GenerateCRC import GenerateCRC
    from Convolutionalencoder import ConvolutionalEncoder, ViterbiDecoder
    from Interleave_BCC import Interleave_BCC
    from Rate_Match import rate_match
    from Scrambler_GC import scrambler_gc

    d, c = params["Defined"], params["Calculated"]
    crc_name = need(d, "CRC", "Defined")
    K = need(d, "ConstraintLength", "Defined")
    polys = need(d, "Polynomials", "Defined")
    tail_biting = bool(need(d, "TailBiting", "Defined"))
    c_init = need(d, "C_init_Dat", "Defined")
    tail_bits = need(c, "TailBits", "Calculated")
    cbs = need(c, "CodeBlockSize", "Calculated")
    repetition = need(c, "Repetition", "Calculated")
    zero_pad = need(c, "ZeroPadding", "Calculated")
    n_data_sigs = need(c, "NumberOfDataSigs", "Calculated")

    # 2. CRC
    crc = np.asarray(GenerateCRC(message, crc_name)).ravel().astype(np.uint8)
    data_blk = np.concatenate([message, crc])
    check_size("Attach CRC", data_blk, need(c, "DataBlockSize", "Calculated"), "DataBlockSize")
    log.info("CRC %s: %d + %d = %d bits", crc_name, message.size, crc.size, data_blk.size)

    # 3. Tail bits
    enc_in = np.concatenate([data_blk, np.zeros(tail_bits, dtype=np.uint8)])
    check_size("Tail bits", enc_in, need(c, "EncoderInputSize", "Calculated"), "EncoderInputSize")

    # 4. BCC encode, then decode straight back as a check
    code_blk = np.asarray(ConvolutionalEncoder(enc_in, K, polys, tail_biting, "Hard")).ravel().astype(np.uint8)
    check_size("BCC encoder", code_blk, cbs, "CodeBlockSize")
    decoded = np.asarray(ViterbiDecoder(code_blk, K, polys, tail_biting, "Hard")).ravel()
    if not np.array_equal(decoded, enc_in):
        raise ValueError("Viterbi decoding the code block did not give back the encoder input")
    log.info("BCC K=%d %s: %d -> %d bits (decodes back correctly)", K, polys, enc_in.size, code_blk.size)

    # 5. Interleave
    inter = np.asarray(Interleave_BCC(code_blk, cbs)).ravel().astype(np.uint8)
    if inter.size != cbs or inter.sum() != code_blk.sum():
        raise ValueError("Interleave_BCC did not return a permutation of the code block")

    # 6. Repeat
    word = np.asarray(rate_match(inter, repetition)).ravel().astype(np.uint8)
    check_size("Repetition", word, need(c, "CodeWordSize", "Calculated"), "CodeWordSize")
    log.info("Interleaved and repeated: %d x (%d + 1) = %d bits", cbs, repetition, word.size)

    # 7. Zero padding
    seq = np.concatenate([word, np.zeros(zero_pad, dtype=np.uint8)])
    check_size("Zero padding", seq, n_data_sigs, "NumberOfDataSigs")
    log.info("len(Sequence) = %d = NumberOfDataSigs (incl. %d ZeroPadding)", seq.size, zero_pad)

    # 8. Scramble (scrambler_gc always writes its Gold code to an .h5 file; give it a
    #    temporary one so nothing is kept in Tx/ControlSignals)
    with tempfile.TemporaryDirectory() as tmp:
        scrambled = np.asarray(scrambler_gc(c_init, seq, h5_file=str(Path(tmp) / "GC.h5")))
    scrambled = scrambled.ravel().astype(np.uint8)
    check_size("Scrambler", scrambled, n_data_sigs, "NumberOfDataSigs")
    log.info("Scrambled %d bits with C_init_Dat = %d (%d bits changed)",
             scrambled.size, c_init, int(np.count_nonzero(scrambled != seq)))

    for name, b in (("ControlMessage", message), ("DataBlock", data_blk), ("CodeBlock", code_blk),
                    ("InterleavedBlock", inter)):
        log.debug("%-16s %s", name, bits_str(b))
    return {"data": scrambled, "unscrambled": seq, "DataBlock": data_blk,
            "CodeBlock": code_blk, "InterleavedBlock": inter}


def save(path: Path, stages: dict[str, np.ndarray], attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        for name, b in stages.items():
            f.create_dataset(name, data=b)
        for k, v in attrs.items():
            f.attrs[k] = v


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Encode the ControlMessage into the control field bit sequence.")
    ap.add_argument("--params", type=Path, default=TX_CONTROL_SIGNALS / PARAMS_FILENAME,
                    help=f"path to {PARAMS_FILENAME} (default: Tx/ControlSignals)")
    ap.add_argument("-v", "--verbose", action="store_true", help="also log every stage's bits")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")
    try:
        try:
            import h5py  # noqa: F401
        except ImportError:
            raise ValueError("Reading .h5 files needs h5py - install it with: pip install h5py") from None
        params = load_params(a.params)
        in_path, out_path = file_path(params, INPUT_KEY), file_path(params, OUTPUT_KEY)
        message = load_message(in_path, need(params["Defined"], "ControlMessageSize", "Defined"))
        log.info("Read ControlMessage (%d bits) from %s", message.size, in_path)
        stages = encode(message, params)
        c = params["Calculated"]
        save(out_path, stages, {
            "NumberOfDataSigs": stages["data"].size, "CodeBlockSize": c["CodeBlockSize"],
            "Repetition": c["Repetition"], "ZeroPadding": c["ZeroPadding"],
            "C_init_Dat": params["Defined"]["C_init_Dat"], "CRC": params["Defined"]["CRC"],
            "SourceFile": in_path.name})
        log.info("Encoded control field saved to %s", out_path)
    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Control field encoding failed")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
