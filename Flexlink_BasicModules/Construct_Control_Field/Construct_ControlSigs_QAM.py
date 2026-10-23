#!/usr/bin/env python3
"""
Construct_ControlSigs_QAM.py  (lives in Flexlink_BasicModules/Construct_Control_Field)

Map the encoded control field bits to BPSK values. The control field version of
Tx/Construct_Datasigs_QAM.py.

Steps:
  1. Read Tx/ControlSignals/Parameters_Control_Field.json.
  2. Read the scrambled bits ('data') from File/Bitstream_EncodedControl_Tx
     (written by Construct_Encoded_Control_Tx.py) and check there are
     NumberOfDataSigs of them (2200).
  3. Map them with qam_map() from QAM_Mapper/QAM_Mapper.py using Defined/Mod
     (BPSK: bit 0 -> -1, bit 1 -> +1, IEEE 802.11 mapping as for the payload).
  4. Save to File/Sequence_ControlSigs_QAM in Tx/ControlSignals.

Usage:
    python Flexlink_BasicModules/Construct_Control_Field/Construct_ControlSigs_QAM.py

The output .h5 file contains:
    data    (2200,) complex128   BPSK values, in the order they fill the grid
    attrs   Mod, BitsPerSymbol, NumberOfDataSigs, SourceFile
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX_CONTROL_SIGNALS, QAM_MAPPER  # noqa: E402

if str(QAM_MAPPER) not in sys.path:
    sys.path.insert(0, str(QAM_MAPPER))

PARAMS_FILENAME = "Parameters_Control_Field.json"
INPUT_KEY = "Bitstream_EncodedControl_Tx"
OUTPUT_KEY = "Sequence_ControlSigs_QAM"
MOD_BITS = {"BPSK": 1}

log = logging.getLogger("Construct_ControlSigs_QAM")


def file_path(params: dict, key: str) -> Path:
    name = params.get("File", {}).get(key)
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File/{key} not found in {PARAMS_FILENAME}")
    return TX_CONTROL_SIGNALS / name.strip()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Map the encoded control field bits to BPSK.")
    ap.add_argument("--params", type=Path, default=TX_CONTROL_SIGNALS / PARAMS_FILENAME)
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")
    try:
        try:
            import h5py
        except ImportError:
            raise ValueError("Reading .h5 files needs h5py - install it with: pip install h5py") from None
        from QAM_Mapper import qam_map

        params = json.loads(Path(a.params).read_text(encoding="utf-8"))
        mod = params["Defined"]["Mod"]
        if mod not in MOD_BITS:
            raise ValueError(f"Defined/Mod is {mod!r}; the control field is {' or '.join(MOD_BITS)}")
        n_data_sigs = params["Calculated"]["NumberOfDataSigs"]
        in_path, out_path = file_path(params, INPUT_KEY), file_path(params, OUTPUT_KEY)

        if not in_path.is_file():
            raise FileNotFoundError(f"{in_path} not found - run Construct_Encoded_Control_Tx.py first")
        with h5py.File(in_path, "r") as f:
            bits = np.asarray(f["data"][()]).ravel().astype(np.uint8)
        if bits.size != n_data_sigs:
            raise ValueError(f"{in_path.name} has {bits.size} bits but NumberOfDataSigs is {n_data_sigs} "
                             "- rerun Initialize_Control_Field.py and Construct_Encoded_Control_Tx.py")
        log.info("Read %d bits from %s", bits.size, in_path)

        symbols = np.asarray(qam_map(bits, mod), dtype=np.complex128).ravel()
        if symbols.size != n_data_sigs // MOD_BITS[mod]:
            raise ValueError(f"qam_map returned {symbols.size} values, expected {n_data_sigs // MOD_BITS[mod]}")
        log.info("%s: %d bits -> %d symbols, first 8: %s", mod, bits.size, symbols.size,
                 " ".join(f"{s.real:+.0f}" for s in symbols[:8]))

        with h5py.File(out_path, "w") as f:
            f.create_dataset("data", data=symbols)
            f.attrs["Mod"] = mod
            f.attrs["BitsPerSymbol"] = MOD_BITS[mod]
            f.attrs["NumberOfDataSigs"] = n_data_sigs
            f.attrs["SourceFile"] = in_path.name
        log.info("ControlSigs saved to %s", out_path)
    except (FileNotFoundError, ValueError, KeyError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Control field mapping failed")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
