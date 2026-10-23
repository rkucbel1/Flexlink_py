"""
Construct_Preamble.py

Builds the full Flexlink preamble:

    [ AGC burst | Preamble A | Preamble B ]

and saves it to Preamble.h5 (in this script's folder) as a single complex64
dataset. Parameters are read from <Flexlink_py>/Tx/Parameters_Tx_payload.json
via flexlink_paths.

HDF5 layout:
    /Preamble              complex64, shape (N,)
        attrs: SampleRate, AgcBurstLength, PreambleALength, PreambleBLength
               (part lengths in samples, in transmit order)

Usage:
    python Construct_Preamble.py [path/to/Parameters_Tx_payload.json] [output.h5]
"""

import sys
from pathlib import Path

import h5py
import numpy as np

# Importing the generators also puts the Flexlink_py root on sys.path
# (each one locates flexlink_paths.py), and gives the default JSON path.
from Generate_AGC_Burst import DEFAULT_PARAM_FILE, generate_agc_burst, load_sample_rate
from Generate_Preamble_A import generate_preamble_a
from Generate_Preamble_B import generate_preamble_b

# Preamble.h5 is written next to this script
DEFAULT_OUTPUT_FILE = Path(__file__).resolve().parent / "Preamble.h5"
DATASET_NAME = "Preamble"


def construct_preamble(sample_rate):
    """Return (preamble, parts) where parts is a dict of the individual pieces."""
    parts = {
        "AgcBurst": generate_agc_burst(sample_rate),
        "PreambleA": generate_preamble_a(sample_rate),
        "PreambleB": generate_preamble_b(sample_rate),
    }
    preamble = np.concatenate(list(parts.values())).astype(np.complex64)
    return preamble, parts


def save_preamble(preamble, parts, sample_rate, output_file=DEFAULT_OUTPUT_FILE):
    with h5py.File(output_file, "w") as f:
        dset = f.create_dataset(DATASET_NAME, data=preamble)
        dset.attrs["SampleRate"] = sample_rate
        dset.attrs["AgcBurstLength"] = len(parts["AgcBurst"])
        dset.attrs["PreambleALength"] = len(parts["PreambleA"])
        dset.attrs["PreambleBLength"] = len(parts["PreambleB"])


def main():
    param_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PARAM_FILE
    output_file = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUTPUT_FILE
    if not Path(param_file).is_file():
        sys.exit(f"Parameter file not found: {param_file}")

    sample_rate = load_sample_rate(param_file)
    preamble, parts = construct_preamble(sample_rate)
    save_preamble(preamble, parts, sample_rate, output_file)

    print(f"SampleRate : {sample_rate / 1e6:.0f} MHz")
    for name, p in parts.items():
        print(f"  {name:<10}: {len(p):6d} samples ({len(p) / sample_rate * 1e6:7.2f} us)")
    print(f"  {'Total':<10}: {len(preamble):6d} samples ({len(preamble) / sample_rate * 1e6:7.2f} us)")
    print(f"Saved '{DATASET_NAME}' ({preamble.dtype}) to {output_file}")


if __name__ == "__main__":
    main()
