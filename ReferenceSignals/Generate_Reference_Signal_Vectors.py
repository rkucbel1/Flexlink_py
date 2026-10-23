"""
Generate_Reference_Signal_Vectors.py

Builds the Ref_P0 and Ref_P1 reference-signal vectors for every combination of
  - ReferenceSymbolProfile X   : 3, 6, 12, 24
  - QAM option (bits/symbol) Y : 1 (BPSK), 2 (QPSK), 4 (16QAM), 6 (64QAM)
-> 16 Ref_P0 + 16 Ref_P1 vectors, each written to Ref_Pz_X_Y.h5 in the folder
   this script is in.

Steps
  1. Vector_N        : alternating 1, 0, 1, 0, ... of length N = 840 / X
                       (280, 140, 70, 35 = number of Ref_P0 / Ref_P1 signals in
                       one Symbol_Ref, from Construct_Resource_Grid.py)
  2. Vector_N_P0/P1  : Vector_N scrambled by Scrambler_GC.scrambler_gc with
                       C_init_ID1 (P0) or C_init_ID2 (P1)
  3. Ref_Pz_X_Y      : Vector_N_Pz mapped to constellation values
                         BPSK          : 0 -> +1, 1 -> -1 (3GPP polarity, 1 - 2b)
                         QPSK/16/64QAM : 1 -> lower right corner, 0 -> upper left
                                         corner, power normalized
"""

import json
import os
import sys
import tempfile

import h5py
import numpy as np

# Make Construct_Grid and Scrambler importable from anywhere in Flexlink_py
try:
    from flexlink_paths import CONSTRUCT_GRID, SCRAMBLER, TX
except ImportError:  # flexlink package not installed -> locate folders relative to this file
    from pathlib import Path
    _root = next(p for p in Path(__file__).resolve().parents if (p / "Flexlink_BasicModules").is_dir())
    CONSTRUCT_GRID = _root / "Flexlink_BasicModules" / "Construct_Grid"
    SCRAMBLER = _root / "Flexlink_BasicModules" / "Scrambler"
    TX = _root / "Tx"
for _folder in (CONSTRUCT_GRID, SCRAMBLER):
    if str(_folder) not in sys.path:
        sys.path.insert(0, str(_folder))

from Construct_Resource_Grid import REF_SYMBOL_LAYOUTS, VALID_REF_PROFILES, SignalType
from Scrambler_GC import scrambler_gc

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))   # .../ReferenceSignals

# Gold code initial values for the scrambler, read from Tx/Parameters_Tx_payload.json
PARAMETERS_FILE = os.path.join(TX, "Parameters_Tx_payload.json")


def read_c_init_ids(parameters_file=PARAMETERS_FILE):
    """Return (C_init_ID1, C_init_ID2) from the "Defined" section of the parameter file."""
    with open(parameters_file, "r") as f:
        defined = json.load(f)["Defined"]
    try:
        return int(defined["C_init_ID1"]), int(defined["C_init_ID2"])
    except KeyError as e:
        raise KeyError(f"{e.args[0]} not found in 'Defined' section of {parameters_file}") from None


C_init_ID1, C_init_ID2 = read_c_init_ids()   # ID1 -> Ref_P0 vectors, ID2 -> Ref_P1 vectors

# Y value -> modulation name
QAM_OPTIONS = {1: "BPSK", 2: "QPSK", 4: "16QAM", 6: "64QAM"}

# Largest amplitude per axis (unnormalized) and the power normalization factor
#   QPSK: +/-1, 16QAM: +/-1,+/-3, 64QAM: +/-1..+/-7
MAX_LEVEL = {2: 1, 4: 3, 6: 7}
NORM = {2: np.sqrt(2), 4: np.sqrt(10), 6: np.sqrt(42)}

# Corner points on the Gray-coded constellation (3GPP TS 38.211 bit labels)
#   QPSK : upper left 10,     lower right 01
#   16QAM: upper left 1011,   lower right 0111
#   64QAM: upper left 101111, lower right 011111


def corner_value(Y, corner):
    """Power-normalized 'upper_left' or 'lower_right' corner for QAM option Y."""
    a = MAX_LEVEL[Y]
    v = complex(-a, a) if corner == "upper_left" else complex(a, -a)
    return v / NORM[Y]


# ---------------------------------------------------------------------------
# Step 1: alternating 1, 0, 1, 0, ... vectors, one per Ref vector length
# ---------------------------------------------------------------------------
def alternating_vector(n):
    """Length-n vector 1, 0, 1, 0, ... (starts with 1)."""
    return (np.arange(n) % 2 == 0).astype(np.uint8)


Vector_280 = alternating_vector(280)   # ReferenceSymbolProfile 3
Vector_140 = alternating_vector(140)   # ReferenceSymbolProfile 6
Vector_70 = alternating_vector(70)     # ReferenceSymbolProfile 12
Vector_35 = alternating_vector(35)     # ReferenceSymbolProfile 24

ALTERNATING_VECTORS = {280: Vector_280, 140: Vector_140, 70: Vector_70, 35: Vector_35}


# ---------------------------------------------------------------------------
# Step 2: scrambled vectors Vector_N_P0 (C_init_ID1) and Vector_N_P1 (C_init_ID2)
# ---------------------------------------------------------------------------
def scramble_vector(vector, c_init):
    """
    Scramble a 0/1 vector with Scrambler_GC.scrambler_gc using Gold code c_init.

    scrambler_gc always writes its Gold code to an h5 file as a side effect; that
    file goes to a temporary folder so the Scrambler folder's
    GC_Scrambler_Sequence.h5 is left untouched.
    """
    with tempfile.TemporaryDirectory() as tmp:
        return scrambler_gc(c_init, vector, h5_file=os.path.join(tmp, "GC_tmp.h5"))


def make_scrambled_vectors(c_init_id1=C_init_ID1, c_init_id2=C_init_ID2):
    """Returns {(N, z): Vector_N_Pz} for N in 280, 140, 70, 35 and z in 0, 1."""
    c_init = {0: c_init_id1, 1: c_init_id2}
    return {(n, z): scramble_vector(vec, c_init[z])
            for n, vec in ALTERNATING_VECTORS.items() for z in (0, 1)}


SCRAMBLED_VECTORS = make_scrambled_vectors()
Vector_280_P0 = SCRAMBLED_VECTORS[(280, 0)]
Vector_280_P1 = SCRAMBLED_VECTORS[(280, 1)]
Vector_140_P0 = SCRAMBLED_VECTORS[(140, 0)]
Vector_140_P1 = SCRAMBLED_VECTORS[(140, 1)]
Vector_70_P0 = SCRAMBLED_VECTORS[(70, 0)]
Vector_70_P1 = SCRAMBLED_VECTORS[(70, 1)]
Vector_35_P0 = SCRAMBLED_VECTORS[(35, 0)]
Vector_35_P1 = SCRAMBLED_VECTORS[(35, 1)]


# ---------------------------------------------------------------------------
# Step 3: map scrambled bits to constellation values -> Ref_Pz_X_Y
# ---------------------------------------------------------------------------
def num_ref_signals(X, z):
    """Number of Ref_Pz signals in one Symbol_Ref for ReferenceSymbolProfile X."""
    code = SignalType.REF_P0.value if z == 0 else SignalType.REF_P1.value
    return int(np.sum(REF_SYMBOL_LAYOUTS[X] == code))


def map_bits(bits, Y):
    """
    BPSK (Y=1)  : 0 -> +1, 1 -> -1, i.e. 1 - 2b (3GPP polarity, kept on the real axis)
    Y = 2, 4, 6 : 1 -> lower right corner, 0 -> upper left corner (power normalized)
    """
    bits = np.asarray(bits)
    if Y == 1:
        return (1.0 - 2.0 * bits).astype(complex)
    return np.where(bits == 1, corner_value(Y, "lower_right"), corner_value(Y, "upper_left"))


def reference_vector(z, X, Y, scrambled=SCRAMBLED_VECTORS):
    """Ref_Pz vector for ReferenceSymbolProfile X and QAM option Y."""
    n = num_ref_signals(X, z)
    return map_bits(scrambled[(n, z)], Y)


def save_vector_h5(vec, bits, z, X, Y, c_init, out_dir=OUTPUT_DIR):
    """
    Write one vector to <out_dir>/Ref_Pz_X_Y.h5

      /data   complex  reference values
      /bits   uint8    scrambled bits the values were mapped from
      attributes: Ref_P, ReferenceSymbolProfile, Bits_per_symbol, Modulation,
                  Length, C_init (the Gold code value used for this Ref_P)
    """
    filename = os.path.join(out_dir, f"Ref_P{z}_{X}_{Y}.h5")
    with h5py.File(filename, "w") as f:
        f.create_dataset("data", data=vec)
        f.create_dataset("bits", data=bits)
        f.attrs["Ref_P"] = z
        f.attrs["ReferenceSymbolProfile"] = X
        f.attrs["Bits_per_symbol"] = Y
        f.attrs["Modulation"] = QAM_OPTIONS[Y]
        f.attrs["Length"] = vec.size
        f.attrs["C_init"] = c_init
    return filename


def generate_all(c_init_id1=C_init_ID1, c_init_id2=C_init_ID2, write_files=True,
                 out_dir=OUTPUT_DIR):
    """Build all 32 vectors. Returns {(z, X, Y): vector}."""
    scrambled = make_scrambled_vectors(c_init_id1, c_init_id2)
    c_init = {0: c_init_id1, 1: c_init_id2}
    vectors = {}
    for z in (0, 1):
        for X in VALID_REF_PROFILES:
            for Y in QAM_OPTIONS:
                vec = reference_vector(z, X, Y, scrambled)
                vectors[(z, X, Y)] = vec
                if write_files:
                    bits = scrambled[(num_ref_signals(X, z), z)]
                    save_vector_h5(vec, bits, z, X, Y, c_init[z], out_dir)
    return vectors


if __name__ == "__main__":
    print(f"From {PARAMETERS_FILE}:")
    print(f"C_init_ID1 = {C_init_ID1} (P0), C_init_ID2 = {C_init_ID2} (P1)")
    vectors = generate_all(C_init_ID1, C_init_ID2, write_files=True)
    print(f"{'File':<16}{'Mod':<7}{'Len':>5}   First two values")
    for (z, X, Y), vec in vectors.items():
        name = f"Ref_P{z}_{X}_{Y}"
        print(f"{name:<16}{QAM_OPTIONS[Y]:<7}{vec.size:>5}   "
              f"{vec[0]:.4f}, {vec[1]:.4f}")
    print(f"{len(vectors)} files written to {OUTPUT_DIR}")
