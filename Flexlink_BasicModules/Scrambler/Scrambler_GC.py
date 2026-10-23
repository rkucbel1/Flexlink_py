"""
Scrambler_GC.py

Scrambles a bit sequence by modulo-2 addition with the Gold code stored in
GC_Scrambler_Sequence.h5 (written by Scrambler_Gold_Code_Generator.py).

MATLAB equivalent:
    ScrambledSequence = mod((SequenceToScramble + Sequence), 2);

The length of SequenceToScramble is checked against the length stored in the
h5 file.  If they match, the stored Sequence is used.  If they do not match
(or the file is missing, or it was generated with a different id), the Gold
code generator is called to regenerate the Sequence (which also rewrites the
h5 file), so the next call with the same length and id can reuse it.
"""

import os

import h5py
import numpy as np

from Scrambler_Gold_Code_Generator import gold_code_generator, H5_FILE


def _read_h5(filename):
    """Return (Sequence, length, c_init) from the h5 file, or None if absent."""
    if not os.path.isfile(filename):
        return None
    with h5py.File(filename, "r") as f:
        sequence = f["Sequence"][:]
        length = int(f["length"][()])
        c_init = int(f.attrs["c_init"])
    return sequence, length, c_init


def scrambler_gc(id=130, sequence_to_scramble=None, h5_file=H5_FILE):
    """
    Parameters
    ----------
    id : int
        Gold code initial value (c_init). Default 130.
    sequence_to_scramble : array-like of 0/1
        Bits to scramble.
    h5_file : str
        h5 file holding the stored Sequence and its length.

    Returns
    -------
    ScrambledSequence : np.ndarray, dtype=np.uint8, same length as the input
    """
    bits = np.asarray(sequence_to_scramble).astype(np.int64).ravel()
    number_of_output_bits = len(bits)

    stored = _read_h5(h5_file)
    if (stored is not None
            and stored[1] == number_of_output_bits   # length matches
            and stored[2] == int(id) & 0x7FFFFFFF):  # same c_init
        sequence = stored[0].astype(np.int64)
    else:
        sequence = gold_code_generator(id, number_of_output_bits, h5_file=h5_file)
        sequence = sequence.astype(np.int64)

    return ((bits + sequence) % 2).astype(np.uint8)


if __name__ == "__main__":
    # Test: random 1024-bit vector, id = 130
    ID = 130
    SequenceToScramble = np.random.default_rng().integers(0, 2, size=1024)

    ScrambledSequence = scrambler_gc(ID, SequenceToScramble)

    # Scrambling again with the same id must recover the input
    assert len(ScrambledSequence) == len(SequenceToScramble)
    assert np.array_equal(scrambler_gc(ID, ScrambledSequence), SequenceToScramble)
