"""
Generate_Test_Inputs.py

Creates a 2048-element test sequence that is exactly 25% ones and 75% zeros
(512 ones, 1536 zeros) in random positions, and writes it to Test_Input_2048.h5.

HDF5 contents:
    /SequenceToScramble   dataset, uint8 array of 0/1
    /length               dataset, number of elements
"""

import h5py
import numpy as np

LENGTH = 2048
FRACTION_ONES = 0.25
H5_FILE = "Test_Input_2048.h5"


def generate_test_input(length=LENGTH, fraction_ones=FRACTION_ONES):
    number_of_ones = int(round(length * fraction_ones))
    sequence = np.zeros(length, dtype=np.uint8)
    sequence[:number_of_ones] = 1
    np.random.default_rng().shuffle(sequence)  # positions of the values don't matter
    return sequence


if __name__ == "__main__":
    SequenceToScramble = generate_test_input()

    with h5py.File(H5_FILE, "w") as f:  # "w" creates or overwrites
        f.create_dataset("SequenceToScramble", data=SequenceToScramble)
        f.create_dataset("length", data=len(SequenceToScramble))
