"""
Generate_Preamble_B.py

Flexlink preamble - Part B.

Preamble B is used at the receiver to acquire timing. It is built from a
Zadoff-Chu sequence with good auto-correlation behaviour; the received
waveform is correlated against a copy of Preamble B.

Length: 1140 samples at 20 MHz, 2280 samples at 40 MHz (57 us),
i.e. a 1024/2048-sample body preceded by a 116/232-sample cyclic prefix.

Port of the MATLAB function GeneratePreambleB_Flexlink(SampleRate).

Usage:
    python Generate_Preamble_B.py [path/to/Parameters_Tx_payload.json]
    (defaults to <Flexlink_py>/Tx/Parameters_Tx_payload.json via flexlink_paths)

As a module:
    from Generate_Preamble_B import generate_preamble_b
    pb = generate_preamble_b(40e6)
"""

import json
import sys
from pathlib import Path

import numpy as np

# Locate the Flexlink_py root (the folder holding flexlink_paths.py) by walking
# up from this file, so the script runs from any working directory.
_HERE = Path(__file__).resolve().parent
for _p in (_HERE, *_HERE.parents):
    if (_p / "flexlink_paths.py").is_file():
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))
        break
else:
    raise ImportError("flexlink_paths.py not found in this folder or any parent folder")

from flexlink_paths import TX  # noqa: E402

DEFAULT_PARAM_FILE = TX / "Parameters_Tx_payload.json"


def load_sample_rate(param_file=DEFAULT_PARAM_FILE):
    """Read SampleRate from the 'Defined' section of the Tx parameter JSON."""
    with open(param_file, "r") as f:
        params = json.load(f)
    return float(params["Defined"]["SampleRate"])


def generate_preamble_b(sample_rate):
    """
    Generate Flexlink Preamble B (with cyclic prefix).

    Parameters
    ----------
    sample_rate : float
        20e6 or 40e6.

    Returns
    -------
    np.ndarray (complex64)
        1140 samples at 20 MHz, 2280 samples at 40 MHz (57 us).
    """
    if sample_rate not in (20e6, 40e6):
        raise ValueError(f"Invalid SampleRate: {sample_rate} (must be 20e6 or 40e6)")

    # Body length = IFFT size; CP is the last 116 / 232 samples of the body
    if sample_rate == 20e6:
        num_samples_to_retain = 1024
        ifft_size = 1024
        cp_start = 908        # MATLAB index 909 (1-based)
    else:
        num_samples_to_retain = 2048
        ifft_size = 2048
        cp_start = 1816       # MATLAB index 1817 (1-based)

    # Zadoff-Chu sequence definition
    n_zc = 887
    sc_positive = int(np.ceil(n_zc / 2))   # Positive subcarriers incl. 0 Hz (444)
    sc_negative = int(np.floor(n_zc / 2))  # Negative subcarriers (443)
    u1 = 54                                # Root index
    n = np.arange(n_zc)

    # Generate the Zadoff-Chu sequence (double precision)
    zc = np.exp(-1j * np.pi * u1 * n * (n + 1) / n_zc)

    # DFT
    preamble_b_dft = np.fft.fft(zc)

    # Map the DFT output into the IFFT input buffer
    ifft_input = np.zeros(ifft_size, dtype=np.complex128)
    ifft_input[:sc_positive] = preamble_b_dft[:sc_positive]
    ifft_input[-sc_negative:] = preamble_b_dft[-sc_negative:]
    ifft_input[0] = 0.0                    # Null out the DC term

    # numpy's ifft uses the same 1/N scaling as MATLAB's ifft
    preamble_b_full = np.fft.ifft(ifft_input)
    body = (len(preamble_b_full) / n_zc) * preamble_b_full[:num_samples_to_retain]

    # Prepend cyclic prefix
    cp = body[cp_start:num_samples_to_retain]
    preamble_b = np.concatenate([cp, body])

    return preamble_b.astype(np.complex64)


def main():
    param_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PARAM_FILE
    if not Path(param_file).is_file():
        sys.exit(f"Parameter file not found: {param_file}")

    sample_rate = load_sample_rate(param_file)
    pb = generate_preamble_b(sample_rate)

    power = np.mean(np.abs(pb) ** 2)
    papr_db = 10 * np.log10(np.max(np.abs(pb) ** 2) / power)
    print(f"SampleRate        : {sample_rate / 1e6:.0f} MHz")
    print(f"Preamble B length : {len(pb)} samples ({len(pb) / sample_rate * 1e6:.2f} us)")
    print(f"dtype             : {pb.dtype}")
    print(f"Mean power        : {power:.4f}")
    print(f"PAPR              : {papr_db:.2f} dB")


if __name__ == "__main__":
    main()
