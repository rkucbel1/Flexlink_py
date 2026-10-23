"""
Generate_AGC_Burst.py

Flexlink preamble - AGC burst.

The AGC burst is used at the receiver for fast detection of the signal
magnitude. It is built from a Zadoff-Chu sequence, giving a wideband signal
with a low peak-to-average power ratio.

Port of the MATLAB function GenerateAgcBurst_Flexlink(SampleRate).

Usage:
    python Generate_AGC_Burst.py [path/to/Parameters_Tx_payload.json]
    (defaults to <Flexlink_py>/Tx/Parameters_Tx_payload.json via flexlink_paths)

As a module:
    from Generate_AGC_Burst import generate_agc_burst
    agc = generate_agc_burst(40e6)
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


def generate_agc_burst(sample_rate):
    """
    Generate the Flexlink AGC burst.

    Parameters
    ----------
    sample_rate : float
        20e6 or 40e6.

    Returns
    -------
    np.ndarray (complex64)
        100 samples at 20 MHz, 200 samples at 40 MHz (approx. 5 us).
    """
    if sample_rate not in (20e6, 40e6):
        raise ValueError(f"Invalid SampleRate: {sample_rate} (must be 20e6 or 40e6)")

    # AGC burst length and IFFT size depend on the sample rate
    if sample_rate == 20e6:
        num_samples_to_retain = 100
        ifft_size = 1024
    else:
        num_samples_to_retain = 200
        ifft_size = 2048

    # Zadoff-Chu sequence definition
    n_zc = 887                       # Forces a broadband waveform
    sc_positive = int(np.ceil(n_zc / 2))   # Positive subcarriers incl. 0 Hz (444)
    sc_negative = int(np.floor(n_zc / 2))  # Negative subcarriers (443)
    u1 = 54                          # Root index
    n = np.arange(n_zc)              # Discrete time indices

    # Generate the Zadoff-Chu sequence (computed in double precision)
    zc = np.exp(-1j * np.pi * u1 * n * (n + 1) / n_zc)

    # 887-point DFT
    agc_burst_dft = np.fft.fft(zc)

    # Map the DFT output into the IFFT input buffer
    ifft_input = np.zeros(ifft_size, dtype=np.complex128)
    ifft_input[:sc_positive] = agc_burst_dft[:sc_positive]
    ifft_input[-sc_negative:] = agc_burst_dft[-sc_negative:]
    ifft_input[0] = 0.0              # Null out the DC term

    # numpy's ifft uses the same 1/N scaling as MATLAB's ifft
    agc_burst_full = np.fft.ifft(ifft_input)

    # Retain only the first 100 or 200 samples and rescale
    agc_burst = (len(agc_burst_full) / n_zc) * agc_burst_full[:num_samples_to_retain]

    return agc_burst.astype(np.complex64)


def main():
    param_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PARAM_FILE
    if not Path(param_file).is_file():
        sys.exit(f"Parameter file not found: {param_file}")

    sample_rate = load_sample_rate(param_file)
    agc = generate_agc_burst(sample_rate)

    power = np.mean(np.abs(agc) ** 2)
    papr_db = 10 * np.log10(np.max(np.abs(agc) ** 2) / power)
    print(f"SampleRate      : {sample_rate / 1e6:.0f} MHz")
    print(f"AGC burst length: {len(agc)} samples ({len(agc) / sample_rate * 1e6:.2f} us)")
    print(f"dtype           : {agc.dtype}")
    print(f"Mean power      : {power:.4f}")
    print(f"PAPR            : {papr_db:.2f} dB")


if __name__ == "__main__":
    main()
