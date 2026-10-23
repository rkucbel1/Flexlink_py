"""
Generate_Preamble_A.py

Flexlink preamble - Part A.

Preamble A is used to:
  - detect the presence of a packet
  - determine the frequency offset of the packet

It is the sum of two cosines at 625 kHz and 1.875 MHz, lasting 250 us
(5000 samples at 20 MHz, 10000 samples at 40 MHz).

Port of the MATLAB function GeneratePreambleA_Flexlink(SampleRate).

Usage:
    python Generate_Preamble_A.py [path/to/Parameters_Tx_payload.json]
    (defaults to <Flexlink_py>/Tx/Parameters_Tx_payload.json via flexlink_paths)

As a module:
    from Generate_Preamble_A import generate_preamble_a
    pa = generate_preamble_a(40e6)
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


def generate_preamble_a(sample_rate):
    """
    Generate Flexlink Preamble A.

    Parameters
    ----------
    sample_rate : float
        20e6 or 40e6.

    Returns
    -------
    np.ndarray (complex64)
        5000 samples at 20 MHz, 10000 samples at 40 MHz (250 us).
        The waveform is real-valued; it is returned as complex64 (zero
        imaginary part) so it concatenates directly with the other parts.
    """
    if sample_rate not in (20e6, 40e6):
        raise ValueError(f"Invalid SampleRate: {sample_rate} (must be 20e6 or 40e6)")

    # Preamble A lasts 250 us
    num_samples = 5000 if sample_rate == 20e6 else 10000

    # Two tones: 625 kHz and 1.875 MHz (MATLAB note: "Repeat 32 and 96 times")
    cosine_frequency_a = 625000.0
    cosine_frequency_b = 1875000.0
    n = np.arange(num_samples)

    preamble_a = (np.cos(2 * np.pi * cosine_frequency_a * n / sample_rate + np.pi / 4)
                  + np.cos(2 * np.pi * cosine_frequency_b * n / sample_rate + 3 * np.pi / 4))

    return preamble_a.astype(np.complex64)


def main():
    param_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PARAM_FILE
    if not Path(param_file).is_file():
        sys.exit(f"Parameter file not found: {param_file}")

    sample_rate = load_sample_rate(param_file)
    pa = generate_preamble_a(sample_rate)

    power = np.mean(np.abs(pa) ** 2)
    papr_db = 10 * np.log10(np.max(np.abs(pa) ** 2) / power)
    print(f"SampleRate        : {sample_rate / 1e6:.0f} MHz")
    print(f"Preamble A length : {len(pa)} samples ({len(pa) / sample_rate * 1e6:.2f} us)")
    print(f"dtype             : {pa.dtype}")
    print(f"Mean power        : {power:.4f}")
    print(f"PAPR              : {papr_db:.2f} dB")


if __name__ == "__main__":
    main()
