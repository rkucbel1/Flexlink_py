# Flexlink_py

A Python model of the FlexLink OFDM datalink: transmitter chain, channel defects
(multipath, noise, frequency and timing offsets, phase noise, IQ imbalance), receiver chain
with up to four receive antennas, and a desktop app to set parameters, run packets and look
at the results.

The same chains drive the hardware: an FPGA transmitter on an ADALM-Pluto and FPGA receivers
on a Zedboard (with an ESP32-S31 CoreBoard or an AD-FMCOMMS3 as the radio), with apps for
packets, files, live video and MAVLink telemetry.

## Quick start

You need **Python 3.10 or newer** ([python.org/downloads](https://www.python.org/downloads/)).
On Windows, tick *Add python.exe to PATH* in the installer.

1. Download the project: on this page click **Code → Download ZIP** and unzip it, or
   ```
   git clone https://github.com/rkucbel1/Flexlink_py.git
   ```
2. Start the app:
   - **Windows:** double-click `run_flexlink.bat`
   - **Linux / macOS:** in a terminal, in the project folder:
     ```
     ./run_flexlink.sh
     ```
     (If it says *permission denied*: `chmod +x run_flexlink.sh` once.)

The first start takes a minute or two: it creates a virtual environment in `.venv` and
installs numpy, scipy, h5py, matplotlib and FreeSimpleGUI into it. Later starts open the
app straight away. Nothing is installed outside the project folder.

On Linux the app also needs Tkinter, which some distributions ship separately:
`sudo apt install python3-tk` (Debian/Ubuntu) or `sudo dnf install python3-tkinter` (Fedora).

### Installing by hand

```
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -e .
python App/Simulation/Flexlinksimulation.py
```

`pip install -e .` installs the dependencies and `flexlink_paths`, which the scripts use to
find each other, so they run from any folder.

## Using the app

- **Set Defects:** channel model, SNR per antenna, frequency offset, phase noise, IQ
  imbalance, timing; a Doppler-to-velocity table.
- **Set Tx / Set Rx:** packet length, modulation, code rate, block size, repetition,
  reference-signal spacing, equalizer and number of receive antennas.
- **Simulate:** runs one packet through the whole link (Initialize, Tx chain, Rx chain) and
  shows the control field results, the CRC of every block, bit errors and plots. It can also
  run an SNR sweep (BER, BLER, EVM and goodput against SNR).
- **Hardware in the loop:** on Set Rx, *Receiver* → *Hardware in the loop: Zedboard FPGA*
  (and Save) sends each Run's waveform through the FPGA receiver
  (`HW_Development/Packet_Detector`, bitstream 9.0 / firmware build 16 or later) instead of the Python Rx
  chain, and checks every decoded transport block against the one the Tx sent. One waveform
  only (Number of Rays 1). Each run is added to `App/Simulation/HIL/HIL_results.csv`.

**Save** writes the settings to the parameter files; **Run** runs the simulation.

## Command line

Everything the app does is also a script:

```
python Tx/Initialize_Tx.py && python Tx/Run_Tx_Chain.py
python Rx/Initialize_Rx.py && python Rx/Run_Rx_Chain.py --no-model
python App/Simulation/Run_SNR_Sweep.py --start 0 --stop 20 --step 2
python App/Simulation/Run_File_Transfer.py --file some_file.jpg      # send a file and get it back
python App/Simulation/Run_HIL.py                                     # last Tx waveform through the Zedboard FPGA
```

Each script prints its options with `--help`.

## Hardware apps

The apps for the real radios are in `App/Tx_Pluto/`, `App/Rx_CoreBoard/` and `App/Rx_FCOMMS3/`;
[App/README.md](App/README.md) lists every script and what it does. Start them with the Python in
`.venv`, e.g. `.venv/bin/python App/Tx_Pluto/FlexlinkTxWithPluto.py`
(Windows: `.venv\Scripts\python.exe App\Tx_Pluto\FlexlinkTxWithPluto.py`).

Some need more than `pip install -e .` installs:

- Pluto loopback (`FlexlinkTxWithPluto.py`): `pip install pyadi-iio` and the libiio library
- Live video: FFmpeg's `ffplay` on the PATH; on the transmitter side, SSH access to the Raspberry Pi
- MAVLink from a serial port: `pip install pyserial`

The FPGA designs, firmware and their test tools are in `HW_Development/` (each folder has a README or, for
`Packet_Detector_FCOMM`, `HANDOVER_FMCOMMS3.md`).

## Project layout

| Folder | What is in it |
| --- | --- |
| `App/Simulation/` | the app (`Flexlinksimulation.py`), plots, SNR sweep, file transfer, hardware in the loop (`Run_HIL.py`) |
| `App/Tx_Pluto/` | transmitting with the Pluto: packets, files, video, video + MAVLink |
| `App/Rx_CoreBoard/` | receiving with the CoreBoard: capture, emulator, processing, video and MAVLink receivers |
| `App/Rx_FCOMMS3/` | receiving with the AD-FMCOMMS3 on the Zedboard: video and MAVLink receivers |
| `App/Mavlink/` | MAVLink modules shared by the transmitter and the receivers |
| `Tx/` | transmitter chain, `Parameters_Tx_payload.json`, control field parameters |
| `Rx/` | receiver chain, `Parameters_Rx_payload.json` |
| `Flexlink_BasicModules/` | building blocks: LDPC and BCC coding, interleaver, scrambler, QAM, OFDM, preamble, channel estimator, channel defects (`Channel_Defects.json`) |
| `ReferenceSignals/` | precomputed reference signals and the script that makes them |
| `HW_Development/` | FPGA receivers (`Packet_Detector` for the CoreBoard, `Packet_Detector_FCOMM` for the FMCOMMS3) and transmitter (`Packet_Transmitter`, Pluto): RTL, Vivado builds, firmware, simulation, PC test tools |
| `Docs/` | specifications (control field, packet detector, payload input, payload receiver) and the implementation analysis |

Each app folder keeps its own parameter files (`Parameters_*.json`) and output folders; all
folder locations are defined in `flexlink_paths.py`.

Descriptions of the chains: `Tx/Docs/Flexlink_Payload _Tx_Chain.md` and
`Rx/Docs/Flexlink_Payload_Rx_Chain.md`.

Run outputs (`.h5` files in `Tx/` and `Rx/`, logs, sweeps, HIL results, captures, received
files and video) are recreated by the scripts and are not part of the repository (one 8 MB
CoreBoard capture is kept as a test signal).

## License

MIT, see [LICENSE](LICENSE).
