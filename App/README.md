# App

The Flexlink applications: the simulator GUI and the tools that run the link on real radios.
Each folder holds its scripts, the parameter files they use (`Parameters_*.json`) and the
folders they write to. Output folders are recreated by the scripts and are not in git.

Run any script from the project folder, e.g. `python App/Simulation/Flexlinksimulation.py`.
Every command-line script prints its options with `--help`.

## Simulation/

The simulated link (Python Tx chain, channel defects, Python Rx chain), plus hardware in the loop.

| Script | What it does |
| --- | --- |
| `Flexlinksimulation.py` | The main GUI: set defects, Tx and Rx parameters, run packets, show results and plots, start SNR sweeps. |
| `Flexlink_Plots.py` | Plots for the GUI (constellations, EVM, Tx/Rx packet waveforms, SNR sweep curves). Imported, not run. |
| `Run_SNR_Sweep.py` | Monte Carlo sweep over SNR; records BER, BLER, PER, EVM and goodput. |
| `Run_File_Transfer.py` | Sends a file through the simulated link and rebuilds it at the receiver. |
| `Run_HIL.py` | Hardware in the loop: sends the last Tx waveform through the Zedboard FPGA receiver instead of the Python Rx chain and checks the decoded blocks. |

| File / folder | |
| --- | --- |
| `Parameters_HIL.json` | Receiver choice (simulation or HIL) and the Zedboard's address. |
| `Sweeps/` | One `Sweep_<date>_<time>/` folder per sweep: `results.csv`, `settings.json`, `sweep.png`. |
| `HIL/` | HIL results (`HIL_results.csv` gets a row per run), last result, test vectors, log. |

## Tx_Pluto/

Transmitting with the Flexlink transmitter in an ADALM-Pluto's FPGA.

| Script | What it does |
| --- | --- |
| `FlexlinkTxWithPluto.py` | Transmitter GUI: random packets, a selected file, or loopback (the Pluto receives its own signal and the Python Rx chain decodes it). |
| `FlexlinkTxVideoWithPluto.py` | Live H.264 video from a Raspberry Pi (file or camera) over the link; can also loop back on this PC without a Pluto. |
| `FlexlinkTxMavlinkWithPluto.py` | The video app plus a MAVLink telemetry stream in the same packets. |

| File / folder | |
| --- | --- |
| `Parameters_Pluto.json` | Pluto address, radio settings (carrier, Tx/Rx gain, RF bandwidth) and what to send. |
| `Parameters_Video.json` | Raspberry Pi and PC addresses, video source, share of the link used for video. |
| `Loopback/` | Output of loopback runs: captures, log, and `Loopback_Received.ts`. |
| `Test_Files/` | Files to send in "Selected file" mode (where the Browse dialog starts). |

## Rx_CoreBoard/

Receiving with an ESP32-S31 CoreBoard as the radio, either decoded on the PC from a capture or
live through the Zedboard FPGA receiver.

| Script | What it does |
| --- | --- |
| `FlexlinkCaptureWithCoreBoard.py` | Records a few seconds of I/Q from the CoreBoard over Ethernet. |
| `FlexlinkProcessCoreBoardCapturedWaveform.py` | Finds and decodes the Flexlink packets in a capture with the Python Rx chain; rebuilds transferred files. |
| `FlexlinkCoreBoardEmulator.py` | Stands in for the CoreBoard on the PC, streaming a capture file, so the other tools can be tested without the board. |
| `FlexlinkRxVideoWithZedboard_CoreBoard.py` | Live video receiver GUI: CoreBoard streams to the Zedboard, which decodes and sends the video to this PC. |
| `FlexlinkRxMavlinkWithZedboard_CoreBoard.py` | The video receiver plus the MAVLink telemetry window. |

| File / folder | |
| --- | --- |
| `Parameters_CoreBoard.json` | CoreBoard address and radio settings (carrier, sample rate, gain). |
| `Parameters_RxVideo.json` | Zedboard receiver address and ports, playback and save options. |
| `Captures/` | `.cs8` captures and their `.json`; `Segment_newfw_3820ms.cs8` is a tracked known-good test signal. |
| `Results/` | Output of the processing script: packets, results, logs, received files. |
| `Video/` | `Received.ts` when "Save stream" is on. |

## Rx_FCOMMS3/

Receiving with an AD-FMCOMMS3 on the Zedboard; the FPGA decodes and sends the data to this PC.

| Script | What it does |
| --- | --- |
| `FlexlinkRxVideoWithZedboard_FCOMMS3.py` | Live video receiver GUI. |
| `FlexlinkRxMavlinkWithZedboard_FCOMMS3.py` | The video receiver plus the MAVLink telemetry window. |

| File / folder | |
| --- | --- |
| `Parameters_FMCOMMS3.json` | Zedboard address and AD9361 settings (carrier, sample rate, gain), playback options. |
| `Video/` | `Received.ts` when "Save stream" is on. |

## Mavlink/

MAVLink modules shared by the Pluto transmitter and both receivers.

| Script | What it does |
| --- | --- |
| `Mavlink_Lite.py` | Minimal MAVLink 2: builds and parses frames, reads and writes `.tlog` files (no pymavlink needed). |
| `Mavlink_Framing.py` | Packs MAVLink frames into the transport blocks next to the video and unpacks them; the Tx sources and Rx outputs (file, UDP, serial). |
| `Rx_Mavlink.py` | The MAVLink part of the receiver apps: unpacks the frames, forwards MAVLink and video to a ground station (UDP), saves `Received.tlog`. |
| `Telemetry_Window.py` | Telemetry window for the receivers: artificial horizon, compass, position, speeds, battery, status messages. |
| `Generate_Sim_Telemetry.py` | Writes `Sim_Telemetry.tlog`, simulated quadcopter telemetry for testing without a flight controller. |

| File | |
| --- | --- |
| `Parameters_Mavlink.json` | Transmitter MAVLink source and options. |
| `Parameters_RxMavlink.json` | Receiver MAVLink outputs (shared by both receivers). |
| `Sim_Telemetry.tlog` | The test telemetry file. |
| `Received.tlog`, `Loopback_Received.tlog` | Recorded telemetry (receivers, and the Tx loopback mode); not in git. |
