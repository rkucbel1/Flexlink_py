#!/usr/bin/env python3
"""
Run_File_Transfer.py  (lives in Flexlink_py/App)

Sends a file through the whole link and gets it back at the receiver:

    once         Tx/Initialize_Tx.py, Rx/Initialize_Rx.py (skip with --no-init)
    per packet   Tx/Generate_FileData/Generate_File_Bitstream.py   (in place of step 3: the next
                     AvailableDataBlocks blocks of the file into Bitstream_TransportBlks_Tx.h5)
                 Tx/Run_Tx_Chain.py --keep-blocks                  (steps 4-8, new channel per packet
                                                                    when defects are modeled)
                 Rx/Run_Rx_Chain.py --no-model                     (steps 3-9)
                 Rx/Receive_File.py                                (keeps the blocks whose CRC passed)
    per round    round 1 sends every block; each later round resends only the blocks the
                 receiver is still missing (the receiver's list is used directly, as an ideal
                 feedback channel), up to --rounds rounds.

The file is cut into blocks as described in Flexlink_BasicModules/File_Transfer/File_Framing.py:
every block carries its file ID and block number, the first ones the file name, size and
SHA-256. The received file is written to Rx/Received_Files/ (or --out-dir) once its size and
SHA-256 check out, and is then also compared byte for byte with the file that was sent.

At the current settings (TransportBlockSize 1056, AvailableDataBlocks 5) a packet carries
5 x 123 = 615 bytes of the file, so a 100 kB file is about 165 packets per round.

Progress lines on stdout (for a GUI):
    PROGRESS <text>
    PACKET <json: round, packet, blocks, rx exit, new blocks, received, total>
    DONE <received file, or empty if not recovered>

Usage
-----
    python App/Run_File_Transfer.py --file photo.jpg
    python App/Run_File_Transfer.py --file photo.jpg --rounds 6 --seed 7
    python App/Run_File_Transfer.py --file notes.txt --no-defects        # clean channel
    python App/Run_File_Transfer.py --file photo.jpg --max-packets 20    # stop early (testing)

Exit status: 0 the file came back identical, 1 blocks still missing after the last round,
3 a step stopped with an error (initialization or the Tx chain), or bad arguments.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parent
try:
    sys.path.insert(0, str(APP.parent))
    from flexlink_paths import BASIC_MODULES, CHANNEL_DEFECTS, RX, TX
except ImportError:  # pragma: no cover
    raise
sys.path.insert(0, str(TX / "Generate_FileData"))
sys.path.insert(0, str(RX))
sys.path.insert(0, str(BASIC_MODULES / "File_Transfer"))
import Generate_File_Bitstream as gen  # noqa: E402
import Receive_File as recv  # noqa: E402
from File_Framing import FileReceiver  # noqa: E402

DEFECTS_FILE = CHANNEL_DEFECTS / "Channel_Defects.json"
TX_PARAMS = TX / "Parameters_Tx_payload.json"
RX_PARAMS = RX / "Parameters_Rx_payload.json"
LOG_FILE = RX / "File_Transfer.log"
RX_EXIT = {0: "all CRC pass", 1: "some CRC fail", 2: "no packet", 3: "Rx error", 4: "bad control field"}


def say(kind: str, text: str) -> None:
    print(f"{kind} {text}", flush=True)


class Transfer:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.py = sys.executable
        self.log_lines: list[str] = []

    # One place that runs a script, so tests can replace it.
    def run(self, label: str, cmd: list) -> tuple[int, str]:
        p = subprocess.run([self.py, *map(str, cmd)], capture_output=True, text=True)
        out = p.stdout + p.stderr
        self.log_lines.append(f"===== {label} (exit {p.returncode})\n{out}")
        if self.a.verbose:
            print(f"--- {label} (exit {p.returncode})\n{out}", end="", flush=True)
        return p.returncode, out

    def write_log(self) -> None:
        try:
            Path(self.a.log).write_text("\n".join(self.log_lines))
        except OSError:
            pass

    def initialize(self) -> bool:
        for label, script in (("Initialize_Tx", TX / "Initialize_Tx.py"), ("Initialize_Rx", RX / "Initialize_Rx.py")):
            rc, _ = self.run(label, [script])
            if rc:
                say("PROGRESS", f"{label} failed (exit {rc}), see {self.a.log}")
                return False
        return True

    def send_packet(self, sender, blocks: list[int], seed: int | None) -> int:
        """Tx then Rx for one packet. Returns the Rx chain's exit status, or -1 if the Tx failed."""
        gen.write_packet(self.tx_blocks_path, sender, blocks)
        cmd = [TX / "Run_Tx_Chain.py", "--keep-blocks", "--defects-file", self.a.defects_file]
        if seed is not None:
            cmd += ["--channel-seed", seed]
        if self.a.no_defects:
            cmd.append("--no-defects")
        rc, _ = self.run("Run_Tx_Chain", cmd)
        if rc:
            return -1
        cmd = [RX / "Run_Rx_Chain.py", "--no-model", "--defects-file", self.a.defects_file]
        if self.a.eq_mode:
            cmd += ["--eq-mode", self.a.eq_mode]
        rc, _ = self.run("Run_Rx_Chain", cmd)
        return rc

    def status(self, fid: int) -> dict | None:
        """The receiver's view of the file, read fresh from the state folder."""
        try:
            return FileReceiver(self.a.state_dir).status(fid)
        except KeyError:
            return None

    def main(self) -> int:
        a = self.a
        if not a.file.is_file():
            print(f"Error: file not found: {a.file}", file=sys.stderr)
            return 3
        t0 = time.time()
        if not a.no_init:
            say("PROGRESS", "Initialize_Tx.py, Initialize_Rx.py")
            if not self.initialize():
                self.write_log()
                return 3
        try:
            sender = gen.make_sender(a.file, TX_PARAMS, a.file_id)
            self.tx_blocks_path = gen.load_parameters(TX_PARAMS)[2]
        except (FileNotFoundError, ValueError) as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 3
        fid = sender.file_id
        if not a.resume:
            FileReceiver(a.state_dir).reset(fid)
        plan = sender.packet_plan()
        say("PROGRESS", f"{sender.name}: {sender.size} bytes, file ID {fid}, {sender.total} block(s) of "
                        f"{sender.chunk} bytes, {len(plan)} packet(s) per full round "
                        f"({sender.blocks_per_packet} blocks of {sender.tb_bits} bits per packet)")

        st = self.status(fid)
        missing = st["missing"] if (a.resume and st) else list(range(sender.total))
        sent = blocks_sent = lost_packets = tx_errors_in_row = 0
        written: Path | None = None
        counts = {k: 0 for k in RX_EXIT}
        for rnd in range(1, a.rounds + 1):
            if not missing:
                break
            plan = sender.packet_plan(missing)
            say("PROGRESS", f"Round {rnd}: {len(missing)} block(s) in {len(plan)} packet(s)")
            for k, blocks in enumerate(plan, 1):
                if a.max_packets and sent >= a.max_packets:
                    break
                seed = None if a.seed is None else a.seed + sent
                rc = self.send_packet(sender, blocks, seed)
                sent += 1
                blocks_sent += len(blocks)
                if rc == -1:
                    tx_errors_in_row += 1
                    say("PROGRESS", f"Run_Tx_Chain failed for packet {k}, see {a.log}")
                    if tx_errors_in_row >= 3:
                        self.write_log()
                        say("DONE", "")
                        return 3
                    continue
                tx_errors_in_row = 0
                counts[rc if rc in counts else 3] += 1
                new = 0
                if rc in (0, 1):
                    path = recv.default_blocks_file(RX_PARAMS)
                    if path.is_file() and not recv.is_stale(path, RX_PARAMS):
                        bits, crc = recv.load_blocks(path)
                        rep, status, w = recv.receive(bits, crc, a.state_dir, a.out_dir)
                        new = rep.new
                        if status.get(fid, {}).get("error"):
                            say("PROGRESS", status[fid]["error"])
                        if w:
                            written = w[0]
                    else:
                        lost_packets += 1
                else:
                    lost_packets += 1
                st = self.status(fid)
                rcv = st["received"] if st else 0
                say("PACKET", json.dumps({"round": rnd, "packet": k, "of": len(plan), "blocks": blocks,
                                          "rx_exit": rc, "rx": RX_EXIT.get(rc, "?"), "new": new,
                                          "received": rcv, "total": sender.total}))
                if written:
                    break
            if written or (a.max_packets and sent >= a.max_packets):
                break
            st = self.status(fid)
            missing = st["missing"] if st else list(range(sender.total))

        self.write_log()
        secs = time.time() - t0
        print(f"Packets sent {sent} ({blocks_sent} blocks, file has {sender.total}), "
              f"Rx: " + ", ".join(f"{v} {RX_EXIT[k]}" for k, v in counts.items() if v)
              + f"; {secs:.1f} s")
        if written:
            same = written.read_bytes() == a.file.read_bytes()
            print(f"Received {written} ({written.stat().st_size} bytes): "
                  + ("identical to the file sent" if same else "DIFFERENT from the file sent"))
            say("DONE", str(written))
            return 0 if same else 1
        st = self.status(fid)
        if st:
            print(f"Not recovered: {st['received']} of {st['total']} block(s) received, missing "
                  f"{recv.ranges(st['missing'])}. Run again with --resume to send only those.")
        else:
            print("Not recovered: no block of the file was received")
        say("DONE", "")
        return 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Send a file through the Tx and Rx chains and get it back.")
    ap.add_argument("--file", type=Path, required=True, help="file to send")
    ap.add_argument("--rounds", type=int, default=8,
                    help="send rounds: 1 = every block once, later rounds resend what is missing (default 8)")
    ap.add_argument("--max-packets", type=int, default=0, help="stop after this many packets (0 = no limit)")
    ap.add_argument("--seed", type=int, help="channel seed of the first packet (+1 per packet): repeatable runs")
    ap.add_argument("--file-id", type=int, help="0..65535 (default: from the file name and contents)")
    ap.add_argument("--resume", action="store_true",
                    help="keep the blocks already received for this file and send only the missing ones")
    ap.add_argument("--no-defects", action="store_true", help="never run Defect_Model.py (clean channel)")
    ap.add_argument("--defects-file", type=Path, default=DEFECTS_FILE)
    ap.add_argument("--eq-mode", help="override Defined_Rx/EqualizerMode")
    ap.add_argument("--no-init", action="store_true", help="skip Initialize_Tx.py and Initialize_Rx.py")
    ap.add_argument("--state-dir", type=Path, default=recv.STATE_DIR)
    ap.add_argument("--out-dir", type=Path, default=recv.OUT_DIR, help="where the received file goes")
    ap.add_argument("--log", type=Path, default=LOG_FILE, help="every step's output")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every step's output")
    a = ap.parse_args(argv)
    if a.rounds < 1:
        ap.error("--rounds must be at least 1")
    return Transfer(a).main()


if __name__ == "__main__":
    sys.exit(main())
