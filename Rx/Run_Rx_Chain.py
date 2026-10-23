#!/usr/bin/env python3
"""
Run_Rx_Chain.py  (lives in Flexlink_py/Rx)

Runs the payload Rx chain, steps 3 to 9, in the right order for one or several
receive antennas:

    0   Tx/Defect_Model.py                         when defects are modeled and the waveform
                                                   is missing or older than Channel_Defects.json
                                                   (always with --new-channel, never with
                                                   --no-model)
    3   Decimate_Payload_Rx.py                     once
    4   Packet_Detector.py                         once per antenna (--ray 1 .. N)
    4a  Process_Control_Field_Rx.py                once, on every antenna with a packet, when
                                                   Defined_Rx/Use_ControlField is 1 (default): decodes the
                                                   control field, updates Defined_Rx (and runs
                                                   Initialize_Rx.py) if the payload settings
                                                   changed, and cuts the control field off
    5-6 Demodulate, Equalize                       once per antenna with a packet
    6b  Combine_Antennas_Rx.py                     when N > 1 (MRC of the antennas that worked)
    7-9 Get_DataSigs_QAM, Demap, Decode            once, on the combined grid (N > 1) or ray 1

N is the NumberOfAntennas that Decimate_Payload_Rx.py records: 1 without defects, else
Defined_Rx/NumberOfAntennas. An antenna whose packet detection fails (exit code 2) is left
out of the combining; the run stops only if no antenna has a packet.

By default every file goes to Rx/ under its usual name (*_Ray1..N, *_Combined), exactly as
running the scripts by hand. --out-dir writes them to another folder instead (used by
Diagnose_Defects.py). Nothing is written to the JSON files, except by step 4a when the
control field carries payload settings that differ from Defined_Rx.

Usage
-----
    python Rx/Run_Rx_Chain.py                       # decode and compare with the Tx blocks
    python Rx/Run_Rx_Chain.py --per-ray             # also decode every antenna on its own
    python Rx/Run_Rx_Chain.py --eq-mode PCA --max-doppler 400
    python Rx/Run_Rx_Chain.py --new-channel         # rerun Tx/Defect_Model.py first (new random channel)
    python Rx/Run_Rx_Chain.py --out-dir my_run --input other_waveform.h5
    python Rx/Run_Rx_Chain.py --dry-run             # print the commands only

Exit status: 0 all blocks pass, 1 a block failed, 2 no packet on any antenna, 3 a step
stopped with an error, 4 bad packet (the control field CRC failed or it held an invalid value).
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

RX = Path(__file__).resolve().parent
ROOT = RX.parent
TX = ROOT / "Tx"
sys.path.insert(0, str(RX))
from Rx_Antennas import combined_file_name  # noqa: E402
from flexlink_paths import ray_file_name  # noqa: E402

PARAMS_FILE = RX / "Parameters_Rx_payload.json"
DEFECTS_FILE = ROOT / "Flexlink_BasicModules" / "Channel_Defects" / "Channel_Defects.json"
COMPARE_FILE = ROOT / "Tx" / "Generate_RandomPacketOfData" / "Bitstream_TransportBlks_Tx.h5"
KEYS = {"dec": ("Waveform_Payload_Dec_Rx", "Waveform_Payload_Dec_Rx.h5"),
        "pkt": ("Waveform_Payload_Packet_Rx", "Decimated_Payload_Packet_Rx.h5"),
        "grid": ("Grid_Payload_Rx", "Grid_Payload_Rx.h5"),
        "eq": ("Grid_Equalized_Rx", "Grid_Equalized_Rx.h5"),
        "qam": ("Sequence_DataSigs_QAM", "Sequence_DataSigs_QAM.h5"),
        "bits": ("Bitstream_Encoded_Rx", "Bitstream_Encoded_Rx.h5"),
        "blocks": ("Bitstream_TransportBlks_Rx", "Bitstream_TransportBlks_Rx.h5"),
        "control": ("ControlField_Rx", "ControlField_Rx.h5")}


def find(pattern: str, text: str) -> str | None:
    m = re.search(pattern, text)
    return m.group(1) if m else None


class Chain:
    def __init__(self, params_path: Path, out_dir: Path, dry: bool = False, verbose: bool = False):
        self.params_path = Path(params_path)
        self.params = json.loads(self.params_path.read_text())
        self.out = Path(out_dir)
        self.dry, self.verbose = dry, verbose
        self.py = sys.executable
        self.log: list[str] = []

    def name(self, key: str) -> str:
        k, default = KEYS[key]
        return self.params.get("File_Rx", {}).get(k) or default

    def path(self, key: str, ray: int | str | None) -> Path:
        n = self.name(key)
        if ray == "combined":
            n = combined_file_name(n)
        elif ray is not None:
            n = ray_file_name(n, ray)
        return self.out / n

    def run(self, step: str, cmd: list[Any]) -> tuple[int, str]:
        cmd = [str(c) for c in cmd]
        if self.dry:
            print("   ", " ".join(cmd))
            return 0, ""
        p = subprocess.run(cmd, capture_output=True, text=True)
        out = p.stdout + p.stderr
        self.log.append(f"===== {step} (exit {p.returncode})\n{out}")
        if self.verbose:
            print(f"--- {step} (exit {p.returncode})\n{out}", end="")
        return p.returncode, out

    def tail(self, ray: int | str, qam_in: Path, demap_args, compare) -> dict[str, Any]:
        """Steps 7-9 for one input grid; returns the decode results."""
        prm = ["--params", self.params_path]
        qam, bits, blocks = self.path("qam", ray), self.path("bits", ray), self.path("blocks", ray)
        rc, out = self.run(f"Get_DataSigs ({ray})", [self.py, RX / "Get_DataSigs_QAM.py", *prm,
                                                    "--grid", qam_in, "--out", qam])
        if rc:
            return {"error": "Get_DataSigs"}
        rc, out = self.run(f"Demap ({ray})", [self.py, RX / "Get_Demapped_Bitstream.py", *prm,
                                             "--input", qam, "--out", bits, *demap_args])
        if rc:
            return {"error": "Demap"}
        cmp = ["--compare", compare] if compare else []
        rc, out = self.run(f"Decode ({ray})", [self.py, RX / "Get_Decoded_Bitstream.py", *prm,
                                              "--input", bits, "--out", blocks, *cmp])
        p = re.search(r"CRC passed for (\d+) of (\d+)", out)
        res = {"crc": f"{p.group(1)}/{p.group(2)}" if p else None,
               "all_pass": bool(p and p.group(1) == p.group(2)),
               "blocks": " ".join(("P" if s == "PASS" else "F") + c for _, s, c in
                                  re.findall(r"Block (\d+): CRC (PASS|FAIL) \((\d+) bit", out))}
        m = re.search(r"MISMATCH: (\d+) of (\d+) bits differ", out)
        res["bit_errors"] = m.group(1) if m else ("0" if "MATCH:" in out else None)
        if p is None and not self.dry:
            res["error"] = "Decode"
        return res

    def defects_waveform(self) -> Path:
        """Where Decimate_Payload_Rx.py looks for the defects waveform (Rx/, else Tx/)."""
        name = self.params.get("File_Rx", {}).get("Waveform_Payload_DefectsModeled_Rx",
                                                   "Waveform_Payload_DefectsModeled_Tx.h5")
        return RX / name if (RX / name).is_file() else TX / name

    def model_defects(self, defects_file: Path, when: str) -> tuple[bool, str]:
        """Step 0: rerun Tx/Defect_Model.py if needed. Returns (ok, message)."""
        cfg = json.loads(Path(defects_file).read_text())
        if not any(int(v) for v in cfg.get("Mode", {}).values()) or when == "never":
            return True, ""
        wave = self.defects_waveform()
        reason = ""
        if when == "always":
            reason = "--new-channel"
        elif not wave.is_file():
            reason = f"{wave.name} does not exist yet"
        elif wave.stat().st_mtime < Path(defects_file).stat().st_mtime:
            reason = f"{Path(defects_file).name} was changed after {wave.name} was made"
        if not reason:
            return True, f"Using {wave.name} (newer than {Path(defects_file).name})"
        rc, out = self.run("Defect_Model", [self.py, TX / "Defect_Model.py", "--defects", defects_file])
        return rc == 0, f"Ran Tx/Defect_Model.py ({reason})"

    def run_all(self, defects_file: Path, wave_input: Path | None = None, eq_args=(), demap_args=(),
                compare: Path | None = COMPARE_FILE, per_ray: bool = False,
                model: str = "never") -> dict[str, Any]:
        self.out.mkdir(parents=True, exist_ok=True)
        prm = ["--params", self.params_path]
        res: dict[str, Any] = {"rays": {}, "status": 0}
        if wave_input is None:
            ok, msg = self.model_defects(defects_file, model)
            res["model_msg"] = msg
            if not ok:
                res.update(status=3, error="Defect_Model")
                return res

        dec = self.path("dec", None)
        rc, out = self.run("Decimate", [self.py, RX / "Decimate_Payload_Rx.py", *prm,
                                        "--defects-file", defects_file,
                                        *(["--input", wave_input] if wave_input else []), "--output", dec])
        if rc:
            res.update(status=3, error="Decimate")
            return res
        if self.dry:
            n = int(self.params.get("Defined_Rx", {}).get("NumberOfAntennas", 1))
        else:
            m = re.search(r"x (\d+) waveform", out)
            n = int(m.group(1)) if m else 1
        res["antennas"] = n

        # Step 4 on every antenna
        found = []
        for r in range(1, n + 1):
            info: dict[str, Any] = {}
            res["rays"][r] = info
            rc, out = self.run(f"Packet_Detector (ray {r})", [self.py, RX / "Packet_Detector.py", *prm,
                                                             "--input", dec, "--output", self.path("pkt", r),
                                                             "--ray", r])
            info["cfo_khz"] = find(r"total ([+-]?[\d.]+) kHz", out)
            if rc == 2:
                info["error"] = "no packet"
            elif rc:
                info["error"] = "Packet_Detector"
            else:
                found.append(r)
        if not found:
            res["status"] = 2 if all(v.get("error") == "no packet" for v in res["rays"].values()) else 3
            return res

        # Step 4a: the control field, from every antenna that found the packet
        if str(self.params.get("Defined_Rx", {}).get("Use_ControlField", 1)).strip() == "1":
            rc, out = self.run("Control field", [self.py, RX / "Process_Control_Field_Rx.py", *prm,
                                                 "--inputs", *[self.path("pkt", r) for r in found],
                                                 "--out", self.path("control", None), *eq_args])
            res["control"] = {"crc": find(r"control field CRC (\w+)", out),
                              "changed": "Initialize_Rx.py done" in out,
                              "changes": re.findall(r"Defined_Rx/(\w+): \S+ -> \S+", out)}
            if rc == 4:
                res.update(status=4, error="Control field (bad packet)")
                return res
            if rc:
                res.update(status=3, error="Control field")
                return res

        # Steps 5-6 on every antenna that found the packet
        ok_rays = []
        for r in found:
            info = res["rays"][r]
            pkt, grid, eq = self.path("pkt", r), self.path("grid", r), self.path("eq", r)
            rc, out = self.run(f"Demodulate (ray {r})", [self.py, RX / "Demodulate_Waveform_Rx.py", *prm,
                                                        "--input", pkt, "--output", grid])
            if rc:
                info["error"] = "Demodulate"
                continue
            rc, out = self.run(f"Equalize (ray {r})", [self.py, RX / "Equalize_Payload_Grid.py", *prm,
                                                      "--grid", grid, "--out", eq, *eq_args])
            info["snr_db"] = find(r"SNR ~ ([-\d.]+) dB", out)
            if rc:
                info["error"] = "Equalize"
                continue
            ok_rays.append(r)
            if per_ray and n > 1:
                info.update(self.tail(r, eq, demap_args, compare))

        if not ok_rays:
            res["status"] = 3
            return res

        if n > 1:
            comb = self.path("eq", "combined")
            rc, out = self.run("Combine", [self.py, RX / "Combine_Antennas_Rx.py", *prm,
                                           "--inputs", *[self.path("eq", r) for r in ok_rays], "--out", comb])
            if rc:
                res.update(status=3, error="Combine")
                return res
            res["combined_snr_db"] = find(r"MRC of \d+ antenna\(s\): median per-signal SNR ([-\d.]+) dB", out)
            res["used"] = ok_rays
            final = self.tail("combined", comb, demap_args, compare)
        else:
            final = self.tail(1, self.path("eq", 1), demap_args, compare)
        res.update(final)
        if "error" in final:
            res["status"] = 3
        elif not final.get("all_pass") and not self.dry:
            res["status"] = 1
        return res


def summary_lines(res: dict[str, Any]) -> list[str]:
    lines = []
    if "control" in res:
        c = res["control"]
        line = f"  control field: CRC {c.get('crc') or '-'}"
        if c.get("changes"):
            line += f", Defined_Rx updated ({', '.join(c['changes'])})" \
                    + (", Initialize_Rx.py run" if c.get("changed") else "")
        elif c.get("crc") == "PASS":
            line += ", Defined_Rx unchanged"
        lines.append(line)
    for r, i in res.get("rays", {}).items():
        line = f"  ray {r}: CFO {i.get('cfo_khz') or '-':>8} kHz  EQ SNR {i.get('snr_db') or '-':>5} dB"
        if i.get("error"):
            line += f"  stopped: {i['error']}"
        elif i.get("crc"):
            line += f"  alone: CRC {i['crc']}  blocks {i.get('blocks', '')}  bit errors {i.get('bit_errors')}"
        lines.append(line)
    if res.get("error") and res.get("status") in (3, 4) and "crc" not in res:
        lines.append(f"  stopped at {res['error']}")
    elif "crc" in res:
        what = (f"combined ({len(res.get('used', []))} of {res.get('antennas')} antennas, "
                f"median SNR {res.get('combined_snr_db')} dB)" if res.get("antennas", 1) > 1 else "result")
        lines.append(f"  {what}: CRC {res['crc']}  blocks {res.get('blocks', '')}  "
                     f"bit errors {res.get('bit_errors')}")
    return lines


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Run Rx steps 3 to 9 for one or several antennas.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--defects-file", type=Path, default=DEFECTS_FILE,
                    help="Channel_Defects.json whose Mode flags decide the input of step 3")
    ap.add_argument("--input", type=Path, help="waveform for step 3 (default: from the Rx json)")
    ap.add_argument("--out-dir", type=Path, default=RX, help="folder for every file (default: Rx/)")
    ap.add_argument("--eq-mode", help="override Defined_Rx/EqualizerMode")
    ap.add_argument("--max-doppler", type=float, help="override Defined_Rx/EqualizerMaxDoppler (Hz)")
    ap.add_argument("--demap-noise-var", type=float, help="fixed demapper noise variance (testing)")
    ap.add_argument("--compare", type=Path, nargs="?", const=COMPARE_FILE, default=COMPARE_FILE,
                    help="Tx transport blocks to compare with (default: the Tx packet)")
    ap.add_argument("--no-compare", action="store_true", help="skip the comparison with the Tx blocks")
    ap.add_argument("--per-ray", action="store_true", help="also decode every antenna on its own")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--new-channel", action="store_true",
                   help="always rerun Tx/Defect_Model.py first (a new random channel unless RandomSeed is set)")
    g.add_argument("--no-model", action="store_true",
                   help="never rerun Tx/Defect_Model.py, even if Channel_Defects.json is newer")
    ap.add_argument("--log", type=Path, help="write every step's output to this file")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every step's output")
    ap.add_argument("--dry-run", action="store_true", help="print the commands without running them")
    a = ap.parse_args(argv)

    eq_args = (["--mode", a.eq_mode] if a.eq_mode else []) + \
              (["--max-doppler", a.max_doppler] if a.max_doppler else [])
    demap_args = ["--noise-var", a.demap_noise_var] if a.demap_noise_var else []
    compare = None if a.no_compare or (a.compare and not a.compare.is_file() and not a.dry_run) else a.compare

    chain = Chain(a.params, a.out_dir, a.dry_run, a.verbose)
    d_cfg = json.loads(a.defects_file.read_text())
    on = [k for k, v in d_cfg.get("Mode", {}).items() if int(v)]
    st = d_cfg.get("Settings", {})
    n_json = chain.params.get("Defined_Rx", {}).get("NumberOfAntennas", 1)
    print(f"{a.params.name}: NumberOfAntennas {n_json}, EqualizerMode "
          f"{a.eq_mode or chain.params.get('Defined_Rx', {}).get('EqualizerMode')}")
    if on:
        print(f"{a.defects_file.name}: {', '.join(on)}; SNR "
              f"{[st.get(f'SNR{k}_dB') for k in range(1, 5)]} dB, NumberOfRays {st.get('NumberOfRays')}, "
              f"{st.get('MultipathModel')}, RandomSeed {st.get('RandomSeed')}")
    else:
        print(f"{a.defects_file.name}: no defects (single waveform, NumberOfAntennas not used)")
    model = "always" if a.new_channel else "never" if (a.no_model or a.input) else "auto"
    res = chain.run_all(a.defects_file, a.input, eq_args, demap_args, compare, a.per_ray, model)
    if res.get("model_msg") and not a.dry_run:
        print(res["model_msg"])
    if a.log:
        a.log.write_text("\n".join(chain.log))
    if not a.dry_run:
        print(f"Rx chain, {res.get('antennas', '?')} antenna(s), files in {a.out_dir}")
        for line in summary_lines(res):
            print(line)
        if res["status"] == 3 and not a.verbose:
            print("A step stopped with an error - rerun with -v or --log to see its output")
    return res["status"]


if __name__ == "__main__":
    sys.exit(main())
