#!/usr/bin/env python3
"""
Diagnose_Defects.py  (lives in Flexlink_py/Rx)

Finds which channel defect hurts decoding, and how much the receive antennas help.
For each case it runs Tx/Defect_Model.py and the Rx chain (Run_Rx_Chain.py: steps 3
to 9, per antenna and combined) and reports the frequency offset found, the equalizer
SNR per antenna, the SNR after combining, the CRC result per block and the bit errors
against the Tx transport blocks.

Cases (default): no defects, each defect alone, all defects. The Settings
(SNR, MultipathModel, FrequencyOffset, ...) are taken from
Flexlink_BasicModules/Channel_Defects/Channel_Defects.json; only the Mode flags
change from case to case. The number of antennas is Defined_Rx/NumberOfAntennas from
the Rx json, or each value of --antennas. Channel_Defects.json and the Rx json are NOT
modified: every case uses temporary copies, and all intermediate files are written to a
temporary folder, so the files in Tx/ and Rx/ are left as they are. One channel
realization (seed) is shared by all variants of a case, so they compare like for like.

Usage:
    python Rx/Diagnose_Defects.py
    python Rx/Diagnose_Defects.py --all-only --seeds 50 --antennas 1 2 4
                                                         # diversity gain on the full setup
    python Rx/Diagnose_Defects.py --all-only --seeds 20 --antennas 4 --per-ray
                                                         # each antenna alone next to the combined result
    python Rx/Diagnose_Defects.py --seeds 3              # 3 random channels per case
    python Rx/Diagnose_Defects.py --cases Multipath TimingDrift
    python Rx/Diagnose_Defects.py --all-but              # all defects minus one at a time
    python Rx/Diagnose_Defects.py --keep my_diag_folder  # keep the intermediate files and logs
    python Rx/Diagnose_Defects.py --dry-run              # print the commands only
    python Rx/Diagnose_Defects.py --cases Multipath --seeds 20 --max-doppler 300 600
    python Rx/Diagnose_Defects.py --cases Multipath --seed-list 14 17 --no-baselines --eq-mode LMMSE PCA
    python Rx/Diagnose_Defects.py --cases Multipath --seed-list 14 --no-baselines --demap-noise-var 0.001
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

RX = Path(__file__).resolve().parent
ROOT = RX.parent
TX = ROOT / "Tx"
sys.path.insert(0, str(RX))
from Run_Rx_Chain import COMPARE_FILE, PARAMS_FILE, Chain  # noqa: E402

DEFECTS_FILE = ROOT / "Flexlink_BasicModules" / "Channel_Defects" / "Channel_Defects.json"
MODE_ORDER = ("Multipath", "TimingOffset", "TimingDrift", "Freq_Offset",
              "PhaseNoise", "ThermalNoise", "IQ_Imbalance")


def make_waveform(d: Path, modes: list[str], base: dict, seed: int, rays: int, dry: bool) -> tuple[Path, Path | None]:
    """Channel_Defects.json copy for the case and, with defects, the Defect_Model.py waveform."""
    d.mkdir(parents=True, exist_ok=True)
    cfg = json.loads(json.dumps(base))
    cfg["Mode"] = {k: (1 if k in modes else 0) for k in cfg["Mode"]}
    cfg.setdefault("Settings", {})["NumberOfRays"] = rays
    defects_json = d / "Channel_Defects.json"
    defects_json.write_text(json.dumps(cfg, indent=2))
    if not modes:
        return defects_json, None                      # Decimate reads File_Rx.Waveform_Payload_Rx
    wave = d / "Waveform_DefectsModeled.h5"
    if not wave.is_file() or dry:
        cmd = [sys.executable, TX / "Defect_Model.py", "--defects", defects_json, "--output", wave,
               "--seed", seed]
        if dry:
            print("   ", " ".join(str(c) for c in cmd))
        else:
            p = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
            (d / "Defect_Model.log").write_text(p.stdout + p.stderr)
            if p.returncode:
                raise RuntimeError(f"Defect_Model.py failed (see {d / 'Defect_Model.log'})")
    return defects_json, wave


def describe(res: dict, n_ant: int) -> str:
    rays = res.get("rays", {})
    r1 = rays.get(1, {})
    if n_ant <= 1:
        head = f"CFO {r1.get('cfo_khz') or '-':>8} kHz  EQ SNR {r1.get('snr_db') or '-':>5} dB"
    else:
        snrs = "/".join((v.get("snr_db") or "x") if not v.get("error") else "x" for v in rays.values())
        head = f"EQ SNR {snrs} dB  MRC {res.get('combined_snr_db') or '-'} dB"
    if res.get("status") == 3 and "crc" not in res:
        return head + f"  stopped at {res.get('error') or 'a step'} (see log)"
    if res.get("status") == 2:
        return head + "  no packet on any antenna"
    tail = (f"  CRC {res.get('crc') or '-':>4}  blocks {res.get('blocks') or '-':<20}  "
            f"bit errors {res.get('bit_errors') or '-'}")
    alone = [f"r{r}:{v['crc']}" for r, v in rays.items() if v.get("crc")]
    lost = [f"r{r}:{v['error']}" for r, v in rays.items() if v.get("error")]
    extra = ("  alone " + " ".join(alone) if alone else "") + ("  " + " ".join(lost) if lost else "")
    return head + tail + extra


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Find which channel defect hurts decoding.")
    ap.add_argument("--defects", type=Path, default=DEFECTS_FILE,
                    help="Channel_Defects.json to take the Settings from")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE, help="Rx json to start from")
    ap.add_argument("--cases", nargs="+", choices=MODE_ORDER,
                    help="defects to test alone (default: all of them)")
    ap.add_argument("--all-but", action="store_true",
                    help="also run all defects with one left out at a time")
    ap.add_argument("--seeds", type=int, default=1, help="random channels per case: seeds 1..N (default 1)")
    ap.add_argument("--seed-list", type=int, nargs="+", help="run exactly these seeds instead of 1..N")
    ap.add_argument("--no-baselines", action="store_true", help="skip the None and All cases")
    ap.add_argument("--all-only", action="store_true", help="run only the All case (every defect on)")
    ap.add_argument("--antennas", type=int, nargs="+", choices=(1, 2, 3, 4),
                    help="NumberOfAntennas value(s) to compare (default: the Rx json)")
    ap.add_argument("--per-ray", action="store_true", help="also decode every antenna on its own")
    ap.add_argument("--demap-noise-var", type=float, nargs="+",
                    help="fixed demapper noise variance(s) to compare (Get_Demapped_Bitstream.py --noise-var)")
    ap.add_argument("--max-doppler", type=float, nargs="+",
                    help="equalizer Doppler design(s) in Hz to compare (default: Defined_Rx/EqualizerMaxDoppler)")
    ap.add_argument("--eq-mode", nargs="+",
                    help="EqualizerMode(s) to compare, e.g. LMMSE PCA (default: Defined_Rx/EqualizerMode)")
    ap.add_argument("--keep", type=Path, help="folder to keep the intermediate files and logs in")
    ap.add_argument("--dry-run", action="store_true", help="print the commands without running them")
    a = ap.parse_args(argv)

    base = json.loads(a.defects.read_text())
    rx_base = json.loads(a.params.read_text())
    modes = [m for m in MODE_ORDER if m in base.get("Mode", {})]
    single = a.cases or modes
    cases = [(m, [m]) for m in single]
    if a.all_only:
        cases = [("All", modes)]
    elif not a.no_baselines:
        cases = [("None", [])] + cases + [("All", modes)]
    if a.all_but:
        cases += [(f"All-{m}", [x for x in modes if x != m]) for m in modes]

    antennas = a.antennas or [int(rx_base.get("Defined_Rx", {}).get("NumberOfAntennas", 1))]
    rays_needed = max(antennas)

    s = base.get("Settings", {})
    print(f"Settings from {a.defects}: SNR {[s.get(f'SNR{k}_dB') for k in range(1, 5)]} dB "
          f"({s.get('SNR_Reference', 'Transmit')}), {s.get('MultipathModel')}, Doppler {s.get('DopplerFreq')} Hz, "
          f"K {s.get('KFactor')}, CFO {s.get('FrequencyOffset')} Hz, offset {s.get('Sample_Offset')}, "
          f"drift {s.get('Drift_ppm')} ppm; antennas {antennas}")

    variants = []      # (n_ant, eq_args, demap_args, tag)
    for n_ant in antennas:
        for md in (a.max_doppler or [None]):
            for em in (a.eq_mode or [None]):
                for nv in [None] + (a.demap_noise_var or []):
                    eq_args, label = [], []
                    if len(antennas) > 1 or n_ant > 1:
                        label.append(f"A{n_ant}")
                    if md is not None:
                        eq_args += ["--max-doppler", md]; label.append(f"{md:g}Hz")
                    if em is not None:
                        eq_args += ["--mode", em]; label.append(em)
                    demap_args = [] if nv is None else ["--noise-var", nv]
                    if nv is not None:
                        label.append(f"nv{nv:g}")
                    variants.append((n_ant, eq_args, demap_args, ("_" + "_".join(label)) if label else ""))
    seeds = a.seed_list or list(range(1, a.seeds + 1))
    compare = COMPARE_FILE if COMPARE_FILE.is_file() or a.dry_run else None

    work = a.keep or Path(tempfile.mkdtemp(prefix="flexlink_diag_"))
    try:
        for name, m in cases:
            for n_ant, eq_args, demap_args, tag in variants:
                if not m and n_ant != antennas[0]:
                    continue                           # no defects: always one waveform, run once
                passed = 0
                for seed in seeds:
                    if a.dry_run:
                        print(f"-- case {name}{tag}, seed {seed}")
                    cdir = work / f"{name}_s{seed}"
                    try:
                        defects_json, wave = make_waveform(cdir, m, base, seed, rays_needed, a.dry_run)
                    except RuntimeError as exc:
                        print(f"{(name + tag):<28} seed {seed:<3} {exc}", flush=True)
                        continue
                    vdir = cdir / (tag.strip("_") or "default")
                    vdir.mkdir(parents=True, exist_ok=True)
                    rx = json.loads(json.dumps(rx_base))
                    rx.setdefault("Defined_Rx", {})["NumberOfAntennas"] = n_ant
                    rx_json = vdir / "Parameters_Rx_payload.json"
                    rx_json.write_text(json.dumps(rx, indent=2))
                    chain = Chain(rx_json, vdir, a.dry_run)
                    res = chain.run_all(defects_json, wave, eq_args, demap_args, compare, a.per_ray)
                    (vdir / "log.txt").write_text("\n".join(chain.log))
                    if a.dry_run:
                        continue
                    passed += int(bool(res.get("all_pass")))
                    print(f"{(name + tag):<28} seed {seed:<3} {describe(res, res.get('antennas', 1))}", flush=True)
                if len(seeds) > 1 and not a.dry_run:
                    print(f"{'':<28} -> {passed} of {len(seeds)} packets fully decoded", flush=True)
    finally:
        if a.keep:
            print(f"Intermediate files and logs kept in {work}")
        else:
            shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
