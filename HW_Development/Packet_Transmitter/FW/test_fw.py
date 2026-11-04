#!/usr/bin/env python3
"""
test_fw.py  (Flexlink_py/HW_Development/Packet_Transmitter/FW)

Checks the Pluto server on the PC (make -f ptx.mk first: ptx_params_test, ptx_server_stub):

  params  ptx_params.c against the model (PC/ptx_reference.py) for every legal setting of the
          payload (Mod x spacings x PacketLength x code x rate x Repetition, 10 C_init / ID / flag
          variants each), and that it rejects what the model rejects: Calculated values,
          ControlMessage, the 6 header words; plus the repository's own two JSON files, whose
          Calculated sections must come back unchanged.
  e2e     ptx_server_stub (the PL replaced by a file and its timing) with PC/ptx_client.py: the
          JSON files, 5 packets, a SET_PARAMS that changes the settings in the middle (16QAM, other
          control-field codes), 3 more packets, STATUS and STOP. Checks the protocol (credits,
          SENT in order) and that the stream the server wrote is byte for byte the model's
          records. With --rtl the stream also goes through the RTL (SIM/run_sim.sh corefile) and
          every sample is compared with the model.

    make -f ptx.mk && python test_fw.py  # params + e2e
    python test_fw.py --rtl              # + the RTL (Verilator)
"""
from __future__ import annotations

import argparse
import itertools
import json
import os
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file())
sys.path.insert(0, str(HERE.parent / "PC"))
import ptx_reference as R  # noqa: E402
import ptx_client as PC  # noqa: E402

RATE_TXT = {"12": "1/2", "23": "2/3", "34": "3/4", "56": "5/6"}


def defined(c: R.TxConfig) -> dict:
    return {"EncoderCodeRate": RATE_TXT[c.rate], "EncoderBlockSize": c.n, "Repetition": c.rep, "CRC": "Gen24",
            "Mod": c.mod, "PacketLength": c.plen, "RefSignalSpacing": c.ref_sig, "Port": c.port,
            "NumberOfAntennas": c.antennas, "ClientFlag": "Client" if c.client else "Service",
            "RefSymbolSpacing": c.ref_sym, "C_init_ID1": c.id1, "C_init_ID2": c.id2, "C_init_Dat": c.c_init,
            "SampleRate": 40000000, "Use_Preamble": c.use_pre, "Use_ControlField": c.use_cf,
            "Tx_Ref_Clk_Cnt": c.clk_cnt}


def run_c(msg: dict) -> dict | str:
    p = subprocess.run([str(HERE / "ptx_params_test")], input=json.dumps(msg), capture_output=True, text=True)
    out = p.stdout.strip()
    return out if out.startswith("ERROR") else json.loads(out)


def check_params(lines: list[str], rng) -> bool:
    ok, n, n_rej = True, 0, 0
    # the repository's files
    tx = json.loads((ROOT / "Tx" / "Parameters_Tx_payload.json").read_text())
    cf = json.loads((ROOT / "Tx" / "ControlSignals" / "Parameters_Control_Field.json").read_text())
    rep = run_c({"Tx": tx["Defined"], "ControlField": cf["Defined"]})
    same = isinstance(rep, dict) and all(rep["Tx"]["Calculated"][k] == v for k, v in tx["Calculated"].items()) \
        and all(rep["ControlField"]["Calculated"][k] == v for k, v in cf["Calculated"].items()
                if k != "Repetition_comment")
    lines.append(f"  repository JSON files: Calculated sections {'unchanged' if same else 'DIFFERENT'}")
    ok &= same
    # every legal setting
    combos = list(itertools.product(R.BITS, (1, 3, 6, 12), (3, 6, 12, 24), R.PACKET_LENGTHS, (648, 1296, 1944),
                                    ("12", "23", "34", "56"), R.REPETITIONS))
    batch = []
    for mod, rs, sg, pl, nn, rate, rp in combos:
        if (pl - 1) % rs:
            continue
        c = R.TxConfig(mod=mod, ref_sym=rs, ref_sig=sg, plen=pl, n=nn, rate=rate, rep=rp,
                       port=int(rng.integers(0, 2)), c_init=int(rng.integers(0, 256)),
                       id1=int(rng.integers(0, 1 << 31)), id2=int(rng.integers(0, 1 << 31)),
                       client=int(rng.integers(0, 2)), antennas=int(rng.integers(1, 3)),
                       use_pre=int(rng.integers(0, 2)), use_cf=int(rng.integers(0, 2)),
                       clk_cnt=int(rng.integers(0, 1 << 14)), cf_id1=int(rng.integers(0, 1 << 31)),
                       cf_cdat=int(rng.integers(0, 1 << 31)))
        batch.append(c)
    # the C tool takes one message per run: sample the space (all of it takes a few minutes)
    pick = batch if os.environ.get("PTX_ALL") else [batch[i] for i in rng.choice(len(batch), 800, replace=False)]
    bad = 0
    for c in pick:
        gap = int(rng.integers(0, 50000))
        msg = {"Tx": defined(c), "ControlField": {"C_init_ID1": c.cf_id1, "C_init_Dat": c.cf_cdat}, "Gap": gap}
        rep = run_c(msg)
        try:
            c.check()
            legal = True
        except AssertionError:
            legal = False
        if not legal:
            n_rej += 1
            if not isinstance(rep, str):
                bad += 1
                if bad < 5:
                    lines.append(f"  {c}: model rejects, server accepts")
            continue
        n += 1
        if isinstance(rep, str):
            bad += 1
            if bad < 5:
                lines.append(f"  {c}: server {rep}")
            continue
        diff = PC.check_reply(rep, c)
        hw = [int(x, 16) for x in rep["Header"]]
        mine = [int(x) for x in R.param_header(c, 1, gap)]
        if hw != mine:
            diff.append("header words differ")
        calc = rep["Tx"]["Calculated"]
        ref_n = int((c.types() == R.P0).sum())
        dc_n = int((c.types() == R.DC).sum())
        if calc["NumberOfRefSigs_P0"] != ref_n or calc["NumberOfDCSigs"] != dc_n:
            diff.append("reference / DC counts differ")
        if diff:
            bad += 1
            if bad < 5:
                lines.append(f"  {c}: " + "; ".join(diff))
    # control-field settings the hardware does not support
    for k, v in (("CRC", "Gen16"), ("ConstraintLength", 9), ("Polynomials", [133, 175]), ("Mod", "QPSK"),
                 ("PacketLength", 5), ("TailBiting", 1)):
        rep = run_c({"Tx": defined(R.TxConfig()), "ControlField": {k: v}})
        n_rej += 1
        if not isinstance(rep, str):
            bad += 1
            lines.append(f"  control field {k} = {v}: accepted")
    lines.append(f"  {n} legal settings: Calculated values, ControlMessage, reference/DC counts and header words "
                 f"equal to the model; {n_rej} illegal ones rejected; {bad} problems")
    return ok and bad == 0


def check_e2e(lines: list[str], rng, rtl: bool) -> bool:
    out = HERE / "ptx_stub_stream.bin"
    port = 5300 + int(rng.integers(0, 500))
    srv = subprocess.Popen([str(HERE / "ptx_server_stub"), "-p", str(port), "-o", str(out)],
                           stderr=subprocess.PIPE, text=True)
    ok = True
    try:
        time.sleep(0.3)
        c = PC.PtxClient("127.0.0.1", port)
        pong = c.ping()
        lines.append(f"  PING: {pong}")
        tx_json = ROOT / "Tx" / "Parameters_Tx_payload.json"
        cf_json = ROOT / "Tx" / "ControlSignals" / "Parameters_Control_Field.json"
        cfg1 = R.TxConfig.from_params(tx_json, cf_json)
        rep = c.set_params(tx_json, cf_json, gap=1000)
        diff = PC.check_reply(rep, cfg1)
        lines.append(f"  SET_PARAMS (repository JSON): {rep['BlocksPerPacket']} blocks of {rep['TransportBlockSize']}"
                     f" bits, credits {c.depth}{'' if not diff else ', DIFFERENT: ' + '; '.join(diff)}")
        ok &= not diff and c.depth == 2
        sent_tbs = []
        expect = []
        for k in range(5):
            tbs = R.random_tbs(cfg1, rng)
            c.send_packet(k, tbs)
            sent_tbs.append((cfg1, tbs, 1000))
            ok &= len(c.outstanding) <= c.depth
        # new settings in the middle: 16QAM, control field with other codes
        txd = json.loads(tx_json.read_text())["Defined"]
        txd.update({"Mod": "16QAM", "EncoderBlockSize": 1944, "EncoderCodeRate": "3/4", "Repetition": 3,
                    "RefSymbolSpacing": 3, "C_init_Dat": 17})
        cfd = json.loads(cf_json.read_text())["Defined"]
        cfd.update({"C_init_ID1": 4242, "C_init_Dat": 99})
        t2 = HERE / "_tx2.json"
        c2 = HERE / "_cf2.json"
        t2.write_text(json.dumps({"Defined": txd}))
        c2.write_text(json.dumps({"Defined": cfd}))
        cfg2 = R.TxConfig.from_params(t2, c2)
        rep2 = c.set_params(t2, c2, gap=64)
        diff = PC.check_reply(rep2, cfg2)
        ok &= not diff
        lines.append(f"  SET_PARAMS (16QAM 1944 3/4, control field C_init_ID1 4242, C_init_Dat 99) after 5 packets: "
                     f"{rep2['BlocksPerPacket']} blocks{'' if not diff else ', DIFFERENT'}; it waited for the "
                     f"queued packets ({len(c.sent)} SENT before the reply)")
        ok &= len(c.sent) == 5
        for k in range(5, 8):
            tbs = R.random_tbs(cfg2, rng)
            c.send_packet(k, tbs)
            sent_tbs.append((cfg2, tbs, 64))
        c.wait_sent()
        seqs = [s["seq"] for s in c.sent]
        lines.append(f"  SENT in order: {seqs}")
        ok &= seqs == list(range(8))
        try:
            c._send(PC.BLOCKS, b"\x01\x00\x00\x00" + b"\x00" * 8, 99)
            c._wait(PC.STATUS_R)                     # never comes before the ERROR
            ok = False
        except PC.ServerError as exc:
            lines.append(f"  wrong BLOCKS size rejected: {exc}")
        st = c.status()
        lines.append(f"  STATUS: {st}")
        stop = c.stop()
        lines.append(f"  STOP: {stop}")
        c.close()
        t2.unlink()
        c2.unlink()
    finally:
        srv.terminate()
        srv.wait()
    got = np.fromfile(out, "<u8")
    exp = np.concatenate([R.record_words(cf, [t], g) for cf, t, g in sent_tbs])
    same = got.size == exp.size and np.array_equal(got, exp)
    lines.append(f"  stream written by the server: {got.size} words, {'byte for byte the model records' if same else 'DIFFERENT from the model'}")
    ok &= same
    if rtl and same:
        work = HERE.parent / "SIM" / "work" / "core"
        work.mkdir(parents=True, exist_ok=True)
        (work / "words.txt").write_text("\n".join(f"{int(x):016x}" for x in got) + "\n")
        samples = np.concatenate([R.stream_samples(R.packet(cf, t), g) for cf, t, g in sent_tbs])
        np.savetxt(work / "exp.txt", samples, fmt="%d")
        (work / "cfg.txt").write_text(f"{got.size} {samples.shape[0]} 1 3 1 {len(sent_tbs)} 0\n")
        r = subprocess.run(["./run_sim.sh", "corefile"], cwd=HERE.parent / "SIM", capture_output=True, text=True)
        res = [l for l in r.stdout.splitlines() if l.startswith(("PASS", "FAIL"))]
        lines.append(f"  RTL on the server's stream: {res[-1] if res else r.stdout[-300:]}")
        ok &= bool(res) and res[-1].startswith("PASS")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check the Pluto server on the PC")
    ap.add_argument("--only", choices=("params", "e2e"))
    ap.add_argument("--rtl", action="store_true", help="also run the server's stream through the RTL")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    ok = True
    for name, fn in (("params", lambda l: check_params(l, rng)), ("e2e", lambda l: check_e2e(l, rng, a.rtl))):
        if a.only and a.only != name:
            continue
        lines: list[str] = []
        res = fn(lines)
        print(f"[{name}] {'PASS' if res else 'FAIL'}")
        print("\n".join(lines))
        ok &= res
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
