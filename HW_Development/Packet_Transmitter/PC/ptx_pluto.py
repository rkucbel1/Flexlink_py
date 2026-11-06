#!/usr/bin/env python3
"""
ptx_pluto.py  (Flexlink_py/HW_Development/Packet_Transmitter/PC)

Flexlink packets over the air between two ADALM-Plutos (or one, TX looped to RX by cable), with
the Zedboard receiver at the end. Every packet comes from the bit-true transmitter model
(ptx_reference.py), so the stock-firmware waveform (--mode iq) and the PL transmitter's waveform
(--mode pl) are the same samples.

  tx        transmit packets cyclically until Ctrl-C, and write <run>.npz (settings and transport
            blocks of every packet, for the check)
              --mode iq   stock Pluto firmware: the 12-bit samples go into a cyclic TX buffer
                          (the DMA replays them from the Pluto's RAM; nothing crosses USB after
                          the first push)
              --mode pl   Pluto firmware with the Flexlink transmitter (HW/patch_pluto_hdl.sh)
                          and FW/ptx_server running on the Pluto: SET_PARAMS with the Defined
                          sections of both JSON files, then one packet of transport blocks per
                          credit, cycling the --packets packets until Ctrl-C (PC/ptx_client.py);
                          the PL encodes and modulates
  rx        capture one burst at 40 Msps (default 2^20 samples, 26 ms) and write a Zedboard test
            vector: <out>_40.bin (int16 I, Q, 12-bit values, as pd_make_vector writes for
            input_stage 2) and <out>.json with the bit-true model's expected events
  loopback  tx and rx on the same Pluto (TX1 to RX1 through a cable and 30-40 dB attenuation)
  check     decode a capture with the receiver's bit-true model (and with --board on the
            Zedboard) and compare every transport block with the ones in <run>.npz
  model     no radio: the transmitter model's samples (with a little noise) as a Zedboard vector,
            the digital loopback of the 40 Msps path

Examples
    python ptx_pluto.py tx --uri ip:192.168.2.1 --run run1 --packets 4 --seed 1
    python ptx_pluto.py rx --uri ip:192.168.3.1 --out cap1
    python ptx_pluto.py check cap1.json --run run1.npz
    python ptx_pluto.py check cap1.json --run run1.npz --board        # Zedboard at 192.168.50.10
    cd ../../Packet_Detector/PC && python pd_hw_test.py ../../Packet_Transmitter/PC/cap1.json --input-stage 2

Settings: --params Tx/Parameters_Tx_payload.json (default), or --mod/--n/--rate/... overrides.
Needs pyadi-iio (pip install pyadi-iio) for tx, rx and loopback; numpy only for model and check.

NOT YET RUN ON HARDWARE (written without a Pluto at hand): the pyadi calls follow pyadi-iio 0.0.17+.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
PD_PC = ROOT / "HW_Development" / "Packet_Detector" / "PC"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PD_PC))
import ptx_reference as R  # noqa: E402

FS = 40_000_000
# axi_ad9361 DAC GPIO registers (0x40BC / 0x40B8 in the core's map). cf-ad9361-dds-core-lpc counts
# register offsets from the TX (DAC) part, which starts at 0x4000.
DAC_GPIO_OUT = 0x00BC         # DAC GPIO out (bit 4: Flexlink enable)
DAC_GPIO_IN = 0x00B8          # status (RTL/ptx_pluto.v)
PTX_ENABLE = 1 << 4


# --------------------------------------------------------------------------------------------
# packets

def config(a) -> R.TxConfig:
    cfg = R.TxConfig.from_params(Path(a.params)) if a.params else R.TxConfig()   # + Parameters_Control_Field.json
    over = {k: v for k, v in (("mod", a.mod), ("n", a.n), ("rate", a.rate), ("rep", a.rep),
                              ("plen", a.plen), ("ref_sym", a.ref_sym), ("ref_sig", a.ref_sig),
                              ("c_init", a.c_init)) if v is not None}
    cfg = replace(cfg, **over)
    cfg.check()
    return cfg


def build(cfg: R.TxConfig, n_packets: int, seed: int, gap: int, backoff: float):
    rng = np.random.default_rng(seed)
    pkts = [R.packet(cfg, R.random_tbs(cfg, rng), backoff_db=backoff) for _ in range(n_packets)]
    return pkts


def save_run(path: Path, cfg: R.TxConfig, pkts, gap: int, mode: str, extra: dict) -> Path:
    path = path.with_suffix(".npz")
    meta = {"cfg": asdict(cfg), "gap": gap, "mode": mode, "gains": pkts[0].gains,
            "message": R.cf_message(cfg).tolist(), **extra}
    np.savez(path, tbs=np.array([p.bits.codewords[:, :cfg.tb_size] for p in pkts], np.uint8),
             meta=json.dumps(meta))
    return path


def iq_stream(pkts, gap: int) -> np.ndarray:
    """Samples of the cyclic buffer (iq mode): packet, gap, packet, gap, ... (12-bit, int16)."""
    parts = []
    for p in pkts:
        s = R.stream_samples(p, gap)
        parts.append(s[:, :2])
    return np.concatenate(parts).astype(np.int16)


def pl_stream(pkts, gap: int) -> np.ndarray:
    """The PL stream (pl mode): parameter header + the transport blocks of every packet, as int16
    pairs (what the DMA reads from the buffer)."""
    cfg = pkts[0].cfg
    words = R.record_words(cfg, [p.bits.codewords[:, :cfg.tb_size] for p in pkts], gap).astype("<u8")
    return words.view("<i2").reshape(-1, 2)


# --------------------------------------------------------------------------------------------
# radio (pyadi-iio)

def pluto(uri: str, lo: float, bw: float):
    try:
        import adi
    except ImportError:
        sys.exit("pyadi-iio is not installed: pip install pyadi-iio")
    sdr = adi.Pluto(uri)
    sdr.sample_rate = FS
    sdr.tx_lo = int(lo)
    sdr.rx_lo = int(lo)
    sdr.tx_rf_bandwidth = int(bw)
    sdr.rx_rf_bandwidth = int(bw)
    got = int(sdr.sample_rate)
    if got != FS:
        sys.exit(f"Pluto at {uri} runs at {got} sps, not {FS}")
    return sdr


def dac_gpio(sdr, enable: bool | None = None) -> int:
    dev = sdr._txdac
    v = int(dev.reg_read(DAC_GPIO_OUT))
    if enable is not None:
        v = (v | PTX_ENABLE) if enable else (v & ~PTX_ENABLE)
        dev.reg_write(DAC_GPIO_OUT, v)
    return v


def ptx_status(sdr) -> dict:
    s = int(sdr._txdac.reg_read(DAC_GPIO_IN))
    return {"packets": s & 0xFFFF, "header_errors": (s >> 16) & 0xFF, "underflows": (s >> 24) & 0xF,
            "busy": (s >> 28) & 1, "enabled": (s >> 29) & 1, "raw": f"{s:08x}"}


def start_tx(sdr, a, pkts) -> None:
    sdr.tx_hardwaregain_chan0 = float(a.tx_gain)
    sdr.tx_enabled_channels = [0]
    sdr.tx_cyclic_buffer = True
    if a.mode == "iq":
        try:
            dac_gpio(sdr, False)                # PL transmitter off, if this firmware has it
        except Exception:                       # noqa: BLE001
            pass
        s = iq_stream(pkts, a.gap).astype(np.int32) << 4        # 12-bit -> DAC MSB-aligned
        sdr.tx(s[:, 0] + 1j * s[:, 1])
        print(f"TX iq: {s.shape[0]} samples ({s.shape[0] / FS * 1e3:.2f} ms cycle) in a cyclic buffer")
    else:
        import threading
        import ptx_client
        host = a.uri.split(":", 1)[1] if a.uri.startswith("ip:") else a.uri
        cfg = pkts[0].cfg
        c = ptx_client.PtxClient(host, a.server_port)
        rep = c.set_defined(cfg.defined(), cfg.cf_defined(), a.gap,
                            radio={"TxLo": a.lo, "TxGain": a.tx_gain, "RfBandwidth": a.bw})
        diff = ptx_client.check_reply(rep, cfg)
        if diff:
            sys.exit("server and model disagree: " + "; ".join(diff))
        a._client, a._stop = c, threading.Event()

        def feed():
            seq = 0
            try:
                while not a._stop.is_set():
                    c.send_packet(seq, pkts[seq % len(pkts)].bits.codewords[:, :cfg.tb_size])
                    seq += 1
            except Exception as exc:                    # noqa: BLE001
                print(f"\n(feeding stopped: {exc})")
        a._thread = threading.Thread(target=feed, daemon=True)
        a._thread.start()
        print(f"TX pl: server at {host}:{a.server_port}, {rep['BlocksPerPacket']} blocks per packet, "
              f"{c.depth} credits")


def stop_tx(sdr, a) -> None:
    try:
        if a.mode == "pl":
            a._stop.set()
            a._thread.join(timeout=5)
            print(f"PL status: {a._client.stop()}")
            a._client.close()
        else:
            sdr.tx_destroy_buffer()
    except Exception as exc:                            # noqa: BLE001
        print(f"(stopping TX: {exc})")


def capture(sdr, a) -> np.ndarray:
    sdr.gain_control_mode_chan0 = "manual"
    sdr.rx_hardwaregain_chan0 = float(a.rx_gain)
    sdr.rx_enabled_channels = [0]
    sdr.rx_buffer_size = int(a.samples)
    for _ in range(2):                                  # stale buffers
        sdr.rx()
    x = sdr.rx()
    iq = np.stack([np.real(x), np.imag(x)], axis=1).astype(np.int16)
    peak = int(np.abs(iq).max())
    print(f"RX: {iq.shape[0]} samples, rms {np.sqrt(np.mean(iq.astype(float) ** 2) * 2):.0f}, peak {peak}"
          + ("  (CLIPPING: lower --rx-gain)" if peak >= 2047 else ""))
    return iq


# --------------------------------------------------------------------------------------------
# Zedboard vector (as pd_make_vector.py writes it, input_stage 2 only)

def write_vector(out: Path, iq: np.ndarray, source: str, extra: dict) -> Path:
    import pd_protocol as pr
    import pd_reference as ref
    iq = iq[: iq.shape[0] // 2 * 2]
    p = pr.Params(input_stage=2)
    res = ref.detect(iq.astype(np.int64), p, control=False)
    ev = [{**vars(e), **{k: v for k, v in x.items() if k != "t_trig"}} for e, x in zip(res.events, res.extra)]
    out.parent.mkdir(parents=True, exist_ok=True)
    b40 = out.with_name(out.name + "_40.bin")
    iq.astype("<i2").tofile(b40)
    meta = {"format": "int16 I,Q interleaved, little-endian, 12-bit values, 40 Msps (_40.bin)",
            "files": {"input40": b40.name}, "source": source, "samples": int(iq.shape[0] // 2),
            "samples_40": int(iq.shape[0]), "sample_rate": 20e6, "params": p.as_dict(),
            "expected_events": ev, "expected_events_s2": ev, "truth": [], "defects": {},
            "quantisation_40": {"backoff_db": None, "clipped_values": int(np.count_nonzero(np.abs(iq) >= 2047))},
            "ptx": extra}
    js = out.with_suffix(".json")
    js.write_text(json.dumps(meta, indent=2, default=lambda o: o.item() if isinstance(o, np.generic) else str(o)))
    print(f"Vector {js} + {b40.name}: {iq.shape[0]} samples, {len(ev)} detector event(s) in the model "
          f"(run: pd_hw_test.py {js.name} --input-stage 2)")
    return js


# --------------------------------------------------------------------------------------------
# check

def check(a) -> int:
    import pd_protocol as pr
    import pd_reference as ref
    meta = json.loads(Path(a.vector).with_suffix(".json").read_text())
    iq = np.fromfile(Path(a.vector).with_name(meta["files"]["input40"]), "<i2").reshape(-1, 2).astype(np.int64)
    run = np.load(Path(a.run).with_suffix(".npz"))
    rmeta = json.loads(str(run["meta"]))
    sent = run["tbs"].astype(np.int64)                  # (packets, ndb, tb_size)
    msg = int(sum(int(b) << i for i, b in enumerate(rmeta["message"])))
    p = pr.Params(input_stage=2)
    print(f"Capture {meta['samples_40']} samples; run {Path(a.run).name}: {sent.shape[0]} packet(s) of "
          f"{sent.shape[1]} block(s), settings {rmeta['cfg']}")

    def report(name, control, tbs):
        ok_pk = 0
        by_t = {}
        for r in tbs:
            by_t.setdefault(r.t_trig, {})[r.w] = r
        for c in control:
            line = f"  {name} packet at t_trig {c.t_trig}: control field "
            if c.status != 1 or c.message != msg:
                print(line + ("CRC FAIL" if c.status != 1 else "different from the Tx message"))
                continue
            got = by_t.get(c.t_trig, {})
            best = None
            for k in range(sent.shape[0]):
                eq = sum(1 for w in range(sent.shape[1]) if w in got and got[w].crc_ok
                         and np.array_equal(np.array(got[w].bits), sent[k, w]))
                if best is None or eq > best[1]:
                    best = (k, eq)
            crc = sum(1 for r in got.values() if r.crc_ok)
            good = best[1] == sent.shape[1]
            ok_pk += good
            print(line + f"ok, {len(got)} blocks, {crc} CRC ok, {best[1]}/{sent.shape[1]} equal to Tx packet "
                  f"{best[0]}{'' if good else '  FAIL'}")
        print(f"  {name}: {len(control)} control field(s), {ok_pk} packet(s) completely correct")
        return ok_pk

    res = ref.detect(iq, p)
    n_model = report("model", res.control, res.tb)
    n_board = None
    if a.board:
        sys.path.insert(0, str(ROOT / "App"))
        import Run_HIL
        info, events, timing, control, tbs = Run_HIL.run_board(iq.astype(np.int16), p, a.host, a.port,
                                                               a.timeout)
        n_board = report("board", sorted(control.values(), key=lambda r: r.t_trig), tbs)
        import pd_hw_test
        same, lines = pd_hw_test.compare_tb(tbs, res.tb)
        print("  board vs model transport blocks: " + ("bit for bit equal" if same else "DIFFERENT"))
        for line in lines[:10]:
            print("   " + line)
    ok = n_model > 0 and (n_board is None or n_board > 0)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


# --------------------------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Flexlink packets through ADALM-Pluto radios")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def radio_args(s, rx=True, tx=True):
        s.add_argument("--uri", default="ip:192.168.2.1")
        s.add_argument("--lo", type=float, default=2.4e9, help="carrier, Hz")
        s.add_argument("--bw", type=float, default=18e6, help="analog RF bandwidth, Hz (occupied 16.4 MHz)")
        if tx:
            s.add_argument("--tx-gain", type=float, default=-10.0, help="TX attenuation as gain, dB (<= 0)")
        if rx:
            s.add_argument("--rx-gain", type=float, default=30.0, help="manual RX gain, dB")
            s.add_argument("--samples", type=int, default=1 << 20, help="capture length at 40 Msps")

    def pkt_args(s):
        s.add_argument("--params", default=str(ROOT / "Tx" / "Parameters_Tx_payload.json"),
                       help="settings (Defined section); '' for the defaults")
        for k, t in (("--mod", str), ("--n", int), ("--rate", str), ("--rep", int), ("--plen", int),
                     ("--ref-sym", int), ("--ref-sig", int), ("--c-init", int)):
            s.add_argument(k, type=t)
        s.add_argument("--packets", type=int, default=4, help="different packets in the cycle")
        s.add_argument("--seed", type=int, default=1)
        s.add_argument("--gap", type=int, default=20000, help="idle samples after each packet")
        s.add_argument("--backoff", type=float, default=R.BACKOFF_DB, help="rms below DAC full scale, dB")
        s.add_argument("--mode", choices=("iq", "pl"), default="iq")
        s.add_argument("--server-port", type=int, default=5010, help="ptx_server port (--mode pl)")
        s.add_argument("--run", default="run", help="run file (.npz) with the transmitted blocks")

    s = sub.add_parser("tx"); radio_args(s, rx=False); pkt_args(s)
    s = sub.add_parser("rx"); radio_args(s, tx=False); s.add_argument("--out", default="capture")
    s = sub.add_parser("loopback"); radio_args(s); pkt_args(s); s.add_argument("--out", default="capture")
    s = sub.add_parser("model"); pkt_args(s); s.add_argument("--out", default="model_vector")
    s.add_argument("--noise", type=float, default=2.0, help="rms of the added noise per rail, LSB")
    s = sub.add_parser("check")
    s.add_argument("vector", help="capture .json (rx, loopback or model)")
    s.add_argument("--run", required=True, help="run .npz written by tx / loopback / model")
    s.add_argument("--board", action="store_true", help="also decode on the Zedboard")
    s.add_argument("--host", default="192.168.50.10")
    s.add_argument("--port", type=int, default=5001)
    s.add_argument("--timeout", type=float, default=30.0)
    a = ap.parse_args(argv)
    if hasattr(a, "ref_sym"):
        a.params = a.params or None

    if a.cmd == "check":
        return check(a)
    if a.cmd == "rx":
        sdr = pluto(a.uri, a.lo, a.bw)
        iq = capture(sdr, a)
        write_vector(Path(a.out), iq, f"Pluto {a.uri} capture", {"lo": a.lo, "rx_gain": a.rx_gain})
        return 0

    cfg = config(a)
    pkts = build(cfg, a.packets, a.seed, a.gap, a.backoff)
    cyc = sum(R.stream_samples(p, a.gap).shape[0] for p in pkts)
    print(f"{a.packets} packet(s) {cfg}: {cfg.ndb} blocks of {cfg.tb_size} bits each, "
          f"{pkts[0].i.size} samples + gap {a.gap}, cycle {cyc / FS * 1e3:.2f} ms, gains {pkts[0].gains}")
    run = save_run(Path(a.run), cfg, pkts, a.gap, a.mode, {"seed": a.seed})
    print(f"Run file {run}")

    if a.cmd == "model":
        rng = np.random.default_rng(a.seed + 1000)
        s = iq_stream(pkts, a.gap).astype(np.int64)
        s = np.concatenate([np.zeros((3000, 2), np.int64), s])
        noisy = np.clip(s + np.round(rng.normal(0, a.noise, s.shape)), -2048, 2047).astype(np.int16)
        write_vector(Path(a.out), noisy, "ptx_reference model (digital loopback)", {"run": run.name})
        return 0

    sdr = pluto(a.uri, a.lo, a.bw)
    start_tx(sdr, a, pkts)
    try:
        if a.cmd == "loopback":
            time.sleep(0.2)
            iq = capture(sdr, a)
            write_vector(Path(a.out), iq, f"Pluto {a.uri} loopback ({a.mode})", {"run": run.name, "lo": a.lo})
        else:
            print("Transmitting - Ctrl-C to stop")
            while True:
                time.sleep(1.0)
                if a.mode == "pl" and a._client.sent:
                    s = a._client.sent[-1]
                    print(f"\r  sent {s['sent']}, PL packets {s['pl_packets']}, underflows {s['underflows']}",
                          end="", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        stop_tx(sdr, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
