"""
fmc_protocol.py  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/PC)

The FMCOMMS3 additions to the board protocol. Everything else (header, magic, PING,
READ_SAMPLES, REG_READ / REG_WRITE, send_msg / recv_msg) is ../../Packet_Detector/PC/pd_protocol.py,
imported here unchanged. Keep in step with ../FW/fmc_server.c.

    RADIO_CONFIG 22  b""            -> RadioStatus (64 bytes)
                     RadioConfig    -> RadioStatus
    CAPTURE 23       u32 n_samples, u32 timeout_ms
                                    -> u32 n_samples, u32 flags, u32 time_us, u32 dma_status,
                                       RadioStatus
                     then READ_SAMPLES (pd_protocol) reads the samples, 256 Ki per message.
"""
import os
import socket
import struct
import sys
from dataclasses import dataclass

import numpy as np

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                 "..", "..", "Packet_Detector", "PC")))
import pd_protocol as pdp  # noqa: E402

RADIO_CONFIG, CAPTURE = 22, 23
pdp.CMD_NAMES.update({RADIO_CONFIG: "RADIO_CONFIG", CAPTURE: "CAPTURE"})
BOARD_FMCOMMS3 = 0x800                 # PING flag: samples from an AD-FMCOMMS3 on the FMC connector
RADIO_FLAG_REINIT = 0x1                # RADIO_CONFIG: set the AD9361 up again from reset
CAPTURE_OVERFLOW = 0x1                 # CAPTURE flags: samples were lost

GAIN_MODES = {"manual": 0, "fast_attack": 1, "slow_attack": 2, "hybrid": 3}
GAIN_MODE_NAMES = {v: k for k, v in GAIN_MODES.items()}
ENSM_NAMES = {0: "TX", 1: "RX", 2: "ALERT", 3: "FDD", 4: "WAIT", 5: "SLEEP", 6: "PINCTRL",
              7: "PINCTRL_FDD_INDEP"}
READ_CHUNK = pdp.MAX_PAYLOAD // 4       # samples per READ_SAMPLES


@dataclass
class RadioConfig:
    lo_hz: int = 2_420_000_000
    rate_hz: int = 40_000_000
    bw_hz: int = 18_000_000
    gain_mode: int = 0                  # GAIN_MODES
    gain_db: int = 30
    port: int = 0                       # 0 = Rx1A
    flags: int = 0

    FMT = struct.Struct("<QIIIiII")

    def pack(self) -> bytes:
        return self.FMT.pack(self.lo_hz, self.rate_hz, self.bw_hz, self.gain_mode, self.gain_db,
                             self.port, self.flags)


@dataclass
class RadioStatus:
    ok: int
    last_error: int
    lo_hz: int
    rate_hz: int
    bw_hz: int
    gain_mode: int
    gain_db: int
    port: int
    ensm_mode: int
    clk_freq_raw: int
    adc_status: int
    dma_status: int
    chan_status_i: int
    chan_status_q: int

    FMT = struct.Struct("<IiQIIIiIIIIIII4x")

    @classmethod
    def unpack(cls, b: bytes) -> "RadioStatus":
        return cls(*cls.FMT.unpack(b[:cls.FMT.size]))

    @property
    def l_clk_hz(self) -> float:
        return self.clk_freq_raw * 100e6 / 65536

    def describe(self) -> str:
        return (f"AD9361 {'ready' if self.ok else 'NOT set up'} (last error {self.last_error}), "
                f"LO {self.lo_hz / 1e6:.3f} MHz, {self.rate_hz / 1e6:.3f} Msps, RF BW {self.bw_hz / 1e6:.2f} MHz, "
                f"gain {GAIN_MODE_NAMES.get(self.gain_mode, self.gain_mode)} {self.gain_db} dB, "
                f"port {self.port}, ENSM {ENSM_NAMES.get(self.ensm_mode, self.ensm_mode)}; "
                f"axi_ad9361: l_clk {self.l_clk_hz / 1e6:.2f} MHz, status 0x{self.adc_status:X}, "
                f"DMA status 0x{self.dma_status:X}, channels I 0x{self.chan_status_i:X} Q 0x{self.chan_status_q:X}")


@dataclass
class CaptureInfo:
    n_samples: int
    flags: int
    time_us: int
    dma_status: int
    radio: RadioStatus

    @property
    def overflow(self) -> bool:
        return bool(self.flags & CAPTURE_OVERFLOW)


class FmcBoard:
    """One TCP connection to the board (fmc_server.c)."""

    def __init__(self, host: str = "10.42.0.101", port: int = pdp.DEFAULT_PORT, timeout: float = 30.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.seq = 0

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def call(self, cmd: int, payload: bytes = b"") -> bytes:
        self.seq += 1
        pdp.send_msg(self.sock, cmd, payload, self.seq)
        rcmd, status, seq, reply = pdp.recv_msg(self.sock)
        if rcmd != cmd or seq != self.seq:
            raise pdp.ProtocolError(f"reply cmd {rcmd} seq {seq} to cmd {cmd} seq {self.seq}")
        if status != pdp.ST_OK:
            raise pdp.ProtocolError(f"{pdp.CMD_NAMES.get(cmd, cmd)}: status {status}: "
                                    f"{reply.decode(errors='replace')}")
        return reply

    def ping(self) -> pdp.BoardInfo:
        return pdp.BoardInfo.unpack(self.call(pdp.PING))

    def radio_status(self) -> RadioStatus:
        return RadioStatus.unpack(self.call(RADIO_CONFIG))

    def radio_config(self, cfg: RadioConfig) -> RadioStatus:
        return RadioStatus.unpack(self.call(RADIO_CONFIG, cfg.pack()))

    def capture(self, n_samples: int, timeout_ms: int = 10000) -> CaptureInfo:
        r = self.call(CAPTURE, struct.pack("<II", n_samples, timeout_ms))
        n, flags, t_us, dma = struct.unpack_from("<4I", r)
        return CaptureInfo(n, flags, t_us, dma, RadioStatus.unpack(r[16:]))

    def read_samples(self, n_samples: int, offset: int = 0) -> np.ndarray:
        """(N, 2) int16 I/Q, read in READ_SAMPLES chunks."""
        parts = []
        done = 0
        while done < n_samples:
            k = min(READ_CHUNK, n_samples - done)
            parts.append(pdp.bytes_to_iq(self.call(pdp.READ_SAMPLES, struct.pack("<II", offset + done, k))))
            done += k
        return np.concatenate(parts) if parts else np.zeros((0, 2), np.int16)
