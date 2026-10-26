"""
pd_protocol.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Wire protocol between the PC and the Zedboard packet-detector test bench, shared by
pd_hw_test.py (client) and pd_board_emulator.py (software stand-in for the board).
The Zedboard firmware (ARM, lwIP TCP server) must implement exactly this, so every
structure is plain little-endian with fixed field widths, easy to mirror in C.

Transport: one TCP connection, default port 5001. Strict request/response: the PC sends
one message and waits for the reply before sending the next.

Message = 16-byte header + payload
    u32 magic     0x44504C46 ("FLPD" in memory order)
    u16 cmd       command code (reply uses the same code)
    u16 status    0 in requests; in replies 0 = OK, otherwise an error code and the
                  payload is a UTF-8 error message
    u32 seq       request sequence number, echoed in the reply
    u32 length    payload length in bytes (<= MAX_PAYLOAD)

Commands (request payload -> reply payload)
    PING          -                         -> BoardInfo
    RESET         -                         -> -       detector reset: sample_cnt = 0, blanking
                                                       restarts, sample buffer emptied
    SET_PARAMS    Params                    -> -
    LOAD_SAMPLES  u32 offset, then int16    -> u32 samples now stored
                  I,Q pairs (12-bit values
                  sign-extended to 16 bits)
    READ_SAMPLES  u32 offset, u32 count     -> int16 I,Q pairs   (loopback / plumbing check)
    RUN           u32 count (0 = all        -> u32 n_events, u32 samples processed,
                  stored), u32 flags              then n_events x EventRecord
                  (bit0 = capture debug,
                   bit1 = PL loopback)
    GET_DEBUG     u32 offset, u32 count     -> count x DebugRecord (from the last RUN with
                                                       capture enabled); after a PL-loopback RUN
                                                       the 4-byte records are the samples as they
                                                       came back from the PL

RUN streams buffer samples [0, count) through the detector. The detector state (running
sums, sample_cnt, armed/holdoff) carries over between RUNs until RESET, so a long record
can be loaded and run in pieces. The board may process faster or slower than 20 Msps;
the detector counts samples (tvalid beats), not clocks, so results must be identical.

Fixed-point conventions (see pd_reference.py for the exact arithmetic)
    samples       12-bit signed I/Q in int16, the LPF output (spec 5.1 s_iq_tdata)
    eta_*_sq      unsigned Q0.16 (0.35^2 = 0.1225 -> 8028)
    tone_thresh   dB in Q8.8 (13 dB -> 3328), 0 = check off
    cfo_inc       NCO phase increment for -f_hat, signed 24-bit (sign-extended to 32),
                  LSB = Fs / 2^24 = 1.192 Hz
    lag angles    16-bit CORDIC angle, -32768..32767 = -pi..pi
"""
from __future__ import annotations

import socket
import struct
from dataclasses import astuple, dataclass, fields

import numpy as np

PROTOCOL_VERSION = 1
DEFAULT_PORT = 5001
MAGIC = 0x44504C46
HEADER = struct.Struct("<IHHII")
MAX_PAYLOAD = 1 << 20                  # 1 MiB per message
MAX_LOAD_SAMPLES = 65536               # samples per LOAD_SAMPLES message (256 KiB)

FS = 20e6
NCO_BITS = 24

# Commands
PING, RESET, SET_PARAMS, LOAD_SAMPLES, READ_SAMPLES, RUN, GET_DEBUG = 1, 2, 3, 4, 5, 6, 7
CMD_NAMES = {PING: "PING", RESET: "RESET", SET_PARAMS: "SET_PARAMS", LOAD_SAMPLES: "LOAD_SAMPLES",
             READ_SAMPLES: "READ_SAMPLES", RUN: "RUN", GET_DEBUG: "GET_DEBUG"}

# Reply status codes
ST_OK, ST_BAD_CMD, ST_BAD_ARG, ST_NO_SPACE, ST_NO_DEBUG, ST_INTERNAL = 0, 1, 2, 3, 4, 5

RUN_FLAG_DEBUG = 0x1
RUN_FLAG_PL_LOOPBACK = 0x2             # stream the samples through the PL unchanged into the debug
#                                        buffer (no detector); GET_DEBUG then returns the samples

# BoardInfo.flags bits
BOARD_EMULATOR = 0x1
BOARD_NO_DETECTOR = 0x2                # bitstream has no detector yet (phase 1, PL loopback only)

# EventRecord.status bits
EV_CFO_VALID = 0x1                     # N_CFO samples were available, cfo_inc is valid
EV_TONE_FAIL = 0x2                     # tone-structure check below threshold (packet aborted)
EV_TRUNCATED = 0x4                     # stream ended inside the CFO window

# DebugRecord.bits
DBG_ABOVE_HI = 0x01
DBG_BELOW_LO = 0x02
DBG_ARMED = 0x04                       # trigger enabled (after blanking, not in holdoff)
DBG_TRIG = 0x08                        # this sample is t_trig
DBG_BLANK = 0x10                       # inside the post-reset blanking


def q16(x: float) -> int:
    return int(round(x * 65536))


def q8(x: float) -> int:
    return int(round(x * 256))


@dataclass
class Params:
    """Run-time registers (spec 5.2) plus the test-bench re-arm holdoff."""
    eta_hi_sq: int = q16(0.35 ** 2)    # Q0.16
    eta_lo_sq: int = q16(0.20 ** 2)    # Q0.16
    qual_len: int = 128
    n_cfo: int = 2048
    blank_len: int = 288
    tone_thresh: int = q8(13.0)        # Q8.8 dB, 0 = off
    arm_holdoff: int = 131072          # samples from t_trig until the trigger re-arms; stands in
    #                                    for the Rx FSM's arm pulse (spec 6), must exceed a packet
    input_stage: int = 0               # 0 = samples are the LPF output (DC blocker + LPF on PC)
    #                                    1 = samples are the decimated 12-bit stream; the board runs
    #                                        the DC blocker and LPF (pd_reference.front_end)
    dc_shift: int = 10                 # DC blocker alpha = 1 - 2^-dc_shift, 0 = bypass (spec 5.2);
    #                                    used with input_stage 1 (was "reserved", same position)

    FMT = struct.Struct("<9I")

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    @classmethod
    def unpack(cls, b: bytes) -> "Params":
        return cls(*cls.FMT.unpack(b[:cls.FMT.size]))

    def as_dict(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    def describe(self) -> str:
        return (f"eta_hi {np.sqrt(self.eta_hi_sq / 65536):.3f}, eta_lo {np.sqrt(self.eta_lo_sq / 65536):.3f}, "
                f"Q {self.qual_len}, N_CFO {self.n_cfo}, blank {self.blank_len}, "
                f"tone {self.tone_thresh / 256:.1f} dB, holdoff {self.arm_holdoff}")


@dataclass
class BoardInfo:
    version: int
    build: int
    max_samples: int
    flags: int                         # bit0 = software emulator

    FMT = struct.Struct("<4I")

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    @classmethod
    def unpack(cls, b: bytes) -> "BoardInfo":
        return cls(*cls.FMT.unpack(b[:cls.FMT.size]))


@dataclass
class EventRecord:
    t_trig: int                        # sample_cnt latched at trigger
    cfo_inc: int                       # NCO increment for -f_hat (signed 24-bit)
    cfo_valid_cnt: int                 # sample_cnt when cfo_valid rises (t_trig + N_CFO)
    tone_db_q8: int                    # ToneCheck in Q8.8 dB
    status: int                        # EV_* bits
    ang16: int = 0                     # CORDIC angles of A16, A32, A128, A256
    ang32: int = 0
    ang128: int = 0
    ang256: int = 0

    FMT = struct.Struct("<IiIhH4h")    # 24 bytes

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    @classmethod
    def unpack_many(cls, b: bytes, n: int) -> list["EventRecord"]:
        s = cls.FMT.size
        return [cls(*cls.FMT.unpack(b[i * s:(i + 1) * s])) for i in range(n)]

    @property
    def cfo_hz(self) -> float:
        """Estimated carrier offset f_hat in Hz (cfo_inc is the increment for -f_hat)."""
        return -self.cfo_inc * FS / 2 ** NCO_BITS

    @property
    def tone_db(self) -> float:
        return self.tone_db_q8 / 256


DEBUG_DTYPE = np.dtype([("metric", "<u2"), ("bits", "<u2")])   # 4 bytes per sample
SAMPLE_DTYPE = np.dtype("<i2")


# --------------------------------------------------------------------------- #
# Framing
# --------------------------------------------------------------------------- #
class ProtocolError(RuntimeError):
    pass


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 16))
        if not chunk:
            raise ConnectionError("connection closed by peer")
        buf += chunk
    return bytes(buf)


def send_msg(sock: socket.socket, cmd: int, payload: bytes = b"", seq: int = 0, status: int = ST_OK) -> None:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload {len(payload)} bytes exceeds MAX_PAYLOAD {MAX_PAYLOAD}")
    sock.sendall(HEADER.pack(MAGIC, cmd, status, seq, len(payload)) + payload)


def recv_msg(sock: socket.socket) -> tuple[int, int, int, bytes]:
    """Return (cmd, status, seq, payload)."""
    magic, cmd, status, seq, length = HEADER.unpack(recv_exact(sock, HEADER.size))
    if magic != MAGIC:
        raise ProtocolError(f"bad magic 0x{magic:08X}")
    if length > MAX_PAYLOAD:
        raise ProtocolError(f"payload length {length} exceeds MAX_PAYLOAD")
    return cmd, status, seq, recv_exact(sock, length)


def iq_to_bytes(iq: np.ndarray) -> bytes:
    """(N, 2) int16 I/Q -> interleaved little-endian bytes."""
    return np.ascontiguousarray(iq, dtype=SAMPLE_DTYPE).tobytes()


def bytes_to_iq(b: bytes) -> np.ndarray:
    return np.frombuffer(b, dtype=SAMPLE_DTYPE).reshape(-1, 2)
