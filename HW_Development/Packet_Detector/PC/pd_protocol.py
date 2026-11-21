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
    SET_PARAMS    Params (44 bytes; 40 =    -> -
                  without rearm (holdoff),
                  36 = also without t_pk)
    LOAD_SAMPLES  u32 offset, then int16    -> u32 samples now stored
                  I,Q pairs (12-bit values
                  sign-extended to 16 bits)
    READ_SAMPLES  u32 offset, u32 count     -> int16 I,Q pairs   (loopback / plumbing check)
    RUN           u32 count (0 = all        -> u32 n_events, u32 samples processed,
                  stored), u32 flags              then n_events x EventRecord, then (firmware
                  (bit0 = capture debug,          build 10 and later) u32 n_timing and
                   bit1 = PL loopback)            n_timing x TimingRecord (Preamble B timing,
                                                  input stages 1 and 2; BoardInfo flag
                                                  BOARD_TIMING). Older replies end after the
                                                  events. Then (firmware build 11 and later,
                                                  BoardInfo flag BOARD_CF) u32 n_control and
                                                  n_control x ControlRecord (control field after
                                                  each good timing record). Then (firmware
                                                  build 13 and later, BOARD_PAYLOAD) u32
                                                  n_payload and n_payload x PayloadRecord (one
                                                  per payload symbol of each accepted packet).
                                                  Then (build 14, BOARD_CHEST) u32 n_chest and
                                                  n_chest x PayloadRecord: channel-estimate
                                                  records (sym CH_PKT = packet record).
                                                  Then (build 15, BOARD_LQ) u32 n_lq and n_lq x
                                                  PayloadRecord: decoder-input records (sym =
                                                  code word). Then (build 16, BOARD_TB) u32 n_tb
                                                  and n_tb x TBRecord (variable length:
                                                  decoded transport blocks)
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

# CoreBoard UDP stream test (firmware build 17, PC/pd_udp_test.py)
UDP_START, UDP_STATS, UDP_STOP, GEM_STATS = 14, 15, 16, 17
CMD_NAMES.update({UDP_START: "UDP_START", UDP_STATS: "UDP_STATS", UDP_STOP: "UDP_STOP", GEM_STATS: "GEM_STATS"})
# Streaming receiver (firmware build 25, bitstream 9.1, PC/pd_stream_test.py)
STREAM_START, POLL_RESULTS, STREAM_STOP = 18, 19, 20
CMD_NAMES.update({STREAM_START: "STREAM_START", POLL_RESULTS: "POLL_RESULTS", STREAM_STOP: "STREAM_STOP"})
STREAM_KEEP_PL = 0x1                   # STREAM_START flag: also return payload / channel / decoder-input records
STREAM_PUSH = 21                       # board -> PC only (firmware build 39): results pushed while streaming
CMD_NAMES[STREAM_PUSH] = "STREAM_PUSH"
STREAM_FLAG_PUSH = 0x2                 # STREAM_START flag (build 39); bits 6:2 = push interval / 250 us (0: 1 ms)
PUSH_SLOT_SAMPLES = 336                # receiver samples (20 Msps after the half-band) per datagram slot
STREAM_LOST_NAMES = {0x1: "events", 0x2: "timing", 0x4: "control", 0x8: "payload", 0x10: "channel estimate",
                     0x20: "decoder input", 0x40: "transport blocks"}
HTTP_STATES = {0: "none", 1: "connecting", 2: "sent", 3: "answered", 4: "failed"}
# GEM_STATS reply (firmware build 18): the Zynq GEM statistics registers 0x100..0x1B0, summed
# since UDP_START (UG585, gem.* registers)
GEM_STAT_NAMES = [
    "octets_tx_lo", "octets_tx_hi", "frames_tx", "broadcast_tx", "multicast_tx", "pause_tx",
    "tx_64", "tx_65_127", "tx_128_255", "tx_256_511", "tx_512_1023", "tx_1024_1518", "(0x130)",
    "tx_underruns", "single_collisions", "multi_collisions", "excess_collisions", "late_collisions",
    "deferred_tx", "carrier_sense_errors", "octets_rx_lo", "octets_rx_hi", "frames_rx",
    "broadcast_rx", "multicast_rx", "pause_rx", "rx_64", "rx_65_127", "rx_128_255", "rx_256_511",
    "rx_512_1023", "rx_1024_1518", "(0x180)", "undersize_rx", "oversize_rx", "jabbers_rx",
    "fcs_errors", "length_field_errors", "symbol_errors", "alignment_errors", "rx_resource_errors",
    "rx_overruns", "ip_checksum_errors", "tcp_checksum_errors", "udp_checksum_errors"]
GEM_ERROR_NAMES = {"undersize_rx", "oversize_rx", "jabbers_rx", "fcs_errors", "length_field_errors",
                   "symbol_errors", "alignment_errors", "rx_resource_errors", "rx_overruns",
                   "ip_checksum_errors", "tcp_checksum_errors", "udp_checksum_errors"}
UDP_DEFAULT_PORT = 4000

# Reply status codes
ST_OK, ST_BAD_CMD, ST_BAD_ARG, ST_NO_SPACE, ST_NO_DEBUG, ST_INTERNAL = 0, 1, 2, 3, 4, 5

RUN_FLAG_DEBUG = 0x1
RUN_FLAG_PACK8 = 0x4                   # build 24: the sample buffer holds packed int8 samples, two per
#                                        word ({Q1, I1, Q0, I0}); RUN count in samples (even); the
#                                        PL unpacks them (bitstream 9.1, pd_unpack8.v)
RUN_FLAG_PL_LOOPBACK = 0x2             # stream the samples through the PL unchanged into the debug
#                                        buffer (no detector); GET_DEBUG then returns the samples

# BoardInfo.flags bits
BOARD_EMULATOR = 0x1
BOARD_NO_DETECTOR = 0x2                # bitstream has no detector yet (phase 1, PL loopback only)
BOARD_TIMING = 0x4                     # bitstream has the Preamble B timing (ID 3.0, firmware build 10):
#                                        the RUN reply carries timing records after the events
BOARD_CF = 0x8                         # bitstream has the control-field receiver (ID 4.0, firmware
#                                        build 11): the RUN reply carries control records after the
#                                        timing records
BOARD_REARM = 0x10                     # bitstream has the Rx state machine's re-arm (ID 5.0, firmware
#                                        build 12): Params.rearm is used (else always the holdoff)
BOARD_PAYLOAD = 0x20                   # bitstream has the shared fast FFT with the speculative payload
#                                        FFT (ID 6.0, firmware build 13): the RUN reply carries payload
#                                        records after the control records (only with Params.rearm)
BOARD_CHEST = 0x40                     # bitstream has the payload channel estimate (ID 7.0, firmware
#                                        build 14): the RUN reply carries channel-estimate records
#                                        after the payload records (only with Params.rearm)
CH_PKT = 0x8000                        # PayloadRecord.sym of a channel-estimate packet record
BOARD_LQ = 0x80                        # bitstream has the demapper and combiner (ID 8.0, firmware
#                                        build 15): decoder-input records after the channel-estimate
#                                        records (only with Params.rearm)
BOARD_PACK8 = 0x200                    # bitstream has the int8 unpacker (ID 9.1, firmware build 24)
BOARD_NOREC = 0x400                    # bitstream can discard the debug records (ID 9.2, firmware build 31)
BOARD_TB = 0x100                       # bitstream has the LDPC decoder (ID 9.0, firmware build 16):
#                                        transport blocks after the decoder-input records (rearm)

# EventRecord.status bits
EV_CFO_VALID = 0x1                     # N_CFO samples were available, cfo_inc is valid
EV_TONE_FAIL = 0x2                     # tone-structure check below threshold (packet aborted)
EV_TRUNCATED = 0x4                     # stream ended inside the CFO window

# TimingRecord.status bits (Preamble B timing, spec 4.5)
TM_PEAK_OK = 0x1                       # peak / mean >= T_pk (or T_pk = 0)
TM_LATE = 0x2                          # late trigger: the CFO window ran into Preamble B (abort)
TM_BUSY = 0x4                          # no search: the previous one was still using the samples
TM_GOOD = TM_PEAK_OK                   # status == TM_GOOD: timing found, packet continues

# ControlRecord.status bits (Docs/Specification_ControlField.md)
CF_CRC_OK = 0x1                        # CRC10 over the 64 decoded bits is 0: the message is valid
CF_MSG_BITS = 54

# DebugRecord.bits
DBG_ABOVE_HI = 0x01
DBG_BELOW_LO = 0x02
DBG_ARMED = 0x04                       # trigger enabled (after blanking, not in holdoff)
DBG_TRIG = 0x08                        # this sample is t_trig
DBG_BLANK = 0x10                       # inside the post-reset blanking
DBG_LOW_POWER = 0x20                   # R0 or R_D below the power floor E_MIN (counts as below_lo)


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
    #                                    for the Rx FSM's arm pulse (spec 6), must exceed a packet.
    #                                    Effective value max(arm_holdoff, n_cfo, 1): one CFO lane
    input_stage: int = 0               # 0 = samples are the LPF output (DC blocker + LPF on PC)
    #                                    1 = samples are the decimated 12-bit stream; the board runs
    #                                        the DC blocker and LPF (pd_reference.front_end)
    #                                    2 = samples are the 12-bit stream at 40 Msps; the board runs
    #                                        the half-band decimator too (pd_reference.halfband).
    #                                        RUN counts must be even; t_trig, debug records and
    #                                        all other sample indices are at 20 Msps
    dc_shift: int = 10                 # DC blocker alpha = 1 - 2^-dc_shift, 0 = bypass (spec 5.2);
    #                                    used with input_stage 1 (was "reserved", same position)
    t_pk: int = q8(18.0)               # Preamble B timing: minimum peak / mean, Q8.8, 0 = no check
    #                                    (spec 4.5, T_pk). The timing search runs with input_stage
    #                                    1 and 2 (it needs the unfiltered stream)
    rearm: int = 1                     # 1 = the Rx state machine re-arms the detector after the
    #                                    packet's outcome (pd_reference.REARM_*: tone fail, timing
    #                                    abort, control field rejected, end of the packet);
    #                                    0 = fixed holdoff after every trigger (arm_holdoff).
    #                                    Boards before firmware build 12 always use the holdoff

    FMT = struct.Struct("<11I")
    FMT_V2 = struct.Struct("<10I")     # firmware builds 10, 11: no rearm
    FMT_V1 = struct.Struct("<9I")      # firmware builds <= 9: no t_pk

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    def pack_v1(self) -> bytes:
        return self.FMT_V1.pack(*astuple(self)[:9])

    @classmethod
    def unpack(cls, b: bytes) -> "Params":
        if len(b) < cls.FMT_V2.size:                    # 36-byte SET_PARAMS (builds <= 9)
            return cls(*cls.FMT_V1.unpack(b[:cls.FMT_V1.size]), rearm=0)
        if len(b) < cls.FMT.size:                       # 40 bytes (builds 10, 11)
            return cls(*cls.FMT_V2.unpack(b[:cls.FMT_V2.size]), rearm=0)
        return cls(*cls.FMT.unpack(b[:cls.FMT.size]))

    def as_dict(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    def describe(self) -> str:
        return (f"eta_hi {np.sqrt(self.eta_hi_sq / 65536):.3f}, eta_lo {np.sqrt(self.eta_lo_sq / 65536):.3f}, "
                f"Q {self.qual_len}, N_CFO {self.n_cfo}, blank {self.blank_len}, "
                f"tone {self.tone_thresh / 256:.1f} dB, "
                + ("re-arm by the state machine" if self.rearm else f"holdoff {self.arm_holdoff}")
                + f", T_pk {self.t_pk / 256:g}")


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
class UdpStats:
    """UDP_STATS / UDP_STOP reply (firmware build 17): the board's count of the ESR2 stream."""
    running: int
    mode: int                          # 0 count; 1..3 convert (cached / non-cacheable / cached + flush)
    port: int
    elapsed_ms: int                    # since UDP_START
    active_ms: int                     # first to last datagram of the stream
    datagrams: int                     # good ESR2 datagrams
    bad_size: int
    bad_magic: int
    epoch: int
    epochs: int                        # stream (re)starts seen
    lost: int                          # missing by sample position (net of late arrivals)
    late: int                          # arrived after a later one
    duplicates: int
    gem_frames: int                    # GEM: frames received (all traffic)
    gem_no_buffer: int                 # GEM: frames dropped, no receive buffer (software too slow)
    gem_overrun: int                   # GEM: frames dropped, receive overrun (memory too slow)
    busy_ms: int                       # time in the UDP receive callback
    loops: int                         # main-loop iterations since UDP_START (idle indicator)
    first_pos: int
    last_pos: int
    skipped: int = 0                   # build 21: conversions skipped by the overload guard
    pool_empty: int = 0                # build 22: lwIP driver found the pbuf pool empty (frame lost)

    FMT = struct.Struct("<18IQQ")

    @classmethod
    def unpack(cls, b: bytes) -> "UdpStats":
        n = cls.FMT.size
        extra = [struct.unpack_from("<I", b, n + 4 * i)[0] if len(b) >= n + 4 * (i + 1) else 0 for i in range(2)]
        return cls(*cls.FMT.unpack(b[:n]), *extra)


@dataclass
class StreamStats:
    """Head of the POLL_RESULTS / STREAM_STOP reply (firmware build 25); then the records since
    the last poll in the RUN reply format (parse_run_reply_tb)."""
    size: int                          # bytes of this block (records follow)
    running: int
    elapsed_ms: int                    # since STREAM_START
    datagrams: int                     # stored in the ring
    lost: int                          # zero-filled slots (datagrams that never came)
    late_fixed: int                    # late datagrams put into their slot in time
    too_late: int                      # late datagrams whose slot had been fed already
    ring_full: int                     # datagrams dropped: ring full (receiver slower than the stream)
    resyncs: int                       # stream jumps (big gap, ring full, CoreBoard restart)
    ring_level: int                    # datagrams waiting for the DMA
    ring_max: int
    dma_chunks: int
    dma_errors: int
    det_status: int                    # receiver STATUS register (sticky error bits)
    pool_empty: int                    # lwIP pbuf pool empty in the receive interrupt (frames lost)
    gem_no_buffer: int
    gem_overrun: int
    lost_mask: int                     # record buffers that overflowed since the last poll (STREAM_LOST_NAMES)
    epoch: int
    det_samples: int                   # samples the receiver took (40 Msps, 32 bits)
    fed_samples: int                   # samples the DMA fed (u64)
    http_state: int                    # HTTP request to the CoreBoard (HTTP_STATES)
    http_code: int
    http_err: int
    bad: int                           # datagrams of wrong size or magic
    polls: int
    duplicates: int
    # build 26: DMA watchdog (a transfer not done after 50 ms is recorded, the DMA reset)
    stalls: int = 0
    stall_sr: int = 0                  # MM2S status register at the stall (bit 0 halted, 1 idle, 4..6 errors)
    stall_status: int = 0              # receiver STATUS at the stall
    stall_ev: int = 0                  # FIFO levels at the stall: events, timing, control,
    stall_tm: int = 0                  # payload, channel estimate, decoder input, transport blocks
    stall_cf: int = 0
    stall_pl: int = 0
    stall_ch: int = 0
    stall_lq: int = 0
    stall_tb: int = 0
    stall_samples: int = 0             # receiver sample count at the stall
    dma_sr: int = 0                    # MM2S status register now
    gaps: list = None                  # build 27: first zero-filled gaps [(slot, datagrams), ...]

    FMT = struct.Struct("<20IQ6I")     # 112 bytes (build 25)
    FMT26 = struct.Struct("<12I")      # + 48 bytes (build 26)
    FMT27 = struct.Struct("<17I")      # + 68 bytes (build 27): n, 8 x (slot, length)
    FMT30 = struct.Struct("<8I")       # + 32 bytes (build 30): CPU time, see cpu below

    @classmethod
    def unpack(cls, b: bytes) -> "StreamStats":
        size = struct.unpack_from("<I", b)[0]
        n26 = cls.FMT.size + cls.FMT26.size
        extra = cls.FMT26.unpack_from(b, cls.FMT.size) if size >= n26 else ()
        st = cls(*cls.FMT.unpack(b[:cls.FMT.size]), *extra)
        st.gaps = []
        if size >= n26 + cls.FMT27.size:
            g = cls.FMT27.unpack_from(b, n26)
            st.gaps = [(g[1 + 2 * i], g[2 + 2 * i]) for i in range(min(g[0], 8))]
        st.cpu = None
        n27 = n26 + cls.FMT27.size
        if size >= n27 + cls.FMT30.size:
            st.cpu = dict(zip(("pd_reads", "rx_us", "drain_us", "drain_max_us", "poll_us", "poll_max_us",
                               "loop_max_us", "drains"), cls.FMT30.unpack_from(b, n27)))
        return st


def parse_push(payload: bytes):
    """STREAM_PUSH (build 39) -> (newest datagram slot, StreamStats or None, records).
    Header: u32 flags (bit0: statistics follow), u32 0, u64 newest slot; statistics every 100 ms."""
    flags, _, slot = struct.unpack_from("<IIQ", payload)
    off, st = 16, None
    if flags & 1:
        st = StreamStats.unpack(payload[16:])
        off += st.size
    return slot, st, parse_run_reply_tb(payload[off:])


def parse_poll(reply: bytes):
    """POLL_RESULTS / STREAM_STOP reply -> (StreamStats, parse_run_reply_tb(records))."""
    st = StreamStats.unpack(reply)
    return st, parse_run_reply_tb(reply[st.size:])


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


@dataclass
class TimingRecord:
    """Preamble B timing result, one per event that passed the tone check (spec 4.5)."""
    t_trig: int                        # the event's t_trig
    n_end: int                         # sample index of the last Preamble B sample (0 with TM_BUSY)
    status: int                        # TM_* bits
    n_pos: int                         # positions searched (5065 - N_CFO)
    p_max: int                         # |y|^2 at n_end
    p_sum: int                         # sum of |y|^2 over the window

    FMT = struct.Struct("<IIIIQQ")     # 32 bytes

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    @classmethod
    def unpack_many(cls, b: bytes, n: int) -> list["TimingRecord"]:
        s = cls.FMT.size
        return [cls(*cls.FMT.unpack(b[i * s:(i + 1) * s])) for i in range(n)]

    @property
    def peak_to_mean(self) -> float:
        return self.p_max * self.n_pos / self.p_sum if self.p_sum else 0.0

    @property
    def payload_start(self) -> int:
        return self.n_end + 1


@dataclass
class ControlRecord:
    """Control field, one per good timing record (status == TM_GOOD), in the same order."""
    t_trig: int                        # the event's t_trig
    n_end: int                         # the timing record's n_end (control field from n_end + 1)
    status: int                        # CF_* bits
    shift: int                         # Viterbi input scaling (soft values >> shift)
    metric: int                        # final Viterbi path metric (signed)
    message: int                       # the 54 decoded message bits, bit i = message bit i

    FMT = struct.Struct("<IIIhhQ")     # 24 bytes

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    @classmethod
    def unpack_many(cls, b: bytes, n: int) -> list["ControlRecord"]:
        s = cls.FMT.size
        return [cls(*cls.FMT.unpack(b[i * s:(i + 1) * s])) for i in range(n)]

    @property
    def crc_ok(self) -> bool:
        return bool(self.status & CF_CRC_OK)

    def bits(self) -> list[int]:
        return [(self.message >> i) & 1 for i in range(CF_MSG_BITS)]


@dataclass
class PayloadRecord:
    """Speculative payload FFT: one per payload symbol of an accepted packet (control record CRC
    ok and every field valid, Params.rearm), in order (cf_reference.payload_records)."""
    t_trig: int                        # the packet's t_trig
    sym: int                           # payload symbol 0 .. PacketLength - 1
    sig_a: int                         # grid signature (cf_reference.grid_signature)
    sig_b: int

    FMT = struct.Struct("<IHxxII")     # 16 bytes

    def pack(self) -> bytes:
        return self.FMT.pack(*astuple(self))

    @classmethod
    def unpack_many(cls, b: bytes, n: int) -> list["PayloadRecord"]:
        s = cls.FMT.size
        return [cls(*cls.FMT.unpack(b[i * s:(i + 1) * s])) for i in range(n)]


@dataclass
class TBRecord:
    """A decoded transport block (firmware build 16, BOARD_TB): one per complete code word of an
    accepted packet, in order (pl_reference.tb_records)."""
    t_trig: int
    w: int                             # code word in the packet
    crc_ok: int
    iters: int                         # LDPC iterations (1 .. 10)
    nbits: int                         # K - 24
    bits: tuple                        # the transport block, nbits values 0 / 1

    HDR = struct.Struct("<IHBBH2x")    # 12 bytes, then ceil(nbits / 32) u32 words, bit j in word j // 32

    def pack(self) -> bytes:
        words = [0] * ((self.nbits + 31) // 32)
        for j, b in enumerate(self.bits):
            words[j // 32] |= int(b) << (j % 32)
        return self.HDR.pack(self.t_trig, self.w, self.crc_ok, self.iters, self.nbits) + struct.pack(f"<{len(words)}I", *words)

    @classmethod
    def unpack_many(cls, b: bytes, n: int) -> tuple[list["TBRecord"], int]:
        """n records from b; returns (records, bytes used)."""
        out, off = [], 0
        for _ in range(n):
            t, w, ok, it, nb = cls.HDR.unpack_from(b, off)
            off += cls.HDR.size
            nw = (nb + 31) // 32
            words = struct.unpack_from(f"<{nw}I", b, off)
            off += 4 * nw
            out.append(cls(t, w, ok, it, nb, tuple((words[j // 32] >> (j % 32)) & 1 for j in range(nb))))
        return out, off


def parse_run_reply_all(reply: bytes):
    """RUN reply -> (events, samples processed, timing records, control records, payload
    records); parse_run_reply_chest also returns the channel-estimate records. Replies of older
    firmware end after the events (build <= 9), the timing records (build 10), the control
    records (builds 11, 12) or the payload records (build 13)."""
    return parse_run_reply_tb(reply)[:5]


def parse_run_reply_chest(reply: bytes):
    """RUN reply -> (events, samples processed, timing, control, payload, channel-estimate
    records)."""
    return parse_run_reply_tb(reply)[:6]


def parse_run_reply_lq(reply: bytes):
    """parse_run_reply_tb without the transport blocks."""
    return parse_run_reply_tb(reply)[:7]


def parse_run_reply_tb(reply: bytes):
    """RUN reply -> (events, samples processed, timing, control, payload, channel-estimate,
    decoder-input records, transport blocks)."""
    n_ev, done = struct.unpack_from("<II", reply)
    evs = EventRecord.unpack_many(reply[8:], n_ev)
    off = 8 + EventRecord.FMT.size * n_ev
    tms, cfs, pls, chs, lqs, tbs = [], [], [], [], [], []
    if len(reply) >= off + 4:
        (n_tm,) = struct.unpack_from("<I", reply, off)
        tms = TimingRecord.unpack_many(reply[off + 4:], n_tm)
        off += 4 + TimingRecord.FMT.size * n_tm
        if len(reply) >= off + 4:
            (n_cf,) = struct.unpack_from("<I", reply, off)
            cfs = ControlRecord.unpack_many(reply[off + 4:], n_cf)
            off += 4 + ControlRecord.FMT.size * n_cf
            if len(reply) >= off + 4:
                (n_pl,) = struct.unpack_from("<I", reply, off)
                pls = PayloadRecord.unpack_many(reply[off + 4:], n_pl)
                off += 4 + PayloadRecord.FMT.size * n_pl
                if len(reply) >= off + 4:
                    (n_ch,) = struct.unpack_from("<I", reply, off)
                    chs = PayloadRecord.unpack_many(reply[off + 4:], n_ch)
                    off += 4 + PayloadRecord.FMT.size * n_ch
                    if len(reply) >= off + 4:
                        (n_lq,) = struct.unpack_from("<I", reply, off)
                        lqs = PayloadRecord.unpack_many(reply[off + 4:], n_lq)
                        off += 4 + PayloadRecord.FMT.size * n_lq
                        if len(reply) >= off + 4:
                            (n_tb,) = struct.unpack_from("<I", reply, off)
                            tbs, _ = TBRecord.unpack_many(reply[off + 4:], n_tb)
    return evs, done, tms, cfs, pls, chs, lqs, tbs


def parse_run_reply(reply: bytes) -> tuple[list[EventRecord], int, list[TimingRecord], list[ControlRecord]]:
    """parse_run_reply_all without the payload records."""
    return parse_run_reply_all(reply)[:4]


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
