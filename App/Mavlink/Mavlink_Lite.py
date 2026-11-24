#!/usr/bin/env python3
"""
Mavlink_Lite.py  (lives in Flexlink_py/App/Mavlink)

A small MAVLink implementation for Flexlink: builds MAVLink 2 frames for a handful of common
messages, splits a byte stream into MAVLink 1 / 2 frames (checking the checksum of the messages it
knows), and reads and writes .tlog files. Standard library only, so it needs no pymavlink.

Messages
--------
MESSAGES below lists the messages this module can build and decode, in the order of their
definition in MAVLink's common.xml (extension fields left out). The wire order (fields sorted by
type size) and the CRC_EXTRA byte of every message are worked out from that list, and checked
against the published CRC_EXTRA values (KNOWN_CRC_EXTRA) when the module loads, so a typo in a
field list is caught at once. To add a message, add its fields from common.xml and its CRC_EXTRA
from the MAVLink documentation.

Frames of messages not in the list are still split out of a stream and passed on unchanged (their
checksum cannot be checked without the message's CRC_EXTRA), so a stream from a real flight
controller goes through whole.

    MAVLink 2:  0xFD len incompat compat seq sysid compid msgid(3, LE) payload crc(2) [signature 13]
    MAVLink 1:  0xFE len seq sysid compid msgid payload crc(2)

.tlog (as written by QGroundControl and Mission Planner): every frame is preceded by its receive
time, microseconds since 1970 as a big-endian uint64.

Self-test: python Mavlink_Lite.py --selftest
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

STX_V1 = 0xFE
STX_V2 = 0xFD
HEADER_V1 = 6
HEADER_V2 = 10
SIGNATURE_LEN = 13
IFLAG_SIGNED = 0x01

# type -> (struct format character, size in bytes)
TYPES = {
    "uint8_t": ("B", 1), "int8_t": ("b", 1), "char": ("s", 1),
    "uint16_t": ("H", 2), "int16_t": ("h", 2),
    "uint32_t": ("I", 4), "int32_t": ("i", 4), "float": ("f", 4),
    "uint64_t": ("Q", 8), "int64_t": ("q", 8), "double": ("d", 8),
}

# name: (id, [(field type, field name, array length or 0), ...] in common.xml order)
MESSAGES = {
    "HEARTBEAT": (0, [
        ("uint8_t", "type", 0), ("uint8_t", "autopilot", 0), ("uint8_t", "base_mode", 0),
        ("uint32_t", "custom_mode", 0), ("uint8_t", "system_status", 0), ("uint8_t", "mavlink_version", 0)]),
    "SYS_STATUS": (1, [
        ("uint32_t", "onboard_control_sensors_present", 0), ("uint32_t", "onboard_control_sensors_enabled", 0),
        ("uint32_t", "onboard_control_sensors_health", 0), ("uint16_t", "load", 0),
        ("uint16_t", "voltage_battery", 0), ("int16_t", "current_battery", 0), ("int8_t", "battery_remaining", 0),
        ("uint16_t", "drop_rate_comm", 0), ("uint16_t", "errors_comm", 0), ("uint16_t", "errors_count1", 0),
        ("uint16_t", "errors_count2", 0), ("uint16_t", "errors_count3", 0), ("uint16_t", "errors_count4", 0)]),
    "GPS_RAW_INT": (24, [
        ("uint64_t", "time_usec", 0), ("uint8_t", "fix_type", 0), ("int32_t", "lat", 0), ("int32_t", "lon", 0),
        ("int32_t", "alt", 0), ("uint16_t", "eph", 0), ("uint16_t", "epv", 0), ("uint16_t", "vel", 0),
        ("uint16_t", "cog", 0), ("uint8_t", "satellites_visible", 0)]),
    "ATTITUDE": (30, [
        ("uint32_t", "time_boot_ms", 0), ("float", "roll", 0), ("float", "pitch", 0), ("float", "yaw", 0),
        ("float", "rollspeed", 0), ("float", "pitchspeed", 0), ("float", "yawspeed", 0)]),
    "GLOBAL_POSITION_INT": (33, [
        ("uint32_t", "time_boot_ms", 0), ("int32_t", "lat", 0), ("int32_t", "lon", 0), ("int32_t", "alt", 0),
        ("int32_t", "relative_alt", 0), ("int16_t", "vx", 0), ("int16_t", "vy", 0), ("int16_t", "vz", 0),
        ("uint16_t", "hdg", 0)]),
    "VFR_HUD": (74, [
        ("float", "airspeed", 0), ("float", "groundspeed", 0), ("int16_t", "heading", 0),
        ("uint16_t", "throttle", 0), ("float", "alt", 0), ("float", "climb", 0)]),
    "STATUSTEXT": (253, [("uint8_t", "severity", 0), ("char", "text", 50)]),
}

# CRC_EXTRA as published for these messages (mavlink.io, common.xml); checked against the field lists
KNOWN_CRC_EXTRA = {0: 50, 1: 124, 24: 24, 30: 39, 33: 104, 74: 20, 253: 83}

# Some enum values used by the simulated telemetry and the stats line
MAV_TYPE_QUADROTOR = 2
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_STATE_STANDBY, MAV_STATE_ACTIVE = 3, 4
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_SEVERITY = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR", 4: "WARNING", 5: "NOTICE", 6: "INFO",
                7: "DEBUG"}
COPTER_MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}


# --------------------------------------------------------------------------- #
# CRC (CRC-16/MCRF4XX, "X.25" in MAVLink)
# --------------------------------------------------------------------------- #
def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


@dataclass(frozen=True)
class MessageDef:
    name: str
    msgid: int
    fields: tuple          # wire order: (type, name, array length)
    fmt: str               # struct format of the full payload, little endian
    length: int            # full payload length
    crc_extra: int


def _build(name: str, msgid: int, xml_fields: list) -> MessageDef:
    # wire order: by type size, largest first; equal sizes keep the XML order (a stable sort)
    wire = tuple(sorted(xml_fields, key=lambda f: -TYPES[f[0]][1]))
    crc = x25_crc((name + " ").encode())
    fmt = "<"
    for typ, fname, n in wire:
        crc = x25_crc((typ + " ").encode(), crc)
        crc = x25_crc((fname + " ").encode(), crc)
        if n:
            crc = x25_crc(bytes([n]), crc)
        ch = TYPES[typ][0]
        fmt += f"{n}s" if typ == "char" else (f"{n}{ch}" if n else ch)
    extra = (crc & 0xFF) ^ (crc >> 8)
    return MessageDef(name, msgid, wire, fmt, struct.calcsize(fmt), extra)


DEFS_BY_NAME = {name: _build(name, mid, f) for name, (mid, f) in MESSAGES.items()}
DEFS_BY_ID = {d.msgid: d for d in DEFS_BY_NAME.values()}
for _d in DEFS_BY_NAME.values():
    if KNOWN_CRC_EXTRA.get(_d.msgid, _d.crc_extra) != _d.crc_extra:
        raise RuntimeError(f"{_d.name}: CRC_EXTRA {_d.crc_extra} from the field list, "
                           f"{KNOWN_CRC_EXTRA[_d.msgid]} expected: check the fields")


# --------------------------------------------------------------------------- #
# Building frames
# --------------------------------------------------------------------------- #
class MavlinkEncoder:
    """Builds MAVLink 2 frames from one system / component, with its own sequence counter."""

    def __init__(self, sysid: int = 1, compid: int = 1):
        self.sysid, self.compid = int(sysid), int(compid)
        self.seq = 0

    def frame(self, name: str, **values) -> bytes:
        d = DEFS_BY_NAME[name]
        args = []
        for typ, fname, n in d.fields:
            v = values.get(fname, b"" if typ == "char" else ([0] * n if n else 0))
            if typ == "char":
                v = v.encode() if isinstance(v, str) else bytes(v)
                args.append(v[:n])
            elif n:
                args.extend(v)
            else:
                args.append(float(v) if typ in ("float", "double") else int(v))
        payload = struct.pack(d.fmt, *args)
        n = len(payload)
        while n > 1 and payload[n - 1] == 0:      # MAVLink 2 trims trailing zero bytes (one stays)
            n -= 1
        payload = payload[:n]
        hdr = bytes([STX_V2, n, 0, 0, self.seq, self.sysid, self.compid,
                     d.msgid & 0xFF, (d.msgid >> 8) & 0xFF, d.msgid >> 16])
        self.seq = (self.seq + 1) & 0xFF
        crc = x25_crc(bytes([d.crc_extra]), x25_crc(hdr[1:] + payload))
        return hdr + payload + struct.pack("<H", crc)


# --------------------------------------------------------------------------- #
# Reading frames
# --------------------------------------------------------------------------- #
@dataclass
class Frame:
    raw: bytes             # the whole frame, as received
    version: int           # 1 or 2
    seq: int
    sysid: int
    compid: int
    msgid: int
    payload: bytes         # as sent (MAVLink 2: may be trimmed)
    checked: bool          # checksum checked (the message is in MESSAGES)

    @property
    def name(self) -> str:
        d = DEFS_BY_ID.get(self.msgid)
        return d.name if d else f"MSG_{self.msgid}"

    def decode(self) -> dict | None:
        """The message's fields as a dict, or None for a message not in MESSAGES."""
        d = DEFS_BY_ID.get(self.msgid)
        if d is None:
            return None
        vals = struct.unpack(d.fmt, self.payload[:d.length].ljust(d.length, b"\0"))
        out, i = {}, 0
        for typ, fname, n in d.fields:
            if typ == "char":
                out[fname] = vals[i].split(b"\0", 1)[0].decode("ascii", "replace")
                i += 1
            elif n:
                out[fname] = list(vals[i:i + n])
                i += n
            else:
                out[fname] = vals[i]
                i += 1
        return out


def frame_length(buf, i: int = 0) -> int | None:
    """Length of the frame starting at buf[i] (from its header), or None if the header is not
    complete yet. buf[i] must be a start byte."""
    if buf[i] == STX_V2:
        if len(buf) - i < 3:
            return None
        return HEADER_V2 + buf[i + 1] + 2 + (SIGNATURE_LEN if buf[i + 2] & IFLAG_SIGNED else 0)
    if len(buf) - i < 2:
        return None
    return HEADER_V1 + buf[i + 1] + 2


def parse_frame(raw: bytes) -> Frame | None:
    """A Frame from exactly one frame's bytes; None when the checksum of a known message fails."""
    if raw[0] == STX_V2:
        n = raw[1]
        seq, sysid, compid = raw[4], raw[5], raw[6]
        msgid = raw[7] | (raw[8] << 8) | (raw[9] << 16)
        hl, ver = HEADER_V2, 2
    else:
        n = raw[1]
        seq, sysid, compid, msgid = raw[2], raw[3], raw[4], raw[5]
        hl, ver = HEADER_V1, 1
    payload = raw[hl:hl + n]
    d = DEFS_BY_ID.get(msgid)
    if d is not None:
        crc = x25_crc(bytes([d.crc_extra]), x25_crc(raw[1:hl + n]))
        if crc != struct.unpack_from("<H", raw, hl + n)[0]:
            return None
    return Frame(raw, ver, seq, sysid, compid, msgid, payload, d is not None)


class MavlinkParser:
    """Splits a byte stream into frames. Bytes before a start byte, and a frame whose checksum
    fails, are skipped (the search goes on one byte after the bad frame's start byte)."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad_crc = 0
        self.skipped_bytes = 0

    def reset(self) -> None:
        """Forget a partial frame (call after a gap in the stream)."""
        self.skipped_bytes += len(self.buf)
        self.buf.clear()

    def feed(self, data: bytes) -> list[Frame]:
        self.buf += data
        out = []
        b = self.buf
        while b:
            if b[0] not in (STX_V1, STX_V2):
                i = next((k for k, x in enumerate(b) if x in (STX_V1, STX_V2)), len(b))
                self.skipped_bytes += i
                del b[:i]
                continue
            n = frame_length(b)
            if n is None or len(b) < n:
                break
            f = parse_frame(bytes(b[:n]))
            if f is None:
                self.bad_crc += 1
                self.skipped_bytes += 1
                del b[:1]
                continue
            out.append(f)
            self.frames += 1
            del b[:n]
        return out


# --------------------------------------------------------------------------- #
# .tlog files
# --------------------------------------------------------------------------- #
def write_tlog(path, records) -> int:
    """records: (time_us, frame bytes) pairs. Returns the number written."""
    n = 0
    with open(path, "wb") as f:
        for t_us, raw in records:
            f.write(struct.pack(">Q", int(t_us)) + raw)
            n += 1
    return n


def read_tlog(path) -> list[tuple[int, bytes]]:
    """[(time_us, frame bytes), ...]. Stops at the first record that does not look like one."""
    data = Path(path).read_bytes()
    out, i = [], 0
    while i + 8 < len(data):
        t_us = struct.unpack_from(">Q", data, i)[0]
        j = i + 8
        if data[j] not in (STX_V1, STX_V2):
            break
        n = frame_length(data, j)
        if n is None or j + n > len(data):
            break
        out.append((t_us, data[j:j + n]))
        i = j + n
    return out


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #
def selftest() -> None:
    # the CRC-16/MCRF4XX check value
    assert x25_crc(b"123456789") == 0x6F91, hex(x25_crc(b"123456789"))
    enc = MavlinkEncoder(sysid=7, compid=1)
    hb = enc.frame("HEARTBEAT", type=2, autopilot=3, base_mode=217, custom_mode=3, system_status=4,
                   mavlink_version=3)
    st = enc.frame("STATUSTEXT", severity=6, text="Flexlink test")
    att = enc.frame("ATTITUDE", time_boot_ms=1234, roll=0.1, pitch=-0.05, yaw=1.5)
    # payload sizes: full lengths of the definitions
    assert DEFS_BY_NAME["HEARTBEAT"].length == 9 and DEFS_BY_NAME["GLOBAL_POSITION_INT"].length == 28
    assert DEFS_BY_NAME["SYS_STATUS"].length == 31 and DEFS_BY_NAME["GPS_RAW_INT"].length == 30
    assert DEFS_BY_NAME["VFR_HUD"].length == 20 and DEFS_BY_NAME["STATUSTEXT"].length == 51
    # zero trimming: STATUSTEXT carries 1 + 13 bytes
    assert st[1] == 14, st[1]
    # a stream with garbage, a corrupted frame and a split frame
    bad = bytearray(att)
    bad[12] ^= 0xFF
    stream = b"\x00\x13garbage" + hb + bytes(bad) + st + att
    p = MavlinkParser()
    got = p.feed(stream[:25]) + p.feed(stream[25:])
    assert [f.name for f in got] == ["HEARTBEAT", "STATUSTEXT", "ATTITUDE"], [f.name for f in got]
    assert p.bad_crc == 1
    d = got[0].decode()
    assert d["base_mode"] == 217 and d["custom_mode"] == 3 and d["mavlink_version"] == 3
    assert got[1].decode()["text"] == "Flexlink test"
    a = got[2].decode()
    assert abs(a["roll"] - 0.1) < 1e-6 and a["time_boot_ms"] == 1234 and a["yawspeed"] == 0.0
    # an unknown message passes through unchecked
    unk = bytes([STX_V2, 2, 0, 0, 0, 1, 1, 0x16, 0, 0, 0xAA, 0xBB, 0x12, 0x34])  # msgid 22 (PARAM_VALUE)
    f = MavlinkParser().feed(unk)
    assert len(f) == 1 and not f[0].checked and f[0].msgid == 22
    # tlog round trip
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        pth = Path(td) / "t.tlog"
        write_tlog(pth, [(1_000_000, hb), (1_100_000, att)])
        r = read_tlog(pth)
        assert r == [(1_000_000, hb), (1_100_000, att)]
    for d in DEFS_BY_NAME.values():
        print(f"  {d.name:20s} id {d.msgid:3d}  payload {d.length:2d} bytes  CRC_EXTRA {d.crc_extra:3d}")
    print("selftest passed")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        selftest()
    else:
        print(__doc__)
