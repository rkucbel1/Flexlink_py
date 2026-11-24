"""
Telemetry_Window.py  (lives in Flexlink_py/App/Mavlink)

The received MAVLink telemetry in a window of its own (FreeSimpleGUI), for the receiver apps:

    TelemetryState   collects the frames (in the receiver's thread) and gives snapshots of the
                     latest values (plain dicts, safe to hand to the GUI thread)
    TelemetryWindow  the window: artificial horizon, compass, the values (mode, position,
                     altitude, speeds, GPS, battery) and the vehicle's status messages.
                     The receiver app calls update(snapshot) a few times a second.

The drawing (horizon_shapes, compass_shapes) is plain geometry, so it can be checked without a
display: python Telemetry_Window.py --selftest
"""
from __future__ import annotations

import math
import sys
import time
from collections import Counter, deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import Mavlink_Lite as ML  # noqa: E402

STALE_S = 2.0                 # telemetry older than this is shown as lost
PITCH_SPAN = 30.0             # degrees of pitch from the center to the edge of the horizon

MAV_TYPES = {0: "generic", 1: "fixed wing", 2: "quadrotor", 3: "coaxial", 4: "helicopter", 10: "ground rover",
             11: "boat", 12: "submarine", 13: "hexarotor", 14: "octorotor", 15: "tricopter", 19: "VTOL",
             20: "VTOL", 21: "VTOL", 22: "VTOL"}
MULTIROTORS = {2, 3, 4, 13, 14, 15}
AUTOPILOTS = {0: "generic", 3: "ArduPilot", 8: "no autopilot", 12: "PX4"}
SYSTEM_STATUS = {0: "uninit", 1: "booting", 2: "calibrating", 3: "standby", 4: "active", 5: "CRITICAL",
                 6: "EMERGENCY", 7: "power off", 8: "TERMINATION"}
GPS_FIX = {0: "no GPS", 1: "no fix", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed",
           7: "static", 8: "PPP"}
SEVERITY_COLORS = {0: "red", 1: "red", 2: "red", 3: "red", 4: "#b36b00"}


# --------------------------------------------------------------------------- #
# State (receiver thread) and snapshots (GUI thread)
# --------------------------------------------------------------------------- #
class TelemetryState:
    def __init__(self, n_messages: int = 12):
        self.last: dict[str, dict] = {}
        self.counts: Counter = Counter()
        self.frames = 0
        self.t_last: float | None = None
        self.sysid: int | None = None
        self.messages: deque = deque(maxlen=n_messages)      # (clock time, severity, text)
        self._rate: deque = deque(maxlen=12)                 # (t, frames)

    def add(self, f: ML.Frame, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        self.frames += 1
        self.counts[f.name] += 1
        self.t_last = now
        d = f.decode()
        if d is None:
            return
        if f.name == "HEARTBEAT":
            self.sysid = f.sysid
        self.last[f.name] = d
        if f.name == "STATUSTEXT":
            self.messages.append((time.strftime("%H:%M:%S"), d["severity"], d["text"]))

    def snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        self._rate.append((now, self.frames))
        (ta, fa), (tb, fb) = self._rate[0], self._rate[-1]
        rate = (fb - fa) / (tb - ta) if tb > ta else 0.0
        return {"last": {k: dict(v) for k, v in self.last.items()}, "frames": self.frames, "rate": rate,
                "age": None if self.t_last is None else now - self.t_last, "sysid": self.sysid,
                "messages": list(self.messages), "counts": dict(self.counts)}


# --------------------------------------------------------------------------- #
# Geometry (graph coordinates -1 .. 1, y up)
# --------------------------------------------------------------------------- #
def _rot(x: float, y: float, a: float) -> tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


def horizon_shapes(roll: float, pitch: float) -> dict:
    """roll, pitch in radians (MAVLink ATTITUDE: roll right and pitch up positive). Seen from the
    cockpit, a roll to the right turns the horizon counterclockwise (its right end goes up), and
    the horizon moves down when the nose goes up."""
    off = -math.degrees(pitch) / PITCH_SPAN              # horizon position along the aircraft's up axis
    a = roll                                             # counterclockwise in the picture for roll right
    ground = [_rot(x, y + off, a) for x, y in ((-4, 0), (4, 0), (4, -4), (-4, -4))]
    ladder = []
    for p in (-30, -20, -10, 10, 20, 30):
        y = off + p / PITCH_SPAN
        w = 0.28 if p % 20 == 0 else 0.18
        ladder.append((_rot(-w, y, a), _rot(w, y, a), _rot(w + 0.12, y, a), f"{p:+d}"))
    return {"ground": ground, "horizon": (_rot(-4, off, a), _rot(4, off, a)), "ladder": ladder}


def compass_shapes(heading_deg: float) -> dict:
    """Map style: north up, the arrow points along the heading (clockwise from north)."""
    h = math.radians(heading_deg)
    v = (math.sin(h), math.cos(h))
    n = (-v[1], v[0])
    tip = (0.75 * v[0], 0.75 * v[1])
    base = (0.45 * v[0], 0.45 * v[1])
    return {"shaft": ((-0.55 * v[0], -0.55 * v[1]), base),
            "head": [tip, (base[0] + 0.15 * n[0], base[1] + 0.15 * n[1]),
                     (base[0] - 0.15 * n[0], base[1] - 0.15 * n[1])]}


# --------------------------------------------------------------------------- #
# The window
# --------------------------------------------------------------------------- #
VALUE_ROWS = [("vehicle", "Vehicle"), ("mode", "Mode"), ("link", "Link"), ("pos", "Position"),
              ("alt", "Altitude"), ("speed", "Speed"), ("hdg", "Heading"), ("att", "Attitude"),
              ("gps", "GPS"), ("batt", "Battery"), ("thr", "Throttle")]
SKY, GROUND, DARK = "#3b7dd8", "#8b5a2b", "#1e1e1e"


class TelemetryWindow:
    def __init__(self, sg, title: str = "Flexlink telemetry", font=("Helvetica", 11)):
        self.sg = sg
        self.title = title
        self.font = font
        self.window = None
        self._shown_msgs = None

    def _layout(self):
        sg = self.sg
        rows = [[sg.Text(label, size=(9, 1), justification="right", font=self.font),
                 sg.Text("-", key=f"-TV_{key}-", size=(46, 1), font=self.font)] for key, label in VALUE_ROWS]
        graphs = sg.Column([
            [sg.Graph((260, 260), (-1, -1), (1, 1), background_color=SKY, key="-TG_HORIZON-")],
            [sg.Graph((260, 150), (-1.6, -1.15), (1.6, 1.15), background_color=DARK, key="-TG_COMPASS-")],
        ], vertical_alignment="top")
        values = sg.Column(rows + [
            [sg.Text("Messages", font=self.font)],
            [sg.Multiline("", key="-TV_MSGS-", size=(60, 9), disabled=True, autoscroll=True,
                          font=("Courier", 10))],
        ], vertical_alignment="top")
        return [[graphs, values]]

    @property
    def is_open(self) -> bool:
        return self.window is not None

    def open(self) -> None:
        if self.window is None:
            self.window = self.sg.Window(self.title, self._layout(), finalize=True, resizable=True)
            self._shown_msgs = None
            self._draw_horizon(None)
            self._draw_compass(None)
        else:
            self.window.bring_to_front()

    def close(self) -> None:
        if self.window is not None:
            self.window.close()
            self.window = None

    # ---- drawing ----
    def _draw_horizon(self, att: dict | None, stale: bool = False) -> None:
        g = self.window["-TG_HORIZON-"]
        g.erase()
        if att is None:
            g.draw_rectangle((-1, 1), (1, -1), fill_color="#555555")
            g.draw_text("no attitude", (0, 0), color="white", font=("Helvetica", 14))
            return
        s = horizon_shapes(att["roll"], att["pitch"])
        g.draw_rectangle((-1, 1), (1, -1), fill_color=SKY)
        g.draw_polygon(s["ground"], fill_color=GROUND, line_color=GROUND)
        g.draw_line(*s["horizon"], color="white", width=2)
        for p1, p2, lab, txt in s["ladder"]:
            g.draw_line(p1, p2, color="white", width=1)
            g.draw_text(txt, lab, color="white", font=("Helvetica", 8))
        for seg in (((-0.6, 0), (-0.2, 0)), ((-0.2, 0), (-0.2, -0.07)), ((0.2, 0), (0.6, 0)),
                    ((0.2, 0), (0.2, -0.07))):
            g.draw_line(*seg, color="yellow", width=4)
        g.draw_circle((0, 0), 0.03, fill_color="yellow", line_color="yellow")
        g.draw_text(f"roll {math.degrees(att['roll']):+5.1f}°   pitch {math.degrees(att['pitch']):+5.1f}°",
                    (0, 0.88), color="white", font=("Helvetica", 10, "bold"))
        if stale:
            g.draw_rectangle((-0.75, 0.5), (0.75, 0.2), fill_color="red", line_color="white")
            g.draw_text("NO TELEMETRY", (0, 0.35), color="white", font=("Helvetica", 14, "bold"))

    def _draw_compass(self, heading: float | None) -> None:
        g = self.window["-TG_COMPASS-"]
        g.erase()
        g.draw_circle((0, 0), 1.0, line_color="white", line_width=2)
        for d in range(0, 360, 30):
            a = math.radians(d)
            r0 = 0.88 if d % 90 else 0.8
            g.draw_line((r0 * math.sin(a), r0 * math.cos(a)), (math.sin(a), math.cos(a)), color="white")
        for lab, d in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            a = math.radians(d)
            g.draw_text(lab, (0.65 * math.sin(a), 0.65 * math.cos(a)),
                        color="red" if lab == "N" else "white", font=("Helvetica", 9, "bold"))
        if heading is None:
            return
        s = compass_shapes(heading)
        g.draw_line(*s["shaft"], color="yellow", width=3)
        g.draw_polygon(s["head"], fill_color="yellow", line_color="yellow")
        g.draw_text(f"{heading:05.1f}°", (1.3, 0.95), color="white", font=("Helvetica", 10, "bold"))

    # ---- values ----
    def update(self, snap: dict) -> None:
        if self.window is None:
            return
        w = self.window
        last = snap["last"]
        age = snap["age"]
        stale = age is None or age > STALE_S
        hb, gp, att = last.get("HEARTBEAT"), last.get("GLOBAL_POSITION_INT"), last.get("ATTITUDE")
        hud, gps, ss = last.get("VFR_HUD"), last.get("GPS_RAW_INT"), last.get("SYS_STATUS")
        v = {}
        if hb:
            v["vehicle"] = (f"system {snap['sysid']}: {MAV_TYPES.get(hb['type'], hb['type'])}, "
                            f"{AUTOPILOTS.get(hb['autopilot'], hb['autopilot'])}")
            copter = hb["autopilot"] == ML.MAV_AUTOPILOT_ARDUPILOTMEGA and hb["type"] in MULTIROTORS
            mode = ML.COPTER_MODES.get(hb["custom_mode"]) if copter else None
            armed = "ARMED" if hb["base_mode"] & ML.MAV_MODE_FLAG_SAFETY_ARMED else "disarmed"
            v["mode"] = (f"{mode or 'custom mode ' + str(hb['custom_mode'])}, {armed}, "
                         f"{SYSTEM_STATUS.get(hb['system_status'], hb['system_status'])}")
        v["link"] = ("no telemetry yet" if age is None else
                     f"{snap['rate']:.0f} frames/s, last {age:.1f} s ago" + ("  - LOST" if stale else ""))
        if gp:
            v["pos"] = f"{gp['lat'] / 1e7:.7f}, {gp['lon'] / 1e7:.7f}"
            v["alt"] = f"{gp['relative_alt'] / 1e3:.1f} m above home, {gp['alt'] / 1e3:.1f} m above sea level"
            v["hdg"] = f"{gp['hdg'] / 100:.1f}°" if gp["hdg"] != 65535 else "-"
        if hud:
            v["speed"] = (f"ground {hud['groundspeed']:.1f} m/s, air {hud['airspeed']:.1f} m/s, "
                          f"climb {hud['climb']:+.1f} m/s")
            v["thr"] = f"{hud['throttle']} %"
            v.setdefault("hdg", f"{hud['heading']}°")
        if att:
            v["att"] = (f"roll {math.degrees(att['roll']):+.1f}°, pitch {math.degrees(att['pitch']):+.1f}°, "
                        f"yaw {math.degrees(att['yaw']) % 360:.1f}°")
        if gps:
            v["gps"] = (f"{GPS_FIX.get(gps['fix_type'], gps['fix_type'])}, {gps['satellites_visible']} satellites, "
                        f"HDOP {gps['eph'] / 100:.2f}" if gps["eph"] != 65535 else
                        f"{GPS_FIX.get(gps['fix_type'], gps['fix_type'])}, {gps['satellites_visible']} satellites")
        if ss:
            amps = f", {ss['current_battery'] / 100:.1f} A" if ss["current_battery"] >= 0 else ""
            pct = f", {ss['battery_remaining']} %" if ss["battery_remaining"] >= 0 else ""
            v["batt"] = f"{ss['voltage_battery'] / 1e3:.2f} V{amps}{pct}"
        for key, _label in VALUE_ROWS:
            w[f"-TV_{key}-"].update(v.get(key, "-"), text_color=(
                "red" if key == "link" and stale and age is not None else self.sg.theme_text_color()))
        self._draw_horizon(att, stale and age is not None)
        if gp and gp["hdg"] != 65535:
            self._draw_compass(gp["hdg"] / 100)
        elif hud:
            self._draw_compass(float(hud["heading"]))
        else:
            self._draw_compass(None)
        msgs = snap["messages"]
        if msgs != self._shown_msgs:                     # redrawn only when a message came in
            self._shown_msgs = msgs
            ml = w["-TV_MSGS-"]
            ml.update("")
            for clock, sev, text in msgs:
                ml.print(f"{clock} {ML.MAV_SEVERITY.get(sev, sev):9s} {text}",
                         text_color=SEVERITY_COLORS.get(sev, self.sg.theme_text_color()))


# --------------------------------------------------------------------------- #
# Self-test (geometry and state only)
# --------------------------------------------------------------------------- #
def selftest() -> None:
    # level: horizon through the middle, ground below
    s = horizon_shapes(0.0, 0.0)
    (x1, y1), (x2, y2) = s["horizon"]
    assert abs(y1) < 1e-9 and abs(y2) < 1e-9
    assert max(p[1] for p in s["ground"]) <= 1e-9
    # nose up 15 degrees: the horizon moves down by half the span
    (x1, y1), _ = horizon_shapes(0.0, math.radians(15))["horizon"]
    assert abs(y1 + 0.5) < 1e-9, y1
    # roll right 30 degrees: the horizon's right end is higher than its left end
    (x1, y1), (x2, y2) = horizon_shapes(math.radians(30), 0.0)["horizon"]
    assert x2 > x1 and y2 > y1 and abs(math.degrees(math.atan2(y2 - y1, x2 - x1)) - 30) < 1e-6
    # compass: heading 90 points east (+x)
    tip = compass_shapes(90.0)["head"][0]
    assert tip[0] > 0.7 and abs(tip[1]) < 1e-9
    # state from the simulated log
    st = TelemetryState()
    rec = ML.read_tlog(HERE / "Sim_Telemetry.tlog")
    p = ML.MavlinkParser()
    t = 0.0
    for _t_us, raw in rec[:2000]:
        for f in p.feed(raw):
            t += 0.03
            st.add(f, now=t)
    snap = st.snapshot(now=t + 0.1)
    assert snap["sysid"] == 1 and "ATTITUDE" in snap["last"] and snap["messages"]
    assert abs(snap["age"] - 0.1) < 1e-9
    print(f"frames {snap['frames']}, messages {[m[2] for m in snap['messages']]}")
    print("selftest passed")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        print(__doc__)
