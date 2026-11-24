#!/usr/bin/env python3
"""
Generate_Sim_Telemetry.py  (lives in Flexlink_py/App/Mavlink)

Writes a .tlog of simulated quadcopter telemetry (MAVLink 2, ArduCopter-style) for testing the
MAVLink stream of FlexlinkTxMavlinkWithPluto.py without a flight controller. The file also opens
in QGroundControl (Analyze > Log Replay) or Mission Planner (Telemetry Logs > Load Log), which is
an independent check that the frames are right.

The flight: on the ground disarmed, armed, take-off to the circle altitude (AUTO), laps of a
circle that starts and ends over home, return to home (RTL), landing (LAND), disarmed. Battery
voltage and charge follow the current drawn; GPS and attitude carry a little noise.

Messages and rates (the set Mavlink_Lite.py knows):
    HEARTBEAT 1 Hz, SYS_STATUS 1 Hz, GPS_RAW_INT 5 Hz, GLOBAL_POSITION_INT 10 Hz,
    ATTITUDE 10 Hz, VFR_HUD 4 Hz, STATUSTEXT at the events (arming, take-off, laps, low battery ...)

Run with:  python App/Mavlink/Generate_Sim_Telemetry.py   (options: --help)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import Mavlink_Lite as ML  # noqa: E402

DEFAULT_OUT = HERE / "Sim_Telemetry.tlog"
RATES_HZ = {"HEARTBEAT": 1, "SYS_STATUS": 1, "GPS_RAW_INT": 5, "GLOBAL_POSITION_INT": 10, "ATTITUDE": 10,
            "VFR_HUD": 4}
DT = 0.01                                    # simulation step, s
G = 9.81
EARTH_R = 6_378_137.0
MODE_STABILIZE, MODE_AUTO, MODE_RTL, MODE_LAND = 0, 3, 6, 9
BASE_DISARMED, BASE_ARMED = 89, 89 | ML.MAV_MODE_FLAG_SAFETY_ARMED   # custom mode, stabilize, manual input
# sensors present / enabled / healthy: gyro, accel, mag, baro, GPS, motors, RC, AHRS, battery
SENSORS = 0x01 | 0x02 | 0x04 | 0x08 | 0x20 | 0x8000 | 0x10000 | 0x200000 | 0x2000000


def simulate(duration: float, alt: float, radius: float, speed: float, seed: int):
    """Yields (t, state dict, events) every DT seconds."""
    rng = np.random.default_rng(seed)
    t_arm, t_takeoff = 8.0, 10.0
    climb, descend = 2.5, 1.5
    t_circle = t_takeoff + alt / climb
    # leave time for the return and the landing (the return is at most one diameter)
    t_return = max(t_circle + 10.0, duration - (2 * radius / speed + alt / descend + 12.0))
    x = y = z = 0.0                          # north, east, up (m) from home
    vx = vy = vz = 0.0
    yaw = 0.0
    phase = "ground"
    mode, armed = MODE_STABILIZE, False
    cap_mah, used_mah = 2700.0, 0.0
    laps = 0
    low_batt = False
    t = 0.0
    yield 0.0, None, [(6, "ArduCopter V4.5.7 (simulated by Flexlink)"), (6, "Frame: QUAD/X")]
    while t < duration:
        ev = []
        if phase == "ground" and not armed and t >= t_arm and t < t_return:
            armed, mode = True, MODE_AUTO
            ev.append((6, "Arming motors"))
        if phase == "ground" and armed and t >= t_takeoff:
            phase = "takeoff"
            ev.append((6, f"Mission: 1 Takeoff to {alt:.0f} m"))
        if phase == "takeoff" and z >= alt:
            z, phase, t_lap = alt, "circle", t
            ev.append((6, "Mission: 2 Circle"))
        if phase == "circle" and t >= t_return:
            phase, mode = "return", MODE_RTL
            ev.append((6, "RTL: returning home"))
        if phase == "return" and math.hypot(x, y) < 0.5:
            phase, mode = "land", MODE_LAND
            ev.append((6, "Land: descending"))
        if phase == "land" and z <= 0.0:
            z, phase, armed, mode = 0.0, "landed", False, MODE_LAND
            ev.append((6, "Land complete, disarming"))
        # motion
        ax = ay = 0.0
        if phase == "takeoff":
            vx = vy = 0.0
            vz = climb
        elif phase == "circle":
            w = speed / radius
            a = w * (t - t_lap)                       # angle on the circle; center `radius` north of home
            if int(a / (2 * math.pi)) > laps:
                laps += 1
                ev.append((6, f"Circle: lap {laps} done"))
            x, y = radius - radius * math.cos(a), radius * math.sin(a)
            vx, vy, vz = speed * math.sin(a), speed * math.cos(a), 0.0
            ax, ay = w * speed * math.cos(a), -w * speed * math.sin(a)
        elif phase == "return":
            d = math.hypot(x, y)
            v = min(speed, d / 2 + 0.2)
            vx, vy, vz = -v * x / d, -v * y / d, 0.0
        elif phase == "land":
            vx = vy = 0.0
            vz = -descend if z > 2 else -0.5
        else:
            vx = vy = vz = 0.0
        if phase not in ("circle",):
            x += vx * DT
            y += vy * DT
        z = max(0.0, z + vz * DT)
        gs = math.hypot(vx, vy)
        new_yaw = math.atan2(vy, vx) if gs > 0.5 else yaw
        yaw_rate = math.remainder(new_yaw - yaw, 2 * math.pi) / DT
        yaw = new_yaw
        # attitude: bank for the turn, nose down for the drag, small wobble
        fwd = (ax * math.cos(yaw) + ay * math.sin(yaw))
        lat_acc = (-ax * math.sin(yaw) + ay * math.cos(yaw))
        flying = phase in ("takeoff", "circle", "return", "land")
        drag_pitch = -0.012 * gs * gs / G
        roll = math.atan2(lat_acc, G) + (rng.normal(0, 0.01) if flying else 0.0)
        pitch = drag_pitch - math.atan2(fwd, G) + (rng.normal(0, 0.01) if flying else 0.0)
        # power
        current = 0.6 if not armed else (6.0 if phase == "ground" else 16.0 + 0.25 * gs * gs + 4 * max(vz, 0))
        used_mah += current * DT / 3.6
        remaining = max(0.0, 1.0 - used_mah / cap_mah)
        voltage = 4 * (3.5 + 0.7 * remaining) - 0.012 * current + rng.normal(0, 0.005)
        if remaining < 0.3 and not low_batt:
            low_batt = True
            ev.append((4, "Battery 1 is low"))
        state = dict(x=x, y=y, z=z, vx=vx, vy=vy, vz=vz, yaw=yaw % (2 * math.pi), roll=roll, pitch=pitch,
                     yaw_rate=yaw_rate if flying else 0.0, gs=gs, mode=mode, armed=armed,
                     current=current, voltage=voltage, remaining=remaining,
                     throttle=0 if not armed else (12 if phase == "ground" else int(45 + 8 * vz + 0.3 * gs)),
                     gps_noise=rng.normal(0, 0.25, 3), sats=int(14 + rng.integers(-1, 2)))
        yield t, state, ev
        t = round(t + DT, 6)


def to_latlon(lat0: float, lon0: float, north: float, east: float) -> tuple[float, float]:
    lat = lat0 + math.degrees(north / EARTH_R)
    lon = lon0 + math.degrees(east / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def generate(out: Path, duration: float, lat0: float, lon0: float, home_amsl: float, alt: float,
             radius: float, speed: float, sysid: int, seed: int) -> dict:
    enc = ML.MavlinkEncoder(sysid=sysid, compid=1)
    t_unix0 = int(time.time()) * 1_000_000
    next_due = {k: 0.0 for k in RATES_HZ}
    records = []
    counts = {}

    def emit(t, name, **f):
        records.append((t_unix0 + int(round(t * 1e6)), enc.frame(name, **f)))
        counts[name] = counts.get(name, 0) + 1

    for t, s, events in simulate(duration, alt, radius, speed, seed):
        for sev, text in events:
            emit(t, "STATUSTEXT", severity=sev, text=text[:50])
        if s is None:
            continue
        ms = int(t * 1000)
        lat, lon = to_latlon(lat0, lon0, s["x"], s["y"])
        for name, hz in RATES_HZ.items():
            if t + 1e-9 < next_due[name]:
                continue
            next_due[name] += 1.0 / hz
            if name == "HEARTBEAT":
                emit(t, name, type=ML.MAV_TYPE_QUADROTOR, autopilot=ML.MAV_AUTOPILOT_ARDUPILOTMEGA,
                     base_mode=BASE_ARMED if s["armed"] else BASE_DISARMED, custom_mode=s["mode"],
                     system_status=ML.MAV_STATE_ACTIVE if s["armed"] else ML.MAV_STATE_STANDBY, mavlink_version=3)
            elif name == "SYS_STATUS":
                emit(t, name, onboard_control_sensors_present=SENSORS, onboard_control_sensors_enabled=SENSORS,
                     onboard_control_sensors_health=SENSORS, load=250 + (120 if s["armed"] else 0),
                     voltage_battery=int(s["voltage"] * 1000), current_battery=int(s["current"] * 100),
                     battery_remaining=int(round(100 * s["remaining"])))
            elif name == "GPS_RAW_INT":
                n = s["gps_noise"]
                glat, glon = to_latlon(lat0, lon0, s["x"] + n[0], s["y"] + n[1])
                emit(t, name, time_usec=t_unix0 + int(t * 1e6), fix_type=3, lat=int(glat * 1e7), lon=int(glon * 1e7),
                     alt=int((home_amsl + s["z"] + n[2]) * 1000), eph=80, epv=120, vel=int(s["gs"] * 100),
                     cog=int(math.degrees(s["yaw"]) * 100) % 36000, satellites_visible=s["sats"])
            elif name == "GLOBAL_POSITION_INT":
                emit(t, name, time_boot_ms=ms, lat=int(lat * 1e7), lon=int(lon * 1e7),
                     alt=int((home_amsl + s["z"]) * 1000), relative_alt=int(s["z"] * 1000),
                     vx=int(s["vx"] * 100), vy=int(s["vy"] * 100), vz=int(-s["vz"] * 100),
                     hdg=int(math.degrees(s["yaw"]) * 100) % 36000)
            elif name == "ATTITUDE":
                emit(t, name, time_boot_ms=ms, roll=s["roll"], pitch=s["pitch"],
                     yaw=math.remainder(s["yaw"], 2 * math.pi), rollspeed=0.0, pitchspeed=0.0,
                     yawspeed=s["yaw_rate"])
            elif name == "VFR_HUD":
                emit(t, name, airspeed=s["gs"], groundspeed=s["gs"], heading=int(math.degrees(s["yaw"])) % 360,
                     throttle=max(0, min(100, s["throttle"])), alt=home_amsl + s["z"], climb=s["vz"])
    out.parent.mkdir(parents=True, exist_ok=True)
    ML.write_tlog(out, records)
    span = (records[-1][0] - records[0][0]) / 1e6
    total = sum(len(r) for _, r in records)
    return {"frames": len(records), "seconds": span, "bytes": total, "counts": counts}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Write a .tlog of simulated quadcopter telemetry.")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"output file (default {DEFAULT_OUT.name})")
    ap.add_argument("--duration", type=float, default=300.0, help="seconds of flight log (default 300)")
    ap.add_argument("--lat", type=float, default=50.0755, help="home latitude, degrees")
    ap.add_argument("--lon", type=float, default=14.4378, help="home longitude, degrees")
    ap.add_argument("--home-amsl", type=float, default=250.0, help="home altitude above sea level, m")
    ap.add_argument("--alt", type=float, default=30.0, help="flight altitude above home, m")
    ap.add_argument("--radius", type=float, default=60.0, help="circle radius, m")
    ap.add_argument("--speed", type=float, default=6.0, help="ground speed on the circle, m/s")
    ap.add_argument("--sysid", type=int, default=1, help="MAVLink system id")
    ap.add_argument("--seed", type=int, default=1, help="noise seed")
    a = ap.parse_args(argv)
    if a.duration < 60:
        ap.error("--duration: at least 60 s (take-off, a little circling, return and landing)")
    r = generate(a.out, a.duration, a.lat, a.lon, a.home_amsl, a.alt, a.radius, a.speed, a.sysid, a.seed)
    print(f"Wrote {a.out}: {r['frames']} frames over {r['seconds']:.1f} s, "
          f"{r['bytes'] / r['seconds'] * 8 / 1e3:.1f} kbit/s on average")
    for k, v in sorted(r["counts"].items()):
        print(f"  {k:20s} {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
