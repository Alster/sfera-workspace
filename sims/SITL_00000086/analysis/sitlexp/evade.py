"""Evasion timing. usage: evade.py <inst> <name> <kind>
kind: side25 | sidedown | side_chain (hover: DO_REPOSITION away from a line, 25 m steps) | fw60 (dash, then
GUIDED_CHANGE_HEADING 60 deg off the line). Prints displacement from the start point vs time after the command."""
import sys, math, json, time
from sitllib import *
from pymavlink import mavutil
inst, name, kind = int(sys.argv[1]), sys.argv[2], sys.argv[3]
BRG = 118.0
CAL = EXP + "/" + os.environ.get("MODEL", "tailsitter_cal_d05.json")
ex = {"AIRSPEED_CRUISE": 22, "Q_GUIDED_MODE": 1, "Q_ANGLE_MAX": 3000, "AIRSPEED_MIN": 15, "AIRSPEED_MAX": 30}
s = Sitl(name, inst=inst, speedup=int(os.environ.get("SPEEDUP", 8)), model_json=CAL, extra=ex)
time.sleep(1)
v = V(s); v.wait_ready(150); v.takeoff_hover(40)
home = (v.st["lat"], v.st["lon"])
out = []


def track(secs, p0, alt0):
    t0 = v.st["t"]
    marks = {}
    while v.st["t"] - t0 < secs:
        v.pump(0.1, log=True)
        d = dist_bearing(p0[0], p0[1], v.st["lat"], v.st["lon"])[0]
        d3 = math.hypot(d, v.st.get("alt", alt0) - alt0)
        for th in (5, 10, 20, 30, 50, 80):
            if th not in marks and d3 >= th:
                marks[th] = round(v.st["t"] - t0, 2)
    return marks

if kind in ("side25", "sidedown", "side_chain"):
    # nose along the dash line first (as in the ambush), then evade to the right of the line
    a = offset(home[0], home[1], 25, BRG)
    v.reposition(a[0], a[1], 40)
    v.pump(60, until=lambda st: dist_bearing(st["lat"], st["lon"], a[0], a[1])[0] < 2 and st.get("gs", 9) < 0.4)
    v.pump(5)
    p0 = (v.st["lat"], v.st["lon"]); alt0 = v.st["alt"]
    tgt = offset(p0[0], p0[1], 25, BRG + 90)
    alt = 25 if kind == "sidedown" else 40
    t_cmd = v.st["t"]
    v.reposition(tgt[0], tgt[1], alt)
    if kind == "side_chain":
        marks = {}
        t0 = v.st["t"]; step = 0
        while v.st["t"] - t0 < 40:
            v.pump(0.1, log=True)
            d = dist_bearing(p0[0], p0[1], v.st["lat"], v.st["lon"])[0]
            for th in (5, 10, 20, 30, 50, 80):
                if th not in marks and d >= th:
                    marks[th] = round(v.st["t"] - t0, 2)
            dn = dist_bearing(v.st["lat"], v.st["lon"], tgt[0], tgt[1])[0]
            if dn < 12 and step < 6:
                step += 1
                tgt = offset(p0[0], p0[1], 25 * (step + 1), BRG + 90)
                v.reposition(tgt[0], tgt[1], alt)
    else:
        marks = track(30, p0, alt0)
    print("EVADE %s marks(m->s)=%s gs_max=%s" % (kind, json.dumps(marks), "-"), flush=True)
elif kind == "fw60":
    a = offset(home[0], home[1], 25, BRG)
    v.reposition(a[0], a[1], 40)
    v.pump(60, until=lambda st: dist_bearing(st["lat"], st["lon"], a[0], a[1])[0] < 2 and st.get("gs", 9) < 0.4)
    v.pump(5)
    org = (v.st["lat"], v.st["lon"])
    far = offset(org[0], org[1], 3000, BRG)
    v.reposition(far[0], far[1], 40)
    v.cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, -1, 70)
    v.pump(40, until=lambda st: st.get("vtol") == "FW")
    v.pump(8)  # established dash
    def xt():
        d, b = dist_bearing(org[0], org[1], v.st["lat"], v.st["lon"])
        return d * math.sin(math.radians(b - BRG))
    x0 = xt(); t0 = v.st["t"]; marks = {}
    last = 0
    while v.st["t"] - t0 < 25:
        if v.st["t"] - last >= 0.2:
            last = v.st["t"]
            v.m.mav.command_int_send(v.m.target_system, v.m.target_component, 0,
                                     mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING, 0, 0, 0, (BRG + 60) % 360, 5, 0, 0, 0, 0)
        v.pump(0.1, log=True)
        dx = abs(xt() - x0)
        for th in (5, 10, 20, 30, 50, 80, 120):
            if th not in marks and dx >= th:
                marks[th] = round(v.st["t"] - t0, 2)
    print("EVADE fw60 lateral marks(m->s)=%s" % json.dumps(marks), flush=True)
s.stop()
