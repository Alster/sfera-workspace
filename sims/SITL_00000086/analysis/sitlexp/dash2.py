"""Dash with SFERA-side track loop: GUIDED_CHANGE_HEADING at 5 Hz with cross-track correction.

usage: dash2.py <inst> <name> [thr] [kxt_deg_per_m] [exit_mode]
Hover 40 m -> VTOL step 25 m along the dash bearing (aligns the nose) -> DO_REPOSITION 3 km along the
bearing + DO_CHANGE_SPEED(thr) -> after FW: heading loop for 1500 m -> exit via <exit_mode> (LOITER|QLOITER)
-> GUIDED + DO_REPOSITION back to the ambush point -> hover. Logs xtrack along the line.
"""
import sys, math, json, time
from sitllib import *
from pymavlink import mavutil

BRG = 118.0
CAL = EXP + "/tailsitter_cal.json"
inst, name = int(sys.argv[1]), sys.argv[2]
thr = float(sys.argv[3]) if len(sys.argv) > 3 else 70
kxt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.3
exit_mode = sys.argv[5] if len(sys.argv) > 5 else "LOITER"
BENCH = {"AIRSPEED_CRUISE": 22, "AIRSPEED_MIN": 15, "AIRSPEED_MAX": 30, "Q_GUIDED_MODE": 1}
# WIND="speed,dir_deg,turb": SIM_WIND_* with no altitude profile (SIM_WIND_T=1), for reproducible tests
if os.environ.get("WIND"):
    ws, wd, wt = (float(x) for x in os.environ["WIND"].split(","))
    BENCH.update({"SIM_WIND_SPD": ws, "SIM_WIND_DIR": wd, "SIM_WIND_TURB": wt, "SIM_WIND_T": 1})
s = Sitl(name, inst=inst, speedup=int(os.environ.get("SPEEDUP", 8)), model_json=CAL, extra=BENCH)
time.sleep(1)
v = V(s)
v.wait_ready(150)
v.takeoff_hover(40)
home = (v.st["lat"], v.st["lon"])
hh = []
t_h = v.st["t"]
while v.st["t"] - t_h < 20:
    v.pump(0.2, log=True)
    hh.append((dist_bearing(home[0], home[1], v.st["lat"], v.st["lon"])[0], v.st.get("yaw"), v.st.get("alt")))
print("HOVER20 max_dev=%.1f m yaw %.0f..%.0f alt %.1f..%.1f" % (max(h[0] for h in hh), min(h[1] for h in hh),
      max(h[1] for h in hh), min(h[2] for h in hh), max(h[2] for h in hh)), flush=True)


def xt_along(lat, lon, org):
    d, b = dist_bearing(org[0], org[1], lat, lon)
    return d * math.sin(math.radians(b - BRG)), d * math.cos(math.radians(b - BRG))


GUST = os.environ.get("GUST")
gust = None
if GUST:
    ph, gs_, gd, gdur = GUST.split(",")
    gust = dict(phase=ph, spd=float(gs_), dir=float(gd), dur=float(gdur))


def gust_on(phase):
    """Scripted gust: SIM_WIND_TC 0.3 s, wind -> gust vector for dur s, then back (PARAM_SET on the fly)."""
    if not gust or gust["phase"] != phase:
        return None
    base = (float(os.environ.get("WIND", "0,0,0").split(",")[0]), float(os.environ.get("WIND", "0,0,0").split(",")[1]))
    v.set_param("SIM_WIND_TC", 0.3)
    v.set_param("SIM_WIND_DIR", gust["dir"])
    v.set_param("SIM_WIND_SPD", gust["spd"])
    mark(f"gust on {gust['spd']} m/s from {gust['dir']}")
    return (v.st["t"] + gust["dur"], base)


def gust_off(g):
    if g and v.st["t"] >= g[0]:
        v.set_param("SIM_WIND_SPD", g[1][0])
        v.set_param("SIM_WIND_DIR", g[1][1])
        mark("gust off")
        return None
    return g


def mark(l):
    v.rec.write(json.dumps({"t": v.st["t"], "mark": l}) + "\n"); print("MARK", l, round(v.st["t"], 1), flush=True)

# 1. align the nose: VTOL step 25 m along the dash bearing
amb = offset(home[0], home[1], 25, BRG)
mark("align step"); v.reposition(amb[0], amb[1], 40)
v.pump(60, until=lambda st: dist_bearing(st["lat"], st["lon"], amb[0], amb[1])[0] < 2 and st.get("gs", 9) < 0.5, every=5)
v.pump(5)
yaw0 = v.st.get("yaw"); print("yaw after align", yaw0, flush=True)
org = (v.st["lat"], v.st["lon"])
far = offset(org[0], org[1], 3000, BRG)
mark("dash start"); t0 = v.st["t"]
v.reposition(far[0], far[1], 40)
r = v.cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, -1, thr); print("change_speed", r, flush=True)
g = gust_on("dash")
v.pump(40, until=lambda st: (gust_off(g) or True) and st.get("vtol") == "FW", every=1)
t_fw = v.st["t"]; mark("FW")
xs = []
last = 0
while True:
    x, a = xt_along(v.st["lat"], v.st["lon"], org)
    if a > 1500:
        break
    if v.st["t"] - last >= 0.2:
        last = v.st["t"]
        hdg = BRG - max(-30.0, min(30.0, kxt * x))    # steer back to the line
        v.m.mav.command_int_send(v.m.target_system, v.m.target_component, 0,
                                 mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING, 0, 0,
                                 0, hdg % 360, 5, 0, 0, 0, 0)
        xs.append((round(v.st["t"] - t0, 1), round(a), round(x, 1), round(v.st.get("gs", 0), 1), round(v.st.get("alt", 0), 1)))
    v.pump(0.1, log=True)
    g = gust_off(g)
mark("dash end")
print("XTRACK", json.dumps(xs[::10]), flush=True)
after = [p for p in xs if p[0] > (t_fw - t0) + 5]
print("SUMMARY t_fw=%.1f max|xt| after FW+5s=%.1f max|xt| all=%.1f gs_end=%.1f" % (
    t_fw - t0, max(abs(p[2]) for p in after) if after else -1, max(abs(p[2]) for p in xs), xs[-1][3]), flush=True)
# exit heading mode
v.set_mode(exit_mode); mark("exit " + exit_mode)
v.pump(8, every=1)
v.set_mode("GUIDED"); v.reposition(amb[0], amb[1], 40); mark("return")
alt_max = [0]
def back(st):
    alt_max[0] = max(alt_max[0], st.get("alt", 0))
    return st.get("vtol") == "MC" and dist_bearing(st["lat"], st["lon"], amb[0], amb[1])[0] < 3 and st.get("gs", 9) < 0.5
tb = v.st["t"]
g = gust_on("return")
gw = [g]
def back2(st):
    gw[0] = gust_off(gw[0]) if gw[0] else None
    return back(st)
ok = v.pump(400, until=back2, every=5)
print("RETURN ok=%s dt=%.1f alt_max=%.1f" % (ok, v.st["t"] - tb, alt_max[0]), flush=True)
s.stop()
