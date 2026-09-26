"""Hover 40 m in steady wind: position hold, attitude. usage: hover_wind.py <inst> <name> <wind_spd> <q_angle_max_cd>"""
import sys, math, time
from sitllib import *
inst, name, ws, amax = int(sys.argv[1]), sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
spin_max = float(sys.argv[5]) if len(sys.argv) > 5 else 0.6
CAL = EXP + ("/tailsitter_cal.json" if spin_max == 0.6 else "/tailsitter_cal_sm95.json")
if os.environ.get("MODEL"):
    CAL = EXP + "/" + os.environ["MODEL"]
ex = {"AIRSPEED_CRUISE": 22, "Q_GUIDED_MODE": 1, "Q_ANGLE_MAX": amax, "SIM_WIND_T": 1, "Q_M_SPIN_MAX": spin_max}
s = Sitl(name, inst=inst, speedup=int(os.environ.get("SPEEDUP", 8)), model_json=CAL, extra=ex)
time.sleep(1)
v = V(s); v.wait_ready(150); v.takeoff_hover(40)
home = (v.st["lat"], v.st["lon"])
v.set_param("SIM_WIND_DIR", 118); v.set_param("SIM_WIND_SPD", ws)
v.pump(15)   # wind builds up (TC 5 s)
rows = []
t0 = v.st["t"]
while v.st["t"] - t0 < 40:
    v.pump(0.2, log=True)
    rows.append((dist_bearing(home[0], home[1], v.st["lat"], v.st["lon"])[0], v.st.get("pitch", 0), v.st.get("roll", 0),
                 v.st.get("yaw", 0), v.st.get("alt", 0), (v.st.get("mot") or [0])))
d = [r[0] for r in rows]
mot = [max(r[5]) for r in rows if r[5] and r[5][0]]
sat = sum(1 for r in rows if r[5] and r[5][0] and (max(r[5]) >= 1595 or min(r[5]) <= 1205)) / max(1, len(rows))
print("SAT %s model=%s frac_at_limits=%.3f" % (name, os.environ.get("MODEL", "cal"), sat), flush=True)
print("RESULT %s spin_max=%.2f wind=%.0f amax=%d drift_end=%.1f drift_max=%.1f pitch %.0f..%.0f roll %.0f..%.0f yaw %.0f..%.0f alt %.1f..%.1f motmax=%d" % (
    name, spin_max, ws, amax, d[-1], max(d), min(r[1] for r in rows), max(r[1] for r in rows), min(r[2] for r in rows),
    max(r[2] for r in rows), min(r[3] for r in rows), max(r[3] for r in rows), min(r[4] for r in rows),
    max(r[4] for r in rows), max(mot) if mot else 0), flush=True)
s.stop()
