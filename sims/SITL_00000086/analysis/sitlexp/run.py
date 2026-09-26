"""Scenario runner.  usage: run.py <scenario> <inst> [args...]"""
import sys, time, json, math
from sitllib import *
from pymavlink import mavutil

CAL = EXP + "/tailsitter_cal.json"
BENCH = {"AIRSPEED_CRUISE": 22, "AIRSPEED_MIN": 15, "AIRSPEED_MAX": 30}
BRG = 118.0
SPEED = int(os.environ.get('SPEEDUP', 8))


def stable(v, n=None):
    return v.st.get("vtol") == "MC" and v.st.get("gs", 9) < 0.5 and abs(v.st.get("vz", 9)) < 0.3


class StableTimer:
    def __init__(self, v, hold=5.0, maxd=None):
        self.v, self.hold, self.maxd, self.t0 = v, hold, maxd, None

    def __call__(self, st):
        ok = stable(self.v)
        if ok and self.maxd is not None:
            ok = self.v.snap().get("dtgt", 1e9) < self.maxd
        if ok:
            if self.t0 is None:
                self.t0 = st["t"]
            return st["t"] - self.t0 >= self.hold
        self.t0 = None
        return False


def setup(name, inst, extra):
    s = Sitl(name, inst=inst, speedup=SPEED, model_json=CAL, extra=extra)
    time.sleep(1)
    v = V(s)
    v.wait_ready(150)
    v.takeoff_hover(40)
    v.home0 = (v.st["lat"], v.st["lon"])
    return s, v


def mark(v, label):
    v.rec.write(json.dumps({"t": v.st["t"], "mark": label}) + "\n")
    print(f"  MARK {label} t={v.st['t']:.1f}", flush=True)


def sc_rep(v, dist, **kw):
    lat, lon = offset(v.home0[0], v.home0[1], dist, BRG)
    mark(v, f"reposition {dist}")
    v.reposition(lat, lon, 40)
    T = max(120, dist / 10 + 150) if not v.qgm else max(300, dist / 5 + 300)
    v.pump(T, until=StableTimer(v, 20, maxd=30) if v.qgm else None, every=2)
    mark(v, "end")


def sc_dash(v, thr, hdg=False, **kw):
    lat, lon = offset(v.home0[0], v.home0[1], 3000, BRG)
    mark(v, f"dash thr={thr}")
    v.reposition(lat, lon, 40)
    r = v.cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, -1, thr)
    print("  change_speed ->", r, flush=True)
    if hdg:
        # wait for FW, then command COG hold on the line bearing
        v.pump(60, until=lambda st: st.get("vtol") == "FW", every=2)
        r = v.cmd_int(mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING, 0, 0, BRG, 5)
        print("  GUIDED_CHANGE_HEADING ->", r, flush=True)
        mark(v, "change_heading")
    far = lambda st: dist_bearing(v.home0[0], v.home0[1], st["lat"], st["lon"])[0] >= 2000
    v.pump(250, until=far, every=2)
    mark(v, "return")
    v.reposition(v.home0[0], v.home0[1], 40)
    v.pump(400, until=StableTimer(v, 20, maxd=30), every=2)
    mark(v, "end")


def sc_land(v, mode, dist, fw=False, **kw):
    lat, lon = offset(v.home0[0], v.home0[1], dist, BRG)
    mark(v, f"goto {dist}")
    v.reposition(lat, lon, 40)
    if fw:
        # trigger landing mode while still flying FW toward the point
        v.pump(300, until=lambda st: st.get("vtol") == "FW" and st.get("gs", 0) > 15, every=2)
        v.pump(15, every=2)
    else:
        v.pump(dist / 10 + 200, until=StableTimer(v, 10, maxd=30), every=2)
    d0 = dist_bearing(v.home0[0], v.home0[1], v.st["lat"], v.st["lon"])[0]
    mark(v, f"{mode} from {d0:.0f} m")
    v.tgt = (v.home0[0], v.home0[1], 0)
    v.set_mode(mode)
    v.pump(600, until=lambda st: not st["armed"], every=3)
    v.pump(10, every=5)
    mark(v, "end")


def _vel(v, alt, dur):
    m = v.m
    t_end = v.st["t"] + dur
    while v.st["t"] < t_end:
        m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                  0b0000111111000111, 0, 0, alt, 5.0, 0, 0, 0, 0, 0, 0, 0)
        v.pump(0.5, every=2)


def sc_vel(v, **kw):
    m = v.m
    v.tgt = (v.home0[0], v.home0[1], 40)
    # alt=0 exactly (as SFERA sends) panics SITL (Location(0,0,0) check exists only in SITL build);
    # alt=0.01 is the hardware-equivalent proxy.
    mark(v, "vel-only vn=5 alt=40.0")
    _vel(v, 40.0, 20)
    v.pump(10, every=2)
    mark(v, "vel-only vn=5 alt=0.01 (HW proxy of SFERA alt=0)")
    _vel(v, 0.01, 25)
    mark(v, "stop vel; hold 15s")
    v.pump(15, every=2)
    mark(v, "reposition home alt 40")
    v.reposition(v.home0[0], v.home0[1], 40)
    v.pump(60, until=StableTimer(v, 10, maxd=10), every=2)
    mark(v, "pos-only 100m N alt=40 (SFERA send_pos_global_rel)")
    lat, lon = offset(v.home0[0], v.home0[1], 100, 0)
    t_end = v.st["t"] + 20
    while v.st["t"] < t_end:
        m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                  0b0000111111111000, int(lat * 1e7), int(lon * 1e7), 40.0,
                                                  0, 0, 0, 0, 0, 0, 0, 0)
        v.pump(0.5, every=2)
    v.pump(20, every=2)
    mark(v, "end")


def sc_pos(v, **kw):
    m = v.m
    v.tgt = (v.home0[0], v.home0[1], 40)
    lat, lon = offset(v.home0[0], v.home0[1], 100, 0)
    mark(v, "pos-only 100m N alt=55 (SFERA send_pos_global_rel style)")
    t_end = v.st["t"] + 30
    while v.st["t"] < t_end:
        m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                  0b0000111111111000, int(lat * 1e7), int(lon * 1e7), 55.0,
                                                  0, 0, 0, 0, 0, 0, 0, 0)
        v.pump(0.5, every=2)
    v.pump(20, every=2)
    mark(v, "SET_ATTITUDE_TARGET-free end")


if __name__ == "__main__":
    sc, inst = sys.argv[1], int(sys.argv[2])
    args = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    extra = dict(args.pop("extra", {}))
    if args.pop("bench", False):
        extra.update(BENCH)
    qgm = args.pop("qgm", 1)
    extra["Q_GUIDED_MODE"] = qgm
    name = args.pop("name")
    s, v = setup(name, inst, extra)
    v.qgm = qgm
    w0 = time.time(); t0 = v.st["t"]
    try:
        {"rep": sc_rep, "dash": sc_dash, "land": sc_land, "vel": sc_vel, "pos": sc_pos}[sc](v, **args)
    finally:
        print(f"RTF={(v.st['t'] - t0) / (time.time() - w0):.2f}", flush=True)
        s.stop()
