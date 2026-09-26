"""Helper for SITL experiments with the 00000086 quad tailsitter (ArduPlane 4.6.3)."""
import json, math, os, subprocess, sys, time, signal
from pymavlink import mavutil

EXP = os.path.dirname(os.path.abspath(__file__))
AP = os.path.expanduser("~/ardupilot/ardupilot-4.6.3")
BIN = AP + "/build/sitl/bin/arduplane"
SIMDIR = "/home/anikkhoma/repos/sfera/sfera-workspace/sims/SITL_00000086"
DEFAULTS = [AP + "/Tools/autotest/default_params/quadplane.parm",
            AP + "/Tools/autotest/default_params/quadplane-copter_tailsitter.parm",
            SIMDIR + "/00000086_sitl.param"]
HOME = (49.9480133, 25.5002564, 271.36, 118)

# experiment-only overrides (documented in report)
BASE_EXTRA = {
    "LOG_DISARMED": 0,
    "RELAY1_PIN": -1, "RELAY2_PIN": -1,      # SITL: pins 81/82 do not exist (prearm fail)
    "SIM_IMU_ORIENT": 0, "AHRS_ORIENTATION": 0,  # SITL: 35/35 do not cancel (both apply the same rotation) -> roll -90 on ground
    # SITL: SERVO1-4 FUNCTION=0 -> PWM 0 -> SIM_Plane reads throttle=-1 (reverse thrust, +20 A) and
    # surfaces at -3x full deflection. Passthrough of centred RC1/2/4 (1500) and RC3=1000 = "no surfaces, no fwd motor".
    "SERVO1_FUNCTION": 1, "SERVO2_FUNCTION": 1, "SERVO3_FUNCTION": 1, "SERVO4_FUNCTION": 1,
    "COMPASS_USE2": 0, "COMPASS_USE3": 0,    # SITL: extra internal compasses inconsistent (vehicle has 1 DroneCAN compass)
    "SR0_POSITION": 10, "SR0_EXTRA1": 10, "SR0_EXTRA2": 10, "SR0_EXT_STAT": 4,
    "SR0_RC_CHAN": 4, "SR0_EXTRA3": 4, "SR0_RAW_CTRL": 4,
}

VTOL = {0: "UNDEF", 1: "TR_FW", 2: "TR_MC", 3: "MC", 4: "FW"}


def dist_bearing(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    d = 2 * R * math.asin(math.sqrt(a))
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return d, (math.degrees(math.atan2(y, x)) + 360) % 360


def offset(lat, lon, dist, brg):
    R = 6371000.0
    b = math.radians(brg)
    dn, de = dist * math.cos(b), dist * math.sin(b)
    return lat + math.degrees(dn / R), lon + math.degrees(de / (R * math.cos(math.radians(lat))))


class Sitl:
    def __init__(self, name, inst=7, extra=None, speedup=1, model_json=None, frame="quadplane-copter_tailsitter-x"):
        self.name, self.inst, self.speedup = name, inst, speedup
        self.dir = os.path.join(EXP, "runs", name)
        subprocess.run(["rm", "-rf", self.dir])
        os.makedirs(self.dir)
        p = dict(BASE_EXTRA)
        p.update(extra or {})
        with open(os.path.join(self.dir, "exp.parm"), "w") as f:
            for k, v in p.items():
                f.write(f"{k} {v}\n")
        mj = model_json or (SIMDIR + "/tailsitter_00000086.json")
        # SITL AP_Filesystem strips the leading '/', so the model must be given relative to cwd
        subprocess.run(["cp", mj, os.path.join(self.dir, "model.json")], check=True)
        cmd = [BIN, "-w", f"-I{inst}", "--model", f"{frame}:model.json", "--speedup", str(speedup),
               "--home", ",".join(str(x) for x in HOME), "--serial0", "tcp:0",
               "--defaults", ",".join(DEFAULTS + [os.path.join(self.dir, "exp.parm")])]
        self.log = open(os.path.join(self.dir, "sitl.log"), "w")
        self.proc = subprocess.Popen(cmd, cwd=self.dir, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        self.port = 5760 + 10 * inst

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait(5)
        except Exception:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except Exception:
                pass


class V:
    def __init__(self, sitl, rc=True):
        self.s = sitl
        self.rc = rc
        for _ in range(60):
            try:
                self.m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{sitl.port}", source_system=255)
                break
            except Exception:
                time.sleep(0.5)
        else:
            raise RuntimeError("no connection; see " + sitl.dir + "/sitl.log")
        self.m.wait_heartbeat(timeout=60)
        self.mm = self.m.mode_mapping()
        self.inv = {v: k for k, v in self.mm.items()}
        self.st = {"mode": None, "vtol": None, "armed": False, "t": 0.0}
        self.rec = open(os.path.join(sitl.dir, "telem.jsonl"), "w")
        self.texts = []
        self.last_rc = 0
        self.rc_vals = [1500] * 8
        self.rc_vals[2] = 1000
        self.rc_vals[6] = 1500
        self.rc_vals[4] = 1500
        self.wall0 = time.time()
        for mid, hz in [(245, 5), (74, 10), (33, 10), (30, 10), (62, 5), (36, 5), (147, 2), (1, 2), (24, 2)]:
            self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz, wait=False)

    # --------------------------------------------------------------
    def send_rc(self):
        if not self.rc:
            return
        now = time.time()
        if now - self.last_rc > 0.2:
            self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *self.rc_vals)
            self.last_rc = now

    def handle(self, msg):
        t = msg.get_type()
        st = self.st
        if t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != 6:
            st["mode"] = self.inv.get(msg.custom_mode, msg.custom_mode)
            st["armed"] = bool(msg.base_mode & 128)
            st["mav_type"] = msg.type
        elif t == "EXTENDED_SYS_STATE":
            st["vtol"] = VTOL.get(msg.vtol_state, msg.vtol_state)
            st["landed"] = msg.landed_state
        elif t == "GLOBAL_POSITION_INT":
            st["t"] = msg.time_boot_ms / 1000.0
            st["lat"], st["lon"] = msg.lat / 1e7, msg.lon / 1e7
            st["alt"] = msg.relative_alt / 1000.0
            st["vz"] = msg.vz / 100.0
            st["hdg"] = msg.hdg / 100.0
            st["gs"] = math.hypot(msg.vx, msg.vy) / 100.0
            st["cog"] = (math.degrees(math.atan2(msg.vy, msg.vx)) + 360) % 360
        elif t == "VFR_HUD":
            st["as"] = msg.airspeed
            st["thr"] = msg.throttle
        elif t == "ATTITUDE":
            st["pitch"] = math.degrees(msg.pitch)
            st["roll"] = math.degrees(msg.roll)
            st["yaw"] = (math.degrees(msg.yaw) + 360) % 360
        elif t == "NAV_CONTROLLER_OUTPUT":
            st["wpd"] = msg.wp_dist
            st["alt_err"] = msg.alt_error
        elif t == "SERVO_OUTPUT_RAW":
            st["mot"] = [msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw]
        elif t == "BATTERY_STATUS":
            st["curr"] = msg.current_battery / 100.0
            st["volt"] = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None
        elif t == "STATUSTEXT":
            self.texts.append((st["t"], msg.text))
            self.rec.write(json.dumps({"t": st["t"], "text": msg.text}) + "\n")
            print(f"  [{st['t']:.1f}] ST: {msg.text}", flush=True)
        elif t == "COMMAND_ACK":
            self.last_ack = (msg.command, msg.result)

    def snap(self):
        d = {k: v for k, v in self.st.items() if k != "mav_type"}
        if hasattr(self, "tgt") and self.tgt and "lat" in d:
            d["dtgt"], _ = dist_bearing(d["lat"], d["lon"], self.tgt[0], self.tgt[1])
        return d

    def pump(self, secs, until=None, every=None, log=True):
        """Run for up to secs of SIM time. until(st)->bool stops early."""
        t_end = None
        last_log = -1e9
        wall_end = time.time() + secs / max(self.s.speedup, 1) * 3 + 30
        while True:
            self.send_rc()
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                self.handle(msg)
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    if t_end is None:
                        t_end = self.st["t"] + secs
                    if log:
                        self.rec.write(json.dumps(self.snap()) + "\n")
                    if every and self.st["t"] - last_log >= every:
                        last_log = self.st["t"]
                        print("  " + self.fmt(), flush=True)
                    if until and until(self.st):
                        return True
                    if self.st["t"] >= t_end:
                        return False
            if time.time() > wall_end:
                print("  WALL TIMEOUT", flush=True)
                return False

    def fmt(self):
        s = self.snap()
        g = lambda k, f="%.1f": (f % s[k]) if s.get(k) is not None else "-"
        return (f"t={g('t')} {s.get('mode')} vt={s.get('vtol')} arm={int(s.get('armed', 0))} alt={g('alt')} "
                f"vz={g('vz')} gs={g('gs')} as={g('as')} thr={s.get('thr')} hdg={g('hdg', '%.0f')} "
                f"cog={g("cog", "%.0f")} yaw={g("yaw", "%.0f")} p={g('pitch', '%.0f')} r={g('roll', '%.0f')} dT={g('dtgt', '%.0f')} "
                f"wpd={s.get('wpd')} I={g('curr')} mot={s.get('mot')}")

    def cmd(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait=True):
        self.last_ack = None
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0,
                                     p1, p2, p3, p4, p5, p6, p7)
        if not wait:
            return None
        t0 = time.time()
        while time.time() - t0 < 5:
            self.send_rc()
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg:
                self.handle(msg)
                if msg.get_type() == "COMMAND_ACK" and msg.command == command:
                    return msg.result
        return None

    def cmd_int(self, command, frame, p1=0, p2=0, p3=0, p4=0, x=0, y=0, z=0):
        self.m.mav.command_int_send(self.m.target_system, self.m.target_component, frame, command, 0, 0,
                                    p1, p2, p3, p4, x, y, z)
        t0 = time.time()
        while time.time() - t0 < 5:
            self.send_rc()
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg:
                self.handle(msg)
                if msg.get_type() == "COMMAND_ACK" and msg.command == command:
                    return msg.result
        return None

    def set_mode(self, name):
        r = self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, self.mm[name])
        print(f"  set_mode {name} -> {r}", flush=True)
        return r

    def set_param(self, name, val):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), float(val),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < 5:
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg:
                self.handle(msg)
                if msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                    print(f"  param {name}={msg.param_value}", flush=True)
                    return msg.param_value
        print(f"  param {name} NO ACK", flush=True)

    def get_param(self, name):
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 5:
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg:
                self.handle(msg)
                if msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                    return msg.param_value

    def reposition(self, lat, lon, alt, radius=0):
        self.tgt = (lat, lon, alt)
        r = self.cmd_int(mavutil.mavlink.MAV_CMD_DO_REPOSITION, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                         -1, 0, radius, 0, int(lat * 1e7), int(lon * 1e7), alt)
        print(f"  reposition -> {r}", flush=True)
        return r

    def wait_ready(self, timeout=120):
        # wait for EKF/GPS: GPS_RAW_INT fix>=3 and prearm ok via SYS_STATUS
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.send_rc()
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            self.handle(msg)
            if msg.get_type() == "SYS_STATUS":
                if msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK:
                    return True
        return False

    def arm(self, tries=20):
        for i in range(tries):
            r = self.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if r == 0:
                print("  armed", flush=True)
                return True
            self.pump(2, log=False)
        return False

    def takeoff_hover(self, alt=40, tmax=90):
        self.set_mode("GUIDED")
        ok = self.arm()
        r = self.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        print(f"  takeoff -> {r}", flush=True)
        t0 = self.st["t"]
        self.pump(tmax, until=lambda s: s.get("alt", 0) > alt - 1.0, every=5)
        print(f"  reached {self.st.get('alt')} m in {self.st['t'] - t0:.1f} s", flush=True)
        self.pump(10, every=5)
        return ok
