"""Спільна жива перевірка SFERA <-> sfera-target-sim без GUI.

В одному процесі запускає дрон симулятора (tcpin) і летючу ціль (UDP), будує SFERA
через sfera/tools/sitl/headless.py і проходить місію: підключення -> зліт (Auto-ARM) ->
перехоплення цілі -> LAND -> DISARM. Код виходу 0 = усе гаразд.

Запуск з теки sfera-workspace/ (venv SFERA — у ньому є pymavlink, якого досить модулям sfera_sim без Tk):
    ../sfera/.venv/bin/python tools/joint_check.py
Порти не робочі (TCP 15790, UDP 15591, міст MP вимкнено), тож можна паралельно з GUI.
SFERA пише свої журнали в sfera/LOG/ (у .gitignore).
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]  # тека з обома репозиторіями й sfera-workspace/
SFERA = ROOT / "sfera"
SIM = ROOT / "sfera-target-sim"
sys.path.insert(0, str(SFERA))   # пакети sfera і tools.sitl
sys.path.append(str(SIM))        # пакет sfera_sim (у кінці: його tools/ не має затінювати SFERA)

from sfera_sim.drone import DroneSimWorker  # noqa: E402
from sfera_sim.geo import latlon_add_m  # noqa: E402
from sfera_sim.models import FlyingTargetConfig, GlobalConfig  # noqa: E402
from sfera_sim.target import FlyingTargetWorker  # noqa: E402
from tools.sitl.headless import HeadlessSfera  # noqa: E402

FAILURES: list[str] = []
OUT = sys.stdout  # HeadlessSfera підміняє sys.stdout своїм журналом — друкуємо в справжній


def say(*parts) -> None:
    print(*parts, file=OUT, flush=True)


def check(ok: bool, message: str) -> bool:
    say(("OK   " if ok else "FAIL ") + message)
    if not ok:
        FAILURES.append(message)
    return ok


def wait_for(pred, timeout_s: float, step_s: float = 0.5):
    end = time.time() + timeout_s
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(step_s)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tcp-port", type=int, default=15790)
    ap.add_argument("--udp-port", type=int, default=15591)
    ap.add_argument("--verbose", action="store_true", help="логи симулятора в stdout")
    args = ap.parse_args()

    g = GlobalConfig()  # фізика за замовчуванням симулятора; HOME на 10 м MSL
    sim_log = (lambda m: say("SIM:", m)) if args.verbose else (lambda _m: None)
    drone = DroneSimWorker("Drone", 1, sim_log)
    drone.start(f"tcpin:127.0.0.1:{args.tcp_port}", 115200, g)
    tlat, tlon = latlon_add_m(g.start_lat, g.start_lon, 300.0, 200.0)
    target = FlyingTargetWorker(FlyingTargetConfig(
        uid="t", name="T", start_lat=tlat, start_lon=tlon, start_alt_msl=g.start_alt_msl + 35.0,
        heading_deg=90.0, speed_mps=8.0, mode="straight", udp_host="127.0.0.1", udp_port=args.udp_port), sim_log)
    target.start()

    fd, sfera_log = tempfile.mkstemp(prefix="sfera_joint_", suffix=".log")
    os.close(fd)
    h = HeadlessSfera(str(SFERA), stdout_log=sfera_log)
    try:
        port = f"tcp:127.0.0.1:{args.tcp_port}"
        # Той самий словник, що будує ui/js/controls.js.
        r = h.api.start_link({"port": port, "ports": [port, ""], "baud": 115200,
                              "sim_ports": [args.udp_port], "mp_tcp_port": 0})
        check(bool(r.get("ok")), f"start_link {port}")

        def d1():
            return next((x for x in h.get_state().get("drones", []) if x.get("uid") == "D1"), {})

        def linked():
            x = d1()
            tel = x.get("telemetry") or {}
            ok = x.get("connected") and (x.get("home") or {}).get("ok") and tel.get("mode") and tel.get("gps_fix")
            return x if ok else None

        d = wait_for(linked, 10)
        check(bool(d), "SFERA: heartbeat, режим, GPS і HOME від дрона симулятора")
        if d:
            tel = d.get("telemetry") or {}
            check(tel.get("mode") == "GUIDED" and int(tel.get("gps_fix") or 0) >= 3,
                  f"SFERA: режим {tel.get('mode')}, GPS fix {tel.get('gps_fix')}")
        tg = wait_for(lambda: next((t for t in h.get_state().get("targets", []) if t.get("uid") == "UDP"), None), 5)
        check(bool(tg) and abs(float(tg.get("speed", 0)) - 8.0) < 1.0, "SFERA: ціль UDP, оцінка швидкості ≈ 8 м/с")

        h.set_fast_start("D1", 20.0, True)
        h.set_hold_settings("D1")
        h.select_target("UDP", "D1")
        h.autopilot_auto(True)
        armed = wait_for(lambda: drone.status_snapshot()["armed"], 5)
        check(bool(armed), "симулятор: ARM від SFERA (Auto-ARM)")

        def auto():
            return d1().get("auto") or {}

        climbed = wait_for(lambda: drone.status_snapshot()["alt_rel"] > 15.0, 15)
        check(bool(climbed), "симулятор: зліт від setpoint-ів SFERA (> 15 м)")
        first = wait_for(lambda: auto().get("dist_m"), 15)
        time.sleep(10)
        last = auto().get("dist_m")
        check(bool(first and last and last < first - 50.0),
              f"SFERA: {auto().get('state')}, відстань до цілі {first or 0:.0f} -> {last or 0:.0f} м")
        # Ціль летить на HOME+35 м (MSL), тож дрон має вийти на відносну висоту ~35 м.
        alt = drone.status_snapshot()["alt_rel"]
        check(25.0 < alt < 40.0, f"висота дрона {alt:.1f} м над HOME (ціль — 35 м): узгоджені MSL/REL")

        h.force_land()
        landed = wait_for(lambda: not drone.status_snapshot()["armed"], 40)
        check(bool(landed), "LAND від SFERA -> посадка й DISARM у симуляторі")
    finally:
        h.shutdown()
        drone.stop()
        target.stop()

    say(f"\nЖурнал SFERA: {sfera_log}")
    say("УСПІХ" if not FAILURES else f"ПОМИЛКИ: {len(FAILURES)}")
    return 0 if not FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
