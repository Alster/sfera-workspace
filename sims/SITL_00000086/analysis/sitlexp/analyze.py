import sys, json, math, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitllib import dist_bearing, EXP

BRG = 118.0


def xtrack(h, lat, lon):
    d, b = dist_bearing(h[0], h[1], lat, lon)
    return d * math.sin(math.radians(b - BRG)), d * math.cos(math.radians(b - BRG))


def load(name):
    rows, ev = [], []
    for l in open(os.path.join(EXP, "runs", name, "telem.jsonl")):
        d = json.loads(l)
        if "text" in d or "mark" in d:
            ev.append(d)
        else:
            rows.append(d)
    return rows, ev


def analyze(name, verbose=True):
    rows, ev = load(name)
    marks = [e for e in ev if "mark" in e]
    if not marks:
        print(name, "NO MARKS"); return
    t_start = marks[0]["t"]
    # home0 = position at first mark
    r0 = min(rows, key=lambda r: abs(r["t"] - t_start))
    home = (r0["lat"], r0["lon"])
    out = [f"### {name}"]
    prev = {}
    tl = []
    for r in rows:
        if r["t"] < t_start - 1:
            prev = {"mode": r.get("mode"), "vtol": r.get("vtol")}
            continue
        for k in ("mode", "vtol"):
            if r.get(k) != prev.get(k):
                tl.append((r["t"], f"{k}:{prev.get(k)}->{r.get(k)}", r))
                prev[k] = r.get(k)
    for e in ev:
        if e["t"] >= t_start - 1:
            r = min(rows, key=lambda x: abs(x["t"] - e["t"]))
            tl.append((e["t"], ("MARK " + e["mark"]) if "mark" in e else ("ST " + e["text"]), r))
    tl.sort(key=lambda x: x[0])
    for t, s, r in tl:
        dh = dist_bearing(home[0], home[1], r["lat"], r["lon"])[0]
        out.append(f"  +{t - t_start:6.1f}s {s:55s} dT={r.get('dtgt', float('nan')):6.0f} dH={dh:6.0f} "
                   f"alt={r['alt']:6.1f} gs={r.get('gs', 0):5.1f} as={r.get('as', 0):5.1f} p={r.get('pitch', 0):4.0f}")
    # segment stats
    for i, m in enumerate(marks):
        t1 = marks[i + 1]["t"] if i + 1 < len(marks) else 1e9
        seg = [r for r in rows if m["t"] <= r["t"] < t1]
        if not seg:
            continue
        alts = [r["alt"] for r in seg]
        gss = [r.get("gs", 0) for r in seg]
        xt = [abs(xtrack(home, r["lat"], r["lon"])[0]) for r in seg]
        last = seg[-1]
        out.append(f"  seg[{m['mark']}] dur={seg[-1]['t'] - m['t']:.1f}s alt min/max={min(alts):.1f}/{max(alts):.1f} "
                   f"gs max={max(gss):.1f} xtrack max={max(xt):.0f} m | end: mode={last.get('mode')} vt={last.get('vtol')} "
                   f"arm={last.get('armed')} alt={last['alt']:.1f} dT={last.get('dtgt', float('nan')):.1f} "
                   f"dH={dist_bearing(home[0], home[1], last['lat'], last['lon'])[0]:.1f}")
    txt = "\n".join(out)
    if verbose:
        print(txt)
    return rows, ev, home


if __name__ == "__main__":
    for n in sys.argv[1:]:
        analyze(n)
