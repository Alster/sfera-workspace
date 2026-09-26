import sys, glob, os
from pymavlink import mavutil
def latest_bin(d):
    fs = sorted(glob.glob(os.path.join(d, "logs", "*.BIN")), key=os.path.getmtime)
    return fs[-1] if fs else None
def read(d, types):
    f = latest_bin(d)
    m = mavutil.mavlink_connection(f)
    out = {t: [] for t in types}
    while True:
        x = m.recv_match(type=types)
        if x is None: break
        out[x.get_type()].append(x.to_dict())
    return out
