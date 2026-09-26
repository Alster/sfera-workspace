from pymavlink import mavutil
import collections, pickle
T0=526.365
want=set('ATT RATE CTUN QTUN RCOU RCIN MOTB BAT VIBE PIDP PIDR PIDY PIQP PIQR PIQY PIQA XKF1 XKF2 TSIT AETR ANG CTRL ARSP GPS STAT NTUN DCM AOA MODE IMU POS'.split())
m=mavutil.mavlink_connection('/home/anikkhoma/repos/sfera/sfera-workspace/sims/SITL_00000086/00000086.BIN')
d=collections.defaultdict(list)
while True:
    x=m.recv_match(type=list(want))
    if x is None: break
    r=x.to_dict(); r['t']=r['TimeUS']/1e6-T0
    for k in ('I','C','IMU'):
        pass
    d[x.get_type()].append(r)
pickle.dump(dict(d),open('d.pkl','wb'))
