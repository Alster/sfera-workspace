from u import *
show('AOA',['AOA','SSA'],69,80,0.4)
show('ARSP',['Airspeed','DiffPress'],69,80,0.4)
show('GPS',['Spd','VZ','GCrs'],69,80,0.4)
show('XKF2',['VWN','VWE'],60,80,1.0,filt=lambda r:r['C']==0)
show('XKF1',['VN','VE','VD','GX','GY','GZ'],60,80,2,filt=lambda r:r['C']==0)
for w in [(25,65),(69.4,73.2),(73.2,77.2)]:
    stats('VIBE',['VibeX','VibeY','VibeZ','Clip'],*w,filt=lambda r:r['IMU']==0)
    stats('PIQY',['I','P'],*w); stats('PIQP',['I'],*w); stats('PIQR',['I'],*w)
    stats('RATE',['ROut','POut','YOut'],*w)
    stats('AETR',['Ail','Elev','Rudd'],*w)
show('STAT',['isFlying','Crash','Hit'],76,80,0.4)
