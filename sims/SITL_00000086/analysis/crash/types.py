from pymavlink import mavutil
import collections
m=mavutil.mavlink_connection('/home/anikkhoma/repos/sfera/sfera-workspace/sims/SITL_00000086/00000086.BIN')
c=collections.Counter(); fmt={}
while True:
    x=m.recv_match()
    if x is None: break
    t=x.get_type(); c[t]+=1
    if t not in fmt: fmt[t]=x.get_fieldnames() if hasattr(x,'get_fieldnames') else x._fieldnames
    if t=='MSG': print(x.TimeUS/1e6, x.Message)
    if t=='MODE': print('MODE',x.TimeUS/1e6, x.Mode, x.ModeNum)
    if t=='EV': print('EV',x.TimeUS/1e6,x.Id)
    if t=='ERR': print('ERR',x.TimeUS/1e6,x.Subsys,x.ECode)
for k,v in sorted(c.items()): print(k,v,fmt[k])
