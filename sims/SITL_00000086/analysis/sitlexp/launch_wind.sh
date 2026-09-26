#!/bin/bash
# launch_wind.sh: dash probes with wind on SITL instances 16-19; pids in out/wind.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/wind.pids
ulimit -v 8000000
for p in "16 wh10 10,118,0" "17 wt10 10,298,0" "18 wc10 10,208,0" "19 wc6g 6,208,3"; do
  set -- $p
  WIND=$3 SPEEDUP=8 timeout 1800 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python dash2.py $1 $2 70 0.3 LOITER > out/$2.out 2>&1 &
  echo $! >> out/wind.pids
done
