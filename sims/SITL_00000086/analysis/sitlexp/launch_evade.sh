#!/bin/bash
# launch_evade.sh: evasion timing probes, SITL instances 16-19; pids in out/evade.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/evade.pids
ulimit -v 8000000
i=16
for k in side25 sidedown side_chain fw60; do
  SPEEDUP=8 timeout 1200 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python evade.py $i ev_$k $k > out/ev_$k.out 2>&1 &
  echo $! >> out/evade.pids; i=$((i+1))
done
