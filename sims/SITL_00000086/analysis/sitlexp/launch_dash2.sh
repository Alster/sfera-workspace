#!/bin/bash
# launch_dash2.sh: 4 dash track-loop probes on SITL instances 16-19 (ports 5920+), pids in out/dash2.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/dash2.pids
ulimit -v 8000000
for p in "16 dk03 70 0.3 LOITER" "17 dk06 70 0.6 LOITER" "18 dk03q 70 0.3 QLOITER" "19 dk03t100 100 0.3 LOITER"; do
  set -- $p
  SPEEDUP=8 timeout 1800 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python dash2.py $1 $2 $3 $4 $5 > out/$2.out 2>&1 &
  echo $! >> out/dash2.pids
done
