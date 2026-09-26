#!/bin/bash
# launch_hw.sh: hover-in-wind probes, SITL instances 16-21; pids in out/hw.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/hw.pids
ulimit -v 8000000
for p in "16 hw5 5 2000" "17 hw8 8 2000" "18 hw10 10 2000" "19 hw10a30 10 3000" "20 hw10a45 10 4500" "21 hw0 0 2000"; do
  set -- $p
  SPEEDUP=8 timeout 1200 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python hover_wind.py $1 $2 $3 $4 > out/$2.out 2>&1 &
  echo $! >> out/hw.pids
done
