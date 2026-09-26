#!/bin/bash
# launch_hw2.sh: hover-in-wind with Q_M_SPIN_MAX 0.95 (param + model), instances 16-21; pids in out/hw2.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/hw2.pids
ulimit -v 8000000
for p in "16 hs0 0 2000" "17 hs5 5 2000" "18 hs8 8 2000" "19 hs10 10 2000" "20 hs10a30 10 3000" "21 hs12a30 12 3000"; do
  set -- $p
  SPEEDUP=8 timeout 1200 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python hover_wind.py $1 $2 $3 $4 0.95 > out/$2.out 2>&1 &
  echo $! >> out/hw2.pids
done
