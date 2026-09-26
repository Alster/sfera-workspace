#!/bin/bash
# launch_hw4.sh: diag 0.5 model, Q_ANGLE_MAX 30/45 at 5 and 10 m/s + 10 m/s at 20; instances 16-20
cd "$(dirname "$0")"; mkdir -p out; : > out/hw4.pids
ulimit -v 8000000
i=16
for p in "5 3000" "5 4500" "10 2000" "10 3000" "10 4500"; do set -- $p
  MODEL=tailsitter_cal_d05.json SPEEDUP=8 timeout 1200 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python hover_wind.py $i ha$1_$2 $1 $2 > out/ha$1_$2.out 2>&1 &
  echo $! >> out/hw4.pids; i=$((i+1))
done
