#!/bin/bash
# launch_hw3.sh: diagonal_size (inertia) probes, instances 16-21; pids in out/hw3.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/hw3.pids
ulimit -v 8000000
i=16
for d in 05 07 09; do for w in 0 5; do
  MODEL=tailsitter_cal_d$d.json SPEEDUP=8 timeout 1200 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python hover_wind.py $i hd${d}w$w $w 2000 > out/hd${d}w$w.out 2>&1 &
  echo $! >> out/hw3.pids; i=$((i+1))
done; done
