#!/bin/bash
# launch_gust.sh: scripted gusts (8 m/s cross from 208 deg) at dash start / back transition; inst 20-21
cd "$(dirname "$0")"; mkdir -p out; : > out/gust.pids
ulimit -v 8000000
GUST=dash,8,208,4 SPEEDUP=8 timeout 1800 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python dash2.py 20 gdash 70 0.3 LOITER > out/gdash.out 2>&1 &
echo $! >> out/gust.pids
GUST=return,8,208,15 SPEEDUP=8 timeout 1800 /home/anikkhoma/repos/sfera/sfera/.venv/bin/python dash2.py 21 gret 70 0.3 LOITER > out/gret.out 2>&1 &
echo $! >> out/gust.pids
