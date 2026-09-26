#!/bin/bash
# launch_7kg.sh: wind + evasion probes on the 7 kg model, SITL instances 16-22; pids in out/7kg.pids
cd "$(dirname "$0")"; mkdir -p out; : > out/7kg.pids
ulimit -v 8000000
export MODEL=tailsitter_cal_7kg.json SPEEDUP=8
P=/home/anikkhoma/repos/sfera/sfera/.venv/bin/python
i=16
for w in 5 8 10; do timeout 1200 $P hover_wind.py $i m7w$w $w 3000 > out/m7w$w.out 2>&1 & echo $! >> out/7kg.pids; i=$((i+1)); done
timeout 1200 $P hover_wind.py $i m7w5a20 5 2000 > out/m7w5a20.out 2>&1 & echo $! >> out/7kg.pids; i=$((i+1))
for k in side25 side_chain fw60; do timeout 1200 $P evade.py $i m7ev_$k $k > out/m7ev_$k.out 2>&1 & echo $! >> out/7kg.pids; i=$((i+1)); done
