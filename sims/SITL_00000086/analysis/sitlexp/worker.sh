#!/bin/bash
# worker.sh <inst> <k> <n> : runs every n-th job starting at k
cd "$(dirname "$0")"
P=/home/anikkhoma/repos/sfera/sfera/.venv/bin/python
i=0
while read -r sc args; do
  if [ $((i % $3)) -eq $2 ]; then
    name=$(echo "$args" | sed 's/.*"name":"\([^"]*\)".*/\1/')
    timeout 3000 $P run.py $sc $1 "$args" > out/$name.out 2>&1
    echo "done $name rc=$?" >> out/_progress.txt
  fi
  i=$((i+1))
done < jobs.txt
