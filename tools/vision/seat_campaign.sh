#!/usr/bin/env bash
# Repeats of the rack stops, for the per-holder seat table.
#
# One cycle per repeat because that is what an arrival at the observation pose
# costs; `--dump` is what makes the cycles reusable afterwards. Only holders
# that already have references are driven — `check-run` answers at each stop,
# so a cycle that goes wrong says so instead of quietly dumping a bad frame.
set -u
cd "$(dirname "$0")"
export EPICS_CA_NAME_SERVERS=127.0.0.1:5064
PY=/home/bl9b/micromamba/envs/pydm/bin/python
N=${N:-5}
HOLDERS=${HOLDERS:-"2 3 4"}
OUT=${OUT:-../../vision_survey}

for h in $HOLDERS; do
  $PY -c "
from epics import PV
p = PV('Robot:Holder'); p.wait_for_connection(5); p.put($h, wait=True)
assert p.get(use_monitor=False) == $h"
  for i in $(seq 1 "$N"); do
    echo "=== holder $h, repeat $i/$N ==="
    $PY vision_node.py --dump "$OUT" check-run 2>&1 | rg "ECC=|FAILED|refus|seat is" || true
  done
done
