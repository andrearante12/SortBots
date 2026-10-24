#!/usr/bin/env bash
# Everything behind "after the fixes" in commsim/docs/sim_log.md (step 20):
# spec-as-written vs fixed, TCP vs UDP Status, batman OGM 500 vs 250 ms.
# Every run PINS its OGM interval and Status transport explicitly — the first
# version inherited them from base.yaml, whose defaults later changed (code
# review 2026-10-04). Reroute delay = measured worst case for the OGM interval.
#   commsim/scripts/fix_sweep.sh [OUT_DIR] [JOBS]
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=${1:-commsim/results/FINAL_FIX}
J=${2:-15}
SPEC="--configs commsim/configs/spec_original.yaml"
O500="commsim/configs/ogm500.yaml"
O250="commsim/configs/ogm250.yaml"
RUN="/usr/bin/python3 -m commsim.des.run --seeds 1 2 3 --jobs $J"
SC="/usr/bin/python3 -m commsim.des.scenarios --variants V4 V3 --n 10 15 --seeds 1 2 3 --jobs $J --out $OUT/scen"
NS="10 12 14 16 18 20 22 24 26 28 30"
# failure scenarios
$SC --tag spec $SPEC
for T in tcp udp; do
  $SC --tag $T-500 --status-transport $T --configs $O500
  $SC --tag $T-250 --status-transport $T --configs $O250
done
# capacity
$RUN --variants V1 V2 V3 V4 --suffix /spec $SPEC --n 6 8 10 12 14 16 --until-fail --out $OUT/sweep
$RUN --variants V1 V2 --suffix /spec $SPEC --n 8 10 12 14 16 --out $OUT/sweep_spec
for T in tcp udp; do
  $RUN --variants V2 V3 V4 --suffix /$T-500 --status-transport $T --configs $O500 --n $NS --until-fail --out $OUT/sweep
  $RUN --variants V2 V3 V4 --suffix /$T-250 --status-transport $T --configs $O250 --n $NS --until-fail --out $OUT/sweep
  $RUN --variants V3 V4 --suffix /$T-250 --status-transport $T --configs $O250 --bg 0.15 0.30 --n $NS --until-fail --out $OUT/sweep_bg
done
