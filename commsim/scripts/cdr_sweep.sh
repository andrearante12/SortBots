#!/usr/bin/env bash
# The CDR numbers (2026-10-20): the design as it is now — Status over UDP at
# 3 Hz, batman OGM 250 ms, up to 3 tasks per robot, majority quorum. The
# demo (2 robots + gateway, open floor) is the requirement; the warehouse
# sweeps show how far it scales. V2d = Status to every robot on both bands,
# which is what the robots run; V4 = neighbor-only Status, which needs the
# "which areas I listen to" announcement that is not built yet.
#   commsim/scripts/cdr_sweep.sh [OUT_DIR] [JOBS]
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=${1:-commsim/results/CDR}
J=${2:-12}
PIN="--status-transport udp --configs commsim/configs/ogm250.yaml"
DEMO="commsim/configs/demo.yaml"
SC="/usr/bin/python3 -m commsim.des.scenarios --jobs $J --out $OUT/scen"
RUN="/usr/bin/python3 -m commsim.des.run --seeds 1 2 3 --jobs $J"
DUAL='bands=["g24","g5"]'
NS="10 12 14 16 18 20 22 24 26 28 30"
# demo: every failure scenario, 5 seeds; and its steady-state numbers
$SC --tag demo --variants V2 V4 --n 2 3 --seeds 1 2 3 4 5 --layout open_floor \
    --status-transport udp --configs commsim/configs/ogm250.yaml $DEMO
$RUN --variants V2 --label V2d --set "$DUAL" --n 2 3 --layout open_floor $PIN $DEMO --out $OUT/demo
$RUN --variants V4 --n 2 3 --layout open_floor $PIN $DEMO --out $OUT/demo
# scale: failure scenarios at the sizes the earlier 110/120 used
$SC --tag scale --variants V2 V4 --n 10 15 --seeds 1 2 3 $PIN
# scale: capacity until failure, clean channel and campus background
$RUN --variants V2 --label V2d --set "$DUAL" --n $NS --until-fail $PIN --out $OUT/sweep
$RUN --variants V4 --n $NS --until-fail $PIN --out $OUT/sweep
$RUN --variants V2 --label V2d --set "$DUAL" --bg 0.15 0.30 --n $NS --until-fail $PIN --out $OUT/sweep_bg
$RUN --variants V4 --bg 0.15 0.30 --n $NS --until-fail $PIN --out $OUT/sweep_bg
