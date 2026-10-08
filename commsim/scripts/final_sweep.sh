#!/usr/bin/env bash
# Session-1 headline sweep (sim_log step 13): the spec as written and its
# first two fixes (+LB lease beacon, +SC single connect), all on the ORIGINAL
# design settings — spec_original.yaml is pinned on every run, because
# base.yaml now carries every later fix (code review 2026-10-04).
# Numbers will not match step 13 exactly: the DES itself was corrected
# afterwards (sim_log step 17); step 20 / fix_sweep.sh supersede it.
#   commsim/scripts/final_sweep.sh [OUT_DIR] [JOBS]
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=${1:-commsim/results/FINAL}
J=${2:-8}
RUN="/usr/bin/python3 -m commsim.des.run --seeds 1 2 3 --jobs $J --out $OUT --configs commsim/configs/spec_original.yaml --status-transport tcp"
FIX="--lease-beacon-hz 1 --single-connect"

# 1. spec as written
$RUN --variants V1 --n 6 7 8 9 10 12
$RUN --variants V2 --n 10 11 12 13 14 15
$RUN --variants V3 V4 V5 V6 phase0 --n 10 15
# 2. fixes: lease beacon (+LB), single session per pair (+SC)
$RUN --variants V2 --suffix +LB+SC $FIX --n 10 11 12 13 14 15 16
$RUN --variants V3 V4 --suffix +LB+SC $FIX --n 14 15 16 17 18 19 20
$RUN --variants V5 V6 --suffix +LB+SC $FIX --n 10 15
$RUN --variants V3 V4 --suffix +LB --lease-beacon-hz 1 --n 10 11 12 13 14 15
# 3. campus background load
$RUN --variants V2 V3 V4 --suffix +LB+SC $FIX --bg 0.15 0.30 --n 8 10 12 14 16 18 20 --until-fail
# 4. sensitivity: emulation-like slow rates; near-free-space path loss
$RUN --variants V2 V3 V4 --suffix +LB+SC/slow $FIX --radio ht=0 max_rate_mbps=12 --n 8 10 12 14 15 16 18 20 --until-fail
$RUN --variants V4 --suffix +LB+SC/ple2 $FIX --radio pl_exponent=2.0 --n 15 18 20 22 24 26 28 30 --until-fail
