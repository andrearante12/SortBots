#!/usr/bin/env bash
# One command to drive the physical SO-101 with the Pro Controller.
#
#   scripts/arm.sh            cartesian teleop (sticks move the hand)  <- normal use
#   scripts/arm.sh fk         torque OFF, move the arm by hand, prints hand xyz (check the model)
#   scripts/arm.sh dry        prints the IK targets, sends nothing
#   scripts/arm.sh joint      joint-space teleop (scripts/arm_gamepad.py)
#   scripts/arm.sh -- ARGS    any extra args go straight to the python script (e.g. --flip wrist_flex)
#
# Runs in the `lerobot` conda env's python directly (no activate), and refuses to run in a
# shell with ROS sourced: same separation rule as run_demo.sh. Override the interpreter
# with LEROBOT_PYTHON.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${LEROBOT_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python}"
PORT="${ARM_PORT:-/dev/ttyACM0}"

if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "arm.sh: ROS is sourced in this shell; open a clean one (lerobot env and ROS must stay apart)" >&2
  exit 1
fi
[[ -x "$PY" ]] || { echo "arm.sh: no python at $PY (is the 'lerobot' conda env installed?)" >&2; exit 1; }

script=arm_cartesian.py
extra=()
case "${1:-}" in
  fk)    shift; extra=(--fk) ;;
  dry)   shift; extra=(--dry-run) ;;
  joint) shift; script=arm_gamepad.py ;;
  --)    shift ;;
esac

# The board drops off USB and comes back with root:dialout 660; until the login session has the
# dialout group (log out/in once) every re-plug needs this chmod. Say so instead of a traceback.
if [[ ! -e "$PORT" ]]; then
  echo "arm.sh: $PORT not found: is the board plugged in and the 12 V supply on?" >&2
  exit 2
fi
if [[ ! -r "$PORT" || ! -w "$PORT" ]]; then
  echo "arm.sh: no permission on $PORT. Run:  sudo chmod 666 $PORT   (or log out/in once for dialout)" >&2
  exit 3
fi

cd "$HERE"
exec "$PY" "$script" "${extra[@]}" "$@"
