#!/usr/bin/env bash
# One-time CAD setup: hardware/.venv with CadQuery, reference STEPs, FreeCAD check.
# Idempotent — safe to re-run. Exit 0 = ready, 2 = venv ok but FreeCAD missing.
#
# The venv is built from the SYSTEM python3 with --without-pip + get-pip.py:
# Ubuntu's python3 ships without ensurepip (no python3-venv package), so a
# plain `python3 -m venv` dies with "ensurepip is not available" (2026-10-05).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
REF="$HERE/magnet_effector/reference"
SO_ARM_SHA=5f6d2b876a53a4872e405b991dd925556c9e38a4   # keep in sync with reference/SOURCE.md

if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ROS is sourced in this shell; open a clean one (CAD and ROS never share a shell)." >&2
  exit 1
fi

if [[ ! -x "$VENV/bin/pip" ]]; then
  echo "== creating $VENV"
  /usr/bin/python3 -m venv --without-pip "$VENV"
  curl -sSfL https://bootstrap.pypa.io/get-pip.py | "$VENV/bin/python" - -q
fi
"$VENV/bin/pip" install -q -r "$HERE/requirements-cad.txt"
"$VENV/bin/python" -c "import cadquery; print('cadquery', cadquery.__version__)"

# Reference STEPs are committed; this only restores any that went missing.
for p in STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step \
         STEP/SO101/Follower_Specific/Moving_Jaw_SO101.step \
         STEP/SO101/Wrist_Roll_Pitch_SO101.step \
         STEP/SO100/STS3215_03a.step; do
  f="$REF/$(basename "$p")"
  [[ -s "$f" ]] || curl -sSfL -o "$f" \
    "https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/$SO_ARM_SHA/$p"
done
echo "reference STEPs: $(ls "$REF"/*.step | wc -l) present"

if command -v freecad >/dev/null; then
  echo "freecad: $(command -v freecad)"
else
  # Ubuntu 26.04 has no apt package for FreeCAD; the snap is the maintained build.
  echo "FreeCAD not installed. Run:  sudo snap install freecad" >&2
  exit 2
fi
