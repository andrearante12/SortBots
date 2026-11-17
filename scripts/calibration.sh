#!/usr/bin/env bash
# The real robot's calibration, kept in the repo so any machine can drive the same arm.
#
#   scripts/calibration.sh status    repo copy vs this machine's: same / differs / missing
#   scripts/calibration.sh install   repo -> this machine (a differing file is backed up first)
#   scripts/calibration.sh save      this machine -> repo, after recalibrating; then commit it
#
# lerobot and the arm tools (head.py, handeye.py, so101_kin.py, arm_limits.py) read and write
# these under $HF_LEROBOT_CALIBRATION/robots (default ~/.cache/huggingface/lerobot/calibration),
# OUTSIDE the repo: until 2026-11-16 they lived only on the laptop that made them, and moving
# the robot to the Jetson would have meant redoing every stage of docs/head_camera.md.
#
# The files describe the HARDWARE (servo zeros and ranges, the head camera's fit to the arm,
# the taught poses), not the computer, so they are valid on any machine driving this robot.
# Recalibrate a servo or move the camera and they all change: `save` and commit.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$HERE/.." && pwd)/configs/calibration/robots"
DEST_DIR="${HF_LEROBOT_CALIBRATION:-$HOME/.cache/huggingface/lerobot/calibration}/robots"

# Only these: the dated backups the tools leave beside them stay on the machine that made them.
FILES=(
  so_follower/sortbots_arm.json              # lerobot-calibrate: arm servo ranges and zeros
  so_follower/sortbots_arm_kin_offsets.json  # handeye solve: arm joint offsets
  so_follower/sortbots_arm_stops.json        # arm.sh limits: measured end stops
  sortbots_head/sortbots_head.json           # head.py calibrate: pan/tilt servos
  sortbots_head/sortbots_head_camera.json    # handeye solve: the D435 on the head, to the arm
  sortbots_head/handeye_samples.json         # the touch samples that fit came from
  sortbots_head/pick_place_pose.json         # handeye pick: taught place pose
  sortbots_head/pick_rest_pose.json          # handeye pick: rest pose
  sortbots_head/pick_flat_poses.json         # handeye pick: taught magnet-flat poses
)

cmd="${1:-status}"
stamp="$(date +%Y%m%d-%H%M%S)"
changed=0

for f in "${FILES[@]}"; do
  repo="$REPO_DIR/$f" here="$DEST_DIR/$f"
  case "$cmd" in
    status)
      if [[ ! -e "$repo" ]]; then echo "not in repo   $f"
      elif [[ ! -e "$here" ]]; then echo "missing here  $f"
      elif cmp -s "$repo" "$here"; then echo "same          $f"
      else echo "differs       $f"; fi
      ;;
    install)
      [[ -e "$repo" ]] || { echo "skip (not in repo)  $f"; continue; }
      if [[ -e "$here" ]] && cmp -s "$repo" "$here"; then continue; fi
      mkdir -p "$(dirname "$here")"
      if [[ -e "$here" ]]; then
        cp -p "$here" "${here%.json}.before-install-$stamp.json"
        echo "replaced      $f  (old one kept as $(basename "${here%.json}").before-install-$stamp.json)"
      else
        echo "installed     $f"
      fi
      cp "$repo" "$here"
      changed=1
      ;;
    save)
      [[ -e "$here" ]] || { echo "skip (not on this machine)  $f"; continue; }
      if [[ -e "$repo" ]] && cmp -s "$repo" "$here"; then continue; fi
      mkdir -p "$(dirname "$repo")"
      cp "$here" "$repo"
      echo "saved         $f"
      changed=1
      ;;
    *)
      echo "usage: $0 {status|install|save}" >&2
      exit 2
      ;;
  esac
done

case "$cmd" in
  install) [[ $changed == 1 ]] || echo "already up to date: $DEST_DIR" ;;
  save)    [[ $changed == 1 ]] && echo "now commit configs/calibration/" || echo "repo already matches $DEST_DIR" ;;
esac
