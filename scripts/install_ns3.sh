#!/usr/bin/env bash
# Idempotent install of ns-3-dev into ${NS3_HOME:-~/ns-3-dev} for the
# SortBots 802.11s mesh bridge. Mirrors the style of install_isaac_sim.sh.
#
# Without --accept-download: prints the install plan and exits 0.
# With    --accept-download: clones (or updates) ns-3-dev and builds it.
# With    --setup-privileged: (sudo required) sets the tap-creator binary
#                             setuid root so TapBridge can create tap devices
#                             without running the whole ns-3 process as root.
#
# The ns-3 source tree is NOT committed to SortBots — it lives at $NS3_HOME,
# separate from this repo (same pattern as Isaac Sim / ~/isaacsim/).
# What IS committed: network/ns3_mesh_bridge.cc (the SortBots simulation
# program), which this script symlinks into $NS3_HOME/scratch/ after build.
#
# After a successful install the stamp file $NS3_HOME/.stamps/built is
# written so repeated calls skip the expensive configure+build.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_HOME="${NS3_HOME:-$HOME/ns-3-dev}"
STAMP_DIR="$NS3_HOME/.stamps"
NS3_REPO="https://gitlab.com/nsnam/ns-3-dev.git"
# Pin to a known good commit on the ns-3-dev branch.
# Update this hash when you need a newer ns-3.
NS3_COMMIT="a59a458" # ns-3-dev as of 2025-06-01 (has mesh+tap fixes)
MIN_FREE_GB=6

ACCEPT_DOWNLOAD=0
SETUP_PRIVILEGED=0
for arg in "$@"; do
  case "$arg" in
    --accept-download) ACCEPT_DOWNLOAD=1;;
    --setup-privileged) SETUP_PRIVILEGED=1;;
    -h|--help)
      echo "Usage: $0 [--accept-download] [--setup-privileged]"
      echo "  --accept-download   clone/build ns-3 (skipped if stamp exists)"
      echo "  --setup-privileged  sudo: set tap-creator setuid root (one-time)"
      exit 0;;
    *) echo "ERROR: unknown arg '$arg'" >&2; exit 1;;
  esac
done

# ── Preflight ────────────────────────────────────────────────────────────────
echo "==> 1/5 System prerequisites"

if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git not found. Install: sudo apt install git" >&2; exit 1
fi
if ! command -v cmake >/dev/null 2>&1; then
  echo "ERROR: cmake not found. Install: sudo apt install cmake" >&2; exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 not found." >&2; exit 1
fi

# wireshark is not needed to build or run the mesh, only to look at it —
# scripts/capture_mesh_traffic.sh and the Wireshark section of docs/mesh.md both
# need the dumpcap that ships inside it. Kept in the same soft-warn list rather
# than made a hard requirement, so a headless host can still install ns-3.
MISSING_PKGS=()
for pkg in build-essential g++ libxml2-dev libyaml-cpp-dev ninja-build wireshark; do
  dpkg -s "$pkg" >/dev/null 2>&1 || MISSING_PKGS+=("$pkg")
done
if [[ ${#MISSING_PKGS[@]} -gt 0 ]]; then
  echo "WARN: some build deps may be missing: ${MISSING_PKGS[*]}"
  echo "      Install with: sudo apt install ${MISSING_PKGS[*]}"
fi

echo "==> 2/5 Disk space"
FREE_GB="$(df --output=avail -B1G "$HOME" | tail -n1 | tr -d ' ')"
echo "    $FREE_GB GB free on $HOME"
if [[ "$FREE_GB" -lt "$MIN_FREE_GB" ]]; then
  echo "ERROR: need at least ${MIN_FREE_GB} GB free." >&2; exit 1
fi

echo "==> 3/5 FastDDS Discovery Server"
if command -v fastdds >/dev/null 2>&1; then
  echo "    fastdds found: $(command -v fastdds)"
else
  echo "WARN: 'fastdds' not found on PATH."
  echo "      Install with:  sudo apt install ros-jazzy-fastrtps"
  echo "      Then source:   /opt/ros/jazzy/setup.bash"
fi

# ── Print plan ────────────────────────────────────────────────────────────────
cat <<EOF

Install plan
------------
ns-3 source     : $NS3_REPO
Pinned commit   : $NS3_COMMIT
Target dir      : $NS3_HOME
Approx download : ~200 MB (git clone)
Build time      : ~5-15 min (parallel make)
Modules enabled : mesh, tap-bridge, fd-net-device, wifi (default)

EOF

if [[ "$ACCEPT_DOWNLOAD" -ne 1 && "$SETUP_PRIVILEGED" -ne 1 ]]; then
  echo "Re-run with --accept-download to clone and build."
  echo "Re-run with --setup-privileged (sudo required) to set tap-creator setuid."
  exit 0
fi

# ── Clone / update ────────────────────────────────────────────────────────────
if [[ "$ACCEPT_DOWNLOAD" -eq 1 ]]; then
  echo "==> 4/5 Clone / build ns-3"

  if [[ -d "$NS3_HOME/.git" ]]; then
    echo "    ns-3 repo already exists at $NS3_HOME"
    CURRENT_COMMIT="$(git -C "$NS3_HOME" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "    current commit: $CURRENT_COMMIT"
    if [[ "$CURRENT_COMMIT" == "${NS3_COMMIT:0:7}"* ]]; then
      echo "    already at pinned commit — skipping clone."
    else
      echo "    fetching..."
      git -C "$NS3_HOME" fetch origin
      git -C "$NS3_HOME" checkout "$NS3_COMMIT"
    fi
  else
    echo "    cloning $NS3_REPO ..."
    git clone "$NS3_REPO" "$NS3_HOME"
    git -C "$NS3_HOME" checkout "$NS3_COMMIT"
  fi

  # ── Build ────────────────────────────────────────────────────────────────
  if [[ -f "$STAMP_DIR/built" ]]; then
    echo "    ns-3 already built (stamp exists). Skipping build."
    echo "    Delete $STAMP_DIR/built to force a rebuild."
  else
    echo "    configuring ns-3 (mesh + tap-bridge + fd-net-device + examples)..."
    cd "$NS3_HOME"
    ./ns3 configure --enable-examples --enable-tests 2>&1 | tail -5

    echo "    building (this may take several minutes)..."
    ./ns3 build 2>&1 | tail -10

    mkdir -p "$STAMP_DIR"
    touch "$STAMP_DIR/built"
    echo "    build complete."
    cd - >/dev/null
  fi

  # ── Verify key modules ───────────────────────────────────────────────────
  echo "==> 5/5 Verify modules"
  for lib in mesh tap-bridge fd-net-device; do
    SO="$NS3_HOME/build/lib/libns3-dev-${lib}-default.so"
    if [[ -f "$SO" ]]; then
      echo "    OK  $lib"
    else
      echo "    WARN: $SO not found — build may be incomplete." >&2
    fi
  done

  # ── Symlink SortBots scratch program ─────────────────────────────────────
  SRC="$REPO_ROOT/network/ns3_mesh_bridge.cc"
  DST="$NS3_HOME/scratch/ns3_mesh_bridge.cc"
  if [[ -L "$DST" && "$(readlink "$DST")" == "$SRC" ]]; then
    echo "    symlink already correct: $DST -> $SRC"
  else
    ln -sf "$SRC" "$DST"
    echo "    symlinked: $DST -> $SRC"
  fi

  # Build the scratch program
  echo "    building ns3_mesh_bridge..."
  cd "$NS3_HOME"
  ./ns3 build ns3_mesh_bridge 2>&1 | tail -5
  cd - >/dev/null
  echo "    ns3_mesh_bridge built OK."
fi

# ── setuid tap-creator ────────────────────────────────────────────────────────
if [[ "$SETUP_PRIVILEGED" -eq 1 ]]; then
  echo "==> Privileged setup: setuid tap-creator"
  if [[ $EUID -ne 0 ]]; then
    echo "ERROR: --setup-privileged requires root. Run with sudo." >&2
    exit 1
  fi
  TAP_CREATOR="$NS3_HOME/build/src/tap-bridge/ns3-dev-tap-creator-default"
  if [[ ! -f "$TAP_CREATOR" ]]; then
    echo "ERROR: tap-creator not found at $TAP_CREATOR — did the build succeed?" >&2
    exit 1
  fi
  chown root:root "$TAP_CREATOR"
  chmod 4755 "$TAP_CREATOR"
  echo "    setuid applied: $TAP_CREATOR"

  RAW_CREATOR="$NS3_HOME/build/src/fd-net-device/ns3-dev-raw-sock-creator-default"
  if [[ -f "$RAW_CREATOR" ]]; then
    chown root:root "$RAW_CREATOR"
    chmod 4755 "$RAW_CREATOR"
    echo "    setuid applied: $RAW_CREATOR"
  fi
fi

cat <<EOF

Done. Quick-start:
  scripts/sim_ctl.sh console start
  scripts/sim_ctl.sh start explore_fleet_mesh

See docs/mesh.md for verification commands and fault-injection recipes.
EOF
