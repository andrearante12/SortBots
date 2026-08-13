#!/usr/bin/env bash
# Serve the docs site locally.
#
# It has to be served over HTTP, not opened as a file:// path -- the page uses
# ES modules and fetches the GLB and the map manifest, both of which a browser
# blocks from a file:// origin.
#
#   ./site/serve.sh [port]     default 8090
#
# Deliberately does NOT source ROS: nothing here needs it, and this script is
# safe to run in the same shell as the simulator.
set -euo pipefail
PORT="${1:-8090}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "SortBots docs → http://localhost:${PORT}/"
exec python3 -m http.server "$PORT" --directory "$DIR"
