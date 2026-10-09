# Sourced (not executed) by scripts/run_demo.sh and scripts/run_robot.sh:
# resolves which RTAB-Map database a run opens, and enforces COPY-ON-USE for
# the saved-map library. Both launchers need exactly the same rules, and the
# day they drift apart is the day a demo dirties a tracked git-lfs map.
#
#   prepare_map_db TAG
#     in : MAP_DB (may be empty), ROBOT_ID, LOCALIZE, RESUME, REPO_ROOT
#     out: MAP_DB set to the working database path
#     ret: 0 ok, 1 missing/unfetched map, 2 bad flag combination
#   TAG prefixes progress lines, e.g. "[run_demo]".

prepare_map_db() {
  local tag="$1"

  # Default matches launch/sortbots_rtabmap_robot.launch.py's database_path.
  [[ -z "$MAP_DB" ]] && MAP_DB="$HOME/.ros/sortbots_${ROBOT_ID}.db"

  # COPY-ON-USE for the saved-map library (maps/, see maps/README.md).
  #
  # A library entry is a COMMITTED artifact. RTAB-Map opens its sqlite file
  # read-write even under --localize (Mem/IncrementalMemory=false stops it
  # LEARNING, not WRITING), so pointing the stack straight at maps/ would dirty a
  # tracked ~150-500 MB git-lfs object on every demo. Copy to the working DB and
  # run on that instead; --resume then extends the COPY, and keeping that result
  # is always an explicit `scripts/maps.sh save`.
  MAP_DB="$(readlink -f -- "$MAP_DB" 2>/dev/null || echo "$MAP_DB")"
  if [[ "$MAP_DB" == "$REPO_ROOT/maps/"* ]]; then
    if [[ ! -f "$MAP_DB" ]]; then
      echo "ERROR: no library map at $MAP_DB  (scripts/maps.sh list)"
      return 1
    fi
    # A clone without git-lfs leaves a ~130-byte pointer file here. It LOOKS
    # present, so RTAB-Map opens it and fails deep inside sqlite; say the real
    # thing instead.
    if head -c 23 -- "$MAP_DB" 2>/dev/null | grep -q "^version https://git-lfs"; then
      echo "ERROR: $MAP_DB is an unfetched git-lfs pointer, not a database."
      echo "       Fetch it first:  git lfs pull"
      return 1
    fi
    if [[ "$LOCALIZE" != "true" && "$RESUME" != "true" ]]; then
      echo "ERROR: --map into maps/ needs --localize or --resume; plain mapping"
      echo "       mode would wipe the copy on start (delete_db_on_start:=true)."
      return 2
    fi
    local work_db="$HOME/.ros/sortbots_${ROBOT_ID}.db"
    echo "$tag library map $(basename "$(dirname "$MAP_DB")") -> $work_db (library entry untouched)"
    mkdir -p "$HOME/.ros"
    cp -f -- "$MAP_DB" "$work_db" || return 1
    MAP_DB="$work_db"
  fi

  # Localizing/resuming against a map that was never built is a confusing
  # failure mode — RTAB-Map comes up, publishes nothing, and every nav goal is
  # rejected for want of a map. Catch it here instead.
  if [[ ( "$LOCALIZE" == "true" || "$RESUME" == "true" ) && ! -f "$MAP_DB" ]]; then
    echo "ERROR: --localize/--resume need an existing map, but $MAP_DB does not exist."
    echo "       Build one first with a plain mapping run."
    return 1
  fi
  return 0
}
