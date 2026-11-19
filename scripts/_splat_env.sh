# Sourced by scripts/install_splat.sh and scripts/splat.sh: where the splat
# trainer's python lives. Two kinds of env, one package list
# (requirements-splat.txt):
#   conda  <conda base>/envs/splat               (environment-splat.yml)
#   uv     ~/.local/share/sortbots/splat-venv    where there is no conda: the RTX
#          4070 laptop (Ubuntu 26.04, 2026-11-17) has uv and no conda
# $SORTBOTS_SPLAT_PYTHON overrides both.

SPLAT_UV_VENV="${SORTBOTS_SPLAT_VENV:-$HOME/.local/share/sortbots/splat-venv}"

splat_conda_base() {
  command -v conda >/dev/null 2>&1 || return 1
  conda info --base 2>/dev/null
}

# Prints the env's python if it exists; returns 1 if there is none.
splat_python() {
  if [[ -n "${SORTBOTS_SPLAT_PYTHON:-}" ]]; then
    [[ -x "$SORTBOTS_SPLAT_PYTHON" ]] && { echo "$SORTBOTS_SPLAT_PYTHON"; return 0; }
    return 1
  fi
  local base
  if base="$(splat_conda_base)" && [[ -x "$base/envs/splat/bin/python" ]]; then
    echo "$base/envs/splat/bin/python"; return 0
  fi
  [[ -x "$SPLAT_UV_VENV/bin/python" ]] && { echo "$SPLAT_UV_VENV/bin/python"; return 0; }
  return 1
}
