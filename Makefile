# Offline checks only. Sim/Isaac/ManiSkill are deliberately absent — they need a
# GPU, a display, and separate venvs. Do NOT `source /opt/ros/...` before these:
# webui/session.py exits 1 if AMENT_PREFIX_PATH is set.
#
# CI runs the exact same targets (see .github/workflows/ci.yml).
PYTHON ?= /usr/bin/python3   # never conda's python3 — it has no pytest and leads PATH

.PHONY: ci test smoke deps cad-setup cad cad-arm cad-test cad-open cad-open-arm

deps:
	$(PYTHON) -m pip install -r requirements-dev.txt

test:
	$(PYTHON) -m pytest

# --print-argv builds the run_demo.sh argv without launching anything; it
# exercises the scenario override / --set path that --list alone doesn't.
smoke:
	$(PYTHON) webui/session.py --list
	$(PYTHON) webui/session.py --print-argv explore_fresh
	$(PYTHON) webui/session.py --print-argv explore_fresh --set headless=true
	$(PYTHON) webui/session.py --print-argv real_map_fresh --set resume=true

ci: test smoke

# ---- CAD (hardware/) — its own venv; not part of `ci`, see hardware/README.md
CAD_PY := hardware/.venv/bin/python
CAD_DIR := hardware/magnet_effector

cad-setup:
	hardware/setup_cad.sh

cad:
	$(CAD_PY) $(CAD_DIR)/build.py

cad-arm:
	$(CAD_PY) $(CAD_DIR)/build.py --arm

cad-test:
	$(CAD_PY) -m pytest tests/cad_test.py

cad-open: cad
	freecad $(CAD_DIR)/build/fitcheck.step >/dev/null 2>&1 &

cad-open-arm: cad-arm
	freecad $(CAD_DIR)/build/arm_context.step >/dev/null 2>&1 &
