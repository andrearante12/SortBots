# Offline checks only. Sim/Isaac/ManiSkill are deliberately absent — they need a
# GPU, a display, and separate venvs. Do NOT `source /opt/ros/...` before these:
# webui/session.py exits 1 if AMENT_PREFIX_PATH is set.
#
# CI runs the exact same targets (see .github/workflows/ci.yml).
PYTHON ?= /usr/bin/python3   # never conda's python3 — it has no pytest and leads PATH

.PHONY: ci test smoke deps

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

ci: test smoke
