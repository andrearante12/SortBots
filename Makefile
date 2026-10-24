# Offline checks only. Sim/Isaac/ManiSkill are deliberately absent — they need a
# GPU, a display, and separate venvs. Do NOT `source /opt/ros/...` before these:
# webui/session.py exits 1 if AMENT_PREFIX_PATH is set.
#
# CI runs the exact same targets (see .github/workflows/ci.yml).
PYTHON ?= /usr/bin/python3   # never conda's python3 — it has no pytest and leads PATH

.PHONY: ci test smoke deps lint shellcheck compile fixtures simtest-validate web-test cad-ci cad-setup cad cad-arm cad-test cad-open cad-open-arm

# Stand-ins for the git-lfs library maps the committed tests name; see
# scripts/make_fixture_maps.py. Everything map-dependent runs against this dir.
FIXTURE_MAPS ?= $(CURDIR)/.ci-maps
export SORTBOTS_MAPS_DIR ?= $(FIXTURE_MAPS)

deps:
	$(PYTHON) -m pip install -r requirements-dev.txt

fixtures:
	$(PYTHON) scripts/make_fixture_maps.py $(FIXTURE_MAPS)

# --junitxml + check_skips: a skip is only OK if it is on the list. cadquery
# lives in hardware/.venv (the `cad-ci` job runs those); the STEP assembly is
# a network fetch (hardware/setup_cad.sh).
test: fixtures
	$(PYTHON) -m pytest -rs --junitxml=$(CURDIR)/.pytest-junit.xml
	$(PYTHON) scripts/check_skips.py $(CURDIR)/.pytest-junit.xml 'cadquery' 'SO101_Assembly.step not fetched'

# Correctness lint only (ruff.toml); ruff is `pip install ruff` / see requirements-dev.txt
lint:
	ruff check .

shellcheck:
	shellcheck -S warning scripts/*.sh nodes/*.sh

# Nodes, launch files and Isaac entry points are never imported by a test;
# this at least catches a syntax error before a 7-minute sim start does.
compile:
	$(PYTHON) -m compileall -q nodes launch scripts simtest webui tests

simtest-validate: fixtures
	$(PYTHON) scripts/simtest.py validate

# Headless dashboard test. Needs a chrome: set CHROME_BIN if it isn't on PATH.
web-test: fixtures
	node webui/tests/operator_test.mjs

# --print-argv builds the run_demo.sh argv without launching anything; it
# exercises the scenario override / --set path that --list alone doesn't.
smoke:
	$(PYTHON) webui/session.py --list
	$(PYTHON) webui/session.py --print-argv explore_fresh
	$(PYTHON) webui/session.py --print-argv explore_fresh --set headless=true
	$(PYTHON) webui/session.py --print-argv real_map_fresh --set resume=true

ci: lint shellcheck compile test smoke simtest-validate web-test

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

# CI variant: cadquery installed straight into the runner's python (no venv).
cad-ci:
	$(PYTHON) -m pip install -r hardware/requirements-cad.txt
	$(PYTHON) -m pytest tests/cad_test.py -rs

cad-open: cad
	freecad $(CAD_DIR)/build/fitcheck.step >/dev/null 2>&1 &

cad-open-arm: cad-arm
	freecad $(CAD_DIR)/build/arm_context.step >/dev/null 2>&1 &
