#!/usr/bin/env python3
"""Scenario loading and sim-session supervision for the dashboard.

Imported by `webui/serve.py` (which exposes it over HTTP when started with
`--control`); also runnable standalone for offline checks:

    python3 webui/session.py --list
    python3 webui/session.py --print-argv explore_resume
    python3 webui/session.py --print-argv explore_fresh --set headless=true

Pure stdlib + PyYAML, deliberately no rclpy: this module has to run in a
ROS-free environment, because `scripts/run_demo.sh` hard-refuses to start when
AMENT_PREFIX_PATH is set (and Isaac's activate_isaac.sh does the same).

WHAT THIS DOES NOT DO: it does not supervise ROS nodes itself. It shells out to
`scripts/run_demo.sh`, which is already the project's process manager, and adds
exactly three things on top — a named preset (configs/scenarios/*.yaml), a
readable phase for the UI, and a per-run directory to collect the log.

SECURITY. serve.py binds 0.0.0.0 and is reachable from any tailnet device, so
with --control it is a remote process launcher. The containment is here:
scenario names resolve by allowlist against configs/scenarios/, and every
element of the argv handed to Popen comes from the fixed RUN_FLAGS table below,
built from typed and range-checked values. No free-text string from a request
ever reaches a shell, and Popen is always called with a list and shell=False.
Tailscale ACLs remain the actual perimeter; this is not an authenticated API.

WHY run_demo.sh EXITING IS NOT THE SESSION ENDING: run_demo.sh setsid's both
Isaac and the ROS bringup and then returns, so the process we spawn lives for
~30 seconds and exits 0 on success. Session liveness is therefore a property of
the *pipeline* (see pipeline_alive()), not of our child's pid. That is also
what lets a session survive serve.py restarting — see SessionManager._adopt().
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent

# The saved-map library's schema and listing. Deliberately ROS-free too, so
# importing it here keeps this module runnable in the same bare environment.
sys.path.insert(0, str(REPO_ROOT / "scripts"))
import maps_lib  # noqa: E402

SCENARIO_DIR = REPO_ROOT / "configs" / "scenarios"
# Overridable so scenarios_test.mjs can run against an empty dir instead of
# whatever a previous real console session left in the repo's own data/.
SESSIONS_DIR = Path(os.environ["SORTBOTS_SESSIONS_DIR"]) if os.environ.get("SORTBOTS_SESSIONS_DIR") \
    else REPO_ROOT / "data" / "sessions"
CURRENT_POINTER = SESSIONS_DIR / "current.json"
RUN_DEMO = REPO_ROOT / "scripts" / "run_demo.sh"
RECORD_BAG = REPO_ROOT / "scripts" / "record_explore_bag.sh"
MAPS_SH = REPO_ROOT / "scripts" / "maps.sh"
ROS_SETUP = "/opt/ros/jazzy/setup.bash"

# Same prepend run_demo.sh and sortbots_webui.launch.py use: conda's init hook
# puts its bin dirs at the front of PATH even after `conda deactivate`, and
# ros2/rosbridge scripts with `#!/usr/bin/env python3` shebangs then grab the
# wrong interpreter (rclpy's compiled extension is built for the system 3.12).
SYSTEM_PATH = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

MAX_LOG_CHUNK = 64 * 1024


class ScenarioError(ValueError):
    """A scenario file, or a requested override, is not valid."""


class SessionConflict(RuntimeError):
    """A session is already running and force was not requested."""


# ---------------------------------------------------------------------------
# run: key -> argv. The ONLY place a config value becomes a command-line
# argument. Every builder validates; anything not listed here is rejected at
# load time rather than being forwarded to the shell.
# ---------------------------------------------------------------------------

def _choice(flag: str, choices: set[str]):
    def build(value):
        if value not in choices:
            raise ScenarioError(f"{flag}: expected one of {sorted(choices)}, got {value!r}")
        return [flag, str(value)]
    return build


def _int_range(flag: str, lo: int, hi: int):
    def build(value):
        try:
            n = int(value)
        except (TypeError, ValueError):
            raise ScenarioError(f"{flag}: expected an integer, got {value!r}") from None
        if not lo <= n <= hi:
            raise ScenarioError(f"{flag}: expected {lo}..{hi}, got {n}")
        return [flag, str(n)]
    return build


def _pattern(flag: str, rx: re.Pattern):
    def build(value):
        if not isinstance(value, str) or not rx.match(value):
            raise ScenarioError(f"{flag}: {value!r} does not match {rx.pattern}")
        return [flag, value]
    return build


def _switch(flag: str, when: bool):
    """Emit a bare flag when the value equals `when`, nothing otherwise.

    Covers both polarities: headless=true -> --headless, but chase_cam=false ->
    --no-chase-cam.
    """
    def build(value):
        if not isinstance(value, bool):
            raise ScenarioError(f"{flag}: expected a boolean, got {value!r}")
        return [flag] if value is when else []
    return build


def _chase_cam_robots(flag: str):
    """How many robots get a cosmetic chase cam (first N in roster order).

    None/omitted -> run_demo.sh / spawn_warehouse default (every spawned
    robot). 0 is valid and equivalent to chase_cam:false / --no-chase-cam,
    but kept as its own flag so a fleet scenario can say "1" without
    fighting the boolean. Accepts int or digit-string (CLI --set).
    """
    def build(value):
        if value is None:
            return []
        # bool is a subclass of int; reject true/false before int().
        if isinstance(value, bool):
            raise ScenarioError(f"{flag}: expected an int, got {value!r}")
        try:
            n = int(value)
        except (TypeError, ValueError):
            raise ScenarioError(f"{flag}: expected an int, got {value!r}") from None
        if n < 0 or n > _MAX_ROBOTS:
            raise ScenarioError(f"{flag}: {n} out of range 0..{_MAX_ROBOTS}")
        return [flag, str(n)]
    return build


def _map_path(flag: str):
    def build(value):
        if value in (None, ""):
            return []  # run_demo.sh's own default: ~/.ros/sortbots_<robot_id>.db
        if not isinstance(value, str):
            raise ScenarioError(f"{flag}: expected a path string, got {value!r}")
        resolved = Path(os.path.expanduser(value)).resolve()
        # maps_lib.MAPS_DIR by reference, not a literal `REPO_ROOT/"maps"`: it
        # honours SORTBOTS_MAPS_DIR, which is how scenarios_test.mjs points the
        # whole chain at a temp dir outside the repo (and outside $HOME).
        allowed = (Path.home().resolve(), (REPO_ROOT / "data").resolve(),
                   maps_lib.MAPS_DIR.resolve())
        if not any(resolved == root or root in resolved.parents for root in allowed):
            raise ScenarioError(
                f"{flag}: {resolved} is outside $HOME, {REPO_ROOT / 'data'} "
                f"and {maps_lib.MAPS_DIR}"
            )
        return [flag, str(resolved)]
    return build


def _robot_ids(flag: str):
    """Comma-separated robot ids, e.g. `robot_0,robot_1`. Empty/None omits
    the flag entirely — run_demo.sh then derives it from --robots against
    configs/robots.yaml's own order, same as it does when this key is left
    out of a scenario's `run:` block.
    """
    rx = re.compile(r"^[A-Za-z0-9_]+(,[A-Za-z0-9_]+)*$")

    def build(value):
        if value in (None, ""):
            return []
        if not isinstance(value, str) or not rx.match(value):
            raise ScenarioError(f"{flag}: {value!r} does not match {rx.pattern}")
        return [flag, value]
    return build


def _roster_size() -> int:
    with open(REPO_ROOT / "configs" / "robots.yaml") as f:
        return len(yaml.safe_load(f)["robots"])


# Declaration order is argv order — keep it stable so logged commands are
# comparable between runs.
_MAX_ROBOTS = _roster_size()

RUN_FLAGS = {
    "scene": _choice("--scene", {"nvidia", "primitive"}),
    "robots": _int_range("--robots", 1, _MAX_ROBOTS),
    "robot_id": _pattern("--robot-id", re.compile(r"^[A-Za-z0-9_]+$")),
    "robot_ids": _robot_ids("--robot-ids"),
    "headless": _switch("--headless", when=True),
    "chase_cam": _switch("--no-chase-cam", when=False),
    "chase_cam_robots": _chase_cam_robots("--chase-cam-robots"),
    "teleop": _switch("--teleop", when=True),
    "localize": _switch("--localize", when=True),
    "resume": _switch("--resume", when=True),
    "explore": _switch("--explore", when=True),
    "map": _map_path("--map"),
}

RUN_DEFAULTS = {
    "scene": "nvidia",
    "robots": 1,
    "robot_id": "robot_0",
    "robot_ids": None,
    "headless": False,
    "chase_cam": True,
    "chase_cam_robots": None,
    "teleop": False,
    "localize": False,
    "resume": False,
    "explore": False,
    "map": None,
}


def build_argv(run: dict) -> list[str]:
    """Turn a validated `run:` mapping into run_demo.sh arguments."""
    run = dict(run)
    # chase_cam and chase_cam_robots both resolve to run_demo.sh's single
    # CHASE_CAM_ARGS variable, where the LAST flag on the line wins. RUN_FLAGS
    # order emits --no-chase-cam first, so a scenario carrying chase_cam:false
    # AND a chase_cam_robots count silently got its chase cams back — the
    # count overwrote the "off". Let the boolean be authoritative instead:
    # off means off, whatever the count says. This is what lets a scenario
    # keep "how many when enabled" alongside an off-by-default toggle.
    if run.get("chase_cam") is False:
        run.pop("chase_cam_robots", None)
    argv: list[str] = []
    for key, builder in RUN_FLAGS.items():
        if key in run:
            argv.extend(builder(run[key]))
    return argv


# ---------------------------------------------------------------------------
# Scenario files
# ---------------------------------------------------------------------------

VALID_STATUS = {"ready", "planned"}


def _validate(path: Path, raw: dict) -> dict:
    if not isinstance(raw, dict):
        raise ScenarioError("file must contain a YAML mapping")

    name = raw.get("name")
    if name != path.stem:
        raise ScenarioError(f"name: {name!r} must equal the filename stem {path.stem!r}")

    status = raw.get("status", "ready")
    if status not in VALID_STATUS:
        raise ScenarioError(f"status: expected one of {sorted(VALID_STATUS)}, got {status!r}")

    run = raw.get("run") or {}
    if not isinstance(run, dict):
        raise ScenarioError("run: must be a mapping")
    unknown = sorted(set(run) - set(RUN_FLAGS))
    if unknown:
        raise ScenarioError(f"run: unknown key(s) {unknown} — allowed: {sorted(RUN_FLAGS)}")
    merged = {**RUN_DEFAULTS, **run}
    build_argv(merged)  # type/range check now, not at launch time
    if merged["localize"] and merged["resume"]:
        raise ScenarioError("run: localize and resume are mutually exclusive")

    overrides = raw.get("overrides") or []
    if not isinstance(overrides, list):
        raise ScenarioError("overrides: must be a list of run keys")
    bad = sorted(set(overrides) - set(RUN_FLAGS))
    if bad:
        raise ScenarioError(f"overrides: unknown run key(s) {bad}")

    capture = raw.get("capture") or {}
    if not isinstance(capture, dict) or not isinstance(capture.get("bag", False), bool):
        raise ScenarioError("capture: must be a mapping with a boolean `bag`")

    return {
        "name": name,
        "title": raw.get("title") or name,
        "description": (raw.get("description") or "").strip(),
        "status": status,
        "run": merged,
        "overrides": list(overrides),
        "capture": {"bag": bool(capture.get("bag", False))},
    }


def load_scenarios() -> list[dict]:
    """Parse configs/scenarios/*.yaml.

    A file that fails validation is returned with status "invalid" and an
    `error` string rather than being dropped — a silently missing scenario is
    much harder to debug than a card that says why it won't load.
    """
    out = []
    for path in sorted(SCENARIO_DIR.glob("*.yaml")):
        try:
            with open(path) as f:
                out.append(_validate(path, yaml.safe_load(f)))
        except (ScenarioError, yaml.YAMLError, OSError) as exc:
            out.append({
                "name": path.stem,
                "title": path.stem,
                "description": "",
                "status": "invalid",
                "run": {},
                "overrides": [],
                "capture": {"bag": False},
                "error": f"{path.name}: {exc}",
            })
    return out


def get_scenario(name: str) -> dict:
    for scenario in load_scenarios():
        if scenario["name"] == name:
            return scenario
    raise ScenarioError(f"no such scenario: {name!r}")


def apply_overrides(scenario: dict, overrides: dict | None) -> dict:
    """Merge request-supplied overrides over the scenario's `run:` defaults.

    Only keys the scenario itself lists under `overrides:` are accepted, and
    each value is coerced to the type of the scenario's own default — so a
    request can change a documented knob but can never introduce a new one.
    """
    run = dict(scenario["run"])
    if not overrides:
        return run
    allowed = set(scenario["overrides"])
    for key, value in overrides.items():
        if key not in allowed:
            raise ScenarioError(f"override {key!r} is not offered by scenario {scenario['name']!r}")
        # Coerce from the scenario's own typed default when present, else the
        # RUN_DEFAULTS type. chase_cam_robots defaults to None in RUN_DEFAULTS
        # but scenarios that expose it set an int — use that so CLI --set
        # strings and form values become ints before build_argv.
        default = run[key] if key in run else RUN_DEFAULTS[key]
        if isinstance(default, bool):
            if isinstance(value, str):
                value = value.lower() == "true"
            run[key] = bool(value)
        elif isinstance(default, int) and not isinstance(default, bool):
            run[key] = int(value)
        else:
            run[key] = value
    build_argv(run)  # re-validate the merged result
    if run["localize"] and run["resume"]:
        raise ScenarioError("localize and resume are mutually exclusive")
    return run


# ---------------------------------------------------------------------------
# Environment + liveness
# ---------------------------------------------------------------------------

_ROS_ENV_KEYS = (
    "AMENT_PREFIX_PATH", "AMENT_CURRENT_PREFIX", "ROS_DISTRO", "ROS_VERSION",
    "ROS_PYTHON_VERSION", "RMW_IMPLEMENTATION", "LD_LIBRARY_PATH", "PYTHONPATH",
    "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
)


def clean_env() -> dict:
    """A ROS-free environment for spawning run_demo.sh.

    run_demo.sh exits 1 if AMENT_PREFIX_PATH is set, and activate_isaac.sh does
    the same, because Isaac's venv and system ROS 2 must never share a shell.
    scripts/run_console.sh already starts serve.py from a clean shell, so this
    is belt-and-braces — it keeps the control API working even when serve.py
    was started some other way (e.g. from a ros2 launch).
    """
    env = {
        k: v for k, v in os.environ.items()
        if k not in _ROS_ENV_KEYS and not k.startswith(("AMENT_", "COLCON_"))
    }
    env["PATH"] = SYSTEM_PATH + ":" + env.get("PATH", "")
    return env


def save_map_blocking(name: str, *, robot_id: str = "robot_0",
                      title: str | None = None, description: str | None = None,
                      timeout: float = 300.0) -> dict:
    """Run `scripts/maps.sh save NAME` and return its resulting manifest.

    Blocking on purpose: the caller is serve.py's POST handler, and the answer
    the dashboard needs ("did the pose graph make it in, or is it pending?") is
    only knowable once the save has finished.

    maps.sh needs system ROS 2 SOURCED — the exact opposite of run_demo.sh,
    which exits 1 when AMENT_PREFIX_PATH is set. clean_env() strips ROS for
    run_demo.sh's sake, so re-source it inside a dedicated `bash -c`, and
    prepend SYSTEM_PATH so `ros2` and map_saver_cli don't get conda's python3.

    CONTAINMENT: same story as build_argv's. `name` and `robot_id` are matched
    against maps_lib.NAME_RX / a fixed pattern BEFORE they are interpolated,
    and every interpolated value is shlex.quote'd; free text (title,
    description) never reaches the shell unquoted. SORTBOTS_MAPS_DIR passes
    through clean_env(), which is what lets a test point this at a temp dir.
    """
    if not maps_lib.NAME_RX.match(str(name)):
        raise maps_lib.MapError(
            f"map name {name!r} does not match {maps_lib.NAME_RX.pattern}")
    if not re.match(r"^[A-Za-z0-9_]+$", str(robot_id)):
        raise maps_lib.MapError(f"robot_id {robot_id!r} is not a bare identifier")

    extra = ""
    if title:
        extra += f" --title {shlex.quote(str(title))}"
    if description:
        extra += f" --description {shlex.quote(str(description))}"

    cmd = (
        f"source {shlex.quote(ROS_SETUP)}; "
        f"export PATH={shlex.quote(SYSTEM_PATH)}:\"$PATH\"; "
        f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; "
        f"exec {shlex.quote(str(MAPS_SH))} save {shlex.quote(name)} "
        f"--robot-id {shlex.quote(robot_id)}{extra}"
    )
    try:
        proc = subprocess.run(
            ["bash", "-c", cmd], cwd=str(REPO_ROOT), env=clean_env(),
            capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired as e:
        raise maps_lib.MapError(f"maps.sh save timed out after {timeout:.0f}s") from e

    # Exit 5 is maps.sh's "saved, but db_state is pending" — a partial success
    # the UI reports rather than an error (see its header's exit-code table).
    if proc.returncode not in (0, 5):
        detail = (proc.stderr or proc.stdout or "").strip().splitlines()
        raise maps_lib.MapError(
            f"maps.sh save exited {proc.returncode}: "
            f"{detail[-1] if detail else 'no output'}"
        )
    manifest = maps_lib.read_manifest(name)
    manifest["log"] = (proc.stdout or "") + (proc.stderr or "")
    return manifest


_PIPELINE_PATTERNS = ("spawn_warehouse.py", "sortbots_bringup.launch.py")


def pipeline_alive() -> bool:
    """Is a sim pipeline up right now?

    Checked by process pattern rather than by pid: run_demo.sh setsid's its
    children and exits, so we never hold a handle on the things that matter.
    """
    for pattern in _PIPELINE_PATTERNS:
        found = subprocess.run(
            ["pgrep", "-f", "--", pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        if found.returncode == 0:
            return True
    return False


# ---------------------------------------------------------------------------
# Phases, parsed from run_demo.sh's own console output
# ---------------------------------------------------------------------------
# These strings live in scripts/run_demo.sh, which carries a comment pointing
# back here. Failure patterns are checked first so an error line can't be
# swallowed by a progress match on the same line.

PHASE_PATTERNS = [
    (re.compile(r"Isaac Sim failed to start|^ERROR:", re.M), "failed"),
    (re.compile(r"demo is UP"), "running"),
    (re.compile(r"ROS 2 stack up"), "ros_ready"),
    (re.compile(r"launching RTAB-Map"), "ros_starting"),
    (re.compile(r"Isaac Sim is up and publishing|sim not confirmed ready"), "sim_ready"),
    (re.compile(r"waiting for the warehouse to load"), "sim_loading"),
    (re.compile(r"launching Isaac Sim"), "sim_starting"),
]

PHASE_ORDER = [
    "idle", "sim_starting", "sim_loading", "sim_ready",
    "ros_starting", "ros_ready", "running",
]

PHASE_LABELS = {
    "idle": "idle",
    "sim_starting": "starting Isaac Sim",
    "sim_loading": "loading the warehouse",
    "sim_ready": "sim publishing",
    "ros_starting": "starting RTAB-Map + Nav2",
    "ros_ready": "ROS 2 stack up",
    "running": "running",
    "failed": "failed",
}


def phase_for_line(line: str) -> str | None:
    for rx, phase in PHASE_PATTERNS:
        if rx.search(line):
            return phase
    return None


# ---------------------------------------------------------------------------
# Session supervision
# ---------------------------------------------------------------------------

class SessionManager:
    """Owns at most one sim session at a time."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._session: dict | None = None
        self._proc: subprocess.Popen | None = None
        self._bag_proc: subprocess.Popen | None = None
        self._alive_cache = (0.0, False)
        self._adopt()

    # -- public API --------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            if self._session is None:
                return {"state": "idle", "phase": "idle", "phase_label": "idle",
                        "scenario": None, "session_id": None}
            session = dict(self._session)
            # A pipeline can die without us: Isaac crashing, someone running
            # `run_demo.sh stop` in a terminal, an OOM kill. Reflect that
            # rather than showing a session that stopped minutes ago as live.
            if session["state"] == "running" and not self._pipeline_alive_cached():
                self._session["state"] = "exited"
                self._session["error"] = self._session.get("error") or \
                    "pipeline is no longer running (stopped outside the dashboard?)"
                self._persist()
                session = dict(self._session)
            session["phase_label"] = PHASE_LABELS.get(session["phase"], session["phase"])
            # Freeze the clock once the session ends, or a finished run keeps
            # reporting an ever-growing runtime for as long as the console lives.
            end = session.get("stopped_at") or time.time()
            session["elapsed_s"] = round(end - session["started_at"], 1)
            return session

    def start(self, name: str, overrides: dict | None = None, force: bool = False) -> dict:
        with self._lock:
            if self.is_busy():
                if not force:
                    raise SessionConflict(
                        f"session {self._session['session_id']} is {self._session['state']}"
                    )
                self._stop_blocking()

            scenario = get_scenario(name)
            if scenario["status"] != "ready":
                raise ScenarioError(
                    f"scenario {name!r} has status {scenario['status']!r} and cannot be started"
                )
            run = apply_overrides(scenario, overrides)
            argv = [str(RUN_DEMO), *build_argv(run), "--keep-console"]

            session_id = f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{name}"
            session_dir = SESSIONS_DIR / session_id
            session_dir.mkdir(parents=True, exist_ok=True)
            log_path = session_dir / "console.log"

            self._session = {
                "session_id": session_id,
                "scenario": name,
                "title": scenario["title"],
                "dir": str(session_dir),
                "log": str(log_path),
                "argv": argv,
                "run": run,
                "state": "starting",
                "phase": "idle",
                "started_at": time.time(),
                "started_at_iso": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "exit_code": None,
                "error": None,
                "capture_bag": scenario["capture"]["bag"],
                "bag_dir": str(session_dir / "bag") if scenario["capture"]["bag"] else None,
            }

            log_fh = open(log_path, "wb")
            try:
                log_fh.write(f"$ {shlex.join(argv)}\n".encode())
                log_fh.flush()
                # start_new_session so a serve.py restart never takes the
                # launcher (or its children) down with it.
                self._proc = subprocess.Popen(
                    argv, cwd=str(REPO_ROOT), env=clean_env(),
                    stdout=log_fh, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True,
                )
            finally:
                log_fh.close()

            self._session["pid"] = self._proc.pid
            self._persist()
            threading.Thread(target=self._watch, args=(self._proc, log_path),
                             daemon=True).start()
            return self.status()

    def stop(self) -> dict:
        """Ask for teardown; returns immediately with state "stopping".

        run_demo.sh stop sleeps ~3s between its SIGINT and SIGKILL sweeps, and
        finalizing a bag adds more, so this runs on a thread rather than
        holding an HTTP request open for the duration. The UI polls /api/session.
        """
        with self._lock:
            if self._session is None:
                return self.status()
            self._session["state"] = "stopping"
            self._persist()
            threading.Thread(target=self._stop_blocking, daemon=True).start()
            return self.status()

    def is_busy(self) -> bool:
        with self._lock:
            if self._session is None:
                return False
            return self._session["state"] in ("starting", "running", "stopping")

    def log_tail(self, offset: int | None, limit: int = MAX_LOG_CHUNK) -> dict:
        """Incremental read of the console log by byte offset.

        offset=None (or a negative value) means "the tail" — the UI uses that
        on first load so opening the tab mid-run doesn't dump the whole log.
        """
        with self._lock:
            session = self._session
        if session is None:
            return {"offset": 0, "size": 0, "text": ""}
        path = Path(session["log"])
        if not path.exists():
            return {"offset": 0, "size": 0, "text": ""}
        size = path.stat().st_size
        limit = max(1, min(int(limit), MAX_LOG_CHUNK))
        if offset is None or offset < 0:
            start = max(0, size - limit)
        elif offset > size:
            start = 0  # log was replaced under us (new session)
        else:
            start = offset
        with open(path, "rb") as fh:
            fh.seek(start)
            data = fh.read(limit)
        return {
            "offset": start + len(data),
            "size": size,
            "text": data.decode("utf-8", "replace"),
        }

    # -- internals ---------------------------------------------------------

    def _pipeline_alive_cached(self) -> bool:
        # status() is polled once a second per open tab; pgrep twice per poll
        # per tab is silly. 2s of staleness is invisible in the UI.
        now = time.monotonic()
        stamp, value = self._alive_cache
        if now - stamp < 2.0:
            return value
        value = pipeline_alive()
        self._alive_cache = (now, value)
        return value

    def _advance(self, phase: str) -> None:
        """Move the phase forward only — never backward."""
        current = self._session["phase"]
        if phase == "failed":
            self._session["phase"] = "failed"
            return
        if current == "failed":
            return
        if PHASE_ORDER.index(phase) > PHASE_ORDER.index(current):
            self._session["phase"] = phase

    def _watch(self, proc: subprocess.Popen, log_path: Path) -> None:
        """Tail the launcher's output for phase transitions, then reap it."""
        with open(log_path, "rb") as fh:
            while True:
                line = fh.readline()
                if line:
                    self._consume(line.decode("utf-8", "replace"))
                    continue
                code = proc.poll()
                if code is None:
                    time.sleep(0.25)
                    continue
                for rest in fh.read().decode("utf-8", "replace").splitlines():
                    self._consume(rest)
                self._finish(proc, code)
                return

    def _consume(self, line: str) -> None:
        phase = phase_for_line(line)
        if phase is None:
            return
        with self._lock:
            if self._session is None or self._proc is None:
                return
            before = self._session["phase"]
            self._advance(phase)
            if self._session["phase"] != before:
                self._persist()
                if self._session["phase"] == "running" and self._session["capture_bag"]:
                    self._start_bag()

    def _finish(self, proc: subprocess.Popen, code: int) -> None:
        with self._lock:
            if self._session is None or self._proc is not proc:
                return  # superseded by a newer session
            self._session["exit_code"] = code
            if self._session["state"] == "stopping":
                pass  # _stop_blocking owns the final state
            elif code == 0 and self._session["phase"] == "running":
                # The expected success path: run_demo.sh printed its banner and
                # returned, leaving the setsid'd pipeline up.
                self._session["state"] = "running"
            else:
                self._session["state"] = "failed"
                self._session["phase"] = "failed"
                self._session["error"] = (
                    f"run_demo.sh exited {code} — see the log above"
                )
            self._persist()

    def _start_bag(self) -> None:
        """Record scripts/record_explore_bag.sh alongside the session log.

        Needs ROS 2 sourced (it shells out to `ros2 bag record`), which is
        exactly why it goes through `bash -c "source ...; exec ..."` from a
        clean env rather than inheriting ours.
        """
        session = self._session
        cmd = (
            f"source {shlex.quote(ROS_SETUP)}; "
            f"export PATH={shlex.quote(SYSTEM_PATH)}:\"$PATH\"; "
            f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; "
            f"exec {shlex.quote(str(RECORD_BAG))} --output {shlex.quote(session['bag_dir'])}"
        )
        env = clean_env()
        env["ROBOT_ID"] = session["run"]["robot_id"]
        log_fh = open(Path(session["dir"]) / "bag.log", "wb")
        try:
            self._bag_proc = subprocess.Popen(
                ["bash", "-c", cmd], cwd=str(REPO_ROOT), env=env,
                stdout=log_fh, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True,
            )
        finally:
            log_fh.close()

    def _stop_bag(self) -> None:
        proc = self._bag_proc
        self._bag_proc = None
        if proc is None or proc.poll() is not None:
            return
        # SIGINT, not SIGTERM: rosbag2 only closes and indexes the file on a
        # clean interrupt. Signal the group because it runs under `bash -c`.
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()

    def _stop_blocking(self) -> None:
        with self._lock:
            if self._session is None:
                return
            session_dir = Path(self._session["dir"])
            self._session["state"] = "stopping"
            self._persist()
            self._stop_bag()

        # --keep-console: spare rosbridge / web_video_server / serve.py, which
        # are the console this request arrived on.
        with open(session_dir / "console.log", "ab") as log_fh:
            log_fh.write(b"\n[session] stopping...\n")
            log_fh.flush()
            try:
                subprocess.run(
                    [str(RUN_DEMO), "stop", "--keep-console"],
                    cwd=str(REPO_ROOT), env=clean_env(),
                    stdout=log_fh, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, timeout=120,
                )
            except subprocess.TimeoutExpired:
                log_fh.write(b"[session] run_demo.sh stop timed out\n")

        with self._lock:
            if self._session is not None:
                self._session["state"] = "stopped"
                self._session["stopped_at"] = time.time()
                self._persist()
            self._proc = None
            self._alive_cache = (0.0, False)

    def _persist(self) -> None:
        session = self._session
        if session is None:
            return
        SESSIONS_DIR.mkdir(parents=True, exist_ok=True)
        body = json.dumps(session, indent=2)
        for path in (Path(session["dir"]) / "session.json", CURRENT_POINTER):
            tmp = path.with_suffix(".tmp")
            tmp.write_text(body)
            tmp.replace(path)

    def _adopt(self) -> None:
        """Re-attach to a session that outlived a previous serve.py.

        The pipeline is setsid'd, so restarting the console (or crashing it)
        leaves the sim running. Without this the tab would claim "idle" while
        Isaac and Nav2 are plainly up, and Start would fight a live pipeline.
        """
        if not CURRENT_POINTER.exists():
            return
        try:
            session = json.loads(CURRENT_POINTER.read_text())
        except (OSError, ValueError):
            return
        if session.get("state") not in ("starting", "running", "stopping"):
            self._session = session
            return
        if pipeline_alive():
            session["state"] = "running"
            session["adopted"] = True
        else:
            session["state"] = "exited"
            session["error"] = "pipeline was not running when the console started"
        self._session = session
        self._persist()


# ---------------------------------------------------------------------------
# Offline CLI — check scenarios and argv without launching anything
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--list", action="store_true", help="list scenarios and their status")
    parser.add_argument("--print-argv", metavar="SCENARIO",
                        help="print the run_demo.sh command a scenario would run")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="apply an override before printing (repeatable)")
    args = parser.parse_args()

    if args.list or not args.print_argv:
        for scenario in load_scenarios():
            mark = {"ready": "•", "planned": "·", "invalid": "!"}.get(scenario["status"], "?")
            print(f"{mark} {scenario['name']:<20} {scenario['status']:<8} {scenario['title']}")
            if scenario.get("error"):
                print(f"    {scenario['error']}")
        if not args.print_argv:
            return

    scenario = get_scenario(args.print_argv)
    overrides = {}
    for item in args.set:
        key, _, value = item.partition("=")
        overrides[key] = value
    run = apply_overrides(scenario, overrides)
    print(shlex.join([str(RUN_DEMO), *build_argv(run), "--keep-console"]))


if __name__ == "__main__":
    main()
