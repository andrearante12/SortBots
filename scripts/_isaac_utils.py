"""Shared helpers for Isaac Sim scripts (verify, importer, future tests).

Kept stdlib-only at import time: `isaacsim` / `omni` must be imported lazily
inside functions because constructing `SimulationApp` has side effects that
must happen before any other Kit imports.
"""
from __future__ import annotations

import os
import signal
import sys
from typing import Callable, Optional


def emit_factory(result_path: str) -> Callable[[str], None]:
    """Return an `emit(line)` callable that writes to stdout AND `result_path`.

    Kit captures stdout during its startup and may swallow late writes, so we
    also persist the same lines to a file the caller can read after exit.
    Truncates `result_path` on first call (per-process).
    """
    state = {"truncated": False}

    def emit(line: str) -> None:
        if not state["truncated"]:
            open(result_path, "w").close()
            state["truncated"] = True
        print(line, flush=True)
        with open(result_path, "a") as f:
            f.write(line + "\n")

    return emit


def install_timeout(
    seconds: int,
    hint: str,
    close_app: Optional[Callable[[], None]] = None,
) -> None:
    """SIGALRM handler that prints `hint` then exits 2.

    `close_app` is called best-effort before exit (typically
    `simulation_app.close`). Pass None if no app exists yet.
    """

    def handler(signum, frame):  # noqa: ARG001
        print(f"ERROR: timed out after {seconds} s.", file=sys.stderr)
        for line in hint.splitlines():
            print(f"       {line}", file=sys.stderr)
        if close_app is not None:
            try:
                close_app()
            except Exception:
                pass
        sys.exit(2)

    signal.signal(signal.SIGALRM, handler)
    signal.alarm(seconds)


def cancel_timeout() -> None:
    signal.alarm(0)


def gpu_name() -> str:
    """Return the renderer's chosen GPU name, or 'unknown'."""
    try:
        import carb.settings

        s = carb.settings.get_settings()
        name = s.get("/renderer/activeDevice/name") or s.get("/renderer/activeDeviceName")
        if name:
            return str(name)
    except Exception:
        pass
    return "unknown"


def guard_against_ros2() -> None:
    """Exit 1 if ROS 2 has been sourced into this shell.

    `AMENT_PREFIX_PATH` is the canonical signal. ROS 2 rewrites PYTHONPATH in
    ways that break Kit's extension resolver, so we fail fast rather than
    produce cryptic import errors deep in extension startup.
    """
    if os.environ.get("AMENT_PREFIX_PATH"):
        print(
            "ERROR: ROS 2 is sourced in this shell (AMENT_PREFIX_PATH set).\n"
            "       Open a fresh terminal and run `source scripts/activate_isaac.sh`\n"
            "       without sourcing /opt/ros/*/setup.bash.",
            file=sys.stderr,
        )
        sys.exit(1)
