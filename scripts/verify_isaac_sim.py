#!/usr/bin/env python3
"""Headless Isaac Sim 5.1 verification — step an empty stage 100 times.

Run after `source scripts/activate_isaac.sh`:
    python scripts/verify_isaac_sim.py

Exit code 0 on success. The 100-step loop is wrapped in a 120 s timeout so
"Kit silently fell back to llvmpipe" failures surface clearly.
"""
from __future__ import annotations

import os
import signal
import sys
import time

# SimulationApp MUST be constructed before any other omni / isaacsim imports.
from isaacsim import SimulationApp  # noqa: E402

CONFIG = {
    "headless": True,
    "renderer": "RayTracedLighting",
}

simulation_app = SimulationApp(CONFIG)


def _timeout_handler(signum, frame):  # noqa: ARG001
    print("ERROR: verify timed out after 120 s.", file=sys.stderr)
    print("       Likely Kit did not pick up the dGPU. Check vulkaninfo and", file=sys.stderr)
    print("       __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia.", file=sys.stderr)
    try:
        simulation_app.close()
    finally:
        sys.exit(2)


signal.signal(signal.SIGALRM, _timeout_handler)
signal.alarm(120)


def _gpu_name() -> str:
    # `carb.settings` exposes the renderer's chosen GPU.
    try:
        import carb.settings

        s = carb.settings.get_settings()
        name = s.get("/renderer/activeDevice/name") or s.get("/renderer/activeDeviceName")
        if name:
            return str(name)
    except Exception:
        pass
    return "unknown"


RESULT_FILE = os.environ.get("ISAAC_VERIFY_RESULT", "/tmp/isaac_verify_result.txt")


def _emit(line: str) -> None:
    """Write a line both to stdout and to RESULT_FILE.

    Kit captures stdout during its startup and may swallow late writes, so we
    also persist the same lines to a file the caller can read after exit.
    """
    print(line, flush=True)
    with open(RESULT_FILE, "a") as f:
        f.write(line + "\n")


def main() -> int:
    import isaacsim
    from isaacsim.core.api import World

    # Truncate the result file at the start of each run.
    open(RESULT_FILE, "w").close()

    version = getattr(isaacsim, "__version__", "unknown")
    _emit(f"isaacsim {version} | GPU: {_gpu_name()}")

    world = World()
    world.reset()

    n_steps = 100
    t0 = time.perf_counter()
    for _ in range(n_steps):
        world.step(render=False)
    elapsed = time.perf_counter() - t0

    sim_time = float(world.current_time)
    _emit(f"Stepped {n_steps}/{n_steps} frames in {elapsed:.2f}s | sim_time={sim_time:.3f}s")

    assert sim_time > 0.0, "world.current_time did not advance"

    _emit("verify_isaac_sim: OK")
    return 0


try:
    rc = main()
except Exception as e:
    print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
    rc = 1
finally:
    signal.alarm(0)
    simulation_app.close()

sys.exit(rc)
