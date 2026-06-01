#!/usr/bin/env python3
"""Headless Isaac Sim 5.1 verification — step an empty stage 100 times.

Run after `source scripts/activate_isaac.sh`:
    python scripts/verify_isaac_sim.py

Exit code 0 on success. The 100-step loop is wrapped in a 120 s timeout so
"Kit silently fell back to llvmpipe" failures surface clearly.
"""
from __future__ import annotations

import os
import sys
import time

from _isaac_utils import (
    cancel_timeout,
    emit_factory,
    gpu_name,
    guard_against_ros2,
    install_timeout,
)

guard_against_ros2()

# SimulationApp MUST be constructed before any other omni / isaacsim imports.
from isaacsim import SimulationApp  # noqa: E402

CONFIG = {
    "headless": True,
    "renderer": "RayTracedLighting",
}

simulation_app = SimulationApp(CONFIG)

install_timeout(
    seconds=120,
    hint=(
        "Likely Kit did not pick up the dGPU. Check vulkaninfo and\n"
        "__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia."
    ),
    close_app=simulation_app.close,
)

RESULT_FILE = os.environ.get("ISAAC_VERIFY_RESULT", "/tmp/isaac_verify_result.txt")
_emit = emit_factory(RESULT_FILE)


def main() -> int:
    import isaacsim
    from isaacsim.core.api import World

    version = getattr(isaacsim, "__version__", "unknown")
    _emit(f"isaacsim {version} | GPU: {gpu_name()}")

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
    cancel_timeout()
    simulation_app.close()

sys.exit(rc)
