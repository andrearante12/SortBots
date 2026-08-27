#!/usr/bin/env python3
"""Run an XLeRobot example script from the submodule with mani_skill envs registered.

The XLeRobot examples (in third_party/XLeRobot/simulation/Maniskill/examples/)
do not `import mani_skill.envs`, so running them standalone leaves gym's env
registry empty and `gym.make("ReplicaCAD_SceneManipulation-v1", ...)` fails
with `NameNotFound`. This shim imports the env registry first, then re-execs
the chosen example with the remaining argv passed through unchanged.

Usage:
    python scripts/run_xle_demo.py <demo_name> [demo args...]

Where <demo_name> is the file stem under
    third_party/XLeRobot/simulation/Maniskill/examples/
e.g. `demo_ctrl_action`, `demo_ctrl_action_ee_keyboard`, `demo_ctrl_action_ee_keyboard_single`.
"""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

import mani_skill.envs  # noqa: F401  -- side effect: register gym envs


EXAMPLES_DIR = (
    Path(__file__).resolve().parent.parent
    / "third_party"
    / "XLeRobot"
    / "simulation"
    / "Maniskill"
    / "examples"
)


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        print("Available demos:")
        for p in sorted(EXAMPLES_DIR.glob("*.py")):
            print(f"  {p.stem}")
        return 0
    demo_name = sys.argv[1]
    demo_path = EXAMPLES_DIR / f"{demo_name}.py"
    if not demo_path.is_file():
        sys.exit(f"ERROR: demo '{demo_name}' not found at {demo_path}")
    # Reshape argv so the example sees itself as argv[0] and gets its own args.
    sys.argv = [str(demo_path)] + sys.argv[2:]
    runpy.run_path(str(demo_path), run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main())
