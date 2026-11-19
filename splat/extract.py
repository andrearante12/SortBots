"""Turn a pose-graph DB into rtabmap-export's file layout (splat/dataset.py's input).

Two backends, picked by configs/splat.yaml `extract.backend`:

  python          splat/rtabmap_db.py: reads the sqlite DB directly. No ROS, so it
                  runs on any machine with the splat env, including the RTX 4070
                  laptop on Ubuntu 26.04, where Jazzy has no packages (2026-11-17).
                  It also chooses between the optimized and the odometry poses by
                  measuring which agree better across keyframes (rtabmap_db.py,
                  "Which poses").
  rtabmap-export  the C++ exporter from system Jazzy, in a ROS-sourced subshell.
                  Always the optimized graph. Kept for comparison, and for a DB
                  newer than rtabmap_db.py understands.
  auto            python, on every machine. Not "rtabmap-export where Jazzy has it":
                  only the python backend picks its poses, and on
                  maps/fleet_warehouse_test the pose set it picked (odometry) agreed
                  with the keyframes' depth to 0.60 cm against 3.81 cm for the
                  optimized graph rtabmap-export always uses (2026-11-17). The same
                  map must not splat worse on the Jazzy desktop than on the laptop.

The rtabmap-export subshell gets a MINIMAL environment, never the worker's:
conda's/the venv's PATH and LD vars must not leak into a ROS process, and
sourcing ROS must not leak back into the trainer (CLAUDE.md invariants). It
needs /opt/ros/jazzy sourced for its shared libs (without it:
"librtabmap_core.so.0.22: cannot open shared object file").

Both read a library DB IN PLACE safely: rtabmap_db.py opens it `mode=ro`, and
unlike the rtabmap node (which opens its sqlite read-write even under
--localize — why run_demo.sh copies) the exporter only reads. Verified
2026-10-10 on maps/warehouse/map.db: mtime and sha256 unchanged after an export.
"""
from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path

import rtabmap_db

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
SYSTEM_PATH = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
BACKENDS = ("auto", "python", "rtabmap-export")

# --poses_camera: optical-frame poses (what a splat camera is), not base_link.
# --poses_format 11: `stamp x y z qx qy qz qw id` — the id ties a pose to its
#   --images_id files; format 10 drops it.
# --cloud: the assembled RTAB-Map cloud, used to initialise the gaussians.
EXPORT_FLAGS = ("--images_id", "--poses_camera", "--poses_format", "11", "--cloud")


def export_argv(db: Path, out_dir: Path, prefix: str) -> list[str]:
    """argv that exports `db` into `out_dir/<prefix>_*`. Pure, for tests."""
    cmd = (
        f"source {shlex.quote(ROS_SETUP)}; "
        # PREPEND, as session.py does: sourcing ROS just put /opt/ros/jazzy/bin
        # on PATH, and replacing PATH here loses rtabmap-export itself.
        f"export PATH={shlex.quote(SYSTEM_PATH)}:\"$PATH\"; "
        f"exec rtabmap-export {' '.join(EXPORT_FLAGS)} "
        f"--output_dir {shlex.quote(str(out_dir))} --output {shlex.quote(prefix)} "
        f"{shlex.quote(str(db))}"
    )
    return ["bash", "-c", cmd]


def export_env() -> dict:
    """Just enough environment for a ROS-sourcing subshell; nothing from conda."""
    keep = ("HOME", "USER", "LANG", "LC_ALL", "TMPDIR")
    env = {k: os.environ[k] for k in keep if k in os.environ}
    env["PATH"] = SYSTEM_PATH
    return env


def jazzy_available() -> bool:
    return Path(ROS_SETUP).is_file() and Path("/opt/ros/jazzy/bin/rtabmap-export").exists()


def pick_backend(backend: str) -> str:
    """configs/splat.yaml extract.backend -> the backend that will run."""
    if backend not in BACKENDS:
        raise ValueError(f"extract.backend {backend!r}: expected one of {', '.join(BACKENDS)}")
    if backend == "auto":
        return "python"
    return backend


def run_export(db: Path, out_dir: Path, prefix: str, log, timeout: float = 3600, *,
               backend: str = "python", poses: str = "auto") -> dict:
    """Export `db`, writing progress to the open file `log`. Raises on failure.
    Returns rtabmap_db's summary (python backend) or {} (rtabmap-export)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    backend = pick_backend(backend)
    if backend == "python":
        def say(msg):
            log.write(msg + "\n")
            log.flush()
        say(f"rtabmap_db export {db} (poses: {poses})")
        try:
            return rtabmap_db.export(db, out_dir, prefix, log=say, poses_source=poses)
        except rtabmap_db.DbError as e:
            raise RuntimeError(f"rtabmap_db: {e}") from e
    if not jazzy_available():
        raise RuntimeError("extract.backend rtabmap-export needs system Jazzy's "
                           "/opt/ros/jazzy/bin/rtabmap-export; "
                           "use backend: python on a machine without Jazzy")
    log.write(f"$ {' '.join(export_argv(db, out_dir, prefix))}\n")
    log.flush()
    proc = subprocess.run(export_argv(db, out_dir, prefix), env=export_env(),
                          stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(f"rtabmap-export exited {proc.returncode} (see job log)")
    return {}
