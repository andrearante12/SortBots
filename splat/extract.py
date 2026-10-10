"""Run `rtabmap-export` on a pose-graph DB. The only ROS touch in the pipeline.

The worker runs in the `splat` conda env (python 3.10, torch). rtabmap-export
is a C++ binary from system Jazzy and needs /opt/ros/jazzy sourced for its
shared libs (without it: "librtabmap_core.so.0.22: cannot open shared object
file"). So it gets its own `bash -c` subshell with a MINIMAL environment —
never the worker's: conda's PATH/LD vars must not leak into a ROS process, and
sourcing ROS must not leak back into the trainer (CLAUDE.md invariants).

Running it IN PLACE on a library DB is safe: unlike the rtabmap node (which
opens its sqlite read-write even under --localize — why run_demo.sh copies),
the exporter only reads. Verified 2026-10-10 on maps/warehouse/map.db: mtime
and sha256 unchanged after an export, matching map.json's recorded hash.
"""
from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
SYSTEM_PATH = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

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


def run_export(db: Path, out_dir: Path, prefix: str, log, timeout: float = 3600) -> None:
    """Export `db`, streaming output to the open file `log`. Raises on failure."""
    out_dir.mkdir(parents=True, exist_ok=True)
    log.write(f"$ {' '.join(export_argv(db, out_dir, prefix))}\n")
    log.flush()
    proc = subprocess.run(export_argv(db, out_dir, prefix), env=export_env(),
                          stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(f"rtabmap-export exited {proc.returncode} (see job log)")
