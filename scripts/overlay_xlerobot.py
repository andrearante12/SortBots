#!/usr/bin/env python3
"""Wire XLeRobot's ManiSkill files into the active env's site-packages.

Symlinks the agent + asset directories from the submodule, copies the
replacement base_env.py with a one-time backup, and (only if missing)
appends `from .xlerobot import *` to mani_skill/agents/robots/__init__.py.

Idempotent. Pure stdlib. Run inside the activated `lerobot` conda env.
"""
from __future__ import annotations

import os
import shutil
import sys
import sysconfig
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
SUBMODULE = REPO_ROOT / "third_party" / "XLeRobot" / "simulation" / "Maniskill"


def find_mani_skill_root() -> Path:
    sp = Path(sysconfig.get_paths()["purelib"])
    ms = sp / "mani_skill"
    if not ms.is_dir():
        sys.exit(
            f"ERROR: mani_skill not installed at {ms}.\n"
            "Activate the lerobot conda env and `pip install mani-skill` first."
        )
    return ms


def symlink_dir(src: Path, dst: Path) -> None:
    if not src.is_dir():
        sys.exit(f"ERROR: expected submodule directory not found: {src}")
    if dst.is_symlink():
        if dst.resolve() == src.resolve():
            print(f"  ok  symlink already points correctly: {dst}")
            return
        dst.unlink()
    elif dst.exists():
        backup = dst.with_suffix(dst.suffix + ".upstream.bak")
        if not backup.exists():
            print(f"  mv  {dst} -> {backup}")
            shutil.move(str(dst), str(backup))
        else:
            print(f"  rm  {dst} (backup already at {backup})")
            shutil.rmtree(dst) if dst.is_dir() and not dst.is_symlink() else dst.unlink()
    dst.parent.mkdir(parents=True, exist_ok=True)
    os.symlink(src, dst, target_is_directory=True)
    print(f"  ln  {dst} -> {src}")


def overlay_base_env(src: Path, dst: Path) -> None:
    if not src.is_file():
        sys.exit(f"ERROR: expected file not found: {src}")
    backup = dst.with_suffix(dst.suffix + ".upstream.bak")
    if dst.exists() and not backup.exists():
        # Only back up if dst is the *upstream* file (not already our overlay).
        try:
            same = dst.read_bytes() == src.read_bytes()
        except OSError:
            same = False
        if not same:
            print(f"  mv  {dst} -> {backup}  (one-time backup of upstream)")
            shutil.copy2(dst, backup)
    shutil.copy2(src, dst)
    print(f"  cp  {src} -> {dst}")
    _split_render_cam_branch(dst)


# XLeRobot ships a base_env.py that collapses fetch+xlerobot into one
# `_default_human_render_camera_configs` branch using `self.agent.top_base_link`.
# Fetch has no `top_base_link`, so any non-xlerobot ReplicaCAD demo (e.g. the
# default `demo_random_action`) crashes with AttributeError. Split the branch
# back into separate fetch (upstream render config) and xlerobot branches.

_COMBINED_BRANCH = (
    '        if self.robot_uids == "fetch" or self.robot_uids == "xlerobot":\n'
    '\n'
    '            robot_camera_pose = sapien_utils.look_at([1, 0, 0.6], [0, 0, 0.3])\n'
    '            robot_camera_config = CameraConfig(\n'
    '                "render_camera",  \n'  # NOTE: upstream XLeRobot has two trailing spaces here
    '                robot_camera_pose,\n'
    '                640,\n'
    '                480,\n'
    '                1,\n'
    '                0.01,\n'
    '                100,\n'
    '                mount=self.agent.top_base_link,\n'
    '            )\n'
    '            return [robot_camera_config]'
)

_SPLIT_BRANCHES = '''        if self.robot_uids == "fetch":
            room_camera_pose = sapien_utils.look_at([2.5, -2.5, 3], [0.0, 0.0, 0])
            room_camera_config = CameraConfig(
                "render_camera",
                room_camera_pose,
                512, 512, 1, 0.01, 100,
            )
            robot_camera_pose = sapien_utils.look_at([2, 0, 1], [0, 0, -1])
            robot_camera_config = CameraConfig(
                "robot_render_camera",
                robot_camera_pose,
                512, 512, 1.5, 0.01, 100,
                mount=self.agent.torso_lift_link,
            )
            return [room_camera_config, robot_camera_config]

        if self.robot_uids == "xlerobot":
            robot_camera_pose = sapien_utils.look_at([1, 0, 0.6], [0, 0, 0.3])
            robot_camera_config = CameraConfig(
                "render_camera",
                robot_camera_pose,
                640, 480, 1, 0.01, 100,
                mount=self.agent.top_base_link,
            )
            return [robot_camera_config]'''


def _split_render_cam_branch(base_env_path: Path) -> None:
    text = base_env_path.read_text()
    if _SPLIT_BRANCHES.splitlines()[0] in text and 'self.robot_uids == "xlerobot":' in text and "torso_lift_link" in text:
        print(f"  ok  base_env.py render-cam branches already split for fetch+xlerobot")
        return
    if _COMBINED_BRANCH not in text:
        print(
            f"  !!  WARNING: expected combined-branch pattern not found in {base_env_path}.\n"
            f"      The XLeRobot base_env.py may have changed shape — inspect manually."
        )
        return
    new = text.replace(_COMBINED_BRANCH, _SPLIT_BRANCHES, 1)
    base_env_path.write_text(new)
    print(f"  +   split fetch/xlerobot render-cam branches in base_env.py (Fetch demos restored)")


def patch_init(init_path: Path) -> None:
    if not init_path.is_file():
        sys.exit(f"ERROR: ManiSkill __init__ not found: {init_path}")
    text = init_path.read_text()
    needle = "from .xlerobot import *"
    if needle in text:
        print(f"  ok  '{needle}' already present in {init_path.name}")
        return
    new = text.rstrip() + f"\n{needle}\n"
    init_path.write_text(new)
    print(f"  +   appended '{needle}' to {init_path}")


def patch_replicacad_scene_builder(sb_path: Path) -> None:
    """Make ReplicaCAD's scene_builder.initialize() position xlerobot the same way it positions fetch.

    Upstream only branches on `robot_uids == "fetch"` and raises NotImplementedError
    for anything else. We rewrite that single line to accept xlerobot too. Idempotent.
    """
    if not sb_path.is_file():
        sys.exit(f"ERROR: ReplicaCAD scene_builder.py not found: {sb_path}")
    text = sb_path.read_text()
    needle_done = 'self.env.robot_uids in ("fetch", "xlerobot")'
    needle_orig = 'if self.env.robot_uids == "fetch":'
    if needle_done in text:
        print(f"  ok  scene_builder.py already accepts xlerobot")
        return
    if needle_orig not in text:
        sys.exit(
            f"ERROR: expected upstream pattern not found in {sb_path}.\n"
            "ManiSkill may have changed; inspect manually."
        )
    backup = sb_path.with_suffix(sb_path.suffix + ".upstream.bak")
    if not backup.exists():
        print(f"  mv  {sb_path} -> {backup}  (one-time backup of upstream)")
        shutil.copy2(sb_path, backup)
    new = text.replace(
        needle_orig,
        'if self.env.robot_uids in ("fetch", "xlerobot"):',
        1,
    )
    sb_path.write_text(new)
    print(f"  +   patched scene_builder.py to accept xlerobot in ReplicaCAD init")


def sanity_check() -> None:
    print("\nSanity check: importing mani_skill.agents.robots.Xlerobot ...")
    import importlib

    mod = importlib.import_module("mani_skill.agents.robots")
    cls = getattr(mod, "Xlerobot", None)
    if cls is None:
        sys.exit("ERROR: `Xlerobot` not exported from mani_skill.agents.robots")
    uid = getattr(cls, "uid", None)
    print(f"  ok  Xlerobot.uid = {uid!r}")


def main() -> int:
    ms = find_mani_skill_root()
    print(f"ManiSkill site-packages root: {ms}")
    print(f"Submodule root:               {SUBMODULE}")

    print("\n[1/5] Symlink agents/robots/xlerobot")
    symlink_dir(
        SUBMODULE / "agents" / "xlerobot",
        ms / "agents" / "robots" / "xlerobot",
    )

    print("\n[2/5] Symlink assets/robots/xlerobot")
    symlink_dir(
        SUBMODULE / "assets" / "xlerobot",
        ms / "assets" / "robots" / "xlerobot",
    )

    print("\n[3/5] Overlay envs/scenes/base_env.py")
    overlay_base_env(
        SUBMODULE / "envs" / "scenes" / "base_env.py",
        ms / "envs" / "scenes" / "base_env.py",
    )

    print("\n[4/5] Guard-patch agents/robots/__init__.py")
    patch_init(ms / "agents" / "robots" / "__init__.py")

    print("\n[5/5] Patch ReplicaCAD scene_builder to position xlerobot")
    patch_replicacad_scene_builder(
        ms / "utils" / "scene_builder" / "replicacad" / "scene_builder.py"
    )

    sanity_check()
    print("\nOverlay complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
