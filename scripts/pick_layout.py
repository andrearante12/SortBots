#!/usr/bin/env python3
"""Lay out a pick world: where a package goes on a deck, where the robot docks, and
whether a world's docks and transits are clear. Pure python + numpy, no ROS, no Isaac.

    scripts/pick_layout.py package --deck -8 10 0.5 0.5 0.80 --normal 90 [--lateral 0.1]
    scripts/pick_layout.py check pick_orientations [--map fleet_warehouse_test]

`package` takes a deck (centre x y, size x y, height) and the direction the magnet plate
should face (degrees, map frame: 0 = +x, 90 = +y) and prints the package's world-file
line, the base's dock and staging poses, and whether the arm can reach the plate from the
dock (nodes/arm_kin.py plan_reach). The package overhangs the deck edge by `--overhang`
(default 2 cm): a full-height deck stops the chassis' frame ~16 cm short of its face, so
a plate flush with the edge is ~1 cm further than the arm likes (simtest/magnet.py).

`check` runs the same questions over a committed world, in file order (the order a test
asks for the packages): is each dock's chassis clear of every prop, is the plate within
the arm's reach from it, and are the straight transits spawn -> staging -> staging clear
of the props. With `--map` it also looks the docks and transits up in a saved grid
(maps/<name>/grid.pgm) and says where they cross an occupied cell. Exit 0 = clean.

The same geometry is asserted by tests/simtest_world_test.py; this is for writing a new
world, where it tells you the numbers instead of just failing.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "nodes"))

from simtest import geometry, magnet, world  # noqa: E402

STAGE_M = 0.35        # nodes/pick_planner.py: the staging point in front of a plate
CIRCLE_R = 0.26       # the chassis' swept radius (its frame's corners, simtest.magnet.CHASSIS_HALF_M)
OVERHANG_M = 0.02
SIZE = world.DEFAULT_PACKAGE_SIZE


def package_on_deck(deck: tuple, normal_deg: float, *, lateral: float = 0.0,
                    overhang: float = OVERHANG_M, size=SIZE) -> dict:
    """The pose of a package (magnet face "+x", yaw = normal) standing on the deck edge
    that faces `normal_deg`, its plate `overhang` past the edge and `lateral` metres along
    it. deck = (cx, cy, sx, sy, height)."""
    cx, cy, sx, sy, h = deck
    th = math.radians(normal_deg)
    n, t = (math.cos(th), math.sin(th)), (-math.sin(th), math.cos(th))
    half = abs(n[0]) * sx / 2 + abs(n[1]) * sy / 2        # deck half-extent along the normal
    reach = half + overhang - size[0] / 2                  # plate at edge+overhang, face +x
    return {"x": cx + n[0] * reach + t[0] * lateral, "y": cy + n[1] * reach + t[1] * lateral,
            "z": h, "yaw": th}


def dock_for(pose: dict, size=SIZE, face: str = "+x") -> dict:
    """Plate, dock pose and staging pose for a package at `pose` ({x, y, z, yaw})."""
    centre, normal = magnet.magnet_face_pose(pose, size, face)
    base = magnet.dock_pose(centre, normal)
    n = (normal[0], normal[1])
    stage = (base[0] + n[0] * STAGE_M, base[1] + n[1] * STAGE_M, base[2])
    return {"plate": centre, "normal": normal, "dock": base, "stage": stage}


def reach_of(dock: dict) -> dict | None:
    """arm_kin.plan_reach for the plate from the dock, or None (also when the URDF
    submodule is missing: the caller says which)."""
    import arm_kin

    kin = arm_kin.ArmKin()
    return arm_kin.plan_reach(kin, magnet.to_base(dock["dock"], dock["plate"]),
                              magnet.dir_to_base(dock["dock"], dock["normal"]))


class Grid:
    """A saved map's occupancy grid (maps/<name>/grid.pgm + grid.yaml). Occupied cells
    block; unknown cells do not (the robot's own footprint is unknown in a map)."""

    def __init__(self, name: str):
        import numpy as np
        import yaml

        d = REPO_ROOT / "maps" / name
        meta = yaml.safe_load((d / "grid.yaml").read_text())
        with open(d / "grid.pgm", "rb") as f:
            assert f.readline().strip() == b"P5"
            w, h = map(int, f.readline().split())
            f.readline()
            self.img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
        self.w, self.h = w, h
        self.res, (self.ox, self.oy) = float(meta["resolution"]), meta["origin"][:2]

    def blocked(self, x: float, y: float) -> bool:
        i = int((x - self.ox) / self.res)
        j = self.h - 1 - int((y - self.oy) / self.res)
        return not (0 <= i < self.w and 0 <= j < self.h) or self.img[j, i] < 100


def chassis_blocked(grid: Grid, base: tuple, margin: float = 0.04) -> bool:
    hx, hy = magnet.CHASSIS_HALF_M
    c, s = math.cos(base[2]), math.sin(base[2])
    xs = [i * 0.05 - (hx + margin) for i in range(int(2 * (hx + margin) / 0.05) + 2)]
    ys = [i * 0.05 - (hy + margin) for i in range(int(2 * (hy + margin) / 0.05) + 2)]
    return any(grid.blocked(base[0] + c * x - s * y, base[1] + s * x + c * y)
               for x in xs for y in ys)


def check_world(name: str, grid: Grid | None) -> list[str]:
    w = world.load(name)
    if not w["packages"]:
        return [f"{name}: no packages"]
    import yaml

    spawn = tuple(next(r for r in yaml.safe_load((REPO_ROOT / "configs" / "robots.yaml").read_text())
                       ["robots"] if r["id"] == "robot_0")["spawn"]["nvidia"][:2])
    problems, stages = [], [spawn]
    hx, hy = magnet.CHASSIS_HALF_M
    for pkg in w["packages"]:
        x, y, yaw = pkg["pose"]
        d = dock_for({"x": x, "y": y, "z": pkg["z"], "yaw": yaw}, pkg["size"], pkg["magnet_face"])
        dock, stage = d["dock"], d["stage"]
        stages.append(stage[:2])
        chassis = geometry.rect(dock[0], dock[1], dock[2], 2 * hx, 2 * hy)
        for prop in w["props"]:
            gap = geometry.distance(chassis, world.prop_footprint(prop))
            if gap <= 0.005:
                problems.append(f"{pkg['id']}: dock chassis touches {prop['id']}")
        try:
            plan = reach_of(d)
            if plan is None:
                problems.append(f"{pkg['id']}: no arm solution for the plate from the dock "
                                f"(plate z {d['plate'][2]:.2f})")
        except Exception as e:  # noqa: BLE001 - e.g. the submodule's URDF is missing
            problems.append(f"{pkg['id']}: reach not checked ({e})")
        if grid and chassis_blocked(grid, dock):
            problems.append(f"{pkg['id']}: dock chassis is on an occupied map cell")
    for a, b in zip(stages, stages[1:]):
        n = max(2, int(math.dist(a, b) / 0.05))
        for k in range(n + 1):
            px, py = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            hit = [p["id"] for p in w["props"] if geometry.distance(
                geometry.circle(px, py, CIRCLE_R), world.prop_footprint(p)) <= 0.02]
            if hit or (grid and any(grid.blocked(px + CIRCLE_R * math.cos(t), py + CIRCLE_R * math.sin(t))
                                    for t in [i * math.pi / 6 for i in range(12)])):
                problems.append(f"transit ({a[0]:.2f}, {a[1]:.2f}) -> ({b[0]:.2f}, {b[1]:.2f}) "
                                f"is blocked near ({px:.2f}, {py:.2f})" + (f" by {hit}" if hit else ""))
                break
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("package", help="a package on a deck: its pose and the robot's dock")
    p.add_argument("--deck", nargs=5, type=float, required=True,
                   metavar=("X", "Y", "SX", "SY", "H"), help="deck centre, footprint and height")
    p.add_argument("--normal", type=float, required=True, help="plate faces this way, degrees (0=+x)")
    p.add_argument("--lateral", type=float, default=0.0, help="metres along the edge")
    p.add_argument("--overhang", type=float, default=OVERHANG_M)
    c = sub.add_parser("check", help="a committed world's docks, reach and transits")
    c.add_argument("world")
    c.add_argument("--map", help="a saved map under maps/ to check the docks and transits against")
    a = ap.parse_args(argv)

    if a.cmd == "package":
        pose = package_on_deck(tuple(a.deck), a.normal, lateral=a.lateral, overhang=a.overhang)
        d = dock_for(pose)
        print(f'{{id: pkg_0, pose: [{pose["x"]:.3f}, {pose["y"]:.3f}, {pose["yaw"]:.4f}], '
              f'z: {pose["z"]:g}, magnet_face: "+x"}}')
        print(f"plate   ({d['plate'][0]:.3f}, {d['plate'][1]:.3f}, {d['plate'][2]:.3f}), "
              f"normal ({d['normal'][0]:.2f}, {d['normal'][1]:.2f})")
        print(f"dock    x {d['dock'][0]:.3f}  y {d['dock'][1]:.3f}  yaw {d['dock'][2]:.4f}")
        print(f"stage   x {d['stage'][0]:.3f}  y {d['stage'][1]:.3f}  ({STAGE_M} m out from the dock)")
        try:
            plan = reach_of(d)
        except Exception as e:  # noqa: BLE001
            print(f"reach   not checked ({e})")
            return 0
        if plan is None:
            print("reach   NO arm solution for this plate from the dock")
            return 1
        print(f"reach   ok: approach tilted {plan['elevation_deg']:+.0f} deg, "
              f"{plan['angle_deg']:.1f} deg off the plate's normal")
        return 0

    grid = Grid(a.map) if a.map else None
    problems = check_world(a.world, grid)
    for p_ in problems:
        print("-", p_)
    print("clean" if not problems else f"{len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
