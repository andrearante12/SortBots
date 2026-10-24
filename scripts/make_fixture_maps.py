#!/usr/bin/env python3
"""Synthesize stand-ins for the saved maps that committed tests name.

The real library entries (maps/<name>/, RTAB-Map pose graphs in git-lfs) are
150-500 MB and are not in a hosted-CI checkout, so every check that resolves a
map by name (simtest validate, the operator dashboard test) used to skip or
fail there. A stand-in needs only what those offline checks read: a schema-valid
manifest, a trinary grid, and a readable sqlite file. It is NOT loadable by
RTAB-Map — never point a live run at it.

    python3 scripts/make_fixture_maps.py OUTDIR [NAME ...]
    SORTBOTS_MAPS_DIR=OUTDIR python3 scripts/simtest.py validate

Default NAMEs are the maps configs/ reference. Deliberately ROS-free (stdlib +
what maps_lib already imports), same as the rest of the offline layer.
"""
from __future__ import annotations

import os
import sqlite3
import sys
from pathlib import Path

DEFAULT_NAMES = ("fleet_warehouse_test",)
W, H, RES = 160, 120, 0.25   # 40 x 30 m, covers the spawn poses in configs/robots.yaml


def _grid(d: Path) -> Path:
    # trinary map_saver output: 254 free, 0 occupied, 205 unknown. A bordered room.
    rows = []
    for j in range(H):
        row = bytearray([254] * W)
        if j in (0, H - 1):
            row = bytearray([0] * W)
        else:
            row[0] = row[W - 1] = 0
        rows.append(bytes(row))
    (d / "grid.pgm").write_bytes(b"P5\n%d %d\n255\n" % (W, H) + b"".join(rows))
    (d / "grid.yaml").write_text(
        "image: grid.pgm\nmode: trinary\nresolution: %s\norigin: [-20.0, -10.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n" % RES)
    return d / "grid.yaml"


def make(out: Path, name: str) -> None:
    os.environ["SORTBOTS_MAPS_DIR"] = str(out)   # maps_lib reads this at import
    import importlib
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    maps_lib = importlib.import_module("maps_lib")
    importlib.reload(maps_lib)

    d = out / name
    d.mkdir(parents=True, exist_ok=True)
    src = d / "src.db"
    c = sqlite3.connect(src)
    c.execute("create table fixture (id integer primary key, note text)")
    c.execute("insert into fixture (note) values ('ci stand-in, not a pose graph')")
    c.commit(); c.close()

    m = maps_lib.new_manifest(name, title=f"{name} (CI fixture)",
                              description="Synthetic stand-in; not loadable by RTAB-Map.")
    m["grid"], m["coverage"] = maps_lib.grid_stats(_grid(d))
    m["dbs"]["robot_0"] = maps_lib.copy_db(src, d / "map.db", vacuum=False)
    m["db_state"] = maps_lib.rollup_db_state(m["dbs"])
    src.unlink()
    maps_lib.write_manifest(name, m)
    print(f"[fixture] {name}: {d}")


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    out = Path(argv[0]).resolve()
    for name in argv[1:] or DEFAULT_NAMES:
        make(out, name)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
