#!/usr/bin/env python3
"""Turn a recorded rosbag2 into the small JSON fixture the dashboard tests use.

    scripts/record_dashboard_bag.sh          # needs the sim (once)
    python3 scripts/bag_to_fixture.py <bag>  # <- this file, offline
    node webui/tests/dashboard_test.mjs      # offline, no ROS at all

Why a fixture and not just `ros2 bag play`: the headless dashboard test should
run with no ROS installed, in seconds, so a CSS change can be checked without
sourcing anything. The bag stays the source of truth (it's replayable through
a real rosbridge with scripts/replay_dashboard_bag.sh); this derives a trimmed,
committable snapshot from it.

FIDELITY: messages are serialised with rosbridge's *own* converter,
`rosbridge_library.internal.message_conversion.extract_values` — the exact
function rosbridge_server calls before putting a message on the websocket. That
matters more than it looks: rosbridge base64-encodes `uint8[]` fields (which is
why webui/app.js's cloud_map handler has a "string or byte array" branch) but
emits plain JSON arrays for `int8[]` like OccupancyGrid.data. Hand-rolling this
would produce a fixture whose shape the real system never emits, and the tests
would happily pass against a format that can't occur in production.

Camera frames are handled separately: the dashboard fetches JPEG snapshots from
web_video_server over HTTP, not over rosbridge, so raw sensor_msgs/Image
messages get decoded to .jpg files that the test harness serves in place of
web_video_server.

Run with system ROS 2 Jazzy sourced (needs rosbag2_py + rclpy + rosbridge_library):

    source /opt/ros/jazzy/setup.bash
    python3 scripts/bag_to_fixture.py data/bags/dashboard_20260719_120000/
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "webui" / "testdata"

# Per-topic caps. These exist to keep the committed fixture small; every one of
# them drops data, so any drop is reported in the summary rather than being
# silently applied (a fixture that looks complete but isn't would send us
# chasing phantom dashboard bugs).
CAPS = {
    "map": 2,             # first + last: enough to prove the grid re-renders
    "global_costmap/costmap": 2,
    "plan": 8,
    "info": 12,
    "task_status": 40,
    "odom": 60,
    "tf": 200,            # the trail needs a real run of poses
    "tf_static": 4,
}
MAX_CLOUD_POINTS = 50_000   # decimate cloud_map below this
MAX_IMAGE_FRAMES = 6        # per camera topic
DEFAULT_MAX_MB = 5.0


def _fail(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(1)


try:
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbridge_library.internal import message_conversion
    from rosbridge_library.internal.message_conversion import extract_values
except ImportError as exc:  # pragma: no cover - depends on the shell's env
    _fail(
        f"missing ROS 2 / rosbridge Python packages ({exc}).\n"
        "  Source ROS 2 first:  source /opt/ros/jazzy/setup.bash\n"
        "  and make sure `which python3` is /usr/bin/python3, not conda's."
    )

# extract_values() asserts unless the module has been configured to pick a
# binary encoder — normally rosbridge_server does this at startup from its own
# parameters. Calling configure() with no arguments selects the same default
# the live stack uses (standard_b64encode), because
# launch/sortbots_webui.launch.py doesn't override `binary_encoder`. If that
# launch file ever sets it, mirror the value here or the fixture stops matching
# the wire format.
message_conversion.configure()


def open_bag(bag_dir: Path):
    """Open a rosbag2 directory and return (reader, {topic: type_name})."""
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as exc:
        _fail(f"could not open bag at {bag_dir}: {exc}")
    return reader, {t.name: t.type for t in reader.get_all_topics_and_types()}


def short_name(topic: str, robot_id: str) -> str:
    """`/robot_0/global_costmap/costmap` -> `global_costmap/costmap`."""
    return topic.lstrip("/").removeprefix(f"{robot_id}/")


def decimate_cloud(msg, max_points: int) -> int:
    """Thin a PointCloud2 in place to at most `max_points`. Returns points kept.

    Operates on the raw byte buffer by point_step so it stays agnostic to the
    field layout (RTAB-Map's cloud is xyz+rgb, but this doesn't need to know).
    """
    total = msg.width * msg.height
    if total <= max_points:
        return total
    step = msg.point_step
    stride = (total + max_points - 1) // max_points
    raw = bytes(msg.data)
    kept = bytearray()
    for i in range(0, total, stride):
        kept += raw[i * step:(i + 1) * step]
    n = len(kept) // step
    msg.data = bytes(kept)
    msg.height = 1
    msg.width = n
    msg.row_step = step * n
    msg.is_dense = True
    return n


def image_to_jpeg(msg, path: Path) -> None:
    """Decode a sensor_msgs/Image to JPEG.

    Deliberately not via cv_bridge: this keeps the converter working under a
    plain `python3` with numpy+PIL, and the encodings Isaac publishes are all
    trivially reshapeable anyway.
    """
    from PIL import Image

    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        arr = buf.reshape(msg.height, msg.width, 3)
        if enc == "bgr8":
            arr = arr[:, :, ::-1]
    elif enc in ("rgba8", "bgra8"):
        arr = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
        if enc == "bgra8":
            arr = arr[:, :, ::-1]
    elif enc == "mono8":
        arr = buf.reshape(msg.height, msg.width)
    else:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    Image.fromarray(arr).save(path, "JPEG", quality=80)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path, help="rosbag2 directory from record_dashboard_bag.sh")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT,
                    help=f"fixture output directory (default: {DEFAULT_OUT})")
    ap.add_argument("--robot-id", default="robot_0")
    ap.add_argument("--max-mb", type=float, default=DEFAULT_MAX_MB,
                    help=f"fail if the fixture exceeds this (default: {DEFAULT_MAX_MB})")
    ap.add_argument("--max-cloud-points", type=int, default=MAX_CLOUD_POINTS)
    args = ap.parse_args()

    if not args.bag.exists():
        _fail(f"no such bag: {args.bag}")

    reader, type_map = open_bag(args.bag)
    if not type_map:
        _fail(f"bag at {args.bag} declares no topics")

    frames_dir = args.out / "frames"
    if frames_dir.exists():
        shutil.rmtree(frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)

    topics: dict[str, dict] = {}
    images: dict[str, list[str]] = {}
    counts: dict[str, int] = {}     # messages actually read, per topic
    t0: int | None = None
    cloud_best = None               # (n_points, serialized) — keep the biggest
    dropped: dict[str, int] = {}

    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if t0 is None:
            t0 = stamp_ns
        t_rel = (stamp_ns - t0) / 1e9

        type_name = type_map.get(topic)
        if type_name is None:
            continue
        short = short_name(topic, args.robot_id)

        # --- camera frames -> jpeg on disk ---------------------------------
        if type_name == "sensor_msgs/msg/Image":
            have = images.setdefault(topic, [])
            # Spread the kept frames across the recording instead of taking the
            # first N, so the sample isn't all from the same half-second.
            if len(have) >= MAX_IMAGE_FRAMES:
                dropped[topic] = dropped.get(topic, 0) + 1
                continue
            msg = deserialize_message(raw, get_message(type_name))
            name = f"{short.replace('/', '_')}_{len(have):02d}.jpg"
            try:
                image_to_jpeg(msg, frames_dir / name)
            except Exception as exc:
                print(f"  warning: {topic}: {exc}", file=sys.stderr)
                continue
            have.append(f"frames/{name}")
            continue

        msg = deserialize_message(raw, get_message(type_name))

        # --- point cloud: keep only the largest, decimated ------------------
        if type_name == "sensor_msgs/msg/PointCloud2":
            n_before = msg.width * msg.height
            if cloud_best is None or n_before > cloud_best[0]:
                kept = decimate_cloud(msg, args.max_cloud_points)
                cloud_best = (n_before, {
                    "type": type_name.replace("/msg/", "/"),
                    "messages": [{"t": t_rel, "msg": extract_values(msg)}],
                    "note": f"largest of {counts[topic]} clouds, {n_before} pts decimated to {kept}",
                })
            continue

        entry = topics.setdefault(topic, {
            "type": type_name.replace("/msg/", "/"), "messages": [],
        })
        cap = CAPS.get(short, 50)
        if len(entry["messages"]) >= cap:
            # For the grids, "first + last" is more useful than "first two":
            # the last one is the fully-explored map.
            if short in ("map", "global_costmap/costmap"):
                entry["messages"][-1] = {"t": t_rel, "msg": extract_values(msg)}
            dropped[topic] = dropped.get(topic, 0) + 1
            continue
        entry["messages"].append({"t": t_rel, "msg": extract_values(msg)})

    if cloud_best is not None:
        topics[f"/{args.robot_id}/cloud_map"] = cloud_best[1]

    duration = 0.0
    for entry in topics.values():
        for m in entry["messages"]:
            duration = max(duration, m["t"])

    fixture = {
        "meta": {
            "source_bag": str(args.bag),
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "robot_id": args.robot_id,
            "duration_s": round(duration, 3),
            "generator": "scripts/bag_to_fixture.py",
        },
        "topics": topics,
        "images": images,
    }

    args.out.mkdir(parents=True, exist_ok=True)
    out_json = args.out / "fixture.json"
    out_json.write_text(json.dumps(fixture, separators=(",", ":")))

    # --- summary ---------------------------------------------------------
    total_mb = sum(p.stat().st_size for p in args.out.rglob("*") if p.is_file()) / 1e6
    print(f"wrote {out_json}  ({out_json.stat().st_size / 1e6:.2f} MB)")
    print(f"      {frames_dir}  ({sum(len(v) for v in images.values())} frames)")
    print()
    print(f"{'topic':<42} {'in bag':>7} {'kept':>6}")
    for topic in sorted(set(counts) | set(topics)):
        kept = len(topics.get(topic, {}).get("messages", [])) or len(images.get(topic, []))
        print(f"{topic:<42} {counts.get(topic, 0):>7} {kept:>6}")

    empty = [t for t in type_map if counts.get(t, 0) == 0]
    if empty:
        print("\nWARNING: recorded but EMPTY (nothing was published during the run):")
        for t in empty:
            print(f"  - {t}")
        print("  The panels fed by these topics can't be tested from this fixture.")

    if dropped:
        print("\ndownsampled (capped for fixture size, see CAPS in this file):")
        for t, n in sorted(dropped.items()):
            print(f"  - {t}: dropped {n}")

    print(f"\ntotal fixture size: {total_mb:.2f} MB")
    if total_mb > args.max_mb:
        _fail(
            f"fixture is {total_mb:.2f} MB, over the {args.max_mb} MB limit.\n"
            "  Lower --max-cloud-points, or tighten CAPS in this file. "
            "Don't commit a fixture this large."
        )
    print("\nnext:  node webui/tests/dashboard_test.mjs")


if __name__ == "__main__":
    main()
