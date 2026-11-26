"""A test's success clip: the Tests tab's GIF, cut from a real run that passed.

`scripts/simtest.py run NAME --clip` records Isaac's own Sim view (/sim/view/rgb) through
the scored part of the run and, only if that run passes, encodes it to
webui/test_media/NAME.gif. A failed run leaves no GIF behind, so a "passing run" picture
beside a test is always one that test really passed. `--preview` instead keeps the clip
whatever the verdict, as NAME.preview.gif, captioned as showing what the test sets up.
Commit the file; nothing regenerates it.

Two halves, because the runner is in two environments: `capture` is rclpy, so it runs in
the ROS shell beside the probe (python3 -m simtest.clip capture), and encode() is ffmpeg
on the host. Frames stay in the run's directory, so a clip can be recut without a sim:

    python3 -m simtest.clip encode data/simtests/<run>/runs/0_0/clip webui/test_media/NAME.gif
    python3 -m simtest.clip encode ... --span 0,0.6     # keep only the first 60% of the run

Watching costs what an operator on the Sim view costs: one more render product, so runs
are slower in wall time, and scored in sim time as always.
"""
from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import tempfile
import time
from pathlib import Path

MEDIA_DIR = Path(__file__).resolve().parent.parent / "webui" / "test_media"
CAPTURE_HZ = 3.0        # wall clock, and far more than a clip needs: encode() thins it
BEAT_S = 1.0            # spawn_warehouse.py switches the view off a few seconds after the last beat
STOP_FILE = "stop"
# The first frames after the view switches on are a blur: the renderer accumulates samples,
# and a camera that has only just started has few (crossing_yield, 2026-11-05).
SKIP_FRAMES = 6
# A clip is a timelapse of however long the run took. 8 s is long enough to read what
# happened and short enough to loop; the last frame holds, so a pass is seen, not flashed.
CLIP_S, CLIP_FPS, CLIP_WIDTH, HOLD_S = 8.0, 8, 480, 1.5


def media_path(name: str) -> Path:
    return MEDIA_DIR / f"{name}.gif"


def preview_path(name: str) -> Path:
    """A clip that shows what a test SETS UP, kept whatever the run's verdict
    (`simtest.py run NAME --preview`): for tests that don't pass yet but still
    need a picture of the task. The Tests tab captions it as such, and a real
    passing clip (media_path) wins when both exist."""
    return MEDIA_DIR / f"{name}.preview.gif"


def follow_for(resolved: dict) -> str | None:
    """A test with a `clip_view:`, or a world that aims the camera (`view:`), has already
    framed what it is about. Anything else is a robot crossing the floor, so ride along."""
    world = resolved.get("world") or {}
    return None if resolved.get("clip_view") or world.get("view") else resolved["robots"][0]


def capture_cmd(out_dir: Path, resolved: dict) -> str:
    cmd = f"exec python3 -m simtest.clip capture --out {shlex.quote(str(out_dir))}"
    view = resolved.get("clip_view")
    if view:
        for key in ("eye", "target"):
            # --key=value: a coordinate list that starts with "-" is otherwise read as a flag
            cmd += f" --{key}={','.join(str(c) for c in view[key])}"
    follow = follow_for(resolved)
    return f"{cmd} --follow {shlex.quote(follow)}" if follow else cmd


def stop(out_dir: Path, proc: subprocess.Popen) -> None:
    """A file, not a signal: inside the jazzy distrobox the capture is in another session,
    which a signal only reaches through ros_bash.sh's relay."""
    (out_dir / STOP_FILE).touch()
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()


def pick_frames(frames: list, n: int) -> list:
    """n frames spread evenly over the run, first and last included."""
    if len(frames) <= n:
        return list(frames)
    return [frames[i * (len(frames) - 1) // (n - 1)] for i in range(n)]


def encode(frames_dir: Path, out: Path, span: tuple[float, float] = (0.0, 1.0)) -> Path:
    """`span` keeps that fraction of the run (start, end in 0..1): a recut that drops a
    stretch where nothing happens, e.g. the robot alone on bare floor after the encounter."""
    frames = sorted(Path(frames_dir).glob("*.jpg"))[SKIP_FRAMES:]
    lo, hi = (round(f * len(frames)) for f in span)
    frames = frames[lo:max(hi, lo + 1)]
    if not frames:
        raise ValueError(f"no frames in {frames_dir}")
    picks = pick_frames(frames, int(CLIP_S * CLIP_FPS))
    picks += [picks[-1]] * int(HOLD_S * CLIP_FPS)
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", suffix=".txt") as listing:
        for f in picks:
            listing.write(f"file {shlex.quote(str(f.resolve()))}\nduration {1 / CLIP_FPS}\n")
        listing.flush()
        # Isaac's frame-to-frame render noise is most of a GIF's size: a GIF only saves
        # the pixels that didn't change, and with noise none are unchanged. Denoising
        # first took pick_block from 4.4 MB to 1.2 MB (2026-11-05); no dither for the same
        # reason, at the price of faint banding on the floor.
        graph = (f"hqdn3d=4:3:12:12,fps={CLIP_FPS},scale={CLIP_WIDTH}:-2:flags=lanczos,split[a][b];"
                 "[a]palettegen=max_colors=128:stats_mode=diff[p];"
                 "[b][p]paletteuse=dither=none:diff_mode=rectangle")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0",
                        "-i", listing.name, "-vf", graph, "-loop", "0", str(out)], check=True)
    return out


def capture(out_dir: Path, follow: str | None, eye=None, target=None) -> None:
    import numpy as np
    import rclpy
    from PIL import Image as PilImage
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("simtest_clip")
    cmd = node.create_publisher(String, "/sim/view_cmd", 10)
    state = {"frames": 0, "next": 0.0}

    def on_image(msg):
        now = time.monotonic()
        if now < state["next"]:
            return
        state["next"] = now + 1.0 / CAPTURE_HZ
        channels = msg.step // msg.width
        rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        rgb = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)[:, :, :3]
        PilImage.fromarray(rgb).save(out_dir / f"{state['frames']:06d}.jpg", quality=92)
        state["frames"] += 1

    # sensor_data, always: a RELIABLE image subscriber kills Isaac's image writers.
    node.create_subscription(Image, "/sim/view/rgb", on_image, qos_profile_sensor_data)
    beat = {"on": True, **({"follow": follow} if follow else {})}
    if eye and target:   # a test's clip_view: a fixed camera, resent every beat
        beat.update(eye=eye, target=target)
    last_beat = 0.0
    while not (out_dir / STOP_FILE).exists():
        if time.monotonic() - last_beat >= BEAT_S:
            cmd.publish(String(data=json.dumps(beat)))
            last_beat = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f"[clip] {state['frames']} frames", flush=True)
    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--out", type=Path, required=True)
    c.add_argument("--follow")
    xyz = lambda s: [float(v) for v in s.split(",")]  # noqa: E731
    c.add_argument("--eye", type=xyz)
    c.add_argument("--target", type=xyz)
    e = sub.add_parser("encode")
    e.add_argument("frames", type=Path)
    e.add_argument("out", type=Path)
    e.add_argument("--span", type=lambda s: tuple(float(v) for v in s.split(",")), default=(0.0, 1.0),
                   metavar="START,END", help="keep this fraction of the run, e.g. 0,0.6")
    a = ap.parse_args()
    if a.cmd == "capture":
        capture(a.out, a.follow, a.eye, a.target)
    else:
        print(encode(a.frames, a.out, a.span))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
