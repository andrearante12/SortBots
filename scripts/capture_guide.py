"""Step-by-step guidance for `handeye capture`: which head angle next, where in the image to put
the magnet next, and whether the current view is good enough to press B. Pure python, no
hardware, so tests/capture_guide_test.py runs it offline.

The plan is the spread the fit needs (docs/head_camera.md): a HOME head angle that sees the
workspace with 6 samples, then 4 each at pan +/-20 and tilt +/-10 from home. Each sample has a
target cell of a 3x2 grid over the image, a distance band and a wrist hint, so the samples spread
over the view, over depth and over tool angle instead of clustering where it is comfortable.
HOME is wherever the first sample was taken.

mode="touch" (no marker on the magnet): the box is where to put the PACKAGE, and each sample is
two presses of B: a look at the package's marker with the arm clear, then the magnet touched on
its circle. Every third package goes up on a support, so the samples don't all share one height.
A fixed image grid sends the package out of the arm's reach (2026-11-11), so touch mode boxes
come from a REACH MAP instead: after 3 touches at home, handeye.reach_map fits a rough camera
pose and projects every table spot the magnet can touch (IK, straight down) into the image;
each box goes on the reachable spot farthest from the samples already taken.
"""
from __future__ import annotations

BANDS = {"near": (0.25, 0.40), "far": (0.40, 0.60)}  # metres from the lens
COLS, ROWS = ("left", "middle", "right"), ("top", "bottom")
WRIST = ("wrist level", "wrist tilted up (hold RT + Y)", "wrist tilted down (hold RT + A)")
# (col, row, band) per sample; the wrist hint cycles
HOME_PLAN = [(0, 0, "near"), (2, 0, "far"), (1, 1, "near"), (0, 1, "far"), (2, 1, "near"), (1, 0, "far")]
OTHER_PLAN = [(0, 0, "near"), (2, 1, "far"), (2, 0, "near"), (0, 1, "far")]
OFFSETS = [("pan +20", (20.0, 0.0)), ("pan -20", (-20.0, 0.0)),
           ("tilt +10", (0.0, 10.0)), ("tilt -10", (0.0, -10.0))]
ON_TARGET_DEG = 3.0     # head this close to a stage's angle counts as there
GROUP_DEG = 4.0         # a sample this close to a stage's angle counts toward it
MIN_PX = 25             # a 4x4 marker decodes reliably from about this size
MIN_PX_TOUCH = 18       # the package lies on the table ~55 cm out: 20 mm is ~23 px there
BOX_HALF = (100, 70)    # touch-mode box round a reachable spot, pixels


def stages(home, head_lim=None):
    """[(name, (pan, tilt) or None, plan)]. Angles clamped into the head's range."""
    out = [("home", tuple(home) if home else None, HOME_PLAN)]
    for name, (dp, dt) in OFFSETS:
        if home is None:
            out.append((name, None, OTHER_PLAN))
            continue
        pan, tilt = home[0] + dp, home[1] + dt
        if head_lim:
            pan = min(head_lim["head_pan"][1], max(head_lim["head_pan"][0], pan))
            tilt = min(head_lim["head_tilt"][1], max(head_lim["head_tilt"][0], tilt))
        out.append((name, (pan, tilt), OTHER_PLAN))
    return out


def _near(a, b, tol):
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def cell_of(px, wh):
    return (min(2, int(px[0] / wh[0] * 3)), min(1, int(px[1] / wh[1] * 2)))


def cell_box(cell, wh):
    """(x0, y0, x1, y1) pixels of a grid cell."""
    c, r = cell
    return (int(c * wh[0] / 3), int(r * wh[1] / 2), int((c + 1) * wh[0] / 3), int((r + 1) * wh[1] / 2))


def pick_spot(reach_px, taken_px):
    """The reachable pixel farthest from every sample already taken (the first: the middle)."""
    if not taken_px:
        return min(reach_px, key=lambda p: (p[0] - sum(q[0] for q in reach_px) / len(reach_px)) ** 2
                   + (p[1] - sum(q[1] for q in reach_px) / len(reach_px)) ** 2)
    return max(reach_px, key=lambda p: min((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 for q in taken_px))


def spot_box(px, wh):
    return (max(0, int(px[0] - BOX_HALF[0])), max(0, int(px[1] - BOX_HALF[1])),
            min(wh[0], int(px[0] + BOX_HALF[0])), min(wh[1], int(px[1] + BOX_HALF[1])))


def _in(box, px):
    return box[0] <= px[0] <= box[2] and box[1] <= px[1] <= box[3]


def _touch_place(res, i, name, plan, group, samples, marker, wh, marker_id, reach_px):
    """Where to put the package next, in touch mode."""
    n = len(group)
    # every third package raised, but never the first three: the reach map takes its height
    # from them, and they should share the table's
    raised = len(samples) >= 3 and len(samples) % 3 == 2
    where = "up on a book or box this time" if raised else "on the table"
    lines, box = [], None
    if i == 0 and reach_px:
        box = spot_box(pick_spot(reach_px, [s["px"] for s in group if s.get("px")]), wh)
        lines += [f"Put the PACKAGE so marker {marker_id} sits in the GREEN box, {where}.",
                  "The shaded dots are where the magnet can reach; the box is the reachable spot "
                  "farthest from your earlier samples."]
    elif i == 0:
        lines += [["Aim the head so the arm's reach fills the image (hold RB + right stick): "
                   "this angle becomes HOME.",
                   "Put the PACKAGE anywhere the magnet can touch its circle, on the table."],
                  ["Move the package 15 cm or more from the first spot, still within reach."],
                  ["Third spot: off the line through the first two, so the three make a triangle. "
                   "(Once the arm's joint offsets are measured, a reach map shows here.)"]][min(n, 2)] \
            if n < 3 else [f"Next spot {where}: somewhere you haven't been yet: near the arm, at "
                           "its full reach, left and right of it. Different spots and heights "
                           "are what pin the arm's joint offsets down."]
    else:
        lines += [f"Spread the package over spots the arm can reach and the camera sees: left and "
                  f"right of the arm, near and far; {where}.",
                  "Same reach as at home; only the view has moved."]
    lines.append("Move the arm clear so the camera sees the marker, then press B (LOOK).")
    res.update(title=("Find a HOME head angle" if i == 0 and n == 0 else f"{name}: sample {n + 1} of {len(plan)}"),
               lines=lines, box=box, reach=reach_px if i == 0 else None)
    seen = marker is not None
    big = seen and marker["size_px"] >= MIN_PX_TOUCH
    res["checks"] = [("marker seen", seen)]
    if box:
        res["checks"].append(("in the green box", seen and _in(box, marker["px"])))
    res["checks"].append((f"marker {MIN_PX_TOUCH}+ px" + (f" (now {marker['size_px']:.0f})" if seen else ""), big))
    res["ready"] = all(ok for _, ok in res["checks"])
    return res


def guide(samples, head_now, marker, wh, head_lim=None, mode="magnet", pending=False, marker_id=0,
          reach_px=None):
    """What to do now. `marker` is the magnet marker's {"px", "size_px", "z"} or None.
    -> {"title", "lines", "checks": [(text, ok)], "box": pixels or None, "progress": rows,
        "stage": i, "stages": n, "done": bool, "ready": bool}"""
    home = samples[0]["head_deg"] if samples else None
    plan_stages = stages(home, head_lim)
    progress, cur = [], None
    for i, (name, target, plan) in enumerate(plan_stages):
        group = ([s for s in samples if _near(s["head_deg"], target, GROUP_DEG)] if target
                 else list(samples))
        progress.append({"name": name, "have": len(group), "need": len(plan),
                         "target": [round(v, 1) for v in target] if target else None})
        if cur is None and len(group) < len(plan):
            cur = (i, name, target, plan, group)
    res = {"progress": progress, "stages": len(plan_stages), "checks": [], "box": None,
           "reach": None, "done": cur is None, "ready": False}
    name = cur[1] if cur else "extra"
    target = cur[2] if cur else None
    if pending:  # touch mode, the look is taken: the touch is all that's left for this sample
        res.update(stage=cur[0] + 1 if cur else len(plan_stages),
                   title=f"{name}: touch the circle" if target else "Touch the circle",
                   lines=["Don't move the package or the head.",
                          "Get the magnet FACE FLAT on the CENTRE of the circle: by hand is easiest "
                          "(f in the capture terminal frees the arm, f again holds it).",
                          "Touch lightly, plunger barely pressed. Then press B. (BACK cancels the look.)"],
                   ready=True)
        return res
    if cur is None and mode == "touch":
        # The plan is done but more touches still help: a first full set fitted at ~11 mm because
        # every touch was on one surface and tilted (2026-11-11). Extras alternate raised/table.
        extra = len(samples) - sum(len(p) for _, _, p in plan_stages)
        raised = extra % 2 == 0
        res.update(stage=len(plan_stages), title=f"Extra samples: {extra} taken",
                   lines=[f"Optional, and worth ~10: put the package {'UP ON A BOOK OR BOX' if raised else 'on the table'}, "
                          "somewhere new the arm can reach (any head angle that sees it).",
                          "Arm clear, press B (LOOK); then magnet face FLAT on the circle, press B (TOUCH).",
                          "Done: START (support the arm), then scripts/arm.sh head handeye solve"])
        seen = marker is not None
        res["checks"] = [("marker seen", seen),
                         (f"marker {MIN_PX_TOUCH}+ px" + (f" (now {marker['size_px']:.0f})" if seen else ""),
                          seen and marker["size_px"] >= MIN_PX_TOUCH)]
        res["ready"] = all(ok for _, ok in res["checks"])
        return res
    if cur is None:
        res.update(stage=len(plan_stages), title="All samples taken",
                   lines=["Press START (or x) to save and quit.",
                          "Then run:  scripts/arm.sh head handeye solve"])
        return res
    i, name, target, plan, group = cur
    res["stage"] = i + 1
    if target is not None and not _near(head_now, target, ON_TARGET_DEG):
        dp, dt = target[0] - head_now[0], target[1] - head_now[1]
        axis = (f"pan {'+' if dp > 0 else '-'}" if abs(dp) >= abs(dt) else f"tilt {'+' if dt > 0 else '-'}")
        res.update(title=f"Head to {name}",
                   lines=[f"Move the head to pan {target[0]:+.0f}, tilt {target[1]:+.0f} "
                          f"(now {head_now[0]:+.0f}, {head_now[1]:+.0f}).",
                          f"Hold RB and push the right stick: {axis} next (a/d w/s on the keyboard).",
                          "Keep the arm in view; it doesn't matter where yet."])
        res["checks"] = [(f"pan within {ON_TARGET_DEG:.0f} deg", abs(dp) <= ON_TARGET_DEG),
                         (f"tilt within {ON_TARGET_DEG:.0f} deg", abs(dt) <= ON_TARGET_DEG)]
        return res
    if mode == "touch":
        return _touch_place(res, i, name, plan, group, samples, marker, wh, marker_id, reach_px)
    covered = {cell_of(s["px"], wh) for s in group if s.get("px")}
    todo = [p for p in plan if (p[0], p[1]) not in covered]
    col, row, band = todo[0] if todo else plan[len(group) % len(plan)]
    lo, hi = BANDS[band]
    res["box"] = cell_box((col, row), wh)
    title = "Find a HOME head angle" if target is None else f"{name}: sample {len(group) + 1} of {len(plan)}"
    lines = []
    if target is None:
        lines.append("First, aim the head at the workspace (hold RB + right stick): "
                     "the angle of your first sample becomes HOME.")
    wrist = WRIST[len(samples) % len(WRIST)]
    lines += [f"Drive the magnet so its marker sits in the GREEN box ({COLS[col]} {ROWS[row]}).",
              f"{band.upper()}: {lo * 100:.0f}-{hi * 100:.0f} cm from the camera; {wrist}.",
              "Marker facing the camera. Release RT, keep still, press B."]
    res.update(title=title, lines=lines)
    seen = marker is not None
    in_box = seen and cell_of(marker["px"], wh) == (col, row)
    z_ok = seen and marker.get("z") is not None and lo <= marker["z"] <= hi
    big = seen and marker["size_px"] >= MIN_PX
    res["checks"] = [("marker seen", seen), ("in the green box", in_box),
                     (f"{lo * 100:.0f}-{hi * 100:.0f} cm away" + (f" (now {marker['z'] * 100:.0f})"
                                                                  if seen and marker.get("z") else ""), z_ok),
                     (f"marker {MIN_PX}+ px" + (f" (now {marker['size_px']:.0f})" if seen else ""), big)]
    res["ready"] = seen and in_box and z_ok and big
    return res
