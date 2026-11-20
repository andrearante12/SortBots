"""recon_contamination — did movers get baked into the 3D reconstruction?

map_contamination scores the 2D /map, but the "mapped twice" ghost shows in
the 3D cloud (/fleet/recon_cloud): a person captured where they walked and
again where they stopped. This counts occupied 0.1 m voxels inside the volume
each actor sweeps (its path legs grown by footprint + margin, floor to its
height), with static props' own volumes excluded, on the last sample and the
worst one seen.

It also scores the opposite failure, which masking can cause: static props
that VANISHED from the reconstruction (a SAM mask grabbing a shelf, an
erase rule clearing a wall). A prop counts as kept when at least
`min_prop_voxels` voxels sit within its footprint. Only props the robots could
have seen matter, so `props` defaults to every prop and tests list the ones
their route passes.
"""
from __future__ import annotations

from simtest import geometry
from simtest import world as wl

from . import Metric, MetricError, register


def _near_paths(x, y, actors, margin_m):
    """Per point: the tallest actor whose swept band contains it (0 = none)."""
    import numpy as np

    h = np.zeros(len(x))
    for a in actors:
        r = wl.half_extent(a) + margin_m
        pts = list(a["path"]["waypoints"])
        legs = list(zip(pts[:-1], pts[1:]))
        if a["path"]["mode"] == "loop":
            legs.append((pts[-1], pts[0]))
        inside = np.zeros(len(x), dtype=bool)
        for (ax, ay), (bx, by) in legs:
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = np.clip(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0)
            inside |= np.hypot(x - (ax + t * dx), y - (ay + t * dy)) <= r
        h = np.where(inside, np.maximum(h, wl.top_z(a) + margin_m), h)
    return h


def _in_props(x, y, props, margin_m):
    import numpy as np

    out = np.zeros(len(x), dtype=bool)
    for p in props:
        x0, y0, x1, y1 = geometry.bounds(wl.prop_footprint(p))
        out |= (x >= x0 - margin_m) & (x <= x1 + margin_m) & (y >= y0 - margin_m) & (y <= y1 + margin_m)
    return out


@register
class ReconContamination(Metric):
    name = "recon_contamination"
    version = 1
    streams = {"recon"}
    summary = "Occupied 3D-reconstruction voxels in the volume only actors pass through, and props lost."
    params = {
        "actors": (list, None, "actor ids whose swept volume to check (default: all)"),
        "props": (list, None, "prop ids that must stay in the reconstruction (default: all)"),
        "margin_m": (float, 0.10, "grow each actor's swept band and height by this much"),
        "prop_margin_m": (float, 0.30, "exclude voxels this close to a static prop"),
        "min_height_m": (float, 0.10, "ignore voxels below this (the floor itself)"),
        "min_prop_voxels": (int, 5, "voxels within a prop's footprint for it to count as kept"),
    }
    values = {
        "actor_voxels": "occupied 0.1 m voxels in the actors' swept volume, last sample",
        "actor_m3": "the same, in cubic metres",
        "peak_actor_voxels": "worst actor_voxels over every sample in the run",
        "props_kept_frac": "props with >= min_prop_voxels voxels / props checked, last sample",
        "samples": "reconstruction samples scored",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.actors = ctx.actors(self.p["actors"])
        ids = self.p["props"]
        self.props = [p for p in ctx.props() if ids is None or p["id"] in ids]
        self.last = None
        self.peak = None
        self.samples = 0

    def check_context(self):
        if not self.actors and not self.props:
            raise MetricError("recon_contamination: the world has no actors or props")

    def _score(self, cloud):
        import numpy as np

        c = cloud.centres()
        if len(c) == 0:
            return {"actor": 0, "kept": 0, "voxel": cloud.voxel}
        x, y, z = c[:, 0], c[:, 1], c[:, 2]
        allp = self.ctx.props()
        if self.actors:
            top = _near_paths(x, y, self.actors, self.p["margin_m"])
            hit = (top > 0) & (z >= self.p["min_height_m"]) & (z <= top)
            hit &= ~_in_props(x, y, allp, self.p["prop_margin_m"])
            actor = int(hit.sum())
        else:
            actor = 0
        kept = sum(int(_in_props(x, y, [p], 0.1).sum() >= self.p["min_prop_voxels"])
                   for p in self.props)
        return {"actor": actor, "kept": kept, "voxel": cloud.voxel}

    def on_event(self, ev):
        s = self._score(ev["cloud"])
        self.samples += 1
        self.last = s
        self.peak = s["actor"] if self.peak is None else max(self.peak, s["actor"])

    def result(self):
        s = self.last
        if s is None:
            return {"actor_voxels": None, "actor_m3": None, "peak_actor_voxels": None,
                    "props_kept_frac": None, "samples": 0}
        return {
            "actor_voxels": s["actor"],
            "actor_m3": round(s["actor"] * s["voxel"] ** 3, 4),
            "peak_actor_voxels": self.peak,
            "props_kept_frac": round(s["kept"] / len(self.props), 3) if self.props else None,
            "samples": self.samples,
        }
