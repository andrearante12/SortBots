# Keeping movers out of the map

A person who walks past, stops and walks on used to end up in the map twice:
once where they stopped, once where they stopped again. Two separate causes:

- **Over time, on one robot.** The depth filter only caught things while they
  moved, and only in a band below 1.35 m. Anything standing still went into
  RTAB-Map. The 3D reconstruction (`cloud_map`) is every keyframe's cloud
  stacked together, so once something is captured it stays.
- **Across robots.** `recon_cloud_merge` concatenated each robot's cloud. If
  each robot saw the person at a different moment, both copies stayed, even
  with perfect alignment.

There are two fixes: masking on the camera side, so movers never reach SLAM,
and clearing on the map side, so whatever still gets through fades once
something looks again.

```
rgb ──► dynamic_segment.py ──► /<id>/camera/dynamic_seg (mono8, rgb stamp)
          ▲ prompts                         │
          │ /<id>/camera/dynamic_blobs      ▼
depth ─► dynamic_obstacle_filter.py ── mask |= seg ──► depth_static ─► RTAB-Map (+ offload keyframes)
                                                  └──► dynamic_obstacles ─► Nav2 (short-lived)
RTAB-Map octomap_occupied/empty ─► recon_cloud_relay --source octomap ─► recon_cloud_merge --clear-with-free
```

## Switches

| Setting | Values | What it does |
|---|---|---|
| `dynamic_seg` | `off` (default), `model`, `oracle` | `model` adds the detector and SAM. `oracle` uses Isaac's instance segmentation of the world's actors (sim only). Either one also runs the depth filter on `platform: real`. |
| `seg_policy` | `open` (default), `strict` | `strict` holds each `depth_static` frame until its own mask arrives. Use it when recording a map to keep. |
| `recon_source` | `cloud_map` (default), `octomap` | `octomap` builds the 3D reconstruction from RTAB-Map's ray-traced OctoMap, and lets each robot's empty space erase another robot's stale voxels. |

All three are scenario overrides, `run_demo.sh` / `run_robot.sh` flags
(`--dynamic-seg`, `--seg-policy`, `--recon-source`) and bringup launch
arguments.

## The pieces

**Depth filter** (`nodes/dynamic_obstacle_filter.py`):
- **Motion cue.** Each pixel is warped into the previous frame by odometry
  and compared against it. Only "now closer than the previous ray reached"
  counts, so a moving robot no longer flags shelf edges.
- **Heights.** Real heights above `<id>/odom`, 0.08–2.2 m.
- **`--map-conflict`.** Adds `nodes/map_conflict.py`: depth returns that land
  where this robot has repeatedly seen free space. This catches things that
  arrived and stopped.
- **Prompts.** Both cues are published as blobs on `dynamic_blobs`.
- **`--dynamic-seg`.** Unions the segment mask into the filter's own mask,
  with a dilation plus a depth-aware halo to remove flying pixels.

**Segmenter** (`nodes/dynamic_segment.py`, `nodes/seg_tracks.py`):
- **Detector.** YOLO-World with the vocabulary in
  `configs/dynamic_segment.yaml` baked in.
- **Classes.** `always` classes (person, forklift, …) are masked on sight.
  `if_moving` classes (box, cart, pallet, …) are masked only when a motion or
  conflict blob backs them, or a track already does. Otherwise shelves of
  stock would disappear from the map.
- **Unclassed blobs.** A blob no detection covers is segmented anyway. That
  is what makes this work beyond the class list.
- **SAM.** NanoSAM cuts out each prompt. A mask that mostly falls outside its
  box, or covers most of the image, is rejected.
- **Tracks.** Tracks keep an object masked after it stops:
  - A person the detector still sees stays masked indefinitely.
  - An unclassed object is re-prompted from its predicted box for
    `track_ttl_s`, then becomes map.

**Map side** (`nodes/recon_cloud_relay.py`, `nodes/recon_fuse.py`):
- **OctoMap source.** The OctoMap clears space a later keyframe looks
  through.
- **Erase rule** (`free_space_erase`). Robot B's empty space erases robot A's
  occupied voxel only if:
  - B sees the voxel and at least 4 of its 6 neighbours as empty, so it is
    interior free space;
  - B has nothing occupied within one voxel, which tolerates walls a few cm
    out of alignment.
- **Grid.** The test runs on a 10 cm grid.
- **No "newer wins".** An OctoMap has no per-voxel time.

## Models (`scripts/dynamic_seg_models.sh`)

The weights are not committed, because `models/` is gitignored and an
`.engine` only runs where it was built.

```bash
scripts/dynamic_seg_models.sh export     # anywhere with uv: models/yolo_world.onnx
scripts/dynamic_seg_models.sh sam        # check NanoSAM's two ONNX files (fetched by hand, see output)
scripts/dynamic_seg_models.sh check      # Jetson, inside the container: can it run TensorRT?
scripts/dynamic_seg_models.sh engines    # Jetson: FP16 .engine next to each .onnx
scripts/dynamic_seg_models.sh bench --backend trt   # ms per stage + peak RSS, no ROS
```

Changing `detector.classes` means running `export` and `engines` again. The
node refuses to start if `always` or `if_moving` names a class the export
doesn't have. Licences: YOLO-World is GPL-3.0 / AGPL-3.0 (accepted
2026-11-20), NanoSAM is Apache-2.0.

## Tests

Offline:

```bash
/usr/bin/python3 -m pytest tests/dynamic_obstacle_filter_test.py tests/map_conflict_test.py \
  tests/seg_tracks_test.py tests/dynamic_segment_test.py tests/recon_merge_test.py
```

`tests/_raycast.py` renders depth from boxes, so a moving camera has real
parallax.

Scored: `mapping_walk_stop_walk`, `mapping_cart_push` and
`mapping_static_aisle`, on the `mapping_movers` scenario. Each runs with the
variants `off` / `oracle` / `model` / `model_octomap`:
- **`off`** is today's baseline and is expected to fail the contamination
  criteria.
- **`mapping_static_aisle`** is the false-positive check: every box stack must
  stay.
- **`recon_contamination`** scores the 3D reconstruction, recorded as the
  `recon` stream.

## Live results (RTX 4070 laptop workstation, 2026-11-20)

`mapping_walk_stop_walk` (robot reached its goal, ~90% of the person's floor
observed in every run):

| variant | person in `/map` (occupied_frac) | person in the 3D reconstruction (voxels) | verdict |
|---|---|---|---|
| `off` | 0.043 | 112 | FAIL (baseline) |
| `oracle` | 0 | 0 | PASS |
| `oracle_octomap` | 0 | 0 | PASS |

The filter half works end to end with ground-truth masks. Still open:

- **`crossing_yield` (person, `dynamic_seg` off) failed twice on this branch**:
  1 then 2 contacts, everything else passing (detection recall 0.79, then 0.6).
  It's not known yet whether the motion-cue change causes it. **Run it on
  `main` on the same machine before merging.** The signed test only flags
  things that come closer, so a lateral crosser may be picked up later.
- Not run yet: `off` + octomap (whether the OctoMap clears a ghost on its
  own), `mapping_cart_push`, `mapping_static_aisle`, and every `model`
  variant (needs onnxruntime and the model files on the workstation).

Running these on a second checkout: a git worktree needs
`git submodule update --init third_party/XLeRobot` (it can point at the
main checkout's copy) and `assets/generated/`. Maps have to be real
directories under `maps/`, because the library rejects symlinks that point
outside it.

## Not verified yet

Written and unit-tested offline on a machine with no GPU and no ROS
(2026-11-20). Still open, in order:

1. **TensorRT inside the Jazzy container** (Ubuntu 24.04, py3.12, on JetPack
   6's 22.04 host). `dynamic_seg_models.sh check` reports what the container
   can see. `docker/jetson/Dockerfile` is unchanged until that's known. If it
   can't work, the fallback is a ROS-free inference process on the host.
2. **Orin Nano budget.** Detector + encoder per frame and decoder per prompt,
   with RTAB-Map up. `bench` gives the numbers, and `rate_hz` follows from
   them.
3. **The decoder's I/O.** `sam_feeds` / `sam_postprocess` handle both NanoSAM's
   five-input decoder and SAM's official export, keyed by input and output
   names. They're checked against fakes, not the real files.
4. **Whether RTAB-Map's OctoMap really clears** a walked-through person
   (`recon_source: octomap` on `mapping_walk_stop_walk`). If not, the per-robot
   half needs its own log-odds voxel map from `depth_static`.
5. **The Isaac oracle.** `ROS2CameraHelper` type `instance_segmentation` with
   `enableSemanticLabels`. The labels' JSON shape is handled loosely in
   `oracle_mask`.
6. **Filter cost on the Jetson.** The warp is ~7 ms at 848x480 and a
   map-conflict pass ~44 ms, both measured on a laptop CPU. `mem_probe` will
   say whether the Orin keeps up.
