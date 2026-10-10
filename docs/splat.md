# Gaussian-splat reconstruction (optional)

The 3D panel's **splat** mode shows a photoreal Gaussian splat of a saved map.
The voxel and point modes show RTAB-Map's live cloud; the splat is built
**after** a run, from a map saved in `maps/`. It trains on the **workstation's
GPU** (RTX 4070 Laptop, 8 GB), never on the Jetson. Sim and real use one
pipeline; the only difference is where the pose graph comes from.

```
 sim:  maps/<name>/*.db on the workstation ──read in place──┐
 real: Jetson serve.py  GET /api/maps/<name>/db?robot=<rid> ─┤ (tailnet, after the run)
                                                             ▼
 workstation: splat worker  (scripts/splat.sh serve · conda env `splat` · :8082)
   fetching → extracting (rtabmap-export) → building (keyframe set, `map` frame)
            → [waiting_for_gpu while Isaac runs] → training (gsplat) → done
                                                             │ GET /splats/<map>/latest.splat (CORS)
                                                             ▼
 browser (workstation or laptop): dashboard 3D panel → splat
```

Why this works: a saved RTAB-Map database already holds every keyframe's RGB,
depth and intrinsics, plus the poses after loop closure.
`rtabmap-export --images_id --poses_camera --cloud` dumps all of that. The
splat is then anchored into the fleet `map` frame with the same spawn anchors
bringup uses, so it overlays the point cloud exactly.

## Setup (once, on the workstation)

```bash
scripts/install_splat.sh          # conda env `splat`: python 3.10, torch 2.4/cu124, gsplat 1.5.3 (~5 GB)
scripts/install_splat.sh --check  # ok: torch …, gsplat …, <GPU>
```

The env versions are pinned on purpose. They are the newest combination gsplat
publishes prebuilt CUDA wheels for. Anything else makes gsplat JIT-compile its
kernels, which needs `nvcc`, and this machine only has the driver.

On the robot, also set `worker_url` in `configs/splat.yaml` to the
**workstation's** tailnet name (for example `http://<workstation>.ts.net:8082`).
The browser fetches the splat from there. A per-page override also works:
`?splat_worker=http://host:8082`.

## Use

```bash
scripts/splat.sh serve                         # start the worker (detached)

# sim: save the map, then build it
scripts/sim_ctl.sh stop --save-map warehouse --splat
#   …or by hand:
scripts/splat.sh build warehouse --wait

# real: save on the Jetson as usual, then from the workstation
scripts/splat.sh build warehouse --from http://jetson-orin.tail0d28f9.ts.net:8081 --wait

scripts/splat.sh status                        # health + recent jobs
scripts/splat.sh log <job-id>
```

Or use the dashboard. Choose **splat** in the 3D panel's mode dropdown, pick a
saved map, then click **build**. Progress shows next to the button, and the
splat loads automatically when it's done. `?recon=splat` opens the page
straight into that mode.

A build takes about 4 min for the 1,157-keyframe `warehouse` map: about 1 min
to extract and load, then about 3 min to train 15k steps. The result is
1.3M gaussians and 42 MB. Measured 2026-10-10. Renders from held-in keyframe
poses scored 21–29 dB PSNR.

## Where things live

| | |
|---|---|
| `splat/dataset.py` | export → keyframe set (`transforms.json`, OpenCV c2w in `map`). The future live mode appends to this format. |
| `splat/extract.py` | `rtabmap-export` in a ROS-sourced subshell, the only ROS touch in the pipeline |
| `splat/train.py` | gsplat, SH degree 0, RGB L1 + SSIM + inverse-depth loss, snapshots while training |
| `splat/convert.py` | standard 3DGS `.ply` ↔ the viewer's 32-byte `.splat` |
| `splat/worker.py` | HTTP job server; a single FIFO queue, because the GPU can only fit one job |
| `webui/splat_view.js` | WebGL2 renderer, adapted from antimatter15/splat (MIT) |
| `configs/splat.yaml` | worker URL, iterations, gaussian cap, depth range |
| `~/.cache/sortbots/splats/<map>/<job>/` | `model.splat`, `model.ply`, `job.log` (outside the repo: splats are 40–300 MB) |

## Things that bit, and why the code looks the way it does

- **Isaac and the trainer can't share the 8 GB card.** Jobs wait in
  `waiting_for_gpu` while `spawn_warehouse.py` runs, and `force: true` skips
  the wait. The check uses `pgrep -f "[s]pawn_warehouse.py"`. The bracket
  matters: a plain pattern matches any shell whose command line mentions it,
  and reported a phantom Isaac on 2026-10-10.
- **Maps saved on the robot record `scene: nvidia`.** `maps.sh save` defaults
  to that, and the dashboard save doesn't pass `--scene`. The worker therefore
  anchors with `spawn.real` whenever the source dashboard reports platform
  `real`. This follows bringup's own rule: forced, not trusted.
- **The library DB is read in place.** `rtabmap-export` only reads, unlike the
  rtabmap node: the sha256 was unchanged after an export. A remote DB is
  downloaded, checked against the manifest's sha256, and deleted as soon as
  it's exported.
- **Depth loss uses inverse depth, clamped at 0.3 m.** Where nothing has been
  rendered yet, the rendered depth is about 0. Unclamped, that one term was 20×
  the RGB loss.
- **Far rack tops are soft.** Depth beyond 8 m isn't supervised, and few
  keyframes see the tops of the racks. Near content is sharp.
- **Moving actors in sim** (forklifts, people) leave faint floaters. Opacity
  and size pruning in `convert.py` removes most of them.
- **Jetson browser:** splat mode is never the default on software GL, and is
  labelled "slow: no GPU" there. View it from the workstation or a laptop.

## Tests (offline: no GPU, ROS or torch)

```bash
/usr/bin/python3 -m pytest tests/splat_*_test.py   # dataset, convert, worker (+ real serve.py as the robot)
node webui/tests/splat_test.mjs                     # splat mode in headless Chrome against a fake worker
```

## Later: live mode

Not built yet. The seam for it is the keyframe set plus the worker API:
- **sim:** a workstation node on `/<rid>/mapData` appends frames and re-poses
  them on graph updates;
- **real:** DDS inside the Jetson container is localhost-only, so a small
  Jetson-side HTTP relay would serve new keyframes and the worker would poll it;
- **trainer:** a continuous mode that keeps training and publishes snapshots
  for the browser to reload.
