# Saved map library

Named snapshots of explored environments. A library map lets a demo or test run
start from an already-explored warehouse — Nav2 plans on it and you can
click-to-move robots immediately — instead of re-exploring for twenty minutes.

Each entry is one directory:

```
maps/<name>/
  map.json      # manifest — schema documented in scripts/maps_lib.py
  grid.pgm      # occupancy grid (map_saver trinary output)
  grid.yaml     # its metadata; `image:` stays RELATIVE so the dir relocates
  map.db        # the primary robot's RTAB-Map pose graph  (git-lfs)
  map_<rid>.db  # additional robots from a fleet save       (git-lfs)
```

Names are `^[a-z0-9][a-z0-9_-]{0,63}$` — a name is simultaneously a directory
name, an `<option value>` in the dashboard, and a shell word, so the character
class is kept narrow enough that none of the three needs quoting rules.

## Using one

```bash
scripts/maps.sh list                                    # what's saved
scripts/maps.sh show warehouse_full                     # one entry in detail
scripts/sim_ctl.sh start library_localize map=$PWD/maps/warehouse_full/map.db
scripts/sim_ctl.sh start library_resume   map=$PWD/maps/warehouse_full/map.db
```

or pick the map from the dropdown on either scenario's card in the dashboard's
**Scenarios** tab.

`library_localize` is read-only (`Mem/IncrementalMemory=false`); `library_resume`
keeps mapping. **Neither can dirty the entry**: `run_demo.sh` copies a library
`.db` to `~/.ros/sortbots_<robot_id>.db` and runs on the copy, because RTAB-Map
opens its sqlite file read-write even under `--localize`. Keeping the result of
a resume run therefore takes an explicit `maps.sh save`.

## Saving one

`save` is idempotent and stack-aware — run the same command whenever:

```bash
scripts/maps.sh save warehouse_full --title "NVIDIA warehouse — full coverage"
```

The grid can only be captured while the stack is **up** (it comes off the live
fused `/map`); a plain copy of the sqlite pose graph is only safe while the
stack is **down**, or through RTAB-Map's own backup service, which closes the
file first. So a mid-run save may land as `db_state: "pending"` (grid in, pose
graph not yet) and a second run after teardown promotes it to `"complete"`.
One gesture does both:

```bash
scripts/sim_ctl.sh stop --save-map warehouse_full
```

Or use **Save to library** in the dashboard's explore bar during a run.

## git-lfs

`.db` files are tracked through git-lfs (see the root `.gitattributes`). Once
per machine:

```bash
sudo apt-get install git-lfs
git lfs install
```

Cloning **without** it leaves each `.db` as a ~130-byte pointer file that looks
present. `maps.sh list` flags those as `pointer`, the dashboard disables them,
and `run_demo.sh` refuses to copy one — in every case the fix is `git lfs pull`.

Entries are VACUUMed on save. Each one is still 100+ MB and replacing a map
writes a whole new LFS object rather than a delta, so prefer a new name over
re-saving over an entry you still want.

## Removing one

```bash
scripts/maps.sh rm warehouse_full          # prompts; --force skips
```

Then commit the deletion. The LFS object stays in history — that is the cost
of tracking them, and it is why `maps.sh save` VACUUMs by default.
