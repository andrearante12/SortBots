# Reference geometry (not ours, not printed)

From [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) at commit
`5f6d2b876a53a4872e405b991dd925556c9e38a4`. The licence is Apache-2.0.

| file | upstream path | used for |
|---|---|---|
| `Wrist_Roll_Follower_SO101.step` | `STEP/SO101/Follower_Specific/` | the stock fixed jaw we replace; horn interface measured from it; `fitcheck.step` overlay |
| `Moving_Jaw_SO101.step` | `STEP/SO101/Follower_Specific/` | removed-mass estimate only |
| `Wrist_Roll_Pitch_SO101.step` | `STEP/SO101/` | the part that holds motor 5, for clearance checks in FreeCAD |
| `SO101_Assembly.step` | `STEP/SO101/SO101 Assembly.step` | **gitignored, 20 MB**, fetched by `setup_cad.sh`. The whole arm for `build.py --arm` |
| `STS3215_03a.step` | `STEP/SO100/` | motor 5 in the assembly; horn face at z=18.7, shaft at x=12.5 |

Coordinates in `Wrist_Roll_Follower_SO101.step` are as follows:
- The roll axis is at (x=0, y=-0.22).
- The horn-side face is at z=-0.05.
- The fixed jaw extends along +z.

`model.reference_wrist()` shifts it by (0, +0.22, +0.05).

If you bump the commit, also update `SO_ARM_SHA` in `hardware/setup_cad.sh`. Then
run `make cad-test`, because `test_horn_pattern_matches_stock_part` re-measures the
interface from this file.
