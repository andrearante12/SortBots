# Fleet comms (`commsim/`)

The fleet has no central coordinator, so ownership and liveness are decided over the
radio. commsim tested that design to failure before any radios were bought. The
design is a self-healing Wi-Fi mesh (802.11s + batman-adv), Zenoh, and the auction +
lease protocol. As first written, it failed at every fleet size. After four fixes it
supports about 20 robots in simulation and passes 110 of 120 failure scenarios, and
the same `core/protocol.py` runs on the robots.

| Fix (now the default) | Problem it solved |
|---|---|
| leases renewed by a fleet-wide beacon | robots reclaimed tasks still being carried |
| only the lower robot ID dials | two TCP sessions per pair wasted a third to a half of the airtime |
| status timeout 6 s, Zenoh lease 4 s | one broken link made healthy robots look silent |
| Status subscriber queue depth 32 | 15% of Status messages lost on a clean link |

There are two cross-checked tools. Median latency agrees within 3–10%.

- **`emu/`** runs the real stack in network namespaces over `mac80211_hwsim`, inside
  an Ubuntu 24.04 VM (`commsim/vm/provision_vm.sh`, created with Multipass).
- **`des/`** is a discrete-event model, used for scale and failure sweeps on the host.

All radio numbers are **placeholders** until real hardware is measured. Run
everything with `/usr/bin/python3` from the repo root.

```bash
/usr/bin/python3 -m commsim.des.run --variants V3 V4 --n 10 15 20 --seeds 1 2 3 --until-fail --out commsim/results/mine
/usr/bin/python3 -m commsim.des.scenarios --variants V4 --n 10 --seeds 1 --out commsim/results/scen
/usr/bin/python3 -m commsim.des.crosscheck [n10]          # model vs emulation (commsim/measurements/)
commsim/scripts/fix_sweep.sh                               # full before/after comparison (~1 h)
sudo python3 -m commsim.emu.mesh up --n 3 --line           # in the VM
```

Configs: `configs/base.yaml` (the recommended design), `hw_lab.yaml` (2 robots +
gateway), `spec_original.yaml` (as first written).

**Real routers** (OpenWrt, one per robot):

1. Build images with the mesh packages: `python3 -m commsim.hw.firmware --out DIR`.
2. Flash each router.
3. Generate per-node bundles. Bundles hold the mesh key, so keep them in
   `commsim/results/`:
   ```bash
   python3 -m commsim.hw.deploy commsim/configs/base.yaml commsim/configs/hw_lab.yaml \
       --mesh-key-file KEY --out commsim/results/hw
   ```
4. `scp` and run `router-setup.sh` on each router.
5. Run `sudo ./apply-host.sh` on each host.
6. Check: `batctl n`, ping across the mesh, `commsim/scripts/check_isolation.sh`.
7. `scripts/jetson.sh bridge start BUNDLE` carries only `/fleet/*` over the mesh,
   via zenoh-bridge-ros2dds.
