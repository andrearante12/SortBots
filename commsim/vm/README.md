# commsim/vm — the emulation VM

The emulation (`../emu/`) runs the **real** 802.11s, batman-adv, zenohd and
ROS 2 stacks, one network namespace per robot, over simulated radios. That
needs root, kernel modules (`mac80211_hwsim`, `batman-adv`) and apt ROS 2
Jazzy. The host is the Isaac machine (Ubuntu 25.10, no apt ROS), so all of
it lives in an Ubuntu 24.04 VM. Nothing here touches the host's ROS, Isaac
or conda setup.

| File | What it does |
|---|---|
| `provision_vm.sh` | Installs ROS 2 Jazzy, rmw_zenoh, batman-adv tooling, hwsim/wmediumd and a pinned Mininet-WiFi inside the VM |

## 1. Create the VM (host, needs sudo — run these yourself)

Multipass is the shortest path. Sizing is a placeholder until M1 measures
the per-robot cost (`../docs/plan.md`, open question 8). Don't run sweeps
while an Isaac session is up: `scripts/sim_ctl.sh status` exits 4 when none
is.

```bash
sudo snap install multipass
multipass launch 24.04 --name commsim --cpus 12 --memory 10G --disk 60G
multipass mount ~/SortBots commsim:/home/ubuntu/SortBots
multipass shell commsim
```

## 2. Provision (inside the VM)

```bash
cd ~/SortBots && sudo ./commsim/vm/provision_vm.sh
```

## 3. Build the message package (inside the VM)

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/SortBots/commsim/ws && colcon --log-base /tmp/colcon_log build'
```

## 4. Run an emulation

```bash
sudo python3 -m commsim.emu.mesh up --n 3 --line
sudo python3 -m commsim.emu.m1 --n 10 --layout warehouse --out /run/commsim/m1
sudo python3 -m commsim.emu.reroute --trials 6
sudo python3 -m commsim.emu.mesh down
```

`multipass exec` joins its arguments into one remote shell string, so put
multi-step VM commands in a script under the gitignored `commsim/results/`
(mounted in the VM) and run that. The reference results these runs produced
are in [`../measurements/`](../measurements/README.md).

## Notes

- Memory: the VM holds ~1.4 GB while idle. Stop it (`multipass stop commsim`) before an Isaac run on the same laptop.
- Mininet-WiFi is pinned to `master` in `provision_vm.sh`; it passed the M0 smoke test on 2026-10-03. Pin the commit before the next VM is built.
