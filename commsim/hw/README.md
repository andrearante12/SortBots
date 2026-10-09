# commsim/hw — moving the design onto real routers

The simulation fixed a configuration (`configs/base.yaml`) and measured why
each value is what it is. This folder puts **that same configuration** on
real OpenWrt routers and Jetsons, then measures what the simulation could
only assume. The measurements go back into the configs.

| File | What it does |
|---|---|
| `deploy.py` | Fleet list → one bundle per node: `router-setup.sh` (OpenWrt), host network, Zenoh env, UDP Status endpoint, chrony |
| `probe.py` | UDP round-trip probe: latency, loss and the longest outage (= reroute time). Standard library only, no ROS |
| `router_stats.sh` | Run on a router over ssh: neighbors, link quality, bitrates, channel busy % |

## How many routers

**One router per mesh node.** Each robot carries its own router and the
gateway laptop has one, so the 2-robot demo needs **3**. A single router
can't test this design: there is no mesh with one member. One router as a
plain access point with the Jetsons as Wi-Fi clients tests Zenoh and the
task protocol, but not 802.11s, batman-adv or rerouting.

| Routers | What you can test |
|---|---|
| 2 | Mesh bring-up, SAE, signal/rate/loss vs distance and through shelving, latency, channel busy time, Zenoh + task protocol between 2 hosts |
| 3 | All of the above, plus **rerouting**: block the direct link, traffic goes through the third router. This is the minimum for the demo and for the reroute measurement |

**2.4 GHz only is fine to start.** `configs/hw_lab.yaml` is the 2.4 GHz-only
lab config. At 3 nodes the channel is ~1% busy (sim_log Results), so capacity
isn't the issue. What you give up is the fallback: if 2.4 GHz is jammed (busy
campus channel, microwave) there is no second band. Buy dual-band routers
anyway and turn 5 GHz on after step 6 works.

Router minimum (sim_log Results): runs stock OpenWrt 23.05 or newer;
802.11s mesh on its radios (MediaTek mt76 or Qualcomm ath9k/ath10k; avoid
Broadcom); dual band for later; ≥ 16 MB flash, ≥ 128 MB RAM; an Ethernet
port for the Jetson; runs from robot power. Antennas: ordinary 2–3 dBi
omnis, mounted vertical and above the robot's metal frame.

## 1. Flash OpenWrt

Stock OpenWrt 23.05 or newer from the device page on openwrt.org. Reset to
defaults. The router answers on `192.168.1.1` over its LAN port.

## 2. Install the mesh packages (once per router)

The default `wpad-basic-*` can't do mesh, and batman-adv isn't installed:

```sh
opkg update
opkg remove wpad-basic-mbedtls          # or wpad-basic-wolfssl, whichever is present
opkg install wpad-mesh-mbedtls kmod-batman-adv batctl-default
```

`opkg update` needs internet on the router's WAN port. Use a phone hotspot
or your own router, **never eduroam**. Without internet, download the same
`.ipk` files for your exact release and target on another machine, then
`scp -O` them over and `opkg install ./*.ipk`. OpenWrt 25.x uses `apk add`
in place of `opkg install`.

## 3. Generate the bundles (laptop, from the repo root)

```bash
/usr/bin/python3 -m commsim.hw.deploy commsim/configs/base.yaml commsim/configs/hw_lab.yaml \
    --mesh-key-file ~/.config/sortbots/mesh.key --new-key --out commsim/results/hw
```

Drop `--new-key` after the first run (every router must share one key).
Output: `commsim/results/hw/{gateway,robot_1,robot_2}/` plus `fleet.json`.
It prints each node's host and router address:

```
gateway    host 10.42.0.1       router 10.42.0.254
robot_1    host 10.42.0.2       router 10.42.0.253
robot_2    host 10.42.0.3       router 10.42.0.252
```

The generator refuses to write:
- a config whose timers the measurements say will misfire (`core/timers.py`);
- an isolation violation: any listener or peer off the fleet subnet, scouting on (`core/fleet.py check`);
- a bundle into a tracked repo path, because the bundle holds the mesh key and the repo is public.

Interface names are placeholders in `base.yaml` `hardware:`. Check
`ip link` on each host and pass `--robot-iface` / `--gateway-iface` if they
differ.

## 4. Configure each router

Label the routers first. Then, from a laptop plugged into that router's LAN port:

```bash
scp -O commsim/results/hw/robot_1/router-setup.sh root@192.168.1.1:/tmp/
ssh root@192.168.1.1 'sh /tmp/router-setup.sh && reboot'
```

The script fails early, before changing anything, if a package is missing
or OpenWrt is too old. It finds the radios by band, so it doesn't depend on
the router model. What it sets, and why:

| Setting | Why |
|---|---|
| 802.11s mesh point per band, `mesh_fwding 0`, SAE with the shared key | 802.11s only provides links; batman-adv does the routing |
| batman-adv IV, OGM 250 ms, hard interfaces at MTU 1532 | Worst-case reroute 3.0 s vs 5.7 s at 500 ms (sim_log step 18). The extra 32 B keep bat0 at 1500 |
| bat0 + LAN port in one bridge, static IP, DHCP off | The Jetson's fleet IP is reachable straight over the mesh. One DHCP server per router would conflict across the shared segment |
| Stock AP interfaces removed, WAN off, NTP off | Nothing on a robot bridges the fleet to another network |
| 802.11b rates off, multicast at 12 Mb/s | 1–11 Mb/s frames were the biggest airtime cost in emulation (sim_log step 8) |

After the reboot the router answers on its fleet address, for example `10.42.0.253`,
and no longer on 192.168.1.1.

## 5. Configure each host (Jetson / laptop)

Copy that node's bundle folder to the host, then:

```bash
sudo ./apply-host.sh        # static mesh IP via netplan; chrony (gateway = time source)
```

The mesh interface gets no default route and no DNS. The host's internet
stays on whatever it already uses, and fleet traffic never leaves the mesh.

## 6. Bring-up checks

```bash
ssh root@10.42.0.253 batctl n          # every other router listed as a neighbor
ssh root@10.42.0.253 'iw dev mesh-g24 station dump | grep -E "Station|plink|signal|tx bitrate"'
                                       # mesh plink: ESTAB; tx bitrate never 1/2/5.5/11 Mb/s
ping -c 5 10.42.0.3                    # host to host across the mesh
commsim/scripts/check_isolation.sh --subnet 10.42.0.0/24 --configs commsim/results/hw
```

When this works on 2.4 GHz, set `fleet.bands: [g24, g5]` in `hw_lab.yaml`.
Then regenerate the bundles and rerun step 4.

## 7. Run the stack

On each host, after building `commsim/ws` there:

```bash
source /opt/ros/jazzy/setup.bash && source commsim/ws/install/setup.bash
(source router.env && ros2 run rmw_zenoh_cpp rmw_zenohd) &
source node.env
python3 commsim/stubs/status_stub.py --robot-id 1 --transport udp --bind 10.42.0.2 \
    --peers 10.42.0.1,10.42.0.3 --duration 120 --out /tmp/status_r1.csv --sent-out /tmp/sent_r1.csv
```

`--bind` and `--peers` come from that node's `status_udp.json`. The router and
the nodes need separate env files: one shared override would make every node
try to listen on the router's mesh port.

JetPack 6 is Ubuntu 22.04, but ROS 2 Jazzy targets 24.04. On the Jetson,
plan on a Jazzy container (with `--network host`) or a source build.
Confirm which one before the demo.

## 8. Calibration: what to measure and what it replaces

Run `probe.py echo --bind <far host>` on one host and
`probe.py ping --bind <near host> --to <far host>` on another.
Run `router_stats.sh` on the routers.

| Measure | How | Replaces |
|---|---|---|
| Signal and bitrate vs distance, open floor | `router_stats.sh` at 2, 5, 10, 20 m | Path-loss exponent and reference loss (`des/radio.py`) |
| Same, through one wire shelf and one loaded rack | Same, with the shelf/rack between routers | Shelf and rack attenuation (placeholder, uncited) |
| Loss and latency vs distance | `probe.py ping --hz 100 --duration 60` at each spot | Loss-vs-SNR curve; latency floor |
| **Reroute time** | 3 routers in a triangle. While `probe.py` runs between robot_1 and robot_2, block their direct link on both ends: `iw dev mesh-g24 station set <peer MAC> plink_action block` (peer MAC from `station dump`). Read "longest outage". Undo with `plink_action open` | `measured.reroute_worst_s` (now emulation only: 3.0 s at 250 ms) |
| Status gap on the real stack | `status_stub.py` on all 3 hosts, same link block | `measured.status_gap_worst_s.udp` |
| Background channel busy | `router_stats.sh 30` with the fleet idle, on the channel you'll use | `background.busy_share` |
| Battery-powered router behavior | Run 30 min on robot power | Not in the model; checks brownouts |

Put the numbers into a new `configs/hw_measured.yaml` (same keys as
`base.yaml` `measured:` and `radio`). Then rerun
`commsim.hw.deploy ... hw_measured.yaml`, and the timer check will judge the
real reroute time. Rerun the model with `--configs .../hw_measured.yaml` to
re-predict capacity. If the real reroute is slower than the 4 s Zenoh lease,
the generator refuses, which is the point.

## What is untested on real hardware

Everything in this folder is checked offline: generated files, shell
syntax, addressing and isolation (`commsim/tests/hw_test.py`). None of it
has run on a router yet. The parts most likely to need a fix on first
contact:
- The 802.11b-rate hotplug hook. Check `tx bitrate` in step 6.
- `mcast_rate` in mesh mode on your driver.
- The host interface names.
