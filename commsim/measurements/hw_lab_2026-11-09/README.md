# First hardware measurements — 2 routers, 2026-11-09

The first time the design ran on real routers. Two GL.iNet GL-MT3000 (Beryl
AX, MT7981, mt76) flashed with the `commsim.hw.firmware` image (OpenWrt
25.12.5 + wpad-mesh + batman-adv), configured by `router-setup.sh` from
`commsim.hw.deploy`. Numbers feed [`../../configs/hw_measured.yaml`](../../configs/hw_measured.yaml).

**What this does and does not cover.** Two routers only (no Flint 2 yet): one
radio hop, no relaying, no rerouting, no host-to-host traffic. Routers ~1 m
apart on a desk in a campus building. The far end of every probe is the
robot_1 **router** (no Jetson connected yet).

## Setup

| | robot_1 router | robot_2 router | laptop (gateway stand-in) |
|---|---|---|---|
| Address | 10.42.0.253 | 10.42.0.252 | 10.42.0.1, Windows 11, USB Ethernet into robot_2's LAN port |
| MACs (mesh-g24) | 94:83:c4:98:fe:75 | 94:83:c4:98:fe:55 | |

Path for every probe and transfer: laptop → Ethernet → robot_2 → **radio** → robot_1.
Mesh key: `~/.config/sortbots/mesh.key` on the laptop (rotated once on
2026-11-09 after an accidental print; the routers have the current one).

## Results

### Mesh link (stats/)
- Both bands: `type mesh point`, `mesh plink: ESTAB`, SAE, `mesh_fwding 0`; batman-adv IV lists the peer as neighbour (last seen 0.02–0.3 s), TQ 226–255.
- Signal −18 to −27 dBm; PHY HE (802.11ax) MCS 9–11 × 2 streams = 229–287 Mb/s on both bands.
- Dual band: batman routes over **5 GHz** (TQ 251 vs 244 on 2.4 GHz).
- The routers are isolated: only mesh-point interfaces, WAN off, no default route, internet unreachable (checked 2026-11-09).

### Channel busy with the fleet idle (`router_stats.sh`, 30 s)
2.4 GHz is shared with campus Wi-Fi (GWguest, GWdevice, eduroam). Scan from
both routers: 35–43 networks; ch1 14–18, ch6 7–13, ch11 12–14; networks
stronger than −70 dBm: ch1 3, ch6 3, ch11 0–1.

| Channel | robot_1 | robot_2 | RTT p50 / p95 / p99 (100 Hz, 30 s) |
|---|---|---|---|
| 2.4 ch1 (kept) | 28% (25–36% across the day) | 27% | 11.9 / 15.2 / 22.0 ms |
| 2.4 ch6 | 47% | 40% | 12.1 / 18.7 / 32.4 ms |
| 2.4 ch11 | 30% | 26% | 12.1 / 16.9 / 26.4 ms |
| 5 ch36 | 4% | 3% | — |

Campus-specific: the deployment site will use its own private network, so
use background 0 (or a site measurement) for deployment predictions.

### Latency and throughput
ICMP from the laptop at 100 Hz, 150 B (`tools/icmp_probe.py`, same CSV as
`hw/probe.py`, so `python -m commsim.hw.probe summary` reads it). Load = HTTP
downloads of a 100 MB file from robot_1's RAM (uhttpd), 4 parallel streams.

| | 2.4 GHz only | Dual band (routed on 5 GHz) |
|---|---|---|
| Idle RTT p50 / p95 / p99 | 11.9 / 13.9 / 21.6 ms | 11.9 / 13.1 / 21.6 ms |
| 1 stream | ~141 Mb/s | — |
| 4 streams | 165 Mb/s | **200 Mb/s** |
| RTT under 4-stream load p50 / p95 / p99 / max | 27.3 / 49.5 / 59.5 / 84 ms | 16.9 / 30.1 / 37.7 / 60 ms |
| Loss | 0 | 0 |
| Channel busy under load | 90% (2.4) | 72% (2.4), 50% (5) |

The probe is synchronous: under load it sends below 100 Hz (each echo waits
for its reply). Windows with sent counts are in the raw CSVs.

### DES comparison (`tools/des_labmatch.py`, `python -m commsim.des.run`)
- Lab match (1 robot + gateway, one hop, SNR 72 dB, V3): one-way control latency p50 ~4 ms, p99 ~7 ms, delivery 1.0, fleet adds ~0.3% channel load at any background 0–0.36.
- Capacity, largest passing fleet (3 seeds, `--until-fail`):

| | Open floor | Warehouse |
|---|---|---|
| V4, background 0 | 22 | 20 |
| V4, measured (g24 0.30, g5 0.04) | 14 | 14 |
| V3, measured | 14 | 12 |
| V3, bg 0.30 all bands (`--bg 0.30`) | 15 | 12 |

With measured background the failure is `busy>50%` on 2.4 GHz: the model
routes most warehouse-distance links over 2.4 GHz (better reach) and every
band carries OGMs/floods, so 5 GHz stays at 5–9%. Not yet checked on
hardware at distance.

## Open questions found
- **802.11b rates still used:** `tx bitrate` reads 1.0 Mb/s between bursts despite the `99-sortbots-rates` hook and the mask set by hand. Probably management frames at the mesh basic rate.
- **12 ms idle RTT floor is not airtime:** back-to-back probes return in ~1.5 ms. Router or USB-adapter scheduling; unexplained.
- **DES PHY is ~4× slower than the hardware:** rate table tops out at HT MCS7 65 Mb/s (`des/radio.py`), hardware runs HE 2×2 at 229–287 Mb/s.
- Not measured: multi-hop, reroute time, signal vs distance and through shelving, host-to-host Status, battery-powered routers.

## Files

| Path | What |
|---|---|
| `stats/g24only_*` | `router_stats.sh` snapshots, 2.4 GHz only, ch1, idle and under 4-stream load |
| `stats/chansweep_ch{1,6,11}_*` | idle snapshots with the mesh moved to each 2.4 GHz channel |
| `stats/dual_*` | dual-band idle and under load |
| `tools/icmp_probe.py` | the Windows (no admin) 100 Hz ICMP probe |
| `tools/des_labmatch.py` | DES at the lab setup; run from the repo root |
| `../../results/hw_lab_2026-11-09/` (gitignored, laptop only) | raw probe CSVs (`stress/`, `chan/`, `dual/`), transfer byte counts, DES sweep outputs (`des_sweep`, `des_open`, `des_wh`, `des_meas`) |

Scan dumps were not kept (they list nearby private networks).

## How it was run (Windows laptop, Git Bash)
- `router_stats.sh` from a Windows checkout has CRLF line endings (`core.autocrlf`); feed it as `tr -d '\r' < commsim/hw/router_stats.sh | ssh root@10.42.0.253 'sh -s 30'`.
- Channel sweep: `uci set wireless.<2g radio>.channel=N; wifi reload` on both routers without `uci commit`, so a reboot restores ch1.
