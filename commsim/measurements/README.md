# Emulation reference measurements

Small JSON results from the real-software emulation (802.11s + batman-adv +
zenohd + ROS 2 stubs in the VM, `commsim/emu/`). They are kept in the repo
because the discrete-event model is calibrated and cross-checked against
them (`python3 -m commsim.des.crosscheck n10` reads `spec_config/`), and
because they are the evidence behind numbers quoted in
[`../docs/sim_log.md`](../docs/sim_log.md). Bigger raw outputs
(captures, per-message CSVs, sweeps) stay in the gitignored `commsim/results/`.

## `spec_config/` — the design as first written (session 1)

Both routers of a pair dial each other, Zenoh lease 2 s, Status over TCP.

| File | What it measured | Log step |
|---|---|---|
| `m1_n10_result.json` | 10 nodes, warehouse layout, **802.11b rates still on** (bug) | 8 |
| `m1_n10_no11b_result.json` | same, 11b off, wmediumd interference **off** (bug) | 8 |
| `m1_n10_interf_result.json` | same, 11b off, interference on — the cross-check reference | 8, 11 |
| `m1_n15_result.json`, `m1_n20_result.json` | the overload cliff on the real stack | 15 |
| `m1_n*_mesh_state.json` | node positions and the SNR matrix fed to wmediumd, so the DES can rerun the identical topology | 11, 15 |
| `reroute_500.json`, `reroute_250.json` | batman-adv reroute time, diamond topology, equal-length detour | 9 |

## `fixed_config/` — after the fixes (session 2)

Lower id dials, Zenoh lease 4 s, status timeout 6 s, queue depth 32.

| File | What it measured | Log step |
|---|---|---|
| `line_n3_tcp.json` | 3-node line: single session per pair, delivery with depth 32 | 19 |
| `warehouse_n10_{zenoh,udp}.json` | 10 nodes, Status over TCP vs UDP | 19 |
| `warehouse_n15_{zenoh,udp}.json` | 15 nodes: both overload; UDP degrades gracefully | 19 |
| `linkloss_{zenoh,udp}_{1,2,3}.json` + `_block.json` | Status gap after blocking a link (block time in `_block.json`) | 18 |
| `reroute_by_direction.jsonl` | per-direction recovery on a longer detour, OGM 1000/500/250 ms | 18 |
| `linkloss_ogm250.jsonl` | worst Status gap, OGM 250 ms, TCP vs UDP | 18 |

Field names: `p50_ms`/`p95_ms`/`p99_ms` = one-way Status latency (namespaces
share one clock, so it is exact); `delivery` = received ÷ (sent × peers)
in newer runs — older files carry only `delivery_lower_bound`, computed from
received messages alone, which overstates delivery (log step 24);
`air_busy_frac` = channel airtime from the `hwsim0` capture; `cap_*` = TCP
segment counts from the gateway capture.
