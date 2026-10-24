# commsim/viz — the fleet mesh, on the operator's map

`demo.py` is the engine behind the operator view's **Comms** overlay
(`webui/operator.html`, the toggle next to Map / 3D cloud). It runs the real
discrete-event model — `commsim/des/sim.py`, with `core/protocol.py` in every
node — in real time, and the page draws what it predicts over the live map:
radio links shaded by link quality, each robot's mesh route to the gateway, and
every message as a dot travelling its hop.

There is no separate page or port. `webui/serve.py` owns one `Hub` and streams
it to the page (`GET /api/commsim/events`, Server-Sent Events; commands come back
on `POST /api/commsim/cmd`). The model only steps while a page has the overlay
on. The standalone `demo.html` and the packet replay were removed on 2026-10-23:
the operator view is the one GUI (CLAUDE.md, "One GUI").

## What goes in, what comes out

| In (real) | From |
|---|---|
| Where each robot stands | the operator view's live poses, re-sent as they drive (`move`) |
| What the radio has to get through | the explored `/map`: every blob of occupied cells becomes one obstacle (`layout` → `commsim/des/layout.py` `from_occupancy`) |
| Where the gateway / AP is | the operator's **gateway pin** (map tool ▲, saved to `data/comms_gateway.json`); the map centre until one is dropped |

| Out (predicted, drawn dashed on the map) | |
|---|---|
| Links | dotted lines, thicker for a better link |
| Routes | green arrows: each robot's next hops to the gateway |
| Messages | dots per hop, coloured by kind; a red X where a hop was lost |
| Cut off | a red dashed halo, "model: cut off", when the gateway would call a robot silent |

The predictions never reach the robot cards or the alarm chip. Those show the
**real** fleet: `nodes/fleet_tasks.py --gateway`'s `/fleet/task_board`. A
what-if in the overlay changes the model only.

## What-ifs

- **Pull a link**: click its two ends (again to restore). Frames queued for it
  die, the mesh notices in ~2–3 s (the measured reroute), and the arrows move.
- **Switch off**: click a robot. The others call it silent after
  `status_timeout_s` (6 s).
- **Send A→B**: click a sender, then a receiver. The feed says how many hops it
  took — through the gateway when the two can't hear each other.
- **Range**: the open-air radio range. The model's placeholder transmit power
  reaches ~210 m in free space, so the slider is what makes obstacles matter.
- **Routing chatter**: shows the mesh beacons, transport ACKs and keepalives.
  They are most of the airtime and none of the story, so they're hidden by default.

The model's own task auction is never driven from here. The real auction runs on
the robots, and two auctions disagreeing on one map would be worse than one.
`serve.py` refuses the `task` and `cancel` ops.

## What is real and what is not

The radio range model, carrier-sense contention, batman-style routing with link
quality, every message's size and timing, leases and silence detection are the
model's, the code the sweeps use. Path loss, rack attenuation and link rates are
placeholders until the lab testbed (`commsim/hw/`) measures them, and every
obstacle on the map costs one rack's attenuation (15 dB at 2.4 GHz), whatever
it is made of.

## Tests

```bash
/usr/bin/python3 -m pytest commsim/tests/overlay_test.py   # from_occupancy, the Hub fed like the page feeds it, serve.py's endpoints
node webui/tests/operator_test.mjs                         # the overlay in the operator view, against the real model ("comms:" checks)
```
