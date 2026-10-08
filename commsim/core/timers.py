"""Timer-ordering check: refuse configs the measurements say will misfire.

Required (sim_log steps 9 and 14):
  reroute  <  zenoh transport lease  <  status timeout  <  task lease
and, because Status rides TCP, the status timeout must also clear the
measured Status gap after a reroute (reroute + TCP retry backoff).
The spec only stated the first chain; with its values (lease 2 s, status 3 s)
a single link loss made healthy robots look silent.
"""
from __future__ import annotations


def worst_status_gap(cfg: dict) -> float:
    """Longest Status outage a single link loss causes, for this config's OGM
    interval and Status transport: measured if we have it, else the measured
    worst reroute plus 1 s for TCP retry/backoff."""
    m = cfg.get("measured", {})
    ogm = int(cfg["batman"]["orig_interval_ms"])
    tr = cfg["protocol"].get("status_transport", "tcp")
    meas = {int(k): v for k, v in m.get("status_gap_worst_s", {}).get(tr, {}).items()}
    if ogm in meas:
        return float(meas[ogm])
    worst = {int(k): v for k, v in m.get("reroute_worst_s", {}).items()}
    base = float(worst.get(ogm, m.get("reroute_s", 0.0)))
    return base + (1.0 if tr == "tcp" else 0.0)


def check(cfg: dict) -> list[str]:
    m = cfg.get("measured", {})
    reroute = float(m.get("reroute_s", 0.0))
    p, z = cfg["protocol"], cfg["zenoh"]
    lease = z["transport_lease_ms"] / 1000.0
    status = float(p["status_timeout_s"])
    task = float(p["task_lease_s"])
    gap = worst_status_gap(cfg)
    ogm = int(cfg["batman"]["orig_interval_ms"])
    worst = {int(k): v for k, v in m.get("reroute_worst_s", {}).items()}
    reroute = float(worst.get(ogm, reroute))  # worst case for THIS interval, not the 1.95 s best case
    bad = []
    if not reroute < lease:
        bad.append(f"zenoh lease {lease:.2f}s must exceed the worst measured reroute "
                   f"{reroute:.2f}s at OGM {ogm} ms (sessions would drop during reroutes)")
    if not lease < status:
        bad.append(f"status timeout {status:.2f}s must exceed zenoh lease {lease:.2f}s")
    if not gap < status:
        bad.append(f"status timeout {status:.2f}s must exceed the worst Status gap after a "
                   f"link loss ({gap:.2f}s at OGM {cfg['batman']['orig_interval_ms']} ms, "
                   f"{p.get('status_transport', 'tcp')} Status)")
    if not status < task:
        bad.append(f"task lease {task:.2f}s must exceed status timeout {status:.2f}s")
    if float(p.get("reclaim_margin_s", 0)) <= 0:
        bad.append("reclaim margin must be > 0")
    return bad
