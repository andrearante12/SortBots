"""Auction + lease correctness over the in-memory fleet (no radio model)."""
import math

from commsim.core.loopback import LoopbackNet
from commsim.core.protocol import Allocator, Params, CLAIMED, OPEN, DONE

GW = 0


def fleet(positions, **pkw):
    net = LoopbackNet(Params(**pkw))
    net.add(GW, gateway=True)
    for rid, pos in positions.items():
        net.add(rid, pos)
    return net


def assert_single_owner_throughout(net, task_id, seconds):
    end = net.t + seconds
    while net.t < end:
        net.step()
        assert len(net.self_owners(task_id)) <= 1, (net.t, net.self_owners(task_id))


def test_lowest_cost_wins():
    net = fleet({1: (10, 0), 2: (2, 0), 3: (6, 0)})
    net.run(1.0)
    net.announce(GW, 7, (0, 0), (20, 20))
    net.run(3.0)
    assert net.self_owners(7) == [2]
    # every robot (and the gateway) agrees on the owner
    assert {r.alloc.tasks[7].owner for r in net.robots.values()} == {2}


def test_tie_goes_to_lowest_robot_id():
    net = fleet({3: (4, 0), 1: (-4, 0), 2: (0, 4)})
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (9, 9))
    net.run(3.0)
    assert net.self_owners(1) == [1]


def test_gateway_never_bids():
    net = fleet({1: (100, 0)}, bid_radius_m=1000)
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(3.0)
    assert net.self_owners(1) == [1]
    assert GW not in net.robots[1].alloc.tasks[1].bids


def test_no_bidder_in_radius_rebids_with_wider_radius():
    net = fleet({1: (40, 0), 2: (50, 0)}, bid_radius_m=10, rebid_radius_growth=2.0)
    net.run(1.0)
    net.announce(GW, 3, (0, 0), (1, 1))
    net.run(10.0)
    assert net.events("no_winner")
    assert net.self_owners(3) == [1]


def test_lease_renewed_by_status_never_expires():
    net = fleet({1: (1, 0), 2: (5, 0)})
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(60.0)
    assert net.self_owners(1) == [1]
    assert not net.events("lease_expired")
    assert not net.events("robot_silent")


def test_dead_owner_reclaimed_only_after_lease_plus_margin():
    p = dict(task_lease_s=10.0, reclaim_margin_s=2.0)
    net = fleet({1: (1, 0), 2: (5, 0), 3: (8, 0)}, **p)
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(3.0)
    assert net.self_owners(1) == [1]
    died = net.t
    net.robots[1].alive = False
    net.run(30.0)
    expired = [e for e in net.events("lease_expired") if e["task_id"] == 1]
    # Peers time the lease from the last Status they heard, up to one status
    # period (+ delivery delay) before the owner actually died.
    assert expired and min(e["t"] for e in expired) - died >= 12.0 - 1 / 3.0 - 0.05
    assert net.self_owners(1) == [2]
    assert {e["peer"] for e in net.events("robot_silent")} == {1}


def test_isolated_owner_stops_before_anyone_reclaims():
    net = fleet({1: (1, 0), 2: (5, 0), 3: (8, 0)})
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(3.0)
    net.isolate(1)
    assert_single_owner_throughout(net, 1, 30.0)
    stop = [e for e in net.events("stop_task") if e["robot_id"] == 1]
    exp = net.events("lease_expired")
    assert stop and exp and stop[0]["t"] < min(e["t"] for e in exp)
    assert net.self_owners(1) == [2]
    assert [e for e in net.events("hold_safe") if e["robot_id"] == 1]


def test_isolated_robot_stops_bidding():
    a = Allocator(1, Params(status_timeout_s=3.0), cost_fn=lambda pk: (1.0, 1.0))
    a.on_status(2, [], now=0.0)
    a.on_announce(0, 4, (0, 0), (1, 1), now=0.0)
    a.tick(1.1)  # bid from robot 1 only... but suppose the window found no winner
    a.tasks[4].state, a.tasks[4].rebid_at = OPEN, 5.0
    a.drain()
    a.tick(5.0)  # 5 s without hearing anyone: isolated, rebid round opens
    out, ev = a.drain()
    assert a.isolated and any(e["event"] == "isolated" for e in ev)
    assert not [m for m in out if m["type"] == "Bid"]


def test_partition_heal_converges_by_tie_break():
    # Owner in group A keeps hearing A, so it does NOT stop; group B reclaims
    # after lease + margin. Spec tolerates this; heal must resolve it.
    net = fleet({1: (1, 0), 2: (5, 0), 3: (2, 0), 4: (9, 0)})
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(3.0)
    assert net.self_owners(1) == [1]
    net.partition({GW, 1, 2}, {3, 4})
    net.run(20.0)
    assert sorted(net.self_owners(1)) == [1, 3]  # duplicate during partition (see docs)
    net.heal()
    net.run(2.0)
    owners = net.self_owners(1)
    assert len(owners) == 1
    assert {r.alloc.tasks[1].owner for r in net.robots.values()} == set(owners)
    assert net.events("task_yielded")


def test_clock_offset_and_rate_skew_do_not_expire_leases():
    net = LoopbackNet(Params())
    net.add(GW, gateway=True)
    net.add(1, (1, 0), clock_offset_s=1e6, clock_rate=1.0005)
    net.add(2, (5, 0), clock_offset_s=-3600.0, clock_rate=0.9995)
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(120.0)
    assert net.self_owners(1) == [1]
    assert not net.events("lease_expired") and not net.events("robot_silent")


def _outage(status_timeout_s):
    net = fleet({1: (1, 0), 2: (5, 0)}, status_timeout_s=status_timeout_s)
    net.run(2.0)
    net.blocked = {frozenset((1, 2))}  # stand-in for a 2 s reroute window
    net.run(2.0)
    net.heal()
    net.run(3.0)
    return net.events("robot_silent")


def test_timer_misordering_causes_false_silent():
    assert _outage(status_timeout_s=3.0) == []       # timeout > reroute: correct
    assert _outage(status_timeout_s=1.0)             # negative control: fires


def test_release_reauctions_to_another_robot():
    net = fleet({1: (1, 0), 2: (5, 0)})
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(3.0)
    net.robots[1].alloc.p = Params(max_owned_tasks=0)
    net.robots[1].alloc.release(1, net.t)
    net._flush(1)
    net.run(3.0)
    assert net.self_owners(1) == [2]


def test_complete_marks_done_everywhere():
    net = fleet({1: (1, 0), 2: (5, 0)})
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(3.0)
    net.robots[1].alloc.complete(1, net.t)
    net._flush(1)
    net.run(1.0)
    assert {r.alloc.tasks[1].state for r in net.robots.values()} == {DONE}


def test_early_claim_does_not_beat_cheaper_bid_still_in_window():
    p = Params()
    a = Allocator(1, p, cost_fn=lambda pk: (2.0, 1.0))
    a.on_announce(0, 9, (0, 0), (1, 1), now=0.0)
    a.drain()
    a.on_claim(2, 9, cost=5.0, now=0.3)  # peer closed early, never saw our bid
    a.tick(1.01)
    out, _ = a.drain()
    assert any(m["type"] == "Claim" for m in out)
    assert a.tasks[9].owner == 1 and a.tasks[9].state == CLAIMED
    b = Allocator(2, p)
    b.on_claim(2, 9, 5.0, 0.0)  # b's view of its own claim
    b.tasks[9].owner = 2
    b.on_claim(1, 9, 2.0, 1.1)
    assert b.tasks[9].owner == 1


def _far_robot_run(beacon_hz):
    """Robot 3 never receives robot 1's Status (neighbor-only status, robot 3
    out of robot 1's cells) — only the lease beacon can renew 1's lease at 3."""
    net = fleet({1: (1, 0), 2: (8, 0), 3: (40, 0)}, lease_beacon_hz=beacon_hz)
    net.deliver_filter = lambda src, dst, m: not (m["type"] == "Status" and src == 1 and dst == 3)
    net.run(1.0)
    net.announce(GW, 1, (0, 0), (1, 1))
    net.run(60.0)
    return net


def test_neighbor_only_status_without_beacon_expires_leases():
    net = _far_robot_run(0.0)  # spec as originally written
    assert [e for e in net.events("lease_expired") if e["robot_id"] == 3]


def test_lease_beacon_renews_leases_beyond_neighbors():
    net = _far_robot_run(1.0)
    assert not net.events("lease_expired")
    assert net.self_owners(1) == [1]


def test_robot_never_owns_more_than_max_owned_tasks():
    # Robot 1 is cheapest for BOTH tasks, announced together. It may claim one;
    # the other must go to robot 2 instead of being held and never worked.
    net = fleet({1: (0, 0), 2: (10, 0)})
    net.run(1.0)
    net.announce(GW, 1, (1, 0), (5, 5))
    net.announce(GW, 2, (2, 0), (5, 5))
    for _ in range(int(5 / net.tick_s)):
        net.step()
        assert all(len(r.alloc.owned()) <= 1 for r in net.robots.values())
    owners = {tid: net.self_owners(tid) for tid in (1, 2)}
    assert sorted(o[0] for o in owners.values() if o) == [1, 2], owners


def test_winner_at_capacity_declines_and_releases():
    # Safety net behind bid withdrawal: ownership can also arrive another way
    # (tie-break on a peer's Claim/Status) after we bid. Then we must decline.
    a = Allocator(1, Params(), cost_fn=lambda pk: (1.0, 1.0))
    a.on_announce(0, 2, (0, 0), (1, 1), now=0.0)   # we bid on task 2
    a.on_status(3, [(1, 9.0)], now=0.2)            # learn task 1, owned by 3...
    a.tasks[1].owner, a.tasks[1].owner_cost = 1, 0.5  # ...then it resolves to us
    a.drain()
    a.tick(1.01)                                   # task 2's window: we "win"
    out, ev = a.drain()
    assert not any(m["type"] == "Claim" and m["task_id"] == 2 for m in out)
    assert any(m["type"] == "TaskEvent" and m["task_id"] == 2 and m["kind"] == 1 for m in out)
    assert any(e["event"] == "declined" for e in ev)


def test_claiming_withdraws_other_open_bids():
    a = Allocator(1, Params(), cost_fn=lambda pk: (1.0, 1.0))
    a.on_announce(0, 1, (0, 0), (1, 1), now=0.0)
    a.on_announce(0, 2, (0, 0), (1, 1), now=0.5)
    a.drain()
    a.tick(1.01)  # window for task 1 closes: we win, now at capacity
    out, _ = a.drain()
    assert any(m["type"] == "Claim" and m["task_id"] == 1 for m in out)
    assert any(m["type"] == "TaskEvent" and m["task_id"] == 2 and m["kind"] == 1 for m in out)
    assert 1 not in a.tasks[2].bids
