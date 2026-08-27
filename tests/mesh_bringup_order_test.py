#!/usr/bin/env python3
"""Regression test for scripts/run_demo.sh's mesh tap bring-up ordering.

Pure text/regex over the shell script: no ROS, no Isaac, no GPU, no sudo, no
live ns-3 mesh. Runs in milliseconds — the point, per CLAUDE.md's testing
section ("no ROS").

Bug this pins (diagnosed live, 2026-08-27): run_demo.sh's --mesh block used
to align each tap's MAC to its ns-3 mesh MAC (`ip link set <tap> down`,
`address <mac>`, `up`) *before* moving the tap into its robot's network
namespace via mesh_tap_to_netns.sh. That stacks two live-FD state
transitions back to back on the tap ns-3's TapBridge already has open — a
combination scripts/test_mesh_phase1.sh's proven-safe sequence never
exercises (it moves to netns first, then aligns the MAC *inside* the netns,
as a single down/up cycle). The bad order left both taps NO-CARRIER and the
mesh fully partitioned, even though Isaac Sim and the ROS 2 stacks ran fine
on either side of it — nothing else in the test suite touches run_demo.sh
directly (test_mesh_phase1.sh/phase2.sh reimplement the bring-up inline
rather than calling into it), so this was the only path that could regress
silently.

Expected outcome: the `mesh_tap_to_netns.sh` invocation appears before the
MAC-alignment block, and that block operates via `ip netns exec <ns> ip
link set <tap> ...` (i.e. inside the tap's final netns), not a bare
`ip link set <tap> ...` against the tap while it's still in the root ns.

    python3 -m pytest tests/mesh_bringup_order_test.py
    python3 tests/mesh_bringup_order_test.py            # same, without pytest
"""
from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RUN_DEMO = REPO_ROOT / "scripts" / "run_demo.sh"


def _mesh_block() -> str:
    text = RUN_DEMO.read_text()
    start = text.index("if [[ $MESH -eq 1 ]]; then")
    # First `\nfi` after start closes this if-block (script has no nested
    # `fi` inside it before this point).
    end = text.index("\nfi", start)
    return text[start:end]


def test_run_demo_exists():
    assert RUN_DEMO.is_file(), f"expected {RUN_DEMO} to exist"


def test_tap_to_netns_runs_before_mac_alignment():
    block = _mesh_block()
    move_idx = block.index("mesh_tap_to_netns.sh")
    align_idx = block.index('ip link set "$_TAP" down')
    assert move_idx < align_idx, (
        "mesh_tap_to_netns.sh must run BEFORE the tap MAC-alignment "
        "down/address/up cycle — aligning a tap's MAC while it's still in "
        "the root ns and then immediately netns-moving it leaves the tap "
        "NO-CARRIER (diagnosed live, 2026-08-27). See test module docstring."
    )


def test_mac_alignment_runs_inside_the_target_netns():
    block = _mesh_block()
    # Isolate just the MAC-alignment loop body (from its section comment,
    # not from the `ip link set "$_TAP" down` substring — that also matches
    # mid-string inside the longer `ip netns exec ... ip link set "$_TAP"
    # down` line, which would truncate the region before that line starts).
    align_start = block.index("MAC alignment (inside each netns")
    align_region = block[align_start : align_start + 1200]
    for op in ("down", 'address "$_M"', "up"):
        pattern = re.compile(
            r'sudo ip netns exec "\$_NETNS" ip link set "\$_TAP" ' + re.escape(op)
        )
        assert pattern.search(align_region), (
            f"MAC-alignment op {op!r} must run via "
            '`ip netns exec "$_NETNS" ip link set "$_TAP" ...` (inside the '
            "tap's already-moved netns), not against the root-ns tap. "
            f"align_region={align_region!r}"
        )


if __name__ == "__main__":
    test_run_demo_exists()
    test_tap_to_netns_runs_before_mac_alignment()
    test_mac_alignment_runs_inside_the_target_netns()
    print("all mesh_bringup_order_test checks passed")
