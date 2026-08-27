#!/usr/bin/env python3
"""Unit tests for scripts/_hw_budget.py — the hardware-budget sizing logic
behind scripts/run_demo.sh's auto --robots/--chase-cam-robots pick.

Pure python, no ROS, no Isaac, no GPU: pick() takes detected VRAM/RAM as
plain arguments rather than calling nvidia-smi/proc itself, so this exercises
the arithmetic against fabricated hardware. Run with system python3 per
CLAUDE.md's testing section:

    /usr/bin/python3 -m pytest tests/hw_budget_test.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from _hw_budget import pick  # noqa: E402


def test_8gb_vram_16gb_ram_picks_a_conservative_default():
    robots, chase_cams, notes = pick(
        max_robots=2, vram_mib=8192, ram_mib=16384, pct=90.0,
    )
    assert 1 <= robots <= 2
    assert 0 <= chase_cams <= robots
    assert any("detected 8192 MiB VRAM" in n for n in notes)
    assert any("picked robots=" in n for n in notes)


def test_tiny_budget_degrades_to_minimum_without_raising():
    robots, chase_cams, notes = pick(
        max_robots=2, vram_mib=1024, ram_mib=2048, pct=90.0,
    )
    assert robots == 1
    assert chase_cams == 0
    assert notes  # a note must explain the degrade
    assert not any(n.startswith("ERROR:") for n in notes)  # would flip the
    # dashboard's PHASE_PATTERNS (webui/session.py) to phase "failed"


def test_large_budget_clamps_at_max_robots():
    robots, _chase_cams, _notes = pick(
        max_robots=2, vram_mib=24 * 1024, ram_mib=64 * 1024, pct=90.0,
    )
    assert robots == 2  # never exceeds the roster size, however much headroom


def test_lower_budget_pct_never_picks_more_than_a_higher_one():
    robots_50, chase_50, _ = pick(max_robots=2, vram_mib=8192, ram_mib=16384, pct=50.0)
    robots_90, chase_90, _ = pick(max_robots=2, vram_mib=8192, ram_mib=16384, pct=90.0)
    assert robots_50 <= robots_90
    assert chase_50 <= chase_90


def test_requested_robots_always_passes_through_unchanged():
    robots, _chase_cams, _notes = pick(
        max_robots=2, vram_mib=1024, ram_mib=2048, pct=90.0, requested_robots=2,
    )
    assert robots == 2  # explicit request wins even over a tiny budget


def test_requested_chase_cams_always_passes_through_unchanged():
    _robots, chase_cams, _notes = pick(
        max_robots=2, vram_mib=24 * 1024, ram_mib=64 * 1024, pct=90.0,
        requested_chase_cams=0,
    )
    assert chase_cams == 0


def test_undetectable_hardware_falls_back_to_one_robot():
    robots, chase_cams, notes = pick(max_robots=2, vram_mib=None, ram_mib=None, pct=90.0)
    assert robots == 1
    assert chase_cams == 1  # matches robots — "every spawned robot" default
    assert any("could not detect" in n for n in notes)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
