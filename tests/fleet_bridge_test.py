"""The mesh bridge forwards exactly the topics the fleet nodes use.

commsim/configs/base.yaml lists the topics zenoh-bridge-ros2dds may carry; a
node renaming one would silently fall off the mesh, and a new topic added
there by mistake would put it on the radio. No ROS needed.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import fleet_radio as fr  # noqa: E402
import fleet_tasks as ft  # noqa: E402
from commsim.core import fleet as F  # noqa: E402

CFG = F.load_config(REPO / "commsim/configs/base.yaml", REPO / "commsim/configs/hw_lab.yaml")


def test_bridge_topics_are_the_fleet_nodes_topics():
    assert sorted(CFG["hardware"]["ros_bridge"]["topics"]) == sorted(
        [fr.STATUS_TOPIC, fr.INTENT_TOPIC, ft.TASKS_TOPIC])


def test_gateway_only_topics_stay_off_the_mesh():
    topics = CFG["hardware"]["ros_bridge"]["topics"]
    for local in (ft.REQUEST_TOPIC, ft.CANCEL_TOPIC, ft.BOARD_TOPIC):
        assert local not in topics
