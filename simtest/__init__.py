"""SortBots test layer: worlds, test cases, metrics, and the live-run probe.

    simtest.world     configs/worlds/*.yaml — props + scripted movers
    simtest.spec      configs/tests/*.yaml  — scenario + goals + stop + metrics
    simtest.metrics   the metric registry; every metric is a versioned class
    simtest.events    the event stream metrics consume (live or replayed)
    simtest.probe     rclpy wiring that turns ROS topics into those events
    simtest.report    scoring, aggregation across repeats, markdown

Everything except simtest.probe is pure python + PyYAML (and numpy for grid
metrics), so it is importable from Isaac's venv, from webui/session.py's
ROS-free environment, and from the offline tests. See docs/testing.md.
"""
