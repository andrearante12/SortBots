#!/usr/bin/env python3
"""Static file server for the SortBots dashboard, plus one JSON endpoint.

Pure stdlib except PyYAML (already a dependency of `nodes/task_manager.py`).
Not a ROS node — the dashboard talks to ROS 2 directly over rosbridge's
WebSocket (see index.html); this process only serves index.html/app.js and
translates configs/waypoints.yaml to JSON so the dispatch form doesn't need
a client-side YAML parser.

Run (system ROS 2 Jazzy sourced or not — no rclpy dependency here):

    python3 webui/serve.py --port 8081

Then open http://localhost:8081/. Requires rosbridge_websocket (port 9090)
and web_video_server (port 8080) running separately — see
launch/sortbots_webui.launch.py, which brings up all three together.
"""
from __future__ import annotations

import argparse
import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

WEBUI_DIR = Path(__file__).resolve().parent
WAYPOINTS_CONFIG = WEBUI_DIR.parent / "configs" / "waypoints.yaml"


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEBUI_DIR), **kwargs)

    def end_headers(self):
        # SimpleHTTPRequestHandler sends Last-Modified but no Cache-Control, so
        # browsers apply heuristic freshness (~10% of file age) and happily
        # serve a days-old cached app.js on a "fresh" tab — every stale-page
        # bug in this dashboard's history started that way. no-cache still
        # allows conditional requests (304s), it just forces revalidation.
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def do_GET(self):
        if self.path == "/api/waypoints":
            self._serve_waypoints()
            return
        super().do_GET()

    def _serve_waypoints(self):
        with open(WAYPOINTS_CONFIG) as f:
            stations = yaml.safe_load(f)["stations"]
        body = json.dumps(stations).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass  # SimpleHTTPRequestHandler logs every request to stderr; noisy for a dashboard.


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"SortBots dashboard: http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
