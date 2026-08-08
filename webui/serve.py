#!/usr/bin/env python3
"""Static file server for the SortBots dashboard, plus its JSON endpoints.

Pure stdlib except PyYAML (already a dependency of `nodes/task_manager.py`).
Not a ROS node — the dashboard talks to ROS 2 directly over rosbridge's
WebSocket (see index.html); this process only serves index.html/app.js and
translates configs/*.yaml to JSON so the browser doesn't need a YAML parser.

Run (system ROS 2 Jazzy sourced or not — no rclpy dependency here):

    python3 webui/serve.py --port 8081

Then open http://localhost:8081/. Requires rosbridge_websocket (port 9090)
and web_video_server (port 8080) running separately — see
launch/sortbots_webui.launch.py, which brings up all three together.

CONTROL MODE (--control) additionally exposes the Scenarios tab's API: list
configs/scenarios/*.yaml, start a scenario, stop it, tail its log. That turns
this process into a launcher for `scripts/run_demo.sh`, so it is opt-in and
only ever enabled by scripts/run_console.sh, which runs this server OUTSIDE
the pipeline it starts (see that script and webui/session.py for why the
dashboard has to outlive the sim). Without --control those endpoints answer
503 and the tab degrades to an explanation instead of half-working.

SECURITY: this server binds 0.0.0.0 and is reachable from any tailnet device,
so --control is a remote process launcher. Containment lives in
webui/session.py — scenario names resolve by allowlist, every argv element
comes from a fixed typed table, and Popen is always called with a list.
Tailscale ACLs are the actual perimeter; there is no authentication here.
"""
from __future__ import annotations

import argparse
import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import yaml

import session as session_mod

WEBUI_DIR = Path(__file__).resolve().parent
WAYPOINTS_CONFIG = WEBUI_DIR.parent / "configs" / "waypoints.yaml"
ROBOTS_CONFIG = WEBUI_DIR.parent / "configs" / "robots.yaml"

MAX_BODY_BYTES = 64 * 1024

# Set by main() when --control is given. None means "control endpoints off".
SESSIONS: session_mod.SessionManager | None = None


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

    # -- routing -----------------------------------------------------------
    # Split off the query string before matching: /api/session/log carries
    # ?offset=, and the older exact `self.path == "/api/..."` comparisons
    # silently stopped matching as soon as any endpoint took a parameter.

    def do_GET(self):
        route = urlsplit(self.path).path
        if route == "/api/waypoints":
            self._serve_waypoints()
        elif route == "/api/robots":
            self._serve_robots()
        elif route == "/api/scenarios":
            self._serve_scenarios()
        elif route == "/api/session":
            self._with_control(lambda: self._serve_json(SESSIONS.status()))
        elif route == "/api/session/log":
            self._with_control(self._serve_log)
        else:
            super().do_GET()

    def do_POST(self):
        route = urlsplit(self.path).path
        if route == "/api/session/start":
            self._with_control(self._start_session)
        elif route == "/api/session/stop":
            self._with_control(lambda: self._serve_json(SESSIONS.stop()))
        else:
            self._serve_error(404, f"no such endpoint: {route}")

    # -- config endpoints --------------------------------------------------

    def _serve_waypoints(self):
        with open(WAYPOINTS_CONFIG) as f:
            stations = yaml.safe_load(f)["stations"]
        self._serve_json(stations)

    def _serve_robots(self):
        # configs/robots.yaml is display-only bookkeeping for the dashboard's
        # robot switcher (see that file's header) — if it's missing, degrade
        # to the single-robot default rather than erroring the whole page.
        if ROBOTS_CONFIG.exists():
            with open(ROBOTS_CONFIG) as f:
                data = yaml.safe_load(f)
        else:
            data = {"default": "robot_0", "robots": ["robot_0"]}
        self._serve_json(data)

    def _serve_scenarios(self):
        # Readable without --control on purpose: the tab can then show what
        # the suite contains, and explain that the console isn't running,
        # instead of rendering an empty page.
        self._serve_json({
            "control": SESSIONS is not None,
            "scenarios": session_mod.load_scenarios(),
        })

    # -- session endpoints -------------------------------------------------

    def _serve_log(self):
        query = parse_qs(urlsplit(self.path).query)
        raw = query.get("offset", [None])[0]
        try:
            offset = int(raw) if raw is not None else None
        except ValueError:
            self._serve_error(400, "offset must be an integer")
            return
        self._serve_json(SESSIONS.log_tail(offset))

    def _start_session(self):
        body = self._read_json_body()
        if body is None:
            return
        name = body.get("scenario")
        if not isinstance(name, str):
            self._serve_error(400, "scenario must be a string")
            return
        overrides = body.get("overrides") or {}
        if not isinstance(overrides, dict):
            self._serve_error(400, "overrides must be an object")
            return
        try:
            self._serve_json(SESSIONS.start(name, overrides, force=bool(body.get("force"))))
        except session_mod.SessionConflict as exc:
            self._serve_error(409, str(exc))
        except session_mod.ScenarioError as exc:
            self._serve_error(400, str(exc))
        except OSError as exc:
            self._serve_error(500, f"failed to launch: {exc}")

    # -- plumbing ----------------------------------------------------------

    def _with_control(self, fn):
        if SESSIONS is None:
            self._serve_error(
                503,
                "the dashboard console is not running — start it with "
                "scripts/run_console.sh to launch scenarios from here",
            )
            return
        fn()

    def _read_json_body(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self._serve_error(400, "bad Content-Length")
            return None
        if length > MAX_BODY_BYTES:
            self._serve_error(413, "request body too large")
            return None
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            self._serve_error(400, "body must be JSON")
            return None
        if not isinstance(body, dict):
            self._serve_error(400, "body must be a JSON object")
            return None
        return body

    def _serve_json(self, obj, code: int = 200) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_error(self, code: int, message: str) -> None:
        self._serve_json({"error": message}, code=code)

    def log_message(self, fmt, *args):
        pass  # SimpleHTTPRequestHandler logs every request to stderr; noisy for a dashboard.


def main() -> None:
    global SESSIONS

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument(
        "--control", action="store_true",
        help="enable the Scenarios tab's start/stop API (see this file's docstring)",
    )
    args = parser.parse_args()

    if args.control:
        SESSIONS = session_mod.SessionManager()

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    mode = "control" if args.control else "read-only"
    print(f"SortBots dashboard ({mode}): http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
