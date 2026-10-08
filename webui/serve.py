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
# session.py puts scripts/ on sys.path for this; both are ROS-free by design.
import maps_lib

WEBUI_DIR = Path(__file__).resolve().parent
WAYPOINTS_CONFIG = WEBUI_DIR.parent / "configs" / "waypoints.yaml"
ROBOTS_CONFIG = WEBUI_DIR.parent / "configs" / "robots.yaml"

MAX_BODY_BYTES = 64 * 1024

# Set by main() when --control is given. None means "control endpoints off".
SESSIONS: session_mod.SessionManager | None = None

# sim | real — which platform's scenarios this console lists and can start.
# Set by main() from --platform (default: session_mod.detect_platform()).
PLATFORM = "sim"


def _load_nav_waypoints() -> dict:
    return {"platform": PLATFORM, "waypoints": maps_lib.read_working_waypoints(PLATFORM)}


def _session_robot_ids(roster: list[str]) -> list[str] | None:
    if SESSIONS is None:
        return None
    status = SESSIONS.status()
    if status.get("state") not in ("starting", "running"):
        return None
    run = status.get("run") or {}
    if run.get("robot_ids"):
        return [r for r in str(run["robot_ids"]).split(",") if r]
    if isinstance(run.get("robots"), int) and run["robots"] > 0:
        return roster[:run["robots"]]
    return None


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
        elif route == "/api/nav_waypoints":
            self._serve_json(_load_nav_waypoints())
        elif route == "/api/launch/options":
            self._serve_launch_options()
        elif route == "/api/robots":
            self._serve_robots()
        elif route == "/api/scenarios":
            self._serve_scenarios()
        elif route == "/api/maps":
            self._serve_maps()
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
        elif route == "/api/session/finish":
            self._with_control(self._finish_session)
        elif route == "/api/nav_waypoints":
            self._save_nav_waypoints()
        elif route == "/api/launch/preview":
            self._preview_launch()
        elif route == "/api/presets":
            self._with_control(self._save_preset)
        elif route == "/api/map/save":
            self._with_control(self._save_map)
        elif route == "/api/map/clear":
            self._with_control(self._clear_map)
        elif route == "/api/map/load":
            self._with_control(self._load_map)
        else:
            self._serve_error(404, f"no such endpoint: {route}")

    # -- config endpoints --------------------------------------------------

    def _serve_waypoints(self):
        with open(WAYPOINTS_CONFIG) as f:
            stations = yaml.safe_load(f)["stations"]
        self._serve_json(stations)

    # -- launch form -------------------------------------------------------

    def _serve_launch_options(self):
        """Everything the Scenarios tab's form needs to draw itself.

        Readable without --control (like /api/scenarios), so the form still
        shows what COULD be launched — and why it can't — when the console
        is down.
        """
        roster = ["robot_0"]
        if ROBOTS_CONFIG.exists():
            with open(ROBOTS_CONFIG) as f:
                roster = [r["id"] for r in yaml.safe_load(f)["robots"]]
        maps = []
        for m in maps_lib.list_maps():
            wp = maps_lib.MAPS_DIR / m["name"] / maps_lib.MAP_WAYPOINTS_NAME
            try:
                n_wp = len(json.loads(wp.read_text()).get("waypoints", [])) if wp.is_file() else 0
            except (OSError, ValueError):
                n_wp = 0
            maps.append({
                "name": m["name"], "title": m.get("title") or m["name"],
                "db_state": m.get("db_state"), "error": m.get("error"),
                "free_m2": (m.get("coverage") or {}).get("free_m2"),
                "scene": m.get("scene"), "waypoints": n_wp,
            })
        presets = []
        for sc in session_mod.load_scenarios():
            if sc["platform"] not in (PLATFORM, None):
                continue
            entry = {"name": sc["name"], "title": sc["title"], "status": sc["status"],
                     "description": sc["description"], "origin": sc.get("origin", "file")}
            if sc["status"] == "invalid":
                entry["reason"] = sc.get("error")
            else:
                entry.update(session_mod.run_to_config(sc["run"], sc["platform"],
                                                       sc["capture"]["bag"]))
            presets.append(entry)
        working = session_mod.working_db_path("robot_0")
        self._serve_json({
            "control": SESSIONS is not None,
            "platform": PLATFORM,
            "fields": sorted(session_mod.CONFIG_KEYS[PLATFORM]),
            "defaults": {k: v for k, v in session_mod.CONFIG_DEFAULTS.items()
                         if k in session_mod.CONFIG_KEYS[PLATFORM]},
            "scenes": ["nvidia", "primitive"],
            "roster": roster,
            "maps": maps,
            "working_db": {"path": str(working), "exists": working.exists()},
            "working_waypoints": len(maps_lib.read_working_waypoints(PLATFORM)),
            "presets": presets,
        })

    def _preview_launch(self):
        # Not behind --control: it launches nothing, and the form should be
        # able to explain itself even with the console down.
        body = self._read_json_body()
        if body is None:
            return
        try:
            self._serve_json(session_mod.preview_config(body.get("config") or {}, PLATFORM))
        except session_mod.ScenarioError as exc:
            self._serve_error(400, str(exc))
        except (TypeError, ValueError) as exc:
            self._serve_error(400, f"bad setting: {exc}")

    def _save_preset(self):
        body = self._read_json_body()
        if body is None:
            return
        try:
            self._serve_json(session_mod.write_preset(
                str(body.get("name", "")), body.get("config") or {}, PLATFORM,
                title=body.get("title"), description=body.get("description"),
                overwrite=bool(body.get("overwrite"))))
        except session_mod.ScenarioError as exc:
            self._serve_error(400, str(exc))
        except OSError as exc:
            self._serve_error(500, f"failed to write preset: {exc}")

    def _save_nav_waypoints(self):
        # Storage lives in maps_lib (data/nav_waypoints.json, the WORKING set;
        # saving/loading a map snapshots/restores it). Not behind --control: it
        # only rewrites one JSON file under data/ and
        # needs no ROS shell, so the plain dashboard can place pins too. The
        # client always sends the full list (last write wins) — two operators
        # editing at once is not a case worth a merge protocol.
        body = self._read_json_body()
        if body is None:
            return
        try:
            maps_lib.write_working_waypoints(PLATFORM, body.get("waypoints"))
        except maps_lib.MapError as exc:
            self._serve_error(400, str(exc))
            return
        self._serve_json(_load_nav_waypoints())

    def _serve_robots(self):
        # configs/robots.yaml is the fleet roster (ids + per-scene spawn
        # poses); the dashboard only needs the id list, so reduce each
        # `robots:` entry ({id, spawn}) to its id here. Missing file degrades
        # to the single-robot default rather than erroring the whole page.
        if ROBOTS_CONFIG.exists():
            with open(ROBOTS_CONFIG) as f:
                raw = yaml.safe_load(f)
            robots = raw["robots"]
            # On the robot, only ids with a `real` spawn entry exist in
            # hardware; listing the sim-only ones would offer a switcher
            # entry whose topics nothing publishes.
            if PLATFORM == "real":
                robots = [r for r in robots if "real" in (r.get("spawn") or {})]
            ids = [r["id"] for r in robots]
            # The roster is every robot this CONFIG knows; a running session
            # may use fewer. Listing the roster made a 1-robot run offer
            # robot_1 in the switcher (a page of dead topics) and label the
            # map "2 robots". So narrow to the live session when there is one
            # — same derivation as run_demo.sh: explicit robot_ids, else the
            # first --robots N of the roster, in roster order.
            live = _session_robot_ids(ids)
            data = {
                "default": raw["default"],
                "robots": live or ids,
                "configured": ids,
            }
        else:
            data = {"default": "robot_0", "robots": ["robot_0"]}
        self._serve_json(data)

    def _serve_scenarios(self):
        # Readable without --control on purpose: the tab can then show what
        # the suite contains, and explain that the console isn't running,
        # instead of rendering an empty page.
        # Only this platform's scenarios: the same dashboard is served by the
        # desktop console (Isaac) and the robot's (Jetson), and a card that can
        # never start here is noise. Invalid files (platform None) stay listed
        # on both — a scenario that won't load should say why wherever you look.
        scenarios = [s for s in session_mod.load_scenarios()
                     if s["platform"] in (PLATFORM, None)]
        self._serve_json({
            "control": SESSIONS is not None,
            "platform": PLATFORM,
            "scenarios": scenarios,
        })

    def _serve_maps(self):
        # Readable without --control for the same reason /api/scenarios is: the
        # Scenarios tab's map picker should show what the library contains and
        # explain that the console isn't running, rather than render empty.
        self._serve_json({
            "control": SESSIONS is not None,
            "dir": str(maps_lib.MAPS_DIR),
            "maps": maps_lib.list_maps(),
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
        # Two shapes: {config} from the launch form, or {scenario, overrides}
        # (a preset by name — sim_ctl.sh and older pages).
        config = body.get("config")
        name = body.get("scenario")
        if config is not None:
            if not isinstance(config, dict):
                self._serve_error(400, "config must be an object")
                return
        elif not isinstance(name, str):
            self._serve_error(400, "scenario must be a string (or send a config)")
            return
        overrides = body.get("overrides") or {}
        if not isinstance(overrides, dict):
            self._serve_error(400, "overrides must be an object")
            return
        try:
            self._serve_json(SESSIONS.start(name, overrides, force=bool(body.get("force")),
                                            config=config))
        except session_mod.SessionConflict as exc:
            self._serve_error(409, str(exc))
        except session_mod.ScenarioError as exc:
            self._serve_error(400, str(exc))
        except OSError as exc:
            self._serve_error(500, f"failed to launch: {exc}")

    def _save_map(self):
        """Save the running session's map into the library under a name.

        Blocking: map_saver_cli plus a VACUUMed multi-hundred-MB sqlite copy
        takes tens of seconds, and the answer the dashboard needs — did the
        pose graph make it in, or is it still pending? — isn't knowable until
        it finishes. ThreadingHTTPServer keeps the rest of the UI responsive
        meanwhile.
        """
        body = self._read_json_body()
        if body is None:
            return
        name = body.get("name")
        if not isinstance(name, str):
            self._serve_error(400, "name must be a string")
            return
        try:
            manifest = session_mod.save_map_blocking(
                name,
                robot_id=body.get("robot_id") or "robot_0",
                title=body.get("title"),
                description=body.get("description"),
            )
        except maps_lib.MapError as exc:
            self._serve_error(400, str(exc))
            return
        except OSError as exc:
            self._serve_error(500, f"failed to save map: {exc}")
            return
        self._serve_json(manifest)

    def _finish_session(self):
        """Save the running session into maps/<name>, then stop it."""
        body = self._read_json_body()
        if body is None:
            return
        try:
            self._serve_json(SESSIONS.finish(
                str(body.get("name", "")), body.get("robot_id") or "robot_0"))
        except maps_lib.MapError as exc:
            self._serve_error(400, str(exc))
        except session_mod.SessionConflict as exc:
            self._serve_error(409, str(exc))

    def _clear_map(self):
        """Wipe the live map (RTAB-Map reset + odometry reset)."""
        body = self._read_json_body()
        if body is None:
            return
        try:
            self._serve_json(session_mod.clear_map_blocking(body.get("robot_id") or "robot_0"))
        except maps_lib.MapError as exc:
            self._serve_error(400, str(exc))
        except OSError as exc:
            self._serve_error(500, f"failed to clear map: {exc}")

    def _load_map(self):
        """Load a library entry into the running RTAB-Map (from a copy)."""
        body = self._read_json_body()
        if body is None:
            return
        name = body.get("name")
        if not isinstance(name, str):
            self._serve_error(400, "name must be a string")
            return
        try:
            self._serve_json(session_mod.load_map_blocking(
                name, body.get("robot_id") or "robot_0"))
        except maps_lib.MapError as exc:
            self._serve_error(400, str(exc))
        except OSError as exc:
            self._serve_error(500, f"failed to load map: {exc}")

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
    global SESSIONS, PLATFORM

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument(
        "--platform", choices=session_mod.PLATFORMS, default=None,
        help="sim (desktop, Isaac) or real (robot Jetson); default: "
             "$SORTBOTS_PLATFORM, else real if /etc/nv_tegra_release exists, else sim",
    )
    parser.add_argument(
        "--control", action="store_true",
        help="enable the Scenarios tab's start/stop API (see this file's docstring)",
    )
    args = parser.parse_args()

    PLATFORM = args.platform or session_mod.detect_platform()
    if args.control:
        SESSIONS = session_mod.SessionManager(PLATFORM)

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    mode = "control" if args.control else "read-only"
    print(f"SortBots dashboard ({mode}, {PLATFORM}): http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
