#!/usr/bin/env python3
"""The TensorRT half of nodes/dynamic_segment.py, run on the Jetson HOST.

The robot's ROS stack is a Jazzy container (Ubuntu 24.04, py3.12); JetPack 6's
TensorRT and its Python bindings are the host's (22.04, py3.10), and the
container sees neither (2026-11-20: no tensorrt, no libnvinfer, no CUDA in it).
So the node keeps everything but the engines, and asks this server over
localhost (the container shares the host's network) with its `remote` backend:
the same TrtSession, the same tensors, just on the other side of the wall.
Nothing leaves the robot.

    scripts/jetson.sh seg start      # how it is normally run (host, background)
    python3 scripts/dynamic_seg_server.py [--models models] [--port 7311]

Needs the host's TensorRT (python3-libnvinfer) and cuda-python; jetson.sh puts
cuda-python in models/.pylib on first start. Engines are models/<stem>.engine,
built on this machine (scripts/dynamic_seg_models.sh engines). One request at a
time reaches the GPU: the node is the only client, and two would only queue.
"""
from __future__ import annotations

import argparse
import socketserver
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "nodes"))
from dynamic_segment import TrtSession, recv_msg, send_msg  # noqa: E402


class Engines:
    def __init__(self, models: Path):
        self.models = models
        self.sessions: dict[str, TrtSession] = {}
        self.gpu = threading.Lock()
        self.stats: dict[str, list[float]] = {}

    def get(self, stem: str) -> TrtSession:
        # a stem only: never a path out of the models dir
        if not stem or Path(stem).name != stem or stem.startswith("."):
            raise ValueError(f"bad model name {stem!r}")
        if stem not in self.sessions:
            path = self.models / f"{stem}.engine"
            if not path.is_file():
                raise FileNotFoundError(f"{path} not built (scripts/dynamic_seg_models.sh engines)")
            with self.gpu:
                self.sessions[stem] = TrtSession(str(path))
            print(f"[seg-server] loaded {path.name}", flush=True)
        return self.sessions[stem]

    def run(self, stem: str, feeds: dict) -> dict:
        s = self.get(stem)
        t0 = time.perf_counter()
        with self.gpu:
            out = s.run(feeds)
        self.stats.setdefault(stem, []).append((time.perf_counter() - t0) * 1e3)
        return out


def make_handler(engines: Engines):
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            print(f"[seg-server] client {self.client_address[0]}:{self.client_address[1]}", flush=True)
            while True:
                try:
                    header, arrays = recv_msg(self.request)
                except (ConnectionError, OSError):
                    return
                try:
                    if header.get("op") == "open":
                        s = engines.get(header.get("model", ""))
                        send_msg(self.request, {"input_names": s.input_names, "output_names": s.output_names})
                    elif header.get("op") == "run":
                        send_msg(self.request, {}, engines.run(header.get("model", ""), arrays))
                    else:
                        send_msg(self.request, {"error": f"unknown op {header.get('op')!r}"})
                except Exception as e:  # noqa: BLE001 - every failure goes back to the node as words
                    send_msg(self.request, {"error": f"{type(e).__name__}: {e}"})
    return Handler


def report(engines: Engines, every_s: float) -> None:
    while True:
        time.sleep(every_s)
        line = []
        for stem, ms in list(engines.stats.items()):
            if ms:
                ms.sort()
                line.append(f"{stem} {len(ms)}x median {ms[len(ms) // 2]:.1f} ms")
                engines.stats[stem] = []
        if line:
            print("[seg-server] " + "; ".join(line), flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", type=Path, default=REPO / "models")
    ap.add_argument("--host", default="127.0.0.1", help="localhost only: the node is on this machine")
    ap.add_argument("--port", type=int, default=7311)
    ap.add_argument("--report-s", type=float, default=30.0, help="timing line this often (0 = never)")
    a = ap.parse_args(argv)
    engines = Engines(a.models.resolve())
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    srv = socketserver.ThreadingTCPServer((a.host, a.port), make_handler(engines))
    srv.daemon_threads = True
    if a.report_s > 0:
        threading.Thread(target=report, args=(engines, a.report_s), daemon=True).start()
    print(f"[seg-server] {engines.models} on {a.host}:{a.port}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
