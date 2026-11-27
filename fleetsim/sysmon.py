"""A run's local compute log: system.jsonl and crash.txt in its result dir.

This laptop intermittently segfaults python, or raises errors no code path
can produce, in long runs (docs/fleetsim.md, "Known issues"). The leading
suspect is the CPU, so every run records what it ran on, in a form that
survives the crash:

    system.jsonl   one JSON object per line, written, flushed and fsync'd per
                   line by a daemon thread (a segfault loses at most the
                   current 2 s; fsync also keeps it across a hard lock-up):
      {"type": "info", ...}     once: pid, sampler tid, python, numpy, kernel,
                                CPU model, microcode, online CPUs, governor
      {"type": "sample", ...}   every interval_s: t (epoch), up_s, sim_s, phase,
                                cpu (the core the main thread is on, field 39
                                of /proc/self/stat), core (its physical core
                                id), mhz, pkg_c / core_c / max_c (package, that
                                core, hottest core), load, rss_mb, cpu_s
      {"type": "end", ...}      on a clean exit or a Python exception; a run
                                killed by a signal has no end line
    crash.txt      faulthandler's stack of every thread on a fatal signal, or
                   the traceback of an uncaught exception. Removed when empty,
                   so it exists only for a run that crashed.

faulthandler costs the kernel's own segfault line: the kernel logs a fault
("segfault at ... likely on CPU 5 (core 8, socket 0)") only when no handler
caught it, and faulthandler re-raises with raise(), which the kernel counts as
a sent signal (checked 2026-11-26). The bench gets the CPU of death from the
dead process's /proc stat instead (bench.py), but the faulting instruction's
address is only in the kernel's line: FLEETSIM_FAULTHANDLER=0 turns
faulthandler off (`bench --no-faulthandler`) to get that line back.

Every probe degrades to null when unreadable (another OS, a VM, a container).
Nothing here touches the simulation: the sim never reads the wall clock, so
a run with the monitor is bit-identical to one without.
"""
from __future__ import annotations

import faulthandler
import json
import os
import platform
import sys
import threading
import time
import traceback
from pathlib import Path

INTERVAL_S = 2.0
CPU_DIR = Path("/sys/devices/system/cpu")
# Logged with the info line when set: each has been suspected of, or used to
# rule out, the interpreter faults.
ENV_KEYS = ("PYTHONHASHSEED", "PYTHONMALLOC", "PYTHONFAULTHANDLER", "OPENBLAS_NUM_THREADS",
            "OMP_NUM_THREADS", "FLEETSIM_FAULTHANDLER")


def _read(path) -> str | None:
    try:
        with open(path) as f:
            return f.read()
    except (OSError, UnicodeDecodeError):
        return None


def _num(path, scale: float = 1.0) -> float | None:
    s = _read(path)
    try:
        return round(int(s) / scale, 1)
    except (TypeError, ValueError):
        return None


def cpuinfo() -> dict:
    """CPU model and microcode, from the first processor block only: reading
    the whole of /proc/cpuinfo samples every CPU's frequency, which on recent
    kernels costs milliseconds per CPU."""
    out = {"cpu_model": None, "microcode": _read(CPU_DIR / "cpu0" / "microcode" / "version")}
    out["microcode"] = out["microcode"].strip() if out["microcode"] else None
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if not line.strip():
                    break
                k, _, v = line.partition(":")
                k = k.strip()
                if k == "model name":
                    out["cpu_model"] = v.strip()
                elif k == "microcode" and out["microcode"] is None:
                    out["microcode"] = v.strip()
    except OSError:
        pass
    return out


def core_ids() -> dict[int, int]:
    """Logical CPU -> physical core id: what the kernel's segfault line calls
    "core" and coretemp labels "Core N"."""
    out = {}
    for d in CPU_DIR.glob("cpu[0-9]*"):
        cid = _read(d / "topology" / "core_id")
        try:
            out[int(d.name[3:])] = int(cid)
        except (TypeError, ValueError):
            continue
    return out


def _temp_sensors() -> tuple[str | None, dict]:
    """("coretemp", {label: input path}), else ("thermal", {zone type: path})."""
    for d in sorted(Path("/sys/class/hwmon").glob("hwmon*")):
        if (_read(d / "name") or "").strip() != "coretemp":
            continue
        labels = {}
        for lab in d.glob("temp*_label"):
            name = (_read(lab) or "").strip()
            if name:
                labels[name] = lab.with_name(lab.name.replace("_label", "_input"))
        if labels:
            return "coretemp", labels
    zones = {}
    for z in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        kind = (_read(z / "type") or "").strip()
        if kind:
            zones[kind] = z / "temp"
    return ("thermal", zones) if zones else (None, {})


def static_info() -> dict:
    """What a crash report needs once per process: what ran, on what."""
    try:
        import numpy
        np_version = numpy.__version__
    except Exception:  # noqa: BLE001 - a log line must never stop a run
        np_version = None
    gov = _read(CPU_DIR / "cpu0" / "cpufreq" / "scaling_governor")
    return {
        "python": sys.version.split()[0], "python_build": sys.version, "executable": sys.executable,
        "numpy": np_version, "kernel": platform.release(), "kernel_version": platform.version(),
        **cpuinfo(),
        "cpus_online": (_read(CPU_DIR / "online") or "").strip() or None,
        "cpus_offline": (_read(CPU_DIR / "offline") or "").strip() or None,
        "governor": gov.strip() if gov else None,
        "env": {k: os.environ[k] for k in ENV_KEYS if k in os.environ},
    }


class Monitor:
    """Start it as soon as the run has a dir; stop() it on the way out.

        mon = Monitor(out)
        mon.clock = lambda: sim.now     # optional: sim time in each sample
        mon.phase = "score"             # optional: where the run is
        ...
        mon.stop(exc)                   # exc: the exception that ended it, if any
    """

    def __init__(self, out: Path, interval_s: float = INTERVAL_S):
        self.out = Path(out)
        self.interval_s = float(interval_s)
        self.clock = None
        self.phase = "setup"
        self._t0 = time.monotonic()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._stopped = False
        self._pid = os.getpid()
        self._cores = core_ids()
        self._temps_src, self._temps = _temp_sensors()
        self._page = os.sysconf("SC_PAGE_SIZE") if hasattr(os, "sysconf") else 4096
        self._f = self._open(self.out / "system.jsonl", "w")
        self._crash = self._open(self.out / "crash.txt", "w")
        self._fh_was_on = faulthandler.is_enabled()
        self.faulthandler = os.environ.get("FLEETSIM_FAULTHANDLER", "1") != "0"
        if self._crash is not None and self.faulthandler:
            faulthandler.enable(file=self._crash, all_threads=True)
        self._thread = None
        if self._f is not None:
            self._thread = threading.Thread(target=self._loop, name="fleetsim-sysmon", daemon=True)
            self._thread.start()
            # The info line (with the sampler's tid, which the kernel's segfault
            # line names if that thread is the one that faulted) is on disk
            # before the run proper starts.
            self._ready.wait(1.0)

    @staticmethod
    def _open(path: Path, mode: str):
        try:
            return open(path, mode, buffering=1)
        except OSError:
            return None

    def _write(self, d: dict) -> None:
        if self._f is None:
            return
        with self._lock:
            try:
                self._f.write(json.dumps(d, separators=(",", ":"), default=str) + "\n")
                self._f.flush()
                os.fsync(self._f.fileno())
            except (OSError, ValueError):  # disk full, or closed by stop()
                pass

    def _loop(self) -> None:
        try:
            self._write({"type": "info", "t": round(time.time(), 3), "pid": self._pid,
                         "sampler_tid": threading.get_native_id(), "argv": sys.argv,
                         "interval_s": self.interval_s, "temps": self._temps_src,
                         "faulthandler": self.faulthandler,
                         **static_info()})
        finally:
            self._ready.set()
        err = None
        while True:
            try:
                self._write(self.sample())
            except Exception as e:  # noqa: BLE001 - say so once, keep sampling
                if err is None:
                    err = repr(e)
                    self._write({"type": "error", "t": round(time.time(), 3), "error": err})
            if self._stop.wait(self.interval_s):
                return

    def sample(self) -> dict:
        d = {"type": "sample", "t": round(time.time(), 3),
             "up_s": round(time.monotonic() - self._t0, 2), "sim_s": None, "phase": self.phase}
        clock = self.clock
        if clock is not None:
            try:
                d["sim_s"] = round(float(clock()), 2)
            except Exception:  # noqa: BLE001
                pass
        cpu = None
        stat = _read("/proc/self/stat")
        if stat:
            # comm (field 2) is parenthesised and may hold spaces: split after it.
            # Field 39 is "processor", the CPU the main thread last ran on.
            rest = stat.rsplit(")", 1)[-1].split()
            try:
                cpu = int(rest[39 - 3])
            except (IndexError, ValueError):
                pass
        core = self._cores.get(cpu)
        d["cpu"], d["core"] = cpu, core
        d["mhz"] = None if cpu is None else _num(CPU_DIR / f"cpu{cpu}" / "cpufreq" / "scaling_cur_freq",
                                                  1000.0)
        d.update(self._temps_now(core))
        try:
            d["load"] = [round(x, 2) for x in os.getloadavg()]
        except OSError:
            d["load"] = None
        statm = _read("/proc/self/statm")
        try:
            d["rss_mb"] = round(int(statm.split()[1]) * self._page / 2 ** 20, 1)
        except (AttributeError, IndexError, ValueError):
            d["rss_mb"] = None
        d["cpu_s"] = round(time.process_time(), 2)
        return d

    def _temps_now(self, core: int | None) -> dict:
        out = {"pkg_c": None, "core_c": None, "max_c": None}
        if self._temps_src == "coretemp":
            cores = []
            for label, path in self._temps.items():
                v = _num(path, 1000.0)
                if v is None:
                    continue
                if label.startswith("Package"):
                    out["pkg_c"] = v if out["pkg_c"] is None else max(out["pkg_c"], v)
                elif label.startswith("Core"):
                    cores.append(v)
                    if core is not None and label == f"Core {core}":
                        out["core_c"] = v
            out["max_c"] = max(cores) if cores else None
        elif self._temps_src == "thermal":
            vals = {k: _num(p, 1000.0) for k, p in self._temps.items()}
            out["pkg_c"] = vals.get("x86_pkg_temp")
            good = [v for v in vals.values() if v is not None]
            out["max_c"] = max(good) if good else None
        return out

    def stop(self, exc: BaseException | None = None) -> None:
        """Idempotent. exc: what ended the run, None for a normal return."""
        if self._stopped:
            return
        self._stopped = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        outcome = "ok" if exc is None else ("interrupted" if isinstance(exc, KeyboardInterrupt)
                                            else "error")
        end = {**self.sample(), "type": "end", "outcome": outcome}
        if outcome == "error":
            end["error"] = repr(exc)[:500]
        self._write(end)
        if self._f is not None:
            self._f.close()
            self._f = None
        if self._crash is not None:
            if outcome == "error":
                try:
                    self._crash.write("".join(traceback.format_exception(exc)))
                except OSError:
                    pass
            if self.faulthandler:
                # Off before the file closes: a fatal signal later in the process
                # would otherwise write the dump to whatever reuses that fd.
                faulthandler.disable()
                if self._fh_was_on and sys.__stderr__ is not None:  # pytest, -X faulthandler
                    faulthandler.enable(file=sys.__stderr__, all_threads=True)
            try:
                self._crash.flush()
                empty = os.fstat(self._crash.fileno()).st_size == 0
            except (OSError, ValueError):
                empty = False
            self._crash.close()
            self._crash = None
            if empty:
                try:
                    (self.out / "crash.txt").unlink()
                except OSError:
                    pass


def read(path) -> list[dict]:
    """system.jsonl -> rows; a torn last line (the process died mid-write) is skipped."""
    out = []
    for line in (_read(path) or "").splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out
