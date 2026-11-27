"""Backend for the scenario editor in the operator view's Tests tab
(webui/operator_fleetedit.js): list, read, check and save fleetsim scenarios.

serve.py routes:
  GET  /api/fleetsim/editor                what the editor offers: scenarios, maps, worlds, metrics, base.yaml
  GET  /api/fleetsim/editor/scenario?name  one file: its mapping, header comment, the comments a save drops
  POST /api/fleetsim/editor/check          {scenario, grid}: errors, floor problems, summary, the floor
  POST /api/fleetsim/editor/save           {scenario, header, overwrite, base_sha, source}; --control only

THE INPUT IS UNTRUSTED. The console can be reachable from the internet
(serve.py's docstring), and a saved scenario is something "Run a fleet sim"
then launches. So:
  * the only file ever written is fleetsim/scenarios/<name>.yaml, the name by
    config.NAME_RX, never a path, never through a symlink;
  * `config.world.map` must name a map in the maps library. fleetsim also
    takes a directory path there, which from here would read any grid.yaml on
    the machine (and is why the run form can't set it, fleetsim_runner.py);
  * no other string under `config:` may look like a path either. No setting
    takes one today; this keeps one added later from becoming a hole;
  * values typed as YAML in the page arrive as {"$yaml": text} and are parsed
    here with safe_load, and only plain data survives anywhere in the body
    (no dates, sets, bytes, NaN or infinity);
  * the floor is built by a /usr/bin/python3 child (fleetsim/preview.py) with
    a time and memory limit, one at a time, after its size has been checked.

Saving keeps the file's leading comment block (headers carry the scenario's
rationale, and a YAML round trip drops every comment), and copies each
top-level section nobody changed line for line, so the comments in it stay
too. A changed section is rewritten and loses its comments; `read` lists every
comment by section so the page can say which ones are at stake.
"""
from __future__ import annotations

import collections
import hashlib
import json
import math
import os
import re
import subprocess
import threading
from pathlib import Path

import yaml

from simtest_runner import REPO_ROOT  # must come first, it adds the repo root to sys.path
from fleetsim import config
from fleetsim.config import ScenarioError
from simtest import metrics as metrics_lib
from simtest import world as world_lib

# fleetsim needs numpy and serve.py doesn't (fleetsim_runner.PYTHON).
PYTHON = "/usr/bin/python3"
MARKER = "$yaml"
MAX_BODY_BYTES = 512 * 1024
MAX_TEXT = 8000            # any one string, the description and header included
MAX_DEPTH = 12
MAX_ITEMS = {"tasks": 5000, "disturbances": 500, "actors": world_lib.MAX_ITEMS, "metrics": 50}
# A synthetic floor's size bound, so a typo like width_m: 6000 can't make the
# preview (or a run) allocate gigabytes: 4 M cells is 400 x 400 m at 0.2 m.
MAX_SIDE_M = 1000.0
MAX_CELLS = 4_000_000
# grid.SYNTHETIC_DEFAULTS' floor (grid.py needs numpy, so it isn't imported here).
SYNTHETIC_SIZE = (60.0, 40.0)
PREVIEW_TIMEOUT_S = 20.0
WIDTH = 100                # the line length the writer keeps flow-style lines under

MAP_NAME_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")  # maps_lib.NAME_RX
# The first `path:` of an error message, e.g. "tasks[3].pickup: ..." -> tasks[3].pickup.
_WHERE_RX = re.compile(r"^([A-Za-z_][\w-]*(?:\[\d+\])*(?:\.[\w-]+(?:\[\d+\])*)*): ")
ORDER = ("name", "title", "description", "seed", "duration_s", "robots", "config", "tasks",
         "task_arrival", "disturbances", "actors", "stop", "metrics")
BLOCK_TOP = {"config", "tasks", "disturbances", "actors", "metrics"}
ITEM_BLOCK = {"metrics"}  # `- metric: nav`, as the hand-written files have them


class EditorError(Exception):
    def __init__(self, code: int, message: str, where: str | None = None):
        super().__init__(message)
        self.code, self.where = code, where


def where_of(message: str) -> str | None:
    """The most specific `path:` an error starts with. config.resolve nests
    them ("actors: actors[0].path.speed_mps: ..."), so keep the last one."""
    where, rest = None, message
    while True:
        m = _WHERE_RX.match(rest)
        if not m:
            return where
        where, rest = m.group(1), rest[m.end():]


def sha(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


# ---------------------------------------------------------------------------
# what there is to pick from
# ---------------------------------------------------------------------------

def maps_dir() -> Path:
    # fleetsim/grid.py's MAPS_DIR, by the same rule (that module needs numpy).
    return Path(os.environ.get("SORTBOTS_MAPS_DIR") or REPO_ROOT / "maps")


def library_maps() -> list[dict]:
    """Library maps fleetsim can load: a NAME_RX directory with a grid.yaml,
    really inside the maps dir (a symlink placed in maps/ could point out)."""
    root = maps_dir()
    if not root.is_dir():
        return []
    real = root.resolve()
    out = []
    for d in sorted(root.iterdir()):
        if not (MAP_NAME_RX.match(d.name) and d.is_dir() and (d / "grid.yaml").is_file()):
            continue
        if d.resolve().parent != real:
            continue
        title = d.name
        try:
            title = str(json.loads((d / "map.json").read_text()).get("title") or d.name)
        except (OSError, ValueError, AttributeError):
            pass
        out.append({"name": d.name, "title": title})
    return out


def catalog() -> dict:
    scenarios = []
    for p in sorted(config.SCENARIOS_DIR.glob("*.yaml")):
        entry = {"name": p.stem, "title": p.stem, "description": "", "status": "ready"}
        try:
            raw = yaml.safe_load(p.read_text())
            if isinstance(raw, dict):
                entry["title"] = str(raw.get("title") or p.stem)
                entry["description"] = " ".join(str(raw.get("description") or "").split())
            s = config.load(p.stem)
            entry.update(robots=s["config"]["robots"]["count"], network=s["config"]["network"]["model"],
                         duration_s=s["duration_s"])
        except (ScenarioError, yaml.YAMLError, OSError) as e:
            entry.update(status="invalid", error=str(e))
        scenarios.append(entry)
    return {
        "scenarios": scenarios,
        "maps": library_maps(),
        "worlds": [w["name"] for w in world_lib.list_worlds() if w["status"] == "ready"],
        "metrics": [{"name": m["name"], "summary": m["summary"], "values": m["values"],
                     "params": m["params"]} for m in metrics_lib.catalog()],
        "ops": sorted(metrics_lib.OPS),
        "disturbance_kinds": sorted(config.DISTURBANCE_KINDS),
        "top_keys": sorted(config.TOP_KEYS),
        # Every setting a scenario may override, with its default: the page
        # offers these as it is, so a knob added to base.yaml shows up there
        # without the page changing.
        "base": config.load_base(),
        "free_form": [".".join(f) for f in sorted(config.FREE_FORM)],
        "name_rx": config.NAME_RX.pattern,
    }


# ---------------------------------------------------------------------------
# reading a file: the mapping, the header, the comments a save would drop
# ---------------------------------------------------------------------------

def split_header(text: str) -> tuple[str, int]:
    """(header text without the `# `, index of the first body line)."""
    lines = text.splitlines()
    i = 0
    while i < len(lines) and (not lines[i].strip() or lines[i].lstrip().startswith("#")):
        i += 1
    while i > 0 and not lines[i - 1].strip():  # blank lines before the body aren't header
        i -= 1
    out = []
    for line in lines[:i]:
        s = line.lstrip()
        s = s[1:] if s.startswith("#") else s
        out.append(s[1:] if s.startswith(" ") else s)
    return "\n".join(out), i


def _comment_at(line: str) -> int | None:
    """Where a YAML comment starts in `line`, or None. Quotes only count where a
    scalar can start, so an apostrophe in plain text (Robot's) opens nothing."""
    quote, prev = None, None
    i = 0
    while i < len(line):
        c = line[i]
        if quote == "'":
            if c == "'":
                if line[i + 1:i + 2] == "'":
                    i += 1
                else:
                    quote = None
        elif quote == '"':
            if c == "\\":
                i += 1
            elif c == '"':
                quote = None
        elif c == "#" and (i == 0 or line[i - 1] in " \t"):
            return i
        elif c in "'\"" and prev in (None, ":", "-", "[", "{", ","):
            quote = c
        if not c.isspace():
            prev = c
        i += 1
    return None


_BLOCK_RX = re.compile(r"(?:^|[:\-]\s)\s*[|>][-+0-9]*\s*(?:#.*)?$")
_TOP_RX = re.compile(r"^([A-Za-z_][\w-]*)\s*:(?:\s|$)")


def sections(text: str, start: int) -> tuple[list[str], dict[str, list[str]]]:
    """The body's top-level keys in file order, and each one's lines. Blank
    and column-0 comment lines between two keys lead the next one: a comment
    sits above what it is about."""
    order: list[str] = []
    blocks: dict[str, list[str]] = {}
    cur, pending = None, []
    for line in text.splitlines()[start:]:
        m = _TOP_RX.match(line)
        if m:
            cur = m.group(1)
            order.append(cur)
            blocks[cur] = pending + [line]
            pending = []
        elif cur is None or not line.strip() or line.startswith("#"):
            pending.append(line)
        else:
            blocks[cur].extend(pending + [line])
            pending = []
    if cur is not None:
        blocks[cur].extend(pending)
    return order, blocks


def comments_in(lines: list[str]) -> list[str]:
    out, block_indent = [], None
    for line in lines:
        indent = len(line) - len(line.lstrip())
        if block_indent is not None:
            if not line.strip() or indent > block_indent:
                continue  # inside a | or > block scalar: a # there is text
            block_indent = None
        k = _comment_at(line)
        if k is not None:
            out.append(line[k + 1:].strip())
        if _BLOCK_RX.search(line[:k] if k is not None else line):
            block_indent = indent
    return out


def body_comments(text: str) -> list[dict]:
    """Comments below the header, by top-level section. A save keeps them
    where it keeps the section's own lines, i.e. where nothing in it changed."""
    _, start = split_header(text)
    order, blocks = sections(text, start)
    return [{"section": k, "text": c} for k in order for c in comments_in(blocks[k])]


def read(name: str) -> dict:
    try:
        p = config.path_for(name)
    except ScenarioError as e:
        raise EditorError(400, str(e)) from None
    try:
        text = p.read_text()
    except FileNotFoundError:
        raise EditorError(404, f"no scenario {name!r}") from None  # the name, never the path
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise EditorError(409, f"{name}.yaml isn't valid YAML, fix it in a text editor: {e}") from None
    if not isinstance(raw, dict):
        raise EditorError(409, f"{name}.yaml isn't a mapping")
    try:
        raw = plain(raw, "")
    except ScenarioError as e:
        raise EditorError(409, f"{name}.yaml can't be edited here: {e}") from None
    header, _ = split_header(text)
    return {"name": name, "scenario": raw, "header": header,
            "comments": body_comments(text), "sha": sha(text)}


# ---------------------------------------------------------------------------
# checking what the page sent
# ---------------------------------------------------------------------------

def _join(where: str, key) -> str:
    return f"{where}.{key}" if where else str(key)


def plain(v, where: str, depth: int = 0):
    """`v` if it is plain JSON-able data, else ScenarioError naming where."""
    if depth > MAX_DEPTH:
        raise ScenarioError(f"{where or 'scenario'}: nested too deep")
    if v is None or isinstance(v, (bool, int)):
        return v
    if isinstance(v, float):
        if not math.isfinite(v):
            raise ScenarioError(f"{where}: numbers must be finite")
        return v
    if isinstance(v, str):
        if len(v) > MAX_TEXT or "\x00" in v:
            raise ScenarioError(f"{where}: text too long (max {MAX_TEXT} characters)")
        return v
    if isinstance(v, list):
        return [plain(x, f"{where}[{i}]", depth + 1) for i, x in enumerate(v)]
    if isinstance(v, dict):
        out = {}
        for k, x in v.items():
            if not isinstance(k, str) or not k or len(k) > 128:
                raise ScenarioError(f"{_join(where, k)}: keys must be short strings")
            out[k] = plain(x, _join(where, k), depth + 1)
        return out
    raise ScenarioError(f"{where}: {type(v).__name__} isn't allowed here (plain YAML data only)")


def unmark(v, where: str = ""):
    """Parse every {"$yaml": text} the page sent (a value typed as YAML, read
    the way `--set key=value` reads it) into its value."""
    if isinstance(v, dict):
        if set(v) == {MARKER}:
            text = v[MARKER]
            if not isinstance(text, str) or len(text) > MAX_TEXT:
                raise ScenarioError(f"{where}: expected YAML text")
            try:
                return yaml.safe_load(text) if text.strip() else None
            except yaml.YAMLError as e:
                why = getattr(e, "problem", None) or str(e).splitlines()[0]
                raise ScenarioError(f"{where}: {text!r} isn't a YAML value ({why})") from None
        return {k: unmark(x, _join(where, k)) for k, x in v.items()}
    if isinstance(v, list):
        return [unmark(x, f"{where}[{i}]") for i, x in enumerate(v)]
    return v


def _strings(v, where):
    if isinstance(v, str):
        yield where, v
    elif isinstance(v, dict):
        for k, x in v.items():
            yield from _strings(x, _join(where, k))
    elif isinstance(v, list):
        for i, x in enumerate(v):
            yield from _strings(x, f"{where}[{i}]")


def _looks_like_path(s: str) -> bool:
    return "/" in s or "\\" in s or s.startswith("~") or ".." in s


def untrusted(scenario) -> dict:
    """The page's scenario -> a plain mapping with no path in it, or ScenarioError."""
    if not isinstance(scenario, dict):
        raise ScenarioError("scenario: expected a mapping")
    raw = plain(unmark(scenario), "")
    for key, cap in MAX_ITEMS.items():
        if isinstance(raw.get(key), list) and len(raw[key]) > cap:
            raise ScenarioError(f"{key}: at most {cap} from the console")
    cfg = raw.get("config")
    if cfg is None:
        return raw
    if not isinstance(cfg, dict):
        raise ScenarioError("config: expected a mapping")
    world = cfg.get("world")
    if world is not None and not isinstance(world, dict):
        raise ScenarioError("config.world: expected a mapping")
    world = world or {}
    m = world.get("map")
    if m is not None and (not isinstance(m, str) or m not in {x["name"] for x in library_maps()}):
        raise ScenarioError("config.world.map: pick a map from the maps library (scripts/maps.sh "
                            "list); the console takes a map's name, never a path")
    a = world.get("actors")
    if a is not None and (not isinstance(a, str) or not world_lib.NAME_RX.match(a)):
        raise ScenarioError("config.world.actors: the name of a world in configs/worlds/")
    for where, s in _strings(cfg, "config"):
        if _looks_like_path(s):
            raise ScenarioError(f"{where}: looks like a file path; the console takes names, not paths")
    return raw


def _size_check(scn: dict) -> None:
    w = scn["config"]["world"]
    if w["source"] != "synthetic":
        return
    syn = w.get("synthetic") or {}
    cell = float(w["cell_m"])
    dims = []
    for key, dflt in zip(("width_m", "height_m"), SYNTHETIC_SIZE):
        v = syn.get(key, dflt)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 < v <= MAX_SIDE_M:
            raise ScenarioError(f"config.world.synthetic.{key}: expected metres in (0, {MAX_SIDE_M:g}]")
        dims.append(float(v))
    cells = math.ceil(dims[0] / cell) * math.ceil(dims[1] / cell)
    if cells > MAX_CELLS:
        raise ScenarioError(f"config.world.cell_m: {dims[0]:g} x {dims[1]:g} m at {cell:g} m is "
                            f"{cells:,} cells, over the console's {MAX_CELLS:,}")


_preview_lock = threading.Lock()
_cache: collections.OrderedDict = collections.OrderedDict()
_cache_lock = threading.Lock()


def _preview_env() -> dict:
    env = {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "HOME": os.environ.get("HOME", "/tmp"),
           # One thread: OpenBLAS reserves address space per thread, and the
           # child runs under a memory limit (fleetsim/preview.py main).
           "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}
    for k in ("SORTBOTS_MAPS_DIR", "SORTBOTS_WORLDS_DIR", "FLEETSIM_SCENARIOS_DIR"):
        if os.environ.get(k):
            env[k] = os.environ[k]
    return env


def preview(raw: dict, grid: bool) -> dict:
    """fleetsim/preview.py's answer for a checked scenario. Cached: the page
    asks again for the same scenario on Save, and on every undo."""
    payload = json.dumps({"scenario": raw, "grid": bool(grid)})
    key = sha(json.dumps({"scenario": raw, "grid": bool(grid)}, sort_keys=True))
    with _cache_lock:
        if key in _cache:
            _cache.move_to_end(key)
            return _cache[key]
    # One child at a time: from the internet, this is the route that costs CPU.
    if not _preview_lock.acquire(timeout=PREVIEW_TIMEOUT_S):
        raise EditorError(503, "the floor preview is busy, try again")
    try:
        proc = subprocess.run([PYTHON, "-m", "fleetsim.preview", "-"], input=payload,
                              capture_output=True, text=True, timeout=PREVIEW_TIMEOUT_S,
                              cwd=str(REPO_ROOT), env=_preview_env())
    except subprocess.TimeoutExpired:
        return {"error": f"building the floor took over {PREVIEW_TIMEOUT_S:g} s", "where": "config.world"}
    except OSError as e:
        raise EditorError(500, f"can't start {PYTHON}: {e}") from None
    finally:
        _preview_lock.release()
    try:
        out = json.loads(proc.stdout)
    except ValueError:
        # A crash (see docs/fleetsim.md, Known issues) or the memory limit.
        tail = (proc.stderr or "").strip().splitlines()[-1:] or [f"exit {proc.returncode}"]
        return {"error": f"building the floor failed: {tail[0][:300]}", "where": "config.world"}
    with _cache_lock:
        _cache[key] = out
        while len(_cache) > 16:
            _cache.popitem(last=False)
    return out


def check(scenario, grid: bool = False) -> dict:
    """{ok, error, where, problems, summary, world}. Problems at level `error`
    make ok false: the run would crash on them or quietly do less."""
    try:
        raw = untrusted(scenario)
        scn = config.resolve(raw)
        _size_check(scn)
    except ScenarioError as e:
        return {"ok": False, "error": str(e), "where": where_of(str(e)), "problems": []}
    out = preview(raw, grid)
    if "error" in out:
        return {"ok": False, "error": out["error"], "where": out.get("where") or where_of(out["error"]),
                "problems": []}
    errors = [p for p in out["problems"] if p["level"] == "error"]
    return {"ok": not errors, "error": None, "where": None, "problems": out["problems"],
            "summary": out["summary"], "world": out["world"]}


# ---------------------------------------------------------------------------
# writing a file
# ---------------------------------------------------------------------------

def _scalar(v) -> str:
    """A scalar inside a flow collection: [..] and {..} quote more than a block."""
    if isinstance(v, str) and ("*" in v or "?" in v):
        return json.dumps(v, ensure_ascii=False)  # "shelf_*", as the hand-written files quote globs
    s = yaml.safe_dump([v], default_flow_style=True, width=float("inf"), allow_unicode=True).strip()[1:-1]
    return json.dumps(v, ensure_ascii=False) if "\n" in s else s  # JSON strings are YAML, on one line


def _scalar_block(v) -> str:
    """A scalar as a block mapping's value: commas and brackets stay plain text."""
    s = yaml.safe_dump({"k": v}, default_flow_style=False, width=float("inf"), allow_unicode=True)
    s = s[3:].rstrip("\n")
    return json.dumps(v, ensure_ascii=False) if "\n" in s else s


def _flow(v) -> str:
    if isinstance(v, dict):
        return "{" + ", ".join(f"{_scalar(k)}: {_flow(x)}" for k, x in v.items()) + "}"
    if isinstance(v, list):
        return "[" + ", ".join(_flow(x) for x in v) + "]"
    return _scalar(v)


def _depth(v) -> int:
    if isinstance(v, dict):
        return 1 + max((_depth(x) for x in v.values()), default=0)
    if isinstance(v, list):
        return max((_depth(x) for x in v), default=0)
    return 0


def _emit(lines: list, key, v, indent: int, top: bool = False) -> None:
    head = " " * indent + _scalar_block(key) + ":"
    if not isinstance(v, (dict, list)) or not v:
        lines.append(f"{head} {_scalar_block(v) if not isinstance(v, (dict, list)) else _flow(v)}")
        return
    flow = _flow(v)
    # One level of mapping on a line ({prior: truth}); deeper ones get their
    # own lines (world:/synthetic: {...}), except pass criteria, which are
    # always {value: {op: threshold}} and read best on one line.
    if not (top and key in BLOCK_TOP) and len(head) + 1 + len(flow) <= WIDTH \
            and (_depth(v) <= 1 or key == "pass" or isinstance(v, list)):
        lines.append(f"{head} {flow}")
        return
    lines.append(head)
    if isinstance(v, dict):
        for k, x in v.items():
            _emit(lines, k, x, indent + 2)
    else:
        for x in v:
            _emit_item(lines, x, indent + 2, block=top and key in ITEM_BLOCK)


def _wrapped(pad: str, item: dict) -> list[str] | None:
    """`- {a: 1, b: 2,` / `   c: 3}`: a flow mapping over several lines."""
    first, cont = f"{pad}- {{", f"{pad}   "
    entries = [f"{_scalar(k)}: {_flow(x)}" for k, x in item.items()]
    out, line = [], first
    for i, e in enumerate(entries):
        piece = e + (", " if i < len(entries) - 1 else "}")
        if line not in (first, cont) and len(line) + len(piece.rstrip()) > WIDTH:
            out.append(line.rstrip())
            line = cont
        line += piece
    out.append(line)
    return out if all(len(x) <= WIDTH for x in out) else None


def _emit_item(lines: list, item, indent: int, block: bool = False) -> None:
    pad = " " * indent
    if not isinstance(item, (dict, list)) or not item:
        lines.append(f"{pad}- {_scalar_block(item) if not isinstance(item, (dict, list)) else _flow(item)}")
        return
    if not block:
        flow = _flow(item)
        if len(pad) + 2 + len(flow) <= WIDTH:
            lines.append(f"{pad}- {flow}")
            return
        wrapped = _wrapped(pad, item) if isinstance(item, dict) else None
        if wrapped:
            lines.extend(wrapped)
            return
    if isinstance(item, list):
        lines.append(f"{pad}-")
        for x in item:
            _emit_item(lines, x, indent + 2)
        return
    sub: list = []
    for k, x in item.items():
        _emit(sub, k, x, indent + 2)
    sub[0] = f"{pad}- {sub[0][indent + 2:]}"  # the first key on the dash's line
    lines.extend(sub)


def _description(lines: list, text: str) -> None:
    t = text.strip()
    if t and "\n" not in t and t == " ".join(t.split()):
        lines.append("description: >")  # folded, as the hand-written files have it
        line = ""
        for word in t.split(" "):
            if line and len(line) + 1 + len(word) > 76:
                lines.append("  " + line)
                line = word
            else:
                line = f"{line} {word}" if line else word
        lines.append("  " + line)
    else:
        _emit(lines, "description", text, 0, top=True)


def _same(a, b) -> bool:
    """Equal and of the same types all the way down (1 is not 1.0, nor True)."""
    if type(a) is not type(b):
        return False
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


def _norm(raw: dict) -> dict:
    out = dict(raw)
    if isinstance(out.get("description"), str):
        out["description"] = out["description"].strip()
    return out


def _order(old: list[str], new: list[str]) -> list[str]:
    """The old file's order, with keys it didn't have slotted in by ORDER."""
    rank = {k: i for i, k in enumerate(ORDER)}
    out = [k for k in old if k in new]
    for k in [k for k in ORDER if k in new and k not in out] + [k for k in new if k not in out and k not in rank]:
        i = rank.get(k, len(ORDER))
        after = [j for j, x in enumerate(out) if rank.get(x, len(ORDER)) <= i]
        out.insert(after[-1] + 1 if after else 0, k)
    return out


def dump(raw: dict, header: str = "", old_text: str | None = None) -> tuple[str, list[dict]]:
    """(file text, comments dropped). With `old_text`, a top-level section
    whose value didn't change is copied from it line for line, comments
    included: only what was edited is rewritten."""
    lines = [f"# {line}".rstrip() for line in header.splitlines()] if header.strip() else []
    order, keep, dropped = [k for k in ORDER if k in raw] + [k for k in raw if k not in ORDER], {}, []
    if old_text:
        _, start = split_header(old_text)
        old_order, blocks = sections(old_text, start)
        for k in old_order:
            try:
                was = yaml.safe_load("\n".join(blocks[k]) + "\n")  # a > block keeps its final newline
            except yaml.YAMLError:
                was = None
            if k in raw and isinstance(was, dict) and k in was and _same(was[k], raw[k]):
                keep[k] = blocks[k]
            else:
                dropped += [{"section": k, "text": c} for c in comments_in(blocks[k])]
        order = _order(old_order, order)
    for key in order:
        if key in keep:
            lines.extend(keep[key])
        elif key == "description" and isinstance(raw[key], str):
            _description(lines, raw[key])
        else:
            _emit(lines, key, raw[key], 0, top=True)
    while lines and not lines[-1].strip():
        lines.pop()
    text = "\n".join(lines) + "\n"
    # The writer above is for people reading the file; this guard is for the
    # file meaning what was sent. Anything it gets wrong falls back to plain
    # PyYAML output, which is ugly but exact.
    if _norm(yaml.safe_load(text) or {}) != _norm(raw):
        head = [f"# {line}".rstrip() for line in header.splitlines()] if header.strip() else []
        body = yaml.safe_dump(raw, sort_keys=False, allow_unicode=True, width=WIDTH)
        text = "\n".join(head + [body.rstrip("\n")]) + "\n"
        dropped = body_comments(old_text) if old_text else []
    return text, dropped


_MISSING = object()


def keep_types(new, old=_MISSING):
    """An int where the file had an equal float, e.g. `gte: 1.0`: JSON through
    a browser turns 1.0 into 1, and the file shouldn't change where nobody did."""
    if isinstance(new, dict) and isinstance(old, dict):
        return {k: keep_types(v, old.get(k, _MISSING)) for k, v in new.items()}
    if isinstance(new, list) and isinstance(old, list) and len(new) == len(old):
        return [keep_types(a, b) for a, b in zip(new, old)]
    if type(new) is int and type(old) is float and new == old:
        return old
    return new


def _target(name: str) -> Path:
    p = config.path_for(name)  # NAME_RX: no `/`, no `..`
    root = config.SCENARIOS_DIR.resolve()
    if p.is_symlink() or p.resolve().parent != root:
        raise EditorError(400, f"{name}.yaml isn't a plain file in fleetsim/scenarios/")
    return p


def save(body: dict) -> dict:
    """Validate, then write fleetsim/scenarios/<name>.yaml and nothing else."""
    header = body.get("header") or ""
    if not isinstance(header, str) or len(header) > MAX_TEXT or "\x00" in header:
        raise EditorError(400, f"header: text up to {MAX_TEXT} characters")
    header = header.replace("\r", "")
    res = check(body.get("scenario"), grid=False)
    if not res["ok"]:
        err = res["error"] or "; ".join(f"{p['where']}: {p['error']}" for p in res["problems"]
                                         if p["level"] == "error")
        raise EditorError(400, err, res["where"] or next(
            (p["where"] for p in res["problems"] if p["level"] == "error"), None))
    raw = untrusted(body.get("scenario"))
    name = raw["name"]
    p = _target(name)
    old, old_text = _MISSING, None
    existed = p.exists()
    if existed:
        current = old_text = p.read_text()
        if body.get("overwrite") is not True:
            raise EditorError(409, f"{name}.yaml already exists", "name")
        if body.get("base_sha") and body["base_sha"] != sha(current):
            raise EditorError(409, f"{name}.yaml changed on disk since it was opened here; "
                                   "reopen it to see the change", "name")
        try:
            old = yaml.safe_load(current)
        except yaml.YAMLError:
            old = _MISSING
    elif isinstance(body.get("source"), str) and config.NAME_RX.match(body["source"]):
        # "New from": the source file's untouched sections (and their comments) carry over.
        try:
            old_text = config.path_for(body["source"]).read_text()
            old = yaml.safe_load(old_text)
        except (OSError, yaml.YAMLError):
            old, old_text = _MISSING, None
    text, dropped = dump(keep_types(raw, old), header, old_text)
    try:
        config.resolve(yaml.safe_load(text))  # what fleetsim will read back, before it's on disk
    except (ScenarioError, yaml.YAMLError) as e:
        raise EditorError(500, f"the file as written wouldn't load ({e}); nothing was saved") from None
    tmp = p.with_name(f".{p.name}.tmp")
    try:
        tmp.write_text(text)
        os.replace(tmp, p)  # atomic: a reader never sees half a file
    finally:
        tmp.unlink(missing_ok=True)
    return {"ok": True, "name": name, "sha": sha(text), "created": not existed,
            "file": f"fleetsim/scenarios/{name}.yaml", "dropped": dropped}
