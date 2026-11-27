// SortBots operator view — the fleet sim scenario editor: one fleetsim/scenarios/*.yaml
// on its own floor, with a form beside it. Opened from the Session panel's Tests tab
// ("Fleet sim scenarios": Edit, New from). It covers everything below the header until
// closed; nothing in it is the live fleet.
//
// Talks ONLY to webui/serve.py (webui/fleetsim_editor.py does the work):
//   GET  /api/fleetsim/editor                what there is: scenarios, maps, worlds, metrics, base.yaml
//   GET  /api/fleetsim/editor/scenario?name  one file: its mapping, header comment, comments, sha
//   POST /api/fleetsim/editor/check          {scenario, grid}: config.resolve errors, the floor's
//                                            problems (unknown waypoints, blockages off the map...),
//                                            and the floor itself, built by the sim's own code
//   POST /api/fleetsim/editor/save           --control only, like launching a run
//
// The form edits a copy of the scenario mapping (`st`) and every change is checked by
// the server a moment later, so what the page says is valid is what a run will load.
// Settings the form has no field for (config overrides, synthetic options, keys added
// to the schema later) are key/value rows whose values are typed as YAML: they are
// sent as {"$yaml": text} and parsed by the server the way `--set key=value` is, so
// `false`, `[robot_0]` and `0.5` mean here what they mean in a terminal. A row nobody
// touched is sent as the value it was read as, so an untouched file saves unchanged.
//
// Coordinates: world metres, y up; the floor grid is ROS order (row 0 at origin y).

(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const root = $("#fleetedit");
  if (!root) return;
  const cv = $("#fe-map"), ctx = cv.getContext("2d");
  const tk = $("#fe-ticks"), tctx = tk.getContext("2d");
  const form = $("#fe-form"), slider = $("#fe-slider");

  const CHECK_MS = 450;             // a check this long after the last edit
  const MARKER = "$yaml";
  const KNOWN_TOP = new Set(["name", "title", "description", "seed", "duration_s", "robots", "config", "tasks",
    "task_arrival", "disturbances", "actors", "stop", "metrics"]);
  const WORLD_KEYS = new Set(["source", "map", "cell_m", "gateway", "actors", "synthetic"]);
  const ARRIVAL_KEYS = ["rate_hz", "pickups", "dropoffs", "start_s", "until_s", "count"];
  const KIND_COL = { blockage: "--stop", partition: "--info", block_link: "--info", kill: "--warnink", revive: "--go" };
  const KIND_TXT = { blockage: "Blockage", partition: "Partition", block_link: "Cut a link", kill: "Switch off", revive: "Switch on" };

  let cat = null;                   // GET /api/fleetsim/editor
  let st = null;                    // the scenario as the form holds it (fromDoc)
  let file = null;                  // {name, sha, doc, comments} of the file opened; null for a new one
  let source = null;                // "New from": the scenario this one started as
  let world = null, worldKey = "", floorCv = null;
  let result = null;                // the last check
  let unsaved = false, checking = false, checkTimer = 0, checkSeq = 0;
  let view = null, T = 0, tool = "pan";
  let armed = null;                 // {path, el}: the pickup/dropoff field a waypoint click fills
  let pending = null;               // Tasks tool: the pickup picked, waiting for its dropoff
  let draft = null;                 // a rectangle being dragged / a path being clicked
  let redraw = null;                // {kind: "dist"|"actor", i}: what the draft replaces
  let sel = null;                   // {kind: "task"|"dist"|"actor", i}: highlighted on map and form
  let hover = null, drag = null, tlDrag = null, lanes = [];

  const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  const words = (s) => String(s || "").replace(/_/g, " ");
  const titleOf = (name) => { const w = words(name); return w.charAt(0).toUpperCase() + w.slice(1); };
  const r1 = (v) => Math.round(v * 10) / 10;
  const clone = (v) => (v === undefined ? undefined : JSON.parse(JSON.stringify(v)));
  const isObj = (v) => v !== null && typeof v === "object" && !Array.isArray(v);
  // Key order doesn't matter to YAML or to the server, so compare without it.
  const canon = (v) => JSON.stringify(v, (k, x) => (isObj(x) ? Object.fromEntries(Object.keys(x).sort().map((q) => [q, x[q]])) : x));

  // -- YAML text for a value, so a row shows what the file says ------------------------
  // Only for display: the server parses what comes back. Strings that YAML would read
  // as something else (yes, 010, 1.0, a glob) are JSON-quoted, which YAML reads as-is.
  const PLAIN = /^[A-Za-z_][A-Za-z0-9_.-]*$/, RESERVED = /^(y|n|yes|no|on|off|true|false|null)$/i;
  function yamlText(v) {
    if (v === null || v === undefined) return "null";
    if (typeof v === "boolean") return String(v);
    if (typeof v === "number") { const s = String(v); return /e/.test(s) && !/\./.test(s) ? s.replace("e", ".0e") : s; }
    if (typeof v === "string") return PLAIN.test(v) && !RESERVED.test(v) ? v : JSON.stringify(v);
    if (Array.isArray(v)) return `[${v.map(yamlText).join(", ")}]`;
    return `{${Object.entries(v).map(([k, x]) => `${yamlText(k)}: ${yamlText(x)}`).join(", ")}}`;
  }
  // A key/value row. Untouched, it sends the value it was read as (types and all).
  const row = (key, v) => ({ key, text: v === undefined ? "" : yamlText(v), text0: v === undefined ? null : yamlText(v), orig: v });
  const rowValue = (r) => (r.text === r.text0 ? clone(r.orig) : { [MARKER]: r.text });

  function getPath(o, path) { for (const k of path.split(".")) { if (o == null) return undefined; o = o[k]; } return o; }
  function setPath(o, path, v) {
    const ks = path.split(".");
    for (const k of ks.slice(0, -1)) o = o[k];
    o[ks[ks.length - 1]] = v;
  }

  // -- the scenario <-> the form's state --------------------------------------------------

  function flatten(obj, prefix, out) {
    for (const [k, v] of Object.entries(obj || {})) {
      const key = prefix ? `${prefix}.${k}` : k;
      if (isObj(v) && Object.keys(v).length) flatten(v, key, out); else out.push(row(key, v));
    }
    return out;
  }
  const nodeText = (g) => (g === "rest" ? "rest" : Array.isArray(g) ? g.join(", ") : yamlText(g));

  function fromDoc(d) {
    const cfg = clone(d.config) || {};
    const w = isObj(cfg.world) ? cfg.world : {};
    delete cfg.world;
    const restWorld = Object.fromEntries(Object.entries(w).filter(([k]) => !WORLD_KEYS.has(k)));
    const stop = isObj(d.stop) ? d.stop : {};
    const s = {
      name: d.name ?? "", title: d.title ?? "", description: d.description ?? "",
      seed: d.seed ?? null, robots: d.robots ?? null, duration_s: d.duration_s ?? null,
      stop: { until: stop.until ?? "", settle_s: stop.settle_s ?? null,
        extra: Object.fromEntries(Object.entries(stop).filter(([k]) => k !== "until" && k !== "settle_s")) },
      world: { source: w.source || "synthetic", src0: "source" in w, map: w.map ?? "", cell_m: w.cell_m ?? null,
        gw: Array.isArray(w.gateway) ? w.gateway.join(", ") : "", gw0: w.gateway, actors: w.actors ?? "",
        synth: flatten(w.synthetic, "", []), synth0: "synthetic" in w },
      cfg: flatten(Object.keys(restWorld).length ? { world: restWorld } : {}, "", []).concat(flatten(cfg, "", [])),
      tasks: (Array.isArray(d.tasks) ? d.tasks : []).map((t) => {
        const { at_s, pickup, dropoff, ...extra } = isObj(t) ? t : {};
        return { at_s: at_s ?? null, pickup: pickup ?? "", dropoff: dropoff ?? "", extra };
      }),
      arrival: null,
      dist: (Array.isArray(d.disturbances) ? d.disturbances : []).map(distFrom),
      actors: (Array.isArray(d.actors) ? d.actors : []).map(actorFrom),
      metrics: (Array.isArray(d.metrics) ? d.metrics : []).map(metricFrom),
      other: Object.entries(d).filter(([k]) => !KNOWN_TOP.has(k)).map(([k, v]) => row(k, v)),
      header: "",
    };
    if (isObj(d.task_arrival)) {
      const a = d.task_arrival;
      s.arrival = Object.fromEntries(ARRIVAL_KEYS.map((k) => [k, a[k] ?? (k === "pickups" || k === "dropoffs" ? "" : null)]));
      s.arrival.rows = Object.entries(a).filter(([k]) => !ARRIVAL_KEYS.includes(k)).map(([k, v]) => row(k, v));
    }
    return s;
  }
  function distFrom(x) {
    x = isObj(x) ? x : {};
    const kinds = (cat?.disturbance_kinds || []).filter((k) => k in x);
    const kind = kinds[0] || Object.keys(x).find((k) => !["at_s", "duration_s", "fresh"].includes(k)) || "blockage";
    let v = x[kind];
    if (kind === "blockage") { const { rect, ...extra } = isObj(v) ? v : {}; v = { rect: Array.isArray(rect) ? rect.slice(0, 4) : [null, null, null, null], extra }; }
    else if (kind === "partition") v = Array.isArray(v) ? v.map(nodeText) : ["robot_0", "rest"];
    else if (kind === "block_link") v = Array.isArray(v) ? v.slice(0, 2) : ["robot_0", "gateway"];
    else if (kind === "kill" || kind === "revive") v = typeof v === "string" ? v : "robot_0";
    else v = row(kind, v === null ? undefined : v);
    const skip = new Set(["at_s", "duration_s", "fresh", kind]);
    return { kind, at_s: x.at_s ?? null, duration_s: x.duration_s ?? null, v, fresh: x.fresh === true,
      rows: Object.entries(x).filter(([k]) => !skip.has(k)).map(([k, y]) => row(k, y)) };
  }
  function actorFrom(a) {
    a = isObj(a) ? a : {};
    const p = isObj(a.path) ? a.path : {};
    const own = new Set(["id", "shape", "radius", "size", "height", "path"]);
    const pown = new Set(["waypoints", "speed_mps", "mode", "dwell_s", "start_at_s"]);
    return { id: a.id ?? "", shape: a.shape ?? "", radius: a.radius ?? null, size: row("size", a.size), height: a.height ?? null,
      wps: Array.isArray(p.waypoints) ? clone(p.waypoints) : [], speed_mps: p.speed_mps ?? null, mode: p.mode ?? "",
      dwell_s: p.dwell_s ?? null, start_at_s: p.start_at_s ?? null,
      rows: Object.entries(a).filter(([k]) => !own.has(k)).map(([k, v]) => row(k, v)),
      prows: Object.entries(p).filter(([k]) => !pown.has(k)).map(([k, v]) => row(k, v)) };
  }
  function metricFrom(m) {
    m = isObj(m) ? m : {};
    const pass = [];
    for (const [value, cond] of Object.entries(isObj(m.pass) ? m.pass : {}))
      for (const [op, thr] of Object.entries(isObj(cond) ? cond : {})) pass.push({ value, op, t: row("t", thr) });
    return { metric: m.metric ?? "", id: m.id ?? "", params: row("params", m.params), pass };
  }

  function parseGroup(text) {
    if (text.trim() === "rest") return "rest";
    const out = [];
    for (const tok of text.split(/[\s,]+/).filter(Boolean)) {
      const range = /^(?:robot_|r)?(\d+)\s*(?:-|\.\.)\s*(?:robot_|r)?(\d+)$/i.exec(tok);
      const one = /^(?:robot_|r)?(\d+)$/i.exec(tok);
      if (/^(gateway|gw)$/i.test(tok)) out.push("gateway");
      else if (range) { for (let k = Number(range[1]); k <= Number(range[2]) && out.length < 2000; k++) out.push(`robot_${k}`); }
      else if (one) out.push(`robot_${Number(one[1])}`);
      else out.push(tok); // the server says what's wrong with it
    }
    return out;
  }
  const num = (v) => (v === null || v === "" || !Number.isFinite(Number(v)) ? null : Number(v));
  const put = (o, k, v) => { if (v !== null && v !== undefined && v !== "") o[k] = v; };
  const putRows = (o, rows) => { for (const r of rows) if (r.key.trim()) o[r.key.trim()] = rowValue(r); };

  function toDoc() {
    const s = st, d = {};
    d.name = s.name.trim();
    put(d, "title", s.title);
    put(d, "description", s.description);
    put(d, "seed", s.seed);
    put(d, "duration_s", s.duration_s);
    put(d, "robots", s.robots);
    const cfg = {}, w = {};
    if (s.world.src0 || s.world.source !== "synthetic") w.source = s.world.source;
    if (s.world.source === "map") put(w, "map", s.world.map || null);
    put(w, "cell_m", s.world.cell_m);
    const gw = s.world.gw.split(/[\s,]+/).filter(Boolean).map(Number);
    if (gw.length === 2 && gw.every(Number.isFinite)) w.gateway = canon(gw) === canon(s.world.gw0) ? clone(s.world.gw0) : gw;
    else if (s.world.gw.trim()) w.gateway = { [MARKER]: s.world.gw };
    put(w, "actors", s.world.actors);
    if (s.world.synth.some((r) => r.key.trim()) || s.world.synth0) { w.synthetic = {}; putRows(w.synthetic, s.world.synth); }
    if (Object.keys(w).length) cfg.world = w;
    for (const r of s.cfg) {
      r.err = "";
      const key = r.key.trim();
      if (!key) continue;
      const ks = key.split(".");
      let o = cfg;
      for (const k of ks.slice(0, -1)) {
        if (o[k] === undefined) o[k] = {};
        if (!isObj(o[k]) || MARKER in o[k]) { r.err = `conflicts with config.${ks.slice(0, ks.indexOf(k) + 1).join(".")}`; o = null; break; }
        o = o[k];
      }
      if (o) o[ks[ks.length - 1]] = rowValue(r);
    }
    if (Object.keys(cfg).length) d.config = cfg;
    if (s.tasks.length) d.tasks = s.tasks.map((t) => { const o = {}; put(o, "at_s", t.at_s); put(o, "pickup", t.pickup); put(o, "dropoff", t.dropoff); return { ...o, ...clone(t.extra) }; });
    if (s.arrival) {
      const a = {};
      for (const k of ARRIVAL_KEYS) put(a, k, s.arrival[k]);
      putRows(a, s.arrival.rows);
      d.task_arrival = a;
    }
    if (s.dist.length) d.disturbances = s.dist.map(distTo);
    if (s.actors.length) d.actors = s.actors.map(actorTo);
    const stop = { ...clone(s.stop.extra) };
    put(stop, "until", s.stop.until);
    put(stop, "settle_s", s.stop.settle_s);
    if (Object.keys(stop).length) d.stop = stop;
    if (s.metrics.length) d.metrics = s.metrics.map(metricTo);
    putRows(d, s.other);
    return d;
  }
  function distTo(x) {
    const o = {};
    put(o, "at_s", x.at_s);
    put(o, "duration_s", x.duration_s);
    if (x.kind === "blockage") o.blockage = { rect: x.v.rect.map((c) => num(c)), ...clone(x.v.extra) };
    else if (x.kind === "partition") o.partition = x.v.map(parseGroup);
    else if (x.kind === "block_link" || x.kind === "kill" || x.kind === "revive") o[x.kind] = clone(x.v);
    else o[x.kind] = rowValue(x.v);
    if (x.fresh && (x.kind === "kill" || x.kind === "revive")) o.fresh = true;
    putRows(o, x.rows);
    return o;
  }
  function actorTo(a) {
    const o = {};
    put(o, "id", a.id);
    put(o, "shape", a.shape);
    put(o, "radius", a.radius);
    if (a.size.text.trim()) o.size = rowValue(a.size);
    put(o, "height", a.height);
    putRows(o, a.rows);
    const p = { waypoints: clone(a.wps) };
    put(p, "speed_mps", a.speed_mps);
    put(p, "mode", a.mode);
    put(p, "dwell_s", a.dwell_s);
    put(p, "start_at_s", a.start_at_s);
    putRows(p, a.prows);
    o.path = p;
    return o;
  }
  function metricTo(m) {
    const o = { metric: m.metric };
    put(o, "id", m.id);
    if (m.params.text.trim()) o.params = rowValue(m.params);
    const pass = {};
    for (const c of m.pass) if (c.value) (pass[c.value] = pass[c.value] || {})[c.op] = rowValue(c.t);
    if (Object.keys(pass).length) o.pass = pass;
    return o;
  }

  // What the floor depends on: a change here asks the server for a new grid.
  const worldKeyOf = (d) => canon([d.config?.world, d.robots, d.config?.robots, d.seed]);
  const robotsN = () => st.robots ?? num(rowValue(st.cfg.find((r) => r.key === "robots.count") || row("x", null))) ?? cat?.base?.robots?.count ?? 8;
  const duration = () => (st.duration_s > 0 ? st.duration_s : 600);
  const nodeOptions = () => ["gateway", ...Array.from({ length: Math.min(robotsN(), 1000) }, (_, k) => `robot_${k}`)];

  // -- the server ---------------------------------------------------------------------

  async function api(url, payload) {
    const res = await fetch(url, payload === undefined ? { cache: "no-store" }
      : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) { const e = new Error(body.error || res.statusText); e.status = res.status; e.where = body.where; throw e; }
    return body;
  }
  async function loadCatalog() { cat = await api("/api/fleetsim/editor"); return cat; }

  function changed() {
    unsaved = true;
    checking = true; // from the edit on: the last check's verdict is about the scenario before it
    clearTimeout(checkTimer);
    checkTimer = setTimeout(runCheck, CHECK_MS);
    paintHead();
    paintComments();
    kick();
  }
  async function runCheck() {
    clearTimeout(checkTimer);
    if (!st) return null;
    const doc = toDoc(), wk = worldKeyOf(doc), seq = ++checkSeq;
    checking = true; paintHead();
    let res;
    try {
      res = await api("/api/fleetsim/editor/check", { scenario: doc, grid: wk !== worldKey || !world });
    } catch (e) {
      res = { ok: false, error: `couldn't check it: ${e.message}`, where: null, problems: [] };
    }
    if (seq !== checkSeq || !st) return res; // a newer edit's check is on its way
    checking = false;
    result = res;
    if (res.world) {
      if (res.world.grid) { world = res.world; worldKey = wk; await renderFloor(); if (!view) fit(); }
      else world = { ...res.world, grid: world?.grid };
      fillWaypointList();
      paintWorldFacts();
    }
    paintResult();
    paintHead();
    kick();
    return res;
  }

  // -- opening and closing --------------------------------------------------------------

  async function open(name, { asNew = false } = {}) {
    if (st && unsaved && !confirm(`Discard the unsaved changes to ${st.name}?`)) return false;
    closeOthers();
    st = null; file = null; unsaved = false; checkSeq++;
    reset();
    root.hidden = false;
    placeRoot();
    setMsg("Loading…");
    try {
      if (!cat) await loadCatalog();
      const f = await api(`/api/fleetsim/editor/scenario?name=${encodeURIComponent(name)}`);
      st = fromDoc(f.scenario);
      st.header = f.header || "";
      file = asNew ? null : { name: f.name, sha: f.sha, doc: f.scenario, comments: f.comments || [] };
      source = asNew ? f.name : null;
      if (asNew) { st.name = freeName(f.name); st.title = st.title ? `${st.title} (copy)` : ""; }
      reset();
      setMsg(null);
      renderForm();
      unsaved = asNew;
      paintHead();
      await runCheck();
      return true;
    } catch (e) {
      setMsg(`Couldn't open ${name}: ${e.message}`, true);
      return false;
    }
  }
  function freeName(base) {
    const taken = new Set((cat?.scenarios || []).map((s) => s.name));
    for (let k = 2; k < 1000; k++) { const n = `${base.slice(0, 60)}_${k}`; if (!taken.has(n)) return n; }
    return `${base}_new`;
  }
  function reset() {
    world = null; worldKey = ""; floorCv = null; result = null; view = null; T = 0;
    armed = null; pending = null; draft = null; redraw = null; sel = null; hover = null;
    setTool("pan");
    slider.value = "0";
    $("#fe-as").hidden = true;
    $("#fe-err").textContent = "";
  }
  function close(force) {
    if (!force && st && unsaved && !confirm(`Close without saving ${st.name}?`)) return false;
    root.hidden = true;
    st = null; file = null; unsaved = false;
    clearTimeout(checkTimer); checkSeq++;
    return true;
  }
  function closeOthers() {
    if (!$("#session").hidden) $("#sp-x")?.click(); // the floor is where a scenario is edited
    if (window.sortbotsReplay?.state().open) window.sortbotsReplay.close();
  }
  // Under the header, whatever height it wrapped to (two rows on a phone).
  function placeRoot() {
    const h = document.querySelector("header");
    root.style.setProperty("--fe-top", `${h ? Math.round(h.getBoundingClientRect().bottom) : 50}px`);
  }
  function setMsg(text, err = false) {
    const m = $("#fe-msg");
    m.hidden = !text; m.textContent = text || ""; m.classList.toggle("err", !!err);
  }

  // -- the form ---------------------------------------------------------------------------

  const attrs = (o) => Object.entries(o).filter(([, v]) => v !== undefined && v !== null && v !== false)
    .map(([k, v]) => (v === true ? k : `${k}="${esc(v)}"`)).join(" ");
  function inp(bind, value, o = {}) {
    const type = o.type === "num" || o.type === "int" ? "number" : "text";
    return `<input ${attrs({ type, "data-bind": bind, "data-type": o.type || "str", "data-where": o.where, value: value ?? "",
      placeholder: o.ph, step: o.step ?? (o.type === "int" ? 1 : o.type === "num" ? "any" : undefined), min: o.min, max: o.max,
      list: o.list, title: o.title, class: o.cls, "data-pick": o.pick, spellcheck: type === "text" ? "false" : undefined,
      "aria-label": o.label })}>`;
  }
  function sel_(bind, value, options, o = {}) {
    const opts = options.map((x) => (Array.isArray(x) ? x : [x, x]));
    if (value && !opts.some(([v]) => v === value)) opts.push([value, value]);
    return `<select ${attrs({ "data-bind": bind, "data-type": o.type || "str", "data-where": o.where, "data-struct": o.struct, title: o.title, "aria-label": o.label })}>${
      opts.map(([v, t]) => `<option value="${esc(v)}"${v === (value ?? "") ? " selected" : ""}>${esc(t)}</option>`).join("")}</select>`;
  }
  const field = (label, html, cls = "") => `<label class="f ${cls}">${esc(label)}${html}</label>`;
  const del = (what, title) => `<button type="button" class="fe-del" data-del="${what}" title="${esc(title)}" aria-label="${esc(title)}">&times;</button>`;
  // An error that names only a section ("task_arrival: needs rate_hz > 0") outlines it.
  const SEC_WHERE = { world: "config.world", tasks: "tasks", arrival: "task_arrival", dist: "disturbances", actors: "actors", config: "config", metrics: "metrics" };
  const sec = (key, title, n, body, open = true) =>
    `<details class="fe-sec" data-sec="${key}"${SEC_WHERE[key] ? ` data-where="${SEC_WHERE[key]}"` : ""}${open ? " open" : ""}><summary>${esc(title)}${n != null ? `<span class="n">${n}</span>` : ""}</summary><div class="fe-body">${body}</div></details>`;
  // Key/value rows. `prefix` + the key is where the server's errors about a row point;
  // the row's data-where follows the key as it is typed (onField).
  function kvRows(base, rows, prefix, o = {}) {
    return `<div class="fe-rows">${rows.map((r, i) => `<div class="fe-row kv" data-where="${esc(prefix + r.key.trim())}" data-prefix="${esc(prefix)}">
      ${inp(`${base}.${i}.key`, r.key, { cls: "mono", list: o.list, ph: o.keyPh || "key", label: "Key", where: prefix + r.key.trim() })}
      ${inp(`${base}.${i}.text`, r.text, { cls: "mono", ph: o.valPh ? o.valPh(r) : "YAML value", label: "Value", where: prefix + r.key.trim(), title: r.err || o.valTitle })}
      ${del(`${base}.${i}`, "Remove this row")}</div>`).join("")}</div>`;
  }

  function secScenario() {
    const s = st;
    return sec("scenario", "Scenario", null, `<div class="fe-grid">
      ${field("Name (the file name)", inp("name", s.name, { where: "name", title: "fleetsim/scenarios/NAME.yaml: lowercase letters, digits, - and _" }), "w3")}
      ${field("Title", inp("title", s.title, { where: "title", ph: titleOf(s.name) }), "w3")}
      ${field("Description", `<textarea data-bind="description" data-where="description" rows="3">${esc(s.description.replace(/\n$/, ""))}</textarea>`, "w3")}
      ${field("Robots", inp("robots", s.robots, { type: "int", where: "robots", min: 1, ph: String(cat?.base?.robots?.count ?? 8) }))}
      ${field("Duration (sim s)", inp("duration_s", s.duration_s, { type: "num", where: "duration_s", min: 1, ph: "600" }))}
      ${field("Seed", inp("seed", s.seed, { type: "int", where: "seed", ph: "1", min: 0 }))}
      ${field("Stop the run", sel_("stop.until", s.stop.until, [["", "at the end of the duration"], ["tasks_done", "when every task is done"]], { where: "stop.until" }), "w2")}
      ${s.stop.until ? field("Then wait (s)", inp("stop.settle_s", s.stop.settle_s, { type: "num", where: "stop.settle_s", ph: "0", min: 0 })) : ""}
    </div>`);
  }
  function secWorld() {
    const w = st.world, maps = cat?.maps || [], defs = world?.synthetic_defaults || {};
    const synth = w.source === "synthetic" ? `<div class="fe-note">Synthetic warehouse options (empty value: the default)</div>
      ${kvRows("world.synth", w.synth, "config.world.synthetic.", { list: "fe-syn-keys", keyPh: "option",
        valPh: (r) => (r.key.trim() in defs ? `default ${yamlText(defs[r.key.trim()])}` : "YAML value") })}
      <div class="fe-acts"><button type="button" class="btn small" data-add="synth">Add an option</button></div>` : "";
    return sec("world", "World", null, `<div class="fe-grid">
      ${field("Floor", sel_("world.source", w.source, [["synthetic", "Synthetic warehouse"], ["map", "Saved map"]], { where: "config.world.source", struct: "world" }), "w2")}
      ${field("Cell (m)", inp("world.cell_m", w.cell_m, { type: "num", where: "config.world.cell_m", ph: String(cat?.base?.world?.cell_m ?? 0.2), step: 0.05, min: 0.05, max: 0.5 }))}
      ${w.source === "map" ? field("Map (the maps library)", sel_("world.map", w.map, [["", maps.length ? "pick a map" : "no maps in maps/"], ...maps.map((m) => [m.name, m.title && m.title !== m.name ? `${m.name} · ${m.title}` : m.name])], { where: "config.world.map" }), "w3") : ""}
      ${field("Movers from a world", sel_("world.actors", w.actors, [["", "none"], ...(cat?.worlds || [])], { where: "config.world.actors", title: "Scripted movers from configs/worlds/NAME.yaml, instead of the Movers below" }), "w2")}
      ${field("Gateway x, y", inp("world.gw", w.gw, { where: "config.world.gateway", ph: "default", title: "Saved maps only: the synthetic warehouse puts its gateway on the apron. The Gateway tool places it on the floor" }))}
    </div>${synth}<p class="fe-note" id="fe-facts"></p>`);
  }
  function secTasks() {
    const rows = st.tasks.map((t, i) => `<div class="fe-row${sel?.kind === "task" && sel.i === i ? " sel" : ""}" data-where="tasks[${i}]" data-item="task.${i}">
      ${inp(`tasks.${i}.at_s`, t.at_s, { type: "num", where: `tasks[${i}].at_s`, min: 0, label: "At (s)" })}
      ${inp(`tasks.${i}.pickup`, t.pickup, { where: `tasks[${i}].pickup`, list: "fe-wps", pick: "1", label: "Pickup", ph: "pickup" })}
      ${inp(`tasks.${i}.dropoff`, t.dropoff, { where: `tasks[${i}].dropoff`, list: "fe-wps", pick: "1", label: "Dropoff", ph: "dropoff", title: Object.keys(t.extra).length ? `also: ${yamlText(t.extra)}` : undefined })}
      ${del(`tasks.${i}`, "Remove this task")}</div>`).join("");
    return sec("tasks", "Tasks", st.tasks.length, `${st.tasks.length ? `<div class="fe-row hd"><span>At (s)</span><span>Pickup</span><span>Dropoff</span><span></span></div>` : ""}
      <div class="fe-rows">${rows}</div>
      <div class="fe-acts"><button type="button" class="btn small" data-add="task">Add a task</button>
        <button type="button" class="btn small" data-tool-btn="task" title="Click a pickup, then a dropoff, on the floor">Add on the floor</button>
        ${st.tasks.length > 1 ? `<button type="button" class="btn small" data-sort="tasks">Sort by time</button>` : ""}</div>
      <p class="fe-note">Select a pickup or dropoff field, then click a waypoint on the floor.</p>`);
  }
  function secArrival() {
    const a = st.arrival;
    const body = a ? `<div class="fe-grid">
      ${field("Rate (tasks/s)", inp("arrival.rate_hz", a.rate_hz, { type: "num", where: "task_arrival.rate_hz", min: 0, ph: "0.1" }))}
      ${field("From (s)", inp("arrival.start_s", a.start_s, { type: "num", where: "task_arrival.start_s", min: 0, ph: "0" }))}
      ${field("Until (s)", inp("arrival.until_s", a.until_s, { type: "num", where: "task_arrival.until_s", min: 0, ph: "the end" }))}
      ${field("Pickups (pattern)", inp("arrival.pickups", a.pickups, { cls: "mono", where: "task_arrival.pickups", ph: "*", title: "fnmatch over the waypoint names: shelf_*, shelf_[1-3]_*" }))}
      ${field("Dropoffs (pattern)", inp("arrival.dropoffs", a.dropoffs, { cls: "mono", where: "task_arrival.dropoffs", ph: "*" }))}
      ${field("At most", inp("arrival.count", a.count, { type: "int", where: "task_arrival.count", min: 0, ph: "no cap" }))}
    </div><p class="fe-note" id="fe-arr-note"></p>
    ${kvRows("arrival.rows", a.rows, "task_arrival.", { keyPh: "other key" })}
    <div class="fe-acts"><button type="button" class="btn small" data-add="arrival.rows">Add a key</button>
      <button type="button" class="btn small" data-arrival="off">Remove the stream</button></div>`
      : `<p class="fe-note">No random task stream: only the tasks above.</p><div class="fe-acts"><button type="button" class="btn small" data-arrival="on">Add a Poisson stream</button></div>`;
    return sec("arrival", "Task stream", a ? "on" : "off", body, !!a);
  }
  function distBody(x, i) {
    const b = `dist.${i}.v`, w = `disturbances[${i}]`;
    if (x.kind === "blockage") {
      const rect = x.v.rect;
      return `<div class="fe-line">Rectangle x0, y0, x1, y1 (m)
        ${[0, 1, 2, 3].map((k) => inp(`${b}.rect.${k}`, rect[k], { type: "num", where: `${w}.blockage`, step: 0.1, label: ["x0", "y0", "x1", "y1"][k] })).join("")}
        <button type="button" class="btn small" data-draw="dist.${i}" title="Drag a new rectangle on the floor">Draw</button></div>`;
    }
    if (x.kind === "kill" || x.kind === "revive")
      return `<div class="fe-line">${sel_(b, x.v, nodeOptions(), { where: `${w}.${x.kind}`, label: "Robot" })}
        <label class="fe-line"><input type="checkbox" data-bind="dist.${i}.fresh" data-type="bool"${x.fresh ? " checked" : ""}> ${x.kind === "kill" ? "comes back rebooted" : "rebooted"} (fresh)</label></div>`;
    if (x.kind === "block_link")
      return `<div class="fe-line">between ${sel_(`${b}.0`, x.v[0], nodeOptions(), { where: `${w}.block_link[0]`, label: "From" })} and ${sel_(`${b}.1`, x.v[1], nodeOptions(), { where: `${w}.block_link[1]`, label: "To" })}</div>`;
    if (x.kind === "partition")
      return `<div class="fe-rows">${x.v.map((g, j) => `<div class="fe-row kv" style="grid-template-columns: 4.5em minmax(0, 1fr) 24px">
        <span class="fe-note" style="margin:0">group ${j + 1}</span>${inp(`${b}.${j}`, g, { cls: "mono", where: `${w}.partition[${j}]`, ph: "robot_0, robot_1  or  0-3  or  rest", title: "Robots (robot_N, N, or a range N-M), gateway; or rest: everyone not in another group" })}
        ${x.v.length > 2 ? del(`${b}.${j}`, "Remove this group") : "<span></span>"}</div>`).join("")}</div>
        <div class="fe-acts"><button type="button" class="btn small" data-add="group.${i}">Add a group</button></div>`;
    return `<div class="fe-line">${esc(x.kind)} ${inp(`${b}.text`, x.v.text, { cls: "mono grow", where: `${w}.${x.kind}`, ph: "YAML value", label: "Value" })}</div>`;
  }
  function secDist() {
    const kinds = cat?.disturbance_kinds || Object.keys(KIND_TXT);
    const items = st.dist.map((x, i) => `<div class="fe-item${sel?.kind === "dist" && sel.i === i ? " sel" : ""}" data-where="disturbances[${i}]" data-item="dist.${i}" style="border-left: 3px solid var(${KIND_COL[x.kind] || "--muted"})">
      <div class="fe-item-h">${sel_(`dist.${i}.kind`, x.kind, kinds.map((k) => [k, KIND_TXT[k] || k]), { struct: "dist", where: `disturbances[${i}]`, label: "Kind" })}
        <span class="fe-line grow">at ${inp(`dist.${i}.at_s`, x.at_s, { type: "num", where: `disturbances[${i}].at_s`, min: 0, label: "At (s)" })} s, for
        ${inp(`dist.${i}.duration_s`, x.duration_s, { type: "num", where: `disturbances[${i}].duration_s`, min: 0, ph: "ever", label: "For (s)", title: "Seconds until it ends; empty: until the end of the run" })} s</span>
        ${del(`dist.${i}`, "Remove this disturbance")}</div>
      ${distBody(x, i)}
      ${x.rows.length ? kvRows(`dist.${i}.rows`, x.rows, `disturbances[${i}].`, { keyPh: "other key" }) : ""}</div>`).join("");
    return sec("dist", "Disturbances", st.dist.length, `<div class="fe-items">${items}</div>
      <div class="fe-acts">${sel_("_newkind", "blockage", kinds.map((k) => [k, KIND_TXT[k] || k]), { label: "Kind to add" })}
        <button type="button" class="btn small" data-add="dist">Add at ${clock(T)}</button>
        <button type="button" class="btn small" data-tool-btn="block" title="Drag a rectangle on the floor">Draw a blockage</button></div>
      <p class="fe-note">A disturbance starts at the timeline's time. Drag its bar on the timeline to move it.</p>`);
  }
  function secActors() {
    const items = st.actors.map((a, i) => `<div class="fe-item${sel?.kind === "actor" && sel.i === i ? " sel" : ""}" data-where="actors[${i}]" data-item="actor.${i}" style="border-left: 3px solid var(--warnink)">
      <div class="fe-item-h">${inp(`actors.${i}.id`, a.id, { cls: "mono", where: `actors[${i}].id`, label: "Id", ph: "id" })}
        ${sel_(`actors.${i}.shape`, a.shape, [["", "shape"], "cylinder", "box", "usd"], { where: `actors[${i}].shape`, label: "Shape" })}
        ${del(`actors.${i}`, "Remove this mover")}</div>
      <div class="fe-line">${a.shape === "box" ? `size ${inp(`actors.${i}.size.text`, a.size.text, { cls: "mono grow", where: `actors[${i}].size`, ph: "[x, y, z]" })}`
        : `radius ${inp(`actors.${i}.radius`, a.radius, { type: "num", where: `actors[${i}].radius`, step: 0.05, min: 0 })}`}
        height ${inp(`actors.${i}.height`, a.height, { type: "num", where: `actors[${i}].height`, step: 0.05, min: 0 })}
        walks ${sel_(`actors.${i}.mode`, a.mode, [["", "pingpong"], "pingpong", "loop", "once"], { where: `actors[${i}].path.mode`, label: "Mode", title: "pingpong: there and back; loop: back to the first point; once: one pass" })}
        at ${inp(`actors.${i}.speed_mps`, a.speed_mps, { type: "num", where: `actors[${i}].path.speed_mps`, step: 0.1, min: 0, ph: "1.0", label: "Speed" })} m/s</div>
      <div class="fe-line" data-where="actors[${i}].path">from ${inp(`actors.${i}.start_at_s`, a.start_at_s, { type: "num", where: `actors[${i}].path.start_at_s`, min: 0, ph: "0", label: "Start (s)" })} s,
        waiting ${inp(`actors.${i}.dwell_s`, a.dwell_s, { type: "num", where: `actors[${i}].path.dwell_s`, min: 0, ph: "0", label: "Dwell (s)" })} s at each point ·
        <span class="grow">${a.wps.length} points</span>
        <button type="button" class="btn small" data-draw="actor.${i}" title="Click the path on the floor">Draw the path</button></div>
      ${a.rows.length || a.prows.length ? kvRows(`actors.${i}.rows`, a.rows, `actors[${i}].`, { keyPh: "other key" }) + kvRows(`actors.${i}.prows`, a.prows, `actors[${i}].path.`, { keyPh: "path key" }) : ""}
    </div>`).join("");
    const fromWorld = st.world.actors ? `<p class="fe-note">Movers come from <b>configs/worlds/${esc(st.world.actors)}.yaml</b> (World above), so none can be added here.</p>` : "";
    return sec("actors", "Movers", st.actors.length, `${fromWorld}<div class="fe-items">${items}</div>
      <div class="fe-acts"><button type="button" class="btn small" data-tool-btn="mover"${st.world.actors ? " disabled" : ""} title="Click its path on the floor">Add a mover</button></div>
      <p class="fe-note">Movers walk their path and never yield (simtest/world.py); the floor shows where they are at the timeline's time.</p>`, st.actors.length > 0);
  }
  function secConfig() {
    const base = cat?.base || {};
    const defOf = (k) => { const v = getPath(base, k); return v === undefined ? (cat?.free_form || []).some((f) => k.startsWith(`${f}.`)) ? "free-form block" : "not in base.yaml" : `default ${yamlText(v)}`; };
    return sec("config", "Settings", st.cfg.length, `${kvRows("cfg", st.cfg, "config.", { list: "fe-cfg-keys", keyPh: "setting, e.g. map_share.prior", valPh: (r) => (r.key.trim() ? defOf(r.key.trim()) : "YAML value") })}
      <div class="fe-acts"><button type="button" class="btn small" data-add="cfg">Add a setting</button></div>
      <p class="fe-note">Overrides of fleetsim/configs/base.yaml, by dotted name. Values are YAML, read like <code>--set key=value</code>.</p>`, st.cfg.length > 0);
  }
  function secMetrics() {
    const ms = cat?.metrics || [], ops = cat?.ops || ["eq", "ne", "gt", "gte", "lt", "lte"];
    const items = st.metrics.map((m, i) => {
      const def = ms.find((x) => x.name === m.metric);
      const values = def ? Object.keys(def.values) : [];
      const crits = m.pass.map((c, j) => `<div class="fe-row crit" data-where="metrics[${i}].pass.${esc(c.value)}">
        ${sel_(`metrics.${i}.pass.${j}.value`, c.value, [["", "value"], ...values.map((v) => [v, v])], { where: `metrics[${i}].pass.${c.value}`, label: "Value", title: def && c.value ? def.values[c.value] : undefined })}
        ${sel_(`metrics.${i}.pass.${j}.op`, c.op, ops, { label: "Op" })}
        ${inp(`metrics.${i}.pass.${j}.t.text`, c.t.text, { cls: "mono", where: `metrics[${i}].pass.${c.value}`, ph: "threshold", label: "Threshold" })}
        ${del(`metrics.${i}.pass.${j}`, "Remove this criterion")}</div>`).join("");
      return `<div class="fe-item" data-where="metrics[${i}]" data-item="metric.${i}">
        <div class="fe-item-h">${sel_(`metrics.${i}.metric`, m.metric, [["", "metric"], ...ms.map((x) => [x.name, x.name])], { where: `metrics[${i}]`, struct: "metrics", label: "Metric", title: def?.summary })}
          ${inp(`metrics.${i}.id`, m.id, { cls: "mono", where: `metrics[${i}].id`, ph: "id (if repeated)", label: "Id" })}
          ${del(`metrics.${i}`, "Remove this metric")}</div>
        <div class="fe-line">params ${inp(`metrics.${i}.params.text`, m.params.text, { cls: "mono grow", where: `metrics[${i}].params`, ph: def && Object.keys(def.params).length ? `{${Object.keys(def.params).join(", ")}}` : "none", title: def ? Object.entries(def.params).map(([k, p]) => `${k} (${p.type}, default ${yamlText(p.default)}): ${p.doc}`).join("\n") : undefined })}</div>
        ${m.pass.length ? `<div class="fe-note" style="margin:0">Pass when</div><div class="fe-rows">${crits}</div>` : ""}
        <div class="fe-acts" style="margin-top:0"><button type="button" class="btn small" data-add="crit.${i}">Add a pass criterion</button></div></div>`;
    }).join("");
    return sec("metrics", "Metrics", st.metrics.length, `<div class="fe-items">${items}</div>
      <div class="fe-acts"><button type="button" class="btn small" data-add="metric">Add a metric</button></div>`, st.metrics.length > 0);
  }
  function secOther() {
    return sec("other", "Other keys", st.other.length || null, `${kvRows("other", st.other, "", { list: "fe-top-keys", keyPh: "top-level key" })}
      <div class="fe-acts"><button type="button" class="btn small" data-add="other">Add a key</button></div>
      <p class="fe-note">Top-level scenario keys this form has no fields for. The server says if one isn't allowed.</p>`, st.other.length > 0);
  }
  function secFile() {
    return sec("file", "File", null, `${field("Header comment (kept above the YAML)", `<textarea data-bind="header" rows="4" class="mono">${esc(st.header)}</textarea>`)}
      <p class="fe-note" id="fe-cmt-note"></p><ul class="fe-cmts" id="fe-cmts"></ul>`, false);
  }

  function datalists() {
    const syn = Object.keys(world?.synthetic_defaults || {});
    const leaf = [];
    const walk = (o, p) => { for (const [k, v] of Object.entries(o || {})) { const key = p ? `${p}.${k}` : k; if (isObj(v) && Object.keys(v).length) walk(v, key); else leaf.push(key); } };
    walk(cat?.base, "");
    const keys = leaf.filter((k) => !/^world\.(source|map|cell_m|gateway|actors|synthetic)$/.test(k));
    const top = (cat?.top_keys || []).filter((k) => !KNOWN_TOP.has(k));
    return `<datalist id="fe-wps"></datalist><datalist id="fe-syn-keys">${syn.map((k) => `<option value="${esc(k)}">`).join("")}</datalist>
      <datalist id="fe-cfg-keys">${keys.map((k) => `<option value="${esc(k)}">`).join("")}</datalist>
      <datalist id="fe-top-keys">${top.map((k) => `<option value="${esc(k)}">`).join("")}</datalist>`;
  }
  const SECTIONS = { scenario: secScenario, world: secWorld, tasks: secTasks, arrival: secArrival, dist: secDist,
    actors: secActors, config: secConfig, metrics: secMetrics, other: secOther, file: secFile };
  function renderForm() {
    form.innerHTML = Object.values(SECTIONS).map((f) => f()).join("") + datalists();
    fillWaypointList(); paintWorldFacts(); paintComments(); paintResult(); paintArrival();
    const c = cat?.control;
    $("#fe-save").disabled = !c; $("#fe-saveas").disabled = !c; $("#fe-run").disabled = !c;
    for (const id of ["#fe-save", "#fe-saveas", "#fe-run"])
      $(id).title = c ? $(id).title.replace(/ \(needs the sim console\)$/, "") : `${$(id).title.replace(/ \(needs the sim console\)$/, "")} (needs the sim console)`;
  }
  function renderSec(key) {
    const old = form.querySelector(`[data-sec="${key}"]`);
    if (!old) return;
    const wasOpen = old.open, active = document.activeElement?.dataset?.bind;
    const tmp = document.createElement("div");
    tmp.innerHTML = SECTIONS[key]();
    const fresh = tmp.firstElementChild;
    fresh.open = wasOpen || fresh.open;
    old.replaceWith(fresh);
    if (active) { const el = form.querySelector(`[data-bind="${CSS.escape(active)}"]`); if (el) el.focus({ preventScroll: true }); }
    if (key === "world") paintWorldFacts();
    if (key === "arrival") paintArrival();
    paintResult();
  }
  function openSec(key) { const d = form.querySelector(`[data-sec="${key}"]`); if (d) d.open = true; }

  function fillWaypointList() {
    const dl = form.querySelector("#fe-wps");
    if (dl && world) dl.innerHTML = Object.keys(world.waypoints).map((n) => `<option value="${esc(n)}">`).join("");
    const sk = form.querySelector("#fe-syn-keys");
    if (sk && world) sk.innerHTML = Object.keys(world.synthetic_defaults || {}).map((k) => `<option value="${esc(k)}">`).join("");
  }
  function paintWorldFacts() {
    const el = form.querySelector("#fe-facts");
    if (!el) return;
    if (!world) { el.textContent = "The floor is drawn once the scenario checks out."; return; }
    const g = world.gateway;
    el.innerHTML = `<b>${world.size_m[0]} × ${world.size_m[1]} m</b> at ${world.cell_m} m cells · ${Object.keys(world.waypoints).length} waypoints · ${world.homes.length} homes · gateway at (${g[0]}, ${g[1]})`
      + (world.notes || []).map((n) => `<br>${esc(n)}`).join("");
  }
  // fnmatch, as task_arrival matches waypoint names: * ? [seq] [!seq].
  function globRx(p) {
    let r = "^";
    for (let i = 0; i < p.length; i++) {
      const c = p[i];
      if (c === "*") r += ".*";
      else if (c === "?") r += ".";
      else if (c === "[" && p.indexOf("]", i + 2) > 0) { const j = p.indexOf("]", i + 2); let set = p.slice(i + 1, j); if (set[0] === "!") set = "^" + set.slice(1); r += `[${set.replace(/\\/g, "\\\\")}]`; i = j; }
      else r += c.replace(/[.+^${}()|\\\]]/g, "\\$&");
    }
    return new RegExp(r + "$");
  }
  function paintArrival() {
    const el = form.querySelector("#fe-arr-note");
    if (!el || !st.arrival) return;
    const a = st.arrival, names = world ? Object.keys(world.waypoints) : [];
    const n = (p) => names.filter((x) => globRx(p || "*").test(x)).length;
    const exp = result?.summary?.arrival_expected;
    el.innerHTML = world ? `Pickups match <b>${n(a.pickups)}</b> waypoints, dropoffs <b>${n(a.dropoffs)}</b>.${exp != null ? ` About <b>${exp}</b> tasks over the run.` : ""}` : "";
  }
  // Comments below the header survive a save only in sections nothing changed in.
  function paintComments() {
    const list = form.querySelector("#fe-cmts"), note = form.querySelector("#fe-cmt-note");
    if (!list || !note) return;
    const cs = file?.comments || [];
    if (!cs.length) { note.textContent = file ? "No comments below the header." : "A new file: only the header comment is written."; list.innerHTML = ""; return; }
    const doc = toDoc(), old = file.doc;
    const lost = (k) => canon(doc[k]) !== canon(old[k]);
    note.textContent = "Comments below the header are kept only in sections you leave unchanged; a changed section is rewritten without its comments. Move what matters into the header or the description.";
    list.innerHTML = cs.map((c) => `<li class="${lost(c.section) ? "lost" : ""}">${esc(c.section)}: ${esc(c.text)}${lost(c.section) ? " · <b>dropped on save</b>" : ""}</li>`).join("");
  }

  function paintHead() {
    if (!st) return;
    $("#fe-heading").textContent = `${file ? "Edit" : "New"} ${st.name || "scenario"}${unsaved ? " · unsaved" : ""}`;
    $("#fe-name").textContent = st.title || titleOf(st.name);
    const chip = $("#fe-state");
    const nErr = result ? (result.error ? 1 : 0) + (result.problems || []).filter((p) => p.level === "error").length : 0;
    let text = "…", tone = "busy";
    if (!checking && result) { text = result.ok ? "VALID" : `${nErr} ERROR${nErr === 1 ? "" : "S"}`; tone = result.ok ? "ok" : "bad"; }
    chip.textContent = checking ? "CHECKING" : text; chip.className = `fe-state ${checking ? "busy" : tone}`;
    const sm = result?.summary;
    $("#fe-sum").textContent = sm ? `${sm.robots} robots · ${sm.network} · ${sm.tasks} tasks${sm.arrival_expected ? ` + ~${Math.round(sm.arrival_expected)} streamed` : ""} · ${sm.disturbances} disturbances · ${clock(sm.duration_s)}` : "";
    slider.max = String(duration());
    $("#fe-clock").textContent = `${clock(T)} / ${clock(duration())}`;
  }

  function findWhere(where) {
    if (!where) return null;
    let best = null, bl = -1;
    for (const e of form.querySelectorAll("[data-where]")) {
      const w = e.dataset.where;
      if (w && (where === w || where.startsWith(`${w}.`) || where.startsWith(`${w}[`)) && (w.length > bl || (w.length === bl && best.contains(e)))) { best = e; bl = w.length; }
    }
    return best;
  }
  function problems() {
    if (!result) return [];
    const out = [];
    if (result.error) out.push({ where: result.where, error: result.error, level: "error" });
    for (const r of st.cfg) if (r.err) out.push({ where: `config.${r.key.trim()}`, error: r.err, level: "error" });
    return out.concat(result.problems || []);
  }
  function paintResult() {
    for (const e of form.querySelectorAll(".bad, .warned")) { e.classList.remove("bad", "warned"); if (e.dataset.tip) { e.title = e.dataset.tip === "-" ? "" : e.dataset.tip; delete e.dataset.tip; } }
    const box = $("#fe-err");
    if (!result) { box.innerHTML = ""; return; }
    const ps = problems();
    for (const p of ps) {
      const el = findWhere(p.where);
      if (!el) continue;
      if (!el.dataset.tip) el.dataset.tip = el.title || "-";
      el.classList.add(p.level === "error" ? "bad" : "warned");
      el.title = p.error;
    }
    const ctl = cat && !cat.control ? `<div class="info">Read-only: saving needs the sim console (scripts/run_console.sh). Use a text editor for fleetsim/scenarios/ meanwhile.</div>` : "";
    if (!ps.length) {
      const sm = result.summary;
      box.innerHTML = `<div class="ok">Valid${sm ? `: ${sm.robots} robots, ${sm.tasks} tasks, ${sm.disturbances} disturbances, ${sm.actors} movers, ${sm.metrics} metrics on a ${sm.floor_m[0]} × ${sm.floor_m[1]} m floor` : ""}.</div>${ctl}`;
      return;
    }
    box.innerHTML = ps.slice(0, 40).map((p, k) => `<button type="button" class="p ${p.level}" data-p="${k}" title="Show it in the form">${p.level === "warning" ? "Warning: " : ""}${p.where ? `<b>${esc(p.where)}</b>: ` : ""}${esc(p.error)}</button>`).join("")
      + (ps.length > 40 ? `<div class="info">and ${ps.length - 40} more</div>` : "") + ctl;
  }
  $("#fe-err").addEventListener("click", (e) => {
    const b = e.target.closest("[data-p]");
    if (!b) return;
    const p = problems()[Number(b.dataset.p)], el = findWhere(p?.where);
    if (!el) return;
    el.closest("details")?.setAttribute("open", "");
    el.scrollIntoView({ block: "center" });
    (el.matches("input, select, textarea") ? el : el.querySelector("input, select, textarea"))?.focus({ preventScroll: true });
  });

  // -- form events ---------------------------------------------------------------------

  function readEl(el) {
    const t = el.dataset.type || "str";
    if (t === "bool") return el.checked;
    const v = el.value;
    if (t === "num") return v.trim() === "" || !Number.isFinite(Number(v)) ? null : Number(v);
    if (t === "int") return v.trim() === "" || !Number.isFinite(Number(v)) ? null : Math.round(Number(v));
    return v;
  }
  function onField(e) {
    const el = e.target, bind = el.dataset.bind;
    if (!bind || !st || bind.startsWith("_")) return;
    // A select fires `input` and then `change`; acting on the first would leave a
    // disturbance with its new kind and its old value until the second.
    if (e.type === "input" && el.tagName === "SELECT") return;
    if (bind === "header") { st.header = el.value; changed(); return; }
    setPath(st, bind, readEl(el));
    if (bind === "name") paintHead();
    const kv = /\.key$/.test(bind) && el.closest(".fe-row.kv");
    if (kv) for (const x of [kv, ...kv.querySelectorAll("[data-where]")]) x.dataset.where = kv.dataset.prefix + el.value.trim();
    const struct = el.dataset.struct;
    if (e.type === "change" && struct === "dist") {
      const i = Number(bind.split(".")[1]), x = st.dist[i];
      st.dist[i] = distFrom({ at_s: x.at_s, duration_s: x.duration_s, [x.kind]: defaultFor(x.kind) });
    }
    if (e.type === "change" && struct) renderSec(struct);
    if (e.type === "change" && (bind === "robots" || bind === "stop.until" || /^actors\.\d+\.shape$/.test(bind)))
      renderSec(bind === "robots" ? "dist" : bind === "stop.until" ? "scenario" : "actors");
    if (bind === "world.actors" && e.type === "change") renderSec("actors");
    if (bind.startsWith("arrival.")) paintArrival();
    changed();
  }
  form.addEventListener("input", onField);
  form.addEventListener("change", onField);
  form.addEventListener("submit", (e) => e.preventDefault());

  function defaultFor(kind) {
    if (kind === "blockage") {
      const c = world ? [world.origin[0] + world.size_m[0] / 2, world.origin[1] + world.size_m[1] / 2] : [10, 10];
      return { rect: [r1(c[0] - 0.5), r1(c[1] - 0.5), r1(c[0] + 0.5), r1(c[1] + 0.5)] };
    }
    if (kind === "partition") return [["robot_0"], "rest"];
    if (kind === "block_link") return ["robot_0", "gateway"];
    if (kind === "kill" || kind === "revive") return "robot_0";
    return null;
  }
  const nextTaskAt = () => { const ts = st.tasks.map((t) => t.at_s).filter((v) => v != null); return Math.max(Math.round(T), ts.length ? Math.max(...ts) + 2 : 1); };

  form.addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b || !st) return;
    if (b.dataset.add) addItem(b.dataset.add);
    else if (b.dataset.del) delItem(b.dataset.del);
    else if (b.dataset.sort) { st.tasks.sort((a, c) => (a.at_s ?? 0) - (c.at_s ?? 0)); renderSec("tasks"); changed(); }
    else if (b.dataset.toolBtn) setTool(b.dataset.toolBtn);
    else if (b.dataset.draw) {
      const [kind, i] = b.dataset.draw.split(".");
      redraw = { kind, i: Number(i) }; sel = { kind, i: Number(i) };
      setTool(kind === "dist" ? "block" : "mover", true);
    } else if (b.dataset.arrival) {
      st.arrival = b.dataset.arrival === "on" ? { rate_hz: 0.1, pickups: "shelf_*", dropoffs: "dock_*", start_s: null, until_s: null, count: null, rows: [] } : null;
      renderSec("arrival"); changed();
    }
  });
  function addItem(what) {
    const [kind, i] = what.split(".");
    if (kind === "task") { st.tasks.push({ at_s: nextTaskAt(), pickup: "", dropoff: "", extra: {} }); renderSec("tasks"); focusBind(`tasks.${st.tasks.length - 1}.pickup`); }
    else if (kind === "dist") {
      const k = form.querySelector('[data-bind="_newkind"]')?.value || "blockage";
      st.dist.push(distFrom({ at_s: Math.round(T), [k]: defaultFor(k) })); sel = { kind: "dist", i: st.dist.length - 1 };
      renderSec("dist");
    } else if (kind === "group") { st.dist[Number(i)].v.push(""); renderSec("dist"); focusBind(`dist.${i}.v.${st.dist[Number(i)].v.length - 1}`); }
    else if (kind === "synth") { st.world.synth.push(row("", undefined)); renderSec("world"); focusBind(`world.synth.${st.world.synth.length - 1}.key`); }
    else if (kind === "cfg") { st.cfg.push(row("", undefined)); renderSec("config"); focusBind(`cfg.${st.cfg.length - 1}.key`); }
    else if (kind === "other") { st.other.push(row("", undefined)); renderSec("other"); focusBind(`other.${st.other.length - 1}.key`); }
    else if (kind === "arrival") { st.arrival.rows.push(row("", undefined)); renderSec("arrival"); focusBind(`arrival.rows.${st.arrival.rows.length - 1}.key`); }
    else if (kind === "metric") { st.metrics.push(metricFrom({})); renderSec("metrics"); focusBind(`metrics.${st.metrics.length - 1}.metric`); }
    else if (kind === "crit") { st.metrics[Number(i)].pass.push({ value: "", op: "gte", t: row("t", undefined) }); renderSec("metrics"); }
    changed();
  }
  function delItem(path) {
    const ks = path.split(".");
    const list = getPath(st, ks.slice(0, -1).join("."));
    list.splice(Number(ks[ks.length - 1]), 1);
    const top = { tasks: "tasks", dist: "dist", actors: "actors", cfg: "config", other: "other", metrics: "metrics", world: "world", arrival: "arrival" }[ks[0]];
    if (sel && ((top === "tasks" && sel.kind === "task") || (top === "dist" && sel.kind === "dist") || (top === "actors" && sel.kind === "actor"))) sel = null;
    renderSec(top);
    changed();
  }
  function focusBind(bind) { const el = form.querySelector(`[data-bind="${CSS.escape(bind)}"]`); if (el) { el.closest("details").open = true; el.focus(); } }

  // The pickup/dropoff field that has focus is the one a click on a waypoint fills.
  form.addEventListener("focusin", (e) => {
    const el = e.target;
    if (el.dataset?.pick) arm(el);
    else if (el.matches("input, select, textarea")) arm(null);
    const item = el.closest("[data-item]");
    if (item) { const [k, i] = item.dataset.item.split("."); if (["task", "dist", "actor"].includes(k)) select({ kind: k, i: Number(i) }, false); }
  });
  function arm(el) {
    form.querySelectorAll(".armed").forEach((x) => x.classList.remove("armed"));
    armed = el ? { path: el.dataset.bind } : null;
    if (el) el.classList.add("armed");
    root.classList.toggle("arming", !!armed);
    hint();
  }
  function select(s, scroll = true) {
    const same = sel && s && sel.kind === s.kind && sel.i === s.i;
    sel = s;
    form.querySelectorAll("[data-item].sel").forEach((x) => x.classList.remove("sel"));
    if (s) {
      const el = form.querySelector(`[data-item="${s.kind}.${s.i}"]`);
      if (el) { el.classList.add("sel"); if (scroll && !same) { el.closest("details").open = true; el.scrollIntoView({ block: "nearest" }); } }
    }
    kick();
  }

  // -- tools ---------------------------------------------------------------------------

  function setTool(t, keepRedraw = false) {
    tool = t; draft = null; pending = null;
    if (!keepRedraw) redraw = null;
    root.dataset.tool = t;
    for (const b of $("#fe-tools").querySelectorAll("button")) { b.classList.toggle("active", b.dataset.tool === t); b.setAttribute("aria-checked", String(b.dataset.tool === t)); }
    hint(); kick();
  }
  $("#fe-tools").addEventListener("click", (e) => { const b = e.target.closest("[data-tool]"); if (b) setTool(b.dataset.tool); });
  function hint() {
    let h = "";
    if (tool === "task") h = pending ? `Pickup ${pending}: now click the dropoff. Esc cancels.` : `Click the pickup waypoint, then the dropoff: a task at ${clock(nextTaskAt())}. Esc when done.`;
    else if (tool === "block") h = redraw ? `Drag the new rectangle for disturbance ${redraw.i + 1}. Esc cancels.` : `Drag a rectangle: a blockage at ${clock(Math.round(T))} (the timeline's time). Esc when done.`;
    else if (tool === "mover") h = `${redraw ? `New path for mover ${redraw.i + 1}` : "A new mover"}: click its points (${draft?.pts?.length || 0} so far). Enter or double-click ends it, Esc cancels.`;
    else if (tool === "gateway") h = `Click where the gateway goes.${st?.world.source !== "map" ? " The synthetic warehouse ignores this: its gateway is on the apron." : ""}`;
    else if (armed) h = `Click a waypoint to fill in ${armed.path.replace(/^tasks\.(\d+)\.(\w+)$/, (m, i, f) => `task ${Number(i) + 1}'s ${f}`)}.`;
    $("#fe-hint").textContent = h;
  }

  // -- the floor ---------------------------------------------------------------------------

  async function inflate(z) {
    const bin = Uint8Array.from(atob(z), (c) => c.charCodeAt(0));
    const stream = new Blob([bin]).stream().pipeThrough(new DecompressionStream("deflate"));
    return new Int8Array(await new Response(stream).arrayBuffer());
  }
  function rgb(hex) {
    const m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})/i.exec(hex) || [0, "80", "80", "80"];
    return [parseInt(m[1], 16), parseInt(m[2], 16), parseInt(m[3], 16)];
  }
  let cells = null;
  // Grid -> canvas, a pixel a cell, row 0 at the bottom (as the replay draws it).
  async function renderFloor() {
    if (world?.grid?.z) cells = await inflate(world.grid.z);
    if (!cells || !world?.grid) { floorCv = null; return; }
    const g = world.grid, c = document.createElement("canvas");
    c.width = g.w; c.height = g.h;
    const im = c.getContext("2d").createImageData(g.w, g.h);
    const U = rgb(css("--map-unknown")), F = rgb(css("--map-free")), O = rgb(css("--map-occ"));
    for (let r = 0; r < g.h; r++) {
      const dst = (g.h - 1 - r) * g.w;
      for (let q = 0; q < g.w; q++) {
        const v = cells[r * g.w + q], p = v < 0 ? U : v >= 50 ? O : F, o = (dst + q) * 4;
        im.data[o] = p[0]; im.data[o + 1] = p[1]; im.data[o + 2] = p[2]; im.data[o + 3] = 255;
      }
    }
    c.getContext("2d").putImageData(im, 0, 0);
    floorCv = c;
  }
  function fit() {
    if (!world) return;
    const W = cv.clientWidth, H = cv.clientHeight;
    const top = $("#fe-bar").offsetHeight + 20, bottom = $("#fe-time").offsetHeight + 20;
    const wm = world.size_m[0], hm = world.size_m[1], h = Math.max(40, H - top - bottom);
    const s = Math.min(W / wm, h / hm) * 0.96;
    view = { s, x: (W - wm * s) / 2 - world.origin[0] * s, y: top + (h + hm * s) / 2 + world.origin[1] * s };
  }
  const px = (x, y) => [view.x + x * view.s, view.y - y * view.s];
  const toWorld = (a, b) => [(a - view.x) / view.s, (view.y - b) / view.s];

  // Mover kinematics: simtest/world.py pose_at, so the floor shows them where a run has them.
  function poseAt(a, t) {
    const pts = a.wps;
    if (!pts.length) return null;
    const mode = a.mode || "pingpong", speed = a.speed_mps > 0 ? a.speed_mps : 1.0, dwell = a.dwell_s || 0;
    let seq = pts.slice();
    if (mode === "loop") seq = seq.concat([pts[0]]); else if (mode === "pingpong") seq = seq.concat(pts.slice(0, -1).reverse());
    const sched = [];
    let tt = 0;
    for (let k = 0; k + 1 < seq.length; k++) {
      const A = seq[k], B = seq[k + 1];
      if (dwell > 0) { sched.push([tt, tt + dwell, A, A]); tt += dwell; }
      const d = Math.hypot(B[0] - A[0], B[1] - A[1]) / speed;
      sched.push([tt, tt + d, A, B]); tt += d;
    }
    let rel = t - (a.start_at_s || 0);
    if (rel < 0 || tt <= 0 || !sched.length) return pts[0];
    if (mode === "once" && rel >= tt) return seq[seq.length - 1];
    if (mode !== "once") rel %= tt;
    for (const [t0, t1, A, B] of sched) if (rel <= t1) { const f = t1 > t0 ? Math.min(1, Math.max(0, (rel - t0) / (t1 - t0))) : 0; return [A[0] + f * (B[0] - A[0]), A[1] + f * (B[1] - A[1])]; }
    return seq[seq.length - 1];
  }
  const radiusOf = (a) => {
    if (a.shape === "box") { const m = String(a.size.text).match(/-?\d+(\.\d+)?/g); if (m && m.length >= 2) return Math.hypot(Number(m[0]), Number(m[1])) / 2; }
    return a.radius > 0 ? a.radius : 0.3;
  };
  const spanOf = (x) => [x.at_s ?? 0, x.duration_s != null && x.duration_s > 0 ? (x.at_s ?? 0) + x.duration_s : Infinity];
  const activeAt = (x, t) => { const [a, b] = spanOf(x); return a <= t && t < b; };

  // Who is cut off from whom at T, for the homes' colours: group index per robot.
  function groupsAt(t) {
    const n = robotsN(), out = new Array(n).fill(-1);
    for (const x of st.dist) {
      if (x.kind !== "partition" || !activeAt(x, t)) continue;
      const gs = x.v.map(parseGroup), named = new Set();
      gs.forEach((g, j) => { if (g !== "rest") for (const r of g) { const k = /^robot_(\d+)$/.exec(r); if (k && Number(k[1]) < n) { out[Number(k[1])] = j; named.add(Number(k[1])); } } });
      const rest = gs.indexOf("rest");
      if (rest >= 0) for (let k = 0; k < n; k++) if (!named.has(k)) out[k] = rest;
    }
    return out;
  }
  const deadAt = (t) => {
    const dead = new Set();
    for (const x of [...st.dist].sort((a, b) => (a.at_s ?? 0) - (b.at_s ?? 0))) {
      if (x.kind === "kill" && activeAt(x, t)) dead.add(x.v);
      if (x.kind === "revive" && (x.at_s ?? 0) <= t) dead.delete(x.v);
    }
    return dead;
  };

  let raf = 0;
  function kick() { if (!raf && !root.hidden) raf = requestAnimationFrame(() => { raf = 0; draw(); drawTicks(); }); }
  function draw() {
    const W = cv.clientWidth, H = cv.clientHeight, dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = css("--map-bg"); ctx.fillRect(0, 0, W, H);
    if (!st) return;
    if (!world || !floorCv) { setMsg(result && !result.ok && !world ? "The floor can't be built until the errors are fixed." : "Building the floor…", result && !result.ok && !world); return; }
    setMsg(null);
    if (!view) fit();
    const s = view.s, ink = css("--map-ink"), stop = css("--stop");
    const [fx, fy] = px(world.origin[0], world.origin[1] + world.size_m[1]);
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = worldKey === worldKeyOf(toDoc()) ? 1 : 0.55; // stale until the new one arrives
    ctx.drawImage(floorCv, fx, fy, world.size_m[0] * s, world.size_m[1] * s);
    ctx.globalAlpha = 1;

    // Blockages: filled while up at T, outlined otherwise; the selected one in ink.
    st.dist.forEach((x, i) => {
      if (x.kind !== "blockage") return;
      const r = x.v.rect.map(num);
      if (r.some((v) => v == null)) return;
      const [a, b] = px(Math.min(r[0], r[2]), Math.max(r[1], r[3])), [c2, d] = px(Math.max(r[0], r[2]), Math.min(r[1], r[3]));
      const on = activeAt(x, T), picked = sel?.kind === "dist" && sel.i === i;
      if (on) { ctx.fillStyle = stop; ctx.globalAlpha = 0.4; ctx.fillRect(a, b, c2 - a, d - b); ctx.globalAlpha = 1; }
      ctx.strokeStyle = picked ? ink : stop; ctx.lineWidth = picked ? 2.5 : 1.5; ctx.setLineDash(on ? [] : [4, 3]);
      ctx.strokeRect(a, b, c2 - a, d - b); ctx.setLineDash([]);
    });

    // Tasks: pickup -> dropoff, faint; the selected one bold with an arrowhead.
    const wps = world.waypoints;
    st.tasks.forEach((t, i) => {
      const A = wps[t.pickup], B = wps[t.dropoff];
      if (!A || !B) return;
      const picked = sel?.kind === "task" && sel.i === i;
      const [a, b] = px(A[0], A[1]), [c2, d] = px(B[0], B[1]);
      ctx.strokeStyle = css("--info"); ctx.globalAlpha = picked ? 1 : 0.28; ctx.lineWidth = picked ? 2.5 : 1.2;
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c2, d); ctx.stroke();
      if (picked) {
        const ang = Math.atan2(d - b, c2 - a);
        ctx.fillStyle = css("--info"); ctx.beginPath(); ctx.moveTo(c2, d);
        ctx.lineTo(c2 - 10 * Math.cos(ang - 0.4), d - 10 * Math.sin(ang - 0.4)); ctx.lineTo(c2 - 10 * Math.cos(ang + 0.4), d - 10 * Math.sin(ang + 0.4)); ctx.closePath(); ctx.fill();
      }
      ctx.globalAlpha = 1;
    });

    // Waypoints (floor tape), named once there's room.
    const wpR = Math.max(2.5, Math.min(5, 0.18 * s));
    const mark = new Set([pending, hover?.wp, ...(sel?.kind === "task" && st.tasks[sel.i] ? [st.tasks[sel.i].pickup, st.tasks[sel.i].dropoff] : [])]);
    ctx.fillStyle = css("--tape");
    for (const name in wps) {
      const [a, b] = px(wps[name][0], wps[name][1]), r = mark.has(name) ? wpR + 2.5 : wpR;
      ctx.beginPath(); ctx.moveTo(a, b - r); ctx.lineTo(a + r, b); ctx.lineTo(a, b + r); ctx.lineTo(a - r, b); ctx.closePath(); ctx.fill();
    }
    ctx.font = "10px system-ui, sans-serif"; ctx.textAlign = "left"; ctx.fillStyle = css("--muted");
    for (const name in wps) if (s > 15 || mark.has(name)) { const [a, b] = px(wps[name][0], wps[name][1]); ctx.fillText(name, a + wpR + 3, b + 3); }
    if (pending && wps[pending]) { const [a, b] = px(...wps[pending]); ctx.strokeStyle = ink; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(a, b, wpR + 6, 0, 2 * Math.PI); ctx.stroke(); }

    // Homes: where each robot parks, numbered; coloured by partition group at T, red if off.
    const groups = groupsAt(T), dead = deadAt(T), br = Math.max(3, (world.body_r || 0.25) * s);
    const GROUP = ["--info", "--go", "--warnink", "--tape", "--muted"];
    ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.font = `600 ${Math.max(8, Math.min(11, br))}px system-ui, sans-serif`;
    world.homes.forEach((h, k) => {
      const [a, b] = px(h[0], h[1]), off = dead.has(`robot_${k}`);
      ctx.fillStyle = off ? stop : groups[k] >= 0 ? css(GROUP[groups[k] % GROUP.length]) : css("--faint");
      ctx.globalAlpha = 0.8; ctx.beginPath(); ctx.arc(a, b, br, 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
      if (br >= 6) { ctx.fillStyle = css("--map-robot-ink"); ctx.fillText(String(k), a, b + 0.5); }
    });
    ctx.textBaseline = "alphabetic";

    // Gateway: a triangle; red if a kill names it at T.
    const g = world.gateway;
    if (g) {
      const [a, b] = px(g[0], g[1]);
      ctx.fillStyle = dead.has("gateway") ? stop : ink;
      ctx.beginPath(); ctx.moveTo(a, b - 8); ctx.lineTo(a + 7, b + 5); ctx.lineTo(a - 7, b + 5); ctx.closePath(); ctx.fill();
    }

    // Movers: their paths, and where they are at T. A world's movers are dashed (read-only here).
    const drawMover = (pts, pos, r, style, bold, dashed) => {
      ctx.strokeStyle = style; ctx.lineWidth = bold ? 2.5 : 1.5; ctx.setLineDash(dashed ? [5, 4] : []);
      ctx.beginPath(); pts.forEach((p, k) => { const [a, b] = px(p[0], p[1]); if (k) ctx.lineTo(a, b); else ctx.moveTo(a, b); }); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = style;
      for (const p of pts) { const [a, b] = px(p[0], p[1]); ctx.fillRect(a - 2, b - 2, 4, 4); }
      if (pos) { const [a, b] = px(pos[0], pos[1]); ctx.globalAlpha = 0.85; ctx.beginPath(); ctx.arc(a, b, Math.max(3, r * s), 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1; }
    };
    const warn = css("--warnink");
    if (st.world.actors) for (const a of world.actors || []) drawMover(a.path.waypoints, poseAt({ wps: a.path.waypoints, ...a.path }, T), a.r, warn, false, true);
    else st.actors.forEach((a, i) => { if (a.wps.length) drawMover(a.wps, poseAt(a, T), radiusOf(a), warn, sel?.kind === "actor" && sel.i === i, false); });

    // What is being drawn right now.
    if (draft?.kind === "rect") {
      const [a, b] = px(Math.min(draft.x0, draft.x1), Math.max(draft.y0, draft.y1)), [c2, d] = px(Math.max(draft.x0, draft.x1), Math.min(draft.y0, draft.y1));
      ctx.fillStyle = stop; ctx.globalAlpha = 0.25; ctx.fillRect(a, b, c2 - a, d - b); ctx.globalAlpha = 1;
      ctx.strokeStyle = ink; ctx.setLineDash([4, 3]); ctx.strokeRect(a, b, c2 - a, d - b); ctx.setLineDash([]);
    }
    if (draft?.kind === "path" && draft.pts.length) drawMover(hover?.xy ? [...draft.pts, hover.xy] : draft.pts, null, 0, ink, true, true);
  }

  // -- the timeline ------------------------------------------------------------------------

  function drawTicks() {
    const W = tk.clientWidth, dpr = window.devicePixelRatio || 1;
    const n = st ? st.dist.length : 0, laneH = n > 8 ? Math.max(3, Math.floor(64 / n)) : 7;
    const H = 12 + n * (laneH + 2) + 2;
    if (tk.style.height !== `${Math.max(30, H)}px`) tk.style.height = `${Math.max(30, H)}px`;
    const HH = tk.clientHeight;
    if (tk.width !== Math.round(W * dpr) || tk.height !== Math.round(HH * dpr)) { tk.width = Math.round(W * dpr); tk.height = Math.round(HH * dpr); }
    tctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    tctx.clearRect(0, 0, W, HH);
    lanes = [];
    if (!st) return;
    const dur = duration(), xOf = (t) => Math.max(0, Math.min(W, (t / dur) * W));
    // Along the top: the stream's window as a band, each fixed task a tick.
    if (st.arrival) { tctx.fillStyle = css("--go"); tctx.globalAlpha = 0.18; tctx.fillRect(xOf(st.arrival.start_s || 0), 0, xOf(st.arrival.until_s ?? dur) - xOf(st.arrival.start_s || 0), 10); tctx.globalAlpha = 1; }
    tctx.fillStyle = css("--go");
    for (const t of st.tasks) if (t.at_s != null) tctx.fillRect(Math.round(xOf(t.at_s)), 1, 1.5, 8);
    // A bar per disturbance, in its kind's colour; one that never ends fades out.
    st.dist.forEach((x, i) => {
      const y = 12 + i * (laneH + 2), [a, b] = spanOf(x), x0 = xOf(a), x1 = b === Infinity ? W : xOf(b);
      lanes.push({ i, y, h: laneH, x0, x1, open: b === Infinity });
      tctx.fillStyle = css(KIND_COL[x.kind] || "--muted");
      tctx.globalAlpha = b === Infinity ? 0.45 : 0.85;
      tctx.fillRect(x0, y, Math.max(3, x1 - x0), laneH);
      tctx.globalAlpha = 1;
      if (sel?.kind === "dist" && sel.i === i) { tctx.strokeStyle = css("--map-ink"); tctx.lineWidth = 1.5; tctx.strokeRect(x0 + 0.5, y + 0.5, Math.max(3, x1 - x0) - 1, laneH - 1); }
    });
    tctx.fillStyle = css("--map-ink");
    tctx.fillRect(Math.round(xOf(T)) - 0.5, 0, 1.5, HH);
  }
  function tlHit(e) {
    const r = tk.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
    for (const l of lanes) {
      if (y < l.y - 1 || y > l.y + l.h + 1) continue;
      if (!l.open && Math.abs(x - l.x1) <= 6) return { i: l.i, edge: "end" };
      if (x >= l.x0 - 3 && x <= l.x1 + 3) return { i: l.i, edge: "move" };
    }
    return null;
  }
  const tAt = (e) => { const r = tk.getBoundingClientRect(); return Math.max(0, Math.min(duration(), ((e.clientX - r.left) / r.width) * duration())); };
  tk.addEventListener("pointerdown", (e) => {
    if (!st) return;
    const hit = tlHit(e);
    tk.setPointerCapture(e.pointerId);
    if (hit) {
      const x = st.dist[hit.i];
      tlDrag = { ...hit, t0: tAt(e), at0: x.at_s ?? 0, dur0: x.duration_s, moved: false };
      select({ kind: "dist", i: hit.i });
    } else { tlDrag = { edge: "time" }; seek(tAt(e)); }
  });
  tk.addEventListener("pointermove", (e) => {
    if (!st) return;
    if (!tlDrag) { const h = tlHit(e); tk.style.cursor = h ? (h.edge === "end" ? "ew-resize" : "grab") : "pointer";
      tk.title = h ? `${KIND_TXT[st.dist[h.i].kind] || st.dist[h.i].kind} at ${st.dist[h.i].at_s ?? 0} s${st.dist[h.i].duration_s ? ` for ${st.dist[h.i].duration_s} s` : ""}` : ""; return; }
    if (tlDrag.edge === "time") { seek(tAt(e)); return; }
    const dt = Math.round(tAt(e) - tlDrag.t0), x = st.dist[tlDrag.i];
    if (!dt && !tlDrag.moved) return;
    tlDrag.moved = true;
    if (tlDrag.edge === "move") x.at_s = Math.max(0, tlDrag.at0 + dt);
    else x.duration_s = Math.max(1, (tlDrag.dur0 || 0) + dt);
    const set = (k, v) => { const el = form.querySelector(`[data-bind="dist.${tlDrag.i}.${k}"]`); if (el) el.value = v ?? ""; };
    set("at_s", x.at_s); set("duration_s", x.duration_s);
    kick();
  });
  tk.addEventListener("pointerup", () => { if (tlDrag?.moved) changed(); tlDrag = null; });

  function seek(t) {
    T = Math.max(0, Math.min(duration(), t));
    slider.value = String(Math.round(T));
    $("#fe-clock").textContent = `${clock(T)} / ${clock(duration())}`;
    const add = form.querySelector('[data-add="dist"]');
    if (add) add.textContent = `Add at ${clock(T)}`;
    hint(); kick();
  }
  slider.addEventListener("input", () => seek(Number(slider.value)));

  // -- pointer on the floor --------------------------------------------------------------

  function nearestWp(wx, wy) {
    if (!world) return null;
    let best = null, bd = Math.max(0.7, 12 / view.s);
    for (const [n, p] of Object.entries(world.waypoints)) { const d = Math.hypot(p[0] - wx, p[1] - wy); if (d < bd) { bd = d; best = n; } }
    return best;
  }
  function segDist(p, a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1], L = dx * dx + dy * dy;
    const f = L ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L)) : 0;
    return Math.hypot(p[0] - a[0] - f * dx, p[1] - a[1] - f * dy);
  }
  // A click in Pan selects what is under it: a blockage, a mover's path, a task's line.
  function pickAt(wx, wy) {
    const tol = Math.max(0.4, 8 / view.s);
    for (let i = st.dist.length - 1; i >= 0; i--) {
      const x = st.dist[i];
      if (x.kind !== "blockage") continue;
      const r = x.v.rect.map(num);
      if (r.some((v) => v == null)) continue;
      if (wx >= Math.min(r[0], r[2]) - tol && wx <= Math.max(r[0], r[2]) + tol && wy >= Math.min(r[1], r[3]) - tol && wy <= Math.max(r[1], r[3]) + tol) return { kind: "dist", i };
    }
    for (let i = 0; i < st.actors.length; i++) {
      const w = st.actors[i].wps;
      for (let k = 0; k + 1 < w.length; k++) if (segDist([wx, wy], w[k], w[k + 1]) < tol) return { kind: "actor", i };
    }
    for (let i = 0; i < st.tasks.length; i++) {
      const A = world.waypoints[st.tasks[i].pickup], B = world.waypoints[st.tasks[i].dropoff];
      if (A && B && segDist([wx, wy], A, B) < tol) return { kind: "task", i };
    }
    return null;
  }
  const pointerXY = (e) => { const r = cv.getBoundingClientRect(); return toWorld(e.clientX - r.left, e.clientY - r.top); };

  cv.addEventListener("pointerdown", (e) => {
    if (!view || !st || e.button !== 0) return;
    const [wx, wy] = pointerXY(e);
    cv.setPointerCapture(e.pointerId);
    if (tool === "block") { draft = { kind: "rect", x0: wx, y0: wy, x1: wx, y1: wy }; drag = null; return; }
    drag = { x: e.clientX, y: e.clientY, vx: view.x, vy: view.y, moved: false };
  });
  cv.addEventListener("pointermove", (e) => {
    if (!view || !st) return;
    const [wx, wy] = pointerXY(e);
    if (draft?.kind === "rect" && e.buttons) { draft.x1 = wx; draft.y1 = wy; kick(); return; }
    if (drag) {
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 4) drag.moved = true;
      if (drag.moved) { view.x = drag.vx + dx; view.y = drag.vy + dy; kick(); return; }
    }
    const wp = nearestWp(wx, wy);
    hover = { wp, xy: [r1(wx), r1(wy)] };
    const tip = $("#fe-tip"), r = cv.getBoundingClientRect();
    tip.hidden = !wp;
    if (wp) { tip.textContent = wp; tip.style.left = `${e.clientX - r.left + 14}px`; tip.style.top = `${e.clientY - r.top + 10}px`; }
    kick();
  });
  cv.addEventListener("pointerleave", () => { hover = null; $("#fe-tip").hidden = true; kick(); });
  cv.addEventListener("pointerup", (e) => {
    if (!view || !st) return;
    const [wx, wy] = pointerXY(e);
    if (draft?.kind === "rect") {
      const d = draft; draft = null;
      const rect = [r1(Math.min(d.x0, d.x1)), r1(Math.min(d.y0, d.y1)), r1(Math.max(d.x0, d.x1)), r1(Math.max(d.y0, d.y1))];
      if (rect[2] - rect[0] >= 0.1 && rect[3] - rect[1] >= 0.1) placeBlockage(rect);
      kick(); return;
    }
    const d = drag; drag = null;
    if (d && d.moved) return;
    clickAt(wx, wy);
  });
  function clickAt(wx, wy) {
    const wp = nearestWp(wx, wy);
    if (tool === "task") {
      if (!wp) return;
      if (!pending) { pending = wp; hint(); kick(); return; }
      if (wp === pending) return;
      st.tasks.push({ at_s: nextTaskAt(), pickup: pending, dropoff: wp, extra: {} });
      pending = null;
      sel = { kind: "task", i: st.tasks.length - 1 };
      renderSec("tasks"); openSec("tasks"); select(sel);
      changed(); hint(); return;
    }
    if (tool === "mover") { draft = draft?.kind === "path" ? draft : { kind: "path", pts: [] }; const p = [r1(wx), r1(wy)]; const last = draft.pts[draft.pts.length - 1];
      if (!last || Math.hypot(last[0] - p[0], last[1] - p[1]) >= 0.05) draft.pts.push(p); hint(); kick(); return; }
    if (tool === "gateway") {
      st.world.gw = `${r1(wx)}, ${r1(wy)}`;
      const el = form.querySelector('[data-bind="world.gw"]'); if (el) el.value = st.world.gw;
      openSec("world"); setTool("pan"); changed(); return;
    }
    if (armed && wp) {
      setPath(st, armed.path, wp);
      const el = form.querySelector(`[data-bind="${CSS.escape(armed.path)}"]`);
      if (el) el.value = wp;
      changed();
      // A pickup filled moves on to its dropoff, so a task is two clicks.
      const next = armed.path.endsWith(".pickup") ? armed.path.replace(/pickup$/, "dropoff") : null;
      const nel = next && form.querySelector(`[data-bind="${CSS.escape(next)}"]`);
      if (nel) { nel.focus({ preventScroll: true }); arm(nel); } else { el?.blur(); arm(null); }
      return;
    }
    select(pickAt(wx, wy));
  }
  function placeBlockage(rect) {
    if (redraw?.kind === "dist" && st.dist[redraw.i]?.kind === "blockage") {
      st.dist[redraw.i].v.rect = rect;
      sel = { kind: "dist", i: redraw.i };
      redraw = null;
      setTool("pan");
    } else {
      st.dist.push(distFrom({ at_s: Math.round(T), blockage: { rect } }));
      sel = { kind: "dist", i: st.dist.length - 1 };
    }
    renderSec("dist"); openSec("dist"); select(sel); changed(); hint();
  }
  function finishPath() {
    if (draft?.kind !== "path") return;
    const pts = draft.pts;
    draft = null;
    if (pts.length < 2) { hint(); kick(); return; }
    if (redraw?.kind === "actor" && st.actors[redraw.i]) { st.actors[redraw.i].wps = pts; sel = { kind: "actor", i: redraw.i }; }
    else {
      const ids = new Set(st.actors.map((a) => a.id));
      let k = st.actors.length + 1; while (ids.has(`walker_${k}`)) k++;
      st.actors.push(actorFrom({ id: `walker_${k}`, shape: "cylinder", radius: 0.25, height: 1.75,
        path: { waypoints: pts, speed_mps: 1.2, mode: "pingpong", start_at_s: Math.round(T) } }));
      sel = { kind: "actor", i: st.actors.length - 1 };
    }
    redraw = null;
    setTool("pan");
    renderSec("actors"); openSec("actors"); select(sel); changed();
  }
  cv.addEventListener("dblclick", (e) => {
    if (tool === "mover") { e.preventDefault(); finishPath(); return; }
    if (tool === "pan" && !armed) { view = null; kick(); }
  });
  cv.addEventListener("wheel", (e) => {
    if (!view) return;
    e.preventDefault();
    const r = cv.getBoundingClientRect(), a = e.clientX - r.left, b = e.clientY - r.top, k = Math.exp(-e.deltaY * 0.0015);
    view.x = a - (a - view.x) * k; view.y = b - (b - view.y) * k; view.s *= k;
    kick();
  }, { passive: false });
  $("#fe-fit").addEventListener("click", () => { view = null; kick(); });

  // -- save, run -------------------------------------------------------------------------

  async function save({ as = null, overwrite = false } = {}) {
    if (!st || !cat?.control) return null;
    clearTimeout(checkTimer);
    const doc = toDoc();
    if (as) doc.name = as;
    const same = file && doc.name === file.name;
    const body = { scenario: doc, header: st.header, overwrite: same || overwrite, base_sha: same ? file.sha : null, source: file?.name || source };
    const box = $("#fe-err");
    try {
      const res = await api("/api/fleetsim/editor/save", body);
      st.name = doc.name;
      file = { name: res.name, sha: res.sha, doc: clone(doc), comments: (file?.comments || []).filter((c) => !(res.dropped || []).some((d) => d.section === c.section && d.text === c.text)) };
      source = null;
      unsaved = false;
      const nm = form.querySelector('[data-bind="name"]'); if (nm) nm.value = st.name;
      paintHead(); paintComments();
      await runCheck();
      box.insertAdjacentHTML("afterbegin", `<div class="ok">Saved ${esc(res.file)}.${res.dropped?.length ? ` ${res.dropped.length} comment${res.dropped.length === 1 ? "" : "s"} in changed sections dropped.` : ""}</div>`);
      refreshList(true);
      window.sortbotsFleetRun?.reload();
      return res;
    } catch (e) {
      if (e.status === 409 && /already exists/.test(e.message) && !overwrite) {
        if (confirm(`${doc.name}.yaml already exists. Replace it?`)) return save({ as, overwrite: true });
        return null;
      }
      result = { ok: false, error: `Not saved: ${e.message}`, where: e.where || null, problems: result?.problems || [] };
      paintResult(); paintHead();
      return null;
    }
  }
  $("#fe-save").addEventListener("click", () => save());
  $("#fe-saveas").addEventListener("click", () => {
    $("#fe-as").hidden = false;
    const el = $("#fe-asname");
    el.value = file || source ? freeName(st.name.replace(/_\d+$/, "")) : st.name;
    el.focus(); el.select();
  });
  const saveAs = async () => { const n = $("#fe-asname").value.trim(); if (!n) return; if (await save({ as: n })) $("#fe-as").hidden = true; };
  $("#fe-as-go").addEventListener("click", saveAs);
  $("#fe-asname").addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); saveAs(); } });
  $("#fe-as-no").addEventListener("click", () => { $("#fe-as").hidden = true; });
  $("#fe-validate").addEventListener("click", async () => {
    const res = await runCheck();
    if (res?.ok) $("#fe-err").insertAdjacentHTML("afterbegin", `<div class="info">Checked just now, as a run would load it.</div>`);
  });
  // Run: save first (the run form launches the file, not this page), then hand it over.
  $("#fe-run").addEventListener("click", async () => {
    if (!st) return;
    if ((unsaved || !file) && !(await save())) return;
    const name = file.name;
    close(true);
    window.sortbotsTests?.openTests();
    await window.sortbotsFleetRun?.pick(name);
  });
  $("#fe-x").addEventListener("click", () => close());

  // -- keys --------------------------------------------------------------------------------
  // Captured before operator.js sees them: its map tools (V/F/R/T) and robot driving
  // (WASD) belong to the live map this view covers.
  addEventListener("keydown", (e) => {
    if (root.hidden) return;
    const inField = e.target.closest?.("input, select, textarea");
    // In a field, typing goes on to the field's own handlers (operator.js ignores fields
    // anyway); Escape and everything outside a field stop here.
    if (!inField || e.key === "Escape" || ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "s")) e.stopPropagation();
    if (!st) return;
    if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "s") { e.preventDefault(); save(); return; }
    if (e.key === "Escape") {
      e.preventDefault();
      if (draft || pending || redraw || tool !== "pan") { setTool("pan"); return; }
      if (armed) { arm(null); if (inField) e.target.blur(); return; }
      if (inField) { e.target.blur(); return; }
      close();
      return;
    }
    if (inField || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.key === "Enter" && tool === "mover") { e.preventDefault(); finishPath(); return; }
    const t = { v: "pan", t: "task", b: "block", m: "mover", g: "gateway" }[e.key.toLowerCase()];
    if (t) { setTool(t); return; }
    const step = e.shiftKey ? 10 : 1;
    if (e.key === "ArrowRight") { e.preventDefault(); seek(T + step); }
    else if (e.key === "ArrowLeft") { e.preventDefault(); seek(T - step); }
  }, true);
  addEventListener("beforeunload", (e) => { if (!root.hidden && unsaved) { e.preventDefault(); e.returnValue = ""; } });

  new ResizeObserver(() => { if (!root.hidden) { placeRoot(); view = null; kick(); } }).observe(root);
  // A theme switch recolours the floor: the grid is rendered once per theme.
  const recolour = () => { if (world) renderFloor().then(kick); };
  new MutationObserver(recolour).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
  matchMedia("(prefers-color-scheme: dark)").addEventListener?.("change", recolour);

  // -- the Tests tab's list of scenarios ---------------------------------------------------------

  const list = $("#fe-list");
  let listAt = 0;
  async function refreshList(force) {
    if (!list || (!force && Date.now() - listAt < 5000)) return;
    listAt = Date.now();
    try { await loadCatalog(); } catch (e) { list.innerHTML = `<div class="tp-empty">Scenarios unavailable: ${esc(e.message)}</div>`; return; }
    $("#fe-count").textContent = cat.scenarios.length ? `· ${cat.scenarios.length}` : "";
    list.innerHTML = cat.scenarios.map((s) => {
      const meta = s.status === "ready" ? [s.name, `${s.robots} robots`, s.network, clock(s.duration_s)].join(" · ") : `${s.name} · invalid`;
      return `<div class="fs-run${s.status === "ready" ? "" : " invalid"}" data-name="${esc(s.name)}" title="${esc(s.description || s.error || "")}">
        <div class="fs-t"><b>${esc(s.title)}</b><span class="muted">${esc(meta)}</span>${s.error ? `<span class="muted err">${esc(s.error)}</span>` : ""}</div>
        <div class="fe-btns"><button type="button" class="btn small" data-fe-edit title="Open it on its floor">Edit</button>
          <button type="button" class="btn small" data-fe-new title="A new scenario starting as a copy of this one">New from</button></div></div>`;
    }).join("") || `<div class="tp-empty">No scenarios in fleetsim/scenarios/.</div>`;
  }
  if (list) {
    list.addEventListener("click", (e) => {
      const b = e.target.closest("[data-fe-edit], [data-fe-new]");
      if (!b) return;
      open(b.closest("[data-name]").dataset.name, { asNew: b.hasAttribute("data-fe-new") });
    });
    const card = $("#session .card"), panel = $("#session");
    const check = () => { if (panel && !panel.hidden && card && card.classList.contains("tab-tests")) refreshList(false); };
    if (panel) new MutationObserver(check).observe(panel, { attributes: true, attributeFilter: ["hidden"] });
    if (card) new MutationObserver(check).observe(card, { attributes: true, attributeFilter: ["class"] });
  }

  window.sortbotsFleetEdit = {
    open, close, save, check: runCheck, seek, setTool, refreshList: () => refreshList(true),
    state: () => ({
      open: !root.hidden, name: st?.name ?? null, unsaved, checking, tool, T, armed: armed?.path ?? null, pending,
      ok: result?.ok ?? null, error: result?.error ?? null, where: result?.where ?? null, problems: result?.problems ?? [],
      floor: !!floorCv, waypoints: world ? Object.keys(world.waypoints).length : 0, doc: st ? toDoc() : null,
      sel, toPx: (x, y) => (view ? px(x, y) : null), wp: (name) => world?.waypoints?.[name] ?? null,
    }),
  };
})();
