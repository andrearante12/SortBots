// SortBots operator view — fleet replay: a recorded fleetsim run (fleetsim/) drawn
// on its own floor, with a scrubber. Opened from the Session panel's Tests tab
// ("Fleet sim runs"); it covers the stage until closed and never touches the live
// fleet, the robot cards or the task list — none of what it shows is real.
//
// Talks ONLY to webui/serve.py:
//   GET /api/fleetsim/runs           recent runs in data/fleetsim/ and the benches under
//                                    data/fleetsim/bench/ (fleetsim/replay.py list_runs, list_benches)
//   GET /api/fleetsim/bench?dir=B    one bench's runs, when its group is opened (list_bench)
//   GET /api/fleetsim/run?dir=REF    one run, packed whole (fleetsim/replay.py pack); REF is a
//                                    run dir name or bench/<bench>/<run>
//
// The whole run comes down at once and is scrubbed locally: every stream is a list
// of [t, ...] rows in time order, so "the state at T" is a bisection, and scrubbing
// backwards costs the same as forwards. Poses are interpolated between the 5 Hz
// ground-truth samples, so playback at 1x doesn't step.
//
// What it can show is what fleetsim recorded (record: in fleetsim/configs/base.yaml):
// the true floor (`world`), poses, task states, planned paths (`path`), radio links
// (`links`, radio network only) and, for the robots in record.belief_maps, that
// robot's OWN map over time (`belief_map`) — the map it actually planned on, with
// where it disagrees with the truth marked.
//
// The comms monitor (Traffic) draws the `net` samples (fleetsim/replay.py _comms): channel
// load, traffic by kind, delivered vs lost, and per node (record.net_nodes) each robot's
// load and traffic. It is for tuning the communication scheme, so every number it
// draws can be read: hover a chart, or the legends and the info panel at T.
//
// Coordinates: world metres, y up; the floor grid is ROS order (row 0 at origin y).

(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const root = $("#replay");
  if (!root) return;
  const cv = $("#rp-map"), ctx = cv.getContext("2d");
  const tk = $("#rp-ticks"), tctx = tk.getContext("2d");
  const slider = $("#rp-slider");

  const SPEEDS = [1, 2, 5, 10, 30, 60];
  const MARK_FADE_S = 4;        // a contact / blocked marker stays on the floor this long
  const FEED_N = 6;
  const RUNS_REFRESH_MS = 10000;
  // Mark kind -> [CSS token, label]. Colour = what kind of trouble, so the timeline
  // reads at a glance: red is something hit or failed, amber is something stuck.
  const MARK = {
    contact: ["--stop", "Contact"], failed: ["--stop", "Task failed"], gave_up: ["--stop", "Gave up"],
    blocked: ["--warnink", "Blocked"], plan_failed: ["--warnink", "No path"], empty_shelf: ["--warnink", "Empty shelf"],
    recovery: ["--faint", "Recovery"], done: ["--go", "Delivered"], disturb: ["--info", "Disturbance"],
  };
  const CARRY = new Set(["NAV_TO_DROPOFF", "PLACING"]);
  const FETCH = new Set(["NAV_TO_PICKUP", "AWAIT_PICK", "PICKING"]);

  let run = null;               // the packed run (fleetsim/replay.py _pack)
  let idx = null;               // time columns + derived spans, built once per run
  let T = 0, playing = false, speedI = 2, lastTick = 0;
  let sel = -1;                 // selected robot index
  let view = null;              // {s: px per metre, x, y: px of world (0, 0)}
  let dirty = true, raf = 0, loading = null;
  let truth = null;             // Int8Array, the floor at start
  let floorCv = null;           // the truth, rendered
  const bmapCv = new Map();     // `${rid}|${i}` -> canvas, or "pending"
  let infoSig = "";
  let runs = [], runsAt = 0, runsErr = "";
  let benches = [];                 // fleetsim bench dirs (list_benches), newest first
  const benchRuns = new Map();      // bench dir -> {runs, err, mtime}: fetched when opened
  const openBenches = new Set();    // the bench groups open in the list
  const show = { paths: true, links: true, own: true, load: false };
  // The side panel covers a third of a phone's floor, so it starts closed there.
  let showInfo = !matchMedia("(max-width: 760px), (max-height: 520px) and (orientation: landscape)").matches;

  const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  const words = (s) => String(s || "").replace(/_/g, " ");
  const titleOf = (name) => { const w = words(name); return w.charAt(0).toUpperCase() + w.slice(1); };
  const simTime = (s) => (s == null ? "" : s >= 90 ? `${Math.round(s / 30) / 2} min` : `${Math.round(s)} s`);
  const short = (rid) => String(rid).replace(/^robot_/, "R");

  // Last index i with a[i] <= t (a sorted), or -1.
  function bisect(a, t) {
    let lo = 0, hi = a.length - 1, ans = -1;
    while (lo <= hi) { const m = (lo + hi) >> 1; if (a[m] <= t) { ans = m; lo = m + 1; } else hi = m - 1; }
    return ans;
  }
  const col = (rows) => Float64Array.from(rows, (r) => r[0]);

  async function inflate(z) {
    const bin = Uint8Array.from(atob(z), (c) => c.charCodeAt(0));
    const stream = new Blob([bin]).stream().pipeThrough(new DecompressionStream("deflate"));
    return new Int8Array(await new Response(stream).arrayBuffer());
  }

  // -- loading --------------------------------------------------------------------

  async function open(dir) {
    root.hidden = false;
    setLoading(`Loading ${dir}…`);
    stop();
    run = null; idx = null; truth = null; floorCv = null; bmapCv.clear(); sel = -1; view = null;
    truthKey = ""; truthNow = null; hover = null; legSig = ""; hiddenKinds.clear(); monShown = null;
    showInfo = !phone.matches;
    $("#rp-info-toggle").setAttribute("aria-pressed", String(showInfo));
    const token = (loading = {});
    try {
      const res = await fetch(`/api/fleetsim/run?dir=${encodeURIComponent(dir)}`, { cache: "no-store" });
      const body = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(body.error || res.statusText);
      if (token !== loading) return; // another open() won
      run = body;
      build();
      truth = await inflate(run.world.grid.z);
      if (token !== loading) return;
      renderFloor();
      setLoading(null);
      try { history.replaceState(null, "", `#replay=${encodeURIComponent(dir)}`); } catch { /* file:// */ }
      seek(0);
      fillBar();
    } catch (e) {
      if (token === loading) setLoading(`Couldn't load the run: ${e.message}`, true);
    }
  }
  function close() {
    stop(); loading = null;
    root.hidden = true; run = null; idx = null; truth = null; floorCv = null; bmapCv.clear();
    if (/^#replay=/.test(location.hash)) { try { history.replaceState(null, "", location.pathname + location.search); } catch { /* */ } }
  }
  function setLoading(text, err = false) {
    const m = $("#rp-msg");
    m.hidden = !text; m.textContent = text || ""; m.classList.toggle("err", !!err);
    $("#rp-time").hidden = !!text && !run;
    if (!run) $("#rp-info").hidden = true;
  }

  function build() {
    const r = run, n = r.robots.length;
    idx = {
      poseT: col(r.poses), aposeT: col(r.apose), linksT: col(r.links), boardT: col(r.board),
      netT: col(r.net), beliefT: col(r.belief), markT: col(r.marks),
      pathT: r.robots.map((rid) => col(r.paths[rid] || [])),
      stateT: r.robots.map((rid) => col(r.states[rid] || [])),
      bmapT: r.robots.map((rid) => col(r.belief_maps[rid] || [])),
      spans: [], kills: [],
    };
    // The comms monitor's index. Rows and node rows share their sample index.
    const cm = r.comms || (r.comms = { kinds: [], rows: [], nodes: [], model: r.network, cap: 0.95 });
    idx.commsT = col(cm.rows);
    idx.kOff = cm.kinds.map((_, k) => 6 + 7 * k);
    idx.hasLoad = cm.model === "radio" && cm.rows.some((x) => x[3] != null);
    idx.hasNodeLoad = cm.nodes.some((x) => x[1]);
    idx.hasNodes = cm.nodes.some((x) => x[3]);
    idx.hasAir = cm.rows.some((x) => idx.kOff.some((o) => x[o + 5] != null));
    idx.overS = cm.rows.reduce((a, x) => a + ((x[3] || 0) > cm.cap ? x[1] : 0), 0);
    // Disturbances as spans: an `on` event closes at its matching `off` (same body).
    const openSpans = new Map();
    for (const d of r.disturb) {
      const { t, on, ...body } = d;
      const key = JSON.stringify(body);
      if (on) { const sp = { t0: t, t1: r.t_end, body }; idx.spans.push(sp); openSpans.set(key, sp); }
      else if (openSpans.has(key)) { openSpans.get(key).t1 = t; openSpans.delete(key); }
    }
    // Who is dead when: a kill holds until its own end or a `revive`.
    for (const d of r.disturb) {
      if ("kill" in d && d.on) idx.kills.push({ rid: d.kill, t0: d.t, t1: r.t_end });
      const endsKill = ("kill" in d && !d.on) || ("revive" in d && d.on);
      if (endsKill) {
        const who = d.kill || d.revive;
        const k = idx.kills.filter((x) => x.rid === who && x.t1 >= d.t).pop();
        if (k) k.t1 = d.t;
      }
    }
    idx.rix = new Map(r.robots.map((rid, i) => [rid, i]));
    idx.n = n;
    slider.max = String(r.t_end);
    slider.step = "0.1";
  }

  // -- state at T --------------------------------------------------------------------

  function poseAt(i) {
    const rows = run.poses, ts = idx.poseT;
    if (!rows.length) return null;
    let a = bisect(ts, T);
    if (a < 0) a = 0;
    const A = rows[a], B = rows[Math.min(a + 1, rows.length - 1)];
    const o = 1 + 3 * i;
    if (A[o] == null) return null;
    if (B === A || B[o] == null || B[0] <= A[0]) return [A[o], A[o + 1], A[o + 2]];
    const f = Math.min(1, Math.max(0, (T - A[0]) / (B[0] - A[0])));
    let dy = B[o + 2] - A[o + 2];
    if (dy > Math.PI) dy -= 2 * Math.PI; else if (dy < -Math.PI) dy += 2 * Math.PI;
    return [A[o] + f * (B[o] - A[o]), A[o + 1] + f * (B[o + 1] - A[o + 1]), A[o + 2] + f * dy];
  }
  function actorPoses() {
    const rows = run.apose, out = [];
    if (!rows.length) return out;
    let a = bisect(idx.aposeT, T); if (a < 0) a = 0;
    const A = rows[a], B = rows[Math.min(a + 1, rows.length - 1)];
    const f = B[0] > A[0] ? Math.min(1, Math.max(0, (T - A[0]) / (B[0] - A[0]))) : 0;
    run.actor_ids.forEach((id, k) => {
      const o = 1 + 2 * k;
      if (A[o] == null) return;
      const bx = B[o] == null ? A[o] : B[o], by = B[o + 1] == null ? A[o + 1] : B[o + 1];
      out.push({ id, x: A[o] + f * (bx - A[o]), y: A[o + 1] + f * (by - A[o + 1]), r: (run.world.actors || {})[id] || 0.3 });
    });
    return out;
  }
  const at = (rows, ts) => { const i = bisect(ts, T); return i < 0 ? null : rows[i]; };
  const stateOf = (i) => at(run.states[run.robots[i]] || [], idx.stateT[i]);
  const pathOf = (i) => at(run.paths[run.robots[i]] || [], idx.pathT[i]);
  const deadAt = (rid) => idx.kills.some((k) => k.rid === rid && k.t0 <= T && T < k.t1);
  const activeSpans = () => idx.spans.filter((s) => s.t0 <= T && T < s.t1);

  function modeOf(i) {
    const rid = run.robots[i];
    if (deadAt(rid)) return "dead";
    const st = stateOf(i), p = pathOf(i);
    const s = st ? st[1] : "IDLE";
    if (CARRY.has(s)) return "carry";
    if (FETCH.has(s)) return "fetch";
    if (p && p[1] === "home" && p[2].length) return "home";
    return "idle";
  }
  const MODE_COL = { dead: "--stop", carry: "--go", fetch: "--info", home: "--faint", idle: "--muted" };
  const MODE_TXT = { dead: "switched off", carry: "carrying", fetch: "to pickup", home: "going home", idle: "idle" };

  // Node reachability from the gateway: radio links when recorded, else every
  // pair minus what a disturbance cut. Node 0 = gateway, robot_k = node k+1.
  function cutOff(spans) {
    const n = idx.n + 1, deadNodes = new Set();
    run.robots.forEach((rid, i) => { if (deadAt(rid)) deadNodes.add(i + 1); });
    // kill: gateway (fleetsim, 2026-11-26): no live robot can reach it.
    if (deadAt("gateway")) return new Set(run.robots.map((_, i) => i).filter((i) => !deadNodes.has(i + 1)));
    const nodeOf = (name) => (name === "gateway" ? 0 : idx.rix.has(name) ? idx.rix.get(name) + 1 : -1);
    let adj = null;
    if (run.links.length) {
      const row = at(run.links, idx.linksT) || run.links[0]; // before the first sample: the first
      adj = Array.from({ length: n }, () => []);
      if (row) for (let k = 0; k < row[1].length; k += 3) { const a = row[1][k], b = row[1][k + 1]; adj[a].push(b); adj[b].push(a); }
    } else {
      if (n > 300) return new Set(); // n^2 per frame; a fleet that big is radio anyway
      const blocked = new Set();
      for (const s of spans) {
        if (s.body.block_link) { const [a, b] = s.body.block_link.map(nodeOf); blocked.add(`${Math.min(a, b)}-${Math.max(a, b)}`); }
        if (s.body.partition) {
          const named = new Set();
          const groups = s.body.partition.map((g) => (g === "rest" ? null : new Set(g.map(nodeOf))));
          groups.forEach((g) => g && g.forEach((x) => named.add(x)));
          const sets = groups.map((g) => g || new Set([...Array(n).keys()].filter((x) => !named.has(x))));
          for (let a = 0; a < sets.length; a++) for (let b = a + 1; b < sets.length; b++)
            for (const x of sets[a]) for (const y of sets[b]) blocked.add(`${Math.min(x, y)}-${Math.max(x, y)}`);
        }
      }
      adj = Array.from({ length: n }, (_, a) => [...Array(n).keys()].filter((b) => b !== a && !blocked.has(`${Math.min(a, b)}-${Math.max(a, b)}`)));
    }
    const seen = new Set([0]), q = [0];
    while (q.length) { const a = q.pop(); for (const b of adj[a]) if (!seen.has(b) && !deadNodes.has(b)) { seen.add(b); q.push(b); } }
    const out = new Set();
    for (let k = 1; k < n; k++) if (!seen.has(k) && !deadNodes.has(k)) out.add(k - 1);
    return out;
  }

  // -- drawing ----------------------------------------------------------------------

  function fit() {
    // With the comms monitor up, the floor fits in what it leaves above it.
    const g = run.world.grid, W = cv.clientWidth, H = mon.hidden ? cv.clientHeight : Math.max(120, mon.offsetTop);
    const wm = g.w * g.res, hm = g.h * g.res;
    const s = Math.min(W / wm, H / hm) * 0.94;
    view = { s, x: (W - wm * s) / 2 - g.ox * s, y: (H + hm * s) / 2 + g.oy * s };
  }
  const px = (x, y) => [view.x + x * view.s, view.y - y * view.s];
  const world = (a, b) => [(a - view.x) / view.s, (view.y - b) / view.s];

  function rgb(hex) {
    const m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})/i.exec(hex) || [0, "80", "80", "80"];
    return [parseInt(m[1], 16), parseInt(m[2], 16), parseInt(m[3], 16)];
  }
  // Grid -> canvas, one pixel a cell, row 0 at the bottom. `diff`: mark cells
  // that disagree with the truth (a robot's own map).
  // The truth at T: the start floor plus any blockage up now. A robot's own map
  // is judged against this, so an obstacle it hasn't seen or heard of shows red.
  let truthKey = "", truthNow = null;
  function truthAt(spans) {
    const bl = spans.filter((sp) => sp.body.blockage);
    const key = bl.map((sp) => sp.t0).join(",");
    if (key === truthKey && truthNow) return truthNow;
    const g = run.world.grid, out = truth.slice();
    for (const sp of bl) {
      const [x0, y0, x1, y1] = sp.body.blockage.rect;
      const c0 = Math.floor((Math.min(x0, x1) - g.ox) / g.res), c1 = Math.floor((Math.max(x0, x1) - g.ox) / g.res);
      const r0 = Math.floor((Math.min(y0, y1) - g.oy) / g.res), r1 = Math.floor((Math.max(y0, y1) - g.oy) / g.res);
      for (let r = Math.max(0, r0); r <= Math.min(g.h - 1, r1); r++)
        for (let q = Math.max(0, c0); q <= Math.min(g.w - 1, c1); q++) out[r * g.w + q] = 100;
    }
    truthKey = key; truthNow = out;
    return out;
  }
  function gridCanvas(cells, diff) {
    const g = run.world.grid, c = document.createElement("canvas");
    c.width = g.w; c.height = g.h;
    const im = c.getContext("2d").createImageData(g.w, g.h);
    const U = rgb(css("--map-unknown")), F = rgb(css("--map-free")), O = rgb(css("--map-occ")), X = rgb(css("--stop"));
    for (let r = 0; r < g.h; r++) {
      const dst = (g.h - 1 - r) * g.w;
      for (let q = 0; q < g.w; q++) {
        const i = r * g.w + q, v = cells[i];
        let p = v < 0 ? U : v >= 50 ? O : F;
        if (diff && v >= 0 && (v >= 50) !== (diff[i] >= 50)) p = X;
        const o = (dst + q) * 4;
        im.data[o] = p[0]; im.data[o + 1] = p[1]; im.data[o + 2] = p[2]; im.data[o + 3] = 255;
      }
    }
    c.getContext("2d").putImageData(im, 0, 0);
    return c;
  }
  function renderFloor() { floorCv = truth ? gridCanvas(truth, null) : null; bmapCv.clear(); dirty = true; }

  // The selected robot's own map at T, if recorded; decoded on first use.
  function ownMap() {
    if (sel < 0 || !show.own) return null;
    const rid = run.robots[sel], rows = run.belief_maps[rid];
    if (!rows || !rows.length) return null;
    const i = bisect(idx.bmapT[sel], T);
    if (i < 0) return null;
    const ref = truthAt(activeSpans());
    const key = `${rid}|${i}|${truthKey}`, c = bmapCv.get(key);
    if (c === "pending") return null;
    if (c) return { cv: c, t: rows[i][0] };
    bmapCv.set(key, "pending");
    inflate(rows[i][1].z).then((cells) => { if (run) { bmapCv.set(key, gridCanvas(cells, ref)); dirty = true; infoSig = ""; kick(); } });
    return null;
  }

  // The info panel starts under the bar, however many rows the bar wrapped to.
  const phone = matchMedia("(max-width: 760px), (max-height: 520px) and (orientation: landscape)");
  function placeInfo() {
    const bar = $("#rp-bar"), info = $("#rp-info");
    // A phone has room for one of them over the floor: the monitor wins while it is up.
    info.hidden = !showInfo || !run || (phone.matches && !mon.hidden);
    if (phone.matches) { info.style.top = ""; info.style.maxHeight = ""; return; } // docked above the timeline (CSS)
    const top = bar.offsetTop + bar.offsetHeight + 8;
    info.style.top = `${top}px`;
    info.style.maxHeight = mon.hidden ? "" : `${Math.max(80, mon.offsetTop - top - 8)}px`;
  }

  function draw() {
    dirty = false;
    placeMon();
    placeInfo();
    const W = cv.clientWidth, H = cv.clientHeight, dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = css("--map-bg"); ctx.fillRect(0, 0, W, H);
    if (!run || !floorCv) return;
    if (!view) fit();
    const g = run.world.grid, s = view.s;
    const own = ownMap();
    const [fx, fy] = px(g.ox, g.oy + g.h * g.res);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(own ? own.cv : floorCv, fx, fy, g.w * g.res * s, g.h * g.res * s);
    root.dataset.own = own ? "1" : "";

    const spans = activeSpans();
    // Blockages: a static obstacle that appeared mid-run (the truth changed; the
    // robots only know if they saw it or were told).
    ctx.save();
    for (const sp of spans) {
      if (!sp.body.blockage) continue;
      const [x0, y0, x1, y1] = sp.body.blockage.rect;
      const [a, b] = px(Math.min(x0, x1), Math.max(y0, y1)), [c2, d] = px(Math.max(x0, x1), Math.min(y0, y1));
      // Over a robot's own map only the outline: the cells inside are red if it
      // doesn't know, grey if it does.
      ctx.fillStyle = css("--stop"); ctx.globalAlpha = own ? 0 : 0.35; ctx.fillRect(a, b, c2 - a, d - b);
      ctx.globalAlpha = 1; ctx.strokeStyle = css("--stop"); ctx.lineWidth = 1.5; ctx.strokeRect(a, b, c2 - a, d - b);
    }
    ctx.restore();

    // Waypoints (floor tape) and homes.
    const wps = run.world.waypoints || {};
    const tape = css("--tape"), faint = css("--faint"), ink = css("--map-ink");
    const wpR = Math.max(2, Math.min(5, 0.18 * s));
    ctx.fillStyle = tape;
    for (const name in wps) {
      const [a, b] = px(wps[name][0], wps[name][1]);
      ctx.beginPath(); ctx.moveTo(a, b - wpR); ctx.lineTo(a + wpR, b); ctx.lineTo(a, b + wpR); ctx.lineTo(a - wpR, b); ctx.closePath(); ctx.fill();
    }
    if (s > 28) {
      ctx.fillStyle = css("--muted"); ctx.font = "10px system-ui, sans-serif"; ctx.textAlign = "left";
      for (const name in wps) { const [a, b] = px(wps[name][0], wps[name][1]); ctx.fillText(words(name), a + wpR + 2, b + 3); }
    }
    ctx.fillStyle = faint;
    for (const rid in run.world.homes || {}) { const h = run.world.homes[rid]; const [a, b] = px(h[0], h[1]); ctx.fillRect(a - 1, b - 1, 2, 2); }

    // Robots, current poses.
    const poses = run.robots.map((_, i) => poseAt(i));
    const gw = run.world.gateway;
    // Load (comms monitor): the channel load each node carried in the sample at T.
    const nl = show.load && idx.hasNodeLoad ? nodeRow()?.[1] : null;
    $("#rp-load-leg").hidden = !(show.load && idx.hasNodeLoad);
    const lc = nl ? loadScale() : null;
    loadLinks = [];

    // Radio links: one-hop, alpha by SNR; with Load on, coloured by the busier of
    // their two ends instead (a link is as busy as the channel either end shares).
    // Drawn under everything that moves.
    if (show.links && run.links.length) {
      const row = at(run.links, idx.linksT);
      if (row) {
        const posOf = (node) => (node === 0 ? gw : poses[node - 1]);
        ctx.strokeStyle = css("--info"); ctx.lineWidth = 1;
        for (let k = 0; k < row[1].length; k += 3) {
          const A = posOf(row[1][k]), B = posOf(row[1][k + 1]);
          if (!A || !B) continue;
          if (lc) {
            const f = Math.max(nl[row[1][k]] || 0, nl[row[1][k + 1]] || 0), c = lc(f);
            ctx.strokeStyle = c; ctx.globalAlpha = f >= lc.cap ? 0.9 : 0.3 + 0.6 * (f / lc.cap);
            ctx.lineWidth = 1 + 2 * Math.min(1, f / lc.cap);
            loadLinks.push([row[1][k], row[1][k + 1], f, c]);
          } else ctx.globalAlpha = Math.max(0.08, Math.min(0.55, (row[1][k + 2] - 5) / 40));
          const [a, b] = px(A[0], A[1]), [c2, d] = px(B[0], B[1]);
          ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c2, d); ctx.stroke();
        }
        ctx.globalAlpha = 1; ctx.lineWidth = 1;
      }
    }

    // Planned paths: everyone's faint, the selected robot's solid.
    const drawPath = (pts, style, w, dash) => {
      if (!pts || pts.length < 2) return;
      ctx.strokeStyle = style; ctx.lineWidth = w; ctx.setLineDash(dash);
      ctx.beginPath();
      pts.forEach((p, k) => { const [a, b] = px(p[0], p[1]); if (k) ctx.lineTo(a, b); else ctx.moveTo(a, b); });
      ctx.stroke(); ctx.setLineDash([]);
    };
    if (show.paths) {
      ctx.globalAlpha = 0.45;
      run.robots.forEach((_, i) => { if (i !== sel) { const p = pathOf(i); if (p) drawPath(p[2], css(MODE_COL[modeOf(i)]), 1.2, []); } });
      ctx.globalAlpha = 1;
    }
    if (sel >= 0) {
      const p = pathOf(sel);
      if (p) drawPath(p[2], ink, 2, [6, 4]);
    }

    // Gateway.
    if (gw) {
      const [a, b] = px(gw[0], gw[1]);
      if (lc) halo(a, b, 9, nl[0] || 0, lc);
      ctx.fillStyle = ink; ctx.beginPath(); ctx.moveTo(a, b - 7); ctx.lineTo(a + 6, b + 4); ctx.lineTo(a - 6, b + 4); ctx.closePath(); ctx.fill();
    }

    // Movers.
    for (const a of actorPoses()) {
      const [x, y] = px(a.x, a.y);
      ctx.fillStyle = css("--warnink"); ctx.globalAlpha = 0.85;
      ctx.beginPath(); ctx.arc(x, y, Math.max(3, a.r * s), 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
    }

    // Robots: body disc to scale (never smaller than legible), heading tick, number.
    const cut = cutOff(spans);
    const br = Math.max(4, (run.world.body_r || 0.25) * s);
    ctx.font = `600 ${Math.max(8, Math.min(11, br))}px system-ui, sans-serif`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    const robotInk = css("--map-robot-ink");
    // Load halos first, all of them: a neighbour's must never cover a robot.
    if (lc) poses.forEach((p, i) => { if (p && !deadAt(run.robots[i])) { const [x, y] = px(p[0], p[1]); halo(x, y, br + 4, nl[i + 1] || 0, lc); } });
    poses.forEach((p, i) => {
      if (!p) return;
      const [x, y] = px(p[0], p[1]), m = modeOf(i);
      ctx.fillStyle = css(MODE_COL[m]);
      ctx.beginPath(); ctx.arc(x, y, br, 0, 2 * Math.PI); ctx.fill();
      ctx.strokeStyle = robotInk; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(p[2]) * br, y - Math.sin(p[2]) * br); ctx.stroke();
      if (cut.has(i)) { // can't reach the gateway: dashed ring
        ctx.strokeStyle = css("--stop"); ctx.setLineDash([3, 3]); ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(x, y, br + 4, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
      }
      if (i === sel) { ctx.strokeStyle = ink; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, br + 3, 0, 2 * Math.PI); ctx.stroke(); }
      if (br >= 7) { ctx.fillStyle = robotInk; ctx.fillText(String(i), x, y + 0.5); }
      else if (s > 6) { ctx.fillStyle = ink; ctx.fillText(String(i), x, y - br - 7); }
    });
    ctx.textBaseline = "alphabetic";

    // Recent trouble where it happened, fading.
    const m1 = bisect(idx.markT, T);
    for (let k = m1; k >= 0 && run.marks[k][0] > T - MARK_FADE_S; k--) {
      const [t, kind, , , x, y] = run.marks[k];
      if (x == null || !MARK[kind] || kind === "done") continue;
      const [a, b] = px(x, y), r = 6;
      ctx.globalAlpha = 1 - (T - t) / MARK_FADE_S;
      ctx.strokeStyle = css(MARK[kind][0]); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(a - r, b - r); ctx.lineTo(a + r, b + r); ctx.moveTo(a + r, b - r); ctx.lineTo(a - r, b + r); ctx.stroke();
    }
    ctx.globalAlpha = 1;
    window.__replayFrames = (window.__replayFrames || 0) + 1;
  }

  // A load colour: one hue, from the floor (an idle channel recedes) to --info at the
  // cap, and --stop past it, where the radio loses the excess (or, capacity:
  // unlimited, could not have carried it).
  function loadScale() {
    const A = rgb(css("--map-free")), B = rgb(css("--info")), stop = css("--stop"), cap = run.comms.cap;
    const f = (v) => {
      if (v >= cap) return stop;
      const k = Math.max(0, Math.min(1, v / cap));
      return `rgb(${A.map((a, j) => Math.round(a + (B[j] - a) * k)).join(", ")})`;
    };
    f.cap = cap;
    return f;
  }
  function halo(x, y, r, v, lc) {
    ctx.fillStyle = lc(v); ctx.globalAlpha = v >= lc.cap ? 0.85 : 0.65;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
  }

  // -- timeline ---------------------------------------------------------------------

  function drawTicks() {
    const W = tk.clientWidth, H = tk.clientHeight, dpr = window.devicePixelRatio || 1;
    if (tk.width !== Math.round(W * dpr) || tk.height !== Math.round(H * dpr)) { tk.width = Math.round(W * dpr); tk.height = Math.round(H * dpr); }
    tctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    tctx.clearRect(0, 0, W, H);
    if (!run || !run.t_end) return;
    const xOf = (t) => (t / run.t_end) * W;
    tctx.fillStyle = css("--info"); tctx.globalAlpha = 0.18;
    for (const sp of idx.spans) tctx.fillRect(xOf(sp.t0), 0, Math.max(2, xOf(sp.t1) - xOf(sp.t0)), H);
    tctx.globalAlpha = 1;
    // Deliveries along the bottom, trouble along the top: a stuck stretch and the
    // tasks it delayed are then visibly the same minutes.
    for (const m of run.marks) {
      const def = MARK[m[1]]; if (!def) continue;
      tctx.fillStyle = css(def[0]);
      const top = m[1] === "done" ? H * 0.55 : 0, h = m[1] === "done" ? H * 0.45 : H * 0.5;
      tctx.fillRect(Math.round(xOf(m[0])), top, 1, h);
    }
  }
  function markNear(clientX) {
    if (!run) return null;
    const r = tk.getBoundingClientRect(), t = ((clientX - r.left) / r.width) * run.t_end;
    const tol = (4 / r.width) * run.t_end;
    let best = null, bd = Infinity;
    for (const m of run.marks) { const d = Math.abs(m[0] - t); if (d < bd && d <= tol) { bd = d; best = m; } }
    return best;
  }
  tk.addEventListener("mousemove", (e) => {
    const m = markNear(e.clientX);
    tk.title = m ? `${clock(m[0])} · ${m[2] ? short(m[2]) + " · " : ""}${m[3]}` : "";
  });
  tk.addEventListener("click", (e) => {
    if (!run) return;
    const m = markNear(e.clientX), r = tk.getBoundingClientRect();
    seek(m ? Math.max(0, m[0] - 1) : ((e.clientX - r.left) / r.width) * run.t_end);
    if (m && m[2] && idx.rix.has(m[2])) select(idx.rix.get(m[2]));
  });

  // -- comms monitor ------------------------------------------------------------------
  // fleetsim/replay.py _comms: a row per net sample, [t, dt, load_mean, load_max,
  // over_cap, ogm_air, then per kind sent, delivered, lost, bytes, msgs, air_ms,
  // p95_ms] (counts over dt), and a node row beside it, [t, node_load, node_air, per
  // kind NODE_FIELDS lists]. Both charts span the whole run on the scrubber's own x
  // axis, so the cursor (T) sits over the slider's thumb and the ticks below line up.

  // Kind -> chart series slot (--series-N, operator.html), fixed per kind and never
  // by rank in a run, so a kind keeps its colour from run to run (replay.py KIND_ORDER).
  const KIND_SLOT = { status: 1, tasks: 2, map: 3, intent: 4, obstacle: 5, resync: 6 };
  const kindCol = (k) => `--series-${KIND_SLOT[k] || 7}`;
  const MEASURE = {
    sent: { title: "Copies sent per s", at: (r, o) => r[o] / r[1] },
    lost: { title: "Copies lost per s", at: (r, o) => r[o + 2] / r[1] },
    kbs: { title: "Offered kB/s", at: (r, o) => r[o + 3] / r[1] / 1000 },
    air: { title: "Airtime, % of a channel", at: (r, o) => (r[o + 5] == null ? null : r[o + 5] / 10 / r[1]) },
  };
  const mon = $("#rp-mon"), monLoad = $("#rm-load"), monKind = $("#rm-kind"), tip = $("#rm-tip");
  let showMon = false, measure = "sent", monDirty = false, hover = null, scrubbing = false;
  const hiddenKinds = new Set();
  const monBox = {};             // chart -> its last geometry, for the pointer
  let monShown = null;           // what the last drawMon drew (the test hook reads it)
  let loadLinks = [];            // the last frame's load-coloured links [a, b, load, colour]
  let legSig = "";
  try { showMon = localStorage.getItem("sortbots.replayMon") === "1"; measure = localStorage.getItem("sortbots.replayMeasure") || "sent"; } catch { /* private window */ }
  if (!MEASURE[measure]) measure = "sent";

  const sampleAt = () => bisect(idx.commsT, T);           // the last whole sample at T
  const nodeRow = () => { const i = sampleAt(); return i < 0 ? null : run.comms.nodes[i]; };
  function fmt(v) {
    if (v == null || !isFinite(v)) return "–";
    if (v === 0) return "0";
    if (v >= 1e4) return `${Math.round(v / 1000)}k`;
    if (v >= 1000) return `${(v / 1000).toFixed(1).replace(/\.0$/, "")}k`;
    if (v >= 100) return String(Math.round(v));
    if (v >= 1) return v.toFixed(1).replace(/\.0$/, "");
    if (v >= 0.01) return v.toFixed(2).replace(/0$/, "");
    return "<0.01";
  }
  const pct = (v) => (v == null || !isFinite(v) ? "–" : `${v >= 0.1 || v === 0 ? Math.round(v * 100) : (v * 100).toFixed(1)} %`);
  function nice(v) {
    if (!(v > 0)) return 1;
    const e = 10 ** Math.floor(Math.log10(v)), f = v / e;
    return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 2.5 ? 2.5 : f <= 5 ? 5 : 10) * e;
  }
  const measureOk = (m) => m !== "air" || idx.hasAir;

  function placeMon() {
    const on = showMon && !!run && !!idx;
    mon.hidden = !on;
    $("#rp-mon-toggle").setAttribute("aria-pressed", String(on));
    if (!on) return;
    const time = $("#rp-time");
    mon.style.bottom = `${root.clientHeight - time.offsetTop + 8}px`;
    // The legend column takes what the timeline's buttons take, so the plots start
    // where the scrubber does. Too narrow for that (a phone: the scrubber has its
    // own row): legends go above the plots.
    const mr = mon.getBoundingClientRect(), sr = tk.getBoundingClientRect();
    const legend = sr.left - mr.left - (parseFloat(getComputedStyle(mon).paddingLeft) || 0) - 8;
    mon.classList.toggle("stack", legend < 150);
    if (legend >= 150) mon.style.setProperty("--rm-legend", `${Math.floor(legend)}px`);
  }

  function chartBox(c) {
    const W = c.clientWidth, H = c.clientHeight, dpr = window.devicePixelRatio || 1;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const cr = c.getBoundingClientRect(), sr = tk.getBoundingClientRect();
    let x0 = sr.left - cr.left, x1 = sr.right - cr.left;
    if (x1 - x0 > 60 && x0 >= -4 && x1 <= W + 4) { x0 = Math.max(0, x0); x1 = Math.min(W, x1); } else { x0 = 2; x1 = W - 2; }
    const span = Math.max(1e-6, run.t_end);
    return { g, W, H, x0, x1, y0: 5, y1: H - 3, xOf: (t) => x0 + (t / span) * (x1 - x0), tOf: (x) => Math.max(0, Math.min(span, ((x - x0) / (x1 - x0)) * span)) };
  }
  // Recessive hairline grid, the values right of the plot when there is room for
  // them (the timeline's jump buttons sit below), else inside it, top left.
  function grid(b, yOf, ticks, label) {
    const { g } = b, out = b.W - b.x1 >= 34;
    g.strokeStyle = css("--line"); g.lineWidth = 1; g.fillStyle = css("--muted");
    g.font = '10px "IBM Plex Sans", system-ui, sans-serif'; g.textBaseline = "middle";
    for (const v of ticks) {
      const y = Math.round(yOf(v)) + 0.5;
      g.beginPath(); g.moveTo(b.x0, y); g.lineTo(b.x1, y); g.stroke();
      if (v === 0) continue;
      if (out) { g.textAlign = "left"; g.fillText(label(v), b.x1 + 4, y); }
      else { g.textAlign = "left"; g.fillText(label(v), b.x0 + 2, y + 7); }
    }
  }
  function series(b, pts, color, w, dash) {
    const { g } = b;
    g.strokeStyle = color; g.lineWidth = w; g.setLineDash(dash); g.lineJoin = "round"; g.lineCap = "round";
    g.beginPath();
    let pen = false;
    for (const [t, v] of pts) {
      if (v == null || !isFinite(v)) { pen = false; continue; }
      const x = b.xOf(t), y = b.yOf(v);
      if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; }
    }
    g.stroke(); g.setLineDash([]);
  }
  function cursorAndHover(b, which, dots) {
    const { g } = b, x = Math.round(b.xOf(T)) + 0.5;
    g.strokeStyle = css("--map-ink"); g.globalAlpha = 0.6; g.lineWidth = 1;
    g.beginPath(); g.moveTo(x, 0); g.lineTo(x, b.H); g.stroke(); g.globalAlpha = 1;
    if (!hover || hover.which !== which) return;
    const r = run.comms.rows[hover.i]; if (!r) return;
    const hx = Math.round(b.xOf(r[0])) + 0.5, surface = css("--surface");
    g.strokeStyle = css("--muted"); g.lineWidth = 1;
    g.beginPath(); g.moveTo(hx, 0); g.lineTo(hx, b.H); g.stroke();
    for (const [v, color] of dots(hover.i)) {
      if (v == null || !isFinite(v)) continue;
      g.fillStyle = color; g.strokeStyle = surface; g.lineWidth = 2;
      g.beginPath(); g.arc(hx, b.yOf(v), 4, 0, 2 * Math.PI); g.fill(); g.stroke();
    }
  }

  function drawLoadChart() {
    const cm = run.comms, rows = cm.rows, b = chartBox(monLoad), { g } = b;
    monBox.load = b;
    const ri = sel >= 0 && idx.hasNodeLoad ? sel + 1 : -1;
    if (!rows.length || !idx.hasLoad) return { yMax: 0, over: 0 };
    let top = cm.cap;
    for (const r of rows) top = Math.max(top, r[3] || 0);
    if (ri >= 0) for (const nr of cm.nodes) if (nr[1]) top = Math.max(top, nr[1][ri] || 0);
    const yMax = top <= 1 ? 1 : Math.ceil(top * 2) / 2;
    b.yOf = (v) => b.y1 - (v / yMax) * (b.y1 - b.y0);
    const step = yMax <= 2 ? 0.5 : yMax <= 4 ? 1 : Math.ceil(yMax / 4);
    const ticks = []; for (let v = 0; v <= yMax + 1e-9; v += step) ticks.push(v);
    grid(b, b.yOf, ticks, (v) => `${Math.round(v * 100)} %`);
    // The seconds some node's channel was past the cap, shaded above it. Enforced,
    // the radio lost or queued the excess; capacity: unlimited, it could not have
    // carried it. A band, not a fill under the line: an excursion is usually one
    // sample, a sliver the line itself covered (fleet_scale, 2026-11-26).
    const capY = b.yOf(cm.cap);
    let over = 0, band = null;
    g.fillStyle = css("--stop"); g.globalAlpha = 0.16;
    const flush = () => { if (band) g.fillRect(band[0], 0, Math.max(1.5, band[1] - band[0]), capY); band = null; };
    for (const r of rows) {
      if ((r[3] || 0) <= cm.cap) { flush(); continue; }
      over++;
      const a = b.xOf(Math.max(0, r[0] - r[1])), z = b.xOf(r[0]);
      if (band && a <= band[1] + 0.5) band[1] = z; else { flush(); band = [a, z]; }
    }
    flush();
    g.globalAlpha = 1;
    g.strokeStyle = css("--stop"); g.lineWidth = 1; g.setLineDash([4, 3]);
    g.beginPath(); g.moveTo(b.x0, Math.round(capY) + 0.5); g.lineTo(b.x1, Math.round(capY) + 0.5); g.stroke(); g.setLineDash([]);
    const s1 = css("--series-1"), s2 = css("--series-2");
    series(b, rows.map((r) => [r[0], r[2]]), s1, 1.5, [4, 3]);
    series(b, rows.map((r) => [r[0], r[3]]), s1, 2, []);
    if (ri >= 0) series(b, cm.nodes.map((n) => [n[0], n[1] ? n[1][ri] : null]), s2, 2, []);
    cursorAndHover(b, "load", (i) => [[rows[i][3], s1], [rows[i][2], s1], ...(ri >= 0 ? [[cm.nodes[i][1]?.[ri], s2]] : [])]);
    return { yMax, over };
  }

  function visibleKinds() {
    const cm = run.comms, out = cm.kinds.map((k, j) => ({ k, o: idx.kOff[j] })).filter((x) => !hiddenKinds.has(x.k));
    return { kinds: out, ogm: measure === "air" && !hiddenKinds.has("ogm") && cm.rows.some((r) => r[5] != null) };
  }
  function drawKindChart() {
    const cm = run.comms, rows = cm.rows, b = chartBox(monKind), { g } = b, M = MEASURE[measure];
    monBox.kind = b;
    if (!rows.length) return { yMax: 0 };
    const { kinds, ogm } = visibleKinds();
    let top = 0;
    for (const r of rows) {
      for (const { o } of kinds) top = Math.max(top, M.at(r, o) || 0);
      if (ogm) top = Math.max(top, (r[5] || 0) * 100);
    }
    const yMax = nice(top);
    b.yOf = (v) => b.y1 - (v / yMax) * (b.y1 - b.y0);
    grid(b, b.yOf, [0, yMax / 2, yMax], (v) => `${fmt(v)}${measure === "air" ? " %" : ""}`);
    for (const { k, o } of kinds) series(b, rows.map((r) => [r[0], M.at(r, o)]), css(kindCol(k)), 2, []);
    const muted = css("--muted");
    if (ogm) series(b, rows.map((r) => [r[0], r[5] == null ? null : r[5] * 100]), muted, 1.5, [4, 3]);
    cursorAndHover(b, "kind", (i) => [...kinds.map(({ k, o }) => [M.at(rows[i], o), css(kindCol(k))]), ...(ogm ? [[(rows[i][5] || 0) * 100, muted]] : [])]);
    return { yMax };
  }

  // The legends are the table at T: built when what they list changes, their numbers
  // refreshed in place (re-rendering them every frame of playback ate clicks).
  function monLegends() {
    const cm = run.comms, M = MEASURE[measure], rid = sel >= 0 ? run.robots[sel] : null;
    const sig = `${run.dir}|${measure}|${sel}|${idx.hasNodeLoad}|${[...hiddenKinds].join(",")}`;
    if (sig !== legSig) {
      legSig = sig;
      $("#rm-load-leg").innerHTML = `<div class="rm-k" data-v="load-h"></div>
        <div class="rm-li"><i style="--c: var(--series-1)"></i><span>busiest node</span><b data-v="max"></b><span class="muted"></span></div>
        <div class="rm-li"><i class="dash" style="--c: var(--series-1)"></i><span>fleet mean</span><b data-v="mean"></b><span class="muted"></span></div>
        ${rid && idx.hasNodeLoad ? `<div class="rm-li"><i style="--c: var(--series-2)"></i><span>${esc(titleOf(rid))}</span><b data-v="robot"></b><span class="muted" data-v="robot-own" title="Airtime its own sends took"></span></div>` : ""}
        <div class="rm-li" title="Seconds of the run in which some node's channel was past the cap"><i class="dash" style="--c: var(--stop)"></i><span>${Math.round(cm.cap * 100)} % cap</span><b data-v="over"></b><span class="muted">over</span></div>`;
      $("#rm-kind-leg").innerHTML = `<div class="rm-k" data-v="kind-h"></div>` + cm.kinds.map((k) =>
        `<button type="button" class="rm-li" data-kind="${esc(k)}" aria-pressed="${!hiddenKinds.has(k)}" title="Show or hide ${esc(k)}"><i style="--c: var(${kindCol(k)})"></i><span>${esc(k)}</span><b data-v="k:${esc(k)}"></b><span class="muted" data-v="l:${esc(k)}" title="Share of its copies lost in this sample"></span></button>`).join("")
        + (measure === "air" && idx.hasAir ? `<button type="button" class="rm-li" data-kind="ogm" aria-pressed="${!hiddenKinds.has("ogm")}" title="batman's routing messages (OGMs): airtime only, no copies"><i class="dash" style="--c: var(--muted)"></i><span>batman OGM</span><b data-v="ogm"></b><span class="muted"></span></button>` : "")
        + (cm.kinds.length ? "" : `<div class="muted">No traffic recorded</div>`);
    }
    const i = sampleAt(), r = i >= 0 ? cm.rows[i] : null, nr = i >= 0 ? cm.nodes[i] : null;
    const at = r ? ` · ${clock(r[0])}` : "";
    const vals = { "load-h": `Channel load${at}`, "kind-h": `${M.title}${at}`, max: pct(r && r[3]), mean: pct(r && r[2]), over: `${Math.round(idx.overS)} s` };
    if (rid) { vals.robot = pct(nr && nr[1] ? nr[1][sel + 1] : null); vals["robot-own"] = nr && nr[2] ? `${pct(nr[2][sel + 1])} own` : ""; }
    const bad = new Set();
    cm.kinds.forEach((k, j) => {
      const o = idx.kOff[j];
      vals[`k:${k}`] = r ? fmt(M.at(r, o)) : "–";
      const sent = r ? r[o] : 0, lost = r ? r[o + 2] : 0;
      vals[`l:${k}`] = sent ? `${pct(lost / sent)} lost` : "";
      if (sent && lost / sent > 0.05) bad.add(`l:${k}`);
    });
    if (r) vals.ogm = r[5] == null ? "–" : fmt(r[5] * 100);
    for (const el of mon.querySelectorAll("[data-v]")) {
      const v = vals[el.dataset.v] ?? "";
      if (el.textContent !== v) el.textContent = v;
      el.classList.toggle("bad", bad.has(el.dataset.v));
    }
    return vals;
  }

  function renderTip() {
    if (!hover || !monBox[hover.which]) { tip.hidden = true; return; }
    const cm = run.comms, r = cm.rows[hover.i], nr = cm.nodes[hover.i];
    if (!r) { tip.hidden = true; return; }
    const line = (v, label, c, dash) => `<div><i class="${dash ? "dash" : ""}" style="--c: var(${c})"></i><b>${esc(v)}</b>${esc(label)}</div>`;
    const M = MEASURE[measure];
    let h = `<div class="tm">${clock(Math.max(0, r[0] - r[1]))}–${clock(r[0])}${hover.which === "load" ? " · channel load" : ` · ${esc(M.title.toLowerCase())}`}</div>`;
    if (hover.which === "load") {
      h += line(pct(r[3]), "busiest node", "--series-1") + line(pct(r[2]), "fleet mean", "--series-1", true);
      if (sel >= 0 && nr && nr[1]) h += line(pct(nr[1][sel + 1]), titleOf(run.robots[sel]), "--series-2");
      if (r[4]) h += line(pct(r[4]), "of nodes past the cap", "--stop", true);
    } else {
      const { kinds, ogm } = visibleKinds();
      for (const { k, o } of kinds) {
        const sent = r[o], lost = r[o + 2];
        h += line(fmt(M.at(r, o)), `${k}${sent ? ` · ${pct((sent - lost) / sent)} delivered` : ""}${r[o + 6] != null ? ` · p95 ${fmt(r[o + 6])} ms` : ""}`, kindCol(k));
      }
      if (ogm) h += line(fmt((r[5] || 0) * 100), "batman OGM", "--muted", true);
    }
    tip.innerHTML = h;
    tip.hidden = false;
    const c = hover.which === "load" ? monLoad : monKind;
    const w = tip.offsetWidth, x = c.offsetLeft + hover.x;
    tip.style.left = `${x + 14 + w > mon.clientWidth ? Math.max(0, x - w - 14) : x + 14}px`;
    tip.style.top = `${Math.max(0, c.offsetTop + 2)}px`;
  }

  function drawMon() {
    monDirty = false;
    if (mon.hidden || !run) return;
    $("#rm-load-row").hidden = !idx.hasLoad;
    const cm = run.comms;
    $("#rm-model").textContent = cm.model === "radio"
      ? `radio · ${cm.contention || "global"} contention${cm.contention === "spatial" ? "" : ": one channel, every node the same load"} · capacity ${cm.capacity}${idx.hasNodes ? "" : " · per-robot traffic not recorded (before record.net_nodes)"}`
      : `${cm.model || "ideal"} network: no airtime, so no channel load${idx.hasNodes ? "" : " · per-robot traffic not recorded"}`;
    for (const bt of mon.querySelectorAll("#rm-measure button")) {
      bt.classList.toggle("active", bt.dataset.m === measure);
      bt.disabled = !measureOk(bt.dataset.m);
      bt.hidden = bt.dataset.m === "air" && !idx.hasAir;
    }
    const L = idx.hasLoad ? drawLoadChart() : { yMax: 0, over: 0 };
    const K = drawKindChart();
    const legend = monLegends();
    renderTip();
    monShown = { T, cursorX: monBox.kind ? monBox.kind.xOf(T) : null, x0: monBox.kind?.x0, x1: monBox.kind?.x1, measure, loadMax: L.yMax, kindMax: K.yMax,
      overSamples: L.over, legend, hover: hover ? { which: hover.which, t: cm.rows[hover.i]?.[0] } : null, tip: tip.hidden ? "" : tip.textContent };
  }

  function setMon(on) {
    showMon = !!on;
    try { localStorage.setItem("sortbots.replayMon", showMon ? "1" : "0"); } catch { /* this tab only */ }
    hover = null; view = null; dirty = true; infoSig = ""; kick();
  }
  function setMeasure(m) {
    if (!MEASURE[m] || (run && !measureOk(m))) return;
    measure = m;
    try { localStorage.setItem("sortbots.replayMeasure", m); } catch { /* this tab only */ }
    hover = null; monDirty = true; kick();
  }
  $("#rp-mon-toggle").addEventListener("click", () => setMon(!showMon));
  $("#rm-x").addEventListener("click", () => setMon(false));
  $("#rm-measure").addEventListener("click", (e) => { const b = e.target.closest("button[data-m]"); if (b && !b.disabled) setMeasure(b.dataset.m); });
  $("#rm-kind-leg").addEventListener("click", (e) => {
    const b = e.target.closest("[data-kind]"); if (!b) return;
    const k = b.dataset.kind;
    if (hiddenKinds.has(k)) hiddenKinds.delete(k); else hiddenKinds.add(k);
    monDirty = true; kick();
  });
  // Hover reads a sample; press and drag scrubs, like the slider.
  for (const [c, which] of [[monLoad, "load"], [monKind, "kind"]]) {
    const pick = (e) => {
      const b = monBox[which]; if (!b || !run || !run.comms.rows.length) return null;
      const x = e.clientX - c.getBoundingClientRect().left, t = b.tOf(x);
      let i = Math.max(0, bisect(idx.commsT, t));
      const nx = run.comms.rows[i + 1];
      if (nx && Math.abs(nx[0] - t) < Math.abs(run.comms.rows[i][0] - t)) i++;
      return { which, i, x, t };
    };
    c.addEventListener("pointermove", (e) => {
      const h = pick(e); if (!h) return;
      hover = h;
      if (scrubbing) seek(h.t);
      monDirty = true; kick();
    });
    c.addEventListener("pointerleave", () => { if (!scrubbing) { hover = null; monDirty = true; kick(); } });
    c.addEventListener("pointerdown", (e) => { const h = pick(e); if (!h) return; scrubbing = true; c.setPointerCapture(e.pointerId); seek(h.t); });
    c.addEventListener("pointerup", () => { scrubbing = false; });
    c.addEventListener("pointercancel", () => { scrubbing = false; hover = null; monDirty = true; kick(); });
  }

  // -- side panel -------------------------------------------------------------------

  function renderInfo() {
    if (!run) return;
    const b = at(run.board, idx.boardT), nt = at(run.net, idx.netT), bl = at(run.belief, idx.beliefT);
    const order = ["announced", "claimed", "assigned", "active", "done", "failed", "unassigned"];
    let board = "No board sample yet";
    if (b) {
      const ks = Object.keys(b[1]).sort((x, y) => (order.indexOf(x) + 99) % 99 - (order.indexOf(y) + 99) % 99);
      board = ks.length ? ks.map((k) => `<span><b>${b[1][k]}</b> ${esc(words(k))}</span>`).join("") : "No tasks";
      if (b[2]) board += `<span class="bad"><b>${b[2]}</b> alarm${b[2] === 1 ? "" : "s"}</span>`;
      board += `<span><b>${b[3] ?? "?"}</b> alive</span>`;
    }
    let net = "";
    if (nt) {
      const parts = [`<span><b>${Math.round((nt[3] || 0) * 1000) / 10}%</b> lost</span>`, `<span><b>${(nt[4] / 1000).toFixed(0)}</b> kB/s</span>`];
      const cr = idx.hasLoad ? at(run.comms.rows, idx.commsT) : null;
      if (cr && cr[2] != null) parts.unshift(`<span><b>${Math.round(cr[2] * 100)}%</b> mean load</span>`);
      if (run.network === "radio") parts.unshift(`<span${(nt[1] || 0) > run.comms.cap ? ' class="bad"' : ""}><b>${Math.round((nt[1] || 0) * 100)}%</b> peak load</span>`, `<span><b>${Math.round((nt[2] || 0) * 100)}%</b> reach</span>`);
      net = parts.join("");
    }
    let robot = "";
    if (sel >= 0) {
      const rid = run.robots[sel], m = modeOf(sel), st = stateOf(sel);
      const task = st && st[2] ? run.tasks[st[2]] : null;
      const row = bl && bl[1][sel];
      const hasOwn = (run.belief_maps[rid] || []).length > 0;
      robot = `<div class="rp-k">${esc(titleOf(rid))} <span class="rp-mode" style="--c: var(${MODE_COL[m]})">${esc(MODE_TXT[m])}</span></div>
        <div class="rp-row">${task ? `<span>${esc(st[2])}: ${esc(words(task.pickup))} → ${esc(words(task.dropoff))}</span>` : "<span>No task</span>"}</div>
        ${row ? `<div class="rp-row"><span><b>${Math.round(row[0] * 100)}%</b> of the floor known</span><span><b>${row[1]}</b> cells behind the fleet</span></div>` : ""}
        <div class="rp-row muted">${hasOwn ? (root.dataset.own ? "Floor shows its own map; red = wrong" : "Its own map is recorded: tick Own map") : "Own map not recorded (record.belief_maps)"}</div>
        ${robotTraffic(sel)}`;
    }
    const m1 = bisect(idx.markT, T), feed = [];
    for (let k = m1; k >= 0 && feed.length < FEED_N; k--) {
      const mk = run.marks[k];
      if (sel >= 0 && mk[2] && mk[2] !== run.robots[sel]) continue;
      feed.push(`<li style="--c: var(${(MARK[mk[1]] || ["--muted"])[0]})"><span class="tm">${clock(mk[0])}</span>${mk[2] ? `<b>${esc(short(mk[2]))}</b> ` : ""}${esc(mk[3])}</li>`);
    }
    const html = `<div class="rp-k">Task board</div><div class="rp-row">${board}</div>
      ${net ? `<div class="rp-k">Network · ${esc(run.network)}</div><div class="rp-row">${net}</div>` : ""}
      ${robot}
      <div class="rp-k">${sel >= 0 ? "Its events" : "Events"}</div><ol class="rp-feed">${feed.join("") || "<li class=\"muted\">Nothing yet</li>"}</ol>`;
    if (html !== infoSig) { infoSig = html; $("#rp-info").innerHTML = html; }
  }

  // The selected robot's own traffic in the sample at T: its channel's load, the
  // airtime its own sends took, and per kind what it sent and heard (record.net_nodes).
  function robotTraffic(i) {
    if (!idx.hasNodes) return `<div class="rp-k">Its traffic</div><div class="rp-row muted">Not recorded for this run (record.net_nodes, since 2026-11-26)</div>`;
    const j = bisect(idx.commsT, T);
    if (j < 0) return `<div class="rp-k">Its traffic</div><div class="rp-row muted">No sample yet</div>`;
    const cm = run.comms, r = cm.rows[j], nr = cm.nodes[j], node = i + 1, dt = r[1];
    const head = [];
    if (nr[1]) head.push(`<span${nr[1][node] > cm.cap ? ' class="bad"' : ""}><b>${pct(nr[1][node])}</b> load</span>`);
    if (nr[2]) head.push(`<span title="Airtime its own sends took, OGMs included, as a share of the channel"><b>${pct(nr[2][node])}</b> its own airtime</span>`);
    const v = (a) => (a ? a[node] || 0 : 0);
    const rows = cm.kinds.map((k, q) => {
      const per = nr[3] && nr[3][q];
      if (!per) return "";
      const [sent, bytes, lost, , rxb] = per, n = v(sent), l = v(lost);
      if (!n && !v(rxb)) return "";
      return `<tr><td><i style="--c: var(${kindCol(k)})"></i>${esc(k)}</td><td>${fmt(v(bytes) / dt / 1000)}</td><td>${fmt(v(rxb) / dt / 1000)}</td><td${n && l / n > 0.05 ? ' class="bad"' : ""}>${n ? pct(l / n) : "–"}</td></tr>`;
    }).join("");
    return `<div class="rp-k">Its traffic · ${clock(Math.max(0, r[0] - dt))}–${clock(r[0])}</div>
      ${head.length ? `<div class="rp-row">${head.join("")}</div>` : ""}
      ${rows ? `<table class="rp-tr"><tr><th></th><th title="Bytes it offered per second, every copy">out kB/s</th><th title="Bytes delivered to it per second">in kB/s</th><th title="Share of the copies it sent that were lost">lost</th></tr>${rows}</table>` : `<div class="rp-row muted">Sent and heard nothing</div>`}`;
  }

  function fillBar() {
    const r = run;
    $("#rp-name").textContent = titleOf(r.name);
    $("#rp-name").title = `${r.title}${r.description ? `\n\n${r.description}` : ""}\n\n${r.dir}`;
    const verdict = r.score ? (r.score.ok ? "PASS" : "FAIL") : "";
    const v = $("#rp-verdict");
    v.textContent = verdict; v.hidden = !verdict; v.className = `rp-v ${verdict === "PASS" ? "pass" : "fail"}`;
    const bench = /^bench\/([^/]+)\//.exec(r.dir || "");
    $("#rp-meta").textContent = `${r.robots.length} robots · ${r.network} · seed ${r.seed}${bench ? ` · bench ${words(bench[1].replace(/^\d{8}T\d{6}Z_/, ""))}` : ""}${r.truncated ? " · ended early" : ""}`;
    $("#rp-meta").title = r.truncated ? "No end event: the run crashed or was still running when this was read (docs/fleetsim.md, Known issues)" : "";
    $("#rp-links").disabled = !r.links.length;
    $("#rp-links").closest("label").title = r.links.length ? "One-hop radio links, darker = stronger" : "Recorded on radio-network runs only: an ideal network links everyone";
    $("#rp-load").disabled = !idx.hasNodeLoad;
    $("#rp-load").checked = show.load && idx.hasNodeLoad;
    $("#rp-load").closest("label").title = idx.hasNodeLoad
      ? "Ring each robot, and colour each radio link (the busier of its two ends), by its channel's load: faint = idle, blue = near the 95 % cap, red = past it"
      : run.network === "radio" ? "This run was recorded before per-node load (record.net_nodes, 2026-11-26)" : "An ideal network has no channel load";
    $("#rp-sel").innerHTML = `<option value="-1">No robot</option>` + r.robots.map((rid, i) => `<option value="${i}">${esc(titleOf(rid))}${(r.belief_maps[rid] || []).length ? " · own map" : ""}</option>`).join("");
    $("#rp-sel").value = String(sel);
    requestAnimationFrame(drawTicks); // a new run's marks
  }

  // -- control ----------------------------------------------------------------------

  function seek(t) {
    if (!run) return;
    T = Math.max(0, Math.min(run.t_end, t));
    slider.value = String(T);
    $("#rp-clock").textContent = `${clock(T)} / ${clock(run.t_end)}`;
    dirty = true;
    kick();
  }
  function select(i) {
    sel = i;
    $("#rp-sel").value = String(i);
    dirty = true; infoSig = ""; kick();
  }
  function setPlaying(on) {
    if (on && run && T >= run.t_end - 0.05) seek(0);
    playing = !!on && !!run;
    $("#rp-play").textContent = playing ? "Pause" : "Play";
    $("#rp-play").setAttribute("aria-pressed", String(playing));
    lastTick = performance.now();
    kick();
  }
  function stop() { playing = false; if (raf) cancelAnimationFrame(raf); raf = 0; }
  function kick() { if (!raf && !root.hidden) raf = requestAnimationFrame(frame); }
  function frame(now) {
    raf = 0;
    if (root.hidden) return;
    if (playing && run) {
      const dt = Math.min(0.25, (now - lastTick) / 1000);
      seek(T + dt * SPEEDS[speedI]);
      if (T >= run.t_end) setPlaying(false);
    }
    lastTick = now;
    if (dirty) { draw(); renderInfo(); drawMon(); }
    else if (monDirty) drawMon();
    if (playing || dirty || monDirty) kick();
  }
  function jumpMark(dir) {
    if (!run || !run.marks.length) return;
    const ms = run.marks.filter((m) => m[1] !== "done" && (sel < 0 || m[2] === run.robots[sel]));
    // A jump lands 1 s before the mark, so the mark just jumped to sits at T + 1.
    const next = dir > 0 ? ms.find((m) => m[0] > T + 1.05) : [...ms].reverse().find((m) => m[0] < T + 0.95);
    if (next) seek(Math.max(0, next[0] - 1));
  }

  slider.addEventListener("input", () => { seek(Number(slider.value)); });
  $("#rp-play").addEventListener("click", () => setPlaying(!playing));
  $("#rp-speed").addEventListener("change", (e) => { speedI = Number(e.target.value); });
  $("#rp-prev").addEventListener("click", () => jumpMark(-1));
  $("#rp-next").addEventListener("click", () => jumpMark(1));
  $("#rp-x").addEventListener("click", close);
  $("#rp-sel").addEventListener("change", (e) => select(Number(e.target.value)));
  $("#rp-paths").addEventListener("change", (e) => { show.paths = e.target.checked; dirty = true; kick(); });
  $("#rp-links").addEventListener("change", (e) => { show.links = e.target.checked; dirty = true; kick(); });
  $("#rp-load").addEventListener("change", (e) => { show.load = e.target.checked; dirty = true; kick(); });
  $("#rp-own").addEventListener("change", (e) => { show.own = e.target.checked; dirty = true; infoSig = ""; kick(); });
  $("#rp-info-toggle").addEventListener("click", () => {
    showInfo = !showInfo;
    $("#rp-info-toggle").setAttribute("aria-pressed", String(showInfo));
    dirty = true; infoSig = ""; kick();
  });
  $("#rp-info-toggle").setAttribute("aria-pressed", String(showInfo));
  $("#rp-fit").addEventListener("click", () => { view = null; dirty = true; kick(); });
  $("#rp-speed").innerHTML = SPEEDS.map((s, i) => `<option value="${i}"${i === speedI ? " selected" : ""}>${s}×</option>`).join("");

  // Pan (drag), zoom (wheel, at the cursor), pick (click a robot).
  let drag = null;
  cv.addEventListener("pointerdown", (e) => { if (!view) return; drag = { x: e.clientX, y: e.clientY, vx: view.x, vy: view.y, moved: false }; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (Math.abs(dx) + Math.abs(dy) > 4) drag.moved = true;
    if (drag.moved) { view.x = drag.vx + dx; view.y = drag.vy + dy; dirty = true; kick(); }
  });
  cv.addEventListener("pointerup", (e) => {
    const d = drag; drag = null;
    if (!d || d.moved || !run) return;
    const r = cv.getBoundingClientRect(), [wx, wy] = world(e.clientX - r.left, e.clientY - r.top);
    let best = -1, bd = Math.max(0.6, 12 / view.s);
    run.robots.forEach((_, i) => { const p = poseAt(i); if (p) { const d2 = Math.hypot(p[0] - wx, p[1] - wy); if (d2 < bd) { bd = d2; best = i; } } });
    select(best);
  });
  cv.addEventListener("wheel", (e) => {
    if (!view) return;
    e.preventDefault();
    const r = cv.getBoundingClientRect(), a = e.clientX - r.left, b = e.clientY - r.top;
    const k = Math.exp(-e.deltaY * 0.0015);
    view.x = a - (a - view.x) * k; view.y = b - (b - view.y) * k; view.s *= k;
    dirty = true; kick();
  }, { passive: false });
  cv.addEventListener("dblclick", () => { view = null; dirty = true; kick(); });

  document.addEventListener("keydown", (e) => {
    if (root.hidden || !run || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.target.closest && e.target.closest("input:not([type=range]):not([type=checkbox]), select, textarea")) return;
    const step = e.shiftKey ? 10 : 1;
    if (e.key === " ") { e.preventDefault(); setPlaying(!playing); }
    else if (e.key === "ArrowRight") { e.preventDefault(); seek(T + step); }
    else if (e.key === "ArrowLeft") { e.preventDefault(); seek(T - step); }
    else if (e.key === "]") jumpMark(1);
    else if (e.key === "[") jumpMark(-1);
    else if (e.key === "t" || e.key === "T") setMon(!showMon);
    else if (e.key === "Escape") close();
  });

  new ResizeObserver(() => { if (!root.hidden) { view = null; dirty = true; drawTicks(); kick(); } }).observe(root);
  // A theme switch recolours the floor: the grids are rendered once per theme.
  const recolour = () => { if (truth) { renderFloor(); drawTicks(); infoSig = ""; kick(); } };
  new MutationObserver(recolour).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
  matchMedia("(prefers-color-scheme: dark)").addEventListener?.("change", recolour);
  // The ticks canvas has no size until the replay is on screen.
  new MutationObserver(() => { if (!root.hidden) { drawTicks(); kick(); } }).observe(root, { attributes: true, attributeFilter: ["hidden"] });

  // -- the Tests tab's list of runs -----------------------------------------------------

  const list = $("#tp-fleet-list");
  const ago = (mtime) => {
    const s = Date.now() / 1000 - mtime;
    return s < 90 ? "just now" : s < 5400 ? `${Math.round(s / 60)} min ago` : s < 129600 ? `${Math.round(s / 3600)} h ago` : `${Math.round(s / 86400)} d ago`;
  };
  function runRow(r, inBench) {
    const why = !r.replayable ? (r.finished ? "Recorded before replays existed: re-run it (same seed, same run)" : "Still running, or it crashed before recording its floor") : "Replay this run on its floor";
    const v = r.verdict ? `<span class="rp-v ${r.verdict === "PASS" ? "pass" : "fail"}">${r.verdict}</span>` : "";
    // A bench run's name is <entry>_r<N>_s<seed>_<variant>; its variant and seed are
    // what tell it from its neighbours, so they lead.
    const meta = (inBench ? [r.variant, r.seed != null && `seed ${r.seed}`] : []).concat([r.robots && `${r.robots} robots`, r.network, simTime(r.sim_s),
      inBench && r.status && !["pass", "fail"].includes(r.status) && r.status, !r.finished && "unfinished", !inBench && ago(r.mtime)]).filter(Boolean).join(" · ");
    return `<div class="fs-run" data-dir="${esc(r.dir)}"><div class="fs-t"><b>${esc(titleOf(r.name))}</b> ${v}<span class="muted">${esc(meta)}</span></div>
      <button type="button" class="btn small" data-replay ${r.replayable ? "" : "disabled"} title="${esc(why)}">Replay</button></div>`;
  }
  function benchBody(b) {
    const got = benchRuns.get(b.dir);
    if (!got) return `<div class="tp-empty">Loading…</div>`;
    if (got.err) return `<div class="tp-empty">Couldn't list it: ${esc(got.err)}</div>`;
    return got.runs.length ? got.runs.map((r) => runRow(r, true)).join("") : `<div class="tp-empty">No recorded runs in it</div>`;
  }
  function renderRuns() {
    if (!list) return;
    $("#tp-fleet-count").textContent = runs.length || benches.length ? `· ${runs.length}${benches.length ? ` · ${benches.length} bench${benches.length === 1 ? "" : "es"}` : ""}` : "";
    if (runsErr) { list.innerHTML = `<div class="tp-empty">Fleet sim runs unavailable: ${esc(runsErr)}</div>`; return; }
    let html = runs.length ? runs.map((r) => runRow(r, false)).join("")
      : `<div class="tp-empty">No runs yet. <code>python3 -m fleetsim run delivery_smoke</code> makes one (seconds, no sim).</div>`;
    if (benches.length) {
      html += `<h5 class="fs-bh">Benchmarks</h5>` + benches.map((b) => {
        const counts = Object.entries(b.counts || {}).filter(([k]) => k !== "pass").map(([k, n]) => `${n} ${k}`);
        const meta = [`${b.runs} run${b.runs === 1 ? "" : "s"}`, ...counts, !b.complete && "unfinished", ago(b.mtime)].filter(Boolean).join(" · ");
        return `<details class="fs-bench" data-bench="${esc(b.dir)}"${openBenches.has(b.dir) ? " open" : ""}>
          <summary title="${esc(`${b.dir}${b.description ? `\n\n${b.description}` : ""}`)}"><span class="fs-t"><b>${esc(titleOf(b.suite))}</b><span class="muted">${esc(meta)}</span></span></summary>
          ${openBenches.has(b.dir) ? benchBody(b) : ""}</details>`;
      }).join("");
    }
    list.innerHTML = html;
  }
  async function loadBench(name, mtime) {
    try {
      const res = await fetch(`/api/fleetsim/bench?dir=${encodeURIComponent(name)}`, { cache: "no-store" });
      const body = await res.json();
      if (!res.ok) throw new Error(body.error || res.statusText);
      benchRuns.set(name, { runs: body.runs || [], mtime });
    } catch (e) { benchRuns.set(name, { runs: [], err: e.message, mtime }); }
    renderRuns();
  }
  async function refreshRuns(force) {
    if (!list || (!force && Date.now() - runsAt < RUNS_REFRESH_MS)) return;
    runsAt = Date.now();
    try {
      const res = await fetch("/api/fleetsim/runs", { cache: "no-store" });
      const body = await res.json();
      if (!res.ok) throw new Error(body.error || res.statusText);
      runs = body.runs || []; benches = body.benches || []; runsErr = "";
    } catch (e) { runsErr = e.message; }
    renderRuns();
    // An open bench that is still writing runs is listed again; a finished one isn't.
    for (const b of benches) {
      const got = benchRuns.get(b.dir);
      if (openBenches.has(b.dir) && got && got.mtime !== b.mtime) loadBench(b.dir, b.mtime);
    }
  }
  if (list) {
    list.addEventListener("click", (e) => {
      const b = e.target.closest("[data-replay]");
      if (!b || b.disabled) return;
      const dir = b.closest(".fs-run").dataset.dir;
      $("#sp-x")?.click(); // the stage is where a replay is watched
      open(dir);
    });
    // toggle doesn't bubble: caught on the way down. A bench's runs are fetched the
    // first time it opens.
    list.addEventListener("toggle", (e) => {
      const d = e.target;
      if (!d.matches || !d.matches("details.fs-bench")) return;
      const name = d.dataset.bench, b = benches.find((x) => x.dir === name);
      if (d.open === openBenches.has(name)) return; // a re-render restoring it
      if (d.open) {
        openBenches.add(name);
        if (!benchRuns.has(name)) { renderRuns(); loadBench(name, b && b.mtime); } else renderRuns();
      } else openBenches.delete(name);
    }, true);
    const card = $("#session .card"), panel = $("#session");
    const check = () => { if (panel && !panel.hidden && card && card.classList.contains("tab-tests")) refreshRuns(false); };
    if (panel) new MutationObserver(check).observe(panel, { attributes: true, attributeFilter: ["hidden"] });
    if (card) new MutationObserver(check).observe(card, { attributes: true, attributeFilter: ["class"] });
  }

  // A shared link (#replay=DIR) opens straight into the run, on load or when
  // pasted into a tab that already has the page (a hash change, no reload).
  const fromHash = () => {
    const m = /^#replay=(.+)$/.exec(location.hash);
    if (m && !(run && run.dir === decodeURIComponent(m[1]) && !root.hidden)) open(decodeURIComponent(m[1]));
  };
  window.addEventListener("hashchange", fromHash);
  fromHash();

  window.sortbotsReplay = {
    open, close, seek, select, play: setPlaying, refreshRuns: () => refreshRuns(true),
    monitor: setMon, measure: setMeasure, load: (on) => { show.load = !!on; $("#rp-load").checked = show.load; dirty = true; kick(); },
    state: () => ({
      open: !root.hidden, loaded: !!run && !!floorCv, T, playing, sel, speed: SPEEDS[speedI],
      own: !!root.dataset.own, frames: window.__replayFrames || 0,
      pose: run && sel >= 0 ? poseAt(sel) : null, mode: run && sel >= 0 ? modeOf(sel) : null,
      cut: run ? [...cutOff(activeSpans())] : [], runs: runs.map((r) => r.dir),
      benches: benches.map((b) => b.dir), benchRuns: Object.fromEntries([...benchRuns].map(([k, v]) => [k, (v.runs || []).map((r) => r.dir)])),
      mon: mon.hidden ? null : monShown, load: show.load, loadLinks: loadLinks.slice(), loadNode: run && sel >= 0 ? nodeRow()?.[1]?.[sel + 1] ?? null : null,
      comms: run ? { kinds: run.comms.kinds, model: run.comms.model, hasNodeLoad: idx.hasNodeLoad, hasNodes: idx.hasNodes } : null,
      toPx: (x, y) => (view ? px(x, y) : null),
    }),
  };
})();
