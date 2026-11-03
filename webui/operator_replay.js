// SortBots operator view — fleet replay: a recorded fleetsim run (fleetsim/) drawn
// on its own floor, with a scrubber. Opened from the Session panel's Tests tab
// ("Fleet sim runs"); it covers the stage until closed and never touches the live
// fleet, the robot cards or the task list — none of what it shows is real.
//
// Talks ONLY to webui/serve.py:
//   GET /api/fleetsim/runs           recent runs in data/fleetsim/ (fleetsim/replay.py list_runs)
//   GET /api/fleetsim/run?dir=NAME   one run, packed whole (fleetsim/replay.py pack)
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
  const FETCH = new Set(["NAV_TO_PICKUP", "PICKING"]);

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
  const show = { paths: true, links: true, own: true };
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
    truthKey = ""; truthNow = null;
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
    const g = run.world.grid, W = cv.clientWidth, H = cv.clientHeight;
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
    info.hidden = !showInfo || !run;
    if (phone.matches) { info.style.top = ""; return; } // docked above the timeline (CSS)
    info.style.top = `${bar.offsetTop + bar.offsetHeight + 8}px`;
  }

  function draw() {
    dirty = false;
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

    // Radio links: one-hop, alpha by SNR. Drawn under everything that moves.
    if (show.links && run.links.length) {
      const row = at(run.links, idx.linksT);
      if (row) {
        const posOf = (node) => (node === 0 ? gw : poses[node - 1]);
        ctx.strokeStyle = css("--info"); ctx.lineWidth = 1;
        for (let k = 0; k < row[1].length; k += 3) {
          const A = posOf(row[1][k]), B = posOf(row[1][k + 1]);
          if (!A || !B) continue;
          ctx.globalAlpha = Math.max(0.08, Math.min(0.55, (row[1][k + 2] - 5) / 40));
          const [a, b] = px(A[0], A[1]), [c2, d] = px(B[0], B[1]);
          ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c2, d); ctx.stroke();
        }
        ctx.globalAlpha = 1;
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
      if (run.network === "radio") parts.unshift(`<span><b>${Math.round((nt[1] || 0) * 100)}%</b> peak load</span>`, `<span><b>${Math.round((nt[2] || 0) * 100)}%</b> reach</span>`);
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
        <div class="rp-row muted">${hasOwn ? (root.dataset.own ? "Floor shows its own map; red = wrong" : "Its own map is recorded: tick Own map") : "Own map not recorded (record.belief_maps)"}</div>`;
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

  function fillBar() {
    const r = run;
    $("#rp-name").textContent = titleOf(r.name);
    $("#rp-name").title = `${r.title}${r.description ? `\n\n${r.description}` : ""}\n\n${r.dir}`;
    const verdict = r.score ? (r.score.ok ? "PASS" : "FAIL") : "";
    const v = $("#rp-verdict");
    v.textContent = verdict; v.hidden = !verdict; v.className = `rp-v ${verdict === "PASS" ? "pass" : "fail"}`;
    $("#rp-meta").textContent = `${r.robots.length} robots · ${r.network} · seed ${r.seed}${r.truncated ? " · ended early" : ""}`;
    $("#rp-meta").title = r.truncated ? "No end event: the run crashed or was still running when this was read (fleetsim/README.md, Known issues)" : "";
    $("#rp-links").disabled = !r.links.length;
    $("#rp-links").closest("label").title = r.links.length ? "One-hop radio links, darker = stronger" : "Recorded on radio-network runs only: an ideal network links everyone";
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
    if (dirty) { draw(); renderInfo(); }
    if (playing || dirty) kick();
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
  function renderRuns() {
    if (!list) return;
    $("#tp-fleet-count").textContent = runs.length ? `· ${runs.length}` : "";
    if (runsErr) { list.innerHTML = `<div class="tp-empty">Fleet sim runs unavailable: ${esc(runsErr)}</div>`; return; }
    list.innerHTML = runs.length ? runs.map((r) => {
      const why = !r.replayable ? (r.finished ? "Recorded before replays existed: re-run it (same seed, same run)" : "Still running, or it crashed before recording its floor") : "Replay this run on its floor";
      const v = r.verdict ? `<span class="rp-v ${r.verdict === "PASS" ? "pass" : "fail"}">${r.verdict}</span>` : "";
      const meta = [r.robots && `${r.robots} robots`, r.network, simTime(r.sim_s), !r.finished && "unfinished", ago(r.mtime)].filter(Boolean).join(" · ");
      return `<div class="fs-run" data-dir="${esc(r.dir)}"><div class="fs-t"><b>${esc(titleOf(r.name))}</b> ${v}<span class="muted">${esc(meta)}</span></div>
        <button type="button" class="btn small" data-replay ${r.replayable ? "" : "disabled"} title="${esc(why)}">Replay</button></div>`;
    }).join("") : `<div class="tp-empty">No runs yet. <code>python3 -m fleetsim run delivery_smoke</code> makes one (seconds, no sim).</div>`;
  }
  async function refreshRuns(force) {
    if (!list || (!force && Date.now() - runsAt < RUNS_REFRESH_MS)) return;
    runsAt = Date.now();
    try {
      const res = await fetch("/api/fleetsim/runs", { cache: "no-store" });
      const body = await res.json();
      if (!res.ok) throw new Error(body.error || res.statusText);
      runs = body.runs || []; runsErr = "";
    } catch (e) { runsErr = e.message; }
    renderRuns();
  }
  if (list) {
    list.addEventListener("click", (e) => {
      const b = e.target.closest("[data-replay]");
      if (!b || b.disabled) return;
      const dir = b.closest(".fs-run").dataset.dir;
      $("#sp-x")?.click(); // the stage is where a replay is watched
      open(dir);
    });
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
    state: () => ({
      open: !root.hidden, loaded: !!run && !!floorCv, T, playing, sel, speed: SPEEDS[speedI],
      own: !!root.dataset.own, frames: window.__replayFrames || 0,
      pose: run && sel >= 0 ? poseAt(sel) : null, mode: run && sel >= 0 ? modeOf(sel) : null,
      cut: run ? [...cutOff(activeSpans())] : [], runs: runs.map((r) => r.dir),
      toPx: (x, y) => (view ? px(x, y) : null),
    }),
  };
})();
