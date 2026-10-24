// commsim replay / live demo — see index.html and demo.html.
//
// Two sources, one renderer:
//   replay  load(trace)      one recorded emulation run (build_trace.py),
//                            scrubbed on a timeline
//   live    live("/events")  the DES stepped in real time by demo.py, events
//                            streamed in; the viewer drags robots and fires
//                            obstacles, tasks, link pulls, kills, messages
//
// Everything is indexed into flat arrays so a 100k-event trace scrubs at
// 60 fps: the per-frame work is a binary search for the active window and
// a draw of what is in it. Time inside the page is `T`, integer microseconds
// (trace: since header.t0_ns; live: sim time). Visible hop durations are
// stretched to MIN_VIS_MS of WALL time at the current speed, because a 400 µs
// hop is invisible otherwise and seeing it is the point.

window.__pageErrors = [];
window.addEventListener("error", (e) => window.__pageErrors.push(String(e.message)));
window.addEventListener("unhandledrejection", (e) => window.__pageErrors.push(String(e.reason)));

(function () {
  "use strict";

  // kind -> [colour, short label on the dot, group]
  const KIND_DEF = {
    status:     ["#4cc9f0", "",     "fleet"],
    announce:   ["#ffd166", "TASK", "task"],
    bid:        ["#f4a261", "BID",  "task"],
    claim:      ["#06d6a0", "CLAIM", "task"],
    lease:      ["#8ac926", "lease", "task"],
    event:      ["#ff9f1c", "done", "task"],
    obstacle:   ["#ff5d8f", "OBST", "fleet"],
    custom:     ["#ffffff", "MSG",  "fleet"],
    map:        ["#9d4edd", "map",  "fleet"],
    linkhealth: ["#577590", "",     "fleet"],
    ka:         ["#3a506b", "",     "transport"],
    zenoh:      ["#c77dff", "",     "transport"],
    tcp_ack:    ["#7a5cff", "",     "transport"],
    icmp:       ["#ffd166", "ping", "transport"],
    ogm:        ["#5c636a", "",     "mesh"],
    bcast:      ["#8d99ae", "",     "mesh"],
    arp:        ["#8d99ae", "arp",  "mesh"],
    mgmt:       ["#495057", "",     "mesh"],
    frag:       ["#f4a261", "",     "transport"],
    other:      ["#adb5bd", "",     "mesh"],
  };
  const KINDS = Object.keys(KIND_DEF);
  const DEFAULT_HIDDEN = new Set(["ogm", "mgmt", "arp", "bcast", "tcp_ack", "other", "ka", "linkhealth", "map", "frag"]);
  const MIN_VIS_MS = 140;
  const MAX_VIS_US = 1_500_000;
  const FLASH_MS = 400;
  const BCAST_RANGE_M = 8;
  const PANEL_MS = 250;
  const LIVE_LAG_US = 150_000;   // draw this far behind the server so hops arrive before they animate
  const LIVE_KEEP_US = 90_000_000;
  const OBSTACLE_FADE_US = 12_000_000;

  const $ = (id) => document.getElementById(id);
  const floor = $("floor"), tl = $("timeline"), ctx = floor.getContext("2d"), tctx = tl.getContext("2d");
  const kindIdx = {};
  KINDS.forEach((k, i) => { kindIdx[k] = i; });

  // ---------------------------------------------------------------- state
  let trace = null, live = false, es = null;
  let A = null;            // air events: parallel arrays
  let S = [];              // state-changing events (replay: in time order; live: applied on arrival)
  let si = 0;
  let T = 0, endT = 0, liveT = 0;
  let playing = false, speed = 1, lastWall = 0, lastPanel = 0;
  let selected = null, routeMode = "gateway";
  const hidden = new Set(DEFAULT_HIDDEN);
  let routes = new Map(), blocked = new Set(), silent = new Map(), dead = new Set();
  let flashes = [], tasks = new Map(), obstacles = [], owned = {}, feed = [];
  let staticLayer = null, staticKey = "", world = null;
  let drawn = { packets: {}, flashes: 0, blocked: 0, routes: 0 };
  // interaction
  let drag = null;         // {id, x, y, moved, lastPost}
  let captionTimer = null;
  let pick = null;         // {op, need, got:[], hint}

  function bisect(arr, x) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] < x) lo = m + 1; else hi = m; } return lo; }
  function emptyA() { return { t: [], end: [], ta: [], ra: [], kind: [], ack: [], ok: [], ev: [], rx: new Map(), tx: new Map() }; }
  function pushAir(e) {
    A.t.push(e.t); A.end.push(e.t + (e.dur || 0)); A.ta.push(e.ta); A.ra.push(e.ra);
    A.kind.push(kindIdx[e.kind] ?? kindIdx.other);
    A.ack.push(e.ack === true ? 1 : e.ack === false ? 0 : 2);
    A.ok.push(e.ok === "ok" ? 1 : e.ok === "lost" ? 2 : e.ok === "lost_after" ? 3 : 0);
    A.ev.push(e);
  }

  // ---------------------------------------------------------------- load (replay)
  async function loadFromUrl(url) {
    const r = await fetch(url);
    if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
    let body = r.body;
    if (/\.gz($|\?)/.test(url) || (r.headers.get("content-type") || "").includes("gzip")) body = body.pipeThrough(new DecompressionStream("gzip"));
    load(JSON.parse(await new Response(body).text()));
  }
  async function loadFromFile(file) {
    let blob = file;
    if (/\.gz$/.test(file.name)) blob = new Response(file.stream().pipeThrough(new DecompressionStream("gzip")));
    load(JSON.parse(await blob.text()));
  }

  function setHeader(h, isLive) {
    trace = { header: h }; live = isLive;
    A = emptyA(); S = []; si = 0; resetState(); flashes = [];
    tasks = new Map(); obstacles = []; owned = {}; feed = [];
    for (const nd of h.nodes) if (nd.alive === false) dead.add(nd.id);
    for (const t of h.tasks || []) tasks.set(t.task_id, t);
    staticLayer = null; selected = null;
    $("name").textContent = `${h.name} · n=${h.n} · ${h.transport}`;
    const hint = $("hint"); if (hint) hint.style.display = "none";
    buildFilters(); buildLegend();
  }

  function load(obj) {
    if (!obj || obj.format !== "commsim-viz/1" || !obj.header || !Array.isArray(obj.events)) throw new Error("not a commsim-viz/1 trace");
    if (es) { es.close(); es = null; }
    setHeader(obj.header, false);
    for (const e of obj.events) {
      if (e.k === "air") pushAir(e);
      else if (e.k === "app_rx") { const k = `${e.dst},${e.src}`; (A.rx.get(k) || A.rx.set(k, []).get(k)).push(e); }
      else if (e.k === "app_tx") { (A.tx.get(e.src) || A.tx.set(e.src, []).get(e.src)).push(e.t); }
      else S.push(e);
    }
    for (const [k, v] of A.rx) A.rx.set(k, { t: v.map((e) => e.t), seq: v.map((e) => e.seq) });
    endT = obj.events.length ? Math.max(obj.events[obj.events.length - 1].t, A.end.length ? A.end[A.end.length - 1] : 0) : 0;
    if (obj.header.duration_s) endT = Math.max(endT, obj.header.duration_s * 1e6);
    trace.tour = obj.header.tour || [];
    buildTour();
    seek(0); pause(); renderPanel(); drawAll();
  }

  // ---------------------------------------------------------------- live
  function liveStart(url) {
    if (es) es.close();
    es = new EventSource(url);
    es.onmessage = (m) => { let e; try { e = JSON.parse(m.data); } catch { return; } onLive(e); };
    es.onerror = () => { $("name").textContent = "demo server: reconnecting…"; };
  }

  function onLive(e) {
    switch (e.k) {
      case "init":
        setHeader(e.header, true);
        speed = e.speed || 1; const sp = $("speed"); if (sp) sp.value = String(speed);
        T = liveT = 0; endT = 0; playing = !e.paused;
        pushFeed("sys", `simulation ${e.header.n - 1} robots + gateway, ${e.header.variant}`);
        syncControls(); renderPanel(); drawAll();
        if (!window.__liveLoop) { window.__liveLoop = true; requestAnimationFrame(liveFrame); }
        return;
      case "ctl": speed = e.speed; playing = !e.paused; syncControls(); return;
      case "error": pushFeed("error", e.text); return;
      case "air": pushAir(e); return;
      case "now":
        liveT = e.t; endT = e.t; owned = e.owned || owned; return;
      case "app_rx": {
        const k = `${e.dst},${e.src}`; let r = A.rx.get(k); if (!r) A.rx.set(k, (r = { t: [], seq: [] }));
        r.t.push(e.t); r.seq.push(e.hops);
        if (e.kind !== "status" && e.kind !== "ka" && e.kind !== "linkhealth" && e.kind !== "map") {
          pushFeed("rx", `${name(e.dst)} got ${e.type} from ${name(e.src)}: ${e.hops} hop${e.hops === 1 ? "" : "s"}, ${(e.lat_us / 1000).toFixed(1)} ms`);
        }
        return;
      }
      case "pos": { const nd = trace.header.nodes[e.id]; if (nd && !(drag && drag.id === e.id)) nd.pos = e.pos; staticLayer = null; return; }
      case "snr": trace.header.snr = e.snr; if (e.range_m) trace.header.range_m = e.range_m; staticLayer = null; return;
      case "task": {
        applyState(e);
        const who = e.owner != null ? name(e.owner) : "";
        const txt = { announced: `task ${e.task_id} announced by the gateway`, claimed: `${who} won task ${e.task_id}${e.cost != null ? ` (bid ${Number(e.cost).toFixed(1)} s)` : ""}`,
                      carrying: `${who} picked up task ${e.task_id} — committed, it cannot move now`, done: `task ${e.task_id} delivered`,
                      open: `task ${e.task_id} is open again`, cancelled: `task ${e.task_id} cancelled` }[e.state] || `task ${e.task_id}: ${e.state}`;
        pushFeed("task", txt); return;
      }
      case "obstacle": obstacles.push(e); pushFeed("obst", `${name(e.id)} saw an obstacle and told the fleet`); return;
      case "fleet": {
        const n = name(e.node);
        const txt = { sent_custom: `${n} sent a ${e.size} B message to ${name(e.to)}`, isolated: `${n} hears nobody: isolated`, rejoined: `${n} rejoined`,
                      hold_safe: `${n} holding safe`, lease_expired: `${n}: lease on task ${e.task_id} (owner ${name(e.owner)}) expired`,
                      owner_missing: `${n}: task ${e.task_id}'s owner is silent with the item on board`, battery_gated: `${n} did not bid on task ${e.task_id}: battery`,
                      declined: `${n} declined task ${e.task_id} (at capacity)`, no_winner: `nobody bid on task ${e.task_id} (round ${e.round})`,
                      task_yielded: `${n} yielded task ${e.task_id} to ${name(e.to)}`, stop_task: `${n} stopped task ${e.task_id} (${e.reason})`,
                      robot_silent: `${n} declares ${name(e.peer)} silent`, robot_back: `${n} hears ${name(e.peer)} again` }[e.event];
        if (txt && !["claimed", "committed"].includes(e.event)) pushFeed("fleet", txt);
        return;
      }
      case "node": if (e.action === "kill") dead.add(e.id); else dead.delete(e.id);
                   pushFeed("sys", `${name(e.id)} ${e.action === "kill" ? "went down" : "is back"}`); applyState(e); return;
      case "link": pushFeed("sys", `link ${name(e.a)}–${name(e.b)} ${e.action === "block" ? "pulled" : "restored"}`); applyState(e); return;
      default: applyState(e); return;
    }
  }

  function liveFrame(now) {
    if (!live) { window.__liveLoop = false; return; }
    const dt = (now - lastWall) * 1000 * speed; lastWall = now;
    const target = Math.max(0, liveT - LIVE_LAG_US);
    if (playing) T = Math.min(target, T + dt); else T = Math.min(T, target);
    const flashUs = FLASH_MS * 1000 * speed;
    flashes = flashes.filter((f) => T - f.t < flashUs);
    obstacles = obstacles.filter((o) => T - o.t < OBSTACLE_FADE_US);
    if (A.t.length > 4000 && T - A.t[0] > LIVE_KEEP_US) { const cut = bisect(A.t, T - LIVE_KEEP_US); for (const k of ["t", "end", "ta", "ra", "kind", "ack", "ok", "ev"]) A[k].splice(0, cut); }
    drawAll(); updateTime();
    if (now - lastPanel > PANEL_MS) { lastPanel = now; renderPanel(); }
    requestAnimationFrame(liveFrame);
  }

  async function cmd(c) {
    try { await fetch("/cmd", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(c) }); }
    catch (e) { pushFeed("error", `command failed: ${e.message}`); }
  }

  // ---------------------------------------------------------------- replay state
  function resetState() { routes = new Map(); blocked = new Set(); silent = new Map(); dead = new Set(); si = 0; if (!live) tasks = new Map(); }
  function applyState(e) {
    switch (e.k) {
      case "route": { let m = routes.get(e.node); if (!m) routes.set(e.node, (m = new Map())); if (e.nh < 0) m.delete(e.dst); else m.set(e.dst, { nh: e.nh, tq: e.tq }); break; }
      case "link": { const key = `${Math.min(e.a, e.b)}-${Math.max(e.a, e.b)}`; if (e.action === "block") blocked.add(key); else blocked.delete(key); break; }
      case "robot_silent": { let s = silent.get(e.obs); if (!s) silent.set(e.obs, (s = new Set())); s.add(e.peer); break; }
      case "robot_back": { const s = silent.get(e.obs); if (s) s.delete(e.peer); break; }
      case "node": if (e.action === "kill") dead.add(e.id); else dead.delete(e.id); break;
      case "task": tasks.set(e.task_id, { ...(tasks.get(e.task_id) || {}), ...e }); break;
      case "linkstat": if (e.failed > 0) flashes.push({ t: e.t, kind: "fail", a: e.a, b: e.b }); break;
      case "drop": flashes.push({ t: e.t, kind: "drop", a: e.src }); break;
      default: break;
    }
  }
  function advanceTo(t) {
    if (t < T) resetState();
    while (si < S.length && S[si].t <= t) applyState(S[si++]);
    // while playing, a tour stop we pass shows its caption for a few seconds
    if (playing && trace.tour) { const s = trace.tour.find((x) => x.t > T && x.t <= t); if (s) { showCaption(s); clearTimeout(captionTimer); captionTimer = setTimeout(() => showCaption(null), 5000); } }
    T = t;
    const flashUs = FLASH_MS * 1000 * speed;
    flashes = flashes.filter((f) => T - f.t < flashUs);
  }
  function seek(t) { if (live) return; advanceTo(Math.max(0, Math.min(endT, t))); drawAll(); updateTime(); }
  function play() { if (!trace) return; if (live) { cmd({ op: "pause", paused: false }); return; } playing = true; lastWall = performance.now(); syncControls(); requestAnimationFrame(frame); }
  function pause() { if (live) { cmd({ op: "pause", paused: true }); return; } playing = false; syncControls(); }
  function frame(now) {
    if (!playing || live) return;
    const dt = (now - lastWall) * 1000 * speed; lastWall = now;
    advanceTo(Math.min(endT, T + dt));
    if (T >= endT) pause();
    drawAll(); updateTime();
    if (now - lastPanel > PANEL_MS) { lastPanel = now; renderPanel(); }
    if (playing) requestAnimationFrame(frame);
  }
  function updateTime() { $("time").textContent = `${(T / 1e6).toFixed(live ? 1 : 3)} s`; }
  function syncControls() { const p = $("play"); if (p) p.textContent = playing ? "⏸" : "▶"; const sp = $("speed"); if (sp) sp.value = String(speed); }

  // ---------------------------------------------------------------- geometry
  function fitWorld() {
    const lay = trace.header.layout, pad = 34, W = floor.clientWidth, H = floor.clientHeight;
    const ox = lay.origin ? lay.origin[0] : 0, oy = lay.origin ? lay.origin[1] : 0;
    const s = Math.min((W - 2 * pad) / lay.width, (H - 2 * pad) / lay.height);
    const x0 = (W - lay.width * s) / 2, y0 = (H - lay.height * s) / 2;
    world = { s, ox, oy, toX: (x) => x0 + (x - ox) * s, toY: (y) => H - (y0 + (y - oy) * s),
              fromX: (px) => (px - x0) / s + ox, fromY: (py) => (H - py - y0) / s + oy, W, H };
  }
  function nodePos(i) { const nd = trace.header.nodes[i]; return nd ? [world.toX(nd.pos[0]), world.toY(nd.pos[1])] : [0, 0]; }
  function name(i) { return i === 0 ? "gw" : i == null ? "?" : `robot ${i}`; }

  function buildStatic() {
    const lay = trace.header.layout, h = trace.header;
    const c = document.createElement("canvas"); c.width = floor.width; c.height = floor.height;
    const g = c.getContext("2d"); g.scale(devicePixelRatio, devicePixelRatio);
    g.fillStyle = "#13161b";
    g.fillRect(world.toX(world.ox), world.toY(world.oy + lay.height), lay.width * world.s, lay.height * world.s);
    g.fillStyle = "#262b33";
    for (const r of lay.racks) g.fillRect(world.toX(r[0]), world.toY(r[3]), (r[2] - r[0]) * world.s, (r[3] - r[1]) * world.s);
    g.fillStyle = "#1e2229";
    for (const r of lay.shelves || []) g.fillRect(world.toX(r[0]), world.toY(r[3]), (r[2] - r[0]) * world.s, (r[3] - r[1]) * world.s);
    const min = h.knobs.snr_link_min_db ?? 4;
    for (let i = 0; i < h.n; i++) for (let j = i + 1; j < h.n; j++) {
      if (dead.has(i) || dead.has(j)) continue;
      const snr = Math.max(h.snr[i][j], h.snr[j][i]);
      if (snr < min) continue;
      const q = Math.min(1, (snr - min) / 25);
      const [ax, ay] = nodePos(i), [bx, by] = nodePos(j);
      g.strokeStyle = `rgba(120,160,200,${0.1 + 0.4 * q})`; g.lineWidth = 0.6 + 2.4 * q;
      g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke();
    }
    staticLayer = c;
  }

  // ---------------------------------------------------------------- drawing
  function resize() {
    const r = floor.getBoundingClientRect();
    floor.width = Math.max(1, Math.round(r.width * devicePixelRatio)); floor.height = Math.max(1, Math.round(r.height * devicePixelRatio));
    const tr = tl.getBoundingClientRect();
    tl.width = Math.max(1, Math.round(tr.width * devicePixelRatio)); tl.height = Math.max(1, Math.round(tr.height * devicePixelRatio));
    staticLayer = null;
    if (trace) drawAll();
  }

  function drawAll() {
    if (!trace) return;
    const key = `${floor.width}x${floor.height}`;
    if (!staticLayer || staticKey !== key) { fitWorld(); buildStatic(); staticKey = key; }
    ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.drawImage(staticLayer, 0, 0);
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    drawn = { packets: {}, flashes: 0, blocked: 0, routes: 0 };
    drawTasks(); drawObstacles(); drawRoutes(); drawBlocked(); drawPackets(); drawNodes(); drawFlashes(); drawDrag(); drawTimeline();
  }

  function arrow(ax, ay, bx, by, color, width) {
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    const ang = Math.atan2(by - ay, bx - ax), L = 7, mx = (ax + bx) / 2, my = (ay + by) / 2;
    ctx.beginPath(); ctx.moveTo(mx + L * Math.cos(ang), my + L * Math.sin(ang));
    ctx.lineTo(mx + L * Math.cos(ang + 2.5), my + L * Math.sin(ang + 2.5)); ctx.lineTo(mx + L * Math.cos(ang - 2.5), my + L * Math.sin(ang - 2.5)); ctx.closePath(); ctx.fill();
  }

  function drawRoutes() {
    if (routeMode === "none") return;
    const n = trace.header.n, pairs = [];
    if (routeMode === "gateway") { for (let i = 1; i < n; i++) if (!dead.has(i)) pairs.push([i, 0]); }
    else if (selected != null) { for (let d = 0; d < n; d++) if (d !== selected) pairs.push([selected, d]); }
    const seen = new Set();
    for (const [src, dst] of pairs) {
      let cur = src;
      for (let hop = 0; hop < n && cur !== dst; hop++) {
        const r = routes.get(cur)?.get(dst); if (!r) break;
        const key = `${cur}>${r.nh}`;
        if (!seen.has(key)) { seen.add(key); const [ax, ay] = nodePos(cur), [bx, by] = nodePos(r.nh); arrow(ax, ay, bx, by, `rgba(80,200,120,${0.25 + 0.5 * (r.tq / 255)})`, 1.5); drawn.routes++; }
        cur = r.nh;
      }
    }
  }

  function drawBlocked() {
    for (const key of blocked) {
      const [a, b] = key.split("-").map(Number), [ax, ay] = nodePos(a), [bx, by] = nodePos(b);
      ctx.setLineDash([6, 4]); ctx.strokeStyle = "#ff3b3b"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
      const mx = (ax + bx) / 2, my = (ay + by) / 2; ctx.beginPath(); ctx.moveTo(mx - 6, my - 6); ctx.lineTo(mx + 6, my + 6); ctx.moveTo(mx + 6, my - 6); ctx.lineTo(mx - 6, my + 6); ctx.stroke();
      drawn.blocked++;
    }
  }

  function drawTasks() {
    for (const t of tasks.values()) {
      if (t.state === "done" || t.state === "cancelled") continue;
      const [px, py] = [world.toX(t.pickup[0]), world.toY(t.pickup[1])], [dx, dy] = [world.toX(t.dropoff[0]), world.toY(t.dropoff[1])];
      ctx.setLineDash([3, 5]); ctx.strokeStyle = "rgba(255,209,102,0.45)"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(dx, dy); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = t.state === "carrying" ? "#06d6a0" : "#ffd166"; ctx.fillRect(px - 6, py - 6, 12, 12);
      ctx.strokeStyle = "#ffd166"; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(dx, dy, 7, 0, Math.PI * 2); ctx.stroke();
      ctx.fillStyle = "#ffd166"; ctx.font = "10px monospace"; ctx.textAlign = "center";
      ctx.fillText(`task ${t.task_id}`, px, py - 10); ctx.fillText(`→ ${t.task_id}`, dx, dy - 10);
      if (t.owner != null && t.owner !== 0) { const [ox, oy] = nodePos(t.owner); ctx.strokeStyle = "rgba(6,214,160,0.5)"; ctx.setLineDash([2, 4]); ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(t.state === "carrying" ? dx : px, t.state === "carrying" ? dy : py); ctx.stroke(); ctx.setLineDash([]); }
    }
  }

  function drawObstacles() {
    for (const o of obstacles) {
      const a = 1 - (T - o.t) / OBSTACLE_FADE_US; if (a <= 0) continue;
      const [x, y] = [world.toX(o.pos[0]) + 14, world.toY(o.pos[1]) - 14];
      ctx.globalAlpha = Math.max(0.15, a); ctx.fillStyle = "#ff5d8f"; ctx.beginPath();
      for (let k = 0; k < 6; k++) { const ang = Math.PI / 3 * k; ctx.lineTo(x + 7 * Math.cos(ang), y + 7 * Math.sin(ang)); }
      ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
    }
  }

  function drawPackets() {
    if (!A.t.length) return;
    const minVis = MIN_VIS_MS * 1000 * speed, flashUs = FLASH_MS * 1000 * speed;
    const lo = bisect(A.t, T - Math.max(MAX_VIS_US, minVis + flashUs)), hi = bisect(A.t, T + 1);
    ctx.font = "bold 9px system-ui, sans-serif"; ctx.textAlign = "center";
    for (let i = lo; i < hi; i++) {
      const kind = KINDS[A.kind[i]]; if (hidden.has(kind)) continue;
      const vis = Math.max(A.end[i] - A.t[i], minVis), p = (T - A.t[i]) / vis;
      const [color, label] = KIND_DEF[kind], [ax, ay] = nodePos(A.ta[i]);
      if (p <= 1) {
        if (A.ra[i] < 0) {
          ctx.strokeStyle = color; ctx.globalAlpha = 0.5 * (1 - p); ctx.lineWidth = 1.2;
          ctx.beginPath(); ctx.arc(ax, ay, 4 + p * BCAST_RANGE_M * world.s, 0, Math.PI * 2); ctx.stroke(); ctx.globalAlpha = 1;
        } else {
          const [bx, by] = nodePos(A.ra[i]), x = ax + (bx - ax) * p, y = ay + (by - ay) * p;
          const p0 = Math.max(0, p - 0.18);
          ctx.strokeStyle = color; ctx.globalAlpha = 0.35; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(ax + (bx - ax) * p0, ay + (by - ay) * p0); ctx.lineTo(x, y); ctx.stroke(); ctx.globalAlpha = 1;
          ctx.shadowColor = color; ctx.shadowBlur = 8; ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, label ? 4.5 : 3.5, 0, Math.PI * 2); ctx.fill(); ctx.shadowBlur = 0;
          if (label && speed <= 2) { ctx.fillStyle = "#fff"; ctx.fillText(label, x, y - 8); }
          if (A.ok[i] === 1 && p > 0.85) { ctx.strokeStyle = "#8f8"; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.stroke(); }
        }
        drawn.packets[kind] = (drawn.packets[kind] || 0) + 1;
      } else if (A.ack[i] === 0 && T - A.end[i] < flashUs + minVis && A.ra[i] >= 0) {
        const [bx, by] = nodePos(A.ra[i]), x = ax + (bx - ax) * 0.8, y = ay + (by - ay) * 0.8;
        ctx.strokeStyle = "#ff4d4d"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x - 5, y - 5); ctx.lineTo(x + 5, y + 5); ctx.moveTo(x + 5, y - 5); ctx.lineTo(x - 5, y + 5); ctx.stroke();
        drawn.flashes++;
      }
    }
  }

  function drawFlashes() {
    for (const f of flashes) {
      if (f.kind === "fail") { const [ax, ay] = nodePos(f.a), [bx, by] = nodePos(f.b); ctx.strokeStyle = "rgba(255,120,60,0.8)"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); }
      else { const [ax, ay] = nodePos(f.a); ctx.strokeStyle = "#ff4d4d"; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(ax, ay, 12, 0, Math.PI * 2); ctx.stroke(); }
      drawn.flashes++;
    }
  }

  function drawNodes() {
    const h = trace.header;
    ctx.font = "11px monospace"; ctx.textAlign = "center";
    for (let i = 0; i < h.n; i++) {
      const [x, y] = nodePos(i), gw = h.nodes[i].gateway, isDead = dead.has(i), nSilent = silent.get(i)?.size || 0;
      const held = (owned[i] || []).length;
      if (!isDead && !gw) { ctx.shadowColor = "#4cc9f0"; ctx.shadowBlur = 10; }
      ctx.fillStyle = isDead ? "#444" : gw ? "#ffd166" : "#4cc9f0";
      ctx.strokeStyle = selected === i ? "#fff" : "#0e1013"; ctx.lineWidth = selected === i ? 2.5 : 1.5;
      ctx.beginPath();
      if (gw) { ctx.moveTo(x, y - 10); ctx.lineTo(x + 9, y + 7); ctx.lineTo(x - 9, y + 7); ctx.closePath(); } else ctx.arc(x, y, 8, 0, Math.PI * 2);
      ctx.fill(); ctx.stroke(); ctx.shadowBlur = 0;
      if (gw) { ctx.strokeStyle = "rgba(255,209,102,0.7)"; ctx.lineWidth = 1.2; for (const r of [14, 19]) { ctx.beginPath(); ctx.arc(x, y - 2, r, -2.2, -0.95); ctx.stroke(); } }
      if (isDead) { ctx.strokeStyle = "#f55"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x - 6, y - 6); ctx.lineTo(x + 6, y + 6); ctx.moveTo(x + 6, y - 6); ctx.lineTo(x - 6, y + 6); ctx.stroke(); }
      ctx.fillStyle = "#cde"; ctx.fillText(gw ? "gateway / AP" : `robot ${i}`, x, y - 14);
      let sub = [];
      if (nSilent) sub.push(`${nSilent} silent`);
      if (held) sub.push(`${held} task${held > 1 ? "s" : ""}`);
      if (sub.length) { ctx.fillStyle = nSilent ? "#f77" : "#8ac926"; ctx.fillText(sub.join(" · "), x, y + 21); }
    }
  }

  function drawDrag() {
    if (!drag || !live) return;
    const h = trace.header, rng = h.range_m ? h.range_m.robot : 0;
    const [x, y] = nodePos(drag.id);
    if (rng) { ctx.strokeStyle = "rgba(76,201,240,0.35)"; ctx.setLineDash([4, 6]); ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(x, y, rng * world.s, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = "rgba(76,201,240,0.6)"; ctx.font = "10px monospace"; ctx.fillText(`open-air range ≈ ${rng} m`, x, y + rng * world.s + 12); }
  }

  function drawTimeline() {
    const W = tl.clientWidth, H = tl.clientHeight;
    tctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    tctx.fillStyle = "#0e1013"; tctx.fillRect(0, 0, W, H);
    tctx.fillStyle = "#1c1f24"; tctx.fillRect(0, H / 2 - 2, W, 4);
    const t0 = live ? Math.max(0, T - LIVE_KEEP_US * 0.66) : 0, t1 = live ? Math.max(t0 + 1, T + LIVE_KEEP_US * 0.05) : endT;
    const x = (t) => ((t - t0) / (t1 - t0)) * W;
    const marks = live ? feed.filter((f) => f.t != null) : S;
    for (const e of marks) {
      if (e.t < t0 || e.t > t1) continue;
      if (e.k === "link" || e.kind === "sys") { tctx.fillStyle = e.action === "unblock" ? "#8f8" : "#ff3b3b"; tctx.fillRect(x(e.t) - 1, 6, 2, H - 12); }
      else if (e.k === "robot_silent" || e.kind === "fleet") { tctx.fillStyle = "#fb7"; tctx.fillRect(x(e.t) - 0.5, H / 2 - 10, 1, 10); }
      else if (e.k === "robot_back") { tctx.fillStyle = "#7bf"; tctx.fillRect(x(e.t) - 0.5, H / 2, 1, 10); }
      else if (e.k === "mark" || e.kind === "task" || e.kind === "obst") { tctx.fillStyle = e.kind === "task" ? "#ffd166" : "#ff5d8f"; tctx.fillRect(x(e.t) - 0.5, 2, 1, H - 4); }
    }
    if (trace.tour) for (const s of trace.tour) { tctx.fillStyle = "#9d4edd"; tctx.fillRect(x(s.t) - 1, 0, 2, 6); }
    tctx.fillStyle = "#fff"; tctx.fillRect(x(T) - 1, 0, 2, H);
    tctx.fillStyle = "#9aa"; tctx.font = "10px monospace"; tctx.textAlign = "left"; tctx.fillText(`${(t0 / 1e6).toFixed(0)} s`, 3, H - 3);
    tctx.textAlign = "right"; tctx.fillText(`${(t1 / 1e6).toFixed(0)} s`, W - 3, H - 3);
  }

  // ---------------------------------------------------------------- side panel
  function fmt(x, d = 1) { return x == null || !Number.isFinite(x) ? "—" : x.toFixed(d); }
  function esc(s) { return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;"); }
  function pushFeed(kind, text) { feed.push({ t: live ? liveT : T, kind, text, wall: new Date() }); if (feed.length > 60) feed.shift(); }
  function lastHeard(obs, peer) { const r = A.rx.get(`${obs},${peer}`); if (!r) return null; const i = bisect(r.t, T + 1) - 1; return i >= 0 ? { t: r.t[i], seq: r.seq[i] } : null; }

  function renderPanel() {
    if (!trace) return;
    const h = trace.header, body = $("side-body"), feedEl = $("feed");
    const tmo = h.knobs.status_timeout_s * 1e6;
    if (feedEl) {
      feedEl.innerHTML = feed.slice().reverse().map((f) => `<div class="f-${f.kind}"><span class="ft">${f.t != null ? (f.t / 1e6).toFixed(1) + " s" : ""}</span>${esc(f.text)}</div>`).join("");
    }
    if (selected == null) {
      let alive = 0; for (let i = 1; i < h.n; i++) if (!silent.get(0)?.has(i) && !dead.has(i)) alive++;
      const c = h.counts || {};
      const openTasks = [...tasks.values()].filter((t) => !["done", "cancelled"].includes(t.state));
      body.innerHTML =
        `<table><tr><th>robots</th><td>${h.n - 1} + gateway</td></tr>` +
        `<tr><th>alive (gateway's view)</th><td>${alive} / ${h.n - 1}</td></tr>` +
        `<tr><th>blocked links</th><td>${blocked.size}</td></tr>` +
        (live ? `<tr><th>open tasks</th><td>${openTasks.map((t) => `${t.task_id}${t.owner != null ? " → " + name(t.owner) : ""}`).join(", ") || "—"}</td></tr>`
              : `<tr><th>status delivery</th><td>${c.delivery == null ? "—" : (100 * c.delivery).toFixed(1) + " %"}</td></tr><tr><th>lost hops</th><td>${c.unacked ?? "—"}</td></tr>`) +
        `</table>` + (h.limits ? `<h2>What this is</h2><div class="limits">${h.limits.map((s) => "• " + esc(s)).join("\n")}</div>` : "") +
        `<h2>Hint</h2><div class="limits">${live ? "drag a robot to move it; click one for its view of the fleet" : "click a node for its view of the fleet"}</div>`;
      return;
    }
    const nd = h.nodes[selected], rows = [];
    for (let p = 0; p < h.n; p++) {
      if (p === selected) continue;
      const lh = lastHeard(selected, p), isSilent = silent.get(selected)?.has(p), age = lh ? (T - lh.t) / 1e6 : null;
      const r = A.rx.get(`${selected},${p}`), tx = A.tx.get(p);
      let got = 0, sent = 0;
      if (r) got = bisect(r.t, T + 1) - bisect(r.t, T - 10e6);
      if (tx) sent = bisect(tx, T + 1) - bisect(tx, T - 10e6);
      rows.push(`<tr class="${isSilent ? "silent" : ""}"><td>${name(p)}</td><td>${age == null ? "never" : fmt(age) + " s"}</td>` +
                `<td>${live ? got : lh ? lh.seq : "—"}</td><td>${sent ? ((100 * got) / sent).toFixed(0) + " %" : "—"}</td>` +
                `<td>${isSilent ? "SILENT" : age != null && age * 1e6 > tmo ? "stale" : "ok"}</td></tr>`);
    }
    const rt = [...(routes.get(selected) || [])].map(([d, r]) => `${name(d)} via ${name(r.nh)} (${r.tq})`).join(", ");
    body.innerHTML =
      `<table><tr><th>node</th><td>${nd.gateway ? "gateway" : "robot " + selected}${nd.ns ? " · " + esc(nd.ns) : ""}</td></tr>` +
      `<tr><th>pos</th><td>${fmt(nd.pos[0])}, ${fmt(nd.pos[1])} m</td></tr>` +
      (live ? `<tr><th>owns</th><td>${(owned[selected] || []).map((t) => "task " + t).join(", ") || "—"}</td></tr>` : "") +
      `<tr><th>next hops</th><td>${rt || "—"}</td></tr></table>` +
      `<h2>Its view of the fleet</h2><table><tr><th>peer</th><th>heard</th><th>${live ? "rx/10 s" : "seq"}</th><th>10 s</th><th></th></tr>${rows.join("")}</table>` +
      `<h2>Hint</h2><div class="limits">click the floor to deselect</div>`;
  }

  // ---------------------------------------------------------------- controls
  function buildFilters() {
    const f = $("filters"); if (!f) return; f.innerHTML = "";
    const present = live ? new Set(KINDS) : new Set(Object.keys(trace.header.counts.air_by_kind || {}));
    for (const k of KINDS) {
      if (!present.has(k)) continue;
      const l = document.createElement("label");
      l.innerHTML = `<input type="checkbox" ${hidden.has(k) ? "" : "checked"}> <span style="color:${KIND_DEF[k][0]}">${k}</span>`;
      l.querySelector("input").addEventListener("change", (ev) => { if (ev.target.checked) hidden.delete(k); else hidden.add(k); drawAll(); });
      f.appendChild(l);
    }
  }
  function buildLegend() {
    const l = $("legend"); if (!l) return;
    const present = live ? ["status", "announce", "bid", "claim", "lease", "obstacle", "custom"] : KINDS.filter((k) => trace.header.counts.air_by_kind?.[k]);
    l.innerHTML = present.map((k) => `<span style="--c:${KIND_DEF[k][0]}">${k}</span>`).join("") + `<span style="--c:#ff4d4d">lost / blocked</span><span style="--c:#50c878">route</span>`;
  }
  function buildTour() {
    const el = $("tour"); if (!el) return;
    const tour = trace.tour || [];
    el.innerHTML = tour.length ? tour.map((s, i) => `<button data-i="${i}" title="${esc(s.text)}">${(s.t / 1e6).toFixed(1)} s · ${esc(s.title)}</button>`).join("") : "";
    el.querySelectorAll("button").forEach((b) => b.addEventListener("click", () => { const s = tour[Number(b.dataset.i)]; seek(s.t); showCaption(s); }));
  }
  function showCaption(s) { const c = $("caption"); if (!c) return; c.textContent = s ? `${(s.t / 1e6).toFixed(1)} s — ${s.text}` : ""; c.style.display = s ? "" : "none"; }
  function stateEvents() { return S.filter((e) => e.k === "link" || e.k === "robot_silent" || e.k === "robot_back" || e.k === "mark"); }
  function nextEvent() { const e = stateEvents().find((x) => x.t > T + 1); if (e) seek(e.t); }
  function prevEvent() { const es_ = stateEvents().filter((x) => x.t < T - 1); if (es_.length) seek(es_[es_.length - 1].t); }

  const on = (id, ev, fn) => { const el = $(id); if (el) el.addEventListener(ev, fn); };
  on("play", "click", () => (playing ? pause() : play()));
  on("next", "click", nextEvent); on("prev", "click", prevEvent);
  on("speed", "change", (e) => { speed = Number(e.target.value); if (live) cmd({ op: "speed", speed }); });
  on("routes", "change", (e) => { routeMode = e.target.value; drawAll(); });
  on("file", "change", (e) => { if (e.target.files[0]) loadFromFile(e.target.files[0]).catch(fail); });
  document.addEventListener("keydown", (e) => {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    if (e.code === "Space") { e.preventDefault(); playing ? pause() : play(); }
    else if (!live && e.code === "ArrowRight") seek(T + 1e6 * (e.shiftKey ? 10 : 1));
    else if (!live && e.code === "ArrowLeft") seek(T - 1e6 * (e.shiftKey ? 10 : 1));
    else if (e.code === "Escape") { pick = null; setPickHint(""); }
  });
  document.addEventListener("dragover", (e) => e.preventDefault());
  document.addEventListener("drop", (e) => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) loadFromFile(f).catch(fail); });

  // demo buttons (present on demo.html only)
  function setPickHint(t) { const el = $("pick-hint"); if (el) el.textContent = t; }
  function startPick(op, need, hint) { pick = { op, need, got: [], hint }; setPickHint(hint); }
  on("d-task", "click", () => cmd({ op: "task" }));
  on("d-obstacle", "click", () => startPick("obstacle", 1, "click the robot that sees the obstacle"));
  on("d-link", "click", () => startPick("link", 2, "click the two ends of the link to pull (again to restore)"));
  on("d-kill", "click", () => startPick("node", 1, "click a robot to switch it off (again to bring it back)"));
  on("d-send", "click", () => startPick("send", 2, "click the sender, then the receiver"));
  on("d-reset", "click", () => { const n = Number(($("d-n") || {}).value || 4); cmd({ op: "reset", n: n + 1 }); });
  on("d-n", "change", (e) => cmd({ op: "reset", n: Number(e.target.value) + 1 }));
  let rangeTimer = null;
  on("d-range", "input", (e) => {
    const v = Number(e.target.value); const lbl = $("d-range-v"); if (lbl) lbl.textContent = `${v} m`;
    clearTimeout(rangeTimer); rangeTimer = setTimeout(() => cmd({ op: "range", metres: v }), 120);
  });

  function nodeAt(x, y) { let best = null, bd = 16; for (let i = 0; i < trace.header.n; i++) { const [nx, ny] = nodePos(i); const d = Math.hypot(nx - x, ny - y); if (d < bd) { bd = d; best = i; } } return best; }
  function finishPick(i) {
    pick.got.push(i);
    if (pick.got.length < pick.need) { setPickHint(`${pick.hint} — ${name(i)} selected`); return; }
    const [a, b] = pick.got;
    if (pick.op === "obstacle") cmd({ op: "obstacle", id: a });
    else if (pick.op === "node") cmd({ op: "node", id: a, action: dead.has(a) ? "revive" : "kill" });
    else if (pick.op === "link") cmd({ op: "link", a, b, action: blocked.has(`${Math.min(a, b)}-${Math.max(a, b)}`) ? "unblock" : "block" });
    else if (pick.op === "send") cmd({ op: "send", src: a, dst: b, size: Number(($("d-size") || {}).value || 200) });
    pick = null; setPickHint("");
  }

  floor.addEventListener("mousedown", (e) => {
    if (!trace) return;
    const r = floor.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top, i = nodeAt(x, y);
    if (live && i != null && i !== 0 && !pick) { drag = { id: i, moved: false, lastPost: 0 }; e.preventDefault(); }
  });
  window.addEventListener("mousemove", (e) => {
    if (!drag || !trace) return;
    const r = floor.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
    const nd = trace.header.nodes[drag.id];
    nd.pos = [world.fromX(x), world.fromY(y)]; drag.moved = true; staticLayer = null;
    const now = performance.now();
    if (now - drag.lastPost > 70) { drag.lastPost = now; cmd({ op: "move", id: drag.id, x: nd.pos[0], y: nd.pos[1] }); }
  });
  window.addEventListener("mouseup", (e) => {
    if (drag) { const nd = trace.header.nodes[drag.id]; if (drag.moved) cmd({ op: "move", id: drag.id, x: nd.pos[0], y: nd.pos[1] }); const d = drag; drag = null; if (d.moved) return; }
    if (scrubbing) { scrubbing = false; return; }
    if (!trace || e.target !== floor) return;
    const r = floor.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top, i = nodeAt(x, y);
    if (pick && i != null) { finishPick(i); return; }
    selected = i; renderPanel(); drawAll();
  });
  let scrubbing = false;
  const scrub = (e) => { if (live) return; const r = tl.getBoundingClientRect(); seek(((e.clientX - r.left) / r.width) * endT); showCaption(null); };
  tl.addEventListener("mousedown", (e) => { scrubbing = true; scrub(e); });
  window.addEventListener("mousemove", (e) => { if (scrubbing) scrub(e); });
  window.addEventListener("resize", resize);

  function fail(err) { const h = $("hint"); if (h) { h.style.display = ""; h.textContent = String(err.message || err); } console.error(err); }

  // ---------------------------------------------------------------- test hook
  window.__viz = {
    load, live: liveStart, cmd, seek: (s) => seek(s * 1e6), play, pause,
    speed: (x) => { speed = x; syncControls(); if (live) cmd({ op: "speed", speed: x }); },
    select: (i) => { selected = i; renderPanel(); drawAll(); },
    setFilter: (kind, on_) => { if (on_) hidden.delete(kind); else hidden.add(kind); drawAll(); },
    state: () => ({
      loaded: !!trace, live, t: T / 1e6, n: trace ? trace.header.n : 0, playing, speed,
      active: drawn.packets, flashes: drawn.flashes, routesDrawn: drawn.routes,
      blocked: [...blocked], routes: Object.fromEntries([...routes].map(([k, m]) => [k, Object.fromEntries(m)])),
      silent: Object.fromEntries([...silent].map(([k, s]) => [k, [...s]])), dead: [...dead], selected,
      nodes: trace ? trace.header.nodes.map((n) => n.pos) : [], tasks: [...tasks.values()], owned,
      feed: feed.slice(-12).map((f) => f.text), airCount: A ? A.t.length : 0,
      panel: $("side-body").textContent,
    }),
  };

  resize();
  const params = new URLSearchParams(location.search);
  if (params.get("trace")) loadFromUrl(params.get("trace")).catch(fail);
  else if (document.body.dataset.live) liveStart(document.body.dataset.live);
})();
