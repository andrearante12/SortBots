// Comms overlay for the operator view (webui/operator.html): the commsim model
// (commsim/viz/demo.py, the real DES with core/protocol.py in every node) drawn
// over the live map. Derived from commsim/viz/viz.js (live path only); the
// replay and the standalone pages it served are gone (CLAUDE.md, "One GUI").
//
// What feeds the model is real: the robots stand where their poses say (operator.js
// posts a move per robot as it drives), the racks are the explored map's occupied
// cells (`layout`, commsim/des/layout.py from_occupancy), the gateway is the
// operator's gateway pin. What it shows is PREDICTED: which links would hold, how
// the mesh would route, every message hop, who would go silent. So everything here
// is drawn dashed/haloed on the map and never touches the robot cards or the alarm
// chip — those come from the real fleet (/fleet/task_board).
//
// The model's own task auction is never used: the real one is on the task board.
// What-ifs (pull a link, kill a robot, send A -> B, the range slider) act on the
// model only.
//
// Model coordinates are metres from the map grid's corner (the DES floor is
// [0, w] x [0, h]); world = model + origin, where origin is the map origin the last
// layout was posted with. Model node 0 is the gateway, robot_N is node N+1 — the
// same wire ids nodes/fleet_tasks.py uses.
//
// serve.py: GET /api/commsim/events (Server-Sent Events), POST /api/commsim/cmd.
// The model only steps while a page is subscribed, so turning the overlay off
// (closing the EventSource) parks it.

(() => {
  "use strict";

  // kind -> [colour, short label on the dot, group]. Message colours are the
  // legend, so they stay fixed across themes; everything else reads CSS tokens.
  const KIND_DEF = {
    status: ["#4cc9f0", "", "fleet"], announce: ["#ffd166", "TASK", "task"], bid: ["#f4a261", "BID", "task"],
    claim: ["#06d6a0", "CLAIM", "task"], lease: ["#8ac926", "lease", "task"], event: ["#ff9f1c", "done", "task"],
    obstacle: ["#ff5d8f", "OBST", "fleet"], custom: ["#c77dff", "MSG", "fleet"], map: ["#9d4edd", "map", "fleet"],
    linkhealth: ["#577590", "", "fleet"], ka: ["#3a506b", "", "transport"], zenoh: ["#c77dff", "", "transport"],
    tcp_ack: ["#7a5cff", "", "transport"], ogm: ["#8d99ae", "", "mesh"], other: ["#adb5bd", "", "mesh"],
  };
  // Routing chatter is most of the airtime and none of the story; one toggle shows it.
  const CHATTER = new Set(["ogm", "tcp_ack", "other", "ka", "linkhealth", "map", "zenoh"]);
  const MIN_VIS_MS = 140;      // a 400 µs hop is invisible otherwise, and seeing it is the point
  const MAX_VIS_US = 1_500_000;
  const FLASH_MS = 400;
  const LAG_US = 150_000;      // draw this far behind the model so hops arrive before they animate
  const KEEP_US = 30_000_000;
  const POSE_EPS_M = 0.1;      // don't re-run the model's topology for centimetres
  const LAYOUT_EVERY_MS = 10000;
  const CELL_M = 0.5;          // model rack resolution: coarser than the map, fine for attenuation

  let opts = null;             // from operator.js: see mount()
  let canvas = null, ctx = null;
  let on = false, es = null, connected = false;
  let header = null, T = 0, liveT = 0, lastWall = 0, playing = true, speed = 1;
  let air = [];                // [{t, end, ta, ra, kind, ack}] in time order
  let routes = new Map(), blocked = new Set(), silent = new Map(), dead = new Set();
  let feed = [], showChatter = false, pick = null;
  let origin = null;           // [x, y] world metres of the model's (0, 0); null until a map is posted
  let layoutAt = -Infinity, layoutMapStamp = null, racks = 0;
  const sentPos = new Map();   // node id -> [x, y] model, last posted
  let rosterKey = "";

  const nodeOf = (robotId) => { const m = /^robot_(\d+)$/.exec(robotId); return m ? Number(m[1]) + 1 : null; };
  const nameOf = (i) => (i === 0 ? "Gateway" : i == null ? "?" : `Robot ${i - 1}`);
  const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

  // One at a time, in order: a layout must land before the reset that places robots
  // on it, and concurrent fetches don't promise that.
  let chain = Promise.resolve();
  const cmd = (c) => (chain = chain.then(() => send(c)));
  async function send(c) {
    try {
      const r = await fetch("/api/commsim/cmd", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(c) });
      if (!r.ok) pushFeed("error", `model: ${(await r.json().catch(() => ({}))).error || r.status}`);
    } catch (e) { pushFeed("error", `model unreachable: ${e.message}`); }
  }

  // -- event stream ---------------------------------------------------------------

  function connect() {
    if (es) return;
    es = new EventSource("/api/commsim/events");
    es.onopen = () => { connected = true; renderBar(); };
    es.onmessage = (m) => { let e; try { e = JSON.parse(m.data); } catch { return; } onEvent(e); };
    es.onerror = () => { connected = false; renderBar(); };
  }
  function disconnect() {
    if (es) { es.close(); es = null; }
    connected = false; header = null;
  }

  function onEvent(e) {
    switch (e.k) {
      case "init":
        header = e.header; speed = e.speed || 1; playing = !e.paused;
        T = liveT = 0; air = []; routes = new Map(); blocked = new Set(); silent = new Map();
        for (const [node, dst, nh, tq] of header.routes || []) {
          if (!routes.has(node)) routes.set(node, new Map());
          routes.get(node).set(dst, { nh, tq });
        }
        dead = new Set(header.nodes.filter((n) => n.alive === false).map((n) => n.id));
        // A fresh model (ours or another page's reset): re-send the layout and every
        // position. rosterKey stays: a reset of the right size must not trigger
        // another reset, or the two would chase each other forever.
        sentPos.clear(); layoutAt = -Infinity; layoutMapStamp = null;
        sync(true); renderBar(); return;
      case "ctl": speed = e.speed; playing = !e.paused; renderBar(); return;
      case "error": pushFeed("error", e.text); return;
      case "air": air.push({ t: e.t, end: e.t + (e.dur || 0), ta: e.ta, ra: e.ra, kind: KIND_DEF[e.kind] ? e.kind : "other", ack: e.ack }); return;
      case "now": liveT = e.t; return;
      case "pos": { const nd = header && header.nodes[e.id]; if (nd) nd.pos = e.pos; return; }
      case "snr": if (header) { header.snr = e.snr; if (e.range_m) header.range_m = e.range_m; } return;
      case "layout": racks = e.racks; if (header) { header.layout.width = e.width; header.layout.height = e.height; } renderBar(); return;
      case "route": { let m = routes.get(e.node); if (!m) routes.set(e.node, (m = new Map())); if (e.nh < 0) m.delete(e.dst); else m.set(e.dst, { nh: e.nh, tq: e.tq }); return; }
      case "link": { const key = `${Math.min(e.a, e.b)}-${Math.max(e.a, e.b)}`; if (e.action === "block") blocked.add(key); else blocked.delete(key);
        pushFeed("sys", `Link ${nameOf(e.a)}–${nameOf(e.b)} ${e.action === "block" ? "pulled" : "restored"}`); return; }
      case "node": if (e.action === "kill") dead.add(e.id); else dead.delete(e.id);
        pushFeed("sys", `${nameOf(e.id)} ${e.action === "kill" ? "switched off" : "back on"}`); return;
      case "robot_silent": { let s = silent.get(e.obs); if (!s) silent.set(e.obs, (s = new Set())); s.add(e.peer); return; }
      case "robot_back": { const s = silent.get(e.obs); if (s) s.delete(e.peer); return; }
      case "app_rx":
        if (e.kind === "custom") pushFeed("rx", `${nameOf(e.dst)} got the message from ${nameOf(e.src)}: ${e.hops} hop${e.hops === 1 ? "" : "s"}, ${(e.lat_us / 1000).toFixed(1)} ms`);
        return;
      case "fleet": {
        const txt = { sent_custom: `${nameOf(e.node)} sent ${e.size} B to ${nameOf(e.to)}`, isolated: `${nameOf(e.node)} would hear nobody`,
          rejoined: `${nameOf(e.node)} rejoined`, robot_silent: `${nameOf(e.node)} would call ${nameOf(e.peer)} silent`,
          robot_back: `${nameOf(e.node)} hears ${nameOf(e.peer)} again` }[e.event];
        if (txt) pushFeed("fleet", txt);
        return;
      }
      default: return;
    }
  }

  function pushFeed(kind, text) {
    feed.push({ kind, text, t: liveT });
    if (feed.length > 40) feed.shift();
    if (opts) renderFeed();
  }

  // -- feeding the model from the live operator state --------------------------------

  // Called by operator.js on its 250 ms tick (and by init above). Posts only what changed.
  function sync(force = false) {
    if (!on || !header || !opts) return;
    const map = opts.getMap();
    if (!map || !map.info || !map.info.width) return; // no floor yet: nothing to place robots on
    const now = performance.now();
    const stamp = `${map.info.width}x${map.info.height}@${map.info.origin.position.x},${map.info.origin.position.y}`;
    const grew = stamp !== layoutMapStamp;
    if (force || grew || now - layoutAt > LAYOUT_EVERY_MS) {
      if (force || grew || map !== sync.lastMsg) postLayout(map);
      layoutAt = now; layoutMapStamp = stamp; sync.lastMsg = map;
      if (grew) sentPos.clear(); // the origin may have moved: every model position is stale
    }
    if (!origin) return;
    const robots = opts.getRobots().filter((r) => r.pose && nodeOf(r.id) != null);
    const n = robots.length ? Math.max(...robots.map((r) => nodeOf(r.id))) : 0;
    const key = robots.map((r) => r.id).sort().join(",");
    const gw = opts.getGateway() || defaultGateway(map);
    const toModel = (x, y) => [x - origin[0], y - origin[1]];
    if (n && (key !== rosterKey || header.n !== n + 1)) {
      // A robot joined or left: the model needs one node per robot, the gateway first.
      const pos = [toModel(gw.x, gw.y)];
      for (let i = 1; i <= n; i++) {
        const r = robots.find((x) => nodeOf(x.id) === i);
        pos.push(r ? toModel(r.pose.x, r.pose.y) : pos[0]);
      }
      rosterKey = key; sentPos.clear();
      pos.forEach((p, i) => sentPos.set(i, p));
      cmd({ op: "reset", n, positions: pos });
      return;
    }
    const want = [[0, toModel(gw.x, gw.y)], ...robots.map((r) => [nodeOf(r.id), toModel(r.pose.x, r.pose.y)])];
    for (const [id, p] of want) {
      const prev = sentPos.get(id);
      if (prev && Math.hypot(p[0] - prev[0], p[1] - prev[1]) < POSE_EPS_M) continue;
      sentPos.set(id, p);
      cmd(id === 0 ? { op: "gateway", x: p[0], y: p[1] } : { op: "move", id, x: p[0], y: p[1] });
    }
  }
  const defaultGateway = (map) => {
    const { width: w, height: h, resolution: res, origin: o } = map.info;
    return { x: o.position.x + (w * res) / 2, y: o.position.y + (h * res) / 2, guessed: true };
  };

  // Downsample the occupancy grid to CELL_M squares, one character each: a square is
  // occupied if any map cell in it is (>= 65, the usual occupied threshold).
  function postLayout(map) {
    const { width: w, height: h, resolution: res, origin: o } = map.info;
    const k = Math.max(1, Math.round(CELL_M / res)), cell = k * res;
    const w2 = Math.ceil(w / k), h2 = Math.ceil(h / k), out = new Uint8Array(w2 * h2);
    for (let row = 0; row < h; row++) {
      const r2 = ((row / k) | 0) * w2;
      for (let col = 0; col < w; col++) if (map.data[row * w + col] >= 65) out[r2 + ((col / k) | 0)] = 1;
    }
    let occ = "";
    for (let i = 0; i < out.length; i++) occ += out[i] ? "1" : "0";
    origin = [o.position.x, o.position.y];
    cmd({ op: "layout", occ, w: w2, h: h2, cell });
  }

  // -- drawing ----------------------------------------------------------------------

  function resize() {
    if (!canvas) return;
    const r = canvas.parentElement.getBoundingClientRect(), d = window.devicePixelRatio || 1;
    canvas.width = Math.round(r.width * d); canvas.height = Math.round(r.height * d);
    ctx.setTransform(d, 0, 0, d, 0, 0);
  }

  function px(i) {
    const nd = header.nodes[i];
    return nd && origin ? opts.toPx(nd.pos[0] + origin[0], nd.pos[1] + origin[1]) : null;
  }

  function frame(now) {
    if (!on) return;
    requestAnimationFrame(frame);
    const d = window.devicePixelRatio || 1, W = canvas.width / d, H = canvas.height / d;
    ctx.clearRect(0, 0, W, H);
    if (!header || !origin) { lastWall = now; return; }
    const dt = (now - lastWall) * 1000 * speed; lastWall = now;
    const target = Math.max(0, liveT - LAG_US);
    T = playing ? Math.min(target, T + dt) : Math.min(T, target);
    if (air.length > 3000 && T - air[0].t > KEEP_US) air.splice(0, air.findIndex((a) => T - a.t <= KEEP_US));
    drawLinks(); drawRoutes(); drawBlocked(); drawPackets(); drawNodes();
  }

  function drawLinks() {
    const n = header.n, min = header.knobs.snr_link_min_db ?? 4, ink = css("--muted");
    ctx.setLineDash([2, 4]);
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) {
      if (dead.has(i) || dead.has(j)) continue;
      const snr = Math.max(header.snr[i][j], header.snr[j][i]);
      if (snr < min) continue;
      const a = px(i), b = px(j); if (!a || !b) continue;
      const q = Math.min(1, (snr - min) / 25);
      ctx.strokeStyle = ink; ctx.globalAlpha = 0.25 + 0.5 * q; ctx.lineWidth = 0.8 + 2.2 * q;
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }
    ctx.setLineDash([]); ctx.globalAlpha = 1;
  }

  function arrow(a, b, color, alpha) {
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.globalAlpha = alpha; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    const ang = Math.atan2(b[1] - a[1], b[0] - a[0]), L = 7, mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
    ctx.beginPath(); ctx.moveTo(mx + L * Math.cos(ang), my + L * Math.sin(ang));
    ctx.lineTo(mx + L * Math.cos(ang + 2.5), my + L * Math.sin(ang + 2.5)); ctx.lineTo(mx + L * Math.cos(ang - 2.5), my + L * Math.sin(ang - 2.5));
    ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
  }

  // Every robot's route to the gateway, hop by hop: the path its status would take.
  function drawRoutes() {
    const go = css("--go"), seen = new Set();
    for (let src = 1; src < header.n; src++) {
      if (dead.has(src)) continue;
      let cur = src;
      for (let hop = 0; hop < header.n && cur !== 0; hop++) {
        const r = routes.get(cur)?.get(0); if (!r) break;
        const key = `${cur}>${r.nh}`, a = px(cur), b = px(r.nh);
        if (!seen.has(key) && a && b) { seen.add(key); arrow(a, b, go, 0.35 + 0.5 * (r.tq / 255)); }
        cur = r.nh;
      }
    }
  }

  function drawBlocked() {
    const stop = css("--stop");
    for (const key of blocked) {
      const [i, j] = key.split("-").map(Number), a = px(i), b = px(j); if (!a || !b) continue;
      ctx.setLineDash([6, 4]); ctx.strokeStyle = stop; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.setLineDash([]);
      const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
      ctx.beginPath(); ctx.moveTo(mx - 6, my - 6); ctx.lineTo(mx + 6, my + 6); ctx.moveTo(mx + 6, my - 6); ctx.lineTo(mx - 6, my + 6); ctx.stroke();
    }
  }

  function drawPackets() {
    const minVis = MIN_VIS_MS * 1000 * speed, flashUs = FLASH_MS * 1000 * speed, stop = css("--stop");
    ctx.font = '600 9px "IBM Plex Sans", system-ui, sans-serif'; ctx.textAlign = "center";
    for (let i = air.length - 1; i >= 0; i--) {
      const f = air[i];
      if (f.t > T) continue;
      if (T - f.t > Math.max(MAX_VIS_US, minVis + flashUs)) break;
      if (!showChatter && CHATTER.has(f.kind)) continue;
      const vis = Math.max(f.end - f.t, minVis), p = (T - f.t) / vis, [color, label] = KIND_DEF[f.kind];
      const a = px(f.ta); if (!a) continue;
      if (p <= 1) {
        if (f.ra < 0) { // broadcast: a ring
          ctx.strokeStyle = color; ctx.globalAlpha = 0.5 * (1 - p); ctx.lineWidth = 1.2;
          ctx.beginPath(); ctx.arc(a[0], a[1], 4 + p * 8 * opts.scale(), 0, Math.PI * 2); ctx.stroke(); ctx.globalAlpha = 1;
          continue;
        }
        const b = px(f.ra); if (!b) continue;
        const x = a[0] + (b[0] - a[0]) * p, y = a[1] + (b[1] - a[1]) * p;
        ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, label ? 4.5 : 3.5, 0, Math.PI * 2); ctx.fill();
        if (label && speed <= 2) { ctx.fillStyle = css("--ink"); ctx.fillText(label, x, y - 8); }
      } else if (f.ack === false && f.ra >= 0 && T - f.end < flashUs + minVis) { // lost on this hop
        const b = px(f.ra); if (!b) continue;
        const x = a[0] + (b[0] - a[0]) * 0.8, y = a[1] + (b[1] - a[1]) * 0.8;
        ctx.strokeStyle = stop; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(x - 5, y - 5); ctx.lineTo(x + 5, y + 5); ctx.moveTo(x + 5, y - 5); ctx.lineTo(x - 5, y + 5); ctx.stroke();
      }
    }
  }

  // The real robots are drawn by operator.js; this only adds the model's verdict
  // around them: a dashed halo, red when the gateway would call it silent.
  function drawNodes() {
    const stop = css("--stop"), muted = css("--muted"), gwSilent = silent.get(0) || new Set();
    ctx.font = '500 10px "IBM Plex Sans", system-ui, sans-serif'; ctx.textAlign = "center";
    for (let i = 1; i < header.n; i++) {
      const a = px(i); if (!a) continue;
      const cut = dead.has(i) || gwSilent.has(i);
      ctx.setLineDash([3, 3]); ctx.strokeStyle = cut ? stop : muted; ctx.lineWidth = cut ? 2 : 1;
      ctx.beginPath(); ctx.arc(a[0], a[1], 22, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      if (dead.has(i)) { ctx.fillStyle = stop; ctx.fillText("model: switched off", a[0], a[1] - 27); }
      else if (gwSilent.has(i)) { ctx.fillStyle = stop; ctx.fillText("model: cut off", a[0], a[1] - 27); }
      if (pick && pick.got.includes(i)) { ctx.strokeStyle = css("--info"); ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(a[0], a[1], 25, 0, Math.PI * 2); ctx.stroke(); }
    }
    // With a pin, operator.js draws the gateway. Without one the model still has a
    // gateway (the map centre), and every route arrow ends there, so draw it as a
    // ghost: arrows into empty floor read as a bug (2026-10-24, live).
    const g0 = px(0);
    if (g0 && !opts.getGateway()) {
      ctx.globalAlpha = 0.6; ctx.setLineDash([3, 3]); ctx.strokeStyle = muted; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(g0[0], g0[1] - 10); ctx.lineTo(g0[0] + 9, g0[1] + 7); ctx.lineTo(g0[0] - 9, g0[1] + 7); ctx.closePath(); ctx.stroke();
      ctx.setLineDash([]); ctx.fillStyle = muted;
      ctx.fillText("Gateway (map centre)", g0[0], g0[1] + 21);
      ctx.fillText("drop a pin with ▲", g0[0], g0[1] + 33);
      ctx.globalAlpha = 1;
    }
    if (pick && pick.got.includes(0)) { const g = px(0); if (g) { ctx.strokeStyle = css("--info"); ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(g[0], g[1], 18, 0, Math.PI * 2); ctx.stroke(); } }
  }

  // -- controls ---------------------------------------------------------------------

  const $ = (s) => document.querySelector(s);
  const PICK_HINT = {
    link: "Click the two ends of the link to pull (again to restore it)",
    node: "Click a robot to switch it off in the model (again to bring it back)",
    send: "Click the sender, then the receiver",
  };
  function startPick(op, need) {
    pick = { op, need, got: [] };
    canvas.style.pointerEvents = "auto"; canvas.style.cursor = "crosshair";
    opts.hint(PICK_HINT[op]);
  }
  function endPick() {
    pick = null;
    if (canvas) { canvas.style.pointerEvents = "none"; canvas.style.cursor = ""; }
    if (opts) opts.hint(null);
  }
  function nodeAt(x, y) {
    let best = null, bd = 22;
    for (let i = 0; i < (header ? header.n : 0); i++) {
      const a = px(i); if (!a) continue;
      const d = Math.hypot(a[0] - x, a[1] - y);
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }
  function onPick(e) {
    if (!pick) return;
    const r = canvas.getBoundingClientRect(), i = nodeAt(e.clientX - r.left, e.clientY - r.top);
    if (i == null || pick.got.includes(i) || (pick.op === "node" && i === 0)) return;
    pick.got.push(i);
    if (pick.got.length < pick.need) return;
    const [a, b] = pick.got;
    if (pick.op === "node") cmd({ op: "node", id: a, action: dead.has(a) ? "revive" : "kill" });
    else if (pick.op === "link") cmd({ op: "link", a, b, action: blocked.has(`${Math.min(a, b)}-${Math.max(a, b)}`) ? "unblock" : "block" });
    else if (pick.op === "send") cmd({ op: "send", src: a, dst: b, size: 200 });
    endPick();
  }

  function renderBar() {
    const bar = $("#comms-bar"); if (!bar) return;
    const map = opts && opts.getMap();
    // Only what the operator can act on; the obstacle count is in __viz.state() for tests.
    $("#cm-state").textContent = !connected ? "Connecting to the model…" : !map ? "Waiting for a map"
      : opts.getGateway() ? "" : "No gateway pin: using the map centre";
    $("#cm-play").textContent = playing ? "Pause" : "Play";
    $("#cm-speed").value = String(speed);
    const rng = header && header.range_m ? Math.round(header.range_m.robot) : null;
    if (rng && document.activeElement !== $("#cm-range")) { $("#cm-range").value = String(rng); $("#cm-range-v").textContent = `${rng} m`; }
  }
  function renderFeed() {
    const el = $("#comms-feed"); if (!el) return;
    el.innerHTML = feed.slice(-5).reverse().map((f) => `<li class="${f.kind}">${f.text.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]))}</li>`).join("");
  }

  function wire() {
    $("#cm-play").addEventListener("click", () => cmd({ op: "pause", paused: playing }));
    $("#cm-speed").addEventListener("change", (e) => cmd({ op: "speed", speed: Number(e.target.value) }));
    $("#cm-link").addEventListener("click", () => startPick("link", 2));
    $("#cm-kill").addEventListener("click", () => startPick("node", 1));
    $("#cm-send").addEventListener("click", () => startPick("send", 2));
    $("#cm-chatter").addEventListener("change", (e) => { showChatter = e.target.checked; });
    let rangeTimer = null;
    $("#cm-range").addEventListener("input", (e) => {
      const v = Number(e.target.value); $("#cm-range-v").textContent = `${v} m`;
      clearTimeout(rangeTimer); rangeTimer = setTimeout(() => cmd({ op: "range", metres: v }), 150);
    });
    canvas.addEventListener("pointerdown", onPick);
    addEventListener("keydown", (e) => { if (e.key === "Escape" && pick) endPick(); });
    addEventListener("resize", resize);
  }

  // -- public -----------------------------------------------------------------------

  window.sortbotsComms = {
    // opts: {canvas, toPx(x, y) -> [px, py], scale() -> px per m, getMap() -> last
    //        OccupancyGrid, getRobots() -> [{id, pose}], getGateway() -> {x, y} | null,
    //        hint(text | null)}
    mount(o) { opts = o; canvas = o.canvas; ctx = canvas.getContext("2d"); wire(); resize(); },
    set(v) {
      if (v === on) return;
      on = v;
      canvas.hidden = $("#comms-bar").hidden = $("#comms-feed").hidden = !on;
      if (on) { resize(); connect(); lastWall = performance.now(); requestAnimationFrame(frame); renderBar(); }
      else { endPick(); disconnect(); }
    },
    get on() { return on; },
    sync() { if (on) { sync(); renderBar(); } },
    // The pin moved: the next sync posts it.
    gatewayMoved() { sentPos.delete(0); sync(); },
  };

  // Test hook (webui/tests/operator_test.mjs).
  window.__viz = {
    cmd,
    state: () => ({
      on, connected, n: header ? header.n : 0, t: T / 1e6, playing, speed, origin, racks,
      nodes: header ? header.nodes.map((n) => n.pos) : [], snr: header ? header.snr : [],
      routes: Object.fromEntries([...routes].map(([k, m]) => [k, Object.fromEntries(m)])),
      blocked: [...blocked], dead: [...dead], silent: Object.fromEntries([...silent].map(([k, s]) => [k, [...s]])),
      airCount: air.length, feed: feed.slice(-12).map((f) => f.text), pick: pick ? pick.op : null,
    }),
  };
})();
