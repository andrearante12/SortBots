// commsim replay — see index.html. One trace (build_trace.py), replayed on a
// 2D canvas. Everything here is indexed once at load into typed arrays so a
// 100k-event trace scrubs at 60 fps: the only per-frame work is a binary
// search for the active window and a draw of what is in it.
//
// Time inside the replay is `T`, integer microseconds since header.t0_ns —
// the same unit as every event's `t`. Visible durations are stretched to a
// minimum of MIN_VIS_MS of WALL time at the current speed: a 400 µs hop is
// invisible otherwise, and the point is to see it.

window.__pageErrors = [];
window.addEventListener("error", (e) => window.__pageErrors.push(String(e.message)));
window.addEventListener("unhandledrejection", (e) => window.__pageErrors.push(String(e.reason)));

(function () {
  "use strict";

  const KINDS =["status", "zenoh", "tcp_ack", "icmp", "ogm", "bcast", "arp", "mgmt", "frag", "other"];
  const COLORS = { status: "#4cc9f0", zenoh: "#c77dff", tcp_ack: "#7a5cff", icmp: "#ffd166",
                   ogm: "#5c636a", bcast: "#8d99ae", arp: "#8d99ae", mgmt: "#495057",
                   frag: "#f4a261", other: "#adb5bd" };
  const DEFAULT_HIDDEN = new Set(["ogm", "mgmt", "arp", "bcast", "tcp_ack", "other"]);
  const MIN_VIS_MS = 120;     // wall-clock floor for one hop's animation
  const MAX_VIS_US = 1_500_000; // never look further back than this for active packets
  const FLASH_MS = 350;       // wall-clock length of a drop / failure flash
  const BCAST_RANGE_M = 8;    // how far a broadcast ring expands
  const PANEL_MS = 250;

  const $ = (id) => document.getElementById(id);
  const floor = $("floor"), tl = $("timeline"), ctx = floor.getContext("2d"), tctx = tl.getContext("2d");

  // ---------------------------------------------------------------- state
  let trace = null;
  let A = null;          // indexed air events
  let S = null;          // state-changing events in time order (route, link, silence, linkstat, drop, mark)
  let si = 0;            // next state event to apply
  let T = 0, endT = 0;
  let playing = false, speed = 1, lastWall = 0, lastPanel = 0;
  let selected = null, routeMode = "gateway";
  const hidden = new Set(DEFAULT_HIDDEN);
  let routes = new Map();  // node -> Map(dst -> {nh, tq})
  let blocked = new Set(); // "a-b" with a<b
  let silent = new Map();  // obs -> Set(peer)
  let flashes = [];        // {t, kind:"lost"|"drop"|"fail", a, b, x, y}
  let staticLayer = null, staticKey = "";
  let world = null;        // world -> pixel mapping
  let kindIdx = {};
  KINDS.forEach((k, i) => { kindIdx[k] = i; });

  function bisect(arr, x) { // first index with arr[i] >= x
    let lo = 0, hi = arr.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] < x) lo = m + 1; else hi = m; }
    return lo;
  }

  // ---------------------------------------------------------------- load
  async function loadFromUrl(url) {
    const r = await fetch(url);
    if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
    let body = r.body;
    if (/\.gz($|\?)/.test(url) || (r.headers.get("content-type") || "").includes("gzip")) {
      body = body.pipeThrough(new DecompressionStream("gzip"));
    }
    load(JSON.parse(await new Response(body).text()));
  }

  async function loadFromFile(file) {
    let blob = file;
    if (/\.gz$/.test(file.name)) blob = new Response(file.stream().pipeThrough(new DecompressionStream("gzip")));
    load(JSON.parse(await blob.text()));
  }

  function load(obj) {
    if (!obj || obj.format !== "commsim-viz/1" || !obj.header || !Array.isArray(obj.events)) {
      throw new Error("not a commsim-viz/1 trace");
    }
    trace = obj;
    const ev = obj.events;
    const air = ev.filter((e) => e.k === "air");
    const n = air.length;
    A = { n, ev: air, t: new Float64Array(n), end: new Float64Array(n), ta: new Int16Array(n),
          ra: new Int16Array(n), kind: new Uint8Array(n), ack: new Uint8Array(n), ok: new Uint8Array(n) };
    for (let i = 0; i < n; i++) {
      const e = air[i];
      A.t[i] = e.t; A.end[i] = e.t + (e.dur || 0); A.ta[i] = e.ta; A.ra[i] = e.ra;
      A.kind[i] = kindIdx[e.kind] ?? kindIdx.other;
      A.ack[i] = e.ack === true ? 1 : e.ack === false ? 0 : 2;
      A.ok[i] = e.ok === "ok" ? 1 : e.ok === "lost" ? 2 : e.ok === "lost_after" ? 3 : 0;
    }
    S = ev.filter((e) => e.k !== "air" && e.k !== "app_tx" && e.k !== "app_rx");
    // per (dst, src) receive times and per src send times, for the panel
    A.rx = new Map(); A.tx = new Map();
    for (const e of ev) {
      if (e.k === "app_rx") { const k = `${e.dst},${e.src}`; (A.rx.get(k) || A.rx.set(k, []).get(k)).push(e); }
      else if (e.k === "app_tx") { (A.tx.get(e.src) || A.tx.set(e.src, []).get(e.src)).push(e.t); }
    }
    for (const [k, v] of A.rx) A.rx.set(k, { t: Float64Array.from(v, (e) => e.t), seq: v.map((e) => e.seq) });
    for (const [k, v] of A.tx) A.tx.set(k, Float64Array.from(v));
    endT = ev.length ? Math.max(ev[ev.length - 1].t, n ? A.end[n - 1] : 0) : 0;
    if (obj.header.duration_s) endT = Math.max(endT, obj.header.duration_s * 1e6);
    staticLayer = null; selected = null;
    $("name").textContent = `${obj.header.name} · n=${obj.header.n} · ${obj.header.transport}`;
    $("hint").style.display = "none";
    buildFilters(); buildLegend();
    seek(0); pause(); renderPanel(); drawAll();
  }

  // ---------------------------------------------------------------- replay state
  function resetState() {
    routes = new Map(); blocked = new Set(); silent = new Map(); flashes = []; si = 0;
  }

  function applyState(e) {
    switch (e.k) {
      case "route": {
        let m = routes.get(e.node); if (!m) routes.set(e.node, (m = new Map()));
        if (e.nh < 0) m.delete(e.dst); else m.set(e.dst, { nh: e.nh, tq: e.tq });
        break;
      }
      case "link": {
        const key = `${Math.min(e.a, e.b)}-${Math.max(e.a, e.b)}`;
        if (e.action === "block") blocked.add(key); else blocked.delete(key);
        break;
      }
      case "robot_silent": { let s = silent.get(e.obs); if (!s) silent.set(e.obs, (s = new Set())); s.add(e.peer); break; }
      case "robot_back": { const s = silent.get(e.obs); if (s) s.delete(e.peer); break; }
      case "linkstat": if (e.failed > 0) flashes.push({ t: e.t, kind: "fail", a: e.a, b: e.b }); break;
      case "drop": flashes.push({ t: e.t, kind: "drop", a: e.src }); break;
      default: break;
    }
  }

  function advanceTo(t) {
    if (t < T) { resetState(); }
    while (si < S.length && S[si].t <= t) applyState(S[si++]);
    T = t;
    const flashUs = FLASH_MS * 1000 * speed;
    flashes = flashes.filter((f) => T - f.t < flashUs);
  }

  function seek(t) { advanceTo(Math.max(0, Math.min(endT, t))); drawAll(); updateTime(); }

  function play() { if (!trace) return; playing = true; lastWall = performance.now(); $("play").textContent = "⏸"; requestAnimationFrame(frame); }
  function pause() { playing = false; $("play").textContent = "▶"; }

  function frame(now) {
    if (!playing) return;
    const dt = (now - lastWall) * 1000 * speed; // µs of trace time
    lastWall = now;
    advanceTo(Math.min(endT, T + dt));
    if (T >= endT) pause();
    drawAll(); updateTime();
    if (now - lastPanel > PANEL_MS) { lastPanel = now; renderPanel(); }
    if (playing) requestAnimationFrame(frame);
  }

  function updateTime() { $("time").textContent = `${(T / 1e6).toFixed(3)} s`; }

  // ---------------------------------------------------------------- geometry
  function fitWorld() {
    const lay = trace.header.layout;
    const pad = 28;
    const W = floor.clientWidth, H = floor.clientHeight;
    const ox = lay.origin ? lay.origin[0] : 0, oy = lay.origin ? lay.origin[1] : 0;
    const s = Math.min((W - 2 * pad) / lay.width, (H - 2 * pad) / lay.height);
    const x0 = (W - lay.width * s) / 2, y0 = (H - lay.height * s) / 2;
    world = { s, toX: (x) => x0 + (x - ox) * s, toY: (y) => H - (y0 + (y - oy) * s), W, H };
  }

  function nodePos(i) { const nd = trace.header.nodes[i]; return nd ? [world.toX(nd.pos[0]), world.toY(nd.pos[1])] : [0, 0]; }

  function buildStatic() {
    const lay = trace.header.layout, h = trace.header;
    const c = document.createElement("canvas"); c.width = floor.width; c.height = floor.height;
    const g = c.getContext("2d");
    g.scale(devicePixelRatio, devicePixelRatio);
    g.fillStyle = "#13161b";
    g.fillRect(world.toX(lay.origin ? lay.origin[0] : 0), world.toY((lay.origin ? lay.origin[1] : 0) + lay.height),
               lay.width * world.s, lay.height * world.s);
    g.fillStyle = "#262b33";
    for (const r of lay.racks) g.fillRect(world.toX(r[0]), world.toY(r[3]), (r[2] - r[0]) * world.s, (r[3] - r[1]) * world.s);
    g.fillStyle = "#1e2229";
    for (const r of lay.shelves || []) g.fillRect(world.toX(r[0]), world.toY(r[3]), (r[2] - r[0]) * world.s, (r[3] - r[1]) * world.s);
    // SNR links: stronger = brighter and wider
    const min = h.knobs.snr_link_min_db ?? 4;
    for (let i = 0; i < h.n; i++) for (let j = i + 1; j < h.n; j++) {
      const snr = Math.max(h.snr[i][j], h.snr[j][i]);
      if (snr < min) continue;
      const q = Math.min(1, (snr - min) / 25);
      const [ax, ay] = nodePos(i), [bx, by] = nodePos(j);
      g.strokeStyle = `rgba(120,160,200,${0.08 + 0.35 * q})`; g.lineWidth = 0.5 + 2 * q;
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

  let drawn = { packets: {}, flashes: 0, blocked: 0, routes: 0 };

  function drawAll() {
    if (!trace) return;
    const key = `${floor.width}x${floor.height}`;
    if (!staticLayer || staticKey !== key) { fitWorld(); buildStatic(); staticKey = key; }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.drawImage(staticLayer, 0, 0);
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    drawn = { packets: {}, flashes: 0, blocked: 0, routes: 0 };
    drawRoutes(); drawBlocked(); drawPackets(); drawFlashes(); drawNodes(); drawTimeline();
  }

  function arrow(ax, ay, bx, by, color, width) {
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    const ang = Math.atan2(by - ay, bx - ax), L = 7;
    const mx = (ax + bx) / 2, my = (ay + by) / 2;
    ctx.beginPath(); ctx.moveTo(mx + L * Math.cos(ang), my + L * Math.sin(ang));
    ctx.lineTo(mx + L * Math.cos(ang + 2.5), my + L * Math.sin(ang + 2.5));
    ctx.lineTo(mx + L * Math.cos(ang - 2.5), my + L * Math.sin(ang - 2.5)); ctx.closePath(); ctx.fill();
  }

  function drawRoutes() {
    if (routeMode === "none") return;
    const n = trace.header.n;
    const pairs = [];
    if (routeMode === "gateway") { for (let i = 1; i < n; i++) pairs.push([i, 0]); }
    else if (selected != null) { for (let d = 0; d < n; d++) if (d !== selected) pairs.push([selected, d]); }
    const seen = new Set();
    for (const [src, dst] of pairs) {
      let cur = src;
      for (let hop = 0; hop < n && cur !== dst; hop++) {
        const r = routes.get(cur)?.get(dst);
        if (!r) break;
        const key = `${cur}>${r.nh}`;
        if (!seen.has(key)) {
          seen.add(key);
          const [ax, ay] = nodePos(cur), [bx, by] = nodePos(r.nh);
          arrow(ax, ay, bx, by, `rgba(80,200,120,${0.25 + 0.5 * (r.tq / 255)})`, 1.5);
          drawn.routes++;
        }
        cur = r.nh;
      }
    }
  }

  function drawBlocked() {
    for (const key of blocked) {
      const [a, b] = key.split("-").map(Number);
      const [ax, ay] = nodePos(a), [bx, by] = nodePos(b);
      ctx.setLineDash([6, 4]); ctx.strokeStyle = "#ff3b3b"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
      const mx = (ax + bx) / 2, my = (ay + by) / 2;
      ctx.beginPath(); ctx.moveTo(mx - 6, my - 6); ctx.lineTo(mx + 6, my + 6); ctx.moveTo(mx + 6, my - 6); ctx.lineTo(mx - 6, my + 6); ctx.stroke();
      drawn.blocked++;
    }
  }

  function drawPackets() {
    if (!A.n) return;
    const minVis = MIN_VIS_MS * 1000 * speed;
    const flashUs = FLASH_MS * 1000 * speed;
    const lo = bisect(A.t, T - Math.max(MAX_VIS_US, minVis + flashUs)), hi = bisect(A.t, T + 1);
    for (let i = lo; i < hi; i++) {
      const kind = KINDS[A.kind[i]];
      if (hidden.has(kind)) continue;
      const vis = Math.max(A.end[i] - A.t[i], minVis);
      const p = (T - A.t[i]) / vis;
      const color = COLORS[kind];
      const [ax, ay] = nodePos(A.ta[i]);
      if (p <= 1) {
        if (A.ra[i] < 0) { // broadcast: expanding ring
          ctx.strokeStyle = color; ctx.globalAlpha = 0.5 * (1 - p); ctx.lineWidth = 1;
          ctx.beginPath(); ctx.arc(ax, ay, 3 + p * BCAST_RANGE_M * world.s, 0, Math.PI * 2); ctx.stroke(); ctx.globalAlpha = 1;
        } else {
          const [bx, by] = nodePos(A.ra[i]);
          const x = ax + (bx - ax) * p, y = ay + (by - ay) * p;
          ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, 3.5, 0, Math.PI * 2); ctx.fill();
          if (A.ok[i] === 1 && p > 0.85) { ctx.strokeStyle = "#8f8"; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.stroke(); }
        }
        drawn.packets[kind] = (drawn.packets[kind] || 0) + 1;
      } else if (A.ack[i] === 0 && T - A.end[i] < flashUs + minVis && A.ra[i] >= 0) {
        // lost on this hop: red X where it should have landed
        const [bx, by] = nodePos(A.ra[i]);
        const x = ax + (bx - ax) * 0.8, y = ay + (by - ay) * 0.8;
        ctx.strokeStyle = "#ff4d4d"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(x - 5, y - 5); ctx.lineTo(x + 5, y + 5); ctx.moveTo(x + 5, y - 5); ctx.lineTo(x - 5, y + 5); ctx.stroke();
        drawn.flashes++;
      }
    }
  }

  function drawFlashes() {
    for (const f of flashes) {
      if (f.kind === "fail") {
        const [ax, ay] = nodePos(f.a), [bx, by] = nodePos(f.b);
        ctx.strokeStyle = "rgba(255,120,60,0.8)"; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      } else { // unroutable drop at the sender
        const [ax, ay] = nodePos(f.a);
        ctx.strokeStyle = "#ff4d4d"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(ax, ay, 12, 0, Math.PI * 2); ctx.stroke();
      }
      drawn.flashes++;
    }
  }

  function drawNodes() {
    const h = trace.header;
    for (let i = 0; i < h.n; i++) {
      const [x, y] = nodePos(i);
      const gw = h.nodes[i].gateway;
      const nSilent = silent.get(i)?.size || 0;
      ctx.fillStyle = gw ? "#ffd166" : "#4cc9f0";
      ctx.strokeStyle = selected === i ? "#fff" : "#0e1013"; ctx.lineWidth = selected === i ? 2.5 : 1.5;
      ctx.beginPath();
      if (gw) ctx.rect(x - 7, y - 7, 14, 14); else ctx.arc(x, y, 7, 0, Math.PI * 2);
      ctx.fill(); ctx.stroke();
      ctx.fillStyle = "#cde"; ctx.font = "11px monospace"; ctx.textAlign = "center";
      ctx.fillText(gw ? "gw" : String(i), x, y - 11);
      if (nSilent) { ctx.fillStyle = "#f77"; ctx.fillText(`${nSilent} silent`, x, y + 20); }
    }
  }

  function drawTimeline() {
    const W = tl.clientWidth, H = tl.clientHeight;
    tctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    tctx.fillStyle = "#0e1013"; tctx.fillRect(0, 0, W, H);
    tctx.fillStyle = "#1c1f24"; tctx.fillRect(0, H / 2 - 2, W, 4);
    const x = (t) => (endT ? (t / endT) * W : 0);
    for (const e of S) {
      if (e.k === "link") { tctx.fillStyle = e.action === "block" ? "#ff3b3b" : "#8f8"; tctx.fillRect(x(e.t) - 1, 6, 2, H - 12); }
      else if (e.k === "robot_silent") { tctx.fillStyle = "#fb7"; tctx.fillRect(x(e.t) - 0.5, H / 2 - 10, 1, 10); }
      else if (e.k === "robot_back") { tctx.fillStyle = "#7bf"; tctx.fillRect(x(e.t) - 0.5, H / 2, 1, 10); }
      else if (e.k === "mark") { tctx.fillStyle = "#556"; tctx.fillRect(x(e.t) - 0.5, 2, 1, H - 4); }
    }
    tctx.fillStyle = "#fff"; tctx.fillRect(x(T) - 1, 0, 2, H);
    tctx.fillStyle = "#9aa"; tctx.font = "10px monospace"; tctx.textAlign = "left";
    tctx.fillText("0 s", 3, H - 3); tctx.textAlign = "right"; tctx.fillText(`${(endT / 1e6).toFixed(0)} s`, W - 3, H - 3);
  }

  // ---------------------------------------------------------------- side panel
  function fmt(x, d = 1) { return x == null || !Number.isFinite(x) ? "—" : x.toFixed(d); }

  function lastHeard(obs, peer) {
    const r = A.rx.get(`${obs},${peer}`);
    if (!r) return null;
    const i = bisect(r.t, T + 1) - 1;
    return i >= 0 ? { t: r.t[i], seq: r.seq[i], i } : null;
  }

  function renderPanel() {
    if (!trace) return;
    const h = trace.header, body = $("side-body");
    const tmo = h.knobs.status_timeout_s * 1e6;
    if (selected == null) {
      let alive = 0;
      for (let i = 1; i < h.n; i++) if (!silent.get(0)?.has(i)) alive++;
      const c = h.counts;
      body.innerHTML =
        `<table><tr><th>robots</th><td>${h.n - 1} + gateway</td></tr>` +
        `<tr><th>alive (gateway's view)</th><td>${alive} / ${h.n - 1}</td></tr>` +
        `<tr><th>blocked links</th><td>${blocked.size}</td></tr>` +
        `<tr><th>status delivery</th><td>${c.delivery == null ? "—" : (100 * c.delivery).toFixed(1) + " %"}</td></tr>` +
        `<tr><th>lost hops</th><td>${c.unacked}</td></tr>` +
        `<tr><th>outcomes</th><td>${Object.entries(c.outcomes || {}).map(([k, v]) => `${k} ${v}`).join(", ") || "—"}</td></tr>` +
        `<tr><th>air frames</th><td>${Object.entries(c.air_by_kind || {}).map(([k, v]) => `${k} ${v}`).join(", ")}</td></tr>` +
        `</table><h2>Limits</h2><div class="limits">${h.limits.map((s) => "• " + esc(s)).join("\n")}</div>` +
        `<h2>Hint</h2><div class="limits">click a node for its view of the fleet</div>`;
      return;
    }
    const nd = h.nodes[selected];
    const rows = [];
    for (let p = 0; p < h.n; p++) {
      if (p === selected) continue;
      const lh = lastHeard(selected, p);
      const isSilent = silent.get(selected)?.has(p);
      const age = lh ? (T - lh.t) / 1e6 : null;
      // delivery over the last 10 s: receptions vs what the peer sent
      const r = A.rx.get(`${selected},${p}`), tx = A.tx.get(p);
      let got = 0, sent = 0;
      if (r) got = bisect(r.t, T + 1) - bisect(r.t, T - 10e6);
      if (tx) sent = bisect(tx, T + 1) - bisect(tx, T - 10e6);
      rows.push(`<tr class="${isSilent ? "silent" : ""}"><td>${p === 0 ? "gw" : p}</td>` +
                `<td>${age == null ? "never" : fmt(age) + " s"}</td><td>${lh ? lh.seq : "—"}</td>` +
                `<td>${sent ? ((100 * got) / sent).toFixed(0) + " %" : "—"}</td>` +
                `<td>${isSilent ? "SILENT" : age != null && age * 1e6 > tmo ? "stale" : "ok"}</td></tr>`);
    }
    const rt = [...(routes.get(selected) || [])].map(([d, r]) => `${d === 0 ? "gw" : d}→${r.nh === 0 ? "gw" : r.nh} (${r.tq})`).join(", ");
    body.innerHTML =
      `<table><tr><th>node</th><td>${nd.gateway ? "gateway" : "robot " + selected} · ${esc(nd.ns)}</td></tr>` +
      `<tr><th>ip / mac</th><td>${esc(nd.ip)} / ${esc(nd.mac)}</td></tr>` +
      `<tr><th>pos</th><td>${fmt(nd.pos[0])}, ${fmt(nd.pos[1])} m</td></tr>` +
      `<tr><th>next hops</th><td>${rt || "—"}</td></tr></table>` +
      `<h2>Its view of the fleet</h2><table><tr><th>peer</th><th>heard</th><th>seq</th><th>10 s</th><th></th></tr>${rows.join("")}</table>` +
      `<h2>Hint</h2><div class="limits">click the floor to deselect</div>`;
  }

  function esc(s) { return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;"); }

  // ---------------------------------------------------------------- controls
  function buildFilters() {
    const f = $("filters"); f.innerHTML = "";
    const present = new Set(Object.keys(trace.header.counts.air_by_kind || {}));
    for (const k of KINDS) {
      if (!present.has(k)) continue;
      const id = `f-${k}`;
      const l = document.createElement("label");
      l.innerHTML = `<input type="checkbox" id="${id}" ${hidden.has(k) ? "" : "checked"}> <span style="color:${COLORS[k]}">${k}</span>`;
      l.querySelector("input").addEventListener("change", (ev) => { if (ev.target.checked) hidden.delete(k); else hidden.add(k); drawAll(); });
      f.appendChild(l);
    }
  }

  function buildLegend() {
    $("legend").innerHTML = KINDS.filter((k) => trace.header.counts.air_by_kind?.[k])
      .map((k) => `<span style="--c:${COLORS[k]}">${k}</span>`).join("") +
      `<span style="--c:#ff4d4d">lost / blocked</span><span style="--c:#50c878">route</span>`;
  }

  function stateEvents() { return S.filter((e) => e.k === "link" || e.k === "robot_silent" || e.k === "robot_back" || e.k === "mark"); }
  function nextEvent() { const e = stateEvents().find((x) => x.t > T + 1); if (e) seek(e.t); }
  function prevEvent() { const es = stateEvents().filter((x) => x.t < T - 1); if (es.length) seek(es[es.length - 1].t); }

  $("play").addEventListener("click", () => (playing ? pause() : play()));
  $("next").addEventListener("click", nextEvent);
  $("prev").addEventListener("click", prevEvent);
  $("speed").addEventListener("change", (e) => { speed = Number(e.target.value); });
  $("routes").addEventListener("change", (e) => { routeMode = e.target.value; drawAll(); });
  $("file").addEventListener("change", (e) => { if (e.target.files[0]) loadFromFile(e.target.files[0]).catch(fail); });
  document.addEventListener("keydown", (e) => {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    if (e.code === "Space") { e.preventDefault(); playing ? pause() : play(); }
    else if (e.code === "ArrowRight") seek(T + 1e6 * (e.shiftKey ? 10 : 1));
    else if (e.code === "ArrowLeft") seek(T - 1e6 * (e.shiftKey ? 10 : 1));
  });
  document.addEventListener("dragover", (e) => e.preventDefault());
  document.addEventListener("drop", (e) => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) loadFromFile(f).catch(fail); });
  floor.addEventListener("click", (e) => {
    if (!trace) return;
    const r = floor.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
    let best = null, bd = 14;
    for (let i = 0; i < trace.header.n; i++) { const [nx, ny] = nodePos(i); const d = Math.hypot(nx - x, ny - y); if (d < bd) { bd = d; best = i; } }
    selected = best; renderPanel(); drawAll();
  });
  let scrubbing = false;
  const scrub = (e) => { const r = tl.getBoundingClientRect(); seek(((e.clientX - r.left) / r.width) * endT); };
  tl.addEventListener("mousedown", (e) => { scrubbing = true; scrub(e); });
  window.addEventListener("mousemove", (e) => { if (scrubbing) scrub(e); });
  window.addEventListener("mouseup", () => { scrubbing = false; });
  window.addEventListener("resize", resize);

  function fail(err) { $("hint").style.display = ""; $("hint").textContent = String(err.message || err); console.error(err); }

  // ---------------------------------------------------------------- test hook
  window.__viz = {
    load, seek: (s) => seek(s * 1e6), play, pause,
    speed: (x) => { speed = x; $("speed").value = String(x); },
    select: (i) => { selected = i; renderPanel(); drawAll(); },
    setFilter: (kind, on) => { if (on) hidden.delete(kind); else hidden.add(kind); drawAll(); },
    state: () => ({
      loaded: !!trace, t: T / 1e6, n: trace ? trace.header.n : 0, playing, speed,
      active: drawn.packets, flashes: drawn.flashes, routesDrawn: drawn.routes,
      blocked: [...blocked], routes: Object.fromEntries([...routes].map(([k, m]) => [k, Object.fromEntries(m)])),
      silent: Object.fromEntries([...silent].map(([k, s]) => [k, [...s]])), selected,
      panel: $("side-body").textContent,
    }),
  };

  resize();
  const q = new URLSearchParams(location.search).get("trace");
  if (q) loadFromUrl(q).catch(fail);
})();
