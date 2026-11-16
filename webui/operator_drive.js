// SortBots operator view — the Drive view: the real robot's D435 and driving its omni base
// with a gamepad, on the machine the robot is plugged into.
//
// Talks ONLY to webui/serve.py over HTTP (no ROS: the laptop the robot is plugged into
// need not have it). serve.py answers these only when started with --drive, and only to a
// browser on that same machine (serve.py _drive_allowed):
//   GET  /api/drive          {enabled, running, camera, base, speed_levels, error?}
//   GET  /api/drive/frame    the latest D435 colour frame (JPEG)
//   POST /api/drive/start    open the camera and the wheel servos (scripts/drive_bridge.py)
//   POST /api/drive/stop     wheels to 0, torque off, both released
//   POST /api/drive/cmd      {x, y, w}: m/s forward, m/s left, rad/s counter-clockwise
//   POST /api/drive/pick/start  start handeye pick (on a pty, webui/drive_proxy.py)
//   POST /api/drive/pick/stop   its own quit: magnet off, base stopped, arm torque OFF
//   POST /api/drive/pick     handeye pick's one button: pick, carry, place, back to rest
//   POST /api/drive/abort    stop the arm where it is
//
// Two backends behind the same routes (webui/drive_proxy.py). With `handeye pick` running
// (the arm branch's pick, scripts/handeye.py) it owns the camera, arm, magnet and base, so
// the view goes through it ({backend: "handeye"}): pad A is its pick-and-place, B aborts,
// RT + sticks still drive the base between picks. Without it, Connect starts the drive-only
// bridge (scripts/drive_bridge.py) and there is no pick.
//
// Safety lives on the bridge, not here: it stops the wheels watchdog_s (0.3 s) after the
// last command, so commands go out ONLY while the operator holds the deadman (RT, or a
// drive key) and simply stop when they let go. One zero is still sent on release so the
// robot stops now rather than at the watchdog, but nothing depends on it arriving.
//
// The Gamepad API lists a pad only after one of its buttons is pressed with the page
// open (a browser privacy rule), and only in a secure context; localhost is one, which is
// the only place this view is served anyway.

(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const view = $("#driveview");
  if (!view) return;

  const SEND_MS = 50;      // 20 Hz: well inside the bridge's 0.3 s watchdog, even with a dropped request
  const FRAME_MS = 66;     // ~15 fps of a 30 fps stream: plenty to drive by, half the JPEGs
  const STATE_MS = 1000;
  const DEAD = 0.12;       // stick deadzone (scripts/xinput_pad.py uses the same)
  const RT_ON = 0.5;       // deadman: RT at least half pulled
  // Standard Gamepad mapping (w3c.github.io/gamepad): axes 0/1 left stick, 2/3 right stick,
  // buttons 7 RT, 12/13 d-pad up/down.
  const B_A = 0, B_B = 1, B_RT = 7, B_UP = 12, B_DOWN = 13;
  // handeye's flags while the arm moves on its own (handeye.py MOVING_FLAGS + the base back-off)
  const PICK_MOVING = new Set(["MOVING", "DESCENDING", "PRESSING", "LIFTING", "SEARCHING", "LOOKING"]);
  const KEYS = { w: [1, 0, 0], s: [-1, 0, 0], a: [0, 1, 0], d: [0, -1, 0], q: [0, 0, 1], e: [0, 0, -1] };

  const api = window.sortbotsDrive = { available: false, padToCmd, keysToCmd };
  let open = false, st = null, level = 0;
  let levels = [{ xy_mps: 0.1, theta_dps: 30 }];
  let wasActive = false, prevUp = false, prevDown = false, prevA = false, prevB = false;
  let note = null; // {text, until}: a pick refusal, shown in the error banner a while
  let inflight = false, queued = null;
  let tickTimer = null, stateTimer = null, frameTimer = null, frameToken = 0;
  const held = new Set();

  const dz = (v) => (Math.abs(v) < DEAD ? 0 : ((Math.abs(v) - DEAD) / (1 - DEAD)) * Math.sign(v));
  const btn = (pad, i) => (pad.buttons[i] ? pad.buttons[i].value || (pad.buttons[i].pressed ? 1 : 0) : 0);
  const speedOf = (lvl) => { const L = levels[Math.min(lvl, levels.length - 1)]; return [L.xy_mps, (L.theta_dps * Math.PI) / 180]; };

  // Stick up is NEGATIVE in the Gamepad API (screen y), and robot +y / +w are left /
  // counter-clockwise, so all three flip. A diagonal is no faster than straight ahead.
  function padToCmd(pad, lvl) {
    const [xy, th] = speedOf(lvl);
    const ax = (i) => dz(pad.axes[i] || 0);
    let x = -ax(1) * xy, y = -ax(0) * xy;
    const n = Math.hypot(x, y);
    if (n > xy) { x *= xy / n; y *= xy / n; }
    return { active: btn(pad, B_RT) >= RT_ON, x, y, w: -ax(2) * th };
  }
  function keysToCmd(keys, lvl) {
    const [xy, th] = speedOf(lvl);
    let x = 0, y = 0, w = 0;
    for (const k of keys) { const v = KEYS[k]; if (v) { x += v[0]; y += v[1]; w += v[2]; } }
    const n = Math.hypot(x, y);
    if (n > 1) { x /= n; y /= n; }
    return { active: keys.size > 0, x: x * xy, y: y * xy, w: Math.max(-1, Math.min(1, w)) * th };
  }

  // -- talking to serve.py -------------------------------------------------------------------
  const post = (url, body) => fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) })
    .then((r) => r.json().catch(() => ({}))).catch((e) => ({ ok: false, error: String(e) }));

  // Latest command wins: a slow request never builds a backlog of stale ones behind it.
  function send(cmd) {
    queued = { x: +cmd.x.toFixed(3), y: +cmd.y.toFixed(3), w: +cmd.w.toFixed(3) };
    if (!inflight) flush();
  }
  function flush() {
    const c = queued; queued = null; inflight = true;
    post("/api/drive/cmd", c).finally(() => { inflight = false; if (queued) flush(); });
  }
  function stopNow() { if (wasActive) { send({ x: 0, y: 0, w: 0 }); wasActive = false; } }

  // -- the control loop ----------------------------------------------------------------------
  function firstPad() {
    const list = navigator.getGamepads ? Array.from(navigator.getGamepads() || []) : [];
    return list.find((p) => p && p.connected) || null;
  }
  function tick() {
    const pad = firstPad();
    const padEl = $("#dv-pad");
    padEl.className = "dv-st " + (pad ? (pad.mapping === "standard" ? "ok" : "warn") : "");
    padEl.querySelector("span").textContent = pad ? pad.id.replace(/\s*\(.*$/, "").slice(0, 32) || "Gamepad" : "No gamepad";
    padEl.title = pad ? (pad.mapping === "standard" ? pad.id : `${pad.id}: not the standard layout, so the buttons may be off`)
      : "A gamepad seen by this browser. Press any button on it if it is plugged in but not listed";
    if (pad) {
      const up = btn(pad, B_UP) > 0.5, down = btn(pad, B_DOWN) > 0.5;
      if (up && !prevUp) level = Math.min(levels.length - 1, level + 1);
      if (down && !prevDown) level = Math.max(0, level - 1);
      prevUp = up; prevDown = down;
      const a = btn(pad, B_A) > 0.5, b = btn(pad, B_B) > 0.5;
      if (a && !prevA) pick();
      if (b && !prevB) abort();
      prevA = a; prevB = b;
    }
    let cmd = pad ? padToCmd(pad, level) : null;
    if (!(cmd && cmd.active) && held.size) cmd = keysToCmd(held, level);
    const active = !!(cmd && cmd.active) && !document.hidden;
    if (active) { send(cmd); wasActive = true; } else stopNow();
    renderLive(active ? cmd : null, pad);
  }
  function renderLive(cmd, pad) {
    const [xy, th] = speedOf(level);
    $("#dv-level").textContent = `Speed ${level + 1}/${levels.length} · ${xy.toFixed(1)} m/s · ${Math.round((th * 180) / Math.PI)}°/s`;
    $("#dv-live").classList.toggle("on", !!cmd);
    $("#dv-cmd").textContent = cmd
      ? `forward ${cmd.x.toFixed(2)} · left ${cmd.y.toFixed(2)} m/s · turn ${Math.round((cmd.w * 180) / Math.PI)}°/s`
      : pad ? "Hold RT to drive" : "Press a button on the gamepad, or hold W A S D";
    $("#dv-cmd").classList.toggle("muted", !cmd);
  }

  // -- pick and place (handeye pick) ---------------------------------------------------------
  const handeye = () => !!(st && st.running && st.backend === "handeye");
  async function pick() {
    if (!handeye()) { note = { text: "Pick & place needs the arm: press Start arm & pick", until: Date.now() + 8000 }; render(); return; }
    stopNow(); // the base stops before the arm moves (handeye stops it too)
    const r = await post("/api/drive/pick");
    if (r && r.ok === false) note = { text: r.error || "pick refused", until: Date.now() + 8000 };
    refresh();
  }
  async function abort() {
    if (!handeye()) return;
    await post("/api/drive/abort");
    refresh();
  }

  // -- state, camera -------------------------------------------------------------------------
  function setSt(el, ok, label, title) {
    el.className = "dv-st " + (ok === true ? "ok" : ok === false ? "bad" : "");
    el.querySelector("span").textContent = label;
    if (title) el.title = title;
  }
  function render() {
    const running = !!(st && st.running);
    const he = handeye();
    const starting = !!(st && st.pick_starting);
    $("#dv-connect").hidden = $("#dv-start-pick").hidden = running || starting;
    $("#dv-disconnect").hidden = !running || he; // the pick has its own Stop arm
    $("#dv-stop-pick").hidden = !he && !starting;
    $("#dv-pick").hidden = $("#dv-abort").hidden = !he;
    const ps = $("#dv-pickst"), pk = (st && st.pick) || {};
    ps.hidden = !he;
    if (he) {
      const moving = PICK_MOVING.has(pk.flag);
      ps.classList.toggle("moving", moving);
      ps.classList.toggle("aborted", pk.flag === "ABORTED");
      $("#dv-pflag").textContent = pk.flag || "";
      $("#dv-ptitle").textContent = pk.title ? `· ${pk.title}` : "";
      $("#dv-pmsg").textContent = pk.msg || (pk.place_set === false ? "No place spot taught yet: a pick stops holding the package" : "");
      $("#dv-pick").disabled = moving;
    }
    const cam = (st && st.camera) || {}, base = (st && st.base) || {};
    setSt($("#dv-cam"), running ? (cam.ok ? true : cam.error ? false : null) : null,
      cam.ok ? (he ? "Camera (handeye)" : `Camera ${cam.fps ? cam.fps.toFixed(0) + " fps" : ""}`.trim()) : "Camera");
    const ids = base.ids ? Object.values(base.ids).join("/") : "";
    setSt($("#dv-base"), running ? (base.ok ? true : base.error ? false : null) : null,
      base.ok && base.volts ? `Wheels ${base.volts.toFixed(1)} V` : base.ok && he ? "Wheels (handeye)" : "Wheels",
      `The three wheel servos${ids ? ` (ids ${ids} on ${base.port})` : ""}, configs/drive_base.yaml`);
    if (note && Date.now() > note.until) note = null;
    const errs = [note && note.text, st && st.error, st && st.pick_error, running && cam.error, running && base.error].filter(Boolean);
    const err = $("#dv-err");
    err.hidden = !errs.length;
    err.replaceChildren(...errs.map((t) => { const p = document.createElement("p"); p.textContent = t; return p; }));
    if (st && Array.isArray(st.speed_levels) && st.speed_levels.length) {
      levels = st.speed_levels; level = Math.min(level, levels.length - 1);
    }
    const msg = $("#dv-msg");
    if (starting) {
      msg.textContent = "Starting the arm: camera, magnet board, then the move to rest…";
      const boot = document.createElement("span"); boot.className = "dv-boot";
      boot.textContent = (st.pick_log || []).slice(-1)[0] || "";
      msg.append(boot);
      view.querySelector(".pic").style.backgroundImage = "";
    } else if (!running) { msg.textContent = "Not connected. Start arm & pick for the full robot, or Drive only for the camera and wheels."; view.querySelector(".pic").style.backgroundImage = ""; }
    else if (!cam.ok && !view.querySelector(".pic").style.backgroundImage) msg.textContent = cam.error ? "No camera picture" : "Starting the camera…";
  }
  async function refresh() {
    try {
      st = await (await fetch("/api/drive")).json();
    } catch (e) { st = { running: false, error: "serve.py is not answering" }; }
    if (st && st.enabled === false) { api.available = false; $('#view [data-v="drive"]').hidden = true; }
    render();
  }
  function frames(on) {
    clearTimeout(frameTimer); frameToken++;
    if (!on) return;
    const token = frameToken, pic = view.querySelector(".pic");
    const next = () => {
      if (token !== frameToken) return;
      // Not connected: no frames to ask for, and the "Not connected" message stays.
      if (!(st && st.running)) { frameTimer = setTimeout(next, 1000); return; }
      const probe = new Image();
      probe.onload = () => { if (token !== frameToken) return; pic.style.backgroundImage = `url(${probe.src})`; $("#dv-msg").textContent = ""; frameTimer = setTimeout(next, FRAME_MS); };
      probe.onerror = () => { if (token !== frameToken) return; frameTimer = setTimeout(next, 1000); };
      probe.src = `/api/drive/frame?_=${Date.now()}`;
    };
    next();
  }

  function setOpen(on) {
    if (on === open) return;
    open = on;
    clearInterval(tickTimer); clearInterval(stateTimer);
    if (!on) { held.clear(); stopNow(); frames(false); return; }
    refresh(); frames(true);
    tickTimer = setInterval(tick, SEND_MS);
    stateTimer = setInterval(refresh, STATE_MS / 2);
    tick();
  }
  addEventListener("sortbots:view", (e) => setOpen(e.detail === "drive"));

  $("#dv-connect").addEventListener("click", async () => {
    const b = $("#dv-connect");
    b.disabled = true; b.textContent = "Connecting…";
    const r = await post("/api/drive/start");
    b.disabled = false; b.textContent = "Connect";
    await refresh();
    if (r && r.ok === false && r.error) { st = { ...(st || {}), error: r.error }; render(); }
    frames(open);
  });
  $("#dv-start-pick").addEventListener("click", async () => {
    if (!confirm("Start the arm? It moves to its rest pose as soon as the program starts: keep clear of it.")) return;
    const b = $("#dv-start-pick");
    b.disabled = true;
    const r = await post("/api/drive/pick/start");
    b.disabled = false;
    if (r && r.ok === false) note = { text: r.error || "the arm's pick did not start", until: Date.now() + 15000 };
    refresh();
  });
  $("#dv-stop-pick").addEventListener("click", async () => {
    if (!confirm("Support the arm: its torque goes OFF when the pick program stops, and the magnet lets go. Stop it?")) return;
    held.clear(); stopNow();
    const b = $("#dv-stop-pick");
    b.disabled = true; b.textContent = "Stopping…";
    await post("/api/drive/pick/stop");
    b.disabled = false; b.textContent = "Stop arm";
    refresh();
  });
  $("#dv-pick").addEventListener("click", pick);
  $("#dv-abort").addEventListener("click", abort);
  $("#dv-disconnect").addEventListener("click", async () => {
    held.clear(); stopNow();
    await post("/api/drive/stop");
    refresh();
  });

  // Keys drive only while this view is open, from anywhere but a form field. Losing focus
  // or hiding the tab lets go of everything: the keyup would never arrive.
  addEventListener("keydown", (e) => {
    if (!open || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.target.closest && e.target.closest("input, select, textarea")) return;
    const k = e.key.toLowerCase();
    if (KEYS[k]) { e.preventDefault(); held.add(k); tick(); }
  });
  addEventListener("keyup", (e) => { if (held.delete(e.key.toLowerCase()) && open) tick(); });
  addEventListener("blur", () => { held.clear(); if (open) tick(); });
  document.addEventListener("visibilitychange", () => { if (document.hidden) { held.clear(); stopNow(); } });
  addEventListener("gamepadconnected", () => { if (open) tick(); });

  // Offered only where serve.py says this browser may drive (--drive, on localhost).
  fetch("/api/drive").then((r) => r.json()).then((s) => {
    if (!s || !s.enabled) return;
    api.available = true; st = s;
    const tab = $('#view [data-v="drive"]');
    tab.hidden = false;
    // The map only ever shows ROS robots; on the machine the real robot is plugged into it can
    // only say "waiting for the robots". Point the way out of it, and come back to this view if
    // it was the last one open (operator.js saves the view but restores only the ones it owns).
    const out = $("#empty-robot");
    if (out) { out.hidden = false; out.addEventListener("click", () => tab.click()); }
    try { if (localStorage.getItem("sortbots.mapView") === "drive") tab.click(); } catch (e) { /* no storage: start on the map */ }
    render();
  }).catch(() => { /* no drive here */ });
})();
