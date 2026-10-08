// SortBots dashboard — Scenarios tab. Starts and stops the simulation itself.
//
// Talks ONLY to webui/serve.py's control API over plain HTTP:
//   GET  /api/launch/options   everything the launch form draws: platform,
//                              roster, saved maps, working DB, Quick start
//                              presets, and whether the console is running
//   POST /api/launch/preview   {config} -> the exact command + a summary
//   POST /api/presets          {name, config} -> configs/scenarios/NAME.yaml
//   GET  /api/session          current session state + phase
//   GET  /api/session/log      incremental console log, by byte offset
//   POST /api/session/start    {config} (the form) or {scenario, overrides}
//   POST /api/session/stop / /api/session/finish
//
// Deliberately no rosbridge: the whole point of this tab is to work when
// nothing is running, and rosbridge is one of the things that may not be.
// Also deliberately separate from app.js — nothing here touches ROS state, and
// app.js's ~15 topic subscriptions have nothing to say about a sim that hasn't
// started yet.
//
// The control API only exists when serve.py runs with --control, which only
// scripts/run_console.sh does. Without it every control endpoint answers 503
// and this tab explains that rather than showing dead buttons.

(function () {
  const viewLive = [
    document.getElementById("stage-panel"),
    document.getElementById("right-col"),
  ];
  const viewScenarios = document.getElementById("view-scenarios");
  const viewFleet = document.getElementById("view-fleet");
  // Live-only header chrome: the stage switcher and the SLAM badge are about
  // the robot, not about which sim to launch or the fleet radio monitor.
  const liveChrome = [
    document.getElementById("stage-mode"),
    document.getElementById("slam-badge"),
  ];

  const formEl = document.getElementById("launch-form");
  const warningEl = document.getElementById("console-warning");
  const stateEl = document.getElementById("session-state");
  const stopBtn = document.getElementById("session-stop");
  const finishBtn = document.getElementById("session-finish");
  const logEl = document.getElementById("session-log");
  const hintEl = document.getElementById("scenario-hint");

  const POLL_ACTIVE_MS = 1000;
  const POLL_IDLE_MS = 3000;
  const ACTIVE_STATES = ["starting", "running", "stopping"];

  // GET /api/launch/options. Readable without --control, so the form still
  // shows what could be launched (and why not) when the console is down.
  let options = null;
  let cfg = {}; // the form's current settings — what POST /api/session/start sends
  let previewOk = false;
  let hasControl = false;
  // "sim" (desktop, Isaac) or "real" (the robot's Jetson), from
  // /api/scenarios. The same page is served by both consoles; serve.py only
  // lists this platform's scenarios, so the tab just has to word itself right.
  let platform = "sim";
  let session = null;
  // null means "we haven't read the log yet" — the first read asks for the
  // tail so opening the tab mid-run doesn't dump the whole file.
  let logOffset = null;
  let logSessionId = null;
  let pollTimer = null;
  let busy = false; // a start/stop request is in flight

  // -- view switching -----------------------------------------------------

  let currentView = "live";

  function setView(view) {
    currentView = view;
    for (const el of viewLive) el.hidden = view !== "live";
    for (const el of liveChrome) el.hidden = view !== "live";
    if (viewScenarios) viewScenarios.hidden = view !== "scenarios";
    if (viewFleet) viewFleet.hidden = view !== "fleet";
    for (const btn of document.querySelectorAll("#view-mode button")) {
      btn.classList.toggle("active", btn.dataset.view === view);
    }
    if (view === "scenarios") {
      // Jump straight to a fresh poll instead of waiting out the idle timer.
      schedulePoll(0);
    }
  }

  for (const btn of document.querySelectorAll("#view-mode button")) {
    btn.addEventListener("click", () => setView(btn.dataset.view));
  }

  // -- rendering ----------------------------------------------------------

  function isActive(s) {
    return s && ACTIVE_STATES.includes(s.state);
  }

  // -- the launch form ------------------------------------------------------

  const $ = (id) => document.getElementById(id);
  const presetSel = $("lf-preset");
  const mapSel = $("lf-map");
  const robotsSel = $("lf-robots");
  const chaseCountSel = $("lf-chase_cam_robots");
  const startBtn = $("lf-start");
  const SEGMENTS = { scene: $("lf-scene"), mode: $("lf-mode"), waypoints: $("lf-waypoints") };
  const CHECKS = ["explore", "headless", "chase_cam", "teleop", "bag"];
  const CFG_KEY = "sortbots.launchConfig";

  // Why a saved map can't be loaded, keyed by maps_lib.py's db_state. Shown in
  // the option's own label: a silently-missing entry is the confusing case.
  const DB_STATE_NOTE = {
    pending: "pose graph not saved yet",
    pointer: "run git lfs pull",
    missing: "database file is gone",
  };

  function option(sel, value, text, disabled) {
    const o = document.createElement("option");
    o.value = value;
    o.textContent = text;
    o.disabled = !!disabled;
    sel.appendChild(o);
    return o;
  }

  function libraryMap(name) {
    return (options && options.maps || []).find((m) => m.name === name);
  }

  // Fill the selects from /api/launch/options. Values survive the rebuild.
  function buildChoices() {
    presetSel.textContent = "";
    option(presetSel, "", "— custom —");
    for (const p of options.presets) {
      const o = option(presetSel, p.name, p.title, p.status !== "ready");
      if (p.origin === "dashboard") o.textContent += "  ★";
    }

    mapSel.textContent = "";
    option(mapSel, "new", "New map (empty)");
    const w = options.working_db;
    option(mapSel, "working",
           w.exists ? "Working map (left by the last run, ~/.ros)"
                    : "Working map — none yet (no ~/.ros/sortbots_robot_0.db)", !w.exists);
    for (const m of options.maps) {
      const ok = !m.error && m.db_state === "complete";
      const bits = [m.title];
      if (m.free_m2 != null) bits.push(`${Math.round(m.free_m2)} m²`);
      if (m.waypoints) bits.push(`${m.waypoints} waypoint${m.waypoints === 1 ? "" : "s"}`);
      option(mapSel, m.name, ok ? `${m.name} — ${bits.join(" · ")}`
                                : `${m.name} — (${m.error ? "invalid" : DB_STATE_NOTE[m.db_state] || m.db_state})`, !ok);
    }

    robotsSel.textContent = "";
    chaseCountSel.textContent = "";
    options.roster.forEach((_, i) => {
      option(robotsSel, String(i + 1), String(i + 1));
      option(chaseCountSel, String(i + 1), `on ${i + 1} robot${i ? "s" : ""}`);
    });

    // Only the fields this platform takes (the robot console has no scene,
    // fleet, window or chase cam).
    const fields = new Set(options.fields);
    for (const el of formEl.querySelectorAll("[data-field]")) {
      el.hidden = !fields.has(el.dataset.field);
    }
    $("lf-advanced").hidden = !["headless", "chase_cam", "teleop", "bag"].some((f) => fields.has(f));
  }

  // cfg -> controls (+ the dependent bits: mode only matters for a non-new
  // map, the chase count only with chase on, "from map" only for a library map).
  function renderForm() {
    if (!options) return;
    // "from map" means nothing without a library map: never show it selected
    // on a disabled button (or let the summary claim "map's waypoints").
    if (cfg.waypoints === "map" && !libraryMap(cfg.map)) cfg.waypoints = "keep";
    for (const [key, seg] of Object.entries(SEGMENTS)) {
      for (const b of seg.querySelectorAll("button")) b.classList.toggle("active", b.dataset.v === cfg[key]);
    }
    mapSel.value = cfg.map;
    robotsSel.value = String(cfg.robots);
    chaseCountSel.value = String(cfg.chase_cam_robots);
    for (const k of CHECKS) $(`lf-${k}`).checked = !!cfg[k];

    const fields = new Set(options.fields);
    $("lf-mode-row").hidden = !fields.has("mode") || cfg.map === "new";
    chaseCountSel.hidden = !fields.has("chase_cam") || !cfg.chase_cam;
    $("lf-robot-ids").textContent = options.roster.slice(0, cfg.robots).join(", ");

    const lib = libraryMap(cfg.map);
    const wpBtns = Object.fromEntries([...SEGMENTS.waypoints.querySelectorAll("button")].map((b) => [b.dataset.v, b]));
    wpBtns.map.textContent = lib ? `from map (${lib.waypoints})` : "from map";
    wpBtns.map.disabled = !lib;
    wpBtns.keep.textContent = `keep current (${options.working_waypoints})`;

    // Things that will launch fine but probably aren't what you meant.
    const notes = [];
    if (lib && lib.scene && fields.has("scene") && lib.scene !== cfg.scene) {
      notes.push(`${lib.name} was mapped in the ${lib.scene} scene, not ${cfg.scene}.`);
    }
    if (cfg.map !== "new" && cfg.mode === "readonly" && cfg.explore && fields.has("explore")) {
      notes.push("Read-only never adds to the map, so exploring won't grow it — use extend for that.");
    }
    if (cfg.map !== "new" && cfg.mode === "extend") {
      notes.push("Runs on a copy: keep what it maps with Save & finish.");
    }
    $("lf-map-note").textContent = notes.join(" ");
    updateStartButton();
  }

  // controls -> cfg, on any edit. An edit means "no longer exactly a preset".
  function readForm() {
    cfg.map = mapSel.value;
    cfg.robots = Number(robotsSel.value) || 1;
    cfg.chase_cam_robots = Number(chaseCountSel.value) || 1;
    for (const k of CHECKS) cfg[k] = $(`lf-${k}`).checked;
  }

  function onEdit() {
    presetSel.value = "";
    $("lf-preset-note").textContent = "";
    readForm();
    saveCfg();
    renderForm();
    schedulePreview();
  }

  for (const [key, seg] of Object.entries(SEGMENTS)) {
    seg.addEventListener("click", (ev) => {
      const b = ev.target.closest("button[data-v]");
      if (!b || b.disabled) return;
      cfg[key] = b.dataset.v;
      onEdit();
    });
  }
  for (const el of [mapSel, robotsSel, chaseCountSel, ...CHECKS.map((k) => $(`lf-${k}`))]) {
    el.addEventListener("change", () => {
      // Picking a library map defaults its waypoints back to the map's own.
      if (el === mapSel && libraryMap(mapSel.value)) cfg.waypoints = "map";
      onEdit();
    });
  }

  presetSel.addEventListener("change", () => {
    const p = options.presets.find((x) => x.name === presetSel.value);
    if (!p) return;
    if (!p.config) {
      $("lf-preset-note").textContent = `Can't show "${p.title}" in the form: ${p.reason}.`;
      return;
    }
    cfg = { ...options.defaults, ...p.config };
    saveCfg();
    $("lf-preset-note").textContent = p.description;
    renderForm();
    presetSel.value = p.name; // renderForm doesn't touch it; keep the label
    schedulePreview();
  });

  // Per-browser memory of the last setup — a convenience, so it fails soft.
  function saveCfg() {
    try { localStorage.setItem(CFG_KEY, JSON.stringify(cfg)); } catch (e) { /* private window */ }
  }
  function restoreCfg() {
    let saved = null;
    try { saved = JSON.parse(localStorage.getItem(CFG_KEY) || "null"); } catch (e) { saved = null; }
    cfg = { ...options.defaults };
    if (saved && typeof saved === "object") {
      for (const k of Object.keys(cfg)) if (k in saved) cfg[k] = saved[k];
    }
    // A remembered map that has since vanished (or a working DB that's gone)
    // must not leave the select pointing at nothing.
    const valid = [...mapSel.options].some((o) => o.value === cfg.map && !o.disabled);
    if (!valid) cfg.map = "new";
    if (cfg.robots > options.roster.length) cfg.robots = options.roster.length;
  }

  // The server's own build_argv decides what's launchable; the form just asks.
  let previewTimer = null;
  let previewSeq = 0;
  function schedulePreview() {
    clearTimeout(previewTimer);
    previewTimer = setTimeout(runPreview, 150);
  }
  async function runPreview() {
    const seq = ++previewSeq;
    const body = {};
    for (const f of options.fields) if (f in cfg) body[f] = cfg[f];
    try {
      const out = await postJson("/api/launch/preview", { config: body });
      if (seq !== previewSeq) return; // a newer edit superseded this answer
      previewOk = true;
      $("lf-summary").textContent = out.summary;
      $("lf-command").textContent = out.command.replace(/^\S*\//, "");
      $("lf-error").textContent = "";
    } catch (e) {
      if (seq !== previewSeq) return;
      previewOk = false;
      $("lf-summary").textContent = "";
      $("lf-command").textContent = "";
      $("lf-error").textContent = e.message;
    }
    updateStartButton();
  }

  function formConfig() {
    const body = {};
    for (const f of options.fields) if (f in cfg) body[f] = cfg[f];
    return body;
  }

  function updateStartButton() {
    const live = isActive(session);
    startBtn.disabled = !hasControl || live || busy || !previewOk;
    startBtn.textContent = live ? "Running…" : "Start";
    startBtn.title = !hasControl ? "The dashboard console isn't running — see the note above."
      : live ? "A session is running — Save & finish or Stop it first (right)."
      : !previewOk ? "Fix the setting shown in red first." : "Launch with these settings";
  }

  formEl.addEventListener("submit", (ev) => {
    ev.preventDefault();
    startSession();
  });

  $("lf-save-preset").addEventListener("click", async () => {
    if (!hasControl) {
      renderWarning("Saving a preset needs the dashboard console (it writes configs/scenarios/).");
      return;
    }
    const name = (prompt("Preset name (a-z 0-9 - _), saved as configs/scenarios/NAME.yaml:") || "").trim();
    if (!name) return;
    const title = (prompt("Title shown in Quick start:", $("lf-summary").textContent || name) || "").trim();
    const payload = { name, title: title || name, config: formConfig() };
    try {
      await postJson("/api/presets", payload);
    } catch (e) {
      if (!/already exists/.test(e.message) || !confirm(`Replace the preset "${name}"?`)) {
        renderWarning(`Could not save preset: ${e.message}`);
        return;
      }
      try {
        await postJson("/api/presets", { ...payload, overwrite: true });
      } catch (e2) {
        renderWarning(`Could not save preset: ${e2.message}`);
        return;
      }
    }
    renderWarning(null);
    await loadOptions();
    presetSel.value = name;
  });

  // A "Save & finish" in flight (session.finish.step not done/failed) owns the
  // session: neither button may start a second teardown under it.
  function isFinishing(s) {
    return !!(s && s.finish && !["done", "failed"].includes(s.finish.step));
  }

  function renderSession() {
    stopBtn.disabled = !hasControl || !isActive(session) || busy || isFinishing(session);
    finishBtn.disabled = !hasControl || !session || session.state !== "running" || busy ||
      isFinishing(session);
    finishBtn.textContent = session && session.read_only && session.map_name
      ? "Save waypoints & finish" : "Save & finish";
    if (!session || session.state === "idle") {
      stateEl.textContent = hasControl ? "no session" : "console not running";
      return;
    }
    // A form launch is "custom"; its one-line summary says what it actually is.
    const bits = [session.scenario === "custom" ? session.summary || "custom" : session.scenario,
                  session.state];
    if (session.phase_label && session.phase_label !== session.state) {
      bits.push(session.phase_label);
    }
    if (typeof session.elapsed_s === "number") {
      bits.push(`${Math.round(session.elapsed_s)}s`);
    }
    if (session.error) bits.push(session.error);
    if (session.finish) {
      const f = session.finish;
      bits.push(f.step === "done" ? `saved to maps/${f.map} ✓`
        : f.step === "failed" ? `save to maps/${f.map} FAILED: ${f.error || "see log"}`
        : `maps/${f.map}: ${f.step}…`);
    }
    stateEl.textContent = bits.join(" · ");
  }

  function renderWarning(message) {
    if (!message) {
      warningEl.hidden = true;
      warningEl.textContent = "";
      return;
    }
    warningEl.hidden = false;
    warningEl.textContent = "";
    warningEl.append(document.createTextNode(message));
  }

  function appendLog(text) {
    if (!text) return;
    // Only stick to the bottom if the user is already there — otherwise
    // scrolling back through a failure gets yanked away on every poll.
    const atBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 40;
    logEl.textContent += text;
    if (atBottom) logEl.scrollTop = logEl.scrollHeight;
  }

  // -- API ----------------------------------------------------------------

  async function getJson(url) {
    const res = await fetch(url, { cache: "no-store" });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw Object.assign(new Error(body.error || res.statusText), { status: res.status });
    return body;
  }

  async function postJson(url, payload) {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
    });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw Object.assign(new Error(body.error || res.statusText), { status: res.status });
    return body;
  }

  async function loadOptions() {
    try {
      options = await getJson("/api/launch/options");
      hasControl = Boolean(options.control);
      platform = options.platform === "real" ? "real" : "sim";
    } catch (e) {
      options = null;
      hasControl = false;
      renderWarning(`Could not load launch options: ${e.message}`);
      renderSession();
      return;
    }
    renderPlatform();
    if (!hasControl) {
      renderWarning(
        "The dashboard console isn't running, so nothing can be launched from here. " +
        (platform === "real"
          ? "Start it on the robot's Jetson with: scripts/jetson.sh console"
          : "Start it from a clean terminal with: scripts/run_console.sh")
      );
    } else {
      renderWarning(null);
    }
    buildChoices();
    restoreCfg();
    renderForm();
    schedulePreview();
    // Also the strip: hasControl decides whether it reads "no session" or
    // "console not running", and in read-only mode poll() bails before ever
    // rendering it — leaving the markup's placeholder text on screen.
    renderSession();
  }

  function renderPlatform() {
    if (platform === "real") {
      hintEl.textContent =
        "Real robot — choose a map and start the RealSense camera and RTAB-Map on this Jetson.";
      hintEl.title =
        "Launches scripts/run_robot.sh. Quick start presets are configs/scenarios/*.yaml with " +
        "platform: real. This dashboard stays up throughout.";
      stopBtn.title = "Tear down the camera and the ROS 2 stack. The dashboard console stays up.";
    }
    // sim: index.html's own wording already describes Isaac Sim.
  }

  async function startSession() {
    if (busy || !options) return;
    busy = true;
    updateStartButton();
    logEl.textContent = "";
    logOffset = 0;
    logSessionId = null;
    try {
      session = await postJson("/api/session/start", { config: formConfig() });
    } catch (e) {
      renderWarning(`Could not start: ${e.message}`);
    } finally {
      busy = false;
    }
    renderSession();
    updateStartButton();
    schedulePoll(0);
  }

  async function stopSession() {
    if (busy) return;
    // Stop discards whatever the run mapped: runs work on a copy in ~/.ros,
    // never on maps/. Only a read-only localize run has nothing to lose.
    if (session && session.state === "running" && !session.read_only &&
        !confirm("Stop WITHOUT saving? Anything this run mapped is not in the maps/ library " +
                 "(use Save & finish to keep it).")) return;
    busy = true;
    renderSession();
    try {
      session = await postJson("/api/session/stop", {});
    } catch (e) {
      renderWarning(`Could not stop the session: ${e.message}`);
    } finally {
      busy = false;
    }
    renderSession();
    updateStartButton();
    schedulePoll(0);
  }

  stopBtn.addEventListener("click", stopSession);

  async function finishSession() {
    if (busy || !session) return;
    const readOnly = session.read_only && session.map_name;
    const name = readOnly ? session.map_name
      : (prompt("Save this run into maps/ as:", session.map_name || "") || "").trim();
    if (!name) return;
    if (readOnly && !confirm(`Save the current waypoints into maps/${name} and stop? ` +
                             "(This run loaded the map read-only, so the map itself is unchanged.)")) return;
    busy = true;
    renderSession();
    try {
      session = await postJson("/api/session/finish", { name });
    } catch (e) {
      renderWarning(`Could not save & finish: ${e.message}`);
    } finally {
      busy = false;
    }
    renderSession();
    schedulePoll(0);
  }
  finishBtn.addEventListener("click", finishSession);

  async function poll() {
    if (!hasControl) {
      schedulePoll(POLL_IDLE_MS * 3);
      return;
    }
    const wasActive = isActive(session);
    try {
      const next = await getJson("/api/session");
      // A different session id means someone (another tab, a restarted
      // console) started a new run — reset the log window rather than
      // appending the new run's output onto the old one's.
      if (next.session_id !== logSessionId) {
        logSessionId = next.session_id;
        logEl.textContent = "";
        logOffset = null;
      }
      session = next;
    } catch (e) {
      if (e.status === 503) {
        hasControl = false;
        await loadOptions();
        schedulePoll(POLL_IDLE_MS);
        return;
      }
    }

    if (session && session.session_id) {
      try {
        const chunk = await getJson(
          `/api/session/log?offset=${logOffset === null ? -1 : logOffset}`
        );
        appendLog(chunk.text);
        logOffset = chunk.offset;
      } catch (e) {
        /* the log lags the session by design; a failed tail is not fatal */
      }
    }

    renderSession();
    // Start's enabled state depends on whether a session is live. On the
    // session ENDING, also reload options: the run may have saved a map or
    // left a working DB that the Map picker should now offer.
    if (wasActive !== isActive(session)) {
      updateStartButton();
      if (!isActive(session)) loadOptions();
    }
    schedulePoll(isActive(session) ? POLL_ACTIVE_MS : POLL_IDLE_MS);
  }

  function schedulePoll(delay) {
    clearTimeout(pollTimer);
    // Polling only matters while this tab is on screen and the document is
    // visible; a phone in a pocket shouldn't keep hitting the server.
    if (currentView !== "scenarios" && !isActive(session)) {
      pollTimer = setTimeout(poll, POLL_IDLE_MS * 4);
      return;
    }
    pollTimer = setTimeout(poll, delay);
  }

  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) schedulePoll(0);
  });

  setView("live");
  loadOptions().then(() => schedulePoll(0));
})();
