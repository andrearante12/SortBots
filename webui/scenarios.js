// SortBots dashboard — Scenarios tab. Starts and stops the simulation itself.
//
// Talks ONLY to webui/serve.py's control API over plain HTTP:
//   GET  /api/scenarios        configs/scenarios/*.yaml, plus whether the
//                              console is running (control: true/false)
//   GET  /api/maps             the saved-map library, for the `map` picker
//   GET  /api/session          current session state + phase
//   GET  /api/session/log      incremental console log, by byte offset
//   POST /api/session/start    {scenario, overrides, force}
//   POST /api/session/stop
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

  const listEl = document.getElementById("scenario-list");
  const warningEl = document.getElementById("console-warning");
  const stateEl = document.getElementById("session-state");
  const stopBtn = document.getElementById("session-stop");
  const logEl = document.getElementById("session-log");

  const POLL_ACTIVE_MS = 1000;
  const POLL_IDLE_MS = 3000;
  const ACTIVE_STATES = ["starting", "running", "stopping"];

  let scenarios = [];
  // The saved-map library (GET /api/maps), for the `map` override picker.
  // Readable without --control, like /api/scenarios, so the picker still shows
  // what's saved when the console is down.
  let maps = [];
  let hasControl = false;
  let session = null;
  let selected = null;
  // Upper bound for the `robots` override input, from configs/robots.yaml's
  // roster length (webui/serve.py's /api/robots) — not hardcoded, so adding
  // a robot_2 entry to the roster widens this automatically. Falls back to 2
  // (today's roster size) if the fetch fails; that's a UI-only ceiling, not
  // a security check (webui/session.py re-validates every override anyway).
  let maxRobots = 2;
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

  // Extra tooltip text for keys whose name can't answer "what does this cost
  // me". Only worth adding where the honest answer is surprising.
  const OVERRIDE_HINTS = {
    chase_cam:
      "Cosmetic 3rd-person view. Off by default because that render product " +
      "costs about 30% of real-time factor (0.47x -> 0.33x, scripts/bench_sim.sh); " +
      "the stage falls back to the head cam without it.",
    chase_cam_robots:
      "How many robots get a chase cam WHEN chase_cam is ticked. Only " +
      "robot_0's feed is ever displayed, so 1 is normally right — each extra " +
      "one is another render product.",
    map:
      "Which saved map to load (maps/, see maps/README.md). run_demo.sh " +
      "copies the entry to ~/.ros/sortbots_<robot_id>.db and runs on the " +
      "copy, so a run can never dirty the committed file — keeping what a " +
      "resume run added takes an explicit scripts/maps.sh save. Blank means " +
      "the working database, whatever the last run left behind.",
  };

  // Why a map can't be loaded, keyed by maps_lib.py's db_state. Shown in the
  // option's own label, because a silently-missing entry is the confusing case.
  const DB_STATE_NOTE = {
    pending: "pose graph not saved yet",
    pointer: "run git lfs pull",
    missing: "database file is gone",
  };

  function mapSelect() {
    const select = document.createElement("select");
    select.dataset.key = "map";

    const blank = document.createElement("option");
    blank.value = "";
    blank.textContent = "(working DB — ~/.ros/sortbots_<id>.db)";
    select.appendChild(blank);

    for (const m of maps) {
      const opt = document.createElement("option");
      opt.value = m.db_path || "";
      const loadable = m.status !== "invalid" && m.db_state === "complete";
      const free = m.coverage ? `${Math.round(m.coverage.free_m2)} m²` : "no grid";
      opt.textContent = loadable
        ? `${m.title || m.name} · ${free} · ${(m.created || "").slice(0, 10)}`
        : `${m.title || m.name} — (${m.error ? "invalid" : DB_STATE_NOTE[m.db_state] || m.db_state})`;
      opt.disabled = !loadable;
      select.appendChild(opt);
    }

    if (!maps.length) {
      const none = document.createElement("option");
      none.textContent = "(library is empty — scripts/maps.sh save NAME)";
      none.disabled = true;
      select.appendChild(none);
    }
    return select;
  }

  function overrideControl(scenario, key) {
    const value = scenario.run[key];
    const label = document.createElement("label");
    label.title = `Override ${key} for this run only (configs/scenarios/${scenario.name}.yaml sets ${value}).`;
    if (OVERRIDE_HINTS[key]) label.title += `\n\n${OVERRIDE_HINTS[key]}`;
    // `map` is a path out of the saved-map library, not a number or a flag —
    // a picker, deliberately, so no free-text path ever reaches the control
    // API (webui/session.py's _map_path re-validates regardless).
    if (key === "map") {
      label.append(document.createTextNode(key), mapSelect());
      return label;
    }
    const input = document.createElement("input");
    input.dataset.key = key;
    if (typeof value === "boolean") {
      input.type = "checkbox";
      input.checked = value;
      label.append(input, document.createTextNode(key));
    } else {
      input.type = "number";
      input.value = value;
      // chase_cam_robots may be 0 (no chase cams); robots stays 1..max.
      input.min = key === "chase_cam_robots" ? "0" : "1";
      input.max = String(maxRobots);
      label.append(document.createTextNode(key), input);
    }
    return label;
  }

  function readOverrides(card) {
    const out = {};
    for (const el of card.querySelectorAll("[data-key]")) {
      // A <select> reports type "select-one", so the old two-way ternary sent
      // Number("/path/to/map.db") — NaN — for the map picker. Branch on the
      // tag, and keep "" meaning "leave the scenario's own default alone"
      // (webui/session.py's _map_path maps null/"" to no --map flag at all).
      out[el.dataset.key] =
        el.type === "checkbox" ? el.checked
        : el.tagName === "SELECT" ? el.value
        : Number(el.value);
    }
    return out;
  }

  function renderScenarios() {
    listEl.textContent = "";
    if (!scenarios.length) {
      const empty = document.createElement("p");
      empty.textContent = "No scenarios found in configs/scenarios/.";
      listEl.appendChild(empty);
      return;
    }

    for (const scenario of scenarios) {
      const runnable = scenario.status === "ready" && hasControl && !isActive(session) && !busy;

      const card = document.createElement("div");
      card.className = "scenario-card";
      if (scenario.status !== "ready") card.classList.add("disabled");
      if (selected === scenario.name) card.classList.add("selected");

      const heading = document.createElement("h3");
      heading.textContent = scenario.title;
      const badge = document.createElement("span");
      badge.className = `sc-badge sc-${scenario.status}`;
      badge.textContent = scenario.status;
      heading.append(" ", badge);

      const body = document.createElement("p");
      body.textContent = scenario.error || scenario.description;

      const row = document.createElement("div");
      row.className = "scenario-row";
      for (const key of scenario.overrides || []) {
        if (key in scenario.run) row.appendChild(overrideControl(scenario, key));
      }

      const start = document.createElement("button");
      start.type = "button";
      start.textContent = isActive(session) ? "Stop the running session first" : "Start";
      start.disabled = !runnable;
      if (scenario.status === "planned") {
        start.title = "This scenario's environment doesn't exist yet.";
      } else if (scenario.status === "invalid") {
        start.title = "This scenario file failed validation — see the message above.";
      } else if (!hasControl) {
        start.title = "The dashboard console isn't running — see the note above.";
      }
      start.addEventListener("click", () => startSession(scenario.name, readOverrides(card)));
      row.appendChild(start);

      card.append(heading, body, row);
      listEl.appendChild(card);
    }
  }

  function renderSession() {
    stopBtn.disabled = !hasControl || !isActive(session) || busy;
    if (!session || session.state === "idle") {
      stateEl.textContent = hasControl ? "no session" : "console not running";
      return;
    }
    const bits = [session.scenario, session.state];
    if (session.phase_label && session.phase_label !== session.state) {
      bits.push(session.phase_label);
    }
    if (typeof session.elapsed_s === "number") {
      bits.push(`${Math.round(session.elapsed_s)}s`);
    }
    if (session.error) bits.push(session.error);
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

  async function loadScenarios() {
    try {
      const data = await getJson("/api/scenarios");
      scenarios = data.scenarios || [];
      hasControl = Boolean(data.control);
    } catch (e) {
      scenarios = [];
      hasControl = false;
      renderWarning(`Could not load scenarios: ${e.message}`);
    }
    // Never fatal: an empty library just means the picker offers the working
    // DB, and this tab's whole point is working when things are down.
    try {
      maps = (await getJson("/api/maps")).maps || [];
    } catch (e) {
      maps = [];
    }
    if (!hasControl) {
      renderWarning(
        "The dashboard console isn't running, so scenarios can't be launched from here. " +
        "Start it from a clean terminal with: scripts/run_console.sh"
      );
    } else {
      renderWarning(null);
    }
    renderScenarios();
    // Also the strip: hasControl decides whether it reads "no session" or
    // "console not running", and in read-only mode poll() bails before ever
    // rendering it — leaving the markup's placeholder text on screen.
    renderSession();
  }

  async function startSession(name, overrides) {
    if (busy) return;
    busy = true;
    selected = name;
    renderScenarios();
    logEl.textContent = "";
    logOffset = 0;
    logSessionId = null;
    try {
      session = await postJson("/api/session/start", { scenario: name, overrides });
    } catch (e) {
      renderWarning(`Could not start ${name}: ${e.message}`);
    } finally {
      busy = false;
    }
    renderSession();
    renderScenarios();
    schedulePoll(0);
  }

  async function stopSession() {
    if (busy) return;
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
    renderScenarios();
    schedulePoll(0);
  }

  stopBtn.addEventListener("click", stopSession);

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
        await loadScenarios();
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
    // The cards' enabled state depends on whether a session is live, so
    // re-render them on that edge (but not every second — the override inputs
    // are live DOM state the user may be mid-edit on).
    if (wasActive !== isActive(session)) renderScenarios();
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
  getJson("/api/robots")
    .then((data) => {
      if (Array.isArray(data.robots) && data.robots.length > 0) {
        maxRobots = data.robots.length;
      }
    })
    .catch(() => { /* keep the fallback of 2 */ })
    .then(() => loadScenarios())
    .then(() => schedulePoll(0));
})();
