// SortBots dashboard — Scenarios tab. Starts and stops the simulation itself.
//
// Talks ONLY to webui/serve.py's control API over plain HTTP:
//   GET  /api/scenarios        configs/scenarios/*.yaml, plus whether the
//                              console is running (control: true/false)
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
  // Live-only header chrome: the stage switcher and the SLAM badge are about
  // the robot, not about which sim to launch.
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
    viewScenarios.hidden = view !== "scenarios";
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

  function overrideControl(scenario, key) {
    const value = scenario.run[key];
    const label = document.createElement("label");
    label.title = `Override ${key} for this run only (configs/scenarios/${scenario.name}.yaml sets ${value}).`;
    let input;
    if (typeof value === "boolean") {
      input = document.createElement("input");
      input.type = "checkbox";
      input.checked = value;
      label.append(input, document.createTextNode(key));
    } else if (value === "auto") {
      // robots: auto (scripts/_hw_budget.py sizes it to the host's VRAM/RAM at
      // launch) — a <input type=number> can't hold a non-numeric value, so
      // offer "auto" as its own option alongside the fixed counts.
      input = document.createElement("select");
      const autoOpt = document.createElement("option");
      autoOpt.value = "auto";
      autoOpt.textContent = "auto";
      input.append(autoOpt);
      for (let n = 1; n <= maxRobots; n++) {
        const opt = document.createElement("option");
        opt.value = String(n);
        opt.textContent = String(n);
        input.append(opt);
      }
      input.value = "auto";
      label.append(document.createTextNode(key), input);
    } else {
      input = document.createElement("input");
      input.type = "number";
      input.value = value;
      // chase_cam_robots may be 0 (no chase cams); robots stays 1..max.
      input.min = key === "chase_cam_robots" ? "0" : "1";
      input.max = String(maxRobots);
      label.append(document.createTextNode(key), input);
    }
    input.dataset.key = key;
    return label;
  }

  function readOverrides(card) {
    const out = {};
    for (const input of card.querySelectorAll("[data-key]")) {
      if (input.type === "checkbox") {
        out[input.dataset.key] = input.checked;
      } else if (input.tagName === "SELECT") {
        out[input.dataset.key] = input.value === "auto" ? "auto" : Number(input.value);
      } else {
        out[input.dataset.key] = Number(input.value);
      }
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
