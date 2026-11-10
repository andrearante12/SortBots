// The "Run a fleet sim" form in the Tests tab. It starts the same run as
// `python3 -m fleetsim run`, shows the log, and can stop it.
//
// The server side is webui/fleetsim_runner.py, through these serve.py routes:
//   GET  /api/fleetsim/scenarios   what the form can offer
//   GET  /api/fleetsim/launch      state and log of the current or last run
//   POST /api/fleetsim/launch      start a run
//   POST /api/fleetsim/cancel      stop it

(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const POLL_MS = 1000;

  const VERDICTS = {
    PASS: { text: "finished", tone: "pass" },
    FAIL: { text: "finished, a pass criterion failed", tone: "fail" },
    DONE: { text: "finished, nothing to score", tone: "" },
    STOPPED: { text: "stopped early, so it has no score", tone: "" },
    CRASHED: { text: "crashed before it could be scored, the log has the error", tone: "fail" },
  };

  const form = $("#fr-form");
  let catalog = null; // loaded once per page load
  let run = null; // the latest state from the server

  function tabIsOpen() {
    return !$("#session").hidden && $("#session .card").classList.contains("tab-tests");
  }

  function formatClock(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = String(Math.floor(seconds % 60)).padStart(2, "0");
    return `${mins}:${secs}`;
  }

  function warn(text) {
    $("#fr-warn").hidden = !text;
    $("#fr-warn").textContent = text;
  }

  async function api(url, payload) {
    let options = { cache: "no-store" };
    if (payload !== undefined) {
      options = {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      };
    }
    const res = await fetch(url, options);
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(body.error || res.statusText);
    return body;
  }

  function overrideField(key, def, scenario) {
    const select = document.createElement("select");
    select.dataset.set = key;
    select.title = `--set ${key}`;
    select.add(new Option(`scenario default (${scenario.settings[key]})`, ""));
    for (const option of def.options) select.add(new Option(option));

    const label = document.createElement("label");
    label.append(def.label, select);
    return label;
  }

  // Picking a scenario resets the form to that scenario's own values.
  function applyScenario() {
    const name = $("#fr-scenario").value;
    const scenario = catalog.scenarios.find((s) => s.name === name);
    $("#fr-about").textContent = scenario.title;
    $("#fr-robots").value = scenario.robots;
    $("#fr-duration").value = scenario.duration_s;
    $("#fr-seed").value = scenario.seed;

    const fields = Object.entries(catalog.settable).map(([key, def]) => overrideField(key, def, scenario));
    $("#fr-sets").replaceChildren(...fields);
  }

  async function loadCatalog() {
    catalog = await api("/api/fleetsim/scenarios");
    for (const scenario of catalog.scenarios) $("#fr-scenario").add(new Option(scenario.name));
    $("#fr-note").hidden = catalog.control;
    applyScenario();
    render();
  }

  function readForm() {
    const sets = {};
    for (const select of form.querySelectorAll("[data-set]")) {
      if (select.value) sets[select.dataset.set] = select.value;
    }
    return {
      name: $("#fr-scenario").value,
      robots: Number($("#fr-robots").value),
      duration: Number($("#fr-duration").value),
      seed: Number($("#fr-seed").value),
      sets,
    };
  }

  function render() {
    $("#fr-run").disabled = !catalog?.control || !!run?.running;

    const box = $("#fr-status");
    box.hidden = !run?.name;
    if (box.hidden) return;

    // There's no verdict while the run is still going.
    const verdict = VERDICTS[run.verdict] || { text: "", tone: "" };
    box.classList.toggle("done", !run.running);
    box.classList.toggle("pass", verdict.tone === "pass");
    box.classList.toggle("fail", verdict.tone === "fail");

    $("#fr-name").textContent = run.name;
    $("#fr-state").textContent = run.running ? `running · ${formatClock(run.elapsed_s)}` : verdict.text;

    const badge = $("#fr-verdict");
    badge.hidden = !run.verdict;
    badge.textContent = run.verdict || "";
    badge.className = `rp-v ${verdict.tone}`;

    $("#fr-stop").hidden = !run.running;
    $("#fr-replay").hidden = run.running;

    const log = $("#fr-log");
    if (log.textContent !== run.log) {
      log.textContent = run.log;
      log.scrollTop = log.scrollHeight;
    }
  }

  $("#fr-scenario").addEventListener("change", applyScenario);

  form.addEventListener("submit", async (event) => {
    event.preventDefault();
    $("#fr-run").disabled = true;
    try {
      run = await api("/api/fleetsim/launch", readForm());
      warn("");
    } catch (err) {
      warn(`Couldn't start it: ${err.message}`);
    }
    render();
  });

  $("#fr-stop").addEventListener("click", async () => {
    try {
      run = await api("/api/fleetsim/cancel", {});
      render();
    } catch (err) {
      warn(`Stop failed: ${err.message}`);
    }
  });

  $("#fr-replay").addEventListener("click", () => {
    $("#sp-x").click(); // close the panel, the replay plays on the map behind it
    window.sortbotsReplay.open(run.dir);
  });

  // Only asks about the run while the tab is open. If a run ends while the panel
  // is closed, `run` still says running, so we catch the change when it reopens.
  async function poll() {
    try {
      if (!catalog) await loadCatalog();
      if (catalog.control && tabIsOpen()) {
        const wasRunning = run?.running;
        run = await api("/api/fleetsim/launch");
        if (wasRunning && !run.running) window.sortbotsReplay.refreshRuns();
        render();
      }
    } catch (err) {
      warn(`Fleet sim unavailable: ${err.message}`);
    }
    setTimeout(poll, POLL_MS);
  }

  poll();
})();
