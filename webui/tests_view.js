// SortBots dashboard — Tests tab. Scored test cases (configs/tests/) and their results.
//
// Talks ONLY to webui/serve.py over plain HTTP, like scenarios.js:
//   GET  /api/tests            test cases (validated) + the metric catalog
//   GET  /api/tests/results    recent result dirs (data/simtests/), newest first
//   GET  /api/tests/report     ?dir=NAME -> that run's report.md
//   GET  /api/tests/run        ?offset= runner state + incremental log
//   POST /api/tests/run        {name, repeats?, variant?}
//   POST /api/tests/cancel
//
// The work is scripts/simtest.py; this tab is a launcher and a reader. A
// running test drives the sim through ordinary dashboard sessions, so the
// "live" view shows it — actors at their true positions on the map — while it
// is being scored. Running needs the sim console (--control); reading doesn't.
//
// View switching lives in scenarios.js (it owns #view-mode); this file only
// polls while #view-tests is shown.

(function () {
  const viewEl = document.getElementById("view-tests");
  if (!viewEl) return;
  const $ = (id) => document.getElementById(id);
  const listEl = $("tests-list");
  const resultsEl = $("tests-results");
  const reportEl = $("tests-report");
  const logEl = $("tests-log");
  const stateEl = $("tests-run-state");
  const cancelBtn = $("tests-cancel");
  const warningEl = $("tests-warning");

  const POLL_RUNNING_MS = 1000;
  const POLL_IDLE_MS = 4000;
  const TESTS_REFRESH_MS = 15000;

  let tests = [];
  let control = false;
  let run = null; // last /api/tests/run
  let logOffset = null;
  let timer = null;
  let testsAt = 0;
  let openReport = null; // result dir whose report is shown

  async function getJson(url) {
    const res = await fetch(url, { cache: "no-store" });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw Object.assign(new Error(body.error || res.statusText), { status: res.status });
    return body;
  }

  async function postJson(url, payload) {
    const res = await fetch(url, {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
    });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw Object.assign(new Error(body.error || res.statusText), { status: res.status });
    return body;
  }

  function warn(message) {
    warningEl.hidden = !message;
    warningEl.textContent = message || "";
  }

  function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text != null) e.textContent = text;
    return e;
  }

  const isRunning = () => !!(run && run.running);

  // -- test cards ---------------------------------------------------------

  function renderTests() {
    listEl.textContent = "";
    if (!tests.length) {
      listEl.append(el("div", "lf-note", "No test cases in configs/tests/."));
      return;
    }
    for (const t of tests) {
      const card = el("div", `test-card${t.status === "ready" ? "" : " invalid"}`);
      const h = el("h4", null, t.title);
      h.append(el("code", null, t.name));
      card.append(h);
      if (t.status !== "ready") {
        card.append(el("p", "lf-warn", t.error));
        listEl.append(card);
        continue;
      }
      if (t.description) card.append(el("p", null, t.description));
      card.append(el("div", "meta",
        `${t.scenario}${t.world ? ` · world ${t.world}` : ""} · ${t.robots.join(", ")} · ` +
        `≤${t.sim_s}s sim · ${t.variants.length} variant(s) × ${t.repeats}`));
      const ul = el("ul");
      for (const m of t.metrics) {
        ul.append(el("li", null, m.criteria.length
          ? `${m.id}: ${m.criteria.join(", ")}` : `${m.id} (reported)`));
      }
      card.append(ul);

      const row = el("div", "row");
      let variantSel = null;
      if (t.variants.length > 1) {
        variantSel = el("select");
        variantSel.append(new Option("all variants", ""));
        for (const v of t.variants) variantSel.append(new Option(v, v));
        row.append(variantSel);
      }
      const reps = el("input");
      reps.type = "number";
      reps.min = "1";
      reps.max = "50";
      reps.value = String(t.repeats);
      reps.title = "Repeats per variant — nothing here is deterministic, so one run is an anecdote";
      row.append(el("span", "lf-note", "repeats"), reps);
      const btn = el("button", "test-run-btn", "Run");
      btn.type = "button";
      btn.disabled = !control || isRunning();
      btn.title = !control ? "Needs the sim dashboard console (scripts/run_console.sh)"
        : isRunning() ? "A test is already running" : "Launch the sim and score this test";
      btn.addEventListener("click", () => startTest(t.name, Number(reps.value) || t.repeats,
        variantSel && variantSel.value ? variantSel.value : null));
      row.append(btn);
      card.append(row);
      listEl.append(card);
    }
  }

  async function startTest(name, repeats, variant) {
    const payload = { name, repeats };
    if (variant) payload.variant = variant;
    try {
      run = await postJson("/api/tests/run", payload);
      warn(null);
      logEl.textContent = "";
      logOffset = 0;
    } catch (e) {
      warn(`Could not start ${name}: ${e.message}`);
    }
    renderTests();
    schedule(0);
  }

  cancelBtn.addEventListener("click", async () => {
    if (!confirm("Cancel the running test? The sim is stopped; finished runs are still reported.")) return;
    try {
      run = await postJson("/api/tests/cancel");
    } catch (e) {
      warn(`Cancel failed: ${e.message}`);
    }
    schedule(0);
  });

  // -- run state + log ------------------------------------------------------

  function renderRun() {
    cancelBtn.disabled = !control || !isRunning();
    if (!control) {
      stateEl.textContent = "read-only — run tests from scripts/simtest.py or the sim console";
    } else if (!run || !run.name) {
      stateEl.textContent = "idle";
    } else if (run.running) {
      stateEl.textContent = `running ${run.name} · ${Math.round(run.elapsed_s || 0)}s — ` +
        "switch to live to watch";
    } else {
      const how = { 0: "passed", 1: "failed", 2: "invalid", 3: "infrastructure error",
                    130: "cancelled" }[run.exit];
      stateEl.textContent = `last: ${run.name} ${how || `exit ${run.exit}`}`;
    }
  }

  function appendLog(text) {
    if (!text) return;
    const atBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 40;
    logEl.textContent += text;
    if (atBottom) logEl.scrollTop = logEl.scrollHeight;
  }

  // -- results --------------------------------------------------------------

  function fmtDir(dir) {
    const m = /^(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})/.exec(dir);
    return m ? `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}Z` : dir;
  }

  function renderResults(results) {
    resultsEl.textContent = "";
    if (!results.length) {
      resultsEl.append(el("div", "lf-note", "No results yet."));
      return;
    }
    for (const r of results) {
      const row = el("div", "result-row");
      const inFlight = r.state === "running" && !r.complete;
      const verdict = inFlight ? "RUNNING" : r.verdict || "—";
      row.append(el("span", `verdict ${inFlight ? "run" : r.verdict === "PASS" ? "pass" : "fail"}`,
        verdict));
      row.append(el("b", null, r.name));
      row.append(el("span", "when", `${fmtDir(r.dir)} · ${r.done}/${r.planned} run(s)`));
      const last = r.runs[r.runs.length - 1];
      if (last) {
        const crit = el("div", "crit");
        if (last.error) {
          crit.append(el("span", "bad", `error: ${last.error}`));
        } else {
          last.criteria.forEach((c, i) => {
            if (i) crit.append(document.createTextNode(" · "));
            crit.append(el("span", c.ok ? null : "bad",
              `${c.label} → ${c.actual == null ? "no data" : c.actual}`));
          });
        }
        row.append(crit);
      }
      row.title = "Show this run's report (data/simtests/" + r.dir + "/report.md)";
      row.addEventListener("click", () => showReport(r.dir));
      resultsEl.append(row);
    }
  }

  async function showReport(dir) {
    if (openReport === dir) {
      openReport = null;
      reportEl.hidden = true;
      return;
    }
    try {
      const out = await getJson(`/api/tests/report?dir=${encodeURIComponent(dir)}`);
      openReport = dir;
      reportEl.textContent = out.markdown;
      reportEl.hidden = false;
    } catch (e) {
      warn(e.message);
    }
  }

  // -- polling --------------------------------------------------------------

  async function poll() {
    timer = null;
    if (viewEl.hidden) return;
    try {
      if (Date.now() - testsAt > TESTS_REFRESH_MS) {
        const out = await getJson("/api/tests");
        tests = out.tests;
        control = !!out.control;
        testsAt = Date.now();
      }
      if (control) {
        const q = logOffset == null ? "" : `?offset=${logOffset}`;
        const wasRunning = isRunning();
        run = await getJson(`/api/tests/run${q}`);
        if (logOffset == null) logEl.textContent = "";
        appendLog(run.log);
        logOffset = run.offset;
        if (wasRunning !== isRunning()) renderTests();
      }
      renderResults((await getJson("/api/tests/results")).results);
      if (!control) warn("Read-only: this dashboard isn't the sim console, so tests can't be " +
        "started here. Results and reports still show.");
      else if (warningEl.textContent.startsWith("Read-only")) warn(null);
    } catch (e) {
      warn(`Tests API unavailable: ${e.message}`);
    }
    renderRun();
    if (!listEl.childElementCount) renderTests();
    schedule(isRunning() ? POLL_RUNNING_MS : POLL_IDLE_MS);
  }

  function schedule(ms) {
    clearTimeout(timer);
    timer = setTimeout(poll, ms);
  }

  // scenarios.js toggles [hidden] on #view-tests; observe that.
  new MutationObserver(() => {
    if (!viewEl.hidden) {
      testsAt = 0;
      listEl.textContent = "";
      schedule(0);
    } else {
      clearTimeout(timer);
      timer = null;
    }
  }).observe(viewEl, { attributes: true, attributeFilter: ["hidden"] });
})();
