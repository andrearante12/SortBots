// SortBots operator view — the Session panel's Tests tab, and the "test running" chip
// over the map. Scored test cases (configs/tests/) launched and watched from here.
//
// Talks ONLY to webui/serve.py over HTTP, the same API as the engineering
// dashboard's tests_view.js:
//   GET  /api/tests            test cases (validated) + whether this server can run them
//   GET  /api/tests/results    recent result dirs (data/simtests/), newest first
//   GET  /api/tests/report     ?dir=NAME -> that run's report.md
//   GET  /api/tests/run        ?offset= runner state + incremental log
//   POST /api/tests/run        {name, repeats?, variant?}
//   POST /api/tests/cancel
//
// A test drives the sim through ordinary sessions (scripts/simtest.py ->
// sim_ctl.sh -> /api/session/start), so watching one is just the operator map:
// robots, their plans, the walkers (operator.js's world overlay) and the obstacles
// they see. Run closes the panel for exactly that, and the chip keeps the test's
// name, clock and a way out on screen while it runs.

(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const card = $("#session .card");
  if (!card || !$("#tests-pane")) return;

  const POLL_RUNNING_MS = 1000;
  const POLL_IDLE_MS = 4000; // panel open on this tab, nothing running
  const POLL_BACKGROUND_MS = 3000; // panel closed: only the chip needs the state
  const TESTS_REFRESH_MS = 15000;
  const EXIT_LABEL = { 0: "passed", 1: "failed", 2: "invalid", 3: "infrastructure error", 130: "cancelled" };

  let tests = [];
  let control = false;
  let testsAt = 0;
  let run = null; // last GET /api/tests/run
  let logOffset = null;
  let openReport = null;
  let timer = null;
  let wasRunning = false;
  let lastListSig = "";

  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  const isRunning = () => !!(run && run.running);
  const onTab = () => card.classList.contains("tab-tests");
  const panelOpen = () => !$("#session").hidden;

  async function api(url, payload) {
    const res = await fetch(url, payload === undefined ? { cache: "no-store" } : {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload),
    });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(body.error || res.statusText);
    return body;
  }
  function warn(text) { $("#tp-warn").hidden = !text; $("#tp-warn").textContent = text || ""; }
  function toast(text) {
    const t = $("#toast");
    if (!t) return;
    t.textContent = text; t.classList.add("on");
    clearTimeout(toast.timer); toast.timer = setTimeout(() => t.classList.remove("on"), 4000);
  }

  // -- tabs -------------------------------------------------------------------------

  function setTab(tab) {
    card.classList.toggle("tab-tests", tab === "tests");
    for (const b of document.querySelectorAll("#sp-tabs button")) b.classList.toggle("active", b.dataset.tab === tab);
    if (tab === "tests") { testsAt = 0; schedule(0); }
  }
  for (const b of document.querySelectorAll("#sp-tabs button")) b.addEventListener("click", () => setTab(b.dataset.tab));
  function openTests() {
    $("#session-chip")?.click(); // operator.js opens the panel (and syncs scenarios.js)
    if ($("#session").hidden) $("#session").hidden = false;
    setTab("tests");
  }
  function closePanel() { $("#sp-x").click(); }

  // -- test list --------------------------------------------------------------------

  function renderList() {
    const sig = JSON.stringify([tests, control, isRunning()]);
    if (sig === lastListSig) return; // don't rebuild under the operator's cursor
    lastListSig = sig;
    const busy = isRunning();
    $("#tp-list").innerHTML = tests.length ? tests.map((t) => {
      if (t.status !== "ready") {
        return `<div class="tp-test invalid"><div class="t">${esc(t.title || t.name)}<code>${esc(t.name)}</code></div>
          <div class="err">${esc(t.error || "invalid")}</div></div>`;
      }
      const crit = t.metrics.flatMap((m) => m.criteria.length ? m.criteria.map((c) => `${m.id}: ${c}`) : []);
      const variants = t.variants.length > 1
        ? `<label>variant<select data-variant><option value="">all ${t.variants.length}</option>${t.variants.map((v) => `<option>${esc(v)}</option>`).join("")}</select></label>` : "";
      const why = !control ? "This server can't launch the sim (it isn't the sim console)" : busy ? "A test is already running" : "Launch the sim, run this test and score it";
      return `<div class="tp-test" data-name="${esc(t.name)}">
        <div class="t">${esc(t.title)}<code>${esc(t.name)}</code></div>
        ${t.description ? `<div class="d" title="${esc(t.description)}">${esc(t.description)}</div>` : ""}
        <div class="m">${esc(t.scenario)}${t.world ? ` · ${esc(t.world)}` : ""} · ${t.robots.length} robot${t.robots.length === 1 ? "" : "s"} · up to ${Math.round(t.sim_s)} s sim · ${t.variants.length} variant${t.variants.length === 1 ? "" : "s"}</div>
        ${crit.length ? `<div class="crit">${crit.map((c) => `<span>${esc(c)}</span>`).join("")}</div>` : ""}
        <div class="go">${variants}<label>repeats<input type="number" min="1" max="50" value="${t.repeats}" data-repeats></label>
          <button type="button" class="btn primary small" data-run ${!control || busy ? "disabled" : ""} title="${esc(why)}">Run</button></div>
      </div>`;
    }).join("") : `<div class="muted">No test cases in configs/tests/.</div>`;
  }

  $("#tp-list").addEventListener("click", async (e) => {
    const btn = e.target.closest("[data-run]");
    if (!btn) return;
    const row = btn.closest(".tp-test"), name = row.dataset.name;
    const payload = { name, repeats: Number(row.querySelector("[data-repeats]").value) || 1 };
    const v = row.querySelector("[data-variant]");
    if (v && v.value) payload.variant = v.value;
    btn.disabled = true;
    try {
      run = await api("/api/tests/run", payload);
      warn(null);
      $("#tp-log").textContent = ""; logOffset = 0;
      toast(`Starting ${name}…`);
      closePanel(); // the map is where a test is watched
    } catch (err) {
      warn(`Couldn't start ${name}: ${err.message}`);
      btn.disabled = false;
    }
    renderRun(); schedule(POLL_RUNNING_MS);
  });

  // -- the running test: panel block + map chip ----------------------------------------

  function renderRun() {
    const box = $("#tp-run"), chip = $("#test-chip");
    const has = !!(run && run.name);
    box.hidden = !has;
    chip.hidden = !isRunning();
    if (!has) return;
    const t = Math.round(run.elapsed_s || 0);
    $("#tp-run-name").textContent = run.name;
    box.classList.toggle("done", !run.running);
    box.classList.toggle("pass", !run.running && run.exit === 0);
    box.classList.toggle("fail", !run.running && run.exit !== 0 && run.exit != null);
    $("#tp-run-state").textContent = run.running ? `running · ${clock(t)}` : `last run ${EXIT_LABEL[run.exit] || `exited ${run.exit}`}`;
    $("#tp-watch").hidden = !run.running;
    $("#tp-cancel").hidden = !run.running;
    $("#tc-name").textContent = run.name;
    $("#tc-time").textContent = clock(t);
  }
  function appendLog(text) {
    if (!text) return;
    const el = $("#tp-log"), atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
    el.textContent += text;
    if (atBottom) el.scrollTop = el.scrollHeight;
  }
  async function cancelRun() {
    if (!confirm("Stop the running test? The sim is torn down; runs that already finished are still scored.")) return;
    try { run = await api("/api/tests/cancel", {}); } catch (e) { warn(`Cancel failed: ${e.message}`); }
    renderRun(); schedule(0);
  }
  $("#tp-cancel").addEventListener("click", cancelRun);
  $("#tc-cancel").addEventListener("click", cancelRun);
  $("#tp-watch").addEventListener("click", closePanel);
  $("#tc-name").addEventListener("click", openTests);

  // -- results --------------------------------------------------------------------------

  function when(dir) {
    const m = /^(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})/.exec(dir);
    if (!m) return dir;
    const d = new Date(Date.UTC(+m[1], m[2] - 1, +m[3], +m[4], +m[5]));
    return d.toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
  }
  function renderResults(results) {
    $("#tp-results").innerHTML = results.length ? results.slice(0, 12).map((r) => {
      const inFlight = r.state === "running" && !r.complete;
      const v = inFlight ? ["run", "RUNNING"] : r.verdict === "PASS" ? ["pass", "PASS"] : r.verdict ? ["fail", r.verdict] : ["", "—"];
      const last = r.runs[r.runs.length - 1];
      const crit = !last ? "" : last.error ? `<span class="bad">error: ${esc(last.error)}</span>`
        : last.criteria.map((c) => `<span${c.ok ? "" : ' class="bad"'}>${esc(c.label)} → ${c.actual == null ? "no data" : esc(c.actual)}</span>`).join(" · ");
      return `<div class="tp-res" data-dir="${esc(r.dir)}" title="Show the full report">
        <span class="v ${v[0]}">${v[1]}</span><b>${esc(r.name)}</b><span class="w">${esc(when(r.dir))} · ${r.done}/${r.planned} run${r.planned === 1 ? "" : "s"}</span>
        ${crit ? `<div class="c">${crit}</div>` : ""}</div>`;
    }).join("") : `<div class="muted" style="font-size:.84rem">No results yet.</div>`;
  }
  $("#tp-results").addEventListener("click", async (e) => {
    const row = e.target.closest(".tp-res");
    if (!row) return;
    const rep = $("#tp-report");
    if (openReport === row.dataset.dir) { openReport = null; rep.hidden = true; return; }
    try {
      const out = await api(`/api/tests/report?dir=${encodeURIComponent(row.dataset.dir)}`);
      openReport = row.dataset.dir; rep.textContent = out.markdown; rep.hidden = false;
    } catch (err) { warn(err.message); }
  });

  // -- polling ----------------------------------------------------------------------------

  async function poll() {
    timer = null;
    const visible = panelOpen() && onTab();
    try {
      if (visible && Date.now() - testsAt > TESTS_REFRESH_MS) {
        const out = await api("/api/tests");
        tests = out.tests || []; control = !!out.control; testsAt = Date.now();
        warn(control ? null : "Read-only: this server isn't the sim console, so tests can't be started here. Results still show.");
      }
      // The run state feeds the chip too, so it's polled with the panel closed;
      // a read-only server answers 403/503 and the chip just never shows.
      const q = logOffset == null ? "" : `?offset=${logOffset}`;
      run = await api(`/api/tests/run${q}`);
      if (logOffset == null) $("#tp-log").textContent = "";
      appendLog(run.log); logOffset = run.offset;
      if (wasRunning && !run.running && run.name) {
        toast(`Test ${run.name} ${EXIT_LABEL[run.exit] || `exited ${run.exit}`}`);
        testsAt = 0; // results changed
      }
      wasRunning = !!run.running;
      if (visible) renderResults((await api("/api/tests/results")).results || []);
    } catch (e) {
      if (visible) warn(`Tests unavailable: ${e.message}`);
    }
    renderRun();
    if (visible) renderList();
    schedule(isRunning() ? POLL_RUNNING_MS : visible ? POLL_IDLE_MS : POLL_BACKGROUND_MS);
  }
  function schedule(ms) { clearTimeout(timer); timer = setTimeout(poll, ms); }

  // Opening the panel on this tab refreshes at once rather than on the next idle tick.
  new MutationObserver(() => { if (panelOpen() && onTab()) { testsAt = 0; schedule(0); } })
    .observe($("#session"), { attributes: true, attributeFilter: ["hidden"] });

  schedule(0);
  window.sortbotsTests = { setTab, openTests, get run() { return run; } };
})();
