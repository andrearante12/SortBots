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
//
// While a test runs, the Launch tab's own session Stop is swapped for Stop test.
// The session's Stop tears the sim down under the probe, which then dies unscored
// and the run reads "infrastructure error" (seen live 2026-10-22, crossing_yield);
// Stop test interrupts simtest.py instead, and what it saw is still scored.

(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const card = $("#session .card");
  if (!card || !$("#tests-pane")) return;

  const POLL_RUNNING_MS = 1000;
  const POLL_IDLE_MS = 4000; // panel open on this tab, nothing running
  const POLL_BACKGROUND_MS = 3000; // panel closed: only the chip needs the state
  const TESTS_REFRESH_MS = 15000;
  const EXIT_LABEL = { 0: "passed", 1: "failed", 2: "invalid", 3: "infrastructure error", 130: "stopped" };

  let tests = [];
  let control = false;
  let testsAt = 0;
  let run = null; // last GET /api/tests/run
  let logOffset = null;
  let openReport = null;
  let timer = null;
  let wasRunning = false;
  let lastListSig = "";
  const expanded = new Set();

  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  const isRunning = () => !!(run && run.running);
  const onTab = () => card.classList.contains("tab-tests");
  const panelOpen = () => !$("#session").hidden;
  const titleOf = (name) => (tests.find((t) => t.name === name) || {}).title || name;
  const plural = (n, w) => `${n} ${w}${n === 1 ? "" : "s"}`;
  const words = (s) => String(s || "").replace(/_/g, " ");
  // "2.5 min", "90 s": sim seconds, as the spec caps them.
  const simTime = (s) => (s >= 90 ? `${Math.round(s / 30) / 2} min` : `${Math.round(s)} s`);
  // "clearance: contacts == 0" reads better as "clearance · contacts = 0".
  const crit = (m, c) => `<b>${esc(words(m))}</b> ${esc(c.replace(/==/g, "=").replace(/>=/g, "≥").replace(/<=/g, "≤").replace(/_/g, " "))}`;

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
    const busy = isRunning();
    const sig = JSON.stringify([tests, control, busy, [...expanded]]);
    if (sig === lastListSig) return; // don't rebuild under the operator's cursor
    lastListSig = sig;
    const ready = tests.filter((t) => t.status === "ready").length;
    $("#tp-count").textContent = tests.length ? `· ${ready} ready` : "";
    const why = !control ? "This server can't launch the sim (it isn't the sim console)" : busy ? "A test is already running" : "Launch the sim, run this test and score it";
    $("#tp-list").innerHTML = tests.length ? tests.map((t) => {
      const open = expanded.has(t.name) ? " open" : "";
      if (t.status !== "ready") {
        return `<div class="tp-test invalid${open}" data-name="${esc(t.name)}">
          <div class="tp-row"><span class="chev">&#9654;</span><span class="t">${esc(t.title || t.name)}</span>
            <span class="m">Can't run here: ${esc(t.error || "invalid")}</span></div>
          <div class="tp-more"><code>configs/tests/${esc(t.name)}.yaml</code></div></div>`;
      }
      const world = t.world ? ` · ${esc(words(t.world))}` : "";
      const checks = t.metrics.flatMap((m) => m.criteria.map((c) => `<li>${crit(m.id, c)}</li>`));
      const reported = t.metrics.filter((m) => !m.criteria.length).map((m) => words(m.id));
      const variant = t.variants.length > 1
        ? `<label>Variant <select data-variant><option value="">all ${t.variants.length}</option>${t.variants.map((v) => `<option value="${esc(v)}">${esc(words(v))}</option>`).join("")}</select></label>` : "";
      return `<div class="tp-test${open}" data-name="${esc(t.name)}">
        <div class="tp-row" title="Show what this test checks">
          <span class="chev">&#9654;</span><span class="t">${esc(t.title)}</span>
          <span class="m">${plural(t.robots.length, "robot")}${world} · up to ${simTime(t.sim_s)} sim${t.variants.length > 1 ? ` · ${t.variants.length} variants` : ""}</span>
          <button type="button" class="btn small" data-run ${!control || busy ? "disabled" : ""} title="${esc(why)}">Run</button>
        </div>
        <div class="tp-more">
          ${t.description ? `<p>${esc(t.description)}</p>` : ""}
          <dl>
            ${checks.length ? `<dt>Passes when</dt><dd><ul>${checks.join("")}</ul></dd>` : ""}
            ${reported.length ? `<dt>Also reports</dt><dd>${esc(reported.join(", "))}</dd>` : ""}
            <dt>Runs</dt><dd>${esc(words(t.scenario))}${world}</dd>
          </dl>
          <div class="opts">${variant}<label>Repeats <input type="number" min="1" max="50" value="${t.repeats}" data-repeats></label>
            <code>configs/tests/${esc(t.name)}.yaml</code></div>
        </div>
      </div>`;
    }).join("") : `<div class="tp-empty">No test cases in configs/tests/.</div>`;
  }

  $("#tp-list").addEventListener("click", async (e) => {
    const item = e.target.closest(".tp-test");
    if (!item) return;
    const name = item.dataset.name, btn = e.target.closest("[data-run]");
    if (!btn) {
      if (!e.target.closest(".tp-row")) return; // a click inside the expanded options
      if (expanded.has(name)) expanded.delete(name); else expanded.add(name);
      item.classList.toggle("open", expanded.has(name));
      lastListSig = JSON.stringify([tests, control, isRunning(), [...expanded]]);
      return;
    }
    // Options only exist once a row has been expanded; collapsed, the spec's defaults.
    const t = tests.find((x) => x.name === name);
    const reps = item.querySelector("[data-repeats]"), v = item.querySelector("[data-variant]");
    const payload = { name, repeats: Number(reps && reps.value) || t.repeats };
    if (v && v.value) payload.variant = v.value;
    btn.disabled = true;
    try {
      run = await api("/api/tests/run", payload);
      warn(null);
      $("#tp-log").textContent = ""; logOffset = 0;
      toast(`Starting ${t.title}…`);
      closePanel(); // the map is where a test is watched
    } catch (err) {
      warn(`Couldn't start ${t.title}: ${err.message}`);
      btn.disabled = false;
    }
    renderRun(); schedule(POLL_RUNNING_MS);
  });

  // -- the running test: panel line + map chip -----------------------------------------

  function renderRun() {
    const box = $("#tp-run"), chip = $("#test-chip");
    const has = !!(run && run.name), live = isRunning();
    box.hidden = !has;
    chip.hidden = !live;
    card.classList.toggle("testing", live);
    if (!has) return;
    const t = Math.round(run.elapsed_s || 0);
    $("#tp-run-name").textContent = titleOf(run.name);
    box.classList.toggle("done", !live);
    box.classList.toggle("pass", !live && run.exit === 0);
    box.classList.toggle("fail", !live && run.exit != null && run.exit !== 0 && run.exit !== 130);
    $("#tp-run-state").textContent = live ? `running · ${clock(t)}` : `last run ${EXIT_LABEL[run.exit] || `exited ${run.exit}`}`;
    $("#tp-watch").hidden = !live;
    $("#tp-cancel").hidden = !live;
    $("#tc-name").textContent = titleOf(run.name);
    $("#tc-time").textContent = clock(t);
  }
  function appendLog(text) {
    if (!text) return;
    const el = $("#tp-log"), atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
    el.textContent += text;
    if (atBottom) el.scrollTop = el.scrollHeight;
  }
  async function cancelRun() {
    if (!confirm("Stop the running test? The sim is torn down; what it saw so far is still scored.")) return;
    try { run = await api("/api/tests/cancel", {}); } catch (e) { warn(`Stop failed: ${e.message}`); }
    renderRun(); schedule(0);
  }
  $("#tp-cancel").addEventListener("click", cancelRun);
  $("#tc-cancel").addEventListener("click", cancelRun);
  $("#sp-test-stop").addEventListener("click", cancelRun);
  $("#tp-watch").addEventListener("click", closePanel);
  $("#tc-name").addEventListener("click", openTests);
  $("#tp-log-toggle").addEventListener("click", (e) => {
    const log = $("#tp-log");
    log.hidden = !log.hidden;
    e.currentTarget.setAttribute("aria-expanded", String(!log.hidden));
    e.currentTarget.textContent = log.hidden ? "Log" : "Hide log";
    if (!log.hidden) log.scrollTop = log.scrollHeight;
  });

  // -- results --------------------------------------------------------------------------

  function when(dir) {
    const m = /^(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})/.exec(dir);
    if (!m) return dir;
    const d = new Date(Date.UTC(+m[1], m[2] - 1, +m[3], +m[4], +m[5]));
    const today = new Date().toDateString() === d.toDateString();
    return d.toLocaleString([], today ? { hour: "numeric", minute: "2-digit" } : { month: "short", day: "numeric", hour: "numeric", minute: "2-digit" });
  }
  function renderResults(results) {
    $("#tp-results").innerHTML = results.length ? results.slice(0, 10).map((r) => {
      const inFlight = r.state === "running" && !r.complete;
      const v = inFlight ? ["run", "RUN"] : r.verdict === "PASS" ? ["pass", "PASS"] : r.verdict ? ["fail", r.verdict] : ["", "—"];
      const last = r.runs[r.runs.length - 1];
      const failed = !last ? [] : last.error ? [`<span class="bad">${esc(last.error)}</span>`]
        : last.criteria.filter((c) => !c.ok).map((c) => `<span class="bad">${esc(c.label)}: got ${c.actual == null ? "no data" : esc(c.actual)}</span>`);
      const detail = failed.length ? failed.join(" · ") : last && last.criteria.length ? `all ${last.criteria.length} checks held` : "";
      return `<div class="tp-res" data-dir="${esc(r.dir)}"><div class="tp-res-row" title="Show the full report">
        <span class="v ${v[0]}">${v[1]}</span><span class="n">${esc(titleOf(r.name))}</span>
        <span class="w">${esc(when(r.dir))}${r.planned > 1 ? ` · ${r.done}/${r.planned}` : ""}</span>
        ${detail ? `<div class="c">${detail}</div>` : ""}</div>
        ${openReport === r.dir ? `<pre id="tp-report"></pre>` : ""}</div>`;
    }).join("") : `<div class="tp-empty">No results yet.</div>`;
    if (openReport && reportText != null) { const rep = $("#tp-report"); if (rep) rep.textContent = reportText; }
  }
  let reportText = null, lastResults = [];
  $("#tp-results").addEventListener("click", async (e) => {
    const row = e.target.closest(".tp-res-row");
    if (!row) return;
    const dir = row.parentElement.dataset.dir;
    if (openReport === dir) { openReport = null; reportText = null; renderResults(lastResults); return; }
    try {
      reportText = (await api(`/api/tests/report?dir=${encodeURIComponent(dir)}`)).markdown;
      openReport = dir; renderResults(lastResults);
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
      // a read-only server answers 503 and the chip just never shows.
      const q = logOffset == null ? "" : `?offset=${logOffset}`;
      run = await api(`/api/tests/run${q}`);
      if (logOffset == null) $("#tp-log").textContent = "";
      appendLog(run.log); logOffset = run.offset;
      if (wasRunning && !run.running && run.name) {
        toast(`${titleOf(run.name)}: ${EXIT_LABEL[run.exit] || `exited ${run.exit}`}`);
        testsAt = 0; // results changed
      }
      wasRunning = !!run.running;
      if (visible) { lastResults = (await api("/api/tests/results")).results || []; renderResults(lastResults); }
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
