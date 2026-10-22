// SortBots operator view — the Session panel's Tests tab, and the "test running" chip
// over the map. Scored test cases (configs/tests/) launched and watched from here.
//
// Talks ONLY to webui/serve.py over HTTP, the same API as the engineering
// dashboard's tests_view.js:
//   GET  /api/tests            test cases (validated) + whether this server can run them
//   GET  /api/tests/results    the finished run's score, for the result card
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
  let timer = null;
  let wasRunning = false;
  let lastListSig = "";
  const expanded = new Set();

  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  const isRunning = () => !!(run && run.running);
  const onTab = () => card.classList.contains("tab-tests");
  const panelOpen = () => !$("#session").hidden;
  // Short names from the file name ("crossing_yield" -> "Crossing yield"): the
  // spec's titles are sentences, and a new test file gets a name without anyone
  // having to pick one. The full title is in the row's details.
  const titleOf = (name) => { const w = String(name).replace(/_/g, " "); return w.charAt(0).toUpperCase() + w.slice(1); };
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
    $("#tp-list").innerHTML = tests.length ? tests.map((t, i) => {
      const open = expanded.has(t.name) ? " open" : "";
      const label = `<span class="t"><span class="num">${i + 1}</span>${esc(titleOf(t.name))}</span>`;
      if (t.status !== "ready") {
        return `<div class="tp-test invalid${open}" data-name="${esc(t.name)}">
          <div class="tp-row"><span class="chev">&#9654;</span>${label}</div>
          <div class="tp-more"><p class="err">Can't run here: ${esc(t.error || "invalid")}</p><code>configs/tests/${esc(t.name)}.yaml</code></div></div>`;
      }
      const world = t.world ? ` · ${esc(words(t.world))}` : "";
      const checks = t.metrics.flatMap((m) => m.criteria.map((c) => `<li>${crit(m.id, c)}</li>`));
      const reported = t.metrics.filter((m) => !m.criteria.length).map((m) => words(m.id));
      const variant = t.variants.length > 1
        ? `<label>Variant <select data-variant><option value="">all ${t.variants.length}</option>${t.variants.map((v) => `<option value="${esc(v)}">${esc(words(v))}</option>`).join("")}</select></label>` : "";
      return `<div class="tp-test${open}" data-name="${esc(t.name)}">
        <div class="tp-row" title="Show what this test checks">
          <span class="chev">&#9654;</span>${label}
          <button type="button" class="btn small" data-run ${!control || busy ? "disabled" : ""} title="${esc(why)}">Run</button>
        </div>
        <div class="tp-more">
          <b>${esc(t.title)}</b>
          ${t.description ? `<p>${esc(t.description)}</p>` : ""}
          <dl>
            ${checks.length ? `<dt>Passes when</dt><dd><ul>${checks.join("")}</ul></dd>` : ""}
            ${reported.length ? `<dt>Also reports</dt><dd>${esc(reported.join(", "))}</dd>` : ""}
            <dt>Runs</dt><dd>${esc(words(t.scenario))}${world} · ${plural(t.robots.length, "robot")} · up to ${simTime(t.sim_s)} sim${t.variants.length > 1 ? ` · ${t.variants.length} variants` : ""}</dd>
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
      $("#result-card").hidden = true; // the last score is gone once the next run starts
      warn(null);
      $("#tp-log").textContent = ""; logOffset = 0;
      toast(`Starting ${titleOf(name)}…`);
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
    // Only while one runs: a finished run's verdict is the toast, and a stale
    // "last run ..." line read as something still wrong.
    box.hidden = !live;
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

  // -- result card: a finished run's score, once ------------------------------------------
  // simtest.py writes every run to data/simtests/; the card reads the newest one for
  // this test when it finishes and shows it until dismissed. Nothing is kept here:
  // a reload, or the next run, starts clean.

  const fmt = (v) => (v == null ? "no data" : typeof v === "number" ? String(Math.round(v * 1000) / 1000) : String(v));
  // "clearance.contacts == 0" -> ["Clearance contacts", "= 0"]
  function splitLabel(label) {
    const m = /^([\w.]+)\s*(==|>=|<=|>|<|!=)\s*(.+)$/.exec(label);
    if (!m) return [label, ""];
    const name = m[1].replace(/[._]/g, " ");
    const op = { "==": "=", ">=": "≥", "<=": "≤", "!=": "≠" }[m[2]] || m[2];
    return [name.charAt(0).toUpperCase() + name.slice(1), `${op} ${m[3]}`];
  }
  async function showResult(name, exit) {
    let r = null;
    try { r = ((await api("/api/tests/results")).results || []).find((x) => x.name === name) || null; } catch (e) { /* card says so */ }
    const runs = r ? r.runs : [];
    const passed = runs.filter((x) => x.ok).length;
    const verdict = exit === 130 ? "STOPPED" : r && r.verdict ? r.verdict : exit === 0 ? "PASS" : "FAIL";
    $("#rc-v").textContent = verdict;
    $("#rc-v").className = `rc-v ${verdict === "PASS" ? "pass" : verdict === "STOPPED" ? "" : "fail"}`;
    $("#rc-name").textContent = titleOf(name);
    const simS = runs.length ? runs[runs.length - 1].sim_s : null;
    $("#rc-meta").textContent = runs.length > 1 ? `${passed} of ${runs.length} runs passed`
      : simS != null ? `${Math.round(simS)} s of sim time${exit === 130 ? ", stopped early" : ""}` : "";
    // One line per check. Over several runs: the share of runs it held in, last value.
    const byLabel = new Map();
    for (const x of runs) for (const c of x.criteria || []) {
      const e = byLabel.get(c.label) || { held: 0, n: 0, actual: null };
      e.n++; if (c.ok) e.held++; e.actual = c.actual;
      byLabel.set(c.label, e);
    }
    $("#rc-checks").innerHTML = [...byLabel].map(([label, e]) => {
      const [what, want] = splitLabel(label), ok = e.held === e.n;
      const got = e.n > 1 ? `${e.held}/${e.n} runs` : `got ${fmt(e.actual)}`;
      return `<li class="${ok ? "" : "bad"}"><span class="${ok ? "ok" : "no"}">${ok ? "✓" : "✗"}</span><span>${esc(what)} ${esc(want)}</span><span class="got">${esc(got)}</span></li>`;
    }).join("");
    const errs = runs.map((x) => x.error).filter(Boolean);
    const err = errs.length ? errs[errs.length - 1] : !r ? "No score was written for this run" : !byLabel.size && exit !== 0 ? `Run ${EXIT_LABEL[exit] || `exited ${exit}`}` : "";
    $("#rc-err").textContent = err; $("#rc-err").hidden = !err;
    $("#result-card").hidden = false;
  }
  $("#rc-x").addEventListener("click", () => { $("#result-card").hidden = true; });

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
      if (wasRunning && !run.running && run.name) showResult(run.name, run.exit);
      wasRunning = !!run.running;
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
