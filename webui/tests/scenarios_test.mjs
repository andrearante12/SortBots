#!/usr/bin/env node
/**
 * Headless test for the dashboard's Scenarios tab.
 *
 *     node webui/tests/scenarios_test.mjs [--screenshot DIR] [--port N]
 *
 * Needs NO fixture, no ROS, no Isaac Sim, no GPU and no display — unlike
 * dashboard_test.mjs, which replays recorded ROS data, this tab talks only to
 * webui/serve.py's own control API, so the real thing can be exercised
 * offline. It runs webui/serve.py twice: once with --control (console mode)
 * and once without, because "the console isn't running" is a state the tab is
 * specifically supposed to explain rather than fail in.
 *
 * It NEVER starts a sim: no POST to /api/session/start is made anywhere here.
 * Everything asserted is page state and read-only API responses.
 *
 * Zero npm dependencies, same as dashboard_test.mjs: node's built-in WebSocket
 * and fetch drive the Chrome DevTools Protocol directly.
 *
 * Exit code is 0 only if every assertion passes.
 */
import { spawn } from 'node:child_process';
import { setTimeout as sleep } from 'node:timers/promises';
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const WEBUI_DIR = path.dirname(HERE);
const REPO_ROOT = path.dirname(WEBUI_DIR);

const args = process.argv.slice(2);
const argVal = (name, dflt) => {
  const i = args.indexOf(name);
  return i >= 0 && args[i + 1] ? args[i + 1] : dflt;
};
const CONTROL_PORT = Number(argVal('--port', '8097'));
const READONLY_PORT = CONTROL_PORT + 1;
const REAL_PORT = CONTROL_PORT + 2;
const CDP_PORT = Number(argVal('--cdp-port', '9223'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-scenario-shots')))
  : null;

// app.js loads roslib and opens a websocket at import time. Stub it out: this
// test is about the scenarios tab, and a page throwing in app.js would take
// scenarios.js's script tag down with it and mask the real failure.
const ROSLIB_STUB = `
window.ROSLIB = {
  Ros: function () { this.on = () => {}; this.connect = () => {}; this.close = () => {};
                     this.callOnConnection = () => {}; },
  Topic: function () { this.subscribe = () => {}; this.unsubscribe = () => {};
                       this.publish = () => {}; this.advertise = () => {}; },
  Message: function (d) { Object.assign(this, d); },
  Service: function () { this.callService = () => {}; },
  ServiceRequest: function (d) { Object.assign(this, d); },
  TFClient: function () { this.subscribe = () => {}; this.unsubscribe = () => {}; this.dispose = () => {}; },
};
`;

// ---------------------------------------------------------------- processes

const children = [];
function spawnTracked(cmd, argv, opts = {}) {
  const p = spawn(cmd, argv, { stdio: 'ignore', ...opts });
  children.push(p);
  return p;
}
function cleanup() {
  for (const p of children) { try { p.kill('SIGKILL'); } catch {} }
}
process.on('exit', cleanup);
process.on('SIGINT', () => { cleanup(); process.exit(130); });

async function waitFor(fn, { tries = 60, delay = 250, what = 'service' } = {}) {
  for (let i = 0; i < tries; i++) {
    try { if (await fn()) return true; } catch {}
    await sleep(delay);
  }
  throw new Error(`timed out waiting for ${what}`);
}

// ---------------------------------------------------------------- CDP

function cdp(wsUrl, onEvent) {
  const ws = new WebSocket(wsUrl);
  let id = 0;
  const pending = new Map();
  const ready = new Promise((r) => ws.addEventListener('open', r));
  ws.addEventListener('message', (ev) => {
    const m = JSON.parse(ev.data);
    if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); return; }
    if (m.method) onEvent?.(m, api);
  });
  const api = {
    ready,
    send(method, params = {}) {
      const msgId = ++id;
      return new Promise((res) => {
        pending.set(msgId, res);
        ws.send(JSON.stringify({ id: msgId, method, params }));
      });
    },
    async eval(expression) {
      const r = await api.send('Runtime.evaluate', {
        expression, returnByValue: true, awaitPromise: true,
      });
      const det = r.result?.exceptionDetails;
      if (det) throw new Error(det.exception?.description || det.text);
      return r.result?.result?.value;
    },
    close: () => ws.close(),
  };
  return api;
}

// ---------------------------------------------------------------- assertions

const results = [];
const check = (name, pass, detail = '') => {
  results.push({ name, pass, detail });
  console.log(`${pass ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
};

const VIEWPORTS = [
  { label: '1920x1080', w: 1920, h: 1080, mobile: false },
  { label: '1366x768', w: 1366, h: 768, mobile: false },
  { label: 'phone-390x844', w: 390, h: 844, mobile: true, allowVScroll: true },
];

const SHOW_SCENARIOS = `document.querySelector('#view-mode button[data-view="scenarios"]').click()`;
const SHOW_FLEET = `document.querySelector('#view-mode button[data-view="fleet"]').click()`;
const SHOW_LIVE = `document.querySelector('#view-mode button[data-view="live"]').click()`;
const VISIBLE = `(id) => {
  const el = document.getElementById(id);
  return el ? getComputedStyle(el).display !== 'none' : null;
}`;

// One complete library entry, written by hand rather than by scripts/maps.sh —
// that script needs a running ROS graph, and this file runs offline. The shape
// has to stay in step with scripts/maps_lib.py's schema; tests/maps_test.py is
// what pins the schema itself.
const SEEDED_MAP = 'seeded_warehouse';

function seedMapLibrary() {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-maps-'));
  const entry = path.join(dir, SEEDED_MAP);
  fs.mkdirSync(entry);
  // A real (tiny) sqlite file: maps_lib reconciles the manifest against the
  // disk on every read, so a zero-byte placeholder would come back as
  // db_state "missing" and the option would render disabled.
  fs.writeFileSync(path.join(entry, 'map.db'),
                   Buffer.concat([Buffer.from('SQLite format 3\0'), Buffer.alloc(4080)]));
  fs.writeFileSync(path.join(entry, 'map.json'), JSON.stringify({
    schema: 1,
    name: SEEDED_MAP,
    title: 'Seeded warehouse',
    description: 'Fixture for scenarios_test.mjs.',
    created: '2026-08-01T12:00:00Z',
    updated: '2026-08-01T12:00:00Z',
    scene: 'nvidia',
    robot_ids: ['robot_0'],
    primary_robot_id: 'robot_0',
    source: {},
    grid: null,
    coverage: { free_m2: 812.4, occupied_m2: 96.2, known_m2: 908.6, cells: {} },
    dbs: { robot_0: { file: 'map.db', bytes: 4096, state: 'complete' } },
    db_state: 'complete',
    versions: {},
  }, null, 2));
  return dir;
}

async function main() {
  // 1. two servers: console mode and read-only mode
  // Isolated sessions dir: a stale current.json from a real console run
  // would otherwise leak in and fail the at-rest / never-launched checks.
  const sessionsDir = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-sessions-'));
  // Isolated maps dir for the same reason, and seeded so the `map` picker has
  // something to render: pointing at the repo's real maps/ would make this
  // test's assertions depend on whatever the developer happens to have saved.
  const mapsDir = seedMapLibrary();
  const serverEnv = { ...process.env, SORTBOTS_MAPS_DIR: mapsDir };
  // --platform pinned: left to auto-detect, this test would see the real
  // robot's scenario list when run on the Jetson and the sim's elsewhere.
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--control', '--platform', 'sim',
                           '--port', String(CONTROL_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT,
                 env: { ...serverEnv, SORTBOTS_SESSIONS_DIR: sessionsDir } });
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--platform', 'sim',
                           '--port', String(READONLY_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: serverEnv });
  // API-only: the robot console's view of the same scenario directory.
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--platform', 'real',
                           '--port', String(REAL_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: serverEnv });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${CONTROL_PORT}/`)).ok,
                { what: `webui/serve.py --control on :${CONTROL_PORT}` });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${READONLY_PORT}/`)).ok,
                { what: `webui/serve.py on :${READONLY_PORT}` });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${REAL_PORT}/`)).ok,
                { what: `webui/serve.py --platform real on :${REAL_PORT}` });

  // The API contract the tab depends on, asserted directly so a UI failure
  // below can be told apart from a server failure.
  const api = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/scenarios`)).json();
  const ready = (api.scenarios || []).filter((s) => s.status === 'ready');
  check('control server reports control: true', api.control === true);
  check('scenarios load from configs/scenarios/', ready.length > 0,
        ready.map((s) => s.name).join(', '));
  const invalid = (api.scenarios || []).filter((s) => s.status === 'invalid');
  check('no scenario file fails validation', invalid.length === 0,
        invalid.map((s) => s.error).join('; '));
  // Scenarios that expose the saved-map picker — derived from the files rather
  // than hardcoded, so adding a third library scenario doesn't fail the count.
  const mapScenarios = (api.scenarios || []).filter((s) => (s.overrides || []).includes('map'));
  check('library scenarios expose a map override', mapScenarios.length > 0,
        mapScenarios.map((s) => s.name).join(', '));

  // One dashboard, two consoles: each lists only its own platform's cards.
  check('sim console reports platform: sim', api.platform === 'sim', api.platform);
  check('sim console lists only sim scenarios',
        (api.scenarios || []).every((s) => s.platform === 'sim'),
        (api.scenarios || []).filter((s) => s.platform !== 'sim').map((s) => s.name).join(', '));
  const realApi = await (await fetch(`http://127.0.0.1:${REAL_PORT}/api/scenarios`)).json();
  const realNames = (realApi.scenarios || []).map((s) => s.name);
  check('real console reports platform: real', realApi.platform === 'real', realApi.platform);
  check('real console lists only real scenarios', realNames.length > 0 &&
        (realApi.scenarios || []).every((s) => s.platform === 'real'), realNames.join(', '));

  // The launch form's contract: settings per platform, and every preset must
  // be expressible as form settings (Quick start fills the form from them).
  const opts = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/launch/options`)).json();
  const realOpts = await (await fetch(`http://127.0.0.1:${REAL_PORT}/api/launch/options`)).json();
  check('launch options: sim form has scene and robots',
        opts.fields.includes('scene') && opts.fields.includes('robots'), opts.fields.join(','));
  check('launch options: real form has no sim-only settings',
        !realOpts.fields.includes('scene') && !realOpts.fields.includes('robots') &&
        realOpts.fields.includes('map'), realOpts.fields.join(','));
  check('launch options: every preset fills the form',
        opts.presets.every((p) => p.config), opts.presets.filter((p) => !p.config)
          .map((p) => `${p.name}: ${p.reason}`).join('; '));
  check('launch options: the seeded map is offered', opts.maps.some((m) => m.name === SEEDED_MAP));
  const bad = await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/launch/preview`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ config: { map: '../../etc' } }) });
  check('launch preview rejects a path as a map', bad.status === 400);
  const realRobots = await (await fetch(`http://127.0.0.1:${REAL_PORT}/api/robots`)).json();
  check('real console lists only robots with a `real` spawn',
        JSON.stringify(realRobots.robots) === JSON.stringify(['robot_0']),
        JSON.stringify(realRobots.robots));

  const idle = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/session`)).json();
  check('no session is running at rest', idle.state === 'idle', idle.state);

  const ro = await fetch(`http://127.0.0.1:${READONLY_PORT}/api/session`);
  check('read-only server refuses control endpoints with 503', ro.status === 503,
        `got ${ro.status}`);

  // The saved-map library. /api/maps is readable WITHOUT --control, for the
  // same reason /api/scenarios is: the picker should show what's saved and
  // explain the console is down, rather than render empty.
  for (const [label, port] of [['control', CONTROL_PORT], ['read-only', READONLY_PORT]]) {
    const res = await fetch(`http://127.0.0.1:${port}/api/maps`);
    const body = await res.json().catch(() => ({}));
    const seeded = (body.maps || []).find((m) => m.name === SEEDED_MAP);
    check(`${label} server serves /api/maps without control`, res.status === 200,
          `got ${res.status}`);
    check(`${label} server lists the seeded map`,
          !!seeded && seeded.db_state === 'complete',
          seeded ? seeded.db_state : 'not listed');
    check(`${label} server gives the seeded map an absolute db_path`,
          !!seeded && path.isAbsolute(seeded.db_path || ''), seeded?.db_path);
  }

  // Writing a map is a control operation even though reading the list isn't.
  const roSave = await fetch(`http://127.0.0.1:${READONLY_PORT}/api/map/save`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: 'should_not_happen' }),
  });
  check('read-only server refuses POST /api/map/save with 503', roSave.status === 503,
        `got ${roSave.status}`);

  // A name is a directory name; the server must re-validate it even though the
  // picker can only produce paths it was given.
  const badName = await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/map/save`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: '../etc' }),
  });
  check('control server rejects a traversing map name with 400', badName.status === 400,
        `got ${badName.status}`);

  // 2. headless chrome
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-chrome-'));
  const chromeBin = ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']
    .find((b) => { try { return spawn(b, ['--version']).pid; } catch { return false; } })
    || 'google-chrome';
  spawnTracked(chromeBin, [
    '--headless=new', `--remote-debugging-port=${CDP_PORT}`,
    '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check',
    '--disable-gpu-sandbox', `--user-data-dir=${profile}`, 'about:blank',
  ]);
  let target;
  await waitFor(async () => {
    const list = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json();
    target = list.find((t) => t.type === 'page');
    return !!target;
  }, { what: `headless chrome on :${CDP_PORT}` });

  const pageErrors = [];
  const page = cdp(target.webSocketDebuggerUrl, (m, cdpApi) => {
    if (m.method === 'Runtime.exceptionThrown') {
      const d = m.params.exceptionDetails;
      pageErrors.push(d.exception?.description || d.text);
      return;
    }
    if (m.method !== 'Fetch.requestPaused') return;
    const { requestId, request } = m.params;
    if (request.url.includes('roslib.min.js')) {
      return cdpApi.send('Fetch.fulfillRequest', {
        requestId, responseCode: 200,
        responseHeaders: [{ name: 'Content-Type', value: 'application/javascript' }],
        body: Buffer.from(ROSLIB_STUB).toString('base64'),
      });
    }
    // No web_video_server here; fail fast so the poller just backs off.
    if (request.url.includes('/snapshot')) {
      return cdpApi.send('Fetch.failRequest', { requestId, errorReason: 'Failed' });
    }
    return cdpApi.send('Fetch.continueRequest', { requestId });
  });
  await page.ready;
  await page.send('Page.enable');
  await page.send('Runtime.enable');
  await page.send('Fetch.enable', { patterns: [{ urlPattern: '*' }] });

  if (SCREENSHOT_DIR) fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });

  // 3. console mode, across viewports
  for (const vp of VIEWPORTS) {
    console.log(`\n--- ${vp.label} (console mode) ---`);
    await page.send('Emulation.setDeviceMetricsOverride', {
      width: vp.w, height: vp.h, deviceScaleFactor: 1, mobile: vp.mobile,
    });
    await page.send('Page.navigate', { url: `http://127.0.0.1:${CONTROL_PORT}/` });
    await sleep(1500);

    const initial = await page.eval(`(() => {
      const vis = ${VISIBLE};
      return { stage: vis('stage-panel'), right: vis('right-col'),
               scenarios: vis('view-scenarios') };
    })()`);
    check(`${vp.label}: live view is the default`,
          initial.stage && initial.right && !initial.scenarios);

    await page.eval(SHOW_FLEET);
    await sleep(400);
    const fleet = await page.eval(`(() => {
      const vis = ${VISIBLE};
      return {
        stage: vis('stage-panel'), right: vis('right-col'),
        fleet: vis('view-fleet'), scenarios: vis('view-scenarios'),
        stageMode: vis('stage-mode'),
        hasStatusCol: !!document.getElementById('fleet-status-list'),
        hasIntentCol: !!document.getElementById('fleet-intent-list'),
        hasLog: !!document.getElementById('fleet-log'),
      };
    })()`);
    check(`${vp.label}: fleet tab hides live and scenarios`,
          !fleet.stage && !fleet.right && fleet.fleet && !fleet.scenarios);
    check(`${vp.label}: fleet tab has status/intent/log panels`,
          fleet.hasStatusCol && fleet.hasIntentCol && fleet.hasLog);
    check(`${vp.label}: fleet tab hides live-only chrome`, !fleet.stageMode);

    await page.eval(SHOW_SCENARIOS);
    await sleep(600);

    const shown = await page.eval(`(() => {
      const vis = ${VISIBLE};
      return {
        stage: vis('stage-panel'), right: vis('right-col'),
        scenarios: vis('view-scenarios'),
        stageMode: vis('stage-mode'), badge: vis('slam-badge'),
        form: vis('launch-form'),
        presets: document.querySelectorAll('#lf-preset option').length - 1, // minus "custom"
        startable: !document.getElementById('lf-start').disabled,
        warning: vis('console-warning'),
        state: document.getElementById('session-state').textContent,
        stopDisabled: document.getElementById('session-stop').disabled,
        sceneRow: vis('lf-scene'),
        summary: document.getElementById('lf-summary').textContent,
        // The seeded entry must render as a SELECTABLE option — an entry whose
        // db is missing or an unfetched git-lfs pointer renders disabled, and
        // that difference is the whole point of maps_lib's db_state.
        seededOption: [...document.querySelectorAll('#lf-map option')]
                        .some((o) => !o.disabled && o.value === ${JSON.stringify(SEEDED_MAP)}),
      };
    })()`);

    check(`${vp.label}: switching hides the live view and shows scenarios`,
          !shown.stage && !shown.right && shown.scenarios);
    check(`${vp.label}: live-only header chrome is hidden`,
          !shown.stageMode && !shown.badge);
    check(`${vp.label}: the launch form renders`, shown.form && shown.sceneRow);
    check(`${vp.label}: Quick start lists every preset`, shown.presets === api.scenarios.length,
          `${shown.presets} presets for ${api.scenarios.length} scenarios`);
    check(`${vp.label}: the form previews and is startable`, shown.startable && !!shown.summary,
          shown.summary || 'no summary');
    check(`${vp.label}: the seeded map is offered as a selectable option`,
          shown.seededOption === true);
    check(`${vp.label}: no console warning in control mode`, shown.warning === false);
    check(`${vp.label}: session strip reports no session`, /no session/.test(shown.state),
          shown.state);
    check(`${vp.label}: Stop is disabled with no session`, shown.stopDisabled === true);

    // The camera pollers must idle while the live view is hidden — otherwise
    // both feeds keep hammering web_video_server behind a display:none panel.
    const feeds = await page.eval(`(() => {
      const ids = ['chase-stream', 'camera-stream'];
      return ids.map((id) => document.getElementById(id).offsetParent === null);
    })()`);
    check(`${vp.label}: camera feeds are off-screen (polling gated)`,
          feeds.every(Boolean));

    const scroll = await page.eval(`({
      v: document.documentElement.scrollHeight - document.documentElement.clientHeight,
      h: document.documentElement.scrollWidth - document.documentElement.clientWidth,
    })`);
    check(`${vp.label}: no horizontal scrolling with scenarios open`, scroll.h <= 1,
          `overflow ${scroll.h}px`);
    if (!vp.allowVScroll) {
      check(`${vp.label}: scenarios fit one viewport`, scroll.v <= 1, `overflow ${scroll.v}px`);
    }

    if (SCREENSHOT_DIR) {
      const shot = await page.send('Page.captureScreenshot', { format: 'png' });
      fs.writeFileSync(path.join(SCREENSHOT_DIR, `${vp.label}-scenarios.png`),
                       Buffer.from(shot.result.data, 'base64'));
    }

    // ...and switching back restores the live view intact.
    await page.eval(SHOW_LIVE);
    await sleep(400);
    const back = await page.eval(`(() => {
      const vis = ${VISIBLE};
      // stage-main can be chase, head, or map — chase is absent on robots
      // without a chase product (explore_fleet: chase_cam_robots: 1), and the
      // offline harness has no web_video_server so the probe also falls back.
      const main = document.querySelector('#stage-panel .stage-main');
      return { stage: vis('stage-panel'), scenarios: vis('view-scenarios'),
               stageMode: vis('stage-mode'),
               hasStageMain: !!(main && main.offsetParent !== null) };
    })()`);
    check(`${vp.label}: switching back restores the live view`,
          back.stage && !back.scenarios && back.stageMode && back.hasStageMain);
  }

  // 4. read-only mode: the tab must explain itself, not silently no-op
  console.log('\n--- read-only mode (no console) ---');
  await page.send('Emulation.setDeviceMetricsOverride', {
    width: 1920, height: 1080, deviceScaleFactor: 1, mobile: false,
  });
  await page.send('Page.navigate', { url: `http://127.0.0.1:${READONLY_PORT}/` });
  await sleep(1500);
  await page.eval(SHOW_SCENARIOS);
  await sleep(600);
  const degraded = await page.eval(`(() => {
    const vis = ${VISIBLE};
    return {
      warning: vis('console-warning'),
      text: document.getElementById('console-warning').textContent,
      presets: document.querySelectorAll('#lf-preset option').length - 1,
      startable: !document.getElementById('lf-start').disabled ? 1 : 0,
      state: document.getElementById('session-state').textContent,
    };
  })()`);
  check('read-only: the form still lists the presets', degraded.presets > 0,
        `${degraded.presets} presets`);
  check('read-only: warning names run_console.sh',
        degraded.warning && /run_console\.sh/.test(degraded.text));
  check('read-only: nothing is startable', degraded.startable === 0);
  check('read-only: session strip says the console is not running',
        /console not running/.test(degraded.state), degraded.state);

  // 5. the form drives the server's preview — without starting anything.
  //
  // Quick start must FILL the form (explore_fleet -> 2 robots), and picking a
  // saved map must turn into --resume/--map in the server's own command line.
  // Reads the preview the Start button would launch, never posts the start.
  await page.send('Page.navigate', { url: `http://127.0.0.1:${CONTROL_PORT}/` });
  await sleep(1500);
  await page.eval(SHOW_SCENARIOS);
  await sleep(600);
  const filled = await page.eval(`(async () => {
    const sel = document.getElementById('lf-preset');
    sel.value = 'explore_fleet';
    sel.dispatchEvent(new Event('change'));
    await new Promise((r) => setTimeout(r, 500));
    const robots = document.getElementById('lf-robots').value;
    const map = document.getElementById('lf-map');
    map.value = ${JSON.stringify(SEEDED_MAP)};
    map.dispatchEvent(new Event('change'));
    await new Promise((r) => setTimeout(r, 600));
    return { robots, preset: sel.value,
             command: document.getElementById('lf-command').textContent,
             summary: document.getElementById('lf-summary').textContent,
             modeShown: document.getElementById('lf-mode-row').offsetParent !== null,
             error: document.getElementById('lf-error').textContent };
  })()`);
  check('Quick start fills the form', filled.robots === '2', `robots=${filled.robots}`);
  check('editing the form drops back to custom', filled.preset === '', filled.preset);
  check('a saved map shows the mode choice', filled.modeShown === true);
  check('the preview launches the saved map in extend mode',
        /--resume/.test(filled.command) && new RegExp(`--map \\S*${SEEDED_MAP}/map\\.db`).test(filled.command),
        filled.error || filled.command);

  // 6. nothing we did started a sim
  const finalSession = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/session`)).json();
  check('the test never launched a session', finalSession.state === 'idle',
        finalSession.state);
  // The same invariant extended to the library: reading /api/maps is fine,
  // writing one is not something this file may do.
  const libraryAfter = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/maps`)).json();
  check('the test never wrote a map', (libraryAfter.maps || []).length === 1,
        `${(libraryAfter.maps || []).length} entries`);

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join('; '));

  page.close();
  if (SCREENSHOT_DIR) console.log(`\nscreenshots: ${SCREENSHOT_DIR}`);

  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} checks passed`);
  if (failed.length) {
    console.log('\nFAILED:');
    for (const f of failed) console.log(`  - ${f.name}${f.detail ? `  (${f.detail})` : ''}`);
  }
  process.exit(failed.length ? 1 : 0);
}

main().catch((err) => {
  console.error('\nharness error:', err.message);
  cleanup();
  process.exit(3);
});
