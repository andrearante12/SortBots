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

async function main() {
  // 1. two servers: console mode and read-only mode
  // Isolated sessions dir: a stale current.json from a real console run
  // would otherwise leak in and fail the at-rest / never-launched checks.
  const sessionsDir = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-sessions-'));
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--control',
                           '--port', String(CONTROL_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: { ...process.env, SORTBOTS_SESSIONS_DIR: sessionsDir } });
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'),
                           '--port', String(READONLY_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${CONTROL_PORT}/`)).ok,
                { what: `webui/serve.py --control on :${CONTROL_PORT}` });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${READONLY_PORT}/`)).ok,
                { what: `webui/serve.py on :${READONLY_PORT}` });

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

  const idle = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/session`)).json();
  check('no session is running at rest', idle.state === 'idle', idle.state);

  const ro = await fetch(`http://127.0.0.1:${READONLY_PORT}/api/session`);
  check('read-only server refuses control endpoints with 503', ro.status === 503,
        `got ${ro.status}`);

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
        cards: document.querySelectorAll('.scenario-card').length,
        startable: [...document.querySelectorAll('.scenario-card button')]
                     .filter((b) => !b.disabled).length,
        warning: vis('console-warning'),
        state: document.getElementById('session-state').textContent,
        stopDisabled: document.getElementById('session-stop').disabled,
        overrides: document.querySelectorAll('.scenario-card [data-key]').length,
      };
    })()`);

    check(`${vp.label}: switching hides the live view and shows scenarios`,
          !shown.stage && !shown.right && shown.scenarios);
    check(`${vp.label}: live-only header chrome is hidden`,
          !shown.stageMode && !shown.badge);
    check(`${vp.label}: a card renders per scenario`, shown.cards === api.scenarios.length,
          `${shown.cards} cards for ${api.scenarios.length} scenarios`);
    check(`${vp.label}: ready scenarios are startable`, shown.startable === ready.length,
          `${shown.startable} enabled`);
    check(`${vp.label}: override controls render`, shown.overrides > 0,
          `${shown.overrides} inputs`);
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
      cards: document.querySelectorAll('.scenario-card').length,
      startable: [...document.querySelectorAll('.scenario-card button')]
                   .filter((b) => !b.disabled).length,
      state: document.getElementById('session-state').textContent,
    };
  })()`);
  check('read-only: the tab still lists the scenarios', degraded.cards > 0,
        `${degraded.cards} cards`);
  check('read-only: warning names run_console.sh',
        degraded.warning && /run_console\.sh/.test(degraded.text));
  check('read-only: nothing is startable', degraded.startable === 0);
  check('read-only: session strip says the console is not running',
        /console not running/.test(degraded.state), degraded.state);

  // 5. nothing we did started a sim
  const finalSession = await (await fetch(`http://127.0.0.1:${CONTROL_PORT}/api/session`)).json();
  check('the test never launched a session', finalSession.state === 'idle',
        finalSession.state);

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
