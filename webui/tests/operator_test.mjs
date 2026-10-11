#!/usr/bin/env node
/**
 * Headless test for the operator view (webui/operator.html + operator.js).
 *
 *     node webui/tests/operator_test.mjs [--screenshot DIR] [--port N]
 *
 * No fixture, no ROS, no Isaac Sim, no GPU, no display. ROSLIB is replaced by a
 * stub that records everything the page publishes and lets this file inject
 * topic messages (/map, /tf, task_status ...), so the whole fleet UI is driven
 * offline. serve.py runs for real (--control, isolated sessions dir) because the
 * session panel is scenarios.js talking to its API; /api/nav_waypoints and
 * /api/session are shimmed in-page so the test never writes data/ or needs a sim.
 *
 * It NEVER starts a sim. Zero npm dependencies, same as the other tests: node's
 * WebSocket and fetch drive the Chrome DevTools Protocol directly.
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
const argVal = (name, dflt) => { const i = args.indexOf(name); return i >= 0 && args[i + 1] ? args[i + 1] : dflt; };
const PORT = Number(argVal('--port', '8201'));
const CDP_PORT = Number(argVal('--cdp-port', '9231'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-operator-shots'))) : null;

// Records publishes / ops / service calls; __emit(topic, msg) feeds a subscriber.
const ROSLIB_STUB = `
window.__topics = {}; window.__published = []; window.__ops = []; window.__services = [];
window.__emit = (name, msg) => (window.__topics[name] || []).forEach((cb) => cb(msg));
window.ROSLIB = {
  Ros: function () { this.on = () => {}; this.connect = () => {}; this.close = () => {};
                     this.callOnConnection = (op) => window.__ops.push(op); },
  Topic: function (o) { this.name = o.name;
    this.subscribe = (cb) => { (window.__topics[o.name] = window.__topics[o.name] || []).push(cb); };
    this.unsubscribe = () => {}; this.advertise = () => {};
    this.publish = (m) => window.__published.push({ topic: o.name, msg: m }); },
  Message: function (d) { Object.assign(this, d); },
  Service: function (o) { this.callService = (req) => window.__services.push({ name: o.name, req }); },
  ServiceRequest: function (d) { Object.assign(this, d); },
};
// Shims: nav waypoints live in memory (the real endpoint writes data/), the session
// can be faked per scenario, everything else goes to the real serve.py.
window.__navWps = [];
window.__fakeSession = null;
const realFetch = window.fetch.bind(window);
window.fetch = (url, opts) => {
  const u = String(url);
  const json = (b) => Promise.resolve(new Response(JSON.stringify(b), { status: 200, headers: { 'Content-Type': 'application/json' } }));
  if (u.startsWith('/api/nav_waypoints')) {
    if (opts && opts.method === 'POST') window.__navWps = JSON.parse(opts.body).waypoints;
    return json({ waypoints: window.__navWps });
  }
  if (u.startsWith('/api/session') && !u.startsWith('/api/session/') && window.__fakeSession) return json(window.__fakeSession);
  return realFetch(url, opts);
};
window.confirm = () => true;
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


const grid = (w, h, res, ox, oy) => {
  const data = new Array(w * h).fill(0);
  for (let i = 0; i < w; i++) { data[i] = 100; data[(h - 1) * w + i] = 100; }
  for (let j = 0; j < h; j++) { data[j * w] = 100; data[j * w + w - 1] = 100; }
  for (let i = 0; i < 12; i++) data[i] = -1; // some unknown
  return { info: { width: w, height: h, resolution: res, origin: { position: { x: ox, y: oy, z: 0 } } }, data };
};
const tf = (parent, child, x, y, yaw = 0) => ({ transforms: [{
  header: { frame_id: parent, stamp: { sec: 1, nanosec: 0 } }, child_frame_id: child,
  transform: { translation: { x, y, z: 0 }, rotation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } } }] });

async function main() {
  const sessionsDir = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-sessions-'));
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--control', '--platform', 'sim',
                           '--port', String(PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: { ...process.env, SORTBOTS_SESSIONS_DIR: sessionsDir } });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${PORT}/operator.html`)).ok, { what: 'serve.py' });
  const roster = (await (await fetch(`http://127.0.0.1:${PORT}/api/robots`)).json()).robots || [];
  check('roster has at least two robots', roster.length >= 2, roster.join(','));
  const [r0, r1] = roster;
  const stations = await (await fetch(`http://127.0.0.1:${PORT}/api/waypoints`)).json();
  const pick = Object.entries(stations).find(([, s]) => s.kind === 'pickup')?.[0];
  const drop = Object.entries(stations).find(([, s]) => s.kind === 'dropoff')?.[0];
  check('task stations are configured', !!pick && !!drop, `${pick} -> ${drop}`);

  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-chrome-'));
  const chromeBin = ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']
    .find((b) => { try { return spawn(b, ['--version']).pid; } catch { return false; } }) || 'google-chrome';
  spawnTracked(chromeBin, ['--headless=new', `--remote-debugging-port=${CDP_PORT}`, '--enable-unsafe-swiftshader',
    '--no-first-run', '--no-default-browser-check', '--disable-gpu-sandbox', `--user-data-dir=${profile}`, 'about:blank']);
  let target;
  await waitFor(async () => {
    const list = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json();
    target = list.find((t) => t.type === 'page'); return !!target;
  }, { what: 'headless chrome' });

  const pageErrors = [];
  const page = cdp(target.webSocketDebuggerUrl, (m, cdpApi) => {
    if (m.method === 'Runtime.exceptionThrown') {
      const d = m.params.exceptionDetails; pageErrors.push(d.exception?.description || d.text); return;
    }
    if (m.method !== 'Fetch.requestPaused') return;
    const { requestId, request } = m.params;
    if (request.url.includes('roslib.min.js')) {
      return cdpApi.send('Fetch.fulfillRequest', { requestId, responseCode: 200,
        responseHeaders: [{ name: 'Content-Type', value: 'application/javascript' }],
        body: Buffer.from(ROSLIB_STUB).toString('base64') });
    }
    if (request.url.includes('/snapshot') || request.url.includes('fonts.googleapis')) {
      return cdpApi.send('Fetch.failRequest', { requestId, errorReason: 'Failed' });
    }
    return cdpApi.send('Fetch.continueRequest', { requestId });
  });
  await page.ready;
  await page.send('Page.enable'); await page.send('Runtime.enable');
  await page.send('Fetch.enable', { patterns: [{ urlPattern: '*' }] });
  if (SCREENSHOT_DIR) fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });
  const shot = async (name) => {
    if (!SCREENSHOT_DIR) return;
    const r = await page.send('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(path.join(SCREENSHOT_DIR, `${name}.png`), Buffer.from(r.result.data, 'base64'));
  };
  const $text = (sel) => page.eval(`(document.querySelector(${JSON.stringify(sel)}) || {}).textContent || ''`);
  const visible = (sel) => page.eval(`(() => { const e = document.querySelector(${JSON.stringify(sel)}); return !!e && !e.hidden && getComputedStyle(e).display !== 'none'; })()`);
  const emit = (name, msg) => page.eval(`window.__emit(${JSON.stringify(name)}, ${JSON.stringify(msg)})`);
  const click = (sel) => page.eval(`document.querySelector(${JSON.stringify(sel)}).click()`);
  // A real pointer press/release on the map at world (x, y), optionally dragged to (x2, y2).
  const mapPointer = async (x, y, x2 = null, y2 = null) => {
    const [px, py] = await page.eval(`(() => { const r = document.getElementById('map').getBoundingClientRect(); const [a, b] = window.__operator.toPx(${x}, ${y}); return [r.left + a, r.top + b]; })()`);
    await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x: px, y: py, button: 'left', clickCount: 1 });
    if (x2 !== null) {
      const [qx, qy] = await page.eval(`(() => { const r = document.getElementById('map').getBoundingClientRect(); const [a, b] = window.__operator.toPx(${x2}, ${y2}); return [r.left + a, r.top + b]; })()`);
      await page.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: qx, y: qy, button: 'left' });
      await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: qx, y: qy, button: 'left', clickCount: 1 });
    } else {
      await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: px, y: py, button: 'left', clickCount: 1 });
    }
    await sleep(150);
  };

  for (const vp of [{ label: '1366x768', w: 1366, h: 768 }, { label: '1920x1080', w: 1920, h: 1080 }]) {
    console.log(`\n--- ${vp.label} ---`);
    await page.send('Emulation.setDeviceMetricsOverride', { width: vp.w, height: vp.h, deviceScaleFactor: 1, mobile: false });
    await page.send('Page.navigate', { url: `http://127.0.0.1:${PORT}/operator.html` });
    await sleep(1500);

    // -- nothing running --------------------------------------------------------
    check(`${vp.label}: no session shows the empty state`, await visible('#empty') && (await $text('#empty-title')) === 'No session running');
    check(`${vp.label}: header says no session`, (await $text('#s-label')) === 'No session', await $text('#s-label'));
    check(`${vp.label}: neither mode is highlighted with no session`, await page.eval(`!document.querySelector('#mode button.active')`));
    check(`${vp.label}: page does not scroll`, await page.eval(`document.documentElement.scrollWidth <= innerWidth && document.documentElement.scrollHeight <= innerHeight`));
    await shot(`${vp.label}-empty`);

    // -- session panel (scenarios.js, reused) ------------------------------------
    await click('#empty-go');
    await sleep(800);
    check(`${vp.label}: Start a session opens the session panel`, await visible('#session') && await visible('#launch-form'));
    check(`${vp.label}: launch form is filled from the API`, await page.eval(`document.querySelectorAll('#lf-preset option').length > 1`));
    check(`${vp.label}: Start is wired (not disabled by a missing console)`, await page.eval(`!document.getElementById('lf-start').disabled`));
    await shot(`${vp.label}-session-panel`);
    await page.eval(`document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true })); window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
    await sleep(200);
    check(`${vp.label}: Escape closes the session panel`, !(await visible('#session')));

    // -- a running fleet ------------------------------------------------------------
    await emit('/map', grid(60, 40, 0.5, -10, -10));
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
    await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
    await page.eval(`window.__emit('/tf', { transforms: [] })`);
    await sleep(700);
    const rows = await page.eval(`[...document.querySelectorAll('#fleet .robot')].map((r) => r.querySelector('.name').textContent)`);
    check(`${vp.label}: every roster robot is listed`, rows.length === roster.length, rows.join(','));
    check(`${vp.label}: empty state clears once robots report`, !(await visible('#empty')));
    check(`${vp.label}: mapped area is shown`, /m²/.test(await $text('#cov-n')), await $text('#cov-n'));
    const online = await $text('#n-robots');
    check(`${vp.label}: online count reflects poses`, online.startsWith('2 of'), online);
    const pose = await page.eval(`JSON.stringify(window.__operator.robots.get(${JSON.stringify(r0)}).pose)`);
    check(`${vp.label}: pose composed from /tf`, JSON.parse(pose).x === -3 && Math.abs(JSON.parse(pose).yaw - 0.5) < 1e-6, pose);

    await emit(`/${r0}/explore_status`, { data: JSON.stringify({ state: 'exploring' }) });
    await sleep(500);
    check(`${vp.label}: exploring robot is labelled`, (await page.eval(`document.querySelector('#fleet .robot .tag').textContent`)) === 'exploring');

    // -- selecting a robot --------------------------------------------------------------
    await mapPointer(4, -2); // click robot 1 on the map
    await sleep(300);
    check(`${vp.label}: clicking a robot on the map selects it`, (await page.eval(`window.__operator.robots && document.querySelector('#fleet .robot.sel .name').textContent`)) === 'Robot 1');
    const href = await page.eval(`document.querySelector('#fleet .robot.sel a.open').getAttribute('href')`);
    check(`${vp.label}: live view link is the per-robot dashboard`, href === `/?robot=${r1}`, href);
    check(`${vp.label}: camera slot is in the selected card`, await page.eval(`!!document.querySelector('#fleet .robot.sel .cam')`));
    await shot(`${vp.label}-selected`);

    // -- waypoints ------------------------------------------------------------------------
    await page.eval(`document.querySelector('#tools [data-t="wp"]').click()`);
    await mapPointer(1, 1);
    await mapPointer(-5, -5, -5, -2); // dragged: heading along +y
    const wps = await page.eval(`JSON.stringify(window.__navWps)`);
    const parsed = JSON.parse(wps);
    check(`${vp.label}: waypoint tool adds named waypoints`, parsed.length === 2 && parsed[0].name === 'A' && parsed[1].name === 'B', wps);
    check(`${vp.label}: a dragged waypoint takes its heading`, Math.abs(parsed[1].yaw - Math.PI / 2) < 0.1 && parsed[0].yaw === 0, `yaw ${parsed[1]?.yaw}`);
    check(`${vp.label}: waypoints are listed`, (await page.eval(`document.querySelectorAll('#wps .wp').length`)) === 2);

    // -- sending a robot ----------------------------------------------------------------------
    await page.eval(`window.__ops.length = 0`);
    await click('#wps [data-go="A"]');
    let ops = await page.eval(`JSON.stringify(window.__ops)`);
    const sent = JSON.parse(ops);
    check(`${vp.label}: a waypoint chip sends the selected robot`, sent.length === 1 && sent[0].op === 'send_action_goal' &&
          sent[0].action === `/${r1}/navigate_to_pose` && sent[0].args.pose.header.frame_id === 'map', ops.slice(0, 160));
    await sleep(300);
    check(`${vp.label}: the robot row says where it is going`, /Going to A/.test(await page.eval(`document.querySelector('#fleet .robot.sel .l2').textContent`)));
    await page.eval(`window.__ops.length = 0`);
    await page.eval(`document.querySelector('#tools [data-t="goto"]').click()`);
    await mapPointer(2, 4);
    const sent2 = JSON.parse(await page.eval(`JSON.stringify(window.__ops)`));
    check(`${vp.label}: the send tool sends a goal where you click`, sent2.length === 1 && Math.abs(sent2[0].args.pose.pose.position.x - 2) < 0.1 &&
          Math.abs(sent2[0].args.pose.pose.position.y - 4) < 0.1, JSON.stringify(sent2[0]?.args?.pose?.pose?.position));
    await click('#wps [data-del="B"]');
    await sleep(100);
    check(`${vp.label}: a waypoint can be deleted`, (await page.eval(`window.__navWps.length`)) === 1);

    // -- tasks --------------------------------------------------------------------------------------
    await page.eval(`document.getElementById('newtask').open = true`);
    await page.eval(`document.getElementById('nt-who').value = ${JSON.stringify(r1)}; document.getElementById('nt-from').value = ${JSON.stringify(pick)}; document.getElementById('nt-to').value = ${JSON.stringify(drop)}`);
    await page.eval(`window.__published.length = 0; document.getElementById('task-form').requestSubmit()`);
    const pub = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
    const disp = pub.find((p) => p.topic === `/${r1}/dispatch_task`);
    const payload = disp && JSON.parse(disp.msg.data);
    check(`${vp.label}: adding a task dispatches it to the chosen robot`, !!payload && payload.pickup === pick && payload.dropoff === drop && !!payload.task_id, JSON.stringify(payload));
    check(`${vp.label}: the new task waits in the list`, /waiting/.test(await $text('#tasks')), (await $text('#tasks')).slice(0, 80));
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: payload.task_id, state: 'NAV_TO_PICKUP', queue_depth: 0 }) });
    await sleep(200);
    check(`${vp.label}: task status moves it to en route`, /en route/.test(await $text('#tasks')));
    check(`${vp.label}: the robot row follows its task`, /Heading to pick-up/.test(await page.eval(`document.querySelector('#fleet .robot.sel .l2').textContent`)));
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: payload.task_id, state: 'DONE', queue_depth: 0 }) });
    await sleep(200);
    check(`${vp.label}: DONE finishes the task`, /done/.test(await $text('#tasks')));
    await shot(`${vp.label}-fleet`);

    // -- stop all ---------------------------------------------------------------------------------------
    await page.eval(`window.__published.length = 0; window.__services.length = 0`);
    await click('#estop');
    const stopped = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
    check(`${vp.label}: Stop all stops exploring on every robot`, roster.every((id) => stopped.some((p) => p.topic === `/${id}/explore_cmd` && p.msg.data === 'stop')));
    check(`${vp.label}: Stop all zeroes every robot's velocity`, roster.every((id) => stopped.some((p) => p.topic === `/${id}/cmd_vel` && p.msg.linear.x === 0)));
    const svc = JSON.parse(await page.eval(`JSON.stringify(window.__services)`));
    check(`${vp.label}: Stop all cancels navigation goals`, roster.every((id) => svc.some((s) => s.name === `/${id}/navigate_to_pose/_action/cancel_goal`)));

    // -- a premapped session in the header -------------------------------------------------------------------
    await page.eval(`window.__fakeSession = { state: 'running', phase: 'running', phase_label: 'running', session_id: 's1', scenario: 'library_localize', config: { map: 'warehouse', mode: 'readonly', robots: 2 } }`);
    await sleep(3400);
    check(`${vp.label}: running session shows in the header`, (await $text('#s-label')) === 'Session running', await $text('#s-label'));
    check(`${vp.label}: a read-only saved map reads as Premapped`, (await page.eval(`document.querySelector('#mode button.active').dataset.m`)) === 'loc' && (await $text('#map-name')) === 'warehouse');
    await click('#mode [data-m="map"]');
    await sleep(300);
    check(`${vp.label}: choosing the other mode opens the session panel`, await visible('#session'));
    await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
    await page.eval(`window.__fakeSession = null`);
  }

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join('; '));
  page.close();
  if (SCREENSHOT_DIR) console.log(`\nscreenshots: ${SCREENSHOT_DIR}`);
  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} checks passed`);
  if (failed.length) { console.log('\nFAILED:'); for (const f of failed) console.log(`  - ${f.name}${f.detail ? `  (${f.detail})` : ''}`); }
  process.exit(failed.length ? 1 : 0);
}

main().catch((err) => { console.error('\nharness error:', err.message); cleanup(); process.exit(3); });
