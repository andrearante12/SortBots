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
window.__testPosts = []; window.__testResults = []; window.__sessionPosts = [];
window.__testRun = { running: false, name: null, exit: null, elapsed_s: null, log: '', offset: 0 };
window.__fakeSession = null;
const realFetch = window.fetch.bind(window);
window.fetch = (url, opts) => {
  const u = String(url);
  const json = (b) => Promise.resolve(new Response(JSON.stringify(b), { status: 200, headers: { 'Content-Type': 'application/json' } }));
  if (u.startsWith('/api/nav_waypoints')) {
    if (opts && opts.method === 'POST') window.__navWps = JSON.parse(opts.body).waypoints;
    return json({ platform: window.__platform || 'sim', waypoints: window.__navWps });
  }
  if (u.startsWith('/api/session') && !u.startsWith('/api/session/') && window.__fakeSession) return json(window.__fakeSession);
  // Stop all's dialog: record, never stop anything real.
  if ((u.startsWith('/api/session/stop') || u.startsWith('/api/session/finish')) && opts && opts.method === 'POST') {
    window.__sessionPosts.push({ url: u, body: JSON.parse(opts.body || '{}') });
    return json(window.__fakeSession || {});
  }
  // Test runs: never launch scripts/simtest.py from a page test. The list itself
  // (/api/tests) is real, read from configs/tests/.
  if (u.startsWith('/api/tests/run')) {
    if (opts && opts.method === 'POST') {
      const body = JSON.parse(opts.body); window.__testPosts.push(body);
      window.__testRun = { running: true, name: body.name, exit: null, elapsed_s: 75, log: '[simtest] launching\\n', offset: 20 };
    }
    return json({ ...window.__testRun, log: '' });
  }
  if (u.startsWith('/api/tests/cancel')) { window.__testRun = { ...window.__testRun, running: false, exit: 130 }; return json(window.__testRun); }
  if (u.startsWith('/api/tests/results')) return json({ results: window.__testResults });
  if (u.startsWith('/api/tests/report')) return json({ markdown: '# crossing_yield\\nrecall 0.12' });
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
// Every temp dir a run makes (Chrome profile, sessions, seeded maps) is removed on
// exit: they were never cleaned, and a day of runs filled the disk (2026-10-22).
const tempDirs = [];
const tmpDir = (prefix) => { const d = fs.mkdtempSync(path.join(os.tmpdir(), prefix)); tempDirs.push(d); return d; };
function cleanup() {
  for (const p of children) { try { p.kill('SIGKILL'); } catch {} }
  // SIGKILL is asynchronous: Chrome writes into its profile while it dies, so wait
  // (up to 2 s, synchronously: this runs in an 'exit' handler) before deleting.
  const nap = new Int32Array(new SharedArrayBuffer(4));
  for (const p of children) {
    for (let i = 0; i < 40 && p.pid; i++) { try { process.kill(p.pid, 0); } catch { break; } Atomics.wait(nap, 0, 0, 50); }
  }
  for (const d of tempDirs) { try { fs.rmSync(d, { recursive: true, force: true }); } catch {} }
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
  const sessionsDir = tmpDir('sortbots-sessions-');
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--control', '--platform', 'sim',
                           '--port', String(PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: { ...process.env, SORTBOTS_SESSIONS_DIR: sessionsDir,
                 // The gateway pin is saved by the real serve.py: never into data/.
                 SORTBOTS_COMMS_GATEWAY: path.join(sessionsDir, 'comms_gateway.json') } });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${PORT}/operator.html`)).ok, { what: 'serve.py' });
  const roster = (await (await fetch(`http://127.0.0.1:${PORT}/api/robots`)).json()).robots || [];
  check('roster has at least two robots', roster.length >= 2, roster.join(','));
  const [r0, r1] = roster;
  // Pick-up / drop-off are plain waypoints now (merged from the old stations).
  // The map test below places A and B; a task dispatches between those two.
  const pick = 'A', drop = 'B';

  const profile = tmpDir('sortbots-chrome-');
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
    check(`${vp.label}: no mode on the session chip without a session`, (await page.eval(`document.getElementById('session-chip').dataset.mode || ''`)) === '');
    check(`${vp.label}: page does not scroll`, await page.eval(`document.documentElement.scrollWidth <= innerWidth && document.documentElement.scrollHeight <= innerHeight`));
    await shot(`${vp.label}-empty`);

    // -- session panel (scenarios.js, reused) ------------------------------------
    await click('#empty-go');
    await sleep(800);
    check(`${vp.label}: Start a session opens the session panel`, await visible('#session') && await visible('#launch-form'));
    check(`${vp.label}: launch form is filled from the API`, await page.eval(`document.querySelectorAll('#lf-preset option').length > 1`));
    check(`${vp.label}: Start is wired (not disabled by a missing console)`, await page.eval(`!document.getElementById('lf-start').disabled`));
    check(`${vp.label}: with no session the panel is just the form`, await visible('#launch-form') && !(await visible('#session-pane')) && !(await visible('#lf-command')) && !(await visible('#lf-summary')));
    check(`${vp.label}: no Quick start row: the form is custom`, !(await visible('#lf-preset')) && (await page.eval(`document.getElementById('lf-preset').value`)) === '');
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
    check(`${vp.label}: no link out to the old dashboard`, await page.eval(`!document.querySelector('#fleet a[href^="/?robot"]')`));
    check(`${vp.label}: camera slot is in the selected card`, await page.eval(`!!document.querySelector('#fleet .robot.sel .cam')`));
    await shot(`${vp.label}-selected`);

    // -- several cards open at once --------------------------------------------------------
    await click(`#fleet .robot[data-id="${r0}"] .l1`);
    await sleep(200);
    check(`${vp.label}: a second card opens without closing the first`, (await page.eval(`document.querySelectorAll('#fleet .robot.sel').length`)) === 2);
    check(`${vp.label}: the newest card takes control`, (await page.eval(`window.__operator.selected`)) === r0 && await page.eval(`!!document.querySelector('#fleet .robot.active[data-id="${r0}"] .ctl')`));
    check(`${vp.label}: each open card has its own camera`, (await page.eval(`document.querySelectorAll('#fleet .robot.sel .cam').length`)) === 2);
    const r1Mode = await page.eval(`window.__operator.cams.get(${JSON.stringify(r1)}).mode`);
    await page.eval(`window.__operator.cams.get(${JSON.stringify(r0)}).mode = 'rgb'`); // the last pick is remembered across loads
    await click(`#fleet .robot[data-id="${r0}"] [data-cam-mode="depth"]`);
    check(`${vp.label}: the camera tile switches to depth`, await page.eval(`window.__operator.cams.get(${JSON.stringify(r0)}).mode === 'depth' && document.querySelector('#fleet .robot[data-id="${r0}"] [data-cam-mode="depth"]').classList.contains('on')`));
    check(`${vp.label}: the other card keeps its own camera mode`, await page.eval(`window.__operator.cams.get(${JSON.stringify(r1)}).mode === ${JSON.stringify(r1Mode)}`));
    check(`${vp.label}: a click inside a card leaves both open`, (await page.eval(`document.querySelectorAll('#fleet .robot.sel').length`)) === 2);
    await emit(`/${r0}/odom`, { twist: { twist: { linear: { x: 0.5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: Math.PI / 4 } } } });
    await emit(`/${r0}/task_status`, { data: JSON.stringify({ task_id: 'tt1', state: 'NAV_TO_GOAL', queue_depth: 1, task: { task_id: 'tt1', kind: 'goto', goto: 'Dock' }, queue: [] }) });
    await emit(`/${r0}/explore_status`, { data: JSON.stringify({ state: 'idle' }) });
    await sleep(400);
    const f0 = (k) => page.eval(`document.querySelector('#fleet .facts[data-id="${r0}"] dd[data-f="${k}"]').textContent`);
    check(`${vp.label}: the card shows position`, (await f0('pos')) === '-3.00, 2.00 m', await f0('pos'));
    check(`${vp.label}: the card shows heading`, (await f0('heading')) === '29°', await f0('heading'));
    check(`${vp.label}: the card shows speed and turn rate from odom`, (await f0('speed')) === '0.50 m/s' && (await f0('turn')) === '45°/s', `${await f0('speed')} ${await f0('turn')}`);
    const pill = await page.eval(`(() => { const r = document.querySelector('#fleet .robot[data-id="${r0}"]'); return r.querySelector('.tag').className + '|' + r.querySelector('.l2').textContent; })()`);
    check(`${vp.label}: the status pill is coloured as moving and the line names the task`, /\bpill\b.*\bmoving\b/.test(pill) && /Going to Dock/.test(pill) && /1 waiting/.test(pill), pill);
    check(`${vp.label}: no separate task tile`, await page.eval(`!document.querySelector('#fleet .tile')`));
    check(`${vp.label}: a busy robot offers Clear tasks`, await page.eval(`!!document.querySelector('#fleet .robot[data-id="${r0}"] [data-act="clear"]')`));
    await page.eval(`window.__published.length = 0`);
    await click(`#fleet .robot[data-id="${r0}"] [data-act="clear"]`);
    check(`${vp.label}: Clear tasks cancels everything on that robot`, await page.eval(`window.__published.some((x) => x.topic === '/${r0}/cancel_task' && x.msg.data === 'all')`));
    await emit(`/${r0}/task_status`, { data: JSON.stringify({ task_id: null, state: 'IDLE', queue_depth: 0, queue: [] }) });
    await sleep(300);
    check(`${vp.label}: an idle robot's pill is the idle colour`, await page.eval(`document.querySelector('#fleet .robot[data-id="${r0}"] .tag').classList.contains('idle')`));

    // -- obstacles (nodes/obstacle_detector.py) ---------------------------------------------
    // r0 sits at (-3, 2) facing 0.5 rad. One person 2 m dead ahead crossing to its right,
    // one static box; no world_frame, so velocity is in base_link like a detector without
    // odom TF would send it.
    const ahead = [-3 + 2 * Math.cos(0.5), 2 + 2 * Math.sin(0.5)];
    const obstacles = (moving) => ({ data: JSON.stringify({ stamp: 1, frame: `${r0}/base_link`, world_frame: null, floor_z: 0,
      w: 848, h: 480, count: 2, moving: moving ? 1 : 0, obstacles: [
        { track_id: 4, moving, speed_mps: 1.2, vel: [0, -1.2], centroid: [2.0, 0, 0.9], min: [1.9, -0.2, 0.1], max: [2.1, 0.2, 1.7],
          extent_m: 0.4, height_m: 1.7, range_m: 1.9, bearing_deg: 0, n: 900, uv: [380, 60, 470, 470], clipped: false, hits: 8 },
        { track_id: 2, moving: false, speed_mps: 0, vel: [0, 0], centroid: [3.0, 1.0, 0.3], min: [2.9, 0.8, 0.1], max: [3.1, 1.2, 0.5],
          extent_m: 0.4, height_m: 0.5, range_m: 3.0, bearing_deg: 18, n: 200, uv: [100, 300, 200, 400], clipped: false, hits: 30 },
      ] }) });
    // Obstacle red (#e5484d) on a canvas, optionally only within `rad` px of (cx, cy).
    const redPx = (sel, cx = null, cy = null, rad = 0) => page.eval(`(() => {
      const c = document.querySelector(${JSON.stringify(sel)});
      if (!c || !c.width) return -1;
      const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data, s = c.width / c.clientWidth;
      let n = 0;
      for (let y = 0; y < c.height; y++) for (let x = 0; x < c.width; x++) {
        if (${cx} !== null && Math.hypot(x / s - ${cx}, y / s - ${cy}) > ${rad}) continue;
        const i = (y * c.width + x) * 4;
        if (d[i] > 200 && d[i + 1] < 100 && d[i + 2] < 100 && d[i + 3] > 128) n++;
      }
      return n;
    })()`);
    check(`${vp.label}: no obstacle red on the map before any detection`, (await redPx('#map')) === 0);
    await emit(`/${r0}/obstacle_tracks`, obstacles(true));
    await sleep(400);
    const [ax, ay] = await page.eval(`window.__operator.toPx(${ahead[0]}, ${ahead[1]})`);
    // The marker, its 1 s velocity arrow and its label, all around the obstacle.
    const near = await redPx('#map', ax, ay, 90), all = await redPx('#map');
    check(`${vp.label}: a moving obstacle is marked on the map 2 m ahead of its robot`, near > 30 && near === all, `near=${near} all=${all}`);
    check(`${vp.label}: the card counts obstacles, moving ones and the nearest`, (await f0('obst')) === '2, 1 moving · nearest 1.9 m', await f0('obst'));
    check(`${vp.label}: the camera tile boxes the obstacles`, (await redPx(`#fleet .robot[data-id="${r0}"] canvas.obst`)) > 50);
    await emit(`/${r0}/obstacle_tracks`, obstacles(false));
    await sleep(300);
    check(`${vp.label}: static obstacles are left off the map`, (await redPx('#map')) === 0);
    check(`${vp.label}: a static-only frame still counts them on the card`, (await f0('obst')) === '2 · nearest 1.9 m', await f0('obst'));
    await emit(`/${r0}/obstacle_tracks`, obstacles(true));
    await sleep(2000); // > OBSTACLE_TTL_MS with nothing new
    check(`${vp.label}: a silent detector's markers expire from the map`, (await redPx('#map')) === 0);
    check(`${vp.label}: ... from the camera tile`, (await redPx(`#fleet .robot[data-id="${r0}"] canvas.obst`)) === 0);
    check(`${vp.label}: ... and from the card`, (await f0('obst')) === '—', await f0('obst'));
    const tfm = await page.eval(`JSON.stringify([
      window.__operator.obstaclesInMap({ obstacles: [{ track_id: 1, centroid: [1, 0, 0], min: [1, 0, 0], max: [1, 0, 0], vel: [1, 0] }] }, { x: 1, y: 2, yaw: Math.PI / 2 }, { yaw: 0 })[0],
      window.__operator.obstaclesInMap({ obstacles: [{ track_id: 1, centroid: [1, 0, 0], min: [1, 0, 0], max: [1, 0, 0], vel: [1, 0] }] }, { x: 1, y: 2, yaw: Math.PI / 2 }, null)[0]])`);
    const [wOdom, wBase] = JSON.parse(tfm), close = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-9);
    check(`${vp.label}: obstacle positions go base_link -> map, velocity stays in its world frame`, close(wOdom.at, [1, 3]) && close(wOdom.vel, [1, 0]), tfm);
    check(`${vp.label}: velocity without a world frame is rotated like a position`, close(wBase.vel, [0, 1]), tfm);
    // That wait also outlasted ONLINE_TTL_MS for both robots; a live one keeps sending /tf.
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
    await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
    await emit(`/${r0}/obstacle_tracks`, obstacles(true));
    await sleep(300);
    await shot(`${vp.label}-obstacles`);
    await shot(`${vp.label}-two-cards`);
    await click(`#fleet .robot[data-id="${r0}"] .l1`);
    await sleep(200);
    check(`${vp.label}: closing a card hands control back to the other open one`, (await page.eval(`window.__operator.selected`)) === r1 && (await page.eval(`document.querySelectorAll('#fleet .robot.sel').length`)) === 1);

    // -- scored tests: the Session panel's Tests tab, the map chip, the walkers ----------------
    await click('#session-chip');
    await sleep(300);
    await click('#sp-tabs [data-tab="tests"]');
    await sleep(900);
    check(`${vp.label}: the session panel has a Tests tab`, await visible('#tests-pane') && !(await visible('#launch-form')));
    const cards = await page.eval(`[...document.querySelectorAll('#tp-list .tp-test')].map((c) => c.dataset.name || 'invalid')`);
    const onDisk = fs.readdirSync(path.join(REPO_ROOT, 'configs', 'tests')).filter((f) => f.endsWith('.yaml')).length;
    check(`${vp.label}: it lists every test case in configs/tests/`, cards.length === onDisk, cards.join(','));
    check(`${vp.label}: the smoke test is runnable here`,
      await page.eval(`(() => { const c = document.querySelector('#tp-list .tp-test[data-name="pipeline_smoke"] [data-run]'); return !!c && !c.disabled; })()`));
    const smokeTitle = await $text('#tp-list .tp-test[data-name="pipeline_smoke"] .t');
    check(`${vp.label}: rows are numbered short names`, /^\d+Pipeline smoke$/.test(smokeTitle), smokeTitle);
    check(`${vp.label}: no time/variant line under the names`, !(await page.eval(`!!document.querySelector('#tp-list .tp-row .m')`)));
    const smokeName = 'Pipeline smoke';
    check(`${vp.label}: rows start collapsed`, !(await visible('#tp-list .tp-test[data-name="pipeline_smoke"] .tp-more')));
    await click('#tp-list .tp-test[data-name="crossing_yield"] .tp-row .t');
    await sleep(100);
    const more = await $text('#tp-list .tp-test[data-name="crossing_yield"] .tp-more');
    check(`${vp.label}: clicking a row shows what it checks`, await visible('#tp-list .tp-test[data-name="crossing_yield"] .tp-more') && /Passes when/.test(more) && /recall ≥ 0.3/.test(more), more.slice(0, 160));
    await click('#tp-list .tp-test[data-name="pipeline_smoke"] .tp-row .t');
    await page.eval(`(() => { const c = document.querySelector('#tp-list .tp-test[data-name="pipeline_smoke"]'); c.querySelector('[data-repeats]').value = '2'; c.querySelector('[data-run]').click(); })()`);
    await sleep(400);
    const posted = JSON.parse(await page.eval(`JSON.stringify(window.__testPosts)`));
    check(`${vp.label}: Run starts that test with the chosen repeats`, posted.length === 1 && posted[0].name === 'pipeline_smoke' && posted[0].repeats === 2, JSON.stringify(posted));
    check(`${vp.label}: ... and closes the panel to watch it on the map`, !(await visible('#session')));
    check(`${vp.label}: a chip over the map names the running test and its clock`,
      await visible('#test-chip') && (await $text('#tc-name')) === smokeName && (await $text('#tc-time')) === '1:15', `${await $text('#tc-name')} ${await $text('#tc-time')}`);
    await shot(`${vp.label}-test-running`);
    await click('#tc-name');
    await sleep(500);
    check(`${vp.label}: the chip's name reopens the Tests tab`, await visible('#tests-pane') && await visible('#tp-run') && /running · 1:15/.test(await $text('#tp-run-state')));
    check(`${vp.label}: ... where Run is disabled while a test runs`,
      await page.eval(`[...document.querySelectorAll('#tp-list [data-run]')].every((b) => b.disabled)`));
    check(`${vp.label}: while a test runs, the session's own Stop gives way to Stop test`,
      await page.eval(`document.querySelector('#session .card').classList.contains('testing')`) &&
      await page.eval(`getComputedStyle(document.getElementById('session-stop')).display === 'none' && getComputedStyle(document.getElementById('sp-test-note')).display !== 'none'`));
    await page.eval(`window.__testResults = [{ dir: '20261022T190000Z_pipeline_smoke', name: 'pipeline_smoke', state: 'done', complete: true, verdict: 'FAIL', done: 1, planned: 1,
      runs: [{ ok: false, error: null, sim_s: 41.2, criteria: [{ label: 'clearance.contacts == 0', ok: false, actual: 3 }, { label: 'goals.success_rate >= 1', ok: true, actual: 1 }] }] }]`);
    await click('#sp-x');
    await click('#tc-cancel');
    await sleep(300);
    check(`${vp.label}: Stop test cancels it and the chip goes`, !(await visible('#test-chip')) && (await page.eval(`window.__testRun.exit`)) === 130);
    await sleep(300);
    const card = await $text('#result-card');
    check(`${vp.label}: a finished test opens a result card with every check`, await visible('#result-card') && /STOPPED/.test(card) &&
      /Clearance contacts = 0/.test(card) && /got 3/.test(card) && /Goals success rate ≥ 1/.test(card), card.slice(0, 200));
    check(`${vp.label}: ... marking the failed ones`, (await page.eval(`document.querySelectorAll('#rc-checks li.bad').length`)) === 1 && (await page.eval(`document.querySelectorAll('#rc-checks .ok').length`)) === 1);
    await shot(`${vp.label}-result-card`);
    await sleep(4500);
    check(`${vp.label}: ... and stays until dismissed`, await visible('#result-card'));
    await click('#rc-x');
    check(`${vp.label}: the × dismisses it`, !(await visible('#result-card')));

    await click('#session-chip');
    await sleep(900);
    check(`${vp.label}: the panel remembers the Tests tab`, await visible('#tests-pane'));
    check(`${vp.label}: no results list, and no status line once the test is over`, !(await page.eval(`!!document.getElementById('tp-results')`)) && !(await visible('#tp-run')));
    await shot(`${vp.label}-tests-tab`);
    await click('#sp-tabs [data-tab="launch"]');
    check(`${vp.label}: the Launch tab brings the form back`, await visible('#launch-form') && !(await visible('#tests-pane')));
    await click('#sp-x');

    // Walkers come from sim ground truth; one green walker at (1, 1).
    const greenNear = (x, y, rad) => page.eval(`(() => {
      const c = document.getElementById('map'), s = c.width / c.clientWidth, [cx, cy] = window.__operator.toPx(${x}, ${y});
      const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let n = 0;
      for (let py = 0; py < c.height; py++) for (let px = 0; px < c.width; px++) {
        if (Math.hypot(px / s - cx, py / s - cy) > ${rad}) continue;
        const i = (py * c.width + px) * 4; if (d[i + 1] > 150 && d[i] < 60 && d[i + 2] < 90) n++;
      }
      return n; })()`);
    check(`${vp.label}: no walker drawn without a sim world`, (await greenNear(1, 1, 30)) === 0);
    await emit('/sim/world', { data: JSON.stringify({ world: 'aisle_crossing', title: 'Aisle crossing', props: [],
      actors: [{ id: 'walker_1', label: 'walker', shape: 'cylinder', radius: 0.3, color: [0.1, 0.75, 0.2], path: [[-1, 1], [3, 1]], mode: 'pingpong' }] }) });
    await emit('/sim/ground_truth', { data: JSON.stringify({ wt: 4.2, running: true, actors: { walker_1: { x: 1, y: 1, yaw: 0, moving: true } } }) });
    await sleep(300);
    check(`${vp.label}: a walker is drawn at its ground-truth position`, (await greenNear(1, 1, 12)) > 40, String(await greenNear(1, 1, 12)));
    await shot(`${vp.label}-walker`);
    await sleep(3600); // > WORLD_TRUTH_TTL_MS with no ground truth
    check(`${vp.label}: walkers vanish when the sim stops sending ground truth`, (await greenNear(1, 1, 30)) === 0);
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5)); // that wait outlasted ONLINE_TTL_MS
    await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));

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
    const rowGoing = await page.eval(`document.querySelector('#fleet .robot.sel .l2').textContent`);
    check(`${vp.label}: the robot row says where it is going`, /Going to A/.test(rowGoing), rowGoing);
    // -- drag a robot to send it (no send tool) ---------------------------------------------------
    check(`${vp.label}: there is no send tool any more`, await page.eval(`!document.querySelector('#tools [data-t="goto"]')`));
    await page.eval(`document.querySelector('#tools [data-t="select"]').click()`);
    const goals = async () => JSON.parse(await page.eval(`JSON.stringify(window.__ops.filter((o) => o.op === 'send_action_goal'))`));
    const at = (x, y) => page.eval(`(() => { const r = document.getElementById('map').getBoundingClientRect(); const [a, b] = window.__operator.toPx(${x}, ${y}); return [r.left + a, r.top + b]; })()`);
    const mouse = (type, [x, y]) => page.send('Input.dispatchMouseEvent', { type, x, y, button: 'left', clickCount: 1 });
    await page.eval(`window.__ops.length = 0`);
    await mouse('mousePressed', await at(4, -2)); // r1
    await mouse('mouseMoved', await at(3, 1));
    await mouse('mouseMoved', await at(2, 4));
    const mid = JSON.parse(await page.eval(`JSON.stringify(window.__operator.drag)`));
    check(`${vp.label}: dragging a robot previews the send`, !!mid && mid.id === r1 && Math.abs(mid.wx - 2) < 0.1 && Math.abs(mid.wy - 4) < 0.1, JSON.stringify(mid));
    check(`${vp.label}: ... and sends nothing until it is dropped`, (await goals()).length === 0);
    await shot(`${vp.label}-drag-send`);
    await mouse('mouseReleased', await at(2, 4));
    await sleep(150);
    let g = await goals();
    const sp = g[0]?.args?.pose?.pose, gyaw = sp ? 2 * Math.atan2(sp.orientation.z, sp.orientation.w) : NaN;
    check(`${vp.label}: dropping it sends that robot there`, g.length === 1 && g[0].action === `/${r1}/navigate_to_pose` &&
          Math.abs(sp.position.x - 2) < 0.1 && Math.abs(sp.position.y - 4) < 0.1, JSON.stringify(sp?.position));
    check(`${vp.label}: ... arriving facing the way it travelled`, Math.abs(gyaw - Math.atan2(6, -2)) < 0.05, `yaw ${gyaw}`);
    check(`${vp.label}: a drag-send leaves the view where it was (no pan)`, await page.eval(`!window.__operator.drag`));

    await page.eval(`window.__ops.length = 0`);
    await mapPointer(4, -2, -5, -5); // onto pin B
    g = await goals();
    const bp = g[0]?.args?.pose?.pose, byaw = bp ? 2 * Math.atan2(bp.orientation.z, bp.orientation.w) : NaN;
    check(`${vp.label}: dropping on a pin sends it to that waypoint, with the pin's heading`, g.length === 1 &&
          Math.abs(bp.position.x + 5) < 1e-6 && Math.abs(bp.position.y + 5) < 1e-6 && Math.abs(byaw - Math.PI / 2) < 0.1, JSON.stringify(bp));
    check(`${vp.label}: ... and names it`, /Going to B/.test(await page.eval(`document.querySelector('#fleet .robot[data-id="${r1}"] .l2').textContent`)));

    await page.eval(`window.__ops.length = 0`);
    const openBefore = await page.eval(`[...window.__operator.open].join()`);
    await mapPointer(-3, 2, -1, 0); // r0, whose card isn't the selected one
    g = await goals();
    check(`${vp.label}: any robot can be dragged, not just the selected one`, g.length === 1 && g[0].action === `/${r0}/navigate_to_pose`, g[0]?.action);
    check(`${vp.label}: ... without opening or closing cards`, (await page.eval(`[...window.__operator.open].join()`)) === openBefore);

    await page.eval(`window.__ops.length = 0`);
    const [r1x, r1y] = await at(4, -2); // a real drag (> 6 px) that ends back on the robot (< 16 px)
    await mouse('mousePressed', [r1x, r1y]);
    await mouse('mouseMoved', [r1x + 10, r1y]);
    await mouse('mouseReleased', [r1x + 10, r1y]);
    await sleep(150);
    check(`${vp.label}: dropping a robot back on itself sends nothing`, (await goals()).length === 0);

    await mouse('mousePressed', await at(4, -2));
    await mouse('mouseMoved', await at(0, 3));
    await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
    await mouse('mouseReleased', await at(0, 3));
    await sleep(150);
    check(`${vp.label}: Escape abandons a drag-send`, (await goals()).length === 0 && await page.eval(`!window.__operator.drag`));

    const vc = await page.eval(`JSON.stringify([window.__operator.view.cx, window.__operator.view.cy])`);
    await mapPointer(0, 3, 1, 4); // empty map
    check(`${vp.label}: dragging empty map still pans`, (await page.eval(`JSON.stringify([window.__operator.view.cx, window.__operator.view.cy])`)) !== vc && (await goals()).length === 0);
    check(`${vp.label}: no instruction text over the map; the select tool's tooltip teaches the drag`,
      !(await page.eval(`!!document.getElementById('hint')`)) && /Drag a robot/.test(await page.eval(`document.querySelector('#tools [data-t="select"]').title`)));
    check(`${vp.label}: map tools, mapped area and the test chip sit in the top bar`,
      await page.eval(`['tools', 'coverage', 'test-chip'].every((id) => document.getElementById(id).closest('header'))`));

    // -- tasks --------------------------------------------------------------------------------------
    // A and B are both still placed here (deletion is exercised after the demo
    // fan-out below, which needs both waypoints to resolve its task pairs).
    await page.eval(`document.getElementById('newtask').open = true; document.querySelector('#nt-kind [data-k="pick"]').click()`);
    check(`${vp.label}: Pick & drop shows the pick-up form`, await visible('#task-form') && !(await visible('#move-form')));
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
    const rowTask = await page.eval(`document.querySelector('#fleet .robot.sel .l2').textContent`);
    check(`${vp.label}: the robot row follows its task`, /Heading to pick-up/.test(rowTask), rowTask);
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: payload.task_id, state: 'DONE', queue_depth: 0 }) });
    await sleep(200);
    check(`${vp.label}: DONE finishes the task`, /done/.test(await $text('#tasks')));
    await shot(`${vp.label}-fleet`);

    // -- demo ----------------------------------------------------------------------------------------------
    // configs/demo_tasks.yaml (served live by serve.py) is one list, queued all at once so
    // the robots run in parallel. Its waypoint names are the warehouse map's, so swap
    // those in for the duration.
    const demo = (await (await fetch(`http://127.0.0.1:${PORT}/api/demo_tasks`)).json()).tasks || [];
    check(`${vp.label}: the demo covers more than one robot`, new Set(demo.map((t) => t.robot)).size >= 2, `${demo.length} tasks`);
    const savedWps = await page.eval(`JSON.stringify(window.__navWps)`);
    const demoNames = [...new Set(demo.flatMap((t) => [t.goto, t.pickup, t.dropoff]).filter(Boolean))];
    await page.eval(`window.__navWps = ${JSON.stringify(demoNames.map((name, i) => ({ name, x: i, y: 1, yaw: 0 })))}; window.__operator.loadWaypoints()`);
    await sleep(200);
    await page.eval(`window.__published.length = 0`);
    await click('#tk-demo');
    await sleep(300);
    const dem = JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).filter((x) => /\/dispatch_task$/.test(x.topic));
    check(`${vp.label}: Run demo queues every task at once`, dem.length === demo.length, `${dem.length}/${demo.length}`);
    check(`${vp.label}: each demo task goes to the robot it names, in order`, dem.every((x, i) => x.topic === `/${demo[i].robot}/dispatch_task`), dem.map((x) => x.topic).join(','));
    check(`${vp.label}: the banner counts the demo's tasks`, (await $text('#demo-status')) === `Demo running: ${demo.length} of ${demo.length} tasks left` && (await $text('#tk-demo')) === 'Stop demo', await $text('#demo-status'));
    const ids = dem.map((x) => [x.topic.split('/')[1], JSON.parse(x.msg.data).task_id]);
    await emit(`/${ids[0][0]}/task_status`, { data: JSON.stringify({ task_id: ids[0][1], state: 'DONE', queue_depth: 0 }) });
    await sleep(200);
    check(`${vp.label}: the banner counts down as tasks finish`, (await $text('#demo-status')).startsWith(`Demo running: ${demo.length - 1} of`), await $text('#demo-status'));
    await page.eval(`window.__published.length = 0`);
    await click('#tk-demo');
    const cancels = JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).filter((x) => /\/cancel_task$/.test(x.topic) && x.msg.data === 'all');
    check(`${vp.label}: Stop demo cancels every robot it used`, new Set(cancels.map((x) => x.topic)).size === new Set(demo.map((t) => t.robot)).size, cancels.map((x) => x.topic).join(','));
    check(`${vp.label}: stopping clears the demo status`, !(await visible('#demo-status')) && (await $text('#tk-demo')) === 'Run demo');
    await page.eval(`window.__navWps = ${savedWps}; window.__operator.loadWaypoints()`);
    await sleep(200);
    // The demo checks outlast ONLINE_TTL_MS; refresh the poses so the robots are live again.
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
    await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));

    // -- move queue ----------------------------------------------------------------------------------------
    // Build a route A, B, A -> reorder -> drop one, then queue it: one goto task per stop,
    // all on the chosen robot, in route order.
    const pickStop = (name) => page.eval(`(() => { const s = document.getElementById('mv-add'); s.value = ${JSON.stringify(name)}; s.dispatchEvent(new Event('change')); })()`);
    const stops = () => page.eval(`[...document.querySelectorAll('#mv-stops li span')].map((x) => x.textContent).join(',')`);
    await page.eval(`document.querySelector('#nt-kind [data-k="moves"]').click(); document.getElementById('nt-who').value = ${JSON.stringify(r1)}`);
    check(`${vp.label}: Moves is the default kind`, await visible('#move-form') && !(await visible('#task-form')));
    await pickStop('A'); await pickStop('B'); await pickStop('A');
    check(`${vp.label}: picking waypoints builds a route`, (await stops()) === 'A,B,A', await stops());
    check(`${vp.label}: the picker resets after each stop`, (await page.eval(`document.getElementById('mv-add').value`)) === '');
    await click('#mv-stops [data-up="1"]');
    check(`${vp.label}: a stop can move earlier`, (await stops()) === 'B,A,A', await stops());
    await click('#mv-stops [data-rm="2"]');
    check(`${vp.label}: a stop can be removed`, (await stops()) === 'B,A', await stops());
    check(`${vp.label}: the button counts the moves`, (await $text('#mv-go')) === 'Queue 2 moves', await $text('#mv-go'));
    await page.eval(`window.__published.length = 0; document.getElementById('move-form').requestSubmit()`);
    const moves = JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).filter((x) => /\/dispatch_task$/.test(x.topic));
    const mp = moves.map((x) => JSON.parse(x.msg.data));
    check(`${vp.label}: a route queues one goto per stop, in order, on one robot`,
          mp.length === 2 && mp[0].goto === 'B' && mp[1].goto === 'A' && moves.every((x) => x.topic === `/${r1}/dispatch_task`) && mp.every((x) => x.task_id), JSON.stringify(mp));
    check(`${vp.label}: queueing clears the route`, (await page.eval(`document.querySelectorAll('#mv-stops li:not(.empty)').length`)) === 0);
    check(`${vp.label}: queued moves are listed`, /Go to B/.test(await $text('#tasks')) && /Go to A/.test(await $text('#tasks')), (await $text('#tasks')).slice(0, 120));

    // task_manager's status: B running, A waiting, plus a task queued from elsewhere.
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: mp[0].task_id, state: 'NAV_TO_GOAL', queue_depth: 2,
      task: { task_id: mp[0].task_id, kind: 'goto', goto: 'B' },
      queue: [{ task_id: mp[1].task_id, kind: 'goto', goto: 'A' }, { task_id: 'ext1', kind: 'pickup', pickup: 'A', dropoff: 'B' }] }) });
    await sleep(200);
    const l2now = await page.eval(`[...document.querySelectorAll('#fleet .robot.sel')].map((e) => e.dataset.id + ":" + e.querySelector(".l2").textContent).join("|")`);
    check(`${vp.label}: the robot row names the move's destination`, /Going to B/.test(l2now), l2now);
    check(`${vp.label}: a task queued elsewhere shows up from the status queue`, await page.eval(`window.__operator.tasks.some((t) => t.id === 'ext1' && t.state === 'QUEUED' && t.pickup === 'A')`));
    await page.eval(`window.__published.length = 0`);
    await click(`#tasks [data-cancel="${mp[1].task_id}"]`);
    const cancel = JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).find((x) => x.topic === `/${r1}/cancel_task`);
    check(`${vp.label}: cancelling a waiting task publishes its id`, !!cancel && cancel.msg.data === mp[1].task_id, JSON.stringify(cancel));
    check(`${vp.label}: the cancelled task reads as cancelled`, await page.eval(`window.__operator.tasks.find((t) => t.id === ${JSON.stringify(mp[1].task_id)}).state === 'CANCELLED'`));
    check(`${vp.label}: finished tasks have no cancel button`, !(await page.eval(`!!document.querySelector('#tasks [data-cancel="${mp[1].task_id}"]')`)));
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: null, state: 'IDLE', queue_depth: 0, queue: [], rejected: { task_id: 'ext1', reason: "unknown waypoint 'Z'" } }) });
    await sleep(200);
    check(`${vp.label}: a rejected task is marked failed`, await page.eval(`window.__operator.tasks.find((t) => t.id === 'ext1').state === 'FAILED'`));

    // -- fleet auction board ------------------------------------------------------------------------
    // nodes/fleet_tasks.py --gateway's /fleet/task_board: liveness by the fleet's own
    // rule, battery, who holds which task, latched alarms. Without one the page is
    // exactly the pre-auction page; with one, "Any robot" goes to the auction.
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
    await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
    await sleep(300);
    const autoLabel = () => page.eval(`document.querySelector('#nt-who option[value="auto"]').textContent`);
    check(`${vp.label}: with no fleet board, auto means any free robot`, (await autoLabel()) === 'Any free robot', await autoLabel());
    check(`${vp.label}: with no fleet board there is no alarm chip`, !(await visible('#alarms')));
    const fboard = (tasks, alarms = [], r0alive = true) => ({ data: JSON.stringify({ t: 100.0,
      robots: [{ id: r0, alive: r0alive, last_heard_s: r0alive ? 0.2 : 9.0, battery_pct: 14.0, mode: 'idle', owned: [], carrying: [] },
               { id: r1, alive: true, last_heard_s: 0.1, battery_pct: 81.5, mode: 'nav', owned: [], carrying: [] }],
      tasks, alarms }) });
    await emit('/fleet/task_board', fboard([], [
      { id: 1, kind: 'robot_silent', robot: r0, task_id: null, t: 95.0, active: true, acked: false, text: `${r0} silent for 6 s` },
      { id: 2, kind: 'lease_expired', robot: r0, task_id: 9, t: 60.0, active: false, acked: false, text: 'task 9: lease expired, re-auctioned' }], false));
    await sleep(400);
    const rowOf = (id) => page.eval(`(() => { const e = document.querySelector('#fleet .robot[data-id="${id}"]'); return e ? e.querySelector('.l1').textContent + ' | ' + e.querySelector('.l2').textContent : ''; })()`);
    check(`${vp.label}: a robot the fleet calls silent reads silent`, /silent/.test(await rowOf(r0)) && /fleet radio/.test(await rowOf(r0)), await rowOf(r0));
    check(`${vp.label}: robot rows show battery from the board`, /14%/.test(await rowOf(r0)) && /82%/.test(await rowOf(r1)), `${await rowOf(r0)} / ${await rowOf(r1)}`);
    check(`${vp.label}: a battery under the reserve is flagged`, await page.eval(`!!document.querySelector('#fleet .robot[data-id="${r0}"] .batt.low') && !document.querySelector('#fleet .robot[data-id="${r1}"] .batt.low')`));
    check(`${vp.label}: the alarm chip counts active alarms`, (await visible('#alarms')) && (await $text('#alarms-label')) === '1 alarm' && await page.eval(`document.getElementById('alarms').classList.contains('on')`), await $text('#alarms-label'));
    await click('#alarms'); await sleep(150);
    check(`${vp.label}: the alarm list puts the active one first`, (await page.eval(`[...document.querySelectorAll('#alarm-list .al')].map((e) => e.dataset.alarm).join(',')`)) === '1,2');
    await page.eval(`window.__published.length = 0`);
    await click('#alarm-list [data-ack="2"]');
    const ack = JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).find((x) => x.topic === '/fleet/alarm_ack');
    check(`${vp.label}: acknowledging publishes the alarm id`, !!ack && JSON.parse(ack.msg.data).id === 2, JSON.stringify(ack));
    await page.eval(`document.getElementById('tasks').dispatchEvent(new PointerEvent('pointerdown', { bubbles: true }))`); await sleep(100);
    check(`${vp.label}: clicking elsewhere closes the alarm list`, !(await visible('#alarm-pop')));
    await shot(`${vp.label}-fleet-alarms`);

    check(`${vp.label}: with the fleet up, auto means the auction`, (await autoLabel()) === 'Any robot (fleet auction)', await autoLabel());
    await page.eval(`document.getElementById('newtask').open = true; document.querySelector('#nt-kind [data-k="pick"]').click()`);
    await page.eval(`document.getElementById('nt-who').value = 'auto'; document.getElementById('nt-from').value = 'A'; document.getElementById('nt-to').value = 'B'`);
    await page.eval(`window.__published.length = 0; document.getElementById('task-form').requestSubmit()`);
    const fpub = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
    const req = fpub.find((x) => x.topic === '/fleet/task_request');
    check(`${vp.label}: an auto pick & drop is requested from the fleet, not dispatched`,
          !!req && JSON.parse(req.msg.data).pickup === 'A' && JSON.parse(req.msg.data).dropoff === 'B' && !fpub.some((x) => /dispatch_task$/.test(x.topic)), JSON.stringify(fpub));
    check(`${vp.label}: the request waits as sent to the fleet`, /sent to the fleet/.test(await $text('#tasks')) && /bidding/.test(await $text('#tasks')), (await $text('#tasks')).slice(0, 160));
    await emit('/fleet/task_board', fboard([{ task_id: 4242, pickup: 'A', dropoff: 'B', state: 'bidding', owner: null, committed: false, age_s: 0.5 }]));
    await sleep(200);
    check(`${vp.label}: the board names the pending row`, await page.eval(`window.__operator.tasks.some((t) => t.id === 'fleet-4242' && t.state === 'BIDDING' && !t.pending) && !window.__operator.tasks.some((t) => t.pending)`));
    check(`${vp.label}: an unclaimed fleet task belongs to the fleet`, await page.eval(`[...document.querySelectorAll('#tasks .tk')].some((e) => /fleet-4242/.test(e.textContent) && e.querySelector('.who').textContent === 'Fleet')`));
    await emit('/fleet/task_board', fboard([{ task_id: 4242, pickup: 'A', dropoff: 'B', state: 'claimed', owner: r1, committed: false, age_s: 2.0 }]));
    await sleep(200);
    check(`${vp.label}: a claim assigns it to the winner`, await page.eval(`(() => { const t = window.__operator.tasks.find((t) => t.id === 'fleet-4242'); return t.state === 'CLAIMED' && t.robot === ${JSON.stringify(r1)}; })()`));
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: 'fleet-4242', state: 'NAV_TO_DROPOFF', queue_depth: 0, queue: [],
      task: { task_id: 'fleet-4242', kind: 'pickup', pickup: 'A', dropoff: 'B' } }) });
    await emit('/fleet/task_board', fboard([{ task_id: 4242, pickup: 'A', dropoff: 'B', state: 'claimed', owner: r1, committed: true, age_s: 9.0 }]));
    await sleep(200);
    check(`${vp.label}: the winner's own status takes over, loaded once committed`, await page.eval(`window.__operator.tasks.find((t) => t.id === 'fleet-4242').state === 'NAV_TO_DROPOFF'`) && /fleet-4242 · loaded/.test(await $text('#tasks')), (await $text('#tasks')).slice(0, 200));
    await page.eval(`window.__published.length = 0`);
    await click('#tasks [data-cancel="fleet-4242"]');
    const fpub2 = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
    const fc = fpub2.find((x) => x.topic === '/fleet/task_cancel');
    check(`${vp.label}: cancelling a fleet task withdraws it at the gateway`, !!fc && JSON.parse(fc.msg.data).task_id === 4242 && !fpub2.some((x) => /\/cancel_task$/.test(x.topic)), JSON.stringify(fpub2));
    await emit('/fleet/task_board', fboard([{ task_id: 4242, pickup: 'A', dropoff: 'B', state: 'cancelled', owner: r1, committed: true, age_s: 10.0 }]));
    await sleep(200);
    check(`${vp.label}: the board's cancel finishes the row`, await page.eval(`window.__operator.tasks.find((t) => t.id === 'fleet-4242').state === 'CANCELLED'`));
    await emit('/fleet/task_board', fboard([]));   // leave a quiet, healthy fleet for what follows
    await sleep(200);
    check(`${vp.label}: no alarms, no chip`, !(await visible('#alarms')));

    // -- renaming a waypoint ------------------------------------------------------------------------------
    await pickStop('B'); // a draft stop follows the rename
    await click('#wps [data-ren="B"]');
    check(`${vp.label}: the pencil turns the chip into a text field`, await page.eval(`document.activeElement.tagName === 'INPUT' && document.activeElement.value === 'B'`));
    await page.eval(`(() => { const i = document.activeElement; i.value = 'A'; i.dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true })); })()`);
    await sleep(100);
    check(`${vp.label}: a duplicate name is refused`, (await page.eval(`window.__navWps.map((w) => w.name).join(',')`)) === 'A,B');
    await click('#wps [data-ren="B"]');
    await page.eval(`(() => { const i = document.activeElement; i.value = 'Dock'; i.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true })); })()`);
    await sleep(100);
    check(`${vp.label}: Escape cancels a rename`, (await page.eval(`window.__navWps.map((w) => w.name).join(',')`)) === 'A,B' && !(await page.eval(`!!document.querySelector('#wps input')`)));
    await click('#wps [data-ren="B"]');
    await page.eval(`(() => { const i = document.activeElement; i.value = '  Dock 1 '; i.dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true })); })()`);
    await sleep(150);
    const renamed = JSON.parse(await page.eval(`JSON.stringify(window.__navWps)`));
    check(`${vp.label}: Enter saves the new name, trimmed, keeping the pose`, renamed[1].name === 'Dock 1' && renamed[1].x === parsed[1].x && renamed.length === 2, JSON.stringify(renamed[1]));
    check(`${vp.label}: the chip and the menus show the new name`, await page.eval(`!!document.querySelector('#wps [data-go="Dock 1"]') && [...document.getElementById('nt-to').options].some((o) => o.value === 'Dock 1')`));
    check(`${vp.label}: the draft route follows the rename`, (await stops()) === 'Dock 1', await stops());
    await pickStop('A'); await pickStop('Dock 1'); await pickStop('A'); await pickStop('Dock 1');
    const fit = await page.eval(`(() => { const a = document.getElementById('right'); a.scrollTop = a.scrollHeight;
      const b = document.getElementById('mv-go').getBoundingClientRect(), r = a.getBoundingClientRect();
      return { btnBottom: Math.round(b.bottom), panelBottom: Math.round(r.bottom), scroll: a.scrollHeight > a.clientHeight }; })()`);
    check(`${vp.label}: the whole queue menu can be scrolled into view`, fit.btnBottom <= fit.panelBottom - 8, JSON.stringify(fit));
    await shot(`${vp.label}-queue`);
    await page.eval(`document.getElementById('right').scrollTop = 0`);
    for (let i = 0; i < 5; i++) await click('#mv-stops [data-rm="0"]');

    // -- deleting a waypoint ----------------------------------------------------------------------------
    await click('#wps [data-del="Dock 1"]');
    await sleep(100);
    check(`${vp.label}: a waypoint can be deleted`, (await page.eval(`window.__navWps.length`)) === 1);

    // -- stop all ---------------------------------------------------------------------------------------
    await page.eval(`window.__published.length = 0; window.__services.length = 0`);
    await click('#estop');
    const stopped = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
    check(`${vp.label}: Stop all stops exploring on every robot`, roster.every((id) => stopped.some((p) => p.topic === `/${id}/explore_cmd` && p.msg.data === 'stop')));
    check(`${vp.label}: Stop all zeroes every robot's velocity`, roster.every((id) => stopped.some((p) => p.topic === `/${id}/cmd_vel` && p.msg.linear.x === 0)));
    const svc = JSON.parse(await page.eval(`JSON.stringify(window.__services)`));
    check(`${vp.label}: Stop all cancels navigation goals`, roster.every((id) => svc.some((s) => s.name === `/${id}/navigate_to_pose/_action/cancel_goal`)));
    check(`${vp.label}: with no session, Stop all only halts the robots (no dialog)`, !(await visible('#stop-dlg')));

    // -- starting up: progress bar ---------------------------------------------------------------------------
    await page.eval(`window.__fakeSession = { state: 'starting', phase: 'sim_loading', phase_label: 'loading the warehouse', session_id: 's0', scenario: 'custom', config: { map: 'warehouse', mode: 'readonly', robots: 2 } }`);
    await sleep(3400);
    const bar = () => page.eval(`document.querySelector('#sp-boot .bar i').style.getPropertyValue('--p')`);
    const p1 = parseInt(await bar(), 10);
    check(`${vp.label}: starting shows a progress bar at the current phase`, await page.eval(`!document.getElementById('sp-boot').hidden`) && p1 > 20 && p1 < 50 && (await $text('#sp-boot .boot-label')) === 'Loading the warehouse', `${p1}% ${await $text('#sp-boot .boot-label')}`);
    check(`${vp.label}: while a session runs the panel hides the form`, await page.eval(`getComputedStyle(document.getElementById('launch-form')).display === 'none'`));
    await page.eval(`window.__fakeSession.phase = 'ros_starting'`);
    await sleep(1400);
    check(`${vp.label}: the bar moves forward with the phase`, parseInt(await bar(), 10) > p1, `${await bar()}`);
    check(`${vp.label}: while starting, the panel shows only the progress text`, !(await visible('#session-state')) && !(await visible('#sp-status')));

    // -- a premapped session in the header -------------------------------------------------------------------
    await page.eval(`window.__fakeSession = { state: 'running', phase: 'running', phase_label: 'running', session_id: 's1', scenario: 'library_localize', config: { map: 'warehouse', mode: 'readonly', robots: 2 } }`);
    await sleep(3400);
    check(`${vp.label}: running session shows in the header`, (await $text('#s-label')) === 'Session running', await $text('#s-label'));
    check(`${vp.label}: a read-only saved map reads as Premapped on the session chip`, (await page.eval(`document.getElementById('session-chip').title`)).startsWith('Premapped · warehouse'), await page.eval(`document.getElementById('session-chip').title`));
    check(`${vp.label}: no mode tile in the header any more`, !(await page.eval(`!!document.getElementById('mode-tile')`)));
    check(`${vp.label}: the view toggle sits in the header`, await page.eval(`!!document.querySelector('header #view')`));
    check(`${vp.label}: running but robots silent still shows the last stretch`, (await $text('#sp-boot .boot-label')).startsWith('Waiting for the robots'), await $text('#sp-boot .boot-label'));
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5)); await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
    await sleep(400);
    check(`${vp.label}: the bar is gone once the robots are up`, await page.eval(`document.getElementById('sp-boot').hidden`));
    check(`${vp.label}: once running, a short status replaces the bar`, /^Running · /.test(await page.eval(`document.getElementById('sp-status').textContent`)) && await page.eval(`!document.getElementById('sp-status').hidden`));

    // -- Stop all ends the sim: save or discard the map ----------------------------------------------------
    await page.eval(`window.__fakeSession = { ...window.__fakeSession, read_only: false, map_name: 'warehouse' }`);
    await sleep(3400);
    await page.eval(`window.__published.length = 0; window.__sessionPosts.length = 0`);
    await click('#estop');
    await sleep(100);
    check(`${vp.label}: Stop all still halts every robot first`, await page.eval(`${JSON.stringify(roster)}.every((id) => window.__published.some((p) => p.topic === '/' + id + '/cmd_vel'))`));
    check(`${vp.label}: ... then asks whether to keep what a mapping run mapped`, await visible('#stop-dlg') && (await $text('#sd-title')) === 'Keep what this run mapped?' &&
      (await page.eval(`document.getElementById('sd-name').value`)) === 'warehouse' && await visible('#sd-save') && await visible('#sd-discard'));
    await shot(`${vp.label}-stop-dialog`);
    await page.eval(`document.getElementById('sd-name').value = 'Bad Name'`);
    await click('#sd-save');
    await sleep(100);
    check(`${vp.label}: a bad map name is refused in the dialog`, await visible('#sd-err') && (await page.eval(`window.__sessionPosts.length`)) === 0);
    await page.eval(`document.getElementById('sd-name').value = 'warehouse-v2'`);
    await click('#sd-save');
    await sleep(200);
    let sposts = JSON.parse(await page.eval(`JSON.stringify(window.__sessionPosts)`));
    check(`${vp.label}: Save & stop saves under that name and ends the session`, sposts.length === 1 && sposts[0].url === '/api/session/finish' && sposts[0].body.name === 'warehouse-v2' && !(await visible('#stop-dlg')), JSON.stringify(sposts));
    await click('#estop'); await sleep(100);
    await click('#sd-discard'); await sleep(200);
    sposts = JSON.parse(await page.eval(`JSON.stringify(window.__sessionPosts)`));
    check(`${vp.label}: Discard & stop ends the session without saving`, sposts.length === 2 && sposts[1].url === '/api/session/stop');
    await page.eval(`window.__fakeSession = { ...window.__fakeSession, read_only: true }`);
    await sleep(3400);
    await click('#estop'); await sleep(100);
    check(`${vp.label}: a read-only run just asks to stop the sim`, (await $text('#sd-title')) === 'Stop the sim?' && !(await visible('#sd-name-row')) && !(await visible('#sd-save')) && /unchanged/.test(await $text('#sd-body')));
    await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
    await sleep(100);
    check(`${vp.label}: Escape keeps it running`, !(await visible('#stop-dlg')) && (await page.eval(`window.__sessionPosts.length`)) === 2);
    await page.eval(`window.__fakeSession = { ...window.__fakeSession, read_only: undefined, map_name: undefined }`);
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5)); await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1)); // those waits outlasted ONLINE_TTL_MS
    // A demo: true session (the Fleet demo preset) starts the task demo by itself, once.
    await page.eval(`window.__published.length = 0; window.__fakeSession = { ...window.__fakeSession, session_id: 's-demo', config: { ...window.__fakeSession.config, demo: true } }`);
    await sleep(3600);
    check(`${vp.label}: a demo session runs the task demo by itself`, await page.eval(`localStorage.getItem('sortbots.autoDemo.s-demo') === '1'`));
    check(`${vp.label}: ...and only once per session`, await page.eval(`(() => { const n = document.getElementById('tk-demo').textContent; return n === 'Run demo' || n === 'Stop demo'; })()`));
    if ((await $text('#tk-demo')) === 'Stop demo') await click('#tk-demo');
    await page.eval(`window.__fakeSession = { ...window.__fakeSession, state: 'stopped', phase: 'running' }`);
    await sleep(3400);
    await click('#session-chip'); await sleep(300);
    check(`${vp.label}: a stopped session leaves just the form`, await visible('#launch-form') && !(await visible('#session-pane')));
    await page.eval(`window.__fakeSession = { ...window.__fakeSession, state: 'failed', error: 'Isaac Sim failed to start' }`);
    await sleep(3400);
    check(`${vp.label}: a failed session shows its status and log`, await visible('#session-pane') && await visible('#session-state') && await page.eval(`document.getElementById('sp-log').open`));
    await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
    await page.eval(`window.__fakeSession = null`);
  }


  // -- comms overlay ---------------------------------------------------------------------------
  // operator_comms.js against the REAL commsim model in this serve.py (no stub): the
  // robots' poses, the map's obstacles and the gateway pin go in; links, routes and
  // what-ifs come out, drawn on the map only.
  console.log('\n--- comms overlay ---');
  const load = async () => { await page.send('Page.navigate', { url: `http://127.0.0.1:${PORT}/operator.html` }); await sleep(1200); };
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await page.eval(`localStorage.clear()`); await load();
  const vz = async () => JSON.parse(await page.eval(`JSON.stringify(window.__viz.state())`));
  // 30 x 20 m floor (border wall) plus a free-standing wall between the two robots.
  const walled = grid(60, 40, 0.5, -10, -10);
  for (let row = 8; row < 32; row++) { walled.data[row * 60 + 21] = 100; walled.data[row * 60 + 22] = 100; }
  await emit('/map', walled);
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
  await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
  await sleep(500);
  check('comms: off by default, no overlay, no model stream', !(await visible('#comms')) && !(await visible('#comms-bar')) && !(await vz()).on);
  await page.eval(`document.querySelector('#tools [data-t="gw"]').click()`);
  await mapPointer(0.5, 8);
  await waitFor(async () => (await page.eval(`JSON.stringify(window.__operator.gateway)`)) !== 'null', { what: 'gateway pin save', tries: 20 });
  const gwPin = JSON.parse(await page.eval(`JSON.stringify(window.__operator.gateway)`));
  check('comms: the gateway tool drops a pin and saves it', Math.abs(gwPin.x - 0.5) < 0.2 && Math.abs(gwPin.y - 8) < 0.2, JSON.stringify(gwPin));
  const saved = await (await fetch(`http://127.0.0.1:${PORT}/api/comms_gateway`)).json();
  check('comms: serve.py keeps the pin per platform', saved.platform === 'sim' && Math.abs(saved.gateway.x - 0.5) < 0.2, JSON.stringify(saved));
  await page.eval(`document.querySelector('#tools [data-t="select"]').click()`);
  await click('#comms-toggle');
  await waitFor(async () => { const s = await vz(); return s.connected && s.n === 3 && s.racks >= 2 && s.origin; }, { what: 'the comms model to take the fleet', tries: 60 });
  let cs = await vz();
  check('comms: toggling on shows the overlay and its bar', (await visible('#comms')) && (await visible('#comms-bar')) && cs.on);
  check('comms: the model gets one node per robot plus the gateway', cs.n === 3, `n=${cs.n}`);
  check('comms: the map\'s obstacles become the model\'s racks', cs.racks === 2, `${cs.racks}`);
  check('comms: the bar only speaks up when there is something to do', (await $text('#cm-state')) === '', await $text('#cm-state'));
  await waitFor(async () => { const s = await vz(); return Math.abs(s.nodes[1][0] - 7) < 0.3 && Math.abs(s.nodes[0][0] - 10.5) < 0.3; }, { what: 'model positions', tries: 30 });
  cs = await vz();
  check('comms: the model\'s robots stand where the real ones are (map origin removed)',
        Math.abs(cs.nodes[1][0] - 7) < 0.3 && Math.abs(cs.nodes[1][1] - 12) < 0.3 && Math.abs(cs.nodes[2][0] - 14) < 0.3 && Math.abs(cs.nodes[2][1] - 8) < 0.3, JSON.stringify(cs.nodes));
  check('comms: the gateway pin is the model\'s gateway', Math.abs(cs.nodes[0][0] - 10.5) < 0.3 && Math.abs(cs.nodes[0][1] - 18) < 0.3, JSON.stringify(cs.nodes[0]));
  // The default range is 8 m; robot_1 here is 10.6 m from the gateway. Set what the
  // routing checks below assume rather than lean on the default.
  check('comms: the model starts at 8 m open-air range', Math.round((await vz()).range.robot) === 8, JSON.stringify((await vz()).range));
  await page.eval(`window.__viz.cmd({ op: 'range', metres: 22 })`);
  await waitFor(async () => { const s = await vz(); return s.routes[1] && s.routes[1][0] && s.routes[2] && s.routes[2][0] && s.airCount > 0; }, { what: 'model routes', tries: 40 });
  check('comms: every robot has a predicted route to the gateway, and messages fly', true);
  const atWorld = (x, y) => page.eval(`(() => { const r = document.getElementById('comms').getBoundingClientRect(); const [a, b] = window.__operator.toPx(${x}, ${y}); return [r.left + a, r.top + b]; })()`);
  const tap = async ([x, y]) => {
    await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x, y, button: 'left', clickCount: 1 });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x, y, button: 'left', clickCount: 1 });
  };
  await click('#cm-send');
  await tap(await atWorld(-3, 2)); await tap(await atWorld(4, -2));
  await waitFor(async () => (await vz()).feed.some((t) => /Robot 1 got the message from Robot 0/.test(t)), { what: 'a message A -> B', tries: 40 });
  check('comms: send A->B delivers a message through the model and says how', /hop/.test((await vz()).feed.find((t) => /got the message/.test(t))), (await vz()).feed.join(' | '));
  await click('#cm-link');
  await tap(await atWorld(-3, 2)); await tap(await atWorld(0.5, 8));
  await waitFor(async () => (await vz()).blocked.includes('0-1'), { what: 'a pulled link', tries: 30 });
  check('comms: pulling a link blocks it in the model', (await vz()).blocked.includes('0-1'));
  await click('#cm-show-range'); await sleep(150);
  check('comms: show range turns on the range circles, sized by the model', (await vz()).showRange && (await vz()).range.robot > 5, JSON.stringify((await vz()).range));
  await shot('comms-range');
  await click('#cm-show-range');
  await page.eval(`window.__viz.cmd({ op: 'link', a: 0, b: 1, action: 'unblock' })`);
  await waitFor(async () => !(await vz()).blocked.includes('0-1'), { what: 'a restored link', tries: 30 });
  await emit('/tf', tf('map', `${r1}/base_link`, 6, -2, 3.1));
  await waitFor(async () => Math.abs((await vz()).nodes[2][0] - 16) < 0.3, { what: 'a pose update to move the model', tries: 30 });
  check('comms: a robot that drives moves in the model too', true);
  await click('#cm-kill');
  check('comms: a what-if asks you to pick a robot', (await vz()).pick === 'node' && /switch it off/.test(await $text('#cm-hint')), await $text('#cm-hint'));
  const node2 = await page.eval(`(() => { const r = document.getElementById('comms').getBoundingClientRect(); const [a, b] = window.__operator.toPx(6, -2); return [r.left + a, r.top + b]; })()`);
  await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x: node2[0], y: node2[1], button: 'left', clickCount: 1 });
  await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: node2[0], y: node2[1], button: 'left', clickCount: 1 });
  await waitFor(async () => (await vz()).dead.includes(2), { what: 'model kill', tries: 30 });
  check('comms: switching a robot off kills it in the model', (await vz()).dead.includes(2) && !(await vz()).pick);
  const row1 = await page.eval(`document.querySelector('#fleet .robot[data-id="${r1}"] .l1').textContent`);
  check('comms: a model what-if never touches the real robot\'s card', !/silent|offline/.test(row1), row1);
  await page.eval(`window.__viz.cmd({ op: 'node', id: 2, action: 'revive' })`);
  await waitFor(async () => !(await vz()).dead.includes(2), { what: 'model revive', tries: 30 });
  await shot('comms-overlay');
  await page.eval(`localStorage.setItem('sortbots.comms', 'on')`); await load();
  await emit('/map', walled);
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
  check('comms: the toggle survives a reload', (await page.eval(`document.getElementById('comms-toggle').getAttribute('aria-checked')`)) === 'true' && (await vz()).on);
  await click('#view [data-v="3d"]'); await sleep(200);
  check('comms: the 3D view turns the overlay off', !(await vz()).on && !(await visible('#comms-toggle')));
  await click('#view [data-v="2d"]'); await sleep(200);
  check('comms: back to 2D brings it back', (await vz()).on);
  await click('#comms-toggle'); await sleep(300);
  check('comms: toggling off closes the model stream', !(await vz()).on && !(await vz()).connected && !(await visible('#comms')));
  await page.eval(`localStorage.clear()`);

  // -- teleport (sim only) ------------------------------------------------------------------------
  console.log('\n--- teleport ---');
  await page.eval(`localStorage.clear()`); await load();
  await emit('/map', walled);
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
  await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
  await sleep(600);
  check('teleport: the default tool is called Drive', /Drive/.test(await $text('#tools [data-t="select"]')), await $text('#tools [data-t="select"]'));
  check('teleport: the tool shows on the sim', await visible('#tools [data-t="tp"]'));
  const dragWorld = async (x0, y0, x1, y1) => {
    const at = (x, y) => page.eval(`(() => { const r = document.getElementById('map').getBoundingClientRect(); const [a, b] = window.__operator.toPx(${x}, ${y}); return [r.left + a, r.top + b]; })()`);
    const [ax, ay] = await at(x0, y0), [bx, by] = await at(x1, y1);
    await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x: ax, y: ay, button: 'left', clickCount: 1 });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: (ax + bx) / 2, y: (ay + by) / 2, button: 'left' });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: bx, y: by, button: 'left' });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: bx, y: by, button: 'left', clickCount: 1 });
  };
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 't' }))`);
  check('teleport: T picks the tool', await page.eval(`document.querySelector('#tools [data-t="tp"]').classList.contains('active')`));
  await page.eval(`window.__published.length = 0; window.__services.length = 0; window.__ops.length = 0`);
  await dragWorld(4, -2, 7, 4);
  const tpPub = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
  const tpMsg = tpPub.find((x) => x.topic === '/sim/teleport');
  const tpBody = tpMsg && JSON.parse(tpMsg.msg.data);
  check('teleport: dropping a robot asks the sim to move it there, in map coordinates, keeping its heading',
        !!tpBody && tpBody.robot === r1 && Math.abs(tpBody.x - 7) < 0.2 && Math.abs(tpBody.y - 4) < 0.2 && Math.abs(tpBody.yaw - 3.1) < 0.05, JSON.stringify(tpBody));
  check('teleport: it never sends a nav goal or a task', !tpPub.some((x) => /dispatch_task/.test(x.topic)) && !(await page.eval(`window.__ops.some((o) => o.op === 'send_action_goal')`)));
  check('teleport: the robot is stopped first (goals cancelled, explorer told to stop)',
        tpPub.some((x) => x.topic === `/${r1}/explore_cmd` && x.msg.data === 'stop') && (await page.eval(`window.__services.some((s) => s.name === '/${r1}/navigate_to_pose/_action/cancel_goal')`)));
  await page.eval(`window.__published.length = 0`);
  await dragWorld(-3, 2, 1.0, 0);   // onto the free-standing wall
  check('teleport: an occupied spot is refused', !JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).some((x) => x.topic === '/sim/teleport') && /occupied/i.test(await $text('#toast')), await $text('#toast'));
  await page.eval(`window.__platform = 'real'; window.__operator.loadWaypoints()`); await sleep(300);
  check('teleport: not offered on the real robot, and the tool falls back to Drive',
        !(await visible('#tools [data-t="tp"]')) && await page.eval(`document.querySelector('#tools [data-t="select"]').classList.contains('active')`));
  await page.eval(`window.__platform = null; window.__operator.loadWaypoints()`); await sleep(200);

  // -- the Sim view (Isaac's own render, only while watched) -------------------------------------
  console.log('\n--- sim view ---');
  check('sim view: offered on the sim', await visible('#view [data-v="sim"]'));
  const beats = () => page.eval(`window.__published.filter((x) => x.topic === '/sim/view_cmd').map((x) => JSON.parse(x.msg.data).on)`);
  await page.eval(`window.__published.length = 0`);
  await click('#view [data-v="sim"]'); await sleep(300);
  check('sim view: opening it shows the sim picture instead of the map',
        (await visible('#simview')) && !(await visible('#map')) && !(await visible('#comms-toggle')) && !(await visible('#tools')));
  check('sim view: opening it asks Isaac to render', JSON.stringify(await beats()) === '[true]', JSON.stringify(await beats()));
  check('sim view: says it is waiting for the first frame', /Starting the sim camera/.test(await $text('#simview .msg')), await $text('#simview .msg'));
  await sleep(2300);
  check('sim view: keeps asking while open (a heartbeat, so a closed tab stops costing)', (await beats()).filter((b) => b).length >= 2, JSON.stringify(await beats()));
  // Steering: Isaac reports the camera on /sim/ground_truth; the page orbits/pans/zooms from it.
  await emit('/sim/ground_truth', { data: JSON.stringify({ t: 1, robots: {}, actors: {}, view: { on: true, follow: null, eye: [2, 5, 6.5], target: [-3, 13.5, 0] } }) });
  await sleep(200);
  const moves = () => page.eval(`window.__published.filter((x) => x.topic === '/sim/view_cmd').map((x) => JSON.parse(x.msg.data)).filter((m) => m.eye)`);
  const svAt = await page.eval(`(() => { const r = document.getElementById('simview').getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; })()`);
  const dist = (m) => Math.hypot(m.eye[0] - m.target[0], m.eye[1] - m.target[1], m.eye[2] - m.target[2]);
  const dragSv = async (dx, dy, mods = 0) => {
    await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x: svAt[0], y: svAt[1], button: 'left', clickCount: 1, modifiers: mods });
    for (let i = 1; i <= 6; i++) { await page.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: svAt[0] + dx * i / 6, y: svAt[1] + dy * i / 6, button: 'left', modifiers: mods }); await sleep(30); }
    await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: svAt[0] + dx, y: svAt[1] + dy, button: 'left', clickCount: 1, modifiers: mods });
    await sleep(250);
  };
  await page.eval(`window.__published.length = 0`);
  await dragSv(120, 0);
  let mv = await moves(), last = mv[mv.length - 1];
  check('sim view: dragging orbits the camera about its target (same distance, same target)',
        !!last && Math.abs(dist(last) - Math.hypot(5, 8.5, 6.5)) < 0.05 && Math.abs(last.target[0] + 3) < 0.01 && Math.abs(last.eye[0] - 2) > 1, JSON.stringify(last));
  check('sim view: camera moves are throttled, not one per mouse event', mv.length >= 1 && mv.length <= 5, String(mv.length));
  await page.eval(`window.__published.length = 0`);
  await page.send('Input.dispatchMouseEvent', { type: 'mouseWheel', x: svAt[0], y: svAt[1], deltaX: 0, deltaY: -400 }); await sleep(250);
  mv = await moves(); last = mv[mv.length - 1];
  check('sim view: scrolling zooms in (shorter distance, same target)', !!last && dist(last) < 11.0 && Math.abs(last.target[1] - 13.5) < 0.01, JSON.stringify(last));
  await page.eval(`window.__published.length = 0`);
  await dragSv(0, 80, 8 /* shift */);
  mv = await moves(); last = mv[mv.length - 1];
  check('sim view: Shift-drag pans (the target moves, the distance does not)', !!last && Math.hypot(last.target[0] + 3, last.target[1] - 13.5) > 0.3 && Math.abs(last.target[2]) < 1e-6, JSON.stringify(last));
  await page.eval(`window.__published.length = 0`);
  await page.eval(`(() => { const s = document.getElementById('sv-follow'); s.value = ${JSON.stringify(r1)}; s.dispatchEvent(new Event('change')); })()`);
  check('sim view: Follow asks Isaac to ride along with a robot', await page.eval(`window.__published.some((x) => x.topic === '/sim/view_cmd' && JSON.parse(x.msg.data).follow === ${JSON.stringify(r1)})`));
  await click('#sv-reset');
  check('sim view: Reset view asks for the default camera', await page.eval(`window.__published.some((x) => x.topic === '/sim/view_cmd' && JSON.parse(x.msg.data).reset === true)`) && (await page.eval(`document.getElementById('sv-follow').value`)) === '');
  await shot('sim-view');
  await click('#view [data-v="2d"]'); await sleep(200);
  check('sim view: leaving tells Isaac to stop', (await beats()).slice(-1)[0] === false && (await visible('#map')) && !(await visible('#simview')));
  const n0 = (await beats()).length; await sleep(2300);
  check('sim view: and the heartbeat stops', (await beats()).length === n0);
  await click('#view [data-v="sim"]'); await sleep(200);
  await load();
  await sleep(600);
  check('sim view: the choice survives a reload once the platform is known', (await page.eval(`window.__operator.stageView`)) === 'sim' && await visible('#simview'));
  await page.eval(`window.__platform = 'real'; window.__operator.loadWaypoints()`); await sleep(300);
  check('sim view: not offered on the real robot (and falls back to the map)', !(await visible('#view [data-v="sim"]')) && (await page.eval(`window.__operator.stageView`)) === '2d');
  await page.eval(`window.__platform = null; window.__operator.loadWaypoints(); localStorage.removeItem('sortbots.mapView')`); await sleep(200);
  await emit('/map', walled);
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
  await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
  await sleep(300);

  // -- manual drive (pad on the camera tile) ----------------------------------------------------------
  console.log('\n--- manual drive ---');
  await page.eval(`window.__operator.select(${JSON.stringify(r1)})`); await sleep(400);
  check('drive: an open robot card has a drive pad on its camera', await page.eval(`!!document.querySelector('#fleet .robot[data-id="${r1}"] .cam .drive [data-drive="fwd"]')`));
  const twists = () => page.eval(`window.__published.filter((x) => x.topic === '/${r1}/cmd_vel').map((x) => [x.msg.linear.x, x.msg.linear.y, x.msg.angular.z])`);
  const padBtn = (cmd) => page.eval(`(() => { const r = document.querySelector('#fleet .robot[data-id="${r1}"] .cam .drive [data-drive="${cmd}"]').getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; })()`);
  await page.eval(`window.__published.length = 0`);
  const [fx, fy] = await padBtn('fwd');
  await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x: fx, y: fy, button: 'left', clickCount: 1 });
  await sleep(350);
  const held = await twists();
  check('drive: holding forward publishes 0.5 m/s, repeatedly', held.length >= 3 && held.every((t) => t[0] === 0.5 && t[1] === 0 && t[2] === 0), JSON.stringify(held));
  check('drive: the held button lights up', await page.eval(`document.querySelector('#fleet .robot[data-id="${r1}"] .drive [data-drive="fwd"]').classList.contains('held')`));
  await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: fx, y: fy, button: 'left', clickCount: 1 });
  await sleep(250);
  const afterUp = await twists(), n = afterUp.length;
  check('drive: letting go stops it at once', JSON.stringify(afterUp[n - 1]) === '[0,0,0]', JSON.stringify(afterUp.slice(-2)));
  await sleep(300);
  check('drive: and nothing more is sent once stopped', (await twists()).length === n);
  check('drive: the pad never opens or closes the card', await page.eval(`window.__operator.open.has(${JSON.stringify(r1)})`));
  await page.eval(`window.__published.length = 0`);
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'w' })); window.dispatchEvent(new KeyboardEvent('keydown', { key: 'a' }))`);
  await sleep(150);
  check('drive: W+A drives the robot in control diagonally', (await twists()).some((t) => t[0] === 0.5 && t[1] === 0.5), JSON.stringify(await twists()));
  await page.eval(`window.dispatchEvent(new Event('blur'))`); await sleep(100);
  check('drive: losing focus mid-drive stops the robot', JSON.stringify((await twists()).slice(-1)[0]) === '[0,0,0]');
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keyup', { key: 'w' })); window.dispatchEvent(new KeyboardEvent('keyup', { key: 'a' }))`);
  // Taking the wheel from an exploring robot stops the explorer first.
  await emit(`/${r1}/explore_status`, { data: JSON.stringify({ state: 'exploring' }) }); await sleep(300);
  await page.eval(`window.__published.length = 0`);
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'e' }))`); await sleep(150);
  const pubs = JSON.parse(await page.eval(`JSON.stringify(window.__published)`));
  check('drive: taking the wheel stops the explorer, then turns', pubs.some((x) => x.topic === `/${r1}/explore_cmd` && x.msg.data === 'stop') && (await twists()).some((t) => t[2] < 0));
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keyup', { key: 'e' }))`);
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'w', bubbles: true }))`);
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keyup', { key: 'w' }))`);
  await shot('drive-pad');
  await page.eval(`window.__operator.select(null)`);

  // -- phones ---------------------------------------------------------------------------------------
  console.log('\n--- phone ---');
  await page.send('Emulation.setDeviceMetricsOverride', { width: 390, height: 844, deviceScaleFactor: 3, mobile: true });
  await page.send('Emulation.setTouchEmulationEnabled', { enabled: true, maxTouchPoints: 5 });
  await page.eval(`localStorage.clear()`); await load();
  await emit('/map', walled);
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
  await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
  await sleep(600);
  const inView = (sel) => page.eval(`(() => { const e = document.querySelector(${JSON.stringify(sel)}); if (!e) return false; const r = e.getBoundingClientRect(); return r.width > 0 && r.left >= -1 && r.right <= innerWidth + 1 && r.top >= -1 && r.bottom <= innerHeight + 1; })()`);
  check('phone: the bottom bar switches panes', await visible('#mnav'));
  check('phone: the map gets the screen by default, the lists are tucked away',
        (await page.eval(`document.getElementById('map').getBoundingClientRect().width`)) > 370 && !(await visible('#left')) && !(await visible('#right')));
  // Against the DEVICE width: a phone widens its layout viewport to fit overflowing content,
  // so innerWidth itself grows with the bug (it read 495 on a 390 px screen).
  check('phone: nothing scrolls sideways', await page.eval(`innerWidth <= 391 && document.documentElement.scrollWidth <= 391`), await page.eval(`'innerWidth ' + innerWidth + ', scrollWidth ' + document.documentElement.scrollWidth`));
  check('phone: Stop all and the session chip are always on screen', (await inView('#estop')) && (await inView('#session-chip')));
  await shot('phone-map');
  const zoom0 = await page.eval(`window.__operator.view.scale`);
  const mid = await page.eval(`(() => { const r = document.getElementById('map').getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; })()`);
  const tp2 = (dx) => [{ x: mid[0] - dx, y: mid[1], id: 1 }, { x: mid[0] + dx, y: mid[1], id: 2 }];
  await page.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: tp2(30) });
  for (const dx of [45, 60, 80]) await page.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: tp2(dx) });
  await page.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  await sleep(150);
  const zoom1 = await page.eval(`window.__operator.view.scale`);
  check('phone: pinching out zooms the map in', zoom1 > zoom0 * 2, `${zoom0.toFixed(1)} -> ${zoom1.toFixed(1)}`);
  check('phone: a pinch never turns into a send', !(await page.eval(`JSON.stringify(window.__published)`)).includes('dispatch_task') && !(await page.eval(`!!window.__operator.drag`)));
  await click('#mnav [data-pane="robots"]'); await sleep(200);
  check('phone: Robots shows the robot list full width', (await visible('#left')) && (await page.eval(`document.getElementById('left').getBoundingClientRect().width`)) > 370);
  check('phone: the bottom bar counts robots online', /2\/2/.test(await $text('#mn-robots')), await $text('#mn-robots'));
  await shot('phone-robots');
  await click('#mnav [data-pane="tasks"]'); await sleep(200);
  check('phone: Tasks shows the task list', (await visible('#right')) && !(await visible('#left')));
  await shot('phone-tasks');
  await load();
  check('phone: the pane survives a reload', await visible('#right'));
  await click('#mnav [data-pane="map"]'); await sleep(300);
  check('phone: back to the map, it is sized again', (await page.eval(`document.getElementById('map').width`)) > 370);
  await click('#session-chip'); await sleep(400);
  check('phone: the session panel fits the screen', await page.eval(`(() => { const r = document.querySelector('#session .card').getBoundingClientRect(); return r.left >= 0 && r.right <= innerWidth + 1; })()`));
  await shot('phone-session');
  await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
  // Sideways: how an operator holds a phone to drive.
  await page.send('Emulation.setDeviceMetricsOverride', { width: 844, height: 390, deviceScaleFactor: 3, mobile: true, screenOrientation: { type: 'landscapePrimary', angle: 90 } });
  await page.eval(`localStorage.clear()`); await load();
  await emit('/map', walled);
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5));
  await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
  await sleep(500);
  const box = (sel) => page.eval(`(() => { const r = document.querySelector(${JSON.stringify(sel)}).getBoundingClientRect(); return { l: r.left, t: r.top, w: r.width, h: r.height }; })()`);
  check('landscape: one header row, so the map keeps the height', (await box('header')).h < 56, JSON.stringify(await box('header')));
  check('landscape: the pane switcher is a rail on the left', (await visible('#mnav')) && (await box('#mnav')).l < 2 && (await box('#mnav')).h > 250, JSON.stringify(await box('#mnav')));
  check('landscape: the map fills the rest', (await box('#map')).w > 700 && (await box('#map')).h > 300, JSON.stringify(await box('#map')));
  check('landscape: Stop all and the session chip are on screen, nothing scrolls sideways',
        (await inView('#estop')) && (await inView('#session-chip')) && await page.eval(`innerWidth <= 845 && document.documentElement.scrollWidth <= 845`));
  const aspect = () => page.eval(`(() => { const c = document.getElementById('map'), r = c.getBoundingClientRect(); return [c.width / c.height, r.width / r.height]; })()`);
  check('landscape: the map is not stretched (pixel and screen aspect agree)', Math.abs((await aspect())[0] - (await aspect())[1]) < 0.02, JSON.stringify(await aspect()));
  await shot('phone-landscape');
  await click('#mnav [data-pane="robots"]'); await sleep(200);
  check('landscape: Robots from the rail', (await visible('#left')) && (await box('#left')).w > 700);
  check('landscape: installable full screen (manifest, home-screen meta)',
        await page.eval(`!!document.querySelector('link[rel="manifest"]') && document.querySelector('meta[name="apple-mobile-web-app-capable"]').content === 'yes'`)
        && (await (await fetch(`http://127.0.0.1:${PORT}/manifest.webmanifest`)).json()).display === 'fullscreen');
  await page.send('Emulation.setTouchEmulationEnabled', { enabled: false });
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await page.eval(`localStorage.clear()`); await load();

  // -- resizable side bars ------------------------------------------------------------------------
  console.log('\n--- side bars ---');
  await page.eval(`localStorage.clear()`); await load();
  const widths = () => page.eval(`[document.getElementById('left').getBoundingClientRect().width, document.getElementById('right').getBoundingClientRect().width, document.getElementById('map').getBoundingClientRect().width]`);
  const [l0, r0w, m0] = await widths();
  const dragX = async (sel, dx) => {
    const [x, y] = await page.eval(`(() => { const r = document.querySelector('${sel}').getBoundingClientRect(); return [r.left + r.width / 2, r.top + 200]; })()`);
    await page.send('Input.dispatchMouseEvent', { type: 'mousePressed', x, y, button: 'left', clickCount: 1 });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: x + dx / 2, y, button: 'left' });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: x + dx, y, button: 'left' });
    await page.send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: x + dx, y, button: 'left', clickCount: 1 });
  };
  await dragX('#rz-left', 120); await dragX('#rz-right', -100); await sleep(200);
  const [l1, r1w, m1] = await widths();
  check('side bars: dragging the left edge widens the robot list', Math.abs(l1 - (l0 + 120)) < 4, `${l0} -> ${l1}`);
  check('side bars: dragging the right edge widens the task list', Math.abs(r1w - (r0w + 100)) < 4, `${r0w} -> ${r1w}`);
  check('side bars: the map gives up the room', Math.abs(m1 - (m0 - 220)) < 6, `${m0} -> ${m1}`);
  await dragX('#rz-left', -600); await sleep(100);
  check('side bars: a bar never shrinks past its minimum', Math.abs((await widths())[0] - 200) < 2, String((await widths())[0]));
  await load();
  check('side bars: widths survive a reload', Math.abs((await widths())[0] - 200) < 2 && Math.abs((await widths())[1] - r1w) < 4, JSON.stringify(await widths()));
  await page.eval(`document.getElementById('rz-left').dispatchEvent(new MouseEvent('dblclick', { bubbles: true })); document.getElementById('rz-right').dispatchEvent(new MouseEvent('dblclick', { bubbles: true }))`);
  await sleep(100);
  // The stage changing size with NO window resize event (an iPhone rotation fires it
  // before the layout settles): the map must still follow, or CSS stretches it.
  await page.eval(`document.body.style.setProperty('--left-w', '420px')`); await sleep(300);
  check('side bars: the map re-fits when its pane changes size without a window resize',
        await page.eval(`(() => { const c = document.getElementById('map'), r = c.getBoundingClientRect(); return Math.abs(c.width / (window.devicePixelRatio || 1) - r.width) < 2 && Math.abs(c.height / (window.devicePixelRatio || 1) - r.height) < 2; })()`));
  await page.eval(`document.body.style.removeProperty('--left-w')`); await sleep(200);
  check('side bars: double-click resets both', Math.abs((await widths())[0] - l0) < 2 && Math.abs((await widths())[1] - r0w) < 2, JSON.stringify(await widths()));
  await page.eval(`localStorage.clear()`);

  // -- 2D map / 3D cloud -----------------------------------------------------------------------
  console.log('\n--- 3d view ---');
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await page.eval(`localStorage.clear()`);
  await page.send('Page.navigate', { url: `http://127.0.0.1:${PORT}/operator.html` }); await sleep(1200);
  check('3d: the map starts in 2D', await visible('#map') && !(await visible('#cloud')));
  await click('#view [data-v="3d"]'); await sleep(300);
  check('3d: the toggle swaps the map for the cloud', await visible('#cloud') && !(await visible('#map')) && !(await visible('#tools')));
  const webgl = await page.eval(`!!document.querySelector('#cloud canvas')`);
  check('3d: subscribes to the fleet cloud while showing (or says WebGL is missing)',
        webgl ? await page.eval(`!!window.__topics['/fleet/recon_cloud']`) : /WebGL/.test(await $text('#cloud .cloud-info')), webgl ? 'webgl' : await $text('#cloud .cloud-info'));
  if (webgl) {
    // Two points with packed rgb, base64 like rosbridge sends them.
    const buf = Buffer.alloc(32); [[1, 2, 0.5, 0xff0000], [3, 4, 1.0, 0x00ff00]].forEach(([x, y, z, c], i) => { buf.writeFloatLE(x, i * 16); buf.writeFloatLE(y, i * 16 + 4); buf.writeFloatLE(z, i * 16 + 8); buf.writeUInt32LE(c, i * 16 + 12); });
    await emit('/fleet/recon_cloud', { width: 2, height: 1, point_step: 16, fields: [{ name: 'x', offset: 0 }, { name: 'y', offset: 4 }, { name: 'z', offset: 8 }, { name: 'rgb', offset: 12 }], data: buf.toString('base64') });
    await sleep(200);
    check('3d: a cloud message is decoded and counted', /^2 points/.test(await $text('#cloud .cloud-info')), await $text('#cloud .cloud-info'));
    await shot('3d-cloud');
  }
  check('3d: the 3D button\'s tooltip explains orbiting (no instruction text on the stage)', /orbit/.test(await page.eval(`document.querySelector('#view [data-v="3d"]').title`)));
  await load();
  check('3d: the chosen view survives a reload', await visible('#cloud'));
  await click('#view [data-v="2d"]'); await sleep(200);
  check('3d: back to 2D restores the map and tools', await visible('#map') && await visible('#tools') && !(await visible('#cloud')));

  // -- light / dark: follows the system until the toggle pins a choice ------------------------
  console.log('\n--- theme ---');
  const bodyBg = () => page.eval(`getComputedStyle(document.body).backgroundColor`);
  const emulate = (scheme) => page.send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: scheme }] });
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await page.eval(`localStorage.clear()`);
  await emulate('dark'); await load();
  check('theme: dark system setting gives dark by default', (await bodyBg()) === 'rgb(15, 18, 21)', await bodyBg());
  check('theme: toggle offers the other mode', (await page.eval(`document.getElementById('theme').title`)) === 'Switch to light mode');
  await emit('/map', grid(60, 40, 0.5, -10, -10));
  await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5)); await sleep(500);
  check('theme: map canvas uses the dark palette', (await page.eval(`(() => { const c = document.getElementById('map'); const d = window.devicePixelRatio || 1; return Array.from(c.getContext('2d').getImageData(4, 4, 1, 1).data).join(','); })()`)) === '15,18,21,255');
  await shot('dark-fleet');
  await click('#theme'); await sleep(200);
  check('theme: clicking pins light', (await page.eval(`document.documentElement.dataset.theme`)) === 'light' && (await bodyBg()) === 'rgb(242, 243, 244)', await bodyBg());
  await load();
  check('theme: the pinned choice survives a reload, even against the system setting', (await bodyBg()) === 'rgb(242, 243, 244)');
  await page.eval(`localStorage.clear()`); await emulate('light'); await load();
  check('theme: light system setting gives light by default', (await bodyBg()) === 'rgb(242, 243, 244)');
  await click('#theme'); await sleep(200);
  check('theme: clicking in light mode pins dark', (await page.eval(`document.documentElement.dataset.theme`)) === 'dark' && (await bodyBg()) === 'rgb(15, 18, 21)');
  await page.eval(`window.__emit('/map', ${JSON.stringify(grid(60, 40, 0.5, -10, -10))})`);
  await click('#empty-go'); await sleep(700); await shot('dark-session-panel');
  await emulate('no-override');

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join('; '));
  page.close();
  if (SCREENSHOT_DIR) console.log(`\nscreenshots: ${SCREENSHOT_DIR}`);
  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} checks passed`);
  if (failed.length) { console.log('\nFAILED:'); for (const f of failed) console.log(`  - ${f.name}${f.detail ? `  (${f.detail})` : ''}`); }
  process.exit(failed.length ? 1 : 0);
}

main().catch((err) => { console.error('\nharness error:', err.message); cleanup(); process.exit(3); });
