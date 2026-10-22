#!/usr/bin/env node
/**
 * Headless test for the dashboard's obstacle overlay (index.html sensors view,
 * drawn from nodes/obstacle_detector.py's /<id>/obstacles JSON).
 *
 *     node webui/tests/obstacle_overlay_test.mjs [--screenshot DIR] [--port N]
 *
 * No fixture, no ROS, no sim: the same ROSLIB stub and raw-CDP harness as
 * operator_test.mjs, so it runs where dashboard_test.mjs (which needs a recorded
 * bag fixture) can't. It injects obstacle messages and reads the canvas back.
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
const PORT = Number(argVal('--port', '8203'));
const CDP_PORT = Number(argVal('--cdp-port', '9233'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-obstacle-shots'))) : null;

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



// Payload as nodes/obstacle_detector.py builds it (build_payload): one moving
// person-sized box left of centre and one static box right of it.
const payload = (stamp, extra = {}) => ({ data: JSON.stringify({
  stamp, frame: 'robot_0/base_link', world_frame: 'robot_0/odom', floor_z: 0.0,
  w: 848, h: 480, count: 2, moving: 1, ...extra,
  obstacles: [
    { uv: [200, 100, 320, 460], range_m: 2.1, speed_mps: 1.1, moving: true, track_id: 7,
      centroid: [2.2, 0.4, 0.8], min: [2.1, 0.2, 0.1], max: [2.3, 0.6, 1.7], extent_m: 0.4,
      height_m: 1.7, bearing_deg: 10, n: 900, vel: [0, -1.1], hits: 9 },
    { uv: [560, 300, 700, 420], range_m: 3.0, speed_mps: 0.0, moving: false, track_id: 3,
      centroid: [3.0, -0.6, 0.3], min: [2.9, -0.8, 0.1], max: [3.1, -0.4, 0.5], extent_m: 0.4,
      height_m: 0.5, bearing_deg: -11, n: 200, vel: [0, 0], hits: 20 },
  ],
}) });

async function main() {
  const sessionsDir = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-sessions-'));
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--platform', 'sim',
                           '--port', String(PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: { ...process.env, SORTBOTS_SESSIONS_DIR: sessionsDir } });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${PORT}/index.html`)).ok, { what: 'serve.py' });

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
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1366, height: 768, deviceScaleFactor: 1, mobile: false });
  await page.send('Page.navigate', { url: `http://127.0.0.1:${PORT}/index.html` });
  await sleep(1500);

  const emit = (name, msg) => page.eval(`window.__emit(${JSON.stringify(name)}, ${JSON.stringify(msg)})`);
  const $text = (sel) => page.eval(`(document.querySelector(${JSON.stringify(sel)}) || {}).textContent || ''`);
  // Pixels on the obstacle canvas: red-dominant (moving) and amber (static).
  const colours = () => page.eval(`(() => {
    const c = document.getElementById('obstacle-overlay');
    if (!c.width || !c.height) return { red: 0, amber: 0, any: 0 };
    const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
    let red = 0, amber = 0, any = 0;
    for (let i = 0; i < d.length; i += 4) {
      if (d[i + 3] < 128) continue;
      any++;
      if (d[i] > 200 && d[i + 1] < 110 && d[i + 2] < 110) red++;
      else if (d[i] > 200 && d[i + 1] > 140 && d[i + 1] < 210 && d[i + 2] < 90) amber++;
    }
    return { red, amber, any };
  })()`);
  const subscribed = () => page.eval(`(window.__topics['/robot_0/obstacles'] || []).length`);

  check('not subscribed outside the sensors view', (await page.eval(`stageMode`)) !== 'sensors' ? (await subscribed()) === 0 : true);
  await page.eval(`setStageMode('sensors')`);
  await sleep(300);
  check('sensors view subscribes to /robot_0/obstacles', (await subscribed()) === 1);
  check('tag waits for the detector before any message', /waiting/.test(await $text('#obstacle-tag')), await $text('#obstacle-tag'));

  await emit('/robot_0/obstacles', payload(1.0));
  await sleep(200);
  const tag = await $text('#obstacle-tag');
  check('tag counts obstacles and moving ones', tag === 'obstacles · 2 · 1 moving', tag);
  check('tag turns to an alert while something moves', await page.eval(`document.getElementById('obstacle-tag').classList.contains('alert')`));
  const c1 = await colours();
  check('moving obstacle drawn red', c1.red > 200, JSON.stringify(c1));
  check('static obstacle drawn amber', c1.amber > 100, JSON.stringify(c1));

  // The moving box sits left of centre in the image: its red must too.
  const redLeft = await page.eval(`(() => {
    const c = document.getElementById('obstacle-overlay');
    const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
    let l = 0, r = 0;
    for (let y = 0; y < c.height; y++) for (let x = 0; x < c.width; x++) {
      const i = (y * c.width + x) * 4;
      if (d[i + 3] > 128 && d[i] > 200 && d[i + 1] < 110 && d[i + 2] < 110) (x < c.width / 2 ? l++ : r++);
    }
    return l > 5 * Math.max(r, 1);
  })()`);
  check('boxes land where their image pixels are (contain-letterbox inverted)', redLeft);

  await page.eval(`document.getElementById('obstacle-toggle-box').click()`);
  await sleep(100);
  check('toggle off clears the boxes', (await colours()).any === 0);
  await page.eval(`document.getElementById('obstacle-toggle-box').click()`);
  await sleep(100);
  check('toggle on draws them again', (await colours()).red > 200);

  await emit('/robot_0/obstacles', { data: JSON.stringify({ stamp: 2, w: 848, h: 480, count: 0, moving: 0, obstacles: [] }) });
  await sleep(100);
  check('an empty frame clears the canvas', (await colours()).any === 0);
  check('tag drops the alert with nothing moving', !(await page.eval(`document.getElementById('obstacle-tag').classList.contains('alert')`)));

  await emit('/robot_0/obstacles', { data: 'not json' });
  await sleep(100);
  check('a malformed message is logged, not thrown', pageErrors.length === 0, pageErrors.join(' | '));

  await page.eval(`setStageMode('map')`);
  await sleep(200);
  check('leaving the sensors view unsubscribes', !(await page.eval(`obstaclesSubscribed`)));

  if (SCREENSHOT_DIR) {
    fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });
    await page.eval(`setStageMode('sensors')`);
    await emit('/robot_0/obstacles', payload(3.0));
    await sleep(300);
    const r = await page.send('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(path.join(SCREENSHOT_DIR, 'sensors-obstacles.png'), Buffer.from(r.result.data, 'base64'));
  }

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join(' | '));
  page.close();
  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} checks passed`);
  cleanup();
  process.exit(failed.length ? 1 : 0);
}

main().catch((e) => { console.error(e); cleanup(); process.exit(1); });
