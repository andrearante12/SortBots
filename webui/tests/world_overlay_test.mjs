#!/usr/bin/env node
/**
 * Headless test for the map's world overlay (spawn_warehouse.py --world):
 * props and scripted movers drawn from /sim/world + /sim/ground_truth.
 *
 *     node webui/tests/world_overlay_test.mjs [--port 8095] [--cdp-port 9225] [--screenshot DIR]
 *
 * No ROS, no sim, no fixture: vendor/roslib.min.js is intercepted and replaced
 * with a stub that records every subscribe() callback and publish() by topic,
 * so the test feeds /map, /sim/world and /sim/ground_truth messages straight
 * into app.js and reads the result off the map canvas's pixels. app.js is a
 * classic script, so its top-level worldToPixel/ctx are reachable from
 * Runtime.evaluate — the test asks the page itself where an actor should be.
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
const HTTP_PORT = Number(argVal('--port', '8095'));
const CDP_PORT = Number(argVal('--cdp-port', '9225'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-world-shots')))
  : null;

const ROSLIB_STUB = `
window.__subs = {};
window.__pubs = {};
window.ROSLIB = {
  Ros: function () { this.on = () => {}; this.connect = () => {}; this.close = () => {};
                     this.callOnConnection = () => {}; },
  Topic: function (o) {
    const name = o.name;
    this.subscribe = (cb) => { (window.__subs[name] = window.__subs[name] || []).push(cb); };
    this.unsubscribe = () => {};
    this.publish = (m) => { (window.__pubs[name] = window.__pubs[name] || []).push(m); };
    this.advertise = () => {};
  },
  Message: function (d) { Object.assign(this, d); },
  Service: function () { this.callService = () => {}; },
  ServiceRequest: function (d) { Object.assign(this, d); },
  TFClient: function () { this.subscribe = () => {}; this.unsubscribe = () => {}; this.dispose = () => {}; },
};
window.__feed = (name, msg) => (window.__subs[name] || []).forEach((cb) => cb(msg));
`;

// A world as spawn_warehouse.py publishes it on /sim/world: one crate (with
// its simtest.geometry footprint) and one cylinder walker.
const WORLD = {
  world: 'unit_world', title: 'Unit world',
  props: [{ id: 'crate_0', label: 'crate_0', shape: 'box', color: [0.2, 0.3, 0.9], height: 0.9,
            footprint: ['poly', [[-8.6, 3.6], [-9.4, 3.6], [-9.4, 2.4], [-8.6, 2.4]]] }],
  actors: [{ id: 'walker_0', label: 'walker', shape: 'cylinder', color: [0.95, 0.45, 0.1],
             height: 1.75, radius: 0.35, size: null, path: [[-11, 6.5], [-6, 6.5]],
             mode: 'pingpong' }],
};
const truth = (wt, x) => ({
  data: JSON.stringify({ t: 100 + wt, wt, running: true, world: 'unit_world',
    robots: { robot_0: { x: -7.15, y: 11.62, yaw: 0 } },
    actors: { walker_0: { x, y: 6.5, yaw: 0, vx: 1, vy: 0, moving: true } } }),
});
// A free 16 x 16 m grid over the area, so the view fits it.
const GRID = (() => {
  const res = 0.1, w = 160, h = 160;
  return { header: { frame_id: 'map' },
           info: { resolution: res, width: w, height: h,
                   origin: { position: { x: -16, y: 0, z: 0 },
                             orientation: { x: 0, y: 0, z: 0, w: 1 } } },
           data: new Array(w * h).fill(0) };
})();

// ---------------------------------------------------------------- plumbing

const children = [];
const spawnTracked = (cmd, argv, opts = {}) => {
  const p = spawn(cmd, argv, { stdio: 'ignore', ...opts });
  children.push(p);
  return p;
};
const cleanup = () => { for (const p of children) { try { p.kill('SIGKILL'); } catch {} } };
process.on('exit', cleanup);
process.on('SIGINT', () => { cleanup(); process.exit(130); });

async function waitFor(fn, what) {
  for (let i = 0; i < 60; i++) {
    try { if (await fn()) return; } catch {}
    await sleep(250);
  }
  throw new Error(`timed out waiting for ${what}`);
}

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
      const r = await api.send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
      const det = r.result?.exceptionDetails;
      if (det) throw new Error(det.exception?.description || det.text);
      return r.result?.result?.value;
    },
    close: () => ws.close(),
  };
  return api;
}

const results = [];
const check = (name, pass, detail = '') => {
  results.push(pass);
  console.log(`${pass ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
};

// Colour at a world point on the map canvas (asks the page where that is).
const PIXEL = (x, y) => `(() => {
  const [px, py] = worldToPixel(${x}, ${y});
  const d = ctx.getImageData(Math.round(px), Math.round(py), 1, 1).data;
  return [d[0], d[1], d[2]];
})()`;
const isOrange = ([r, g, b]) => r > 180 && g > 70 && g < 170 && b < 90;
const isBlueish = ([r, g, b]) => b > r + 40 && b > g;

async function main() {
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--port', String(HTTP_PORT),
                           '--host', '127.0.0.1'], { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${HTTP_PORT}/`)).ok, 'serve.py');

  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-chrome-'));
  spawnTracked('google-chrome', ['--headless=new', `--remote-debugging-port=${CDP_PORT}`,
    '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check',
    '--disable-gpu-sandbox', `--user-data-dir=${profile}`, 'about:blank']);
  let target;
  await waitFor(async () => {
    const list = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json();
    target = list.find((t) => t.type === 'page');
    return !!target;
  }, 'headless chrome');

  const pageErrors = [];
  const page = cdp(target.webSocketDebuggerUrl, (m, api) => {
    if (m.method === 'Runtime.exceptionThrown') {
      const d = m.params.exceptionDetails;
      pageErrors.push(d.exception?.description || d.text);
      return;
    }
    if (m.method !== 'Fetch.requestPaused') return;
    const { requestId, request } = m.params;
    if (request.url.includes('roslib.min.js')) {
      return api.send('Fetch.fulfillRequest', {
        requestId, responseCode: 200,
        responseHeaders: [{ name: 'Content-Type', value: 'application/javascript' }],
        body: Buffer.from(ROSLIB_STUB).toString('base64'),
      });
    }
    if (request.url.includes('/snapshot')) {
      return api.send('Fetch.failRequest', { requestId, errorReason: 'Failed' });
    }
    return api.send('Fetch.continueRequest', { requestId });
  });
  await page.ready;
  await page.send('Page.enable');
  await page.send('Runtime.enable');
  await page.send('Fetch.enable', { patterns: [{ urlPattern: '*' }] });
  await page.send('Emulation.setDeviceMetricsOverride', {
    width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false });
  await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/` });
  await sleep(1500);

  await page.eval(`document.querySelector('#stage-mode button[data-stage="map"]').click()`);
  await page.eval(`window.__feed('/map', ${JSON.stringify(GRID)})`);
  await sleep(200);
  const subscribed = await page.eval(`['/sim/world', '/sim/ground_truth']
    .every((t) => (window.__subs[t] || []).length > 0)`);
  check('app.js subscribes to /sim/world and /sim/ground_truth', subscribed);

  // Nothing published yet: no tracker row, nothing drawn at the walker's spot.
  const before = await page.eval(PIXEL(-8.5, 6.5));
  check('no world drawn before /sim/world arrives', !isOrange(before), before.join(','));
  check('world tracker row hidden without a world',
        await page.eval(`document.getElementById('trk-world-row').hidden`));

  await page.eval(`window.__feed('/sim/world', { data: ${JSON.stringify(JSON.stringify(WORLD))} })`);
  await page.eval(`window.__feed('/sim/ground_truth', ${JSON.stringify(truth(2.5, -8.5))})`);
  await sleep(300);
  const actor = await page.eval(PIXEL(-8.5, 6.5));
  check('the walker is drawn at its ground-truth position', isOrange(actor), actor.join(','));
  const crate = await page.eval(PIXEL(-9.0, 3.0));
  check('the prop is drawn over its footprint', isBlueish(crate), crate.join(','));
  const elsewhere = await page.eval(PIXEL(-13.0, 6.5));
  check('nothing walker-coloured away from the walker', !isOrange(elsewhere), elsewhere.join(','));
  const tracker = await page.eval(`({
    hidden: document.getElementById('trk-world-row').hidden,
    text: document.getElementById('trk-world').textContent })`);
  check('tracker names the world and the actor clock',
        !tracker.hidden && /unit_world/.test(tracker.text) && /t=2\.5s/.test(tracker.text),
        tracker.text);
  if (SCREENSHOT_DIR) {
    fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });
    const shot = await page.send('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(path.join(SCREENSHOT_DIR, 'world-overlay.png'),
                     Buffer.from(shot.result.data, 'base64'));
  }

  // The walker moves: drawn at the new pose, gone from the old one.
  await page.eval(`window.__feed('/sim/ground_truth', ${JSON.stringify(truth(4.0, -7.0))})`);
  await sleep(300);
  check('the walker follows ground truth',
        isOrange(await page.eval(PIXEL(-7.0, 6.5))) && !isOrange(await page.eval(PIXEL(-8.5, 6.5))));

  // Actor controls publish world commands.
  await page.eval(`document.getElementById('world-restart').click();
                   document.getElementById('world-pause').click();`);
  const cmds = await page.eval(`(window.__pubs['/sim/world_cmd'] || []).map((m) => JSON.parse(m.data).cmd)`);
  check('restart / pause publish /sim/world_cmd', JSON.stringify(cmds) === '["restart","pause"]',
        JSON.stringify(cmds));

  // The toggle hides it.
  await page.eval(`document.getElementById('world-toggle').click()`);
  await sleep(200);
  check('the world toggle hides the overlay', !isOrange(await page.eval(PIXEL(-7.0, 6.5))));
  await page.eval(`document.getElementById('world-toggle').click()`);

  // Ground truth goes quiet (sim stopped): stop drawing a stale actor.
  await sleep(3300);
  check('a stale world is not drawn', !isOrange(await page.eval(PIXEL(-7.0, 6.5))));
  check('the tracker row hides when ground truth goes quiet', await page.eval(`(() => {
    window.__feed('/sim/world', { data: ${JSON.stringify(JSON.stringify(WORLD))} });
    return document.getElementById('trk-world-row').hidden; })()`));

  // A sim without --world publishes {world: null}: nothing to draw.
  await page.eval(`window.__feed('/sim/world', { data: JSON.stringify({ world: null, props: [], actors: [] }) })`);
  await page.eval(`window.__feed('/sim/ground_truth', ${JSON.stringify(truth(5.0, -7.0))})`);
  await sleep(300);
  check('no overlay for a sim without a world', !isOrange(await page.eval(PIXEL(-7.0, 6.5))));

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join('; '));
  page.close();
  const passed = results.filter(Boolean).length;
  console.log(`\n${passed}/${results.length} checks passed`);
  process.exit(passed === results.length ? 0 : 1);
}

main().catch((e) => { console.error(`harness error: ${e.message}`); process.exit(2); });
