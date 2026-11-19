#!/usr/bin/env node
/**
 * Headless test for the 3D view's splat mode (webui/operator_splat.js + operator_cloud.js).
 *
 *     node webui/tests/splat_test.mjs [--screenshot DIR] [--port N]
 *
 * Its own file, not a section of operator_test.mjs: that one drives the whole fleet UI and
 * is timing-sensitive, and this must be deterministic. Needs NO fixture, ROS, GPU, splat
 * worker or trained model:
 *   1. pure node: the renderer's Web Worker source runs in a vm with a fake `self` — the
 *      32-byte .splat layout in, texture texels and a near-first depth order out — and
 *      sourceFor() picks "local" vs this page's origin;
 *   2. headless Chrome on SwiftShader, the real serve.py over a seeded one-map library, and
 *      a FAKE splat worker in this process: worker down (the message says how to start it),
 *      then up (list, load, pixels), build -> progress -> done -> auto-load, and leaving
 *      splat mode stops the polls and gives the live cloud its subscription back.
 * Zero npm dependencies, like the other webui tests: node's WebSocket/fetch drive CDP.
 */
import { spawn } from 'node:child_process';
import http from 'node:http';
import vm from 'node:vm';
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
const HTTP_PORT = Number(argVal('--port', '8243'));
const WORKER_PORT = HTTP_PORT + 1;
const DEAD_PORT = HTTP_PORT + 2;   // nothing listens here: "worker not running"
const CDP_PORT = Number(argVal('--cdp-port', '9243'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-splat-shots'))) : null;

const results = [];
const check = (name, pass, detail = '') => {
  results.push({ name, pass: !!pass });
  console.log(`${pass ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
};

// ---------------------------------------------------------------- fixture

const MAP = 'seeded_warehouse';
const N_GAUSSIANS = 2000;

// A 6 m ring of fist-sized gaussians around (2, 3, 1), hue around the ring: the exact
// 32-byte layout splat/convert.py writes.
function makeSplat(n) {
  const buf = Buffer.alloc(n * 32);
  for (let i = 0; i < n; i++) {
    const t = (i / n) * Math.PI * 2, o = i * 32;
    buf.writeFloatLE(2 + 3 * Math.cos(t), o);
    buf.writeFloatLE(3 + 3 * Math.sin(t), o + 4);
    buf.writeFloatLE(1 + 0.5 * Math.sin(5 * t), o + 8);
    for (let a = 0; a < 3; a++) buf.writeFloatLE(0.08, o + 12 + a * 4);
    buf[o + 24] = Math.round(127 + 127 * Math.cos(t));
    buf[o + 25] = Math.round(127 + 127 * Math.cos(t + 2.1));
    buf[o + 26] = Math.round(127 + 127 * Math.cos(t + 4.2));
    buf[o + 27] = 255;
    buf[o + 28] = 255; buf[o + 29] = 128; buf[o + 30] = 128; buf[o + 31] = 128; // identity
  }
  return buf;
}
const SPLAT = makeSplat(N_GAUSSIANS);

const tmpDirs = [];
const tmpDir = (prefix) => { const d = fs.mkdtempSync(path.join(os.tmpdir(), prefix)); tmpDirs.push(d); return d; };

function seedMapLibrary() {
  const dir = tmpDir('sortbots-maps-');
  const entry = path.join(dir, MAP);
  fs.mkdirSync(entry);
  fs.writeFileSync(path.join(entry, 'map.db'), Buffer.concat([Buffer.from('SQLite format 3\0'), Buffer.alloc(4080)]));
  fs.writeFileSync(path.join(entry, 'map.json'), JSON.stringify({
    schema: 1, name: MAP, title: 'Seeded warehouse', description: '',
    created: '2026-10-10T00:00:00Z', updated: '2026-10-10T00:00:00Z', scene: 'nvidia',
    robot_ids: ['robot_0'], primary_robot_id: 'robot_0', source: {}, grid: null,
    coverage: null, dbs: { robot_0: { file: 'map.db', bytes: 4096, state: 'complete' } },
    db_state: 'complete', versions: {},
  }));
  return dir;
}

// ---------------------------------------------------------------- 1. pure node

function loadModule() {
  // operator_splat.js is a browser script: run it against a window with just enough on it.
  const win = {};
  const ctx = vm.createContext({ window: win, location: { search: '' }, URL, URLSearchParams, console });
  vm.runInContext(fs.readFileSync(path.join(WEBUI_DIR, 'operator_splat.js'), 'utf8'), ctx);
  return win.sortbotsSplat;
}

// Runs the renderer's Web Worker source with a fake `self`; returns its posted messages.
function runSortWorker(src) {
  const posted = [];
  const self = { postMessage: (m) => posted.push(m) };
  const timers = [];
  const ctx = vm.createContext({ self, setTimeout: (fn) => timers.push(fn), Float32Array, Uint8Array,
    Uint32Array, Int32Array, Math, Infinity });
  vm.runInContext(src, ctx);
  const flush = () => { while (timers.length) timers.shift()(); };
  return { send: (m) => self.onmessage({ data: m }), flush, posted };
}

function pureChecks() {
  const mod = loadModule();
  check('operator_splat.js exposes SplatView, createPanel, WORKER_SRC, sourceFor',
        mod && typeof mod.SplatView === 'function' && typeof mod.createPanel === 'function'
        && typeof mod.WORKER_SRC === 'string' && typeof mod.sourceFor === 'function');

  const loc = (href) => { const u = new URL(href); return { hostname: u.hostname, origin: u.origin }; };
  check('build source: same host is "local" (sim: the workstation serves the page and trains)',
        mod.sourceFor('http://ws.example:8082', loc('http://ws.example:8081/')) === 'local');
  check('build source: localhost vs 127.0.0.1 is still "local"',
        mod.sourceFor('http://localhost:8082', loc('http://127.0.0.1:8081/')) === 'local');
  check('build source: a page served by the robot hands the worker its own origin',
        mod.sourceFor('http://ws.example:8082', loc('http://jetson.example:8081/op')) === 'http://jetson.example:8081');

  const w = runSortWorker(mod.WORKER_SRC);
  // A tiny splat: 3 gaussians on the camera's z axis at 5, 1, 3 m, distinct colours.
  const tiny = Buffer.alloc(3 * 32);
  [[0, 0, 5], [0, 0, 1], [0, 0, 3]].forEach(([x, y, z], i) => {
    const o = i * 32;
    tiny.writeFloatLE(x, o); tiny.writeFloatLE(y, o + 4); tiny.writeFloatLE(z, o + 8);
    for (let a = 0; a < 3; a++) tiny.writeFloatLE(0.1, o + 12 + a * 4);
    tiny.set([10 * (i + 1), 20 * (i + 1), 30 * (i + 1), 200], o + 24);
    tiny.set([255, 128, 128, 128], o + 28);
  });
  const ab = tiny.buffer.slice(tiny.byteOffset, tiny.byteOffset + tiny.byteLength);
  w.send({ type: 'load', buffer: ab });
  const tex = w.posted.find((m) => m.type === 'texture');
  check('worker: a load posts the texture with every gaussian', tex && tex.count === 3, tex && tex.count);
  const tf = new Float32Array(tex.texdata.buffer), tc = new Uint8Array(tex.texdata.buffer);
  check('worker: centres land in texel 0 of each pair, as float bits',
        tf[0] === 0 && tf[2] === 5 && tf[8 + 2] === 1 && tf[16 + 2] === 3);
  check('worker: colour bytes ride in the second texel\'s w',
        [...tc.subarray(4 * 7, 4 * 7 + 4)].join(',') === '10,20,30,200'
        && [...tc.subarray(4 * 15, 4 * 15 + 4)].join(',') === '20,40,60,200');
  check('worker: bounds come back for the camera fit', tex.bounds && tex.bounds.max[2] >= 3, JSON.stringify(tex.bounds));

  // Identity view (OpenCV w2c, column-major): camera-space z = world z. Near first.
  const view = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  w.send({ type: 'view', view }); w.flush();
  const order = w.posted.filter((m) => m.type === 'order').pop();
  check('worker: a view sorts front to back (1 m, 3 m, 5 m)', order && [...order.depthIndex].join(',') === '1,2,0',
        order && [...order.depthIndex].join(','));
  // Looking the other way (z negated) flips the order.
  const back = view.slice(); back[10] = -1;
  const before = w.posted.length;
  w.send({ type: 'view', view }); w.send({ type: 'view', view: back }); w.flush();
  const sorts = w.posted.slice(before).filter((m) => m.type === 'order');
  check('worker: views queued behind a sort coalesce into one, for the newest',
        sorts.length === 1 && [...sorts[0].depthIndex].join(',') === '0,2,1', sorts.map((s) => [...s.depthIndex].join(',')).join(' | '));
  const n = w.posted.length;
  w.send({ type: 'view', view: back }); w.flush();
  check('worker: an unchanged view does not re-sort', w.posted.length === n);
}

// ---------------------------------------------------------------- fake worker

const worker = {
  posts: [], jobs: [], splatRequests: [], polls: 0,
  splats: [{ map: MAP, job: `${MAP}-20261009T120000Z`, source: 'local', finished: 1791500000,
             bytes: SPLAT.length, gaussians: N_GAUSSIANS, psnr: 27.1,
             url: `/splats/${MAP}/${MAP}-20261009T120000Z.splat` }],
};

function startFakeWorker() {
  const srv = http.createServer((req, res) => {
    const cors = { 'Access-Control-Allow-Origin': '*', 'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
                   'Access-Control-Allow-Headers': 'Content-Type' };
    const json = (obj, code = 200) => { res.writeHead(code, { ...cors, 'Content-Type': 'application/json' }); res.end(JSON.stringify(obj)); };
    const url = new URL(req.url, 'http://x');
    if (req.method === 'OPTIONS') { res.writeHead(204, cors); return res.end(); }
    if (url.pathname === '/api/splats') return json({ splats: worker.splats });
    if (url.pathname === '/api/jobs' && req.method === 'GET') {
      // A submitted job trains for two polls, then finishes and becomes a splat.
      worker.polls++;
      for (const j of worker.jobs) {
        if (j.state === 'queued') j.state = 'training';
        else if (j.state === 'training') {
          j.ticks = (j.ticks || 0) + 1;
          j.progress = { step: 5000 * j.ticks, iters: 15000, psnr: 24.5, eta_s: 120 };
          if (j.ticks >= 2) {
            j.state = 'done'; j.finished = Date.now() / 1000; delete j.progress;
            worker.splats.unshift({ ...worker.splats[0], job: j.id, finished: j.finished, url: `/splats/${MAP}/${j.id}.splat` });
          }
        }
      }
      return json({ jobs: [...worker.jobs].reverse() });
    }
    if (url.pathname === '/api/jobs' && req.method === 'POST') {
      let body = '';
      req.on('data', (c) => { body += c; });
      req.on('end', () => {
        const b = JSON.parse(body);
        worker.posts.push(b);
        const job = { id: `${b.map}-20261010T000000Z`, map: b.map, source: b.source, state: 'queued', created: Date.now() / 1000 };
        worker.jobs.push(job);
        json(job, 201);
      });
      return;
    }
    if (url.pathname.startsWith(`/splats/${MAP}/`)) {
      worker.splatRequests.push(url.pathname);
      res.writeHead(200, { ...cors, 'Content-Type': 'application/octet-stream', 'Content-Length': SPLAT.length });
      return res.end(SPLAT);
    }
    json({ error: 'no such endpoint' }, 404);
  });
  return new Promise((r) => srv.listen(WORKER_PORT, '127.0.0.1', () => r(srv)));
}

// ---------------------------------------------------------------- processes / CDP

const children = [];
function spawnTracked(cmd, argv, opts = {}) { const p = spawn(cmd, argv, { stdio: 'ignore', ...opts }); children.push(p); return p; }
function cleanup() {
  for (const p of children) { try { p.kill('SIGKILL'); } catch {} }
  for (const d of tmpDirs) { try { fs.rmSync(d, { recursive: true, force: true }); } catch {} }
}
process.on('exit', cleanup);
process.on('SIGINT', () => { cleanup(); process.exit(130); });

async function waitFor(fn, { tries = 60, delay = 250, what = 'condition' } = {}) {
  for (let i = 0; i < tries; i++) { try { if (await fn()) return true; } catch {} await sleep(delay); }
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
    send(method, params = {}) { const msgId = ++id; return new Promise((res) => { pending.set(msgId, res); ws.send(JSON.stringify({ id: msgId, method, params })); }); },
    async eval(expression) {
      const r = await api.send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
      const det = r.result?.exceptionDetails;
      if (det) throw new Error(det.exception?.description || det.text);
      return r.result?.result?.value;
    },
  };
  return api;
}

// Records subscriptions so the test can see the cloud topic come and go.
const ROSLIB_STUB = `
window.__topics = {};
window.ROSLIB = {
  Ros: function () { this.on = () => {}; this.connect = () => {}; this.close = () => {}; this.callOnConnection = () => {}; },
  Topic: function (o) { let mine = null;
    this.subscribe = (cb) => { mine = cb; (window.__topics[o.name] = window.__topics[o.name] || []).push(cb); };
    this.unsubscribe = () => { if (mine && window.__topics[o.name]) window.__topics[o.name] = window.__topics[o.name].filter((x) => x !== mine); };
    this.advertise = () => {}; this.publish = () => {}; },
  Message: function (d) { Object.assign(this, d); },
  Service: function () { this.callService = () => {}; },
  ServiceRequest: function (d) { Object.assign(this, d); },
};
`;
// WebGL canvases are cleared after compositing, so a readback from the test would see
// nothing. Keep the buffer: test-only, never in the page itself.
const PRESERVE_BUFFER = `
(() => { const orig = HTMLCanvasElement.prototype.getContext;
  HTMLCanvasElement.prototype.getContext = function (type, attrs) {
    if (type === 'webgl2') attrs = Object.assign({}, attrs, { preserveDrawingBuffer: true });
    return orig.call(this, type, attrs); }; })();
`;
const LIT_PIXELS = `(() => {
  const c = document.querySelector('#cloud canvas.splat-canvas');
  if (!c || !c.width) return -1;
  const gl = c.getContext('webgl2');
  const px = new Uint8Array(c.width * c.height * 4);
  gl.readPixels(0, 0, c.width, c.height, gl.RGBA, gl.UNSIGNED_BYTE, px);
  let lit = 0;
  for (let i = 3; i < px.length; i += 4) if (px[i] > 8) lit++;
  return lit;
})()`;
const STATE = `(() => { const q = (s) => document.querySelector(s);
  return { mode: window.sortbotsCloud && window.sortbotsCloud.mode,
    splatActive: q('#cl-mode [data-m="splat"]').classList.contains('active'),
    ctl: !q('#splat-ctl').hidden, info: q('#cloud .cloud-info').textContent,
    status: q('#splat-status').textContent, msg: q('#splat-msg').textContent,
    msgShown: !q('#splat-msg').hidden && getComputedStyle(q('#splat-msg')).display !== 'none',
    build: q('#splat-build').disabled,
    picks: [...document.querySelectorAll('#splat-pick option')].map((o) => o.value),
    maps: [...document.querySelectorAll('#splat-map option')].map((o) => o.value),
    canvas: (q('#cloud canvas.splat-canvas') && getComputedStyle(q('#cloud canvas.splat-canvas')).display) || null,
    cloudSubs: (window.__topics['/fleet/recon_cloud'] || []).length }; })()`;

async function browserChecks() {
  const mapsDir = seedMapLibrary();
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--platform', 'sim', '--port', String(HTTP_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: { ...process.env, SORTBOTS_MAPS_DIR: mapsDir, SORTBOTS_SESSIONS_DIR: tmpDir('sortbots-sessions-') } });
  const fake = await startFakeWorker();
  await waitFor(async () => { const r = await fetch(`http://127.0.0.1:${HTTP_PORT}/operator.html`); await r.arrayBuffer(); return r.ok; },
                { what: `webui/serve.py on :${HTTP_PORT}` });
  const cfg = await (await fetch(`http://127.0.0.1:${HTTP_PORT}/api/splat/config`)).json();
  check('serve.py exposes the splat worker URL', typeof cfg.worker_url === 'string', cfg.worker_url);

  const profile = tmpDir('sortbots-chrome-');
  const chromeBin = ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']
    .find((b) => { try { return spawn(b, ['--version']).pid; } catch { return false; } }) || 'google-chrome';
  spawnTracked(chromeBin, ['--headless=new', `--remote-debugging-port=${CDP_PORT}`, '--enable-unsafe-swiftshader',
    '--use-angle=swiftshader', '--no-first-run', '--no-default-browser-check', '--disable-gpu-sandbox',
    `--user-data-dir=${profile}`, 'about:blank']);
  let target;
  await waitFor(async () => {
    const list = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json();
    target = list.find((t) => t.type === 'page'); return !!target;
  }, { what: `headless chrome on :${CDP_PORT}` });

  const pageErrors = [];
  const page = cdp(target.webSocketDebuggerUrl, (m, api) => {
    if (m.method === 'Runtime.exceptionThrown') { const d = m.params.exceptionDetails; pageErrors.push(d.exception?.description || d.text); return; }
    if (m.method !== 'Fetch.requestPaused') return;
    const { requestId, request } = m.params;
    if (request.url.includes('roslib.min.js')) {
      return api.send('Fetch.fulfillRequest', { requestId, responseCode: 200,
        responseHeaders: [{ name: 'Content-Type', value: 'application/javascript' }], body: Buffer.from(ROSLIB_STUB).toString('base64') });
    }
    if (request.url.includes('/snapshot')) return api.send('Fetch.failRequest', { requestId, errorReason: 'Failed' });
    return api.send('Fetch.continueRequest', { requestId });
  });
  await page.ready;
  await page.send('Page.enable'); await page.send('Runtime.enable');
  await page.send('Fetch.enable', { patterns: [{ urlPattern: '*' }] });
  await page.send('Page.addScriptToEvaluateOnNewDocument', { source: PRESERVE_BUFFER });
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false });

  const open = async (workerUrl) => {
    await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/operator.html?recon=splat&splat_worker=${encodeURIComponent(workerUrl)}` });
    await waitFor(() => page.eval(`!!(window.__operator && window.sortbotsCloud)`), { what: 'operator view' });
    await page.eval(`window.__operator.setView('3d')`);
  };

  // A. worker down: say so, and how to start it; Build can't be pressed.
  await open(`http://127.0.0.1:${DEAD_PORT}`);
  await waitFor(() => page.eval(`document.getElementById('splat-status').textContent.includes('unreachable')`), { what: 'worker down status' });
  let s = await page.eval(STATE);
  check('?recon=splat opens the 3D view in splat mode', s.mode === 'splat' && s.splatActive && s.ctl, JSON.stringify({ mode: s.mode, ctl: s.ctl }));
  check('worker down: the view says how to start it', s.msgShown && s.msg.includes('scripts/splat.sh serve'), s.msg);
  check('worker down: Build is disabled', s.build === true);
  check('splat mode does not subscribe the live cloud', s.cloudSubs === 0, s.cloudSubs);

  // B. worker up: newest splat listed and loaded, library maps offered, pixels drawn.
  await open(`http://127.0.0.1:${WORKER_PORT}`);
  await waitFor(() => page.eval(`document.querySelector('#cloud .cloud-info').textContent.includes('gaussians')`), { what: 'splat to load', tries: 80 });
  s = await page.eval(STATE);
  check('info line counts the loaded gaussians', s.info.includes(N_GAUSSIANS.toLocaleString('en-US')), s.info);
  check('picker lists the worker\'s splats', s.picks.length === 1 && s.picks[0] === worker.splats[0].url, s.picks.join(','));
  check('build picker lists complete library maps', s.maps.join(',') === MAP, s.maps.join(','));
  check('Build is enabled with a map to build from', s.build === false);
  check('splat canvas is displayed', s.canvas === 'block', s.canvas);
  check('no message over a loaded splat', !s.msgShown, s.msg);
  // No robot is online in this test: the "waiting for the robots" card must not wash over
  // a saved map's splat (it did, 2026-11-17).
  const card = await page.eval(`(() => { const e = document.getElementById('empty'); return { stage: document.getElementById('stage').classList.contains('splatting'), shown: !!e && !e.hidden && getComputedStyle(e).display !== 'none' }; })()`);
  check('splat mode hides the waiting-for-robots card', card.stage && !card.shown, JSON.stringify(card));
  check('splat fetched cross-origin from the worker', worker.splatRequests.length === 1, worker.splatRequests.join(','));
  await sleep(1500); // a sort round-trip through the Web Worker
  const lit = await page.eval(LIT_PIXELS);
  check('splat renders pixels', lit > 500, `${lit} lit px`);
  if (SCREENSHOT_DIR) {
    fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });
    const shot = await page.send('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(path.join(SCREENSHOT_DIR, 'splat.png'), Buffer.from(shot.result.data, 'base64'));
  }

  // C. build: worker and page share a host here (sim), so the source must be "local",
  // not this origin, which would make the worker download the DB from itself.
  await page.eval(`document.getElementById('splat-build').click()`);
  await waitFor(() => worker.posts.length === 1, { what: 'build POST' });
  check('Build posts the selected map with source "local"', worker.posts[0].map === MAP && worker.posts[0].source === 'local', JSON.stringify(worker.posts[0]));
  await waitFor(() => page.eval(`/training \\d+\\/15000/.test(document.getElementById('splat-status').textContent)`), { what: 'training progress', tries: 40 });
  const training = await page.eval(`document.getElementById('splat-status').textContent`);
  check('job progress is shown while training (step/iters, dB, time left)', /training \d+\/15000 · 24\.5 dB · \d+ min left/.test(training), training);
  await waitFor(() => worker.splatRequests.length === 2, { what: 'auto-load of the new splat', tries: 60 });
  s = await page.eval(STATE);
  check('a finished job is picked and loaded automatically', s.picks.length === 2 && worker.splatRequests[1].includes(`${MAP}-20261010T000000Z`), worker.splatRequests.join(','));

  // D. back to the point cloud: splat hidden, polls stop, the cloud subscribed again.
  await page.eval(`document.querySelector('#cl-mode [data-m="cloud"]').click()`);
  await sleep(300);
  const pollsBefore = worker.polls;
  await sleep(3500);
  s = await page.eval(STATE);
  check('Point cloud hides the splat controls and canvas', !s.ctl && s.canvas === 'none', JSON.stringify({ ctl: s.ctl, canvas: s.canvas }));
  check('Point cloud gives the stage back to the robot cards', !(await page.eval(`document.getElementById('stage').classList.contains('splatting')`)));
  check('Point cloud stops polling the worker', worker.polls === pollsBefore, `${worker.polls - pollsBefore} polls after leaving`);
  check('Point cloud subscribes the live cloud again', s.cloudSubs === 1, s.cloudSubs);
  // Leaving the 3D view entirely stops everything, in either mode.
  await page.eval(`document.querySelector('#cl-mode [data-m="splat"]').click()`); await sleep(300);
  await page.eval(`window.__operator.setView('2d')`); await sleep(300);
  const p2 = worker.polls; await sleep(3500);
  check('leaving the 3D view in splat mode stops the polls', worker.polls === p2, `${worker.polls - p2} polls`);

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join(' | '));
  fake.close();
}

async function main() {
  pureChecks();
  await browserChecks();
  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} passed`);
  cleanup();
  process.exit(failed.length ? 1 : 0);
}

main().catch((e) => { console.error(e); cleanup(); process.exit(1); });
