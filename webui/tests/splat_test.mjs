#!/usr/bin/env node
/**
 * Headless test for the 3D panel's splat mode (webui/splat_view.js + app.js).
 *
 *     node webui/tests/splat_test.mjs [--screenshot DIR] [--port N]
 *
 * Needs NO fixture, no ROS, no GPU, no splat worker and no trained model:
 *   - webui/serve.py serves the page over a seeded one-map library;
 *   - a FAKE splat worker (in this process) serves /api/splats, /api/jobs and
 *     a synthetic .splat — a ring of coloured gaussians generated here, in the
 *     exact 32-byte layout splat/convert.py writes;
 *   - headless Chrome on SwiftShader renders it. ?recon=splat forces the mode,
 *     which is exactly the "explicit choice wins on software GL" rule.
 *
 * Asserts the page's contract with the worker (cross-origin fetch, CORS, the
 * POST body a "build" sends, job progress, auto-load on done) and that pixels
 * actually come out of the splat canvas. Zero npm dependencies, like the other
 * webui tests: node's WebSocket/fetch drive CDP directly.
 */
import { spawn } from 'node:child_process';
import http from 'node:http';
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
const HTTP_PORT = Number(argVal('--port', '8093'));
const WORKER_PORT = HTTP_PORT + 1;
const CDP_PORT = Number(argVal('--cdp-port', '9225'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-splat-shots')))
  : null;

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
// WebGL canvases are cleared after compositing, so a readback from the test
// would see nothing. Keep the buffer — test-only, never in the page itself.
const PRESERVE_BUFFER = `
(() => {
  const orig = HTMLCanvasElement.prototype.getContext;
  HTMLCanvasElement.prototype.getContext = function (type, attrs) {
    if (type === 'webgl2') attrs = Object.assign({}, attrs, { preserveDrawingBuffer: true });
    return orig.call(this, type, attrs);
  };
})();
`;

// ---------------------------------------------------------------- fixture

const MAP = 'seeded_warehouse';
const N_GAUSSIANS = 2000;

// A 6 m ring of fist-sized gaussians around (2, 3, 1), hue around the ring.
function makeSplat(n) {
  const buf = Buffer.alloc(n * 32);
  for (let i = 0; i < n; i++) {
    const t = (i / n) * Math.PI * 2;
    const o = i * 32;
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

function seedMapLibrary() {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-maps-'));
  const entry = path.join(dir, MAP);
  fs.mkdirSync(entry);
  fs.writeFileSync(path.join(entry, 'map.db'),
                   Buffer.concat([Buffer.from('SQLite format 3\0'), Buffer.alloc(4080)]));
  fs.writeFileSync(path.join(entry, 'map.json'), JSON.stringify({
    schema: 1, name: MAP, title: 'Seeded warehouse', description: '',
    created: '2026-10-10T00:00:00Z', updated: '2026-10-10T00:00:00Z', scene: 'nvidia',
    robot_ids: ['robot_0'], primary_robot_id: 'robot_0', source: {}, grid: null,
    coverage: null, dbs: { robot_0: { file: 'map.db', bytes: 4096, state: 'complete' } },
    db_state: 'complete', versions: {},
  }));
  return dir;
}

// ---------------------------------------------------------------- fake worker

const worker = {
  posts: [],
  jobs: [],
  splats: [{ map: MAP, job: `${MAP}-20261009T120000Z`, source: 'local', finished: 1791500000,
             bytes: SPLAT.length, gaussians: N_GAUSSIANS, psnr: 27.1,
             url: `/splats/${MAP}/${MAP}-20261009T120000Z.splat` }],
  splatRequests: [],
  polls: 0,
};

function startFakeWorker() {
  const srv = http.createServer((req, res) => {
    const cors = {
      'Access-Control-Allow-Origin': '*',
      'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
      'Access-Control-Allow-Headers': 'Content-Type',
    };
    const json = (obj, code = 200) => {
      res.writeHead(code, { ...cors, 'Content-Type': 'application/json' });
      res.end(JSON.stringify(obj));
    };
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
            j.state = 'done';
            j.finished = Date.now() / 1000;
            worker.splats.unshift({ ...worker.splats[0], job: j.id, finished: j.finished,
                                    url: `/splats/${MAP}/${j.id}.splat` });
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
        const job = { id: `${b.map}-20261010T000000Z`, map: b.map, source: b.source,
                      state: 'queued', created: Date.now() / 1000 };
        worker.jobs.push(job);
        json(job, 201);
      });
      return;
    }
    if (url.pathname.startsWith(`/splats/${MAP}/`)) {
      worker.splatRequests.push(url.pathname);
      res.writeHead(200, { ...cors, 'Content-Type': 'application/octet-stream',
                           'Content-Length': SPLAT.length });
      return res.end(SPLAT);
    }
    json({ error: 'no such endpoint' }, 404);
  });
  return new Promise((r) => srv.listen(WORKER_PORT, '127.0.0.1', () => r(srv)));
}

// ---------------------------------------------------------------- processes / CDP

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

async function waitFor(fn, { tries = 60, delay = 250, what = 'condition' } = {}) {
  for (let i = 0; i < tries; i++) {
    try { if (await fn()) return true; } catch {}
    await sleep(delay);
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
      const r = await api.send('Runtime.evaluate', {
        expression, returnByValue: true, awaitPromise: true,
      });
      const det = r.result?.exceptionDetails;
      if (det) throw new Error(det.exception?.description || det.text);
      return r.result?.result?.value;
    },
  };
  return api;
}

const results = [];
const check = (name, pass, detail = '') => {
  results.push({ name, pass });
  console.log(`${pass ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
};

// Pixels in the splat canvas that differ from its clear colour (alpha > 0).
const LIT_PIXELS = `(() => {
  const c = document.querySelector('#recon-canvas canvas.splat-canvas');
  if (!c || !c.width) return -1;
  const gl = c.getContext('webgl2');
  const px = new Uint8Array(c.width * c.height * 4);
  gl.readPixels(0, 0, c.width, c.height, gl.RGBA, gl.UNSIGNED_BYTE, px);
  let lit = 0;
  for (let i = 3; i < px.length; i += 4) if (px[i] > 8) lit++;
  return lit;
})()`;

async function main() {
  const mapsDir = seedMapLibrary();
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'), '--platform', 'sim',
                           '--port', String(HTTP_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT, env: { ...process.env, SORTBOTS_MAPS_DIR: mapsDir } });
  const fake = await startFakeWorker();
  await waitFor(async () => (await fetch(`http://127.0.0.1:${HTTP_PORT}/`)).ok,
                { what: `webui/serve.py on :${HTTP_PORT}` });

  const cfg = await (await fetch(`http://127.0.0.1:${HTTP_PORT}/api/splat/config`)).json();
  check('serve.py exposes the splat worker URL', typeof cfg.worker_url === 'string', cfg.worker_url);
  const maps = await (await fetch(`http://127.0.0.1:${HTTP_PORT}/api/maps`)).json();
  check('/api/maps reports the platform', maps.platform === 'sim', maps.platform);
  const db = await fetch(`http://127.0.0.1:${HTTP_PORT}/api/maps/${MAP}/db`);
  check('serve.py streams a complete pose graph', db.ok &&
        (await db.arrayBuffer()).byteLength === 4096);

  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-chrome-'));
  const chromeBin = ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']
    .find((b) => { try { return spawn(b, ['--version']).pid; } catch { return false; } })
    || 'google-chrome';
  spawnTracked(chromeBin, [
    '--headless=new', `--remote-debugging-port=${CDP_PORT}`,
    '--enable-unsafe-swiftshader', '--use-angle=swiftshader', '--no-first-run',
    '--no-default-browser-check', '--disable-gpu-sandbox', `--user-data-dir=${profile}`,
    'about:blank',
  ]);
  let target;
  await waitFor(async () => {
    const list = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json();
    target = list.find((t) => t.type === 'page');
    return !!target;
  }, { what: `headless chrome on :${CDP_PORT}` });

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
  await page.send('Page.addScriptToEvaluateOnNewDocument', { source: PRESERVE_BUFFER });
  await page.send('Emulation.setDeviceMetricsOverride', {
    width: 1600, height: 1000, deviceScaleFactor: 1, mobile: false,
  });

  const worker_url = `http://127.0.0.1:${WORKER_PORT}`;
  await page.send('Page.navigate', {
    url: `http://127.0.0.1:${HTTP_PORT}/?recon=splat&splat_worker=${encodeURIComponent(worker_url)}`,
  });

  // 1. splat mode comes up and loads the newest splat from the worker.
  await waitFor(() => page.eval(`document.getElementById('recon-info').textContent.includes('gaussians')`),
                { what: 'splat to load', tries: 80 });
  const state = await page.eval(`(() => ({
    mode: document.getElementById('recon-mode').value,
    bar: !document.getElementById('splat-bar').hidden,
    info: document.getElementById('recon-info').textContent,
    picks: [...document.querySelectorAll('#splat-pick option')].map((o) => o.value),
    maps: [...document.querySelectorAll('#splat-map option')].map((o) => o.value),
    canvas: getComputedStyle(document.querySelector('#recon-canvas canvas.splat-canvas')).display,
  }))()`);
  check('?recon=splat selects splat mode', state.mode === 'splat');
  check('splat bar is shown in splat mode', state.bar);
  check('splat canvas is displayed', state.canvas === 'block', state.canvas);
  check('info line counts the loaded gaussians', state.info.includes(N_GAUSSIANS.toLocaleString('en-US')),
        state.info);
  check('picker lists the worker\'s splats', state.picks.length === 1 &&
        state.picks[0] === worker.splats[0].url, state.picks.join(','));
  check('build picker lists complete library maps', state.maps.join(',') === MAP, state.maps.join(','));
  check('splat fetched cross-origin from the worker', worker.splatRequests.length === 1,
        worker.splatRequests.join(','));

  // 2. the camera was fitted to the splat and it actually draws.
  await sleep(1500);   // a sort round-trip through the Web Worker
  const lit = await page.eval(LIT_PIXELS);
  check('splat renders pixels', lit > 500, `${lit} lit px`);
  if (SCREENSHOT_DIR) {
    fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });
    const shot = await page.send('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(path.join(SCREENSHOT_DIR, 'splat.png'), Buffer.from(shot.result.data, 'base64'));
  }

  // 3. build: the page asks the worker to train from this dashboard's map.
  // Worker and page share a host here (sim: the workstation does both), so
  // the source must be "local" — not this origin, which would make the
  // worker download the DB from itself.
  await page.eval(`document.getElementById('splat-build').click()`);
  await waitFor(() => worker.posts.length === 1, { what: 'build POST' });
  check('build posts the selected map with source "local"',
        worker.posts[0].map === MAP && worker.posts[0].source === 'local',
        JSON.stringify(worker.posts[0]));
  // The fake job reports progress from its second poll on (like train.py,
  // whose first progress.json lands a few seconds into training).
  await waitFor(() => page.eval(`/training \\d+\\/15000/.test(document.getElementById('splat-status').textContent)`),
                { what: 'training progress', tries: 40 });
  const training = await page.eval(`document.getElementById('splat-status').textContent`);
  check('job progress is shown while training', /training \d+\/15000/.test(training), training);
  await waitFor(() => worker.splatRequests.length === 2, { what: 'auto-load of the new splat', tries: 60 });
  const after = await page.eval(`(() => ({
    pick: document.getElementById('splat-pick').value,
    status: document.getElementById('splat-status').textContent,
  }))()`);
  check('a finished job is picked and loaded automatically',
        after.pick.includes(`${MAP}-20261010T000000Z`), after.pick);

  // 4. leaving splat mode hides it and stops polling.
  await page.eval(`(() => { const s = document.getElementById('recon-mode');
                            s.value = 'points'; s.dispatchEvent(new Event('change')); })()`);
  await sleep(300);
  const pollsBefore = worker.polls;
  await sleep(3500);
  const off = await page.eval(`(() => ({
    bar: document.getElementById('splat-bar').hidden,
    canvas: getComputedStyle(document.querySelector('#recon-canvas canvas.splat-canvas')).display,
  }))()`);
  check('points mode hides the splat bar and canvas', off.bar && off.canvas === 'none',
        JSON.stringify(off));
  check('points mode stops polling the worker', worker.polls === pollsBefore,
        `${worker.polls - pollsBefore} polls after leaving`);

  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join(' | '));

  fake.close();
  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} passed`);
  cleanup();
  process.exit(failed.length ? 1 : 0);
}

main().catch((e) => { console.error(e); cleanup(); process.exit(1); });
