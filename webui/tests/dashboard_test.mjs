#!/usr/bin/env node
/**
 * Headless test for the SortBots dashboard, driven by recorded ROS data.
 *
 *     node webui/tests/dashboard_test.mjs [--fixture DIR] [--screenshot DIR] [--keep-open]
 *
 * Needs NO ROS, no Isaac Sim, no GPU and no display — it replays the fixture
 * built by scripts/bag_to_fixture.py from a bag recorded once with
 * scripts/record_dashboard_bag.sh. Runs in a few seconds, so a CSS change can
 * be checked without sourcing anything.
 *
 * Zero npm dependencies on purpose (no package.json, no node_modules to keep
 * in sync): node's built-in WebSocket and fetch drive the Chrome DevTools
 * Protocol directly.
 *
 * How the page is fooled into thinking it's live:
 *
 *   - vendor/roslib.min.js is INTERCEPTED and replaced with a stub. Injecting
 *     the stub earlier does not work — the real vendored bundle loads after
 *     any Page.addScriptToEvaluateOnNewDocument and overwrites window.ROSLIB.
 *     The stub captures each subscribe() callback so the fixture can be fed
 *     straight into them.
 *   - /snapshot?topic=... requests (normally web_video_server on :8080) are
 *     intercepted and fulfilled with the recorded JPEG frames.
 *   - webui/serve.py serves the page itself — it has no ROS dependency, so it
 *     runs standalone on a spare port.
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
const FIXTURE_DIR = path.resolve(argVal('--fixture', path.join(WEBUI_DIR, 'testdata')));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-dashboard-shots')))
  : null;
const HTTP_PORT = Number(argVal('--port', '8099'));
const CDP_PORT = Number(argVal('--cdp-port', '9222'));

// ---------------------------------------------------------------- fixture

const fixturePath = path.join(FIXTURE_DIR, 'fixture.json');
if (!fs.existsSync(fixturePath)) {
  console.error(`no fixture at ${fixturePath}

This test replays real recorded ROS data, which has to be captured once from a
running sim:

  1. bring the demo up:      scripts/run_demo.sh
  2. record (drive around!): scripts/record_dashboard_bag.sh --duration 60
  3. build the fixture:      python3 scripts/bag_to_fixture.py data/bags/<bag>/
  4. re-run this test.
`);
  process.exit(2);
}
const fixture = JSON.parse(fs.readFileSync(fixturePath, 'utf8'));

// Frames served in place of web_video_server, keyed by the topic the dashboard
// asks for in its /snapshot query string.
const frameCache = new Map();
for (const [topic, rels] of Object.entries(fixture.images || {})) {
  const bufs = rels
    .map((rel) => path.join(FIXTURE_DIR, rel))
    .filter((p) => fs.existsSync(p))
    .map((p) => fs.readFileSync(p));
  if (bufs.length) frameCache.set(topic, bufs);
}

const ROSLIB_STUB = `
window.__subs = {};        // topic -> subscribe callback
window.__sentGoals = [];   // callOnConnection payloads (nav goals)
window.__pubs = [];        // [topic, message] published by the page
window.ROSLIB = {
  Ros: function () {
    this.on = (ev, cb) => { if (ev === 'connection') setTimeout(cb, 0); };
    this.connect = () => {};
    this.close = () => {};
    this.callOnConnection = (m) => { window.__sentGoals.push(m); };
  },
  Topic: function (o) {
    this.name = o.name;
    this.subscribe = (cb) => { window.__subs[o.name] = cb; };
    this.unsubscribe = () => {};
    this.publish = (m) => { window.__pubs.push([o.name, m]); };
    this.advertise = () => {};
  },
  Message: function (d) { Object.assign(this, d); },
  Service: function () { this.callService = () => {}; },
  ServiceRequest: function (d) { Object.assign(this, d); },
  // Must exist even though app.js no longer uses it: a missing constructor
  // throws and silently aborts the rest of app.js's top-level execution.
  TFClient: function () {
    this.subscribe = (frame, cb) => { window.__subs['tf:' + frame] = cb; };
    this.unsubscribe = () => {}; this.dispose = () => {};
  },
};
window.__deliver = (topic, msg) => {
  const cb = window.__subs[topic];
  if (!cb) return false;
  cb(msg);
  return true;
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

async function replayFixture(page) {
  // Deliver every recorded message in timestamp order, so the page sees the
  // same interleaving the live system produced.
  const flat = [];
  for (const [topic, entry] of Object.entries(fixture.topics || {})) {
    for (const m of entry.messages) flat.push({ topic, t: m.t, msg: m.msg });
  }
  flat.sort((a, b) => a.t - b.t);
  const payload = JSON.stringify(flat);
  return page.eval(`(() => {
    const msgs = ${payload};
    let delivered = 0, missed = {};
    for (const {topic, msg} of msgs) {
      if (window.__deliver(topic, msg)) delivered++;
      else missed[topic] = (missed[topic] || 0) + 1;
    }
    return { delivered, missed };
  })()`);
}

/**
 * Count pixels on the map canvas matching a channel-dominance predicate.
 *
 * Deliberately NOT an exact RGB match: the trail is a 1px stroke, so canvas
 * antialiasing blends it with the white free-space underneath and the pure
 * colour may never appear in the buffer at all. Dominance survives blending.
 *   trail  #2dd4ff -> blue-dominant  (orange plan and green marker are not)
 *   marker #22dd55 -> green-dominant over both red and blue
 */
const COUNT_PIXELS = `(kind) => {
  const c = document.getElementById('map-canvas');
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  let n = 0;
  for (let i = 0; i < d.length; i += 4) {
    const r = d[i], g = d[i+1], b = d[i+2];
    if (kind === 'trail' && b - r > 60 && b > 150) n++;
    else if (kind === 'marker' && g - r > 60 && g - b > 60) n++;
  }
  return n;
}`;

async function runFullAssertions(page) {
  const replay = await replayFixture(page);
  const missedTopics = Object.keys(replay.missed);
  check('fixture messages delivered to the page', replay.delivered > 0,
        `${replay.delivered} delivered`);
  if (missedTopics.length) {
    console.log(`      note: no subscriber for ${missedTopics.join(', ')}`);
  }
  await sleep(500);

  // --- stage: default + PiP swap ---
  const def = await page.eval(`({
    chase: document.getElementById('chase-stream').className,
    head: document.getElementById('camera-stream').className,
  })`);
  check('default stage is chase-main + head-PiP',
        def.chase.includes('stage-main') && def.head.includes('stage-pip'));

  await page.eval(`document.getElementById('camera-stream').click()`);
  const swapped = await page.eval(`({
    chase: document.getElementById('chase-stream').className,
    head: document.getElementById('camera-stream').className,
  })`);
  check('clicking the PiP swaps the feeds',
        swapped.chase.includes('stage-pip') && swapped.head.includes('stage-main'));
  await page.eval(`document.getElementById('chase-stream').click()`);

  // --- camera frames actually arrived (proves the snapshot path) ---
  const feeds = await page.eval(`(async () => {
    const out = {};
    for (const id of ['chase-stream','camera-stream']) {
      const img = document.getElementById(id);
      out[id] = { src: !!img.src && !img.src.endsWith('/'), w: img.naturalWidth, h: img.naturalHeight };
    }
    return out;
  })()`);
  check('camera frames render from the recorded JPEGs',
        feeds['chase-stream'].w > 0 && feeds['camera-stream'].w > 0,
        `chase ${feeds['chase-stream'].w}x${feeds['chase-stream'].h}, ` +
        `head ${feeds['camera-stream'].w}x${feeds['camera-stream'].h}`);

  // --- map mode ---
  await page.eval(`document.querySelector('#stage-mode button[data-stage="map"]').click()`);
  await sleep(400);
  const mapMode = await page.eval(`({
    map: document.getElementById('map-view').className,
    aimHidden: getComputedStyle(document.getElementById('aim-overlay')).display === 'none',
    driveVisible: getComputedStyle(document.getElementById('drive-overlay')).display !== 'none',
  })`);
  check('map mode swaps in the map and hides the aim pad',
        mapMode.map.includes('stage-main') && mapMode.aimHidden && mapMode.driveVisible);

  // --- the real grid renders undistorted and fits ---
  const box = await page.eval(`(() => {
    const c = document.getElementById('map-canvas');
    const r = c.getBoundingClientRect();
    const s = document.getElementById('stage-panel').getBoundingClientRect();
    return { bw: c.width, bh: c.height, rw: r.width, rh: r.height,
             fits: r.bottom <= s.bottom + 1 && r.right <= s.right + 1 };
  })()`);
  const gridAspect = box.bw / box.bh, drawnAspect = box.rw / box.rh;
  check('real occupancy grid renders undistorted',
        box.bw > 0 && Math.abs(gridAspect - drawnAspect) < 0.01,
        `${box.bw}x${box.bh} grid, aspect ${gridAspect.toFixed(3)} vs drawn ${drawnAspect.toFixed(3)}`);
  check('map fits inside the stage without clipping', box.fits === true);

  // --- robot pose from /tf: the regression test for the dead TFClient ---
  const marker = await page.eval(`(${COUNT_PIXELS})('marker')`);
  check('robot marker drawn from /tf (TFClient regression)', marker > 0,
        `${marker} marker px`);
  const trail = await page.eval(`(${COUNT_PIXELS})('trail')`);
  check('odom trail drawn from /tf', trail > 0, `${trail} trail px`);

  // --- click-to-nav round trip, against the REAL grid geometry ---
  const nav = await page.eval(`(() => {
    const c = document.getElementById('map-canvas');
    const r = c.getBoundingClientRect();
    const mk = (t,x,y) => new MouseEvent(t,{clientX:x,clientY:y,bubbles:true,cancelable:true});
    // Aim at a point a third of the way in, to avoid the exact centre.
    const vx = r.left + r.width/3, vy = r.top + r.height/3;
    const n = window.__sentGoals.length;
    c.dispatchEvent(mk('mousedown', vx, vy));
    window.dispatchEvent(mk('mouseup', vx, vy));
    const g = window.__sentGoals[n];
    if (!g) return { sent: false };
    return { sent: true, action: g.action, pos: g.args.pose.pose.position,
             status: document.getElementById('goal-status').textContent };
  })()`);
  check('click-to-nav sends a NavigateToPose goal',
        nav.sent && nav.action.endsWith('/navigate_to_pose'),
        nav.sent ? `x=${nav.pos.x.toFixed(2)} y=${nav.pos.y.toFixed(2)}` : 'no goal sent');

  // --- 3D reconstruction ingested the real cloud ---
  const recon = await page.eval(`document.getElementById('recon-info').textContent`);
  check('3D panel ingested the recorded cloud', /\d/.test(recon) && !/waiting/i.test(recon),
        recon.trim());

  // The whole point of Grid/3D=true: the cloud must have real vertical extent,
  // not the ~0.15 m pancake a 2D projected grid produces. The info text reports
  // it as "z <min>–<max> m", so assert on the span rather than trusting that
  // *some* cloud arrived.
  const zs = /z\s+(-?[\d.]+)–(-?[\d.]+)\s*m/.exec(recon);
  const zSpan = zs ? parseFloat(zs[2]) - parseFloat(zs[1]) : 0;
  check('reconstruction is genuinely 3D (z extent > 0.3 m)', zSpan > 0.3,
        zs ? `${zSpan.toFixed(2)} m` : 'no z range in info text');
  check('reconstruction renders as voxels by default', /\bvox\b/.test(recon), recon.trim());

  // Everything above is decode-level and independent of rasterization, so it's
  // safe to assert first and then drop to the cheap renderer. Headless Chrome
  // rasterizes through SwiftShader (--enable-unsafe-swiftshader below), where
  // tens of thousands of instanced cubes is ~12 triangles each in SOFTWARE and
  // would stretch every remaining check in this file.
  await page.eval(`(() => {
    const s = document.getElementById('recon-mode');
    s.value = 'points'; s.dispatchEvent(new Event('change'));
  })()`);
  const reconPts = await page.eval(`document.getElementById('recon-info').textContent`);
  check('recon mode select switches to points', /\bpts\b/.test(reconPts), reconPts.trim());

  // --- task queue populated from recorded task_status ---
  const queue = await page.eval(`document.querySelectorAll('#queue-list li').length`);
  check('task queue populated from recorded task_status', queue > 0, `${queue} rows`);

  // --- SLAM badge from recorded rtabmap_msgs/Info ---
  const badge = await page.eval(`document.getElementById('slam-badge').textContent`);
  check('SLAM badge updated from recorded /info', !/waiting/i.test(badge), badge.trim());

  // --- keyboard drives without focusing the pad ---
  await page.eval(`document.querySelector('#stage-mode button[data-stage="camera"]').click()`);
  const kbd = await page.eval(`(() => {
    window.__pubs.length = 0;
    window.dispatchEvent(new KeyboardEvent('keydown',{key:'w',code:'KeyW',bubbles:true}));
    const drove = window.__pubs.filter(([n]) => n.endsWith('cmd_vel')).map(([,m]) => m.linear.x);
    window.dispatchEvent(new KeyboardEvent('keyup',{key:'w',code:'KeyW',bubbles:true}));
    return drove;
  })()`);
  check('W drives without clicking the pad first', kbd.length > 0 && kbd[0] > 0);

  const guard = await page.eval(`(() => {
    window.__pubs.length = 0;
    document.getElementById('pickup-select')
      .dispatchEvent(new KeyboardEvent('keydown',{key:'w',code:'KeyW',bubbles:true}));
    return window.__pubs.filter(([n]) => n.endsWith('cmd_vel')).length;
  })()`);
  check('typing in the dispatch form does not drive', guard === 0);

  // --- off-stage feed stops polling ---
  await page.eval(`document.querySelector('#stage-mode button[data-stage="map"]').click()`);
  const gated = await page.eval(`(() => {
    const h = document.getElementById('camera-stream');
    return h.classList.contains('stage-main') || h.classList.contains('stage-pip');
  })()`);
  check('head cam leaves the stage in map mode (polling gated)', gated === false);
  await page.eval(`document.querySelector('#stage-mode button[data-stage="camera"]').click()`);
}

// ---------------------------------------------------------------- main

const VIEWPORTS = [
  { label: '1920x1080', w: 1920, h: 1080, mobile: false, full: true },
  { label: '1366x768', w: 1366, h: 768, mobile: false },
  { label: '1280x720', w: 1280, h: 720, mobile: false },
  { label: 'phone-390x844', w: 390, h: 844, mobile: true, allowVScroll: true },
  { label: 'phone-360x740', w: 360, h: 740, mobile: true, allowVScroll: true },
];

async function main() {
  // 1. the page's own server (no ROS dependency)
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'),
                           '--port', String(HTTP_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${HTTP_PORT}/`)).ok,
                { what: `webui/serve.py on :${HTTP_PORT}` });

  // 2. headless chrome. swiftshader so the three.js panel actually renders.
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

  // 3. request interception: fake roslib + fake web_video_server
  const page = cdp(target.webSocketDebuggerUrl, (m, api) => {
    if (m.method !== 'Fetch.requestPaused') return;
    const { requestId, request } = m.params;
    const fulfil = (mime, buf) => api.send('Fetch.fulfillRequest', {
      requestId, responseCode: 200,
      responseHeaders: [{ name: 'Content-Type', value: mime },
                        { name: 'Access-Control-Allow-Origin', value: '*' }],
      body: buf.toString('base64'),
    });
    if (request.url.includes('roslib.min.js')) {
      return fulfil('application/javascript', Buffer.from(ROSLIB_STUB));
    }
    if (request.url.includes('/snapshot')) {
      const topic = new URL(request.url).searchParams.get('topic');
      const frames = frameCache.get(topic);
      if (frames?.length) {
        // Cycle frames so successive polls look like a live feed.
        const n = (frameCache.get(topic + ':n') || 0);
        frameCache.set(topic + ':n', n + 1);
        return fulfil('image/jpeg', frames[n % frames.length]);
      }
      return api.send('Fetch.failRequest', { requestId, errorReason: 'Failed' });
    }
    return api.send('Fetch.continueRequest', { requestId });
  });
  await page.ready;
  await page.send('Page.enable');
  await page.send('Runtime.enable');
  await page.send('Fetch.enable', { patterns: [{ urlPattern: '*' }] });

  const consoleErrors = [];
  page.send('Runtime.enable');

  if (SCREENSHOT_DIR) fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });

  // 4. run each viewport
  for (const vp of VIEWPORTS) {
    console.log(`\n--- ${vp.label} ---`);
    await page.send('Emulation.setDeviceMetricsOverride', {
      width: vp.w, height: vp.h, deviceScaleFactor: 1, mobile: vp.mobile,
    });
    await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/` });
    await sleep(1800);

    if (vp.full) await runFullAssertions(page);
    else await replayFixture(page);
    await sleep(300);

    const scroll = await page.eval(`({
      v: document.documentElement.scrollHeight - document.documentElement.clientHeight,
      h: document.documentElement.scrollWidth - document.documentElement.clientWidth,
    })`);
    check(`${vp.label}: no horizontal scrolling`, scroll.h <= 1, `overflow ${scroll.h}px`);
    if (!vp.allowVScroll) {
      check(`${vp.label}: fits one viewport (no vertical scrolling)`, scroll.v <= 1,
            `overflow ${scroll.v}px`);
    }

    const errs = await page.eval(`(window.__errs || []).length`);
    if (errs) consoleErrors.push(`${vp.label}: ${errs}`);

    if (SCREENSHOT_DIR) {
      for (const mode of ['camera', 'map']) {
        await page.eval(`document.querySelector('#stage-mode button[data-stage="${mode}"]').click()`);
        await sleep(400);
        const shot = await page.send('Page.captureScreenshot', { format: 'png' });
        const out = path.join(SCREENSHOT_DIR, `${vp.label}-${mode}.png`);
        fs.writeFileSync(out, Buffer.from(shot.result.data, 'base64'));
      }
    }
  }

  check('no uncaught page errors', consoleErrors.length === 0, consoleErrors.join('; '));

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
