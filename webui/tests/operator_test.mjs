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
  // Pick-up / drop-off are plain waypoints now (merged from the old stations).
  // The map test below places A and B; a task dispatches between those two.
  const pick = 'A', drop = 'B';

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
    check(`${vp.label}: no mode tile without a session`, !(await visible('#mode-tile')));
    check(`${vp.label}: page does not scroll`, await page.eval(`document.documentElement.scrollWidth <= innerWidth && document.documentElement.scrollHeight <= innerHeight`));
    await shot(`${vp.label}-empty`);

    // -- session panel (scenarios.js, reused) ------------------------------------
    await click('#empty-go');
    await sleep(800);
    check(`${vp.label}: Start a session opens the session panel`, await visible('#session') && await visible('#launch-form'));
    check(`${vp.label}: launch form is filled from the API`, await page.eval(`document.querySelectorAll('#lf-preset option').length > 1`));
    check(`${vp.label}: Start is wired (not disabled by a missing console)`, await page.eval(`!document.getElementById('lf-start').disabled`));
    check(`${vp.label}: with no session the panel is just the form`, await visible('#launch-form') && !(await visible('#session-pane')) && !(await visible('#lf-command')));
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
    const href = await page.eval(`document.querySelector('#fleet .robot.sel a.eng').getAttribute('href')`);
    check(`${vp.label}: live view link is the per-robot dashboard`, href === `/?robot=${r1}`, href);
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
    const tile = await page.eval(`(() => { const t = document.querySelector('#fleet .robot[data-id="${r0}"] .tile'); return t.className + '|' + t.textContent.replace(/\s+/g, ' '); })()`);
    check(`${vp.label}: the task tile names the task and is coloured as moving`, /\bmoving\b/.test(tile) && /Go to Dock/.test(tile) && /1 waiting/.test(tile), tile);
    check(`${vp.label}: a busy robot offers Clear tasks`, await page.eval(`!!document.querySelector('#fleet .robot[data-id="${r0}"] [data-act="clear"]')`));
    await page.eval(`window.__published.length = 0`);
    await click(`#fleet .robot[data-id="${r0}"] [data-act="clear"]`);
    check(`${vp.label}: Clear tasks cancels everything on that robot`, await page.eval(`window.__published.some((x) => x.topic === '/${r0}/cancel_task' && x.msg.data === 'all')`));
    await emit(`/${r0}/task_status`, { data: JSON.stringify({ task_id: null, state: 'IDLE', queue_depth: 0, queue: [] }) });
    await sleep(300);
    check(`${vp.label}: an idle robot's tile reads No task`, /No task/.test(await page.eval(`document.querySelector('#fleet .robot[data-id="${r0}"] .tile').textContent`)));
    await shot(`${vp.label}-two-cards`);
    await click(`#fleet .robot[data-id="${r0}"] .l1`);
    await sleep(200);
    check(`${vp.label}: closing a card hands control back to the other open one`, (await page.eval(`window.__operator.selected`)) === r1 && (await page.eval(`document.querySelectorAll('#fleet .robot.sel').length`)) === 1);

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
    check(`${vp.label}: the robot row follows its task`, /Heading to pick-up/.test(await page.eval(`document.querySelector('#fleet .robot.sel .l2').textContent`)));
    await emit(`/${r1}/task_status`, { data: JSON.stringify({ task_id: payload.task_id, state: 'DONE', queue_depth: 0 }) });
    await sleep(200);
    check(`${vp.label}: DONE finishes the task`, /done/.test(await $text('#tasks')));
    await shot(`${vp.label}-fleet`);

    // -- demo ----------------------------------------------------------------------------------------------
    // configs/demo_tasks.yaml (served live by serve.py) runs in phases: phase 2 must not
    // be queued until every task of phase 1 has finished. Its waypoint names are the
    // warehouse map's, so swap those in for the duration.
    const demo = (await (await fetch(`http://127.0.0.1:${PORT}/api/demo_tasks`)).json()).phases || [];
    check(`${vp.label}: the demo is served as phases`, demo.length >= 2 && demo.every((ph) => ph.title && ph.tasks.length), `${demo.length} phases`);
    const savedWps = await page.eval(`JSON.stringify(window.__navWps)`);
    const demoNames = [...new Set(demo.flatMap((ph) => ph.tasks.flatMap((t) => [t.goto, t.pickup, t.dropoff]).filter(Boolean)))];
    await page.eval(`window.__navWps = ${JSON.stringify(demoNames.map((name, i) => ({ name, x: i, y: 1, yaw: 0 })))}; window.__operator.loadWaypoints()`);
    await sleep(200);
    const dispatched = async () => JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).filter((x) => /\/dispatch_task$/.test(x.topic));
    await page.eval(`window.__published.length = 0`);
    await click('#tk-demo');
    await sleep(300);
    const ph1 = await dispatched();
    check(`${vp.label}: Run demo queues only the first phase`, ph1.length === demo[0].tasks.length, `${ph1.length}/${demo[0].tasks.length}`);
    check(`${vp.label}: each demo task goes to the robot it names`, ph1.every((x, i) => x.topic === `/${demo[0].tasks[i].robot}/dispatch_task`), ph1.map((x) => x.topic).join(','));
    check(`${vp.label}: the demo says which phase is running`, (await $text('#demo-status')) === `Demo 1/${demo.length}: ${demo[0].title}` && (await $text('#tk-demo')) === 'Stop demo', await $text('#demo-status'));
    const ids1 = ph1.map((x) => [x.topic.split('/')[1], JSON.parse(x.msg.data).task_id]);
    await emit(`/${ids1[0][0]}/task_status`, { data: JSON.stringify({ task_id: ids1[0][1], state: 'DONE', queue_depth: 1, queue: ids1.slice(1).map(([, id]) => ({ task_id: id, kind: 'goto', goto: 'x' })) }) });
    await sleep(2200);
    check(`${vp.label}: the next phase waits for the whole current one`, (await dispatched()).length === ph1.length);
    for (const [rid, id] of ids1.slice(1)) await emit(`/${rid}/task_status`, { data: JSON.stringify({ task_id: id, state: 'DONE', queue_depth: 0, queue: [] }) });
    await sleep(2200);
    const ph2 = (await dispatched()).slice(ph1.length);
    check(`${vp.label}: once a phase is done the next one starts`, ph2.length === demo[1].tasks.length && ph2.every((x, i) => x.topic === `/${demo[1].tasks[i].robot}/dispatch_task`), ph2.map((x) => x.topic).join(','));
    check(`${vp.label}: the status moves to phase 2`, (await $text('#demo-status')).startsWith(`Demo 2/${demo.length}`), await $text('#demo-status'));
    await page.eval(`window.__published.length = 0`);
    await click('#tk-demo');
    const cancels = JSON.parse(await page.eval(`JSON.stringify(window.__published)`)).filter((x) => /\/cancel_task$/.test(x.topic) && x.msg.data === 'all');
    check(`${vp.label}: Stop demo cancels every robot it used`, new Set(cancels.map((x) => x.topic)).size === new Set([...ph1, ...ph2].map((x) => x.topic.split('/')[1])).size, cancels.map((x) => x.topic).join(','));
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

    // -- a premapped session in the header -------------------------------------------------------------------
    await page.eval(`window.__fakeSession = { state: 'running', phase: 'running', phase_label: 'running', session_id: 's1', scenario: 'library_localize', config: { map: 'warehouse', mode: 'readonly', robots: 2 } }`);
    await sleep(3400);
    check(`${vp.label}: running session shows in the header`, (await $text('#s-label')) === 'Session running', await $text('#s-label'));
    check(`${vp.label}: a read-only saved map reads as Premapped`, (await $text('#mode-label')) === 'Premapped' && (await $text('#map-name')) === 'warehouse' && await visible('#mode-tile'));
    check(`${vp.label}: the mode tile is a status, not a control`, await page.eval(`!document.querySelector('#mode-tile button')`));
    check(`${vp.label}: the view toggle sits in the header`, await page.eval(`!!document.querySelector('header #view')`));
    check(`${vp.label}: running but robots silent still shows the last stretch`, (await $text('#sp-boot .boot-label')).startsWith('Waiting for the robots'), await $text('#sp-boot .boot-label'));
    await emit('/tf', tf('map', `${r0}/base_link`, -3, 2, 0.5)); await emit('/tf', tf('map', `${r1}/base_link`, 4, -2, 3.1));
    await sleep(400);
    check(`${vp.label}: the bar is gone once the robots are up`, await page.eval(`document.getElementById('sp-boot').hidden`));
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
    check(`${vp.label}: a failed session shows its status and log`, await visible('#session-pane') && await page.eval(`document.getElementById('sp-log').open`));
    await page.eval(`window.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }))`);
    await page.eval(`window.__fakeSession = null`);
  }


  // -- 2D map / 3D cloud -----------------------------------------------------------------------
  console.log('\n--- 3d view ---');
  const load = async () => { await page.send('Page.navigate', { url: `http://127.0.0.1:${PORT}/operator.html` }); await sleep(1200); };
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
  check('3d: the hint explains orbiting', /orbit/.test(await $text('#hint')));
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
