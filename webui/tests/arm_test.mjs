#!/usr/bin/env node
/**
 * Headless test for the dashboard's arm teleop pad (webui/arm.js).
 *
 *     node webui/tests/arm_test.mjs [--screenshot DIR] [--port N]
 *
 * Needs NO ROS, no Isaac Sim, no GPU, no display and no fixture — unlike
 * dashboard_test.mjs it drives the pad directly rather than replaying recorded
 * data, so it runs anywhere in a few seconds. Zero npm dependencies: node's
 * built-in WebSocket and fetch drive the Chrome DevTools Protocol, and
 * vendor/roslib.min.js is intercepted and replaced with a stub that records
 * every publish into window.__pubs.
 *
 * The assertion that matters most is the round-trip: the YAML the pad emits is
 * piped through nodes/arm_poses.load_pose_book(). arm.js and arm_poses.py each
 * hand-write that format, and this is what stops them drifting apart.
 *
 * Exit code is 0 only if every assertion passes.
 */
import { spawn, spawnSync } from 'node:child_process';
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
const HTTP_PORT = Number(argVal('--port', '8096'));
const CDP_PORT = Number(argVal('--cdp-port', '9224'));
const SCREENSHOT_DIR = args.includes('--screenshot')
  ? path.resolve(argVal('--screenshot', path.join(os.tmpdir(), 'sortbots-arm-shots')))
  : null;

// Must match webui/arm.js JOINTS and nodes/arm_poses.ARM_JOINT_NAMES.
const JOINT_NAMES = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw'];
const LIMITS = {
  Rotation: [-2.1, 2.1],
  Pitch: [-0.1, 3.45],
  Elbow: [-0.2, 3.14159],
  Wrist_Pitch: [-1.8, 1.8],
  Wrist_Roll: [-3.14159, 3.14159],
  Jaw: [0.0, 1.7],
};

const ROSLIB_STUB = `
window.__subs = {};
window.__pubs = [];
window.ROSLIB = {
  Ros: function () {
    this.on = (ev, cb) => { if (ev === 'connection') setTimeout(cb, 0); };
    this.connect = () => {}; this.close = () => {};
    this.callOnConnection = () => {};
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
  TFClient: function () {
    this.subscribe = (frame, cb) => { window.__subs['tf:' + frame] = cb; };
    this.unsubscribe = () => {}; this.dispose = () => {};
  },
};
window.__deliver = (topic, msg) => {
  const cb = window.__subs[topic];
  if (!cb) return false;
  cb(msg); return true;
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
  { label: 'phone-390x844', w: 390, h: 844, mobile: true },
];

// Holding a jog button is a mousedown/mouseup pair with real time in between,
// because arm.js accumulates on a 10 Hz interval exactly like the head-aim pad.
const HOLD = (sel, ms) => `(async () => {
  const el = document.querySelector(${JSON.stringify(sel)});
  if (!el) return 'no element ' + ${JSON.stringify(sel)};
  el.dispatchEvent(new MouseEvent('mousedown', {bubbles: true}));
  await new Promise((r) => setTimeout(r, ${ms}));
  el.dispatchEvent(new MouseEvent('mouseup', {bubbles: true}));
  return 'ok';
})()`;

const SHOW_ARM = `document.querySelector('#arm-toggle button[data-pad="arm"]').click()`;
const SHOW_HEAD = `document.querySelector('#arm-toggle button[data-pad="head"]').click()`;
const LAST_PUB = `(() => {
  const arm = window.__pubs.filter(([t]) => t.endsWith('/arm_joint_cmd'));
  return arm.length ? {topic: arm[arm.length-1][0], msg: arm[arm.length-1][1], n: arm.length} : null;
})()`;

async function main() {
  spawnTracked('python3', [path.join(WEBUI_DIR, 'serve.py'),
                           '--port', String(HTTP_PORT), '--host', '127.0.0.1'],
               { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${HTTP_PORT}/`)).ok,
                { what: `webui/serve.py on :${HTTP_PORT}` });

  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'sortbots-chrome-arm-'));
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

  const load = async () => {
    await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/` });
    await sleep(1200);
    // A stale capture log in localStorage would corrupt the round-trip check.
    await page.eval(`(() => { try { localStorage.clear(); } catch(e){} })()`);
  };

  // ---------------------------------------------------------------- 1. mount
  await page.send('Emulation.setDeviceMetricsOverride',
                  { width: 1920, height: 1080, deviceScaleFactor: 1, mobile: false });
  await load();

  const mounted = await page.eval(`(() => ({
    overlay: !!document.getElementById('arm-overlay'),
    toggle: !!document.getElementById('arm-toggle'),
    rows: document.querySelectorAll('#arm-rows button[data-jog]').length,
  }))()`);
  check('arm pad mounts itself into #stage-panel',
        mounted.overlay && mounted.toggle, JSON.stringify(mounted));
  check('one jog button pair per joint', mounted.rows === JOINT_NAMES.length * 2,
        `${mounted.rows} buttons`);

  // hidden until you ask for it; the head pad keeps the corner by default
  const initialVis = await page.eval(`(() => {
    const vis = (id) => {
      const el = document.getElementById(id);
      return el ? getComputedStyle(el).display !== 'none' : null;
    };
    return { arm: vis('arm-overlay'), aim: vis('aim-overlay') };
  })()`);
  check('head pad owns the corner by default', initialVis.aim === true && initialVis.arm === false,
        JSON.stringify(initialVis));

  await page.eval(SHOW_ARM);
  await sleep(150);
  const armVis = await page.eval(`(() => {
    const vis = (id) => {
      const el = document.getElementById(id);
      return el ? getComputedStyle(el).display !== 'none' : null;
    };
    return { arm: vis('arm-overlay'), aim: vis('aim-overlay') };
  })()`);
  check('arm toggle swaps the two corner pads', armVis.arm === true && armVis.aim === false,
        JSON.stringify(armVis));

  // ------------------------------------------------- 2. the JointState contract
  await page.eval(`window.__pubs.length = 0`);
  await page.eval(HOLD('#arm-rows button[data-jog="1:1"]', 350));
  await sleep(100);
  const pub = await page.eval(LAST_PUB);
  check('jogging publishes on /<robot>/arm_joint_cmd',
        pub && pub.topic === '/robot_0/arm_joint_cmd', pub ? pub.topic : 'no publish');
  check('published JointState carries ALL SIX names in contract order',
        pub && JSON.stringify(pub.msg.name) === JSON.stringify(JOINT_NAMES),
        pub ? JSON.stringify(pub.msg.name) : 'n/a');
  check('published position has six values',
        pub && Array.isArray(pub.msg.position) && pub.msg.position.length === 6,
        pub ? `len=${pub.msg.position?.length}` : 'n/a');
  check('jogging Pitch moves only Pitch',
        pub && pub.msg.position[1] > 0 &&
          pub.msg.position.filter((v, i) => i !== 1 && v !== 0).length === 0,
        pub ? JSON.stringify(pub.msg.position) : 'n/a');

  // ------------------------------------------------------------- 3. clamping
  // Drive every joint hard into both rails and assert it stops exactly there.
  for (const [idx, jname] of JOINT_NAMES.entries()) {
    const [lo, hi] = LIMITS[jname];
    for (const [dir, want] of [[1, hi], [-1, lo]]) {
      // With arm.js's hold ramp, a rail-to-rail sweep of the widest joint
      // (Wrist_Roll, 2pi) takes ~3.3 s. 4.5 s leaves margin on a slow machine.
      await page.eval(HOLD(`#arm-rows button[data-jog="${idx}:${dir}"]`, 4500));
      await sleep(60);
      const p = await page.eval(LAST_PUB);
      const got = p?.msg?.position?.[idx];
      check(`${jname} clamps at ${dir > 0 ? 'upper' : 'lower'} limit ${want}`,
            typeof got === 'number' && Math.abs(got - want) < 1e-6,
            `got ${got}`);
    }
  }

  // ------------------------------------------------------------- 4. home
  await page.eval(`document.getElementById('arm-home').click()`);
  await sleep(80);
  const homed = await page.eval(LAST_PUB);
  check('home publishes all-zeros',
        homed && homed.msg.position.every((v) => v === 0),
        homed ? JSON.stringify(homed.msg.position) : 'n/a');

  // ---------------------------------------------- 5. capture round-trip
  // The highest-value assertion here: the pad's YAML is parsed by the same
  // loader scripted_pick.py uses, so the two formats cannot drift.
  await page.eval(HOLD('#arm-rows button[data-jog="2:1"]', 300));
  await page.eval(`(() => {
    document.getElementById('arm-capture-name').value = 'approach';
    document.getElementById('arm-capture').click();
  })()`);
  await page.eval(HOLD('#arm-rows button[data-jog="3:1"]', 300));
  await page.eval(`document.getElementById('arm-capture').click()`);
  await sleep(100);

  const captured = await page.eval(`document.getElementById('arm-captures').value`);
  check('capture log renders entries', !!captured && captured.includes('- {name: approach'),
        JSON.stringify(captured?.slice(0, 80) || ''));
  check('capture name auto-increments', !!captured && captured.includes('approach2'),
        JSON.stringify(captured?.split('\n').pop() || ''));

  const bookYaml = [
    'joint_names: [' + JOINT_NAMES.join(', ') + ']',
    'limits:',
    ...JOINT_NAMES.map((n) => `  ${n}: [${LIMITS[n][0]}, ${LIMITS[n][1]}]`),
    'sequences:',
    '  pick:',
    captured,
    '  place:',
    captured,
    '',
  ].join('\n');
  const tmpYaml = path.join(os.tmpdir(), `sortbots-arm-capture-${process.pid}.yaml`);
  fs.writeFileSync(tmpYaml, bookYaml);
  const rt = spawnSync('python3', [path.join(REPO_ROOT, 'nodes', 'arm_poses.py'), tmpYaml],
                       { encoding: 'utf8' });
  check('captured YAML round-trips through nodes/arm_poses.py',
        rt.status === 0, (rt.stderr || rt.stdout || '').trim().slice(0, 220));
  fs.unlinkSync(tmpYaml);

  // ------------------------------------------------------- 6. map mode hides
  await page.eval(`document.querySelector('#stage-mode button[data-stage="map"]').click()`);
  await sleep(200);
  const inMap = await page.eval(`(() => {
    const el = document.getElementById('arm-overlay');
    const tg = document.getElementById('arm-toggle');
    return { arm: getComputedStyle(el).display !== 'none',
             toggle: getComputedStyle(tg).display !== 'none' };
  })()`);
  check('arm pad hides in map mode', inMap.arm === false && inMap.toggle === false,
        JSON.stringify(inMap));

  await page.eval(`document.querySelector('#stage-mode button[data-stage="camera"]').click()`);
  await sleep(200);
  const backToCam = await page.eval(
    `getComputedStyle(document.getElementById('arm-overlay')).display !== 'none'`);
  check('arm pad returns when leaving map mode (survives setStageMode)', backToCam === true);

  // ------------------------------------------------------- 7. layout, all viewports
  for (const vp of VIEWPORTS) {
    await page.send('Emulation.setDeviceMetricsOverride',
                    { width: vp.w, height: vp.h, deviceScaleFactor: 1, mobile: vp.mobile });
    await load();
    await page.eval(SHOW_ARM);
    await sleep(300);

    const geom = await page.eval(`(() => {
      const r = (id) => {
        const el = document.getElementById(id);
        if (!el) return null;
        const b = el.getBoundingClientRect();
        return { x: b.x, y: b.y, w: b.width, h: b.height, r: b.right, b: b.bottom };
      };
      return { arm: r('arm-overlay'), drive: r('drive-overlay'), stage: r('stage-panel'),
               hScroll: document.documentElement.scrollWidth > document.documentElement.clientWidth };
    })()`);

    const overlaps = geom.arm && geom.drive &&
      geom.arm.x < geom.drive.r && geom.drive.x < geom.arm.r &&
      geom.arm.y < geom.drive.b && geom.drive.y < geom.arm.b;
    check(`${vp.label}: arm pad does not overlap the drive pad`, !overlaps,
          overlaps ? `arm=${JSON.stringify(geom.arm)} drive=${JSON.stringify(geom.drive)}` : '');
    check(`${vp.label}: arm pad stays inside the stage panel`,
          geom.arm && geom.stage &&
            geom.arm.r <= geom.stage.r + 1 && geom.arm.b <= geom.stage.b + 1,
          geom.arm ? `arm.r=${geom.arm.r.toFixed(0)} stage.r=${geom.stage.r.toFixed(0)}` : 'n/a');
    check(`${vp.label}: no horizontal page scroll`, geom.hScroll === false);

    if (SCREENSHOT_DIR) {
      const shot = await page.send('Page.captureScreenshot', { format: 'png' });
      fs.writeFileSync(path.join(SCREENSHOT_DIR, `arm-${vp.label}.png`),
                       Buffer.from(shot.result.data, 'base64'));
    }
  }

  // ------------------------------------------------------------- 8. no errors
  check('no uncaught page errors', pageErrors.length === 0, pageErrors.join(' | ').slice(0, 300));

  page.close();

  const failed = results.filter((r) => !r.pass);
  console.log(`\n${results.length - failed.length}/${results.length} checks passed`);
  if (SCREENSHOT_DIR) console.log(`screenshots in ${SCREENSHOT_DIR}`);
  return failed.length === 0 ? 0 : 1;
}

main().then((code) => process.exit(code), (err) => {
  console.error(err);
  process.exit(1);
});
