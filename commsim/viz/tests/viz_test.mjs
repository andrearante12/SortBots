#!/usr/bin/env node
/**
 * Headless test for the commsim replay page (commsim/viz/index.html).
 *
 *     node --experimental-websocket commsim/viz/tests/viz_test.mjs [--port N] [--cdp-port N]
 *
 * Serves the repo root with python3 -m http.server, opens the page on the
 * committed fixture (commsim/viz/testdata/trace_small.json — regenerate with
 * `python3 -m commsim.viz.testdata.make_fixture`), seeks to known moments and
 * asserts both the page's own state (window.__viz) and what is on the canvas.
 *
 * Zero npm dependencies, like webui/tests: node's WebSocket and fetch drive
 * the Chrome DevTools Protocol. Gotchas found setting this up (2026-10-20):
 *   - Node 20 has no global WebSocket: run with --experimental-websocket
 *     (Node 22+ has it built in).
 *   - Chrome: $CHROME_BIN, else google-chrome / chromium on PATH. Chrome for
 *     Testing works: ~/.local/share/chrome-for-testing/chrome/<ver>/chrome-linux64/chrome
 *   - Ubuntu 25.10 restricts unprivileged user namespaces (AppArmor), so an
 *     unprofiled Chrome cannot build its sandbox: --no-sandbox is added when
 *     /proc/sys/kernel/apparmor_restrict_unprivileged_userns reads 1. That is
 *     for this harness only.
 *
 * Exit 0 only if every assertion passes; 3 on a harness problem.
 */
import { spawn } from 'node:child_process';
import { setTimeout as sleep } from 'node:timers/promises';
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(HERE, '..', '..', '..');
const args = process.argv.slice(2);
const argVal = (name, dflt) => { const i = args.indexOf(name); return i >= 0 && args[i + 1] ? args[i + 1] : dflt; };
const HTTP_PORT = Number(argVal('--port', '8111'));
const CDP_PORT = Number(argVal('--cdp-port', '9231'));

const children = [];
function spawnTracked(cmd, argv, opts = {}) { const p = spawn(cmd, argv, { stdio: 'ignore', ...opts }); children.push(p); return p; }
function cleanup() { for (const p of children) { try { p.kill('SIGKILL'); } catch {} } }
process.on('exit', cleanup);
process.on('SIGINT', () => { cleanup(); process.exit(130); });

const failed = [];
function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
  if (!ok) failed.push(name);
}

async function waitFor(fn, { tries = 80, delay = 250, what = 'service' } = {}) {
  for (let i = 0; i < tries; i++) { try { if (await fn()) return; } catch {} await sleep(delay); }
  throw new Error(`timed out waiting for ${what}`);
}

function chromeBin() {
  if (process.env.CHROME_BIN) return process.env.CHROME_BIN;
  for (const b of ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']) {
    try { if (spawn(b, ['--version']).pid) return b; } catch {}
  }
  return 'google-chrome';
}

function needsNoSandbox() {
  try { return fs.readFileSync('/proc/sys/kernel/apparmor_restrict_unprivileged_userns', 'utf8').trim() === '1'; }
  catch { return false; }
}

function cdp(wsUrl) {
  const ws = new WebSocket(wsUrl);
  let id = 0;
  const pending = new Map();
  const ready = new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.onmessage = (m) => {
    const d = JSON.parse(m.data);
    if (d.id && pending.has(d.id)) { const { res, rej } = pending.get(d.id); pending.delete(d.id); d.error ? rej(new Error(JSON.stringify(d.error))) : res(d.result); }
  };
  const send = (method, params = {}) => new Promise((res, rej) => { const i = ++id; pending.set(i, { res, rej }); ws.send(JSON.stringify({ id: i, method, params })); });
  const evalJs = async (expr) => {
    const r = await send('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true });
    if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description || r.exceptionDetails.text);
    return r.result.value;
  };
  return { ready, send, eval: evalJs };
}

// Dominant-colour pixel count on the floor canvas, evaluated inside the page.
const COUNT = (rgb, tol) => `(() => {
  const c = document.getElementById('floor'); const g = c.getContext('2d');
  const d = g.getImageData(0, 0, c.width, c.height).data; let n = 0;
  for (let i = 0; i < d.length; i += 4) {
    if (Math.abs(d[i] - ${rgb[0]}) < ${tol} && Math.abs(d[i+1] - ${rgb[1]}) < ${tol} && Math.abs(d[i+2] - ${rgb[2]}) < ${tol}) n++;
  }
  return n;
})()`;

async function main() {
  spawnTracked('python3', ['-m', 'http.server', String(HTTP_PORT), '--bind', '127.0.0.1'], { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${HTTP_PORT}/commsim/viz/index.html`)).ok, { what: 'http.server' });

  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'commsim-viz-chrome-'));
  const flags = ['--headless=new', `--remote-debugging-port=${CDP_PORT}`, '--enable-unsafe-swiftshader',
                 '--no-first-run', '--no-default-browser-check', '--disable-gpu-sandbox',
                 `--user-data-dir=${profile}`, '--window-size=1400,900', 'about:blank'];
  if (needsNoSandbox()) flags.unshift('--no-sandbox');
  spawnTracked(chromeBin(), flags);
  let target;
  await waitFor(async () => {
    const list = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json();
    target = list.find((t) => t.type === 'page');
    return !!target;
  }, { what: `headless chrome on :${CDP_PORT}` });

  const page = cdp(target.webSocketDebuggerUrl);
  await page.ready;
  await page.send('Page.enable');
  await page.send('Runtime.enable');
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1400, height: 900, deviceScaleFactor: 1, mobile: false });
  await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/commsim/viz/index.html?trace=/commsim/viz/testdata/trace_small.json` });
  await waitFor(async () => page.eval('window.__viz && window.__viz.state().loaded'), { what: 'trace load' });

  const s0 = await page.eval('window.__viz.state()');
  check('trace loads with 3 nodes and the replay at t=0', s0.loaded && s0.n === 3 && s0.t === 0, JSON.stringify({ n: s0.n, t: s0.t }));
  check('first route poll applied before t=0 (poller started before captures)',
        s0.routes['0'] && s0.routes['0']['2'] && s0.routes['0']['2'].nh === 1, JSON.stringify(s0.routes));

  // inside the first hop of Status (2, seq 7): t = 1.0002 s, hop lasts 400 µs
  // but is stretched to 120 ms of wall time at 1x
  await page.eval('window.__viz.seek(1.0002)');
  await sleep(50);
  const s1 = await page.eval('window.__viz.state()');
  const statusPx = await page.eval(COUNT([0x4c, 0xc9, 0xf0], 24));
  check('a Status hop is active and drawn in the status colour', (s1.active.status || 0) >= 1 && statusPx > 10, `active=${JSON.stringify(s1.active)} px=${statusPx}`);
  check('OGMs are hidden by default', !s1.active.ogm, JSON.stringify(s1.active));

  // the lost hop of Status (2, seq 8) at 1.333 s: its animation is stretched to
  // 140 ms of wall time at 1x, then a red X flashes where it should have landed
  await page.eval('window.__viz.seek(1.49)');
  await sleep(50);
  const s2 = await page.eval('window.__viz.state()');
  const redPx = await page.eval(COUNT([0xff, 0x4d, 0x4d], 30));
  check('a lost hop flashes red after it should have landed', s2.flashes >= 1 && redPx > 10, `flashes=${s2.flashes} px=${redPx}`);

  // block at 8.0 s (links.jsonl): dashed red link + X; unblock at 11.0 s
  await page.eval('window.__viz.seek(8.5)');
  await sleep(50);
  const s3 = await page.eval('window.__viz.state()');
  const blockedPx = await page.eval(COUNT([0xff, 0x3b, 0x3b], 20));
  check('a blocked link is in the state and drawn red', s3.blocked.length === 1 && s3.blocked[0] === '0-1' && blockedPx > 10, `blocked=${s3.blocked} px=${blockedPx}`);
  // n0 heard n2 at 1.003 s and then nothing: silent from 7.003 s
  check('the gateway considers robot 2 silent after the status timeout', (s3.silent['0'] || []).includes(2), JSON.stringify(s3.silent));
  await page.eval('window.__viz.select(0)');
  const s3b = await page.eval('window.__viz.state()');
  check('the node panel shows the silent peer', /SILENT/.test(s3b.panel) && s3b.selected === 0, s3b.panel.slice(0, 80));

  await page.eval('window.__viz.seek(11.5)');
  const s4 = await page.eval('window.__viz.state()');
  check('unblock clears the link; the second poll flipped the route to 2 via 2', s4.blocked.length === 0 && s4.routes['0']['2'].nh === 2, JSON.stringify({ b: s4.blocked, r: s4.routes['0'] }));

  await page.eval('window.__viz.seek(14.5)');
  const s5 = await page.eval('window.__viz.state()');
  check('robot 2 is back at the gateway after its next Status', !(s5.silent['0'] || []).includes(2), JSON.stringify(s5.silent));

  // backward seek replays the state tables from zero
  await page.eval('window.__viz.seek(8.5)');
  const s6 = await page.eval('window.__viz.state()');
  check('seeking backwards rebuilds the state (block visible again)', s6.blocked.length === 1 && s6.routes['0']['2'].nh === 1);

  await page.eval('window.__viz.seek(0); window.__viz.speed(5); window.__viz.play()');
  await sleep(400);
  const s7 = await page.eval('window.__viz.state()');
  await page.eval('window.__viz.pause()');
  check('play advances the clock at the chosen speed', s7.playing && s7.t > 1.0 && s7.t < 4.0, `t=${s7.t.toFixed(2)} after 0.4 s at 5x`);

  const errors = await page.eval('window.__pageErrors || []');
  check('no uncaught page errors', errors.length === 0, errors.join('; '));

  console.log(`\n${failed.length ? `${failed.length} failed` : 'all checks passed'}`);
  process.exit(failed.length ? 1 : 0);
}

main().catch((err) => { console.error(`\nharness error: ${err.message}`); process.exit(3); });
