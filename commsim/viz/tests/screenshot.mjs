#!/usr/bin/env node
/**
 * Screenshot the replay page at one moment of one trace (for docs/figures).
 *
 *     node --experimental-websocket commsim/viz/tests/screenshot.mjs \
 *         --trace commsim/results/VIZ_20261020/diamond/trace.json --seek 21.5 \
 *         --select 0 --out commsim/docs/figures/viz_diamond.png [--speed 1]
 *
 * Same harness as viz_test.mjs (headless Chrome over CDP, python http.server
 * at the repo root); see that file's header for the Node/Chrome gotchas.
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
const TRACE = argVal('--trace', 'commsim/viz/testdata/trace_small.json');
// --live: start the demo server instead, stage the AP-relay scenario (robots
// 1 and 2 at the ends of the first aisle, a task posted, a message 1 -> 2)
// and capture after --wait seconds
const LIVE = args.includes('--live');
const WAIT = Number(argVal('--wait', '6'));
const PY = process.env.PYTHON || (process.platform === 'win32' ? 'python' : '/usr/bin/python3');
const SEEK = Number(argVal('--seek', '0'));
const SELECT = argVal('--select', null);
const SPEED = Number(argVal('--speed', '1'));
const OUT = path.resolve(argVal('--out', 'viz.png'));
const HTTP_PORT = Number(argVal('--port', '8112'));
const CDP_PORT = Number(argVal('--cdp-port', '9232'));
const WIDTH = Number(argVal('--width', '1280')), HEIGHT = Number(argVal('--height', '760'));

const children = [];
const spawnTracked = (c, a, o = {}) => { const p = spawn(c, a, { stdio: 'ignore', ...o }); children.push(p); return p; };
process.on('exit', () => { for (const p of children) { try { p.kill('SIGKILL'); } catch {} } });

async function waitFor(fn, what, tries = 80) {
  for (let i = 0; i < tries; i++) { try { if (await fn()) return; } catch {} await sleep(250); }
  throw new Error(`timed out waiting for ${what}`);
}
function chromeBin() {
  if (process.env.CHROME_BIN) return process.env.CHROME_BIN;
  for (const b of ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']) { try { if (spawn(b, ['--version']).pid) return b; } catch {} }
  return 'google-chrome';
}
function needsNoSandbox() {
  try { return fs.readFileSync('/proc/sys/kernel/apparmor_restrict_unprivileged_userns', 'utf8').trim() === '1'; } catch { return false; }
}
function cdp(wsUrl) {
  const ws = new WebSocket(wsUrl); let id = 0; const pending = new Map();
  const ready = new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.onmessage = (m) => { const d = JSON.parse(m.data); if (d.id && pending.has(d.id)) { const { res, rej } = pending.get(d.id); pending.delete(d.id); d.error ? rej(new Error(JSON.stringify(d.error))) : res(d.result); } };
  const send = (method, params = {}) => new Promise((res, rej) => { const i = ++id; pending.set(i, { res, rej }); ws.send(JSON.stringify({ id: i, method, params })); });
  const evalJs = async (expr) => { const r = await send('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true }); if (r.exceptionDetails) throw new Error(r.exceptionDetails.text); return r.result.value; };
  return { ready, send, eval: evalJs };
}

async function main() {
  if (LIVE) spawnTracked(PY, ['-m', 'commsim.viz.demo', '--port', String(HTTP_PORT), '--n', argVal('--n', '4'), '--range-m', argVal('--range', '22')], { cwd: REPO_ROOT });
  else spawnTracked('python3', ['-m', 'http.server', String(HTTP_PORT), '--bind', '127.0.0.1'], { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${HTTP_PORT}/${LIVE ? '' : 'commsim/viz/index.html'}`)).ok, 'server');
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'commsim-viz-shot-'));
  const flags = ['--headless=new', `--remote-debugging-port=${CDP_PORT}`, '--enable-unsafe-swiftshader', '--no-first-run',
                 '--no-default-browser-check', '--disable-gpu-sandbox', `--user-data-dir=${profile}`, `--window-size=${WIDTH},${HEIGHT}`, 'about:blank'];
  if (needsNoSandbox()) flags.unshift('--no-sandbox');
  spawnTracked(chromeBin(), flags);
  let target;
  await waitFor(async () => { const l = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json(); target = l.find((t) => t.type === 'page'); return !!target; }, 'chrome');
  const page = cdp(target.webSocketDebuggerUrl);
  await page.ready;
  await page.send('Page.enable'); await page.send('Runtime.enable');
  await page.send('Emulation.setDeviceMetricsOverride', { width: WIDTH, height: HEIGHT, deviceScaleFactor: 1, mobile: false });
  if (LIVE) {
    await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/` });
    await waitFor(async () => page.eval('window.__viz && window.__viz.state().loaded'), 'live init', 240);
    await page.eval('window.__viz.cmd({op:"move", id:1, x:6, y:1.5}); window.__viz.cmd({op:"move", id:2, x:44, y:1.5});'
                    + 'window.__viz.cmd({op:"move", id:3, x:14, y:7.5}); window.__viz.cmd({op:"move", id:4, x:36, y:13.5});');
    await sleep(2500);
    await page.eval('window.__viz.cmd({op:"task"}); window.__viz.cmd({op:"obstacle", id:3});');
    await sleep(1500);
    await page.eval('window.__viz.cmd({op:"send", src:1, dst:2}); window.__viz.cmd({op:"send", src:2, dst:1});');
    await sleep(WAIT * 1000);
    if (SELECT != null) await page.eval(`window.__viz.select(${Number(SELECT)})`);
    await sleep(200);
  } else {
    const rel = path.relative(REPO_ROOT, path.resolve(TRACE)).split(path.sep).join('/');
    await page.send('Page.navigate', { url: `http://127.0.0.1:${HTTP_PORT}/commsim/viz/index.html?trace=/${rel}` });
    await waitFor(async () => page.eval('window.__viz && window.__viz.state().loaded'), 'trace load', 240);
    await page.eval(`window.__viz.speed(${SPEED}); window.__viz.seek(${SEEK});` + (SELECT != null ? ` window.__viz.select(${Number(SELECT)});` : ''));
    await sleep(300);
  }
  const shot = await page.send('Page.captureScreenshot', { format: 'png' });
  fs.writeFileSync(OUT, Buffer.from(shot.data, 'base64'));
  console.log(`wrote ${OUT} (${JSON.stringify(await page.eval('(() => { const s = window.__viz.state(); return { t: s.t, active: s.active, blocked: s.blocked, silent: s.silent }; })()'))})`);
  process.exit(0);
}
main().catch((e) => { console.error(e.message); process.exit(3); });
