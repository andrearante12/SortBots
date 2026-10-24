#!/usr/bin/env node
/**
 * Headless test for the live demo (commsim/viz/demo.py + demo.html).
 *
 *     node --experimental-websocket commsim/viz/tests/demo_test.mjs [--port N] [--cdp-port N]
 *
 * Starts the demo server (the real DES, 4 robots, 22 m open-air range),
 * opens the page in headless Chrome, then does what a viewer would: puts
 * robot 1 and robot 2 at opposite ends of the first aisle with the gateway
 * between them, sends a message 1 -> 2 and checks it went through the
 * gateway; posts a task and checks a robot won it; pulls a link; kills a
 * robot. Same harness notes as viz_test.mjs (Node 20 --experimental-websocket,
 * $CHROME_BIN, --no-sandbox under AppArmor userns restrictions).
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
const PORT = Number(argVal('--port', '8771'));
const CDP_PORT = Number(argVal('--cdp-port', '9241'));
const PY = process.env.PYTHON || (process.platform === 'win32' ? 'python' : '/usr/bin/python3');

const children = [];
const spawnTracked = (c, a, o = {}) => { const p = spawn(c, a, { stdio: 'ignore', ...o }); children.push(p); return p; };
process.on('exit', () => { for (const p of children) { try { p.kill('SIGKILL'); } catch {} } });
process.on('SIGINT', () => process.exit(130));

const failed = [];
function check(name, ok, detail = '') { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`); if (!ok) failed.push(name); }
async function waitFor(fn, what, tries = 80, delay = 250) { for (let i = 0; i < tries; i++) { try { if (await fn()) return; } catch {} await sleep(delay); } throw new Error(`timed out waiting for ${what}`); }
function chromeBin() { if (process.env.CHROME_BIN) return process.env.CHROME_BIN; for (const b of ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser']) { try { if (spawn(b, ['--version']).pid) return b; } catch {} } return 'google-chrome'; }
function needsNoSandbox() { try { return fs.readFileSync('/proc/sys/kernel/apparmor_restrict_unprivileged_userns', 'utf8').trim() === '1'; } catch { return false; } }
function cdp(wsUrl) {
  const ws = new WebSocket(wsUrl); let id = 0; const pending = new Map();
  const ready = new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.onmessage = (m) => { const d = JSON.parse(m.data); if (d.id && pending.has(d.id)) { const { res, rej } = pending.get(d.id); pending.delete(d.id); d.error ? rej(new Error(JSON.stringify(d.error))) : res(d.result); } };
  const send = (method, params = {}) => new Promise((res, rej) => { const i = ++id; pending.set(i, { res, rej }); ws.send(JSON.stringify({ id: i, method, params })); });
  const evalJs = async (expr) => { const r = await send('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true }); if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description || r.exceptionDetails.text); return r.result.value; };
  return { ready, send, eval: evalJs };
}

async function main() {
  spawnTracked(PY, ['-m', 'commsim.viz.demo', '--port', String(PORT), '--n', '4', '--range-m', '22'], { cwd: REPO_ROOT });
  await waitFor(async () => (await fetch(`http://127.0.0.1:${PORT}/`)).ok, 'demo server');

  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'commsim-demo-chrome-'));
  const flags = ['--headless=new', `--remote-debugging-port=${CDP_PORT}`, '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check', '--disable-gpu-sandbox', `--user-data-dir=${profile}`, '--window-size=1400,900', 'about:blank'];
  if (needsNoSandbox()) flags.unshift('--no-sandbox');
  spawnTracked(chromeBin(), flags);
  let target;
  await waitFor(async () => { const l = await (await fetch(`http://127.0.0.1:${CDP_PORT}/json/list`)).json(); target = l.find((t) => t.type === 'page'); return !!target; }, 'chrome');
  const page = cdp(target.webSocketDebuggerUrl);
  await page.ready;
  await page.send('Page.enable'); await page.send('Runtime.enable');
  await page.send('Emulation.setDeviceMetricsOverride', { width: 1400, height: 900, deviceScaleFactor: 1, mobile: false });
  await page.send('Page.navigate', { url: `http://127.0.0.1:${PORT}/` });
  await waitFor(async () => page.eval('window.__viz && window.__viz.state().loaded && window.__viz.state().live'), 'live init');

  const s0 = await page.eval('window.__viz.state()');
  check('live page connects: 4 robots + gateway, clock running', s0.live && s0.n === 5 && s0.playing, JSON.stringify({ n: s0.n, playing: s0.playing }));
  await sleep(1500);
  const s1 = await page.eval('window.__viz.state()');
  check('the model is stepping: time advances and frames arrive', s1.t > 0.5 && s1.airCount > 20, `t=${s1.t.toFixed(2)} air=${s1.airCount}`);

  // robot 1 at the west end of aisle 0, robot 2 at the east end, gateway in the middle
  await page.eval('window.__viz.cmd({op:"move", id:1, x:6, y:1.5}); window.__viz.cmd({op:"move", id:2, x:44, y:1.5})');
  await waitFor(async () => { const s = await page.eval('window.__viz.state()'); return s.routes['1'] && s.routes['1']['2'] && s.routes['1']['2'].nh === 0; }, 'route 1 -> 2 via the gateway', 60);
  const s2 = await page.eval('window.__viz.state()');
  check('robots 1 and 2 out of each other\'s range route through the gateway', s2.routes['1']['2'].nh === 0 && s2.routes['2']['1'].nh === 0, JSON.stringify(s2.routes['1']));
  check('the page moved the robots where asked', Math.abs(s2.nodes[1][0] - 6) < 0.01 && Math.abs(s2.nodes[2][0] - 44) < 0.01, JSON.stringify([s2.nodes[1], s2.nodes[2]]));

  await page.eval('window.__viz.cmd({op:"send", src:1, dst:2, size:200})');
  await waitFor(async () => (await page.eval('window.__viz.state()')).feed.some((f) => /robot 2 got Custom from robot 1/.test(f)), 'message 1 -> 2 delivered', 40);
  const line = (await page.eval('window.__viz.state()')).feed.find((f) => /robot 2 got Custom from robot 1/.test(f));
  check('a message from robot 1 reaches robot 2 in two hops (1 -> gateway -> 2)', /2 hops/.test(line), line);

  await page.eval('window.__viz.cmd({op:"task"})');
  await waitFor(async () => (await page.eval('window.__viz.state()')).tasks.some((t) => t.state === 'claimed' || t.state === 'carrying'), 'a robot claims the task', 60);
  const s3 = await page.eval('window.__viz.state()');
  const tk = s3.tasks.find((t) => t.state === 'claimed' || t.state === 'carrying');
  check('the auction ran: a robot owns the task and the feed says who won', tk && tk.owner > 0 && s3.feed.some((f) => /won task/.test(f)), JSON.stringify(tk));

  await page.eval('window.__viz.cmd({op:"obstacle", id:1})');
  await waitFor(async () => (await page.eval('window.__viz.state()')).feed.some((f) => /saw an obstacle/.test(f)), 'obstacle reported', 40);
  check('an obstacle seen by a robot is announced to the fleet', true);

  await page.eval('window.__viz.cmd({op:"link", a:0, b:1, action:"block"})');
  await waitFor(async () => (await page.eval('window.__viz.state()')).blocked.includes('0-1'), 'link blocked', 40);
  check('a pulled link shows as blocked', true);
  await page.eval('window.__viz.cmd({op:"link", a:0, b:1, action:"unblock"})');

  await page.eval('window.__viz.cmd({op:"node", id:3, action:"kill"})');
  await waitFor(async () => (await page.eval('window.__viz.state()')).dead.includes(3), 'robot killed', 40);
  check('a switched-off robot is marked dead', true);
  await page.eval('window.__viz.cmd({op:"node", id:3, action:"revive"})');

  await page.eval('window.__viz.cmd({op:"range", metres:60})');
  await waitFor(async () => { const s = await page.eval('window.__viz.state()'); return s.routes['1'] && s.routes['1']['2'] && s.routes['1']['2'].nh === 2; }, 'direct route after range increase', 60);
  check('raising the radio range makes 1 -> 2 direct again', true);

  const errors = await page.eval('window.__pageErrors || []');
  check('no uncaught page errors', errors.length === 0, errors.join('; '));
  console.log(`\n${failed.length ? `${failed.length} failed` : 'all checks passed'}`);
  process.exit(failed.length ? 1 : 0);
}
main().catch((err) => { console.error(`\nharness error: ${err.message}`); process.exit(3); });
