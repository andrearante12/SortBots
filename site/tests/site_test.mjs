/* Offline structural checks for the docs site.
 *
 * Pure Node, no browser, no npm deps, no network -- same shape as
 * webui/tests/*.mjs. Runs in well under a second.
 *
 *   node site/tests/site_test.mjs
 *
 * What it is actually protecting:
 *   - the GLB and the part map drifting apart when the model is rebuilt
 *   - a section losing its plain-language layer or its under-the-hood layer
 *   - an external URL sneaking into the page, which would break the offline
 *     guarantee and leak a request to a third party
 */

import { readFileSync, existsSync, readdirSync, statSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const SITE = join(dirname(fileURLToPath(import.meta.url)), '..');

let failures = 0;
let checks = 0;

function check(name, cond, detail = '') {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${name}${detail ? `\n        ${detail}` : ''}`);
}

function group(name) {
  console.log(`\n${name}`);
}

/* ------------------------------------------------------------ the model --- */

group('model');

const glbPath = join(SITE, 'assets/xlerobot.glb');
check('assets/xlerobot.glb exists', existsSync(glbPath),
  'run: python3 site/build/urdf_to_glb.py');

let glbNodeNames = new Set();
if (existsSync(glbPath)) {
  const size = statSync(glbPath).size;
  // Budget, not a limit for its own sake: the model is fetched before the page
  // can show the robot, and the Draco pass has historically landed near 250 KB.
  check('GLB is within the 2 MB budget', size < 2_000_000, `${(size / 1024).toFixed(0)} KB`);

  const buf = readFileSync(glbPath);
  check('GLB magic is glTF', buf.subarray(0, 4).toString('ascii') === 'glTF');
  const jsonLen = buf.readUInt32LE(12);
  const doc = JSON.parse(buf.subarray(20, 20 + jsonLen).toString('utf8'));
  glbNodeNames = new Set((doc.nodes || []).map((n) => n.name).filter(Boolean));

  check('GLB keeps its named nodes', glbNodeNames.size >= 39,
    `${glbNodeNames.size} named nodes -- the optimize step must keep join/flatten/instance off`);
  check('GLB is Draco compressed',
    (doc.extensionsRequired || []).includes('KHR_draco_mesh_compression'));
}

/* -------------------------------------------------------------- parts.js --- */

group('parts');

const { PART_GROUPS, HOTSPOTS, NODE_TO_PART } = await import(join(SITE, 'js/parts.js'));

for (const [id, g] of Object.entries(PART_GROUPS)) {
  check(`part "${id}" has a label and detail`, !!g.label && !!g.detail);
  check(`part "${id}" lists nodes`, Array.isArray(g.nodes) && g.nodes.length > 0);
  if (glbNodeNames.size) {
    const missing = g.nodes.filter((n) => !glbNodeNames.has(n));
    check(`part "${id}" nodes all exist in the GLB`, missing.length === 0,
      missing.length ? `missing: ${missing.join(', ')}` : '');
  }
}

check('every hotspot resolves to a part group',
  HOTSPOTS.every((h) => PART_GROUPS[h.part]),
  HOTSPOTS.filter((h) => !PART_GROUPS[h.part]).map((h) => h.part).join(', '));

// Overlapping hotspots would fight over the same meshes under the cursor.
const claimed = new Map();
for (const h of HOTSPOTS) {
  for (const node of PART_GROUPS[h.part]?.nodes || []) {
    check(`node "${node}" is claimed by only one hotspot`, !claimed.has(node),
      `also claimed by "${claimed.get(node)}"`);
    claimed.set(node, h.part);
  }
}

check('NODE_TO_PART excludes the overlapping grippers group',
  !([...NODE_TO_PART.values()].includes('grippers')));

/* --------------------------------------------------------- the content --- */

group('content');

const { SUBSYSTEMS, HERO } = await import(join(SITE, 'content/subsystems.js'));

check('hero has a title and tagline', !!HERO.title && !!HERO.tagline);
check('there are subsystem sections', SUBSYSTEMS.length >= 4);

const ids = new Set();
for (const s of SUBSYSTEMS) {
  const at = `section "${s.id}"`;
  check(`${at} has a unique id`, s.id && !ids.has(s.id));
  ids.add(s.id);
  check(`${at} has an eyebrow, namespace and title`, !!s.eyebrow && !!s.ns && !!s.title);

  // The two-layer promise: a plain-language layer and a technical layer.
  check(`${at} has a plain-language lede`, Array.isArray(s.lede) && s.lede.length >= 2);
  check(`${at} has an under-the-hood layer`, !!s.deep && (s.deep.tables?.length || s.deep.code));
  check(`${at} has at least one field note`, Array.isArray(s.notes) && s.notes.length >= 1);

  for (const n of s.notes || []) {
    check(`${at} field note has a title and body`, !!n.title && !!n.body);
    check(`${at} field note date is ISO or null`, n.date == null || /^\d{4}-\d{2}-\d{2}$/.test(n.date), String(n.date));
  }

  for (const p of s.parts || []) {
    check(`${at} references a real part group ("${p}")`, !!PART_GROUPS[p]);
  }

  for (const t of s.deep?.tables || []) {
    check(`${at} table "${t.caption}" has a header`, Array.isArray(t.head) && t.head.length > 0);
    const bad = (t.rows || []).filter((r) => r.length !== t.head.length);
    check(`${at} table "${t.caption}" rows match the header width`, bad.length === 0,
      bad.length ? `${bad.length} row(s) of the wrong width` : '');
  }

  if (s.diagram) {
    const nodeIds = new Set(s.diagram.nodes.map((n) => n.id));
    for (const e of s.diagram.edges) {
      check(`${at} diagram edge endpoints exist`, nodeIds.has(e.from) && nodeIds.has(e.to),
        `${e.from} -> ${e.to}`);
    }
    for (const n of s.diagram.nodes) {
      check(`${at} diagram node "${n.id}" is on the 0-100 grid`,
        n.x >= 0 && n.x <= 100 && n.y >= 0 && n.y <= 100);
    }
    check(`${at} diagram has a caption`, !!s.diagram.caption);
  }
}

// Sections named in the camera framing table must exist, or the camera silently
// falls back to the hero framing for that section.
const robotSrc = readFileSync(join(SITE, 'js/robot.js'), 'utf8');
const framingBlock = robotSrc.slice(robotSrc.indexOf('const FRAMINGS'), robotSrc.indexOf('export function supportsWebGL'));
for (const s of SUBSYSTEMS) {
  check(`section "${s.id}" has a camera framing`, framingBlock.includes(`${s.id}:`));
}

/* ------------------------------------------------------- offline promise --- */

group('offline');

function walk(dir) {
  const out = [];
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const p = join(dir, entry.name);
    if (entry.isDirectory()) {
      if (entry.name === 'vendor' || entry.name === 'assets') continue;
      out.push(...walk(p));
    } else if (/\.(html|css|js|mjs)$/.test(entry.name)) {
      out.push(p);
    }
  }
  return out;
}

// A single external reference breaks the offline guarantee and sends a request
// to a third party from a page that is meant to be self-contained.
const EXTERNAL = /(?:src|href)\s*=\s*["']https?:\/\/|url\(\s*["']?https?:\/\/|(?:fetch|import)\(\s*["']https?:\/\//g;
for (const file of walk(SITE)) {
  const text = readFileSync(file, 'utf8');
  const hits = text.match(EXTERNAL) || [];
  check(`${file.replace(SITE + '/', '')} makes no external requests`, hits.length === 0,
    hits.join(', '));
}

const html = readFileSync(join(SITE, 'index.html'), 'utf8');
check('index.html declares an import map', html.includes('type="importmap"'));
check('index.html has a poster fallback', html.includes('stage-poster'));
check('index.html has a skip link', html.includes('skip-link'));
check('vendored three.js is present', existsSync(join(SITE, 'vendor/three/three.module.js')));
check('draco decoder is vendored', existsSync(join(SITE, 'vendor/three/draco/draco_decoder.wasm')));

// The reduced-motion and focus commitments are cheap to assert and easy to
// delete by accident during a restyle.
const css = readFileSync(join(SITE, 'styles.css'), 'utf8');
check('styles honour prefers-reduced-motion', css.includes('prefers-reduced-motion'));
check('styles define a visible focus ring', css.includes(':focus-visible'));

group('layout');

// Every element wide enough to overflow a phone must sit in its own scroll
// container, or the whole page scrolls sideways. Verified live at 390px:
// #section-nav is a flex item, and without min-width:0 it refused to shrink
// below its eight links and widened the masthead past the viewport.
check('the section nav can shrink below its content',
  /#section-nav\s*\{[^}]*min-width:\s*0/s.test(css),
  'a flex item defaults to min-width:auto and will widen the page on a phone');

for (const [selector, container] of [['.deep table', '.table-wrap'], ['.diagram-wrap svg', '.diagram-wrap']]) {
  const rule = new RegExp(`\\${selector.replace(/[.\s]/g, (m) => (m === '.' ? '\\.' : '\\s'))}\\s*\\{[^}]*min-width:\\s*\\d`, 's');
  if (!rule.test(css)) continue;
  const guard = new RegExp(`\\${container.replace('.', '\\.')}[^{]*\\{[^}]*overflow-x:\\s*auto`, 's');
  check(`${selector} (fixed min-width) scrolls inside ${container}`, guard.test(css));
}

check('the code block scrolls rather than stretching the page',
  /\.code-block\s*\{[^}]*overflow-x:\s*auto/s.test(css));

/* ---------------------------------------------------------------- done --- */

console.log(`\n${checks - failures}/${checks} checks passed`);
if (failures) {
  console.error(`${failures} FAILED`);
  process.exit(1);
}
