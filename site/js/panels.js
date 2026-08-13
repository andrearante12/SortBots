/* Renderers: subsystem sections, flow diagrams, and the map scrubber.
 *
 * All of these build DOM from the data in content/subsystems.js. Prose fields
 * are authored HTML (they carry <code>/<em>), so they go in with innerHTML;
 * everything derived from a value -- labels, numbers, table cells -- goes in as
 * text so a stray character in a constant can't break the page.
 */

const SVG_NS = 'http://www.w3.org/2000/svg';

/* ------------------------------------------------------------- diagrams --- */

/* Node/edge flow diagrams on a 0-100 coordinate grid, scaled to a 1000x420
 * viewBox. Positions are authored by hand in the content file rather than
 * computed: these are five to seven boxes each, and a layout engine would cost
 * more than it saves while giving up control of the reading order.
 *
 * Colour is information here, matching the palette's meaning: cyan edges carry
 * sensor data, amber edges carry intent and commands. */
const VB_W = 1000, VB_H = 420;
const BOX_W = 168, BOX_H = 62;

function el(name, attrs = {}, text) {
  const node = document.createElementNS(SVG_NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (text != null) node.textContent = text;
  return node;
}

function boxRect(node) {
  const cx = (node.x / 100) * VB_W + BOX_W / 2;
  const cy = (node.y / 100) * VB_H + BOX_H / 2;
  return { cx, cy, x: cx - BOX_W / 2, y: cy - BOX_H / 2, w: BOX_W, h: BOX_H };
}

/** Exit point on the edge of a box, aimed at a target point. */
function anchor(rect, tx, ty) {
  const dx = tx - rect.cx, dy = ty - rect.cy;
  // Leave horizontally unless the vertical offset clearly dominates -- these
  // diagrams read left to right, and a mostly-sideways edge that leaves from
  // the top of a box looks like it belongs to a different node.
  if (Math.abs(dx) * 0.85 >= Math.abs(dy)) {
    return { x: rect.cx + Math.sign(dx) * (rect.w / 2), y: rect.cy };
  }
  return { x: rect.cx, y: rect.cy + Math.sign(dy) * (rect.h / 2) };
}

export function renderDiagram(spec) {
  const wrap = document.createElement('figure');
  wrap.className = 'diagram-wrap';

  // Fit the viewBox to the boxes rather than using the full authoring grid.
  // Node positions are hand-placed per diagram, so a fixed viewBox leaves a
  // different amount of dead margin in every card and the clusters read as
  // off-centre. Padding covers edge labels, which sit outside the boxes.
  const pad = 26;
  const boxes = spec.nodes.map(boxRect);
  const minX = Math.min(...boxes.map((b) => b.x)) - pad;
  const maxX = Math.max(...boxes.map((b) => b.x + b.w)) + pad;
  const minY = Math.min(...boxes.map((b) => b.y)) - pad;
  const maxY = Math.max(...boxes.map((b) => b.y + b.h)) + pad;

  const svg = el('svg', {
    viewBox: `${minX} ${minY} ${maxX - minX} ${maxY - minY}`,
    role: 'img',
    'aria-label': spec.caption,
  });

  // One marker per edge colour. Markers can't inherit the path's stroke in all
  // engines, so the fill is set explicitly on each.
  const defs = el('defs');
  for (const [id, color] of [['ar-data', 'var(--cyan)'], ['ar-intent', 'var(--amber)']]) {
    const marker = el('marker', {
      id, viewBox: '0 0 10 10', refX: '9', refY: '5',
      markerWidth: '6', markerHeight: '6', orient: 'auto-start-reverse',
    });
    marker.appendChild(el('path', { d: 'M 0 0 L 10 5 L 0 10 z', fill: color }));
    defs.appendChild(marker);
  }
  svg.appendChild(defs);

  const rects = new Map(spec.nodes.map((n) => [n.id, boxRect(n)]));

  for (const edge of spec.edges) {
    const a = rects.get(edge.from), b = rects.get(edge.to);
    if (!a || !b) continue;
    const from = anchor(a, b.cx, b.cy);
    const to = anchor(b, a.cx, a.cy);
    // Horizontal-tangent cubic: keeps the curve flowing left-to-right even when
    // the two boxes sit on different rows.
    const dx = Math.max(Math.abs(to.x - from.x) * 0.5, 34);
    const path = el('path', {
      d: `M ${from.x} ${from.y} C ${from.x + Math.sign(to.x - from.x || 1) * dx} ${from.y}, ` +
         `${to.x - Math.sign(to.x - from.x || 1) * dx} ${to.y}, ${to.x} ${to.y}`,
      class: `dg-edge${edge.kind === 'intent' ? ' dg-edge--intent' : ''}`,
      'marker-end': `url(#${edge.kind === 'intent' ? 'ar-intent' : 'ar-data'})`,
    });
    svg.appendChild(path);
    if (edge.label) {
      const mx = (from.x + to.x) / 2, my = (from.y + to.y) / 2 - 7;
      const t = el('text', { x: mx, y: my, class: 'dg-edge-label', 'text-anchor': 'middle' }, edge.label);
      svg.appendChild(t);
    }
  }

  for (const node of spec.nodes) {
    const r = rects.get(node.id);
    const g = el('g');
    g.appendChild(el('rect', {
      x: r.x, y: r.y, width: r.w, height: r.h, rx: 12,
      class: `dg-box dg-box--${node.kind}`,
    }));
    g.appendChild(el('text', {
      x: r.cx, y: r.cy - (node.sub ? 4 : -4), class: 'dg-label', 'text-anchor': 'middle',
    }, node.label));
    if (node.sub) {
      g.appendChild(el('text', {
        x: r.cx, y: r.cy + 13, class: 'dg-sub', 'text-anchor': 'middle',
      }, node.sub));
    }
    svg.appendChild(g);
  }

  wrap.appendChild(svg);

  const legend = document.createElement('div');
  legend.className = 'dg-legend';
  legend.innerHTML =
    '<span><i style="background:var(--cyan)"></i>sensor data</span>' +
    '<span><i style="background:var(--amber)"></i>intent &amp; commands</span>';
  wrap.appendChild(legend);

  const cap = document.createElement('figcaption');
  cap.className = 'diagram-caption';
  cap.textContent = spec.caption;
  wrap.appendChild(cap);
  return wrap;
}

/* ------------------------------------------------------------- scrubber --- */

/** Map progression scrubber. Fetches the manifest baked by build/bake_media.py. */
export async function renderScrubber() {
  const wrap = document.createElement('div');
  wrap.className = 'scrubber';

  let manifest;
  try {
    const res = await fetch('assets/maps/manifest.json');
    if (!res.ok) throw new Error(res.status);
    manifest = await res.json();
  } catch (err) {
    // The maps are baked from a recorded run that may not be present in a
    // fresh clone. Absent data is not an error worth breaking the page over.
    console.warn('[scrubber] no baked maps; skipping', err);
    return null;
  }

  const frames = manifest.frames || [];
  if (!frames.length) return null;

  const stage = document.createElement('div');
  stage.className = 'scrubber-stage';
  const img = document.createElement('img');
  img.src = frames[0].src;
  img.alt = 'Occupancy grid built during an exploration run';
  img.decoding = 'async';
  stage.appendChild(img);

  // Preload so dragging the slider doesn't flash white between frames.
  for (const f of frames.slice(1)) { const p = new Image(); p.src = f.src; }

  const controls = document.createElement('div');
  controls.className = 'scrubber-controls';
  const range = document.createElement('input');
  range.type = 'range';
  range.min = '0';
  range.max = String(frames.length - 1);
  range.value = '0';
  range.step = '1';
  range.setAttribute('aria-label', 'Exploration progress');
  const readout = document.createElement('p');
  readout.className = 'scrubber-readout';

  function show(i) {
    const f = frames[i];
    img.src = f.src;
    readout.innerHTML = '';
    readout.append(
      document.createTextNode(`${f.label} · `),
      Object.assign(document.createElement('b'), { textContent: `${f.free_m2} m²` }),
      document.createTextNode(' mapped free')
    );
  }
  range.addEventListener('input', () => show(Number(range.value)));
  show(0);

  controls.append(range, readout);
  wrap.append(stage, controls);

  const note = document.createElement('p');
  note.className = 'diagram-caption';
  note.textContent =
    'Checkpoints from one recorded run, aligned onto a shared world canvas — ' +
    'the grid grows as the robot explores, so each frame is pasted at its true ' +
    'world position rather than rescaled. Dark cells are occupied, tinted cells ' +
    'are confirmed free, and the flat background is still unknown.';
  wrap.appendChild(note);
  return wrap;
}

/* ------------------------------------------------------------- sections --- */

function table(spec) {
  const wrap = document.createElement('div');
  wrap.className = 'table-wrap';
  const t = document.createElement('table');
  if (spec.caption) {
    const c = document.createElement('caption');
    c.textContent = spec.caption;
    t.appendChild(c);
  }
  const thead = document.createElement('thead');
  const hr = document.createElement('tr');
  for (const h of spec.head) {
    const th = document.createElement('th');
    th.scope = 'col';
    th.textContent = h;
    hr.appendChild(th);
  }
  thead.appendChild(hr);
  const tbody = document.createElement('tbody');
  for (const row of spec.rows) {
    const tr = document.createElement('tr');
    for (const cell of row) {
      const td = document.createElement('td');
      td.textContent = cell;
      tr.appendChild(td);
    }
    tbody.appendChild(tr);
  }
  t.append(thead, tbody);
  wrap.appendChild(t);
  return wrap;
}

function deepPanel(deep, id) {
  const details = document.createElement('details');
  details.className = 'deep';
  const summary = document.createElement('summary');
  summary.textContent = 'Under the hood';
  details.appendChild(summary);

  const body = document.createElement('div');
  body.className = 'deep-body';
  if (deep?.blurb) {
    const p = document.createElement('p');
    p.innerHTML = deep.blurb;
    body.appendChild(p);
  }
  if (deep.code) {
    const pre = document.createElement('pre');
    pre.className = 'code-block';
    // Comment lines are dimmed so the actual payload shape reads first.
    pre.innerHTML = deep.code.text
      .split('\n')
      .map((line) => {
        const escaped = line.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
        return /^\s*(\/\/|#)/.test(line) ? `<span class="cm">${escaped}</span>` : escaped;
      })
      .join('\n');
    body.appendChild(pre);
  }
  for (const t of deep.tables || []) body.appendChild(table(t));
  details.appendChild(body);
  details.id = `${id}-deep`;
  return details;
}

function fieldNotes(notes) {
  const frag = document.createDocumentFragment();
  const head = document.createElement('p');
  head.className = 'notes-head';
  head.textContent = notes.length === 1 ? 'Field note' : 'Field notes';
  frag.appendChild(head);

  for (const n of notes) {
    const note = document.createElement('div');
    note.className = 'note';
    if (n.date) {
      const d = document.createElement('span');
      d.className = 'note-date';
      d.textContent = `Diagnosed live · ${n.date}`;
      note.appendChild(d);
    }
    const h = document.createElement('h4');
    h.textContent = n.title;
    const p = document.createElement('p');
    p.innerHTML = n.body;
    note.append(h, p);
    frag.appendChild(note);
  }
  return frag;
}

export async function renderSection(data) {
  const section = document.createElement('section');
  section.className = 'section section--subsystem reveal';
  section.id = data.id;
  section.dataset.section = data.id;

  const eyebrow = document.createElement('p');
  eyebrow.className = 'eyebrow';
  eyebrow.textContent = data.eyebrow;

  const ns = document.createElement('p');
  ns.className = 'ns';
  ns.textContent = data.ns;

  const h2 = document.createElement('h2');
  h2.className = 'display display--md';
  h2.textContent = data.title;

  section.append(eyebrow, ns, h2);

  for (const para of data.lede) {
    const p = document.createElement('p');
    p.className = 'lede';
    p.innerHTML = para;
    section.appendChild(p);
  }

  if (data.facts?.length) {
    const facts = document.createElement('div');
    facts.className = 'facts';
    for (const f of data.facts) {
      const d = document.createElement('div');
      const v = document.createElement('span');
      v.className = 'v';
      v.textContent = f.v;
      const k = document.createElement('span');
      k.className = 'k';
      k.textContent = f.k;
      d.append(v, k);
      facts.appendChild(d);
    }
    section.appendChild(facts);
  }

  if (data.scrubber) {
    const s = await renderScrubber();
    if (s) section.appendChild(s);
  }

  // Diagrams, tables, and field notes all live behind one disclosure so the
  // default view of each section is a short lede beside the robot. The
  // distinctive failure-mode notes are still here -- just not competing with
  // the 3D for first attention.
  if (data.deep || data.diagram || data.notes?.length) {
    const details = deepPanel(data.deep || {}, data.id);
    const body = details.querySelector('.deep-body');
    if (data.diagram) body.insertBefore(renderDiagram(data.diagram), body.firstChild);
    if (data.notes?.length) body.appendChild(fieldNotes(data.notes));
    section.appendChild(details);
  }

  return section;
}
