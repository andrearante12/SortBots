/* Page orchestration: builds the sections, wires the stage to scroll position,
 * and keeps the anatomy hotspots in sync with the 3D scene.
 *
 * Load order matters. Sections are built and inserted BEFORE the 3D stage is
 * initialised, so the page is fully readable while the model is still being
 * fetched and decoded -- and stays readable if it never arrives.
 */

import { SUBSYSTEMS, HERO } from '../content/subsystems.js';
import { renderSection } from './panels.js';
import { PART_GROUPS, HOTSPOTS } from './parts.js';
import * as robot3d from './robot.js';

const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

/* ----------------------------------------------------------------- hero --- */

function buildHero() {
  document.getElementById('hero-title').textContent = HERO.title;
  document.getElementById('hero-tagline').textContent = HERO.tagline;
  const meta = document.getElementById('hero-meta');
  for (const m of HERO.meta) {
    const div = document.createElement('div');
    const dt = document.createElement('dt');
    dt.textContent = m.k;
    const dd = document.createElement('dd');
    dd.textContent = m.v;
    div.append(dt, dd);
    meta.appendChild(div);
  }
}

/* ------------------------------------------------------------------ nav --- */

function buildNav() {
  const nav = document.getElementById('section-nav');
  for (const s of SUBSYSTEMS) {
    const a = document.createElement('a');
    a.href = `#${s.id}`;
    a.textContent = s.eyebrow.split(' & ')[0];
    a.dataset.target = s.id;
    nav.appendChild(a);
  }
}

/* -------------------------------------------------------------- anatomy --- */

/** The real, focusable hotspot controls. These are the accessible path into
 *  every subsystem; the dots floating over the canvas mirror them. */
function buildAnatomyList(onSelect) {
  const list = document.getElementById('hotspot-list');
  for (const { part, subsystem } of HOTSPOTS) {
    const group = PART_GROUPS[part];
    if (!group) continue;
    const li = document.createElement('li');
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.dataset.part = part;
    btn.dataset.subsystem = subsystem;
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = group.label;
    const detail = document.createElement('span');
    detail.className = 'detail';
    detail.textContent = group.detail;
    btn.append(name, detail);
    btn.addEventListener('click', () => onSelect(part, subsystem));
    btn.addEventListener('mouseenter', () => onSelect(part, subsystem, { preview: true }));
    li.appendChild(btn);
    list.appendChild(li);
  }
}

/** Floating labels over the canvas, positioned each frame by robot.js. */
function buildHotspotDots(stage, onSelect) {
  const layer = document.getElementById('hotspot-layer');
  for (const { part, subsystem, side } of HOTSPOTS) {
    const group = PART_GROUPS[part];
    if (!group || !stage.parts.includes(part)) continue;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = side === 'left' ? 'hotspot-dot hotspot-dot--left' : 'hotspot-dot';
    // The list buttons carry the accessible names; these are a visual mirror of
    // the same actions, so they stay out of the tab order and the a11y tree.
    btn.tabIndex = -1;
    btn.setAttribute('aria-hidden', 'true');
    const label = document.createElement('span');
    label.className = 'hotspot-dot__label';
    label.textContent = group.label;
    btn.appendChild(label);
    btn.addEventListener('click', () => onSelect(part, subsystem));
    layer.appendChild(btn);
    stage.registerHotspot(part, btn);
  }
}

/* ------------------------------------------------------------ observers --- */

function observeReveals() {
  const io = new IntersectionObserver((entries) => {
    for (const e of entries) {
      if (e.isIntersecting) {
        e.target.classList.add('is-in');
        io.unobserve(e.target);
      }
    }
  }, { rootMargin: '0px 0px -12% 0px' });
  document.querySelectorAll('.reveal').forEach((n) => io.observe(n));
}

/** Drives the camera framing and the nav highlight from whichever section owns
 *  the middle of the viewport. */
function observeSections(stage) {
  const nav = document.getElementById('section-nav');
  const stageEl = document.getElementById('stage');
  const sections = [...document.querySelectorAll('[data-section], #hero, #anatomy, #colophon')];

  let current = null;
  let first = true;
  const io = new IntersectionObserver((entries) => {
    // Pick the most visible intersecting section rather than the last event:
    // at a section boundary two entries fire in an order that is not stable
    // across browsers, which made the camera jump back and forth.
    let best = null;
    for (const e of entries) if (e.isIntersecting && (!best || e.intersectionRatio > best.intersectionRatio)) best = e;
    if (!best) return;
    const id = best.target.id;
    if (id === current) return;
    current = id;

    // The first section to win is wherever the page actually opened -- top of
    // the document, or a #hash target. Arrive there rather than easing in.
    if (stage) stage.setSection(id, { immediate: first });
    first = false;

    const isAnatomy = id === 'anatomy';
    stageEl.classList.toggle('is-anatomy', isAnatomy);
    if (stage) stage.setInteractive(isAnatomy);

    // Outside the anatomy section, emphasise every part the section owns so a
    // close-up of both arms (or chassis + wheels) does not ghost half of itself.
    const data = SUBSYSTEMS.find((s) => s.id === id);
    if (stage) {
      if (isAnatomy || !data) stage.setEmphasis(null);
      else stage.setEmphasis(data.parts || [], { ghost: true });
    }

    for (const a of nav.querySelectorAll('a')) a.classList.toggle('is-current', a.dataset.target === id);
  }, { threshold: [0.25, 0.55, 0.8], rootMargin: '-18% 0px -35% 0px' });

  sections.forEach((s) => io.observe(s));
}

function observeMasthead() {
  const masthead = document.getElementById('masthead');
  const io = new IntersectionObserver(
    ([e]) => masthead.classList.toggle('is-stuck', !e.isIntersecting),
    { threshold: 1 }
  );
  const sentinel = document.createElement('div');
  sentinel.style.cssText = 'position:absolute;top:0;height:1px;width:1px;';
  document.body.prepend(sentinel);
  io.observe(sentinel);
}

/* ----------------------------------------------------------------- boot --- */

async function main() {
  buildHero();
  buildNav();

  // Sections first: the page must be complete and readable before any 3D work
  // starts, and must stay that way if the stage never initialises.
  const host = document.getElementById('sections');
  const built = await Promise.all(SUBSYSTEMS.map(renderSection));
  for (const node of built) host.appendChild(node);

  observeReveals();
  observeMasthead();

  const stageEl = document.getElementById('stage');

  // Reduced motion: no camera choreography, no turntable, no WebGL context at
  // all. The poster carries the visual, and every hotspot still works as a
  // link into its subsystem.
  if (reduceMotion) {
    stageEl.classList.add('is-fallback');
    buildAnatomyList((part, subsystem) => { location.hash = `#${subsystem}`; });
    observeSections(null);
    return;
  }

  const stage = await robot3d.init({
    canvas: document.getElementById('robot-canvas'),
    hotspotLayer: document.getElementById('hotspot-layer'),
    onPick: (part) => {
      const hit = HOTSPOTS.find((h) => h.part === part);
      if (hit) select(part, hit.subsystem);
    },
  });

  if (!stage) {
    stageEl.classList.add('is-fallback');
    buildAnatomyList((part, subsystem) => { location.hash = `#${subsystem}`; });
    observeSections(null);
    return;
  }

  let previewTimer = 0;
  function select(part, subsystem, { preview = false } = {}) {
    stage.setEmphasis(part, { ghost: true });
    for (const btn of document.querySelectorAll('#hotspot-list button')) {
      btn.classList.toggle('is-active', btn.dataset.part === part);
    }
    if (preview) {
      // A hover preview should decay on its own; a click should not.
      clearTimeout(previewTimer);
      previewTimer = setTimeout(() => stage.setEmphasis(null), 1600);
      return;
    }
    clearTimeout(previewTimer);
    document.getElementById(subsystem)?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  }

  buildAnatomyList(select);
  buildHotspotDots(stage, select);
  observeSections(stage);

  // Debug handle. The camera framings in robot.js are tuned by eye against a
  // live page, and there is no other way to read back where the robot actually
  // landed. Namespaced, read-only in practice, and harmless if unused.
  window.__sortbots = stage;
}

main().catch((err) => {
  // A failure here must not leave sections stuck at opacity 0 mid-reveal.
  console.error('[site] initialisation failed', err);
  document.querySelectorAll('.reveal').forEach((n) => n.classList.add('is-in'));
  document.getElementById('stage')?.classList.add('is-fallback');
});
