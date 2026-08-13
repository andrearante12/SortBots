/* three.js stage: loads the robot, picks parts, drives the camera.
 *
 * Everything here degrades to nothing gracefully. If WebGL is missing, if the
 * GLB fails to fetch, or if the visitor asked for reduced motion, `init()`
 * returns null and the page runs with a poster image instead -- every part the
 * stage can show is also reachable through the real buttons in the anatomy
 * section, which is why the whole stage is aria-hidden.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { NODE_TO_PART, PART_GROUPS } from './parts.js';

const GLB_URL = 'assets/xlerobot.glb';
const DRACO_PATH = './vendor/three/draco/';

// Ground colour from styles.css --ground. Duplicated rather than read back from
// CSS: the renderer needs it before first paint, and getComputedStyle on a
// custom property returns a string that still needs parsing.
const GROUND = 0xf5f5f7;
const CYAN = 0x0a84c4;
const AMBER = 0xff8c26;

/* Camera framings per section, in the model's own units (metres, Y-up, robot
 * standing on y=0).
 *
 * The stage is clipped to the right half of the viewport above SPLIT_MIN_WIDTH
 * (see styles.css), so the robot can never paint over the copy. No view-offset
 * trick is needed -- the canvas *is* the right column, and the camera looks
 * at the centre of it.
 *
 * `focus` zooms to the bounding box of those part ids (from parts.js). Sections
 * that name a specific piece of hardware use it; the rest keep a full-body
 * framing so the page does not lurch into a random close-up. */
const FRAMINGS = {
  hero:          { pos: [2.0, 1.20, 2.6], target: [0, 0.62, 0], fov: 30 },
  anatomy:       { pos: [2.1, 1.10, 2.3], target: [0, 0.62, 0], fov: 32 },
  perception:    { focus: ['head-camera'], fov: 22, pad: 0.18, dolly: 0.72, dir: [0.45, 0.12, 0.88] },
  navigation:    { focus: ['wheels'], fov: 26, pad: 0.28, dolly: 0.70, single: true, dir: [0.72, 0.38, 0.58] },
  exploration:   { focus: ['head-camera'], fov: 22, pad: 0.22, dolly: 0.70, dir: [0.38, 0.22, 0.90] },
  communication: { pos: [2.4, 1.30, 2.5], target: [0, 0.68, 0], fov: 30 },
  fusion:        { pos: [2.3, 1.20, 2.4], target: [0, 0.58, 0], fov: 31 },
  manipulation:  { focus: ['arm-right'], fov: 24, pad: 0.18, dolly: 0.62, dir: [0.55, 0.15, 0.82] },
  simulation:    { pos: [2.5, 1.35, 2.6], target: [0, 0.60, 0], fov: 30 },
  colophon:      { pos: [2.2, 1.15, 2.4], target: [0, 0.58, 0], fov: 31 },
};

/* Below this width the stage goes full-bleed behind the text (dimmed) instead
 * of occupying the right column. Keep in sync with the 1080px block in CSS. */
const SPLIT_MIN_WIDTH = 1080;

export function supportsWebGL() {
  try {
    const c = document.createElement('canvas');
    return !!(window.WebGLRenderingContext && (c.getContext('webgl2') || c.getContext('webgl')));
  } catch {
    return false;
  }
}

export async function init({ canvas, hotspotLayer, onPick }) {
  if (!supportsWebGL()) return null;

  // Alpha clear so the canvas matches the page ground exactly. ACES tone-mapping
  // a solid #f5f5f7 produced a visible seam down the split -- the mapped clear
  // was a different grey than the CSS, and the robot looked like it was sitting
  // on a different sheet of paper than the copy.
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setClearColor(0x000000, 0);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(33, 1, 0.05, 100);

  // Studio-ish lighting: a soft hemisphere for fill so the dark motor housings
  // never go to pure black, one key with a shadow, one cool rim to separate the
  // silhouette from a near-white background (without it the white printed
  // shells disappear into the page).
  scene.add(new THREE.HemisphereLight(0xffffff, 0xd8d8dd, 2.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(2.6, 4.2, 3.0);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  key.shadow.camera.near = 0.5;
  key.shadow.camera.far = 12;
  key.shadow.camera.left = key.shadow.camera.bottom = -1.6;
  key.shadow.camera.right = key.shadow.camera.top = 1.6;
  key.shadow.bias = -0.0012;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xbcd8ee, 1.1);
  rim.position.set(-3.0, 1.6, -2.4);
  scene.add(rim);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  // Contact shadow only -- a ShadowMaterial plane catches the key light without
  // drawing a visible floor, so the robot reads as standing on the page itself.
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(14, 14),
    new THREE.ShadowMaterial({ opacity: 0.13 })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);

  const root = new THREE.Group();
  scene.add(root);

  const loader = new GLTFLoader();
  const draco = new DRACOLoader();
  draco.setDecoderPath(DRACO_PATH);
  loader.setDRACOLoader(draco);

  let gltf;
  try {
    gltf = await loader.loadAsync(GLB_URL);
  } catch (err) {
    console.error('[robot] could not load the model', err);
    renderer.dispose();
    return null;
  }
  root.add(gltf.scene);

  /* Index meshes by part. Each mesh gets its own material instance so one part
   * can be ghosted without dimming every other part that happened to share a
   * material -- the GLB has only four materials across 39 nodes, so sharing
   * would make part highlighting impossible. */
  const partMeshes = new Map();   // part id -> [mesh]
  const allMeshes = [];
  gltf.scene.traverse((obj) => {
    if (!obj.isMesh) return;
    obj.castShadow = true;
    obj.receiveShadow = true;
    obj.material = obj.material.clone();
    obj.userData.baseColor = obj.material.color.clone();
    obj.userData.baseOpacity = obj.material.opacity ?? 1;

    // Node names survive Draco compression, but three.js sanitises some
    // characters; match on the mesh name and fall back to its parent's.
    const part = NODE_TO_PART.get(obj.name) || NODE_TO_PART.get(obj.parent?.name);
    obj.userData.part = part || null;
    if (part) {
      if (!partMeshes.has(part)) partMeshes.set(part, []);
      partMeshes.get(part).push(obj);
    }
    allMeshes.push(obj);
  });

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.075;
  controls.enablePan = false;
  controls.enableZoom = false;   // the page owns the scroll wheel
  controls.minPolarAngle = 0.55;
  controls.maxPolarAngle = 1.75;
  controls.enabled = false;      // only on in the anatomy section

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let hovered = null;
  let selected = null;
  let ghosting = false;

  /* --- hotspot labels ---------------------------------------------------- */

  const hotspotEls = new Map();  // part id -> element
  const hotspotAnchors = new Map();
  for (const [part, meshes] of partMeshes) {
    const box = new THREE.Box3();
    for (const m of meshes) box.expandByObject(m);
    const c = box.getCenter(new THREE.Vector3());
    // Anchor on the outer edge of the part rather than its centre, so the label
    // sits beside the geometry instead of on top of it.
    hotspotAnchors.set(part, new THREE.Vector3(c.x, box.max.y - (box.max.y - c.y) * 0.35, c.z));
  }

  function registerHotspot(part, el) {
    hotspotEls.set(part, el);
  }

  /* --- highlight / ghost ------------------------------------------------- */

  function setEmphasis(part, { ghost = false } = {}) {
    const wanted = part == null
      ? null
      : new Set(Array.isArray(part) ? part : [part]);
    selected = wanted;
    ghosting = ghost;
    for (const mesh of allMeshes) {
      const isTarget = wanted && wanted.has(mesh.userData.part);
      const mat = mesh.material;
      if (!wanted || !ghost) {
        mat.color.copy(mesh.userData.baseColor);
        mat.opacity = 1;
        mat.transparent = false;
        mat.emissive?.setHex(0x000000);
      } else if (isTarget) {
        mat.color.copy(mesh.userData.baseColor);
        mat.opacity = 1;
        mat.transparent = false;
        mat.emissive?.setHex(0x1a1206);
      } else {
        // Desaturate toward the page ground rather than fading to transparent:
        // transparency across 39 overlapping meshes produces depth-sorting
        // artefacts that read as holes in the robot. 0.72 on a close-up, so
        // the subject reads as the only thing in the frame; the body is still
        // there so the part does not float.
        mat.color.copy(mesh.userData.baseColor).lerp(new THREE.Color(GROUND), 0.72);
        mat.opacity = 1;
        mat.transparent = false;
        mat.emissive?.setHex(0x000000);
      }
      mat.needsUpdate = true;
    }
  }

  function setHover(part) {
    if (hovered === part) return;
    hovered = part;
    for (const [id, el] of hotspotEls) el.classList.toggle('is-hover', id === part);
    canvas.style.cursor = part ? 'pointer' : '';
  }

  /* --- pointer ----------------------------------------------------------- */

  function pick(event) {
    const rect = canvas.getBoundingClientRect();
    pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hit = raycaster.intersectObjects(allMeshes, false)[0];
    return hit?.object.userData.part || null;
  }

  canvas.addEventListener('pointermove', (e) => {
    if (!controls.enabled) return;
    setHover(pick(e));
  });
  canvas.addEventListener('pointerdown', (e) => {
    if (!controls.enabled) return;
    const part = pick(e);
    if (part && onPick) onPick(part);
  });
  canvas.addEventListener('pointerleave', () => setHover(null));

  /* --- camera ------------------------------------------------------------ */

  const desired = {
    pos: new THREE.Vector3(...FRAMINGS.hero.pos),
    target: new THREE.Vector3(...FRAMINGS.hero.target),
    fov: FRAMINGS.hero.fov,
  };
  camera.position.copy(desired.pos);
  controls.target.copy(desired.target);

  let spin = true;          // ambient turntable, on until the visitor takes over
  const _box = new THREE.Box3();
  const _size = new THREE.Vector3();
  const _center = new THREE.Vector3();
  const _dir = new THREE.Vector3();

  /** Frame the union of the named parts, tight enough that the subject fills
   *  the right-hand canvas. Falls back to the hero framing if the meshes have
   *  not been indexed yet (GLB still loading). */
  function framingFromParts(partIds, { fov = 28, pad = 0.4, dir, single = false, dolly = 0.7 } = {}) {
    _box.makeEmpty();
    root.updateMatrixWorld(true);
    for (const id of partIds) {
      const meshes = partMeshes.get(id) || [];
      // `single` is for groups that span the whole robot (the two wheels sit
      // on opposite sides of the cart). Framing both pulls the camera back
      // to a full-cart shot. Use the first *named* node from PART_GROUPS, not
      // meshes[0] -- traverse order put the mast in front of the pan joint
      // and exploration framed a pillar instead of the head.
      if (single) {
        const want = PART_GROUPS[id]?.nodes[0];
        const mesh = meshes.find((m) => m.name === want || m.parent?.name === want) || meshes[0];
        if (mesh) _box.expandByObject(mesh);
        break;
      }
      for (const m of meshes) _box.expandByObject(m);
    }
    if (_box.isEmpty()) return null;
    _box.getCenter(_center);
    _box.getSize(_size);
    const maxDim = Math.max(_size.x, _size.y, _size.z, 0.06);
    // `dolly` < 1 crops the part rather than fitting it — that is the
    // "really close" the page promises. 1.0 would frame the whole bbox.
    const dist = Math.max(
      (maxDim * (0.5 + pad)) / Math.tan(THREE.MathUtils.degToRad(fov) * 0.5) * dolly,
      0.12,
    );
    // Front-right-above by default. A head-on shot of the D435 reads as a
    // black rectangle; a three-quarter view keeps the part recognisable.
    if (dir) _dir.set(...dir).normalize();
    else _dir.set(0.58, 0.22, 0.78).normalize();
    return {
      pos: _center.clone().addScaledVector(_dir, dist),
      target: _center.clone(),
      fov,
    };
  }

  function resolveFraming(name) {
    const f = FRAMINGS[name] || FRAMINGS.hero;
    // Close-ups only make sense in the right-hand column. On a phone the
    // robot sits dimmed behind the copy, and a D435 filling the screen is
    // just a black rectangle under the text.
    if (f.focus && window.innerWidth >= SPLIT_MIN_WIDTH) {
      const computed = framingFromParts(f.focus, {
        fov: f.fov, pad: f.pad, dir: f.dir, single: f.single, dolly: f.dolly,
      });
      if (computed) return computed;
    }
    return {
      pos: new THREE.Vector3(...(f.pos || FRAMINGS.hero.pos)),
      target: new THREE.Vector3(...(f.target || FRAMINGS.hero.target)),
      fov: f.fov,
    };
  }

  function frame(name, immediate = false) {
    const f = resolveFraming(name);
    desired.pos.copy(f.pos);
    desired.target.copy(f.target);
    desired.fov = f.fov;
    if (!immediate) return;
    // Snap rather than ease. Used on first paint and when a visitor arrives on
    // a #hash link -- easing in from the hero framing there reads as the page
    // being wrong for a second before correcting itself.
    camera.position.copy(desired.pos);
    controls.target.copy(desired.target);
    camera.fov = desired.fov;
    camera.updateProjectionMatrix();
    controls.update();
    renderer.render(scene, camera);
    updateHotspots();
  }

  function setInteractive(on) {
    controls.enabled = on;
    canvas.classList.toggle('is-interactive', on);
    if (!on) setHover(null);
    // Spin is owned by setSection: a close-up of the camera that also orbits
    // is nauseating, and setInteractive(true) on anatomy used to force-stop
    // the hero turntable on the way past.
  }
  controls.addEventListener('start', () => { spin = false; });

  /* --- loop -------------------------------------------------------------- */

  function resize() {
    const w = canvas.clientWidth || window.innerWidth;
    const h = canvas.clientHeight || window.innerHeight;
    if (canvas.width === w * renderer.getPixelRatio() && canvas.height === h * renderer.getPixelRatio()) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', () => { resize(); frameCurrent(); });

  let currentSection = 'hero';
  function frameCurrent() { frame(currentSection); }

  const projected = new THREE.Vector3();
  let raf = 0;

  /* Project hotspot anchors to screen space. Written straight to
   * style.transform with no CSS transition -- a transition here makes every
   * label lag its part by the transition duration while the model turns. */
  function updateHotspots() {
    if (!hotspotEls.size) return;
    const rect = canvas.getBoundingClientRect();
    for (const [part, el] of hotspotEls) {
      const anchor = hotspotAnchors.get(part);
      if (!anchor) continue;
      projected.copy(anchor).applyMatrix4(root.matrixWorld).project(camera);
      // z > 1 means the anchor is behind the camera; project() still returns
      // finite coordinates there, mirrored, so the label would jump to the
      // wrong side of the screen instead of disappearing.
      el.style.opacity = projected.z > 1 ? '0' : '';
      el.style.transform =
        `translate(${(projected.x * 0.5 + 0.5) * rect.width}px, ${(-projected.y * 0.5 + 0.5) * rect.height}px)`;
    }
  }

  function tick() {
    raf = requestAnimationFrame(tick);
    resize();

    // Ease toward the desired framing. Lerp factor is deliberately low: the
    // camera should feel like it is settling, not snapping, and scroll can
    // change the target several times per second.
    camera.position.lerp(desired.pos, 0.055);
    controls.target.lerp(desired.target, 0.07);

    const fovDelta = desired.fov - camera.fov;
    if (Math.abs(fovDelta) > 0.01) {
      camera.fov += fovDelta * 0.07;
      camera.updateProjectionMatrix();
    }
    if (spin) root.rotation.y += 0.0022;

    controls.update();
    renderer.render(scene, camera);

    updateHotspots();
  }
  resize();
  tick();

  return {
    scene, camera, renderer, controls,
    parts: [...partMeshes.keys()],
    registerHotspot,
    setEmphasis,
    setInteractive,
    setSection(name, { immediate = false } = {}) {
      currentSection = name;
      // Close-ups of a part must not orbit; a spinning D435 is unreadable.
      spin = name === 'hero' || name === 'anatomy' || name === 'colophon';
      frame(name, immediate);
    },
    /** Draw one frame on demand. requestAnimationFrame is paused in a hidden
     *  tab, so anything that needs a guaranteed paint calls this. */
    render() { controls.update(); renderer.render(scene, camera); updateHotspots(); },
    resetSpin() { spin = true; },
    dispose() { cancelAnimationFrame(raf); renderer.dispose(); draco.dispose(); },
  };
}

export { PART_GROUPS };
