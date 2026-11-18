// 3D view for the operator map (webui/operator.html): the fused fleet point cloud
// (/fleet/recon_cloud, nodes/recon_cloud_merge.py) with a marker per robot, as the
// alternative to the 2D map. Two modes, switched by the bar over the view:
//   cloud  the live fused point cloud: points only (no voxels), because this is for
//          "where is everything", not for inspecting the reconstruction;
//   splat  a Gaussian splat of a SAVED map, trained post-run on the workstation
//          (operator_splat.js, splat/worker.py). Photoreal, but never live.
//
// The cloud is subscribed ONLY while the 3D view is showing in cloud mode. It is up to a
// few MB of base64 per message, so a hidden view (or one covered by the splat) must not
// keep rosbridge encoding it.
//
// window.sortbotsCloud = { mount(container, ros), show(), hide(), setRobots(list), setMode(m), mode }
// where list is [{id, color, pose: {x, y, yaw} | null}].

(() => {
  "use strict";

  const TOPIC = "/fleet/recon_cloud";
  // The relay emits every ~3 s, ~4 MB of base64 each. This is an overview, not the
  // inspection panel: one every 8 s halves what rosbridge has to encode and buffer for
  // a far-away (tailnet) browser.
  const THROTTLE_MS = 8000;
  const MAX_POINTS = 400000;         // decimate above this, same budget as app.js

  let container = null, ros = null, info = null;
  let renderer = null, scene, camera, controls, points = null, sub = null;
  let needsRender = true, fitted = false, visible = false;
  const markers = new Map(); // robot id -> Mesh
  // ?recon=splat opens straight into splat mode (as on the old dashboard); otherwise the
  // last choice, except on software GL, where splat is never restored by itself (below).
  const QS = new URLSearchParams(location.search);
  let mode = QS.get("recon") === "splat" ? "splat" : "cloud";
  let splat = null, softwareGL = false, lastBounds = null;

  // A software rasteriser (SwiftShader / llvmpipe: the Jetson's snap Chromium) blends a
  // million translucent quads per frame on the CPU the robot's SLAM needs. Never the
  // default there and labelled so nobody picks it expecting it to be usable; not
  // disabled, an explicit choice (or ?recon=splat, which the offline test uses) wins.
  function isSoftwareGL() {
    try {
      const gl = renderer.getContext();
      const ext = gl.getExtension("WEBGL_debug_renderer_info");
      const name = ext ? String(gl.getParameter(ext.UNMASKED_RENDERER_WEBGL)) : "";
      return /swiftshader|llvmpipe|software/i.test(name);
    } catch (e) { return false; }
  }

  function mount(el, rosConn) {
    container = el; ros = rosConn;
    info = document.createElement("div");
    info.className = "cloud-info";
    info.textContent = "Waiting for the point cloud…";
    container.appendChild(info);
    if (typeof THREE === "undefined") { info.textContent = "3D view unavailable (three.js didn't load)"; return; }
    // No WebGL (GPU-less browser, acceleration off) throws here. Say so instead of
    // leaving a blank pane that reads like a dead topic (see app.js, same trap).
    try { renderer = new THREE.WebGLRenderer({ antialias: true }); } catch (e) {
      info.textContent = "3D view needs WebGL, which this browser isn't providing. Check chrome://gpu.";
      renderer = null; return;
    }
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.insertBefore(renderer.domElement, info);

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0c10);
    camera = new THREE.PerspectiveCamera(60, 1, 0.05, 500);
    camera.up.set(0, 0, 1); // map frame is z-up
    camera.position.set(-6, -6, 8);
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.1;
    controls.maxPolarAngle = Math.PI / 2 - 0.02; // never swing under the floor
    controls.addEventListener("change", () => { needsRender = true; });

    const grid = new THREE.GridHelper(60, 60, 0x2a2e35, 0x1c1f24);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);

    new ResizeObserver(size).observe(container);
    size();
    mountBar();
    requestAnimationFrame(loop);
  }

  // The bar's markup is static in operator.html (#cloud .cl-bar); this wires it.
  function mountBar() {
    softwareGL = isSoftwareGL();
    const $ = (sel) => container.querySelector(sel);
    const splatBtn = $('#cl-mode [data-m="splat"]');
    if (softwareGL && splatBtn) {
      splatBtn.textContent = "Splat (slow: no GPU)";
      splatBtn.title = "This browser renders on the CPU (no GPU WebGL), so a splat is very slow here and competes with the robot's SLAM. View it from the workstation or a laptop";
    }
    if (!QS.has("recon")) {
      try { if (localStorage.getItem("sortbots.cloudMode") === "splat" && !softwareGL) mode = "splat"; } catch (e) { /* cloud */ }
    }
    if (window.sortbotsSplat) {
      splat = window.sortbotsSplat.createPanel(container, {
        pick: $("#splat-pick"), mapSel: $("#splat-map"), build: $("#splat-build"),
        status: $("#splat-status"), msg: $("#splat-msg"),
      }, {
        requestRender: () => { needsRender = true; },
        frame: fitSplat,
        setInfo: (t) => { if (mode === "splat") { info.textContent = t; info.hidden = false; } },
      });
    } else if (splatBtn) splatBtn.hidden = true;
    for (const b of container.querySelectorAll("#cl-mode button")) {
      b.addEventListener("click", () => setMode(b.dataset.m));
    }
    syncMode();
  }

  function setMode(m) {
    if (m !== "splat" && m !== "cloud") return;
    if (m === "splat" && !splat) m = "cloud";
    // Back from a splat the camera stands inside the room it framed: look at the cloud
    // from its overview corner again.
    if (m === "cloud" && mode === "splat" && lastBounds) fit(lastBounds.min, lastBounds.max);
    mode = m;
    try { localStorage.setItem("sortbots.cloudMode", m); } catch (e) { /* this tab only */ }
    syncMode();
  }

  // #stage.splatting hides the "waiting for the robots" / link-down cards (operator.html):
  // a splat is a saved map, so no robot has to be online for it to be worth looking at.
  function markStage() {
    const stage = container && container.parentElement;
    if (stage) stage.classList.toggle("splatting", visible && mode === "splat" && !!splat);
  }

  function syncMode() {
    if (!renderer) return;
    markStage();
    const on = mode === "splat";
    for (const b of container.querySelectorAll("#cl-mode button")) b.classList.toggle("active", b.dataset.m === mode);
    const ctl = container.querySelector("#splat-ctl");
    if (ctl) ctl.hidden = !on;
    if (!visible) return;
    if (on) {
      unsubscribe();
      splat.show();
    } else {
      if (splat) splat.hide();
      subscribe();
      info.textContent = lastBounds ? info.dataset.cloud || "" : "Waiting for the point cloud…";
    }
    needsRender = true;
  }

  // A splat of a warehouse is a closed room: framed from outside like the cloud, all you
  // see is the back of its walls and roof. Stand inside instead, near one corner at about
  // the head camera's height (the trainer only ever saw it from there), looking across.
  // Bounds are the 2nd-98th percentile from the renderer, so one floater 200 m out
  // doesn't frame empty space.
  function fitSplat(b) {
    if (!b) return;
    const { min, max } = b;
    const cx = (min[0] + max[0]) / 2, cy = (min[1] + max[1]) / 2, floor = min[2];
    controls.target.set(cx, cy, floor + 1.2);
    camera.position.set(cx - 0.35 * (max[0] - min[0]), cy - 0.35 * (max[1] - min[1]), floor + 1.6);
    controls.update();
    needsRender = true;
  }

  function size() {
    if (!renderer) return;
    const w = container.clientWidth || 1, h = container.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
    needsRender = true;
  }

  function loop() {
    requestAnimationFrame(loop);
    if (!visible) return;
    if (controls.update()) needsRender = true; // damping keeps it moving after a drag
    if (!needsRender) return;
    needsRender = false;
    if (mode === "splat" && splat) splat.render(camera);
    else renderer.render(scene, camera);
  }

  function b64ToBytes(b64) {
    const bin = atob(b64), out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  function onCloud(msg) {
    const total = (msg.width || 0) * (msg.height || 0);
    if (!total) { // the "map was cleared" signal (recon_cloud_merge.py, on recon_reset)
      if (points) points.visible = false;
      info.textContent = "No point cloud yet"; info.hidden = false; needsRender = true; return;
    }
    const bytes = typeof msg.data === "string" ? b64ToBytes(msg.data) : Uint8Array.from(msg.data);
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const off = {};
    for (const f of msg.fields) off[f.name] = f.offset;
    if (off.x == null || off.y == null || off.z == null) { info.textContent = "Cloud has no x/y/z fields"; return; }
    const stride = Math.max(1, Math.ceil(total / MAX_POINTS)), n = Math.ceil(total / stride);
    const pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
    const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
    let k = 0;
    for (let i = 0; i < total; i += stride) {
      const b = i * msg.point_step;
      const x = dv.getFloat32(b + off.x, true), y = dv.getFloat32(b + off.y, true), z = dv.getFloat32(b + off.z, true);
      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
      pos[k * 3] = x; pos[k * 3 + 1] = y; pos[k * 3 + 2] = z;
      if (off.rgb != null) { // packed 0x00RRGGBB
        const v = dv.getUint32(b + off.rgb, true);
        col[k * 3] = ((v >> 16) & 255) / 255; col[k * 3 + 1] = ((v >> 8) & 255) / 255; col[k * 3 + 2] = (v & 255) / 255;
      } else col[k * 3] = col[k * 3 + 1] = col[k * 3 + 2] = 0.8;
      for (let a = 0; a < 3; a++) { const v = pos[k * 3 + a]; if (v < min[a]) min[a] = v; if (v > max[a]) max[a] = v; }
      k++;
    }
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(pos.subarray(0, k * 3), 3));
    geom.setAttribute("color", new THREE.BufferAttribute(col.subarray(0, k * 3), 3));
    // Fixed PIXEL size: sized in metres (app.js's 3 cm), points framed from high enough
    // to see a whole warehouse are far below a pixel and the view reads as empty.
    if (points) { points.geometry.dispose(); points.geometry = geom; points.visible = true; }
    else { points = new THREE.Points(geom, new THREE.PointsMaterial({ size: 2.5, sizeAttenuation: false, vertexColors: true })); scene.add(points); }
    info.dataset.cloud = `${k.toLocaleString()} points${stride > 1 ? ` (1 in ${stride})` : ""}`;
    lastBounds = { min, max };
    if (mode === "cloud") info.textContent = info.dataset.cloud;
    if (!fitted) { fit(min, max); fitted = true; }
    needsRender = true;
  }

  // Look down at the cloud from a corner, high enough to see the whole floor plan.
  function fit(min, max) {
    const cx = (min[0] + max[0]) / 2, cy = (min[1] + max[1]) / 2;
    const r = Math.max(4, 0.5 * Math.hypot(max[0] - min[0], max[1] - min[1]));
    controls.target.set(cx, cy, 0);
    camera.position.set(cx - r * 0.6, cy - r * 0.9, r * 1.1);
    controls.update();
  }

  function setRobots(list) {
    if (!renderer) return;
    for (const r of list) {
      let m = markers.get(r.id);
      if (!m) {
        m = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.6, 16), new THREE.MeshBasicMaterial({ color: new THREE.Color(r.color) }));
        m.rotation.x = Math.PI / 2; // ConeGeometry points +y; stand it up along +z
        scene.add(m); markers.set(r.id, m);
      }
      m.visible = !!r.pose;
      if (r.pose) {
        const moved = Math.abs(m.position.x - r.pose.x) > 1e-3 || Math.abs(m.position.y - r.pose.y) > 1e-3;
        m.position.set(r.pose.x, r.pose.y, 0.35);
        if (moved) needsRender = true;
      }
    }
  }

  function subscribe() {
    if (!renderer || sub) return;
    sub = new ROSLIB.Topic({ ros, name: TOPIC, messageType: "sensor_msgs/PointCloud2", throttle_rate: THROTTLE_MS, queue_length: 1, reconnect_on_close: true });
    sub.subscribe(onCloud);
  }
  function unsubscribe() {
    if (sub) { sub.unsubscribe(); sub = null; }
  }
  function show() {
    visible = true;
    if (!renderer) return;
    size();
    syncMode(); // subscribes in cloud mode, starts the worker polls in splat mode
  }
  function hide() {
    visible = false;
    markStage();
    unsubscribe();
    if (splat) splat.hide(); // stops polling the worker
  }

  window.sortbotsCloud = { mount, show, hide, setRobots, setMode, get mode() { return mode; } };
})();
