// SortBots operator view (webui/operator.html). Fleet-first: every robot, one map,
// the task list and the waypoints, with one robot's live view a click away. The
// per-robot engineering dashboard (app.js) is deprecated and stays at /?robot=ID.
//
// Talks to two things, like the rest of the webui:
//   - rosbridge (9090)   live topics: /map, /tf, per-robot plan / explore_status /
//                        task_status / action status / obstacles, and the commands this page
//                        sends (dispatch_task, explore_cmd, navigate_to_pose, cmd_vel)
//   - serve.py (HTTP)    /api/robots, /api/nav_waypoints (waypoints, which are
//                        also the task endpoints), /api/session (console control)
// The session panel itself is scenarios.js, unchanged apart from tolerating the
// missing live-view panes; this file only reads /api/session to label the header.
//
// Every ROSLIB.Topic passes reconnect_on_close: true for the same reason app.js
// does: roslib reopens the websocket but does NOT resend subscribe ops.

(() => {
  "use strict";

  const HOST = window.location.hostname || "localhost";
  const ROSBRIDGE_PORT = 9090;
  const VIDEO_PORT = 8080;
  // A robot whose base_link transform hasn't updated for this long is "not responding".
  const ONLINE_TTL_MS = 5000;
  // explore_status / task_status are heartbeats; silence means the node is gone.
  const EXPLORE_TTL_MS = 6000;
  const COLORS = ["#2f6fed", "#ef7d1a", "#8a5cf0", "#e0457b", "#1aa6a6", "#7a8b1f"];
  // obstacle_detector.py publishes at ~10 Hz; a robot whose last list is older than
  // this has its markers dropped rather than left frozen where someone used to be.
  const OBSTACLE_TTL_MS = 1500;
  const OBSTACLE_RED = "#e5484d";
  const GOAL_STATES = { 1: "accepted", 2: "navigating", 3: "cancelling", 4: "arrived", 5: "cancelled", 6: "failed" };

  const $ = (s) => document.querySelector(s);
  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const nameOf = (id) => id.replace(/^robot_/, "Robot ");

  // -- state ------------------------------------------------------------------

  const robots = new Map(); // id -> {id, color, pose, linkAt, plan, explore, exploreAt, task, taskAt, queueDepth, nav, navStamp, odom, odomAt}
  // Cards can be open for several robots at once (each with its own camera). `selected`
  // is the one map commands go to: the card opened or clicked into most recently.
  const open = new Set();
  let selected = null;
  let tool = "select";
  let view3d = false; // main stage: 2D occupancy map, or the 3D point cloud
  let waypoints = []; // [{name, x, y, yaw}], the server's copy — also the task endpoints
  let tasks = []; // [{id, kind: "goto"|"pickup", goto, pickup, dropoff, robot, state, at, reason}]
  let session = null; // last /api/session body, null until the first answer
  let controlAvailable = true; // false once /api/session answers 503
  let mapInfo = null;
  let mapCanvas = null;
  let mappedM2 = 0;
  let knownBox = null; // [minX, minY, maxX, maxY] of non-unknown cells, world metres
  let dirty = true;

  // -- rosbridge --------------------------------------------------------------

  const ros = new ROSLIB.Ros({ url: `ws://${HOST}:${ROSBRIDGE_PORT}` });
  ros.on("connection", () => { $("#link-down").hidden = true; });
  ros.on("close", () => {
    $("#link-down").hidden = false;
    setTimeout(() => ros.connect(`ws://${HOST}:${ROSBRIDGE_PORT}`), 2000);
  });
  ros.on("error", (e) => { $("#link-down").hidden = false; console.error("rosbridge error", e); });

  const topic = (name, messageType, extra) =>
    new ROSLIB.Topic({ ros, name, messageType, reconnect_on_close: true, ...extra });
  const publishers = new Map();
  function publish(name, messageType, msg) {
    if (!publishers.has(name)) publishers.set(name, topic(name, messageType));
    publishers.get(name).publish(new ROSLIB.Message(msg));
  }
  const publishString = (name, data) => publish(name, "std_msgs/String", { data });

  // -- TF: robot poses ----------------------------------------------------------
  // Same approach as app.js: compose map -> <id>/map -> <id>/odom -> <id>/base_link
  // from /tf, /tf_static and /map_anchors ourselves rather than ask a tf2 node.
  // Every robot lives in the one world `map` frame, so one tree serves the fleet.

  const tfTree = new Map(); // child -> {parent, t, q}
  const normFrame = (f) => (f || "").replace(/^\//, "");

  function quatMul(a, b) {
    return {
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
  }
  function quatRotate(q, v) {
    const { x, y, z, w } = q;
    const tx = 2 * (y * v.z - z * v.y), ty = 2 * (z * v.x - x * v.z), tz = 2 * (x * v.y - y * v.x);
    return { x: v.x + w * tx + (y * tz - z * ty), y: v.y + w * ty + (z * tx - x * tz), z: v.z + w * tz + (x * ty - y * tx) };
  }
  function lookupPose(frame) {
    let acc = { t: { x: 0, y: 0, z: 0 }, q: { x: 0, y: 0, z: 0, w: 1 } };
    let cur = normFrame(frame);
    for (let hops = 0; hops < 16; hops++) { // bounded: a cycle must not spin a /tf callback
      if (cur === "map") {
        return { x: acc.t.x, y: acc.t.y, yaw: Math.atan2(2 * (acc.q.w * acc.q.z + acc.q.x * acc.q.y), 1 - 2 * (acc.q.y * acc.q.y + acc.q.z * acc.q.z)) };
      }
      const link = tfTree.get(cur);
      if (!link) return null;
      const r = quatRotate(link.q, acc.t);
      acc = { t: { x: link.t.x + r.x, y: link.t.y + r.y, z: link.t.z + r.z }, q: quatMul(link.q, acc.q) };
      cur = link.parent;
    }
    return null;
  }

  function onTf(msg) {
    const now = performance.now();
    for (const tr of msg.transforms || []) {
      const child = normFrame(tr.child_frame_id);
      tfTree.set(child, { parent: normFrame(tr.header.frame_id), t: tr.transform.translation, q: tr.transform.rotation });
      if (child.endsWith("/base_link")) {
        const r = robots.get(child.slice(0, -"/base_link".length));
        if (r) r.linkAt = now;
      }
    }
  }
  topic("/tf", "tf2_msgs/TFMessage", { throttle_rate: 50 }).subscribe(onTf);
  topic("/tf_static", "tf2_msgs/TFMessage").subscribe(onTf);
  // map_merge's per-robot map -> <id>/map anchors; its own subscription because
  // sharing /tf's throttle window starves it (see app.js).
  topic("/map_anchors", "tf2_msgs/TFMessage").subscribe(onTf);

  // -- map ----------------------------------------------------------------------

  // -- theme --------------------------------------------------------------------------
  // CSS owns the palette (custom properties, dark via prefers-color-scheme or a pinned
  // data-theme). Canvas code can't use var(), so it reads the resolved values here and
  // re-reads them whenever the theme changes.
  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  // "#rrggbb" -> 0xAABBGGRR, the byte order of a Uint32 view over little-endian RGBA.
  const abgr = (hex) => { const n = parseInt(hex.slice(1), 16); return (0xff000000 | ((n & 0xff) << 16) | (n & 0xff00) | (n >> 16)) >>> 0; };
  let palette = {};
  function readPalette() {
    palette = {
      bg: css("--map-bg"), ink: css("--map-ink"), robotInk: css("--map-robot-ink"),
      unknown: abgr(css("--map-unknown")), free: abgr(css("--map-free")), occ: abgr(css("--map-occ")),
    };
  }
  const systemDark = window.matchMedia("(prefers-color-scheme: dark)");
  const isDark = () => (document.documentElement.dataset.theme ? document.documentElement.dataset.theme === "dark" : systemDark.matches);
  function applyTheme() {
    const btn = $("#theme"), dark = isDark();
    btn.textContent = dark ? "\u2600" : "\u263E"; // sun / moon: the mode a click switches to
    btn.title = btn.ariaLabel = dark ? "Switch to light mode" : "Switch to dark mode";
    readPalette();
    if (lastMap) handleMap(lastMap); // the grid is rasterised with palette colours
    dirty = true;
  }
  $("#theme").addEventListener("click", () => {
    const next = isDark() ? "light" : "dark";
    document.documentElement.dataset.theme = next;
    try { localStorage.setItem("sortbots.theme", next); } catch (e) { /* private window: still works for this tab */ }
    applyTheme();
  });
  systemDark.addEventListener("change", applyTheme); // only matters while nothing is pinned

  let lastMap = null;
  topic("/map", "nav_msgs/OccupancyGrid").subscribe((msg) => { lastMap = msg; handleMap(msg); });
  function handleMap(msg) {
    const { width: w, height: h, resolution: res, origin } = msg.info;
    if (!w || !h) return;
    const off = mapCanvas || document.createElement("canvas");
    off.width = w; off.height = h;
    const octx = off.getContext("2d");
    const img = octx.createImageData(w, h);
    const px = new Uint32Array(img.data.buffer);
    let known = 0, minC = w, maxC = -1, minR = h, maxR = -1;
    for (let row = 0; row < h; row++) {
      const dst = (h - 1 - row) * w; // grid row 0 is the bottom; image row 0 is the top
      for (let col = 0; col < w; col++) {
        const v = msg.data[row * w + col];
        if (v < 0) { px[dst + col] = palette.unknown; continue; }
        px[dst + col] = v >= 50 ? palette.occ : palette.free;
        known++;
        if (col < minC) minC = col; if (col > maxC) maxC = col;
        if (row < minR) minR = row; if (row > maxR) maxR = row;
      }
    }
    octx.putImageData(img, 0, 0);
    mapCanvas = off;
    mapInfo = msg.info;
    mappedM2 = known * res * res;
    knownBox = maxC < 0 ? null : [
      origin.position.x + minC * res, origin.position.y + minR * res,
      origin.position.x + (maxC + 1) * res, origin.position.y + (maxR + 1) * res,
    ];
    if (!view.moved) fitView();
    render();
  }

  // -- view (pan / zoom) ----------------------------------------------------------

  const view = { cx: 0, cy: 0, scale: 20, moved: false }; // centre in world m, px per m
  const canvas = $("#map");
  const ctx = canvas.getContext("2d");
  let cssW = 0, cssH = 0;

  function resize() {
    const r = canvas.parentElement.getBoundingClientRect(), d = window.devicePixelRatio || 1;
    cssW = r.width; cssH = r.height;
    canvas.width = Math.round(cssW * d); canvas.height = Math.round(cssH * d);
    ctx.setTransform(d, 0, 0, d, 0, 0);
    if (!view.moved) fitView();
    dirty = true;
  }
  const toPx = (x, y) => [cssW / 2 + (x - view.cx) * view.scale, cssH / 2 - (y - view.cy) * view.scale];
  const toWorld = (px, py) => [view.cx + (px - cssW / 2) / view.scale, view.cy - (py - cssH / 2) / view.scale];

  function fitView() {
    let box = knownBox ? [...knownBox] : null;
    for (const r of robots.values()) {
      if (!r.pose) continue;
      box = box ? [Math.min(box[0], r.pose.x), Math.min(box[1], r.pose.y), Math.max(box[2], r.pose.x), Math.max(box[3], r.pose.y)]
                : [r.pose.x, r.pose.y, r.pose.x, r.pose.y];
    }
    if (!box || !cssW) return;
    const pad = 2; // m of breathing room
    const w = Math.max(box[2] - box[0] + 2 * pad, 8), h = Math.max(box[3] - box[1] + 2 * pad, 8);
    view.scale = Math.min((cssW - 120) / w, (cssH - 80) / h);
    view.cx = (box[0] + box[2]) / 2; view.cy = (box[1] + box[3]) / 2;
    dirty = true;
  }

  // -- obstacles (nodes/obstacle_detector.py) ----------------------------------------
  // Positions arrive in <id>/base_link and velocities in the detector's world frame
  // (<id>/odom, or base_link when it had no odom TF). Both are placed in `map` once, on
  // arrival, with the poses /tf gives right then, so a marker stays put while the robot
  // carries on moving instead of swinging around with it until the next message.
  function obstaclesInMap(p, base, world) {
    const c = Math.cos(base.yaw), s = Math.sin(base.yaw);
    const toMap = (x, y) => [base.x + c * x - s * y, base.y + s * x + c * y];
    const vyaw = world ? world.yaw : base.yaw;
    const vc = Math.cos(vyaw), vs = Math.sin(vyaw);
    return (p.obstacles || []).map((o) => {
      const [x0, y0] = o.min, [x1, y1] = o.max, v = o.vel || [0, 0];
      return {
        id: o.track_id, moving: !!o.moving, speed: o.speed_mps || 0, range: o.range_m,
        at: toMap(o.centroid[0], o.centroid[1]),
        corners: [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([a, b]) => toMap(a, b)),
        vel: [vc * v[0] - vs * v[1], vs * v[0] + vc * v[1]],
      };
    });
  }
  function onObstacles(r, p) {
    r.obstacles = p; r.obstaclesAt = performance.now();
    const base = lookupPose(`${r.id}/base_link`);
    const world = p.world_frame ? lookupPose(p.world_frame) : null;
    const had = r.obstaclesMap.some((o) => o.moving);
    r.obstaclesMap = base ? obstaclesInMap(p, base, world) : [];
    if (had || r.obstaclesMap.some((o) => o.moving)) dirty = true;
    drawCamObstacles(r.id);
  }

  // Map: only MOVING obstacles. Static ones are mostly shelving the map already shows,
  // and drawing every cluster would bury the robots under boxes.
  function drawObstacles() {
    ctx.font = '600 11px "IBM Plex Sans", system-ui, sans-serif'; ctx.textAlign = "center";
    for (const r of robots.values()) {
      for (const o of r.obstaclesMap) {
        if (!o.moving) continue;
        ctx.fillStyle = OBSTACLE_RED; ctx.strokeStyle = OBSTACLE_RED; ctx.lineWidth = 2;
        ctx.globalAlpha = 0.3; ctx.beginPath();
        o.corners.forEach(([x, y], i) => { const [a, b] = toPx(x, y); i ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
        ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1; ctx.stroke();
        const [a, b] = toPx(o.at[0], o.at[1]);
        ctx.beginPath(); ctx.arc(a, b, 5, 0, 7); ctx.fill();
        // A person's footprint is a few pixels at fleet zoom; the halo is what's found.
        ctx.globalAlpha = 0.35; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(a, b, 12, 0, 7); ctx.stroke();
        ctx.globalAlpha = 1; ctx.lineWidth = 2;
        // Where it will be in one second at this velocity, floored so a slow walker's
        // arrow is still visible at fleet zoom.
        const len = Math.max(Math.hypot(o.vel[0], o.vel[1]) * view.scale, 14);
        const ang = Math.atan2(-o.vel[1], o.vel[0]); // canvas y points down
        const ex = a + Math.cos(ang) * len, ey = b + Math.sin(ang) * len;
        ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(ex, ey); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(ex, ey);
        ctx.lineTo(ex - Math.cos(ang - 0.45) * 8, ey - Math.sin(ang - 0.45) * 8);
        ctx.lineTo(ex - Math.cos(ang + 0.45) * 8, ey - Math.sin(ang + 0.45) * 8);
        ctx.closePath(); ctx.fill();
        const label = `moving · ${o.speed.toFixed(1)} m/s`;
        const tw = ctx.measureText(label).width + 10;
        ctx.fillRect(a - tw / 2, b - 30, tw, 16);
        ctx.fillStyle = "#fff"; ctx.fillText(label, a, b - 18);
      }
    }
  }

  // Camera tile: every obstacle, boxed in the depth image's pixels (in sim one camera
  // makes both images; the D435's depth is aligned to colour). The picture is drawn
  // `background-size: cover`, so invert a cover crop, not a contain letterbox.
  function drawCamObstacles(id) {
    const c = cams.get(id);
    if (!c || !open.has(id)) return;
    const cv = c.overlay, w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return;
    if (cv.width !== w) cv.width = w;
    if (cv.height !== h) cv.height = h;
    const g = cv.getContext("2d");
    g.clearRect(0, 0, w, h);
    const r = robots.get(id), p = r && r.obstacles;
    if (!p || performance.now() - r.obstaclesAt > OBSTACLE_TTL_MS || !p.w || !p.h) return;
    const k = Math.max(w / p.w, h / p.h), ox = (w - p.w * k) / 2, oy = (h - p.h * k) / 2;
    g.font = '600 10px "IBM Plex Sans", system-ui, sans-serif'; g.textBaseline = "bottom";
    for (const o of p.obstacles || []) {
      const [u0, v0, u1, v1] = o.uv;
      const x = ox + u0 * k, y = oy + v0 * k, bh = (v1 - v0) * k, color = o.moving ? OBSTACLE_RED : "#f5a524";
      g.strokeStyle = color; g.lineWidth = o.moving ? 2.5 : 1.5;
      g.strokeRect(x, y, (u1 - u0) * k, bh);
      const label = o.moving ? `moving · ${o.range_m.toFixed(1)} m` : `${o.range_m.toFixed(1)} m`;
      // Inside the box's bottom edge: the top of the tile belongs to the Camera/Depth
      // switch, and a person's box reaches it.
      const tw = g.measureText(label).width + 6, ly = Math.min(y + bh, h);
      g.fillStyle = color; g.fillRect(x, ly - 13, tw, 13);
      g.fillStyle = o.moving ? "#fff" : "#15191e"; g.fillText(label, x + 3, ly - 1);
    }
  }

  // -- world overlay: props and scripted walkers (configs/worlds/, sim only) -----------
  // From GROUND TRUTH, never from what a robot perceived: /sim/world is the latched
  // layout (shapes, colours, paths), /sim/ground_truth the 10 Hz poses. Drawn under
  // the robots, so a running test reads at a glance: the walker, where it is going,
  // and whether the robot's red obstacle marker sits on it. Same data and look as
  // app.js's overlay; neither topic exists on a real robot, so nothing draws there.
  const WORLD_TRUTH_TTL_MS = 3000; // sim gone quiet: stop drawing walkers frozen mid-stride
  let worldLayout = null, worldTruth = null, worldTruthAt = -Infinity;
  topic("/sim/world", "std_msgs/String").subscribe((msg) => {
    try { worldLayout = JSON.parse(msg.data); } catch (e) { return; }
    dirty = true;
  });
  topic("/sim/ground_truth", "std_msgs/String", { throttle_rate: 100, queue_length: 1 }).subscribe((msg) => {
    try { worldTruth = JSON.parse(msg.data); } catch (e) { return; }
    worldTruthAt = performance.now(); dirty = true;
  });
  const worldLive = () => !!(worldLayout && worldLayout.world && worldTruth && performance.now() - worldTruthAt < WORLD_TRUTH_TTL_MS);
  const rgba = (c, a) => `rgba(${Math.round(c[0] * 255)},${Math.round(c[1] * 255)},${Math.round(c[2] * 255)},${a})`;
  // A simtest.geometry shape: ["circle", [x, y], r] | ["poly", [[x, y], ...]].
  function traceShape(shape) {
    ctx.beginPath();
    if (shape[0] === "circle") {
      const [a, b] = toPx(shape[1][0], shape[1][1]);
      ctx.arc(a, b, Math.max(3, shape[2] * view.scale), 0, 7);
      return;
    }
    shape[1].forEach(([x, y], i) => { const [a, b] = toPx(x, y); i ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
    ctx.closePath();
  }
  function actorShape(a, p) {
    if (a.shape === "cylinder") return ["circle", [p.x, p.y], a.radius];
    const [sx, sy] = a.size, c = Math.cos(p.yaw), s = Math.sin(p.yaw);
    return ["poly", [[sx / 2, sy / 2], [-sx / 2, sy / 2], [-sx / 2, -sy / 2], [sx / 2, -sy / 2]]
      .map(([x, y]) => [p.x + c * x - s * y, p.y + s * x + c * y])];
  }
  function drawWorld() {
    if (!worldLive()) return;
    ctx.save();
    for (const p of worldLayout.props || []) {
      traceShape(p.footprint);
      ctx.fillStyle = rgba(p.color, 0.35); ctx.strokeStyle = rgba(p.color, 0.9); ctx.lineWidth = 1.5;
      ctx.fill(); ctx.stroke();
    }
    ctx.font = '600 11px "IBM Plex Sans", system-ui, sans-serif'; ctx.textAlign = "center";
    for (const a of worldLayout.actors || []) {
      // The route it walks: dotted and faint, so it never reads as a Nav2 plan.
      ctx.setLineDash([2, 5]); ctx.strokeStyle = rgba(a.color, 0.5); ctx.lineWidth = 1.5;
      ctx.beginPath();
      (a.path || []).forEach(([x, y], i) => { const [px, py] = toPx(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      if (a.mode === "loop") ctx.closePath();
      ctx.stroke(); ctx.setLineDash([]);
      const pose = worldTruth.actors && worldTruth.actors[a.id];
      if (!pose) continue;
      traceShape(actorShape(a, pose));
      ctx.fillStyle = rgba(a.color, 0.9); ctx.strokeStyle = palette.robotInk; ctx.lineWidth = 1.5;
      ctx.fill(); ctx.stroke();
      const [px, py] = toPx(pose.x, pose.y), r = Math.max(5, (a.radius || 0.3) * view.scale);
      // Heading tick, dashed while it stands still at a waypoint.
      ctx.strokeStyle = rgba(a.color, 1); ctx.lineWidth = 2; ctx.setLineDash(pose.moving ? [] : [3, 3]);
      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px + r * 2 * Math.cos(pose.yaw), py - r * 2 * Math.sin(pose.yaw)); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = palette.ink; ctx.fillText(a.label || a.id, px, py + r + 13);
    }
    ctx.restore();
  }

  // -- drawing ----------------------------------------------------------------------

  function drawFrame() {
    requestAnimationFrame(drawFrame);
    if (!dirty || !cssW) return;
    dirty = false;
    ctx.fillStyle = palette.bg;
    ctx.fillRect(0, 0, cssW, cssH);

    if (mapCanvas && mapInfo) {
      const { width: w, height: h, resolution: res, origin } = mapInfo;
      const [dx, dy] = toPx(origin.position.x, origin.position.y + h * res);
      ctx.imageSmoothingEnabled = false; // crisp cells
      ctx.drawImage(mapCanvas, dx, dy, w * res * view.scale, h * res * view.scale);
    }

    drawWorld();

    for (const r of robots.values()) { // planned routes
      if (!r.plan.length || !r.pose) continue;
      ctx.strokeStyle = r.color; ctx.globalAlpha = 0.75; ctx.lineWidth = 2; ctx.setLineDash([6, 5]);
      ctx.beginPath();
      r.plan.forEach(([x, y], i) => { const [a, b] = toPx(x, y); i ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
      ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    }

    ctx.font = '500 11px "IBM Plex Sans", system-ui, sans-serif'; ctx.textAlign = "center";
    for (const w of waypoints) {
      const [a, b] = toPx(w.x, w.y);
      ctx.fillStyle = "#f2b705"; ctx.strokeStyle = palette.ink; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(a, b - 9); ctx.lineTo(a + 6, b); ctx.lineTo(a, b + 9); ctx.lineTo(a - 6, b); ctx.closePath();
      ctx.fill(); ctx.stroke();
      ctx.fillStyle = palette.ink; ctx.fillText(w.name, a, b - 14);
    }

    drawObstacles();

    for (const r of robots.values()) {
      if (!r.pose) continue;
      const [a, b] = toPx(r.pose.x, r.pose.y), live = online(r);
      ctx.globalAlpha = live ? 1 : 0.4;
      if (open.has(r.id)) { ctx.strokeStyle = r.color; ctx.lineWidth = selected === r.id ? 2.5 : 1.25; ctx.beginPath(); ctx.arc(a, b, 17, 0, 7); ctx.stroke(); }
      ctx.fillStyle = r.color; ctx.beginPath(); ctx.arc(a, b, 10, 0, 7); ctx.fill();
      const yaw = -r.pose.yaw; // canvas y points down
      ctx.fillStyle = palette.robotInk; ctx.beginPath();
      ctx.moveTo(a + Math.cos(yaw) * 13, b + Math.sin(yaw) * 13);
      ctx.lineTo(a + Math.cos(yaw + 2.5) * 6, b + Math.sin(yaw + 2.5) * 6);
      ctx.lineTo(a + Math.cos(yaw - 2.5) * 6, b + Math.sin(yaw - 2.5) * 6);
      ctx.fill();
      ctx.fillStyle = palette.ink; ctx.font = '600 11px "IBM Plex Sans", system-ui, sans-serif';
      ctx.fillText(nameOf(r.id), a, b + 28);
      ctx.globalAlpha = 1;
    }
    drawDrag();
  }

  // The send being dragged: robot -> destination, in the robot's colour, with the
  // heading it will arrive at.
  function drawDrag() {
    const r = drag && robots.get(drag.id);
    if (!r || !r.pose) return;
    const [a, b] = toPx(r.pose.x, r.pose.y), [ex, ey] = toPx(drag.wx, drag.wy);
    const yaw = drag.wp ? -(drag.wp.yaw || 0) : Math.atan2(ey - b, ex - a); // canvas y points down
    ctx.strokeStyle = r.color; ctx.fillStyle = r.color; ctx.lineWidth = 2.5;
    ctx.setLineDash([7, 5]); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(ex, ey); ctx.stroke(); ctx.setLineDash([]);
    ctx.globalAlpha = 0.25; ctx.beginPath(); ctx.arc(ex, ey, 14, 0, 7); ctx.fill(); ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(ex, ey, 14, 0, 7); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ex + Math.cos(yaw) * 12, ey + Math.sin(yaw) * 12);
    ctx.lineTo(ex + Math.cos(yaw + 2.5) * 7, ey + Math.sin(yaw + 2.5) * 7);
    ctx.lineTo(ex + Math.cos(yaw - 2.5) * 7, ey + Math.sin(yaw - 2.5) * 7);
    ctx.fill();
    const label = drag.wp ? `Send ${nameOf(r.id)} to ${drag.wp.name}` : `Send ${nameOf(r.id)} here`;
    ctx.font = '600 11px "IBM Plex Sans", system-ui, sans-serif'; ctx.textAlign = "center";
    const tw = ctx.measureText(label).width + 12;
    ctx.fillRect(ex - tw / 2, ey - 38, tw, 18);
    ctx.fillStyle = "#fff"; ctx.fillText(label, ex, ey - 25);
  }
  requestAnimationFrame(drawFrame);

  // -- robots --------------------------------------------------------------------------

  const online = (r) => !!r.pose && performance.now() - r.linkAt < ONLINE_TTL_MS;

  function addRobot(id) {
    if (robots.has(id)) return;
    const r = {
      id, color: COLORS[robots.size % COLORS.length], pose: null, linkAt: -Infinity, plan: [],
      explore: null, exploreAt: -Infinity, task: null, taskAt: -Infinity, queueDepth: 0,
      nav: null, seenStamp: {}, odom: null, odomAt: -Infinity,
      obstacles: null, obstaclesAt: -Infinity, obstaclesMap: [],
    };
    robots.set(id, r);

    // Always subscribed, not just while a card is open: moving obstacles go on the
    // fleet map. A few KB at 5 Hz per robot.
    topic(`/${id}/obstacles`, "std_msgs/String", { throttle_rate: 200, queue_length: 1 }).subscribe((msg) => {
      let p;
      try { p = JSON.parse(msg.data); } catch (e) { return; }
      onObstacles(r, p);
    });

    // Only the twist is used (speed / turn rate on the card); the pose comes from /tf.
    // Throttled: the card shows it to two decimals, 4 Hz is plenty.
    topic(`/${id}/odom`, "nav_msgs/Odometry", { throttle_rate: 250, queue_length: 1 }).subscribe((msg) => {
      r.odom = msg.twist.twist; r.odomAt = performance.now();
    });

    topic(`/${id}/plan`, "nav_msgs/Path").subscribe((msg) => {
      r.plan = (msg.poses || []).map((p) => [p.pose.position.x, p.pose.position.y]);
      dirty = true;
    });
    topic(`/${id}/explore_status`, "std_msgs/String").subscribe((msg) => {
      try { r.explore = JSON.parse(msg.data); } catch (e) { return; }
      r.exploreAt = performance.now();
    });
    topic(`/${id}/task_status`, "std_msgs/String").subscribe((msg) => {
      let s;
      try { s = JSON.parse(msg.data); } catch (e) { return; }
      r.task = s; r.taskAt = performance.now(); r.queueDepth = s.queue_depth || 0;
      noteStatus(id, s);
    });
    // Progress of a goal we sent: from each action's GoalStatusArray (roslib drops
    // rosbridge's action_result), same trick as app.js.
    for (const action of ["navigate_to_pose", "follow_waypoints"]) {
      topic(`/${id}/${action}/_action/status`, "action_msgs/msg/GoalStatusArray").subscribe((msg) => {
        const stamp = (g) => g.goal_info.stamp.sec + g.goal_info.stamp.nanosec * 1e-9;
        const newest = (msg.status_list || []).reduce((a, g) => (!a || stamp(g) > stamp(a) ? g : a), null);
        if (!newest) return;
        // Remember the newest goal seen per action so a goal we send later can tell its
        // own status apart from an earlier, already-finished goal still in the list.
        if (!(r.seenStamp[action] >= stamp(newest))) r.seenStamp[action] = stamp(newest);
        if (!r.nav || !r.nav.action.endsWith(`/${action}`)) return;
        if (stamp(newest) <= r.nav.afterStamp) return; // an earlier goal's status
        r.nav.state = GOAL_STATES[newest.status] || "navigating";
        if (newest.status >= 4) { r.nav.done = true; r.nav.at = performance.now(); }
        renderFleet();
      });
    }
    renderFleet();
  }

  function statusOf(r) {
    if (!online(r)) return { tag: "offline", detail: r.pose ? "Not responding" : "Waiting for the robot" };
    const now = performance.now();
    // DONE / FAILED are one-message transients task_manager publishes just before IDLE.
    const busy = r.task && r.task.state && !FINISHED.includes(r.task.state) && r.task.state !== "IDLE";
    if (busy && now - r.taskAt < EXPLORE_TTL_MS * 4) {
      const goal = r.task.task && r.task.task.goto;
      return { tag: "on a task", detail: r.task.state === "NAV_TO_GOAL" && goal ? `Going to ${goal}` : TASK_PHASE[r.task.state] || r.task.state };
    }
    if (r.nav && !r.nav.done) return { tag: "navigating", detail: `Going to ${r.nav.label}` };
    if (r.explore && r.explore.state === "exploring" && now - r.exploreAt < EXPLORE_TTL_MS) return { tag: "exploring", detail: "Exploring" };
    return { tag: "idle", detail: r.nav && r.nav.state === "failed" ? "Last goal failed" : "Idle" };
  }

  const isFree = (r) => online(r) && statusOf(r).tag === "idle" && !r.queueDepth;

  // -- tasks ---------------------------------------------------------------------------
  // task_manager keeps one queue per robot. Each /<id>/task_status carries the CURRENT
  // task ({task_id, state, task}) and the waiting ones in run order ({queue: [...]}),
  // published on every state change and every queue change. The list here is: tasks
  // this page dispatched (optimistic, QUEUED until a status mentions them) merged with
  // whatever the statuses report, so tasks queued from elsewhere still show up.
  // Any unfinished task can be cancelled (/<id>/cancel_task, by id).

  const TASK_PHASE = {
    NAV_TO_PICKUP: "Heading to pick-up", PICKING: "Picking up", NAV_TO_DROPOFF: "Heading to drop-off",
    PLACING: "Placing", NAV_TO_GOAL: "Moving", DONE: "Done", FAILED: "Failed", CANCELLED: "Cancelled",
  };
  // DONE / FAILED / CANCELLED are one-message transients task_manager publishes just before IDLE.
  const FINISHED = ["DONE", "FAILED", "CANCELLED"];
  // A status published before our dispatch landed can't list the new task yet, so a
  // task only counts as "gone from the queue" once it is older than this.
  const QUEUE_GRACE_MS = 3000;

  function upsertTask(robotId, summary, state) {
    let t = tasks.find((x) => x.id === summary.task_id);
    if (!t) { t = { id: summary.task_id, kind: summary.kind || "pickup", robot: robotId, state, at: Date.now() }; tasks.push(t); }
    if (summary.kind) Object.assign(t, { kind: summary.kind, goto: summary.goto, pickup: summary.pickup, dropoff: summary.dropoff });
    if (t.state !== state) t.at = Date.now();
    t.robot = robotId; t.state = state;
    return t;
  }

  function noteStatus(robotId, s) {
    if (s.rejected) {
      const t = tasks.find((x) => x.id === s.rejected.task_id);
      if (t) { t.state = "FAILED"; t.reason = s.rejected.reason; t.at = Date.now(); }
      toast(`${nameOf(robotId)} rejected a task: ${s.rejected.reason}`);
    }
    if (s.task_id) upsertTask(robotId, s.task || { task_id: s.task_id }, s.state);
    if (Array.isArray(s.queue)) { // an older task_manager sends no queue; trust nothing then
      const waiting = new Set(s.queue.map((q) => q.task_id));
      for (const q of s.queue) upsertTask(robotId, q, "QUEUED");
      for (const t of tasks) {
        if (t.robot !== robotId || t.state !== "QUEUED" || waiting.has(t.id) || t.id === s.task_id) continue;
        if (Date.now() - t.at > QUEUE_GRACE_MS) { t.state = "CANCELLED"; t.at = Date.now(); }
      }
    }
    trimTasks(); renderTasks(); renderFleet(); demoTick();
  }
  function trimTasks() {
    const finished = tasks.filter((t) => FINISHED.includes(t.state)).sort((a, b) => b.at - a.at);
    for (const t of finished.slice(8)) tasks.splice(tasks.indexOf(t), 1);
  }

  const TASK_LABEL = (t) => ({
    QUEUED: ["waiting", ""], NAV_TO_PICKUP: ["en route", "active"], PICKING: ["picking up", "active"],
    NAV_TO_DROPOFF: ["en route", "active"], PLACING: ["placing", "active"], NAV_TO_GOAL: ["en route", "active"],
    DONE: ["done", "done"], FAILED: ["failed", "failed"], CANCELLED: ["cancelled", ""],
  }[t.state] || [t.state.toLowerCase(), "active"]);
  const taskTitle = (t) => (t.kind === "goto" && t.goto ? `Go to ${esc(t.goto)}`
    : t.pickup ? `${esc(t.pickup)} to ${esc(t.dropoff)}` : `Task ${esc(t.id)}`);

  function renderTasks() {
    const rank = (t) => (FINISHED.includes(t.state) ? 2 : t.state === "QUEUED" ? 1 : 0);
    // Waiting tasks keep dispatch order (that IS the run order, per robot); `at` is
    // only the tie-break between robots.
    const list = [...tasks].sort((a, b) => rank(a) - rank(b) || (rank(a) === 1 ? 0 : a.at - b.at));
    $("#tasks").innerHTML = list.length ? list.map((t) => {
      const [label, cls] = TASK_LABEL(t), r = robots.get(t.robot), done = rank(t) === 2;
      return `<div class="tk ${done ? "finished" : ""}"><span class="t1">${taskTitle(t)}</span><span class="st ${cls}">${label}</span>
        ${done ? "" : `<button class="x" data-cancel="${esc(t.id)}" title="Cancel this task" aria-label="Cancel task ${esc(t.id)}">&times;</button>`}
        <span class="t2"${t.reason ? ` title="${esc(t.reason)}"` : ""}>${t.reason ? "rejected: unknown waypoint" : esc(t.id)}</span><span class="who">${r ? `<i style="--c:${r.color}"></i>${esc(nameOf(r.id))}` : ""}</span></div>`;
    }).join("") : `<div class="none">No tasks yet. Queue some below.</div>`;
    const active = tasks.filter((t) => rank(t) === 0).length, waiting = tasks.filter((t) => t.state === "QUEUED").length;
    $("#tk-count").textContent = tasks.length ? `${active} running, ${waiting} waiting` : "";
  }

  function cancelTask(id) {
    const t = tasks.find((x) => x.id === id);
    if (!t) return;
    publishString(`/${t.robot}/cancel_task`, id);
    // A waiting task is simply dropped, and nothing reports on it again. A running one
    // ends with a CANCELLED status, so leave its row to that.
    if (t.state === "QUEUED") { t.state = "CANCELLED"; t.at = Date.now(); }
    renderTasks();
  }

  // -- fleet list ---------------------------------------------------------------------------

  let lastFleetSig = "";
  const ODOM_TTL_MS = 2000; // a stale twist reads as "—", not as the last speed forever

  // What this robot is doing, coloured by kind of activity: drives the status pill
  // (cls) and the line under the name (title / sub, already HTML-escaped).
  function tileOf(r) {
    const st = statusOf(r), s = r.task;
    const waiting = r.queueDepth ? ` · ${r.queueDepth} waiting` : "";
    if (st.tag === "offline") return { cls: "off", title: "Offline", sub: st.detail };
    if (st.tag === "on a task") {
      const t = s.task ? { id: s.task_id, ...s.task } : tasks.find((x) => x.id === s.task_id) || { id: s.task_id };
      const cls = ["PICKING", "PLACING"].includes(s.state) ? "working" : "moving";
      // A move's phase is just "Moving", which the pill already says.
      if (t.kind === "goto" && t.goto) return { cls, title: `Going to ${esc(t.goto)}`, sub: waiting.slice(3) };
      return { cls, title: taskTitle(t), sub: (TASK_PHASE[s.state] || s.state) + waiting };
    }
    if (st.tag === "navigating") return { cls: "moving", title: `Going to ${esc(r.nav.label)}`, sub: "Direct goal" + waiting };
    if (st.tag === "exploring") return { cls: "exploring", title: "Exploring", sub: "Mapping new ground" + waiting };
    if (r.nav && r.nav.state === "failed") return { cls: "failed", title: "Last goal failed", sub: `Was going to ${esc(r.nav.label)}` + waiting };
    return { cls: "idle", title: "No task", sub: (r.queueDepth ? `${r.queueDepth} waiting` : "Ready for work") };
  }

  // Live numbers, patched in place every tick (see renderFleet for why not rebuilt).
  function factsOf(r) {
    const now = performance.now(), st = statusOf(r);
    const tw = r.odom && now - r.odomAt < ODOM_TTL_MS ? r.odom : null;
    // Nav2 replans from the robot's pose, so the plan's length IS the distance left;
    // once the goal is done the last plan lingers, hence the status gate.
    let togo = "—";
    if (r.plan.length > 1 && ["on a task", "navigating", "exploring"].includes(st.tag)) {
      let d = 0;
      for (let i = 1; i < r.plan.length; i++) d += Math.hypot(r.plan[i][0] - r.plan[i - 1][0], r.plan[i][1] - r.plan[i - 1][1]);
      togo = `${d.toFixed(1)} m`;
    }
    const ob = r.obstacles && now - r.obstaclesAt < OBSTACLE_TTL_MS ? r.obstacles : null;
    const nearest = ob && ob.obstacles && ob.obstacles.length ? ` · nearest ${ob.obstacles[0].range_m.toFixed(1)} m` : "";
    return {
      obst: !ob ? "—" : !ob.count ? "None in view" : `${ob.count}${ob.moving ? `, ${ob.moving} moving` : ""}${nearest}`,
      pos: r.pose ? `${r.pose.x.toFixed(2)}, ${r.pose.y.toFixed(2)} m` : "—",
      heading: r.pose ? `${Math.round(((r.pose.yaw * 180) / Math.PI + 360) % 360)}°` : "—",
      speed: tw ? `${Math.hypot(tw.linear.x, tw.linear.y).toFixed(2)} m/s` : "—",
      turn: tw ? `${Math.round((tw.angular.z * 180) / Math.PI)}°/s` : "—",
      togo,
    };
  }
  const FACTS = [["pos", "Position"], ["heading", "Heading"], ["speed", "Speed"], ["turn", "Turn rate"], ["togo", "To go"], ["obst", "Obstacles"]];

  function renderFleet() {
    const list = [...robots.values()];
    // The numbers change every few centimetres; patch them in place rather than
    // rebuilding the list under the operator's cursor (a rebuild between mousedown and
    // mouseup eats the click).
    for (const el of document.querySelectorAll("#fleet .facts[data-id]")) {
      const r = robots.get(el.dataset.id);
      if (!r) continue;
      const f = factsOf(r);
      for (const dd of el.querySelectorAll("dd[data-f]")) if (dd.textContent !== f[dd.dataset.f]) dd.textContent = f[dd.dataset.f];
    }
    const rows = list.map((r) => ({ r, st: statusOf(r), isOpen: open.has(r.id), tile: tileOf(r) }));
    const sig = JSON.stringify(rows.map((x) => [x.r.id, x.st, x.isOpen, x.r.id === selected, online(x.r), x.tile, x.r.queueDepth]));
    if (sig === lastFleetSig) return;
    lastFleetSig = sig;
    const onlineN = list.filter(online).length;
    $("#n-robots").textContent = list.length ? `${onlineN} of ${list.length} online` : "";
    $("#fleet").innerHTML = list.length ? rows.map(({ r, st, isOpen, tile }) => {
      const f = isOpen ? factsOf(r) : null, busy = st.tag === "on a task" || r.queueDepth > 0;
      return `
      <div class="robot ${isOpen ? "sel" : ""} ${r.id === selected ? "active" : ""} ${online(r) ? "" : "offline"}" data-id="${esc(r.id)}" style="--c:${r.color}">
        <div class="l1" tabindex="0" role="button" aria-expanded="${isOpen}"><span class="sw"></span><span class="name">${esc(nameOf(r.id))}</span>${r.id === selected && open.size > 1 ? `<span class="ctl" title="Map clicks and waypoint sends go to this robot">controls</span>` : ""}<span class="tag pill ${tile.cls}">${esc(st.tag)}</span></div>
        <div class="l2">${st.tag === "on a task" ? tile.title + (tile.sub ? ` · ${tile.sub}` : "") : esc(st.detail) + (r.queueDepth ? ` · ${r.queueDepth} waiting` : "")}</div>
        ${isOpen ? `<div class="more">
          <div class="cam-slot" data-cam="${esc(r.id)}"></div>
          <dl class="facts" data-id="${esc(r.id)}">${FACTS.map(([k, label]) => `<div${k === "pos" ? ' class="wide"' : k === "obst" ? ' class="full"' : ""}><dt>${label}</dt><dd data-f="${k}">${esc(f[k])}</dd></div>`).join("")}</dl>
          <div class="acts"><button class="btn small" data-act="stop">Stop</button>
            <button class="btn small" data-act="explore">${st.tag === "exploring" ? "Stop exploring" : "Explore"}</button>
            ${busy ? `<button class="btn small" data-act="clear" title="Cancel this robot's current task and everything queued behind it">Clear tasks</button>` : ""}</div>
        </div>` : ""}
      </div>`;
    }).join("") : `<div class="none">No robots found.</div>`;
    for (const slot of document.querySelectorAll("#fleet .cam-slot")) slot.replaceWith(camFor(slot.dataset.cam).el);
    renderTargets();
  }

  // Camera tile per open robot: short /snapshot polls, never MJPEG, and always
  // qos_profile=sensor_data (see app.js for the failure mode both avoid). The element
  // outlives list rebuilds (renderFleet re-slots it), so the picture doesn't blink.
  // Camera or depth, per card; a new card starts on whichever was picked last.
  const CAM_TOPIC = { rgb: "camera/rgb", depth: "camera/depth_color" };
  const cams = new Map(); // id -> {el, pic, msg, mode, timer, token}
  let camDefault = "rgb";
  try { if (localStorage.getItem("sortbots.camMode") === "depth") camDefault = "depth"; } catch (e) { /* storage blocked: rgb */ }

  function camFor(id) {
    let c = cams.get(id);
    if (c) return c;
    const el = document.createElement("div");
    el.className = "cam";
    el.innerHTML = `<div class="pic"></div><canvas class="obst"></canvas><span class="msg">Connecting to camera…</span>
      <div class="camsel" role="group" aria-label="Camera view"><button type="button" data-cam-mode="rgb">Camera</button><button type="button" data-cam-mode="depth">Depth</button></div>`;
    c = { el, pic: el.querySelector(".pic"), overlay: el.querySelector("canvas.obst"), msg: el.querySelector(".msg"), mode: camDefault, timer: null, token: 0 };
    cams.set(id, c);
    syncCamButtons(c);
    return c;
  }
  function syncCamButtons(c) {
    for (const b of c.el.querySelectorAll("[data-cam-mode]")) b.classList.toggle("on", b.dataset.camMode === c.mode);
  }
  function setCamMode(id, mode) {
    const c = camFor(id);
    if (c.mode === mode) return;
    c.mode = mode; camDefault = mode;
    try { localStorage.setItem("sortbots.camMode", mode); } catch (e) { /* this tab only */ }
    syncCamButtons(c);
    startCam(id);
  }
  function startCam(id) {
    stopCam(id);
    const c = camFor(id), token = c.token, label = c.mode === "depth" ? "depth" : "camera";
    const tick = () => {
      if (token !== c.token) return;
      const probe = new Image();
      probe.onload = () => {
        if (token !== c.token) return;
        c.pic.style.backgroundImage = `url(${probe.src})`;
        c.msg.textContent = "";
        c.timer = setTimeout(tick, 400);
      };
      probe.onerror = () => {
        if (token !== c.token) return;
        c.pic.style.backgroundImage = "";
        c.msg.textContent = `No ${label} feed`;
        c.timer = setTimeout(tick, 3000);
      };
      probe.src = `http://${HOST}:${VIDEO_PORT}/snapshot?topic=/${id}/${CAM_TOPIC[c.mode]}&qos_profile=sensor_data&_=${Date.now()}`;
    };
    c.pic.style.backgroundImage = ""; c.msg.textContent = `Connecting to ${label}…`;
    tick();
  }
  function stopCam(id) { const c = cams.get(id); if (c) { c.token++; clearTimeout(c.timer); } }

  function refreshSelection() {
    lastFleetSig = "";
    renderFleet();
    setTool(tool);
    dirty = true;
  }
  function openRobot(id) {
    const fresh = !open.has(id);
    open.add(id); selected = id;
    if (fresh) startCam(id);
    refreshSelection();
  }
  function closeRobot(id) {
    open.delete(id); stopCam(id);
    if (selected === id) selected = [...open].pop() ?? null; // control falls to the newest still open
    refreshSelection();
  }
  // select(id) opens id's card and gives it control; select(null) closes every card.
  function select(id) {
    if (id === null) { for (const x of [...open]) closeRobot(x); selected = null; refreshSelection(); }
    else openRobot(id);
  }

  // -- commands --------------------------------------------------------------------------------

  const mapPose = (x, y, yaw) => ({
    header: { frame_id: "map" },
    pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } },
  });
  let navSeq = 0;

  function sendNav(id, x, y, yaw, label) {
    const r = robots.get(id);
    if (!r) return;
    // navigate_to_pose is single-goal: an explorer still issuing its own goals would
    // preempt ours and score the abort as a failed frontier (app.js, 2026-08-02).
    if (r.explore && r.explore.state === "exploring" && performance.now() - r.exploreAt < EXPLORE_TTL_MS) {
      publishString(`/${id}/explore_cmd`, "stop");
    }
    const goalId = `sortbots_op_${Date.now()}_${navSeq++}`;
    const action = `/${id}/navigate_to_pose`;
    ros.callOnConnection({ op: "send_action_goal", id: goalId, action, action_type: "nav2_msgs/action/NavigateToPose", args: { pose: mapPose(x, y, yaw) } });
    r.nav = { id: goalId, action, label, state: "accepted", done: false, afterStamp: r.seenStamp.navigate_to_pose ?? -1, at: performance.now() };
    toast(`${nameOf(id)} is on its way to ${label}`);
    lastFleetSig = ""; renderFleet();
  }

  // Cancel EVERY goal on the action server (all-zero goal id + zero stamp is the ROS 2
  // "cancel all" request), so a task_manager goal stops too, not just ours.
  function cancelGoals(id) {
    for (const action of ["navigate_to_pose", "follow_waypoints"]) {
      new ROSLIB.Service({ ros, name: `/${id}/${action}/_action/cancel_goal`, serviceType: "action_msgs/srv/CancelGoal" })
        .callService(new ROSLIB.ServiceRequest({ goal_info: { goal_id: { uuid: new Array(16).fill(0) }, stamp: { sec: 0, nanosec: 0 } } }), () => {}, () => {});
    }
    publish(`/${id}/cmd_vel`, "geometry_msgs/Twist", { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
    const r = robots.get(id);
    if (r && r.nav && !r.nav.done) { r.nav.done = true; r.nav.state = "cancelled"; }
  }
  function stopRobot(id) {
    publishString(`/${id}/explore_cmd`, "stop");
    cancelGoals(id);
    lastFleetSig = ""; renderFleet();
  }

  function toggleExplore(id) {
    const r = robots.get(id);
    const exploring = r && r.explore && r.explore.state === "exploring" && performance.now() - r.exploreAt < EXPLORE_TTL_MS;
    publishString(`/${id}/explore_cmd`, exploring ? "stop" : "start");
  }

  // -- waypoints ------------------------------------------------------------------------------------

  async function loadWaypoints() {
    try {
      const body = await (await fetch("/api/nav_waypoints")).json();
      waypoints = body.waypoints || [];
      renderWaypoints(); dirty = true;
    } catch (e) { console.error("failed to load waypoints", e); }
  }
  async function saveWaypoints(next) {
    try {
      const res = await fetch("/api/nav_waypoints", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ waypoints: next }) });
      const body = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(body.error || res.statusText);
      waypoints = body.waypoints || next;
    } catch (e) { toast(`Couldn't save the waypoint: ${e.message || e}`); }
    renderWaypoints(); dirty = true;
  }
  // A..Z, then A2..Z2 …: the first name not taken (same scheme as the engineering view).
  function nextWaypointName() {
    const taken = new Set(waypoints.map((w) => w.name));
    for (let round = 1; ; round++) {
      for (let c = 65; c <= 90; c++) { const n = String.fromCharCode(c) + (round > 1 ? round : ""); if (!taken.has(n)) return n; }
    }
  }
  let renaming = null; // name of the chip being edited; refreshes leave #wps alone meanwhile
  function renderWaypoints() {
    renderTaskOptions();
    if (renaming) return; // the 10 s poll would otherwise wipe the input mid-word
    $("#wps").innerHTML = waypoints.length ? waypoints.map((w) =>
      `<span class="wp" data-wp="${esc(w.name)}"><button class="go" data-go="${esc(w.name)}" title="Send the selected robot here">${esc(w.name)}</button><button class="ren" data-ren="${esc(w.name)}" title="Rename ${esc(w.name)}" aria-label="Rename ${esc(w.name)}">&#9998;</button><button class="del" data-del="${esc(w.name)}" title="Delete ${esc(w.name)}" aria-label="Delete ${esc(w.name)}">&times;</button></span>`
    ).join("") : `<div class="none" style="padding:0">None yet. Pick the waypoint tool and click the map.</div>`;
  }

  // Inline rename, from the pencil (not a double-click on the label: its two clicks would
  // each send the selected robot there first). The chip's label becomes a text field. Enter or leaving the field
  // saves, Escape cancels. Same limits serve.py enforces (maps_lib.validate_waypoints),
  // checked here first so a typo gets a message instead of a 400.
  function startRename(name) {
    const chip = [...document.querySelectorAll("#wps .wp")].find((c) => c.dataset.wp === name);
    if (!chip) return;
    renaming = name;
    const input = document.createElement("input");
    input.value = name; input.maxLength = 32; input.ariaLabel = `New name for ${name}`;
    chip.querySelector(".go").replaceWith(input);
    chip.querySelector(".ren").hidden = true;
    input.focus(); input.select();
    let finished = false;
    const finish = (save) => {
      if (finished) return;
      finished = true; renaming = null;
      const next = input.value.trim();
      if (save && next !== name) renameWaypoint(name, next); else renderWaypoints();
    };
    input.addEventListener("keydown", (e) => {
      if (e.key === "Enter") { e.preventDefault(); finish(true); }
      if (e.key === "Escape") { e.preventDefault(); e.stopPropagation(); finish(false); }
    });
    input.addEventListener("blur", () => finish(true));
  }
  function renameWaypoint(from, to) {
    if (!to) { toast("A waypoint needs a name."); return renderWaypoints(); }
    if (to.length > 32) { toast("Waypoint names are at most 32 characters."); return renderWaypoints(); }
    if (waypoints.some((w) => w.name === to)) { toast(`There is already a waypoint called ${to}.`); return renderWaypoints(); }
    // Queued tasks are unaffected: task_manager resolves names to poses at dispatch.
    // The draft route isn't sent yet, so it follows the rename.
    moveStops = moveStops.map((n) => (n === from ? to : n));
    saveWaypoints(waypoints.map((w) => (w.name === from ? { ...w, name: to } : w)));
    toast(`Renamed ${from} to ${to}`);
  }

  // -- queue menu -----------------------------------------------------------------------------------------
  // Two kinds of task, one robot picker:
  //   Moves:       an ordered list of stops. Queued as one "goto" task per stop, so
  //                each stop shows (and cancels) on its own in the task list, and
  //                task_manager's FIFO runs them in order.
  //   Pick & drop: pick-up and drop-off are just waypoints (merged from the old
  //                `stations` concept 2026-10-15), so both menus list the same set.
  // Kept in sync with the map by renderWaypoints().

  let moveStops = []; // waypoint names, in order; the route being built

  function renderTaskOptions() {
    const keepFrom = $("#nt-from").value, keepTo = $("#nt-to").value;
    const opts = waypoints.map((w) => `<option>${esc(w.name)}</option>`).join("");
    $("#nt-from").innerHTML = opts || "<option value=''>(no waypoints — place some on the map)</option>";
    $("#nt-to").innerHTML = opts || "<option value=''>(no waypoints — place some on the map)</option>";
    // Keep the operator's current selections across a refresh where possible.
    if (waypoints.some((w) => w.name === keepFrom)) $("#nt-from").value = keepFrom;
    if (waypoints.some((w) => w.name === keepTo)) $("#nt-to").value = keepTo;
    // Need at least two distinct waypoints to form a pickup -> dropoff task.
    $("#nt-add").disabled = waypoints.length < 2;
    renderMoves();
  }

  function renderMoves() {
    // A deleted waypoint leaves the draft too, rather than queueing a move task_manager rejects.
    const known = new Set(waypoints.map((w) => w.name));
    moveStops = moveStops.filter((n) => known.has(n));
    const n = moveStops.length;
    $("#mv-stops").innerHTML = n ? moveStops.map((name, i) => `<li><span>${esc(name)}</span>
        <button type="button" data-up="${i}" ${i ? "" : "disabled"} title="Earlier" aria-label="Move ${esc(name)} earlier">&uarr;</button>
        <button type="button" data-down="${i}" ${i < n - 1 ? "" : "disabled"} title="Later" aria-label="Move ${esc(name)} later">&darr;</button>
        <button type="button" data-rm="${i}" title="Remove" aria-label="Remove ${esc(name)}">&times;</button></li>`).join("")
      : `<li class="empty">No stops yet. Add waypoints in the order to visit them.</li>`;
    $("#mv-add").innerHTML = `<option value="">${waypoints.length ? "Choose a waypoint…" : "(no waypoints — place some on the map)"}</option>` +
      waypoints.map((w) => `<option>${esc(w.name)}</option>`).join("");
    $("#mv-go").disabled = !n;
    $("#mv-go").textContent = n > 1 ? `Queue ${n} moves` : "Queue move";
  }

  function renderTargets() {
    const keep = $("#nt-who").value;
    $("#nt-who").innerHTML = `<option value="auto">Any free robot</option>` +
      [...robots.values()].map((r) => `<option value="${esc(r.id)}">${esc(nameOf(r.id))}</option>`).join("");
    if ([...$("#nt-who").options].some((o) => o.value === keep)) $("#nt-who").value = keep;
  }
  // "auto" -> the first free robot, or null; the caller decides whether that's a hard stop.
  const resolveRobot = (who) => (who === "auto" ? ([...robots.values()].find(isFree) || {}).id || null : who);

  // One dispatch path for the forms and the demo loader, so a demo task is
  // byte-identical to a hand-added one once queued (same topic, same optimistic
  // row). `spec` is {goto} or {pickup, dropoff}; `who` is a robot id.
  function queueTask(spec, who) {
    const id = Math.random().toString(16).slice(2, 10);
    publishString(`/${who}/dispatch_task`, JSON.stringify({ ...spec, task_id: id }));
    tasks.push({ id, kind: spec.goto ? "goto" : "pickup", ...spec, robot: who, state: "QUEUED", at: Date.now() });
    return who;
  }

  $("#task-form").addEventListener("submit", (ev) => {
    ev.preventDefault();
    const pickup = $("#nt-from").value, dropoff = $("#nt-to").value;
    if (!pickup || !dropoff) return;
    if (pickup === dropoff) return toast("Pick-up and drop-off can't be the same waypoint.");
    const who = resolveRobot($("#nt-who").value);
    if (!who) return toast("No robot is free. Pick one to queue the task on it.");
    queueTask({ pickup, dropoff }, who);
    renderTasks();
    toast(`Task queued for ${nameOf(who)}`);
  });

  $("#move-form").addEventListener("submit", (ev) => {
    ev.preventDefault();
    if (!moveStops.length) return;
    // The whole route goes to ONE robot, picked once: per-stop "auto" would scatter it.
    const who = resolveRobot($("#nt-who").value);
    if (!who) return toast("No robot is free. Pick one to queue the moves on it.");
    for (const name of moveStops) queueTask({ goto: name }, who);
    toast(`${moveStops.length > 1 ? `${moveStops.length} moves` : "Move"} queued for ${nameOf(who)}`);
    moveStops = [];
    renderMoves(); renderTasks();
  });
  $("#mv-add").addEventListener("change", (e) => {
    if (e.target.value) moveStops.push(e.target.value);
    renderMoves(); // also resets the picker, so the same stop can be added again later
  });
  $("#mv-stops").addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    const swap = (i, j) => { [moveStops[i], moveStops[j]] = [moveStops[j], moveStops[i]]; };
    if (b.dataset.rm !== undefined) moveStops.splice(+b.dataset.rm, 1);
    else if (b.dataset.up !== undefined) swap(+b.dataset.up, +b.dataset.up - 1);
    else if (b.dataset.down !== undefined) swap(+b.dataset.down, +b.dataset.down + 1);
    renderMoves();
  });
  $("#nt-kind").addEventListener("click", (e) => {
    const k = e.target.dataset.k;
    if (!k) return;
    for (const b of document.querySelectorAll("#nt-kind button")) b.classList.toggle("active", b.dataset.k === k);
    $("#move-form").hidden = k !== "moves";
    $("#task-form").hidden = k !== "pick";
  });

  // -- demo -----------------------------------------------------------------------------------------------
  // Runs configs/demo_tasks.yaml (served at /api/demo_tasks) in one click: every task
  // is queued at once, so each robot works through its own list (task_manager's
  // per-robot FIFO) while the others do the same. Entries are checked against the
  // live waypoints and roster first, so a stale name in the YAML is skipped with a
  // warning rather than queued to fail. "Stop demo" cancels what it queued.
  let demo = null; // {ids, robots}

  function demoStatus(text) {
    $("#demo-status").hidden = !text;
    $("#demo-status").textContent = text || "";
    $("#tk-demo").textContent = demo ? "Stop demo" : "Run demo";
  }

  // Called on every status update: the banner counts down, and clears when all are done.
  function demoTick() {
    if (!demo) return;
    const left = demo.ids.filter((id) => { const t = tasks.find((x) => x.id === id); return t && !FINISHED.includes(t.state); }).length;
    if (left) return demoStatus(`Demo running: ${left} of ${demo.ids.length} tasks left`);
    demo = null;
    demoStatus("");
    toast("Demo finished");
  }

  async function runDemo() {
    if (demo) { // Stop: cancel everything the demo put on the robots
      for (const id of demo.robots) publishString(`/${id}/cancel_task`, "all");
      demo = null;
      demoStatus("");
      return toast("Demo stopped");
    }
    let list = [];
    try { list = ((await (await fetch("/api/demo_tasks")).json()) || {}).tasks || []; } catch (e) { console.error("failed to load the demo", e); return toast("Couldn't load the demo."); }
    if (!list.length) return toast("No demo configured (configs/demo_tasks.yaml).");
    const roster = [...robots.values()].map((r) => r.id), known = new Set(waypoints.map((w) => w.name));
    const next = { ids: [], robots: new Set() };
    let rr = 0, skipped = 0;
    for (const t of list) {
      const spec = t.goto ? { goto: t.goto } : { pickup: t.pickup, dropoff: t.dropoff };
      const names = spec.goto ? [spec.goto] : [spec.pickup, spec.dropoff];
      if (!names.every((n) => n && known.has(n)) || (!spec.goto && spec.pickup === spec.dropoff)) { skipped++; continue; }
      const who = t.robot && t.robot !== "auto" ? (roster.includes(t.robot) ? t.robot : null) : roster[rr++ % roster.length];
      if (!who) { skipped++; continue; }
      queueTask(spec, who);
      next.ids.push(tasks[tasks.length - 1].id);
      next.robots.add(who);
    }
    if (!next.ids.length) return toast(`Nothing to run: all ${skipped} demo task(s) name a missing waypoint or robot`);
    demo = next;
    renderTasks(); demoTick();
    toast(`Demo: ${next.ids.length} tasks on ${next.robots.size} robot(s)${skipped ? `, skipped ${skipped}` : ""}`);
  }
  $("#tk-demo").addEventListener("click", runDemo);

  // -- session / header ----------------------------------------------------------------------------------------

  const ACTIVE = ["starting", "running", "stopping"];
  const sessionActive = () => !!session && ACTIVE.includes(session.state);

  function renderHeader() {
    const s = session, cfg = s && sessionActive() ? s.config : null;
    const premapped = !!cfg && cfg.map && cfg.map !== "new" && cfg.mode === "readonly";
    $("#mode-tile").hidden = !cfg;
    $("#mode-tile").className = premapped ? "loc" : "map";
    $("#mode-label").textContent = premapped ? "Premapped" : "Mapping";
    $("#map-name").textContent = cfg && cfg.map && cfg.map !== "new" ? (cfg.map === "working" ? "last map" : cfg.map) : "";
    const label = !s ? "No session" : { starting: `Starting: ${s.phase_label || "…"}`, running: "Session running", stopping: "Stopping…", failed: "Session failed" }[s.state] || "No session";
    $("#s-label").textContent = controlAvailable ? label : "Session";
    $("#s-dot").className = "dot " + (s && s.state === "running" ? "" : s && ["starting", "stopping"].includes(s.state) ? "busy" : "idle");
    const card = $("#session .card");
    card.classList.toggle("live", sessionActive());
    // The console keeps reporting the LAST session after it ends ("stopped"), which
    // read as a live one ("fleet_demo · stopped · running"). Show the strip only
    // while a run is live, or when one failed and its reason is worth reading.
    card.classList.toggle("has-session", !!s && (sessionActive() || s.state === "failed"));
    card.classList.toggle("failed", !!s && s.state === "failed");
    // A failed run's log is the only place its reason is; open it for that case.
    if (s && s.state === "failed") $("#sp-log").open = true;
    renderEmpty();
  }

  // Startup progress, from session.py's phase (its PHASE_ORDER, in run_demo.sh's
  // order) and then the robots themselves: "running" means the launcher is done, not
  // that every robot is publishing, so the last stretch waits on their poses.
  const PHASE_ORDER = ["idle", "sim_starting", "sim_loading", "sim_ready", "ros_starting", "ros_ready", "running"];
  const PHASE_TEXT = {
    idle: "Preparing", sim_starting: "Starting Isaac Sim", sim_loading: "Loading the warehouse",
    sim_ready: "Simulation up", ros_starting: "Starting navigation and mapping", ros_ready: "Navigation up",
  };
  function bootProgress() {
    if (!session || !["starting", "running"].includes(session.state)) return null;
    const list = [...robots.values()], up = list.filter(online).length;
    if (session.state === "running") {
      if (list.length && up === list.length) return null; // done: no bar
      return { p: 0.92 + 0.08 * (list.length ? up / list.length : 0), text: list.length ? `Waiting for the robots (${up} of ${list.length})` : "Waiting for the robots" };
    }
    const i = Math.max(0, PHASE_ORDER.indexOf(session.phase));
    return { p: (i + 0.5) / PHASE_ORDER.length * 0.92, text: PHASE_TEXT[session.phase] || session.phase_label || "Starting" };
  }
  function renderBoot() {
    const b = bootProgress();
    for (const el of [$("#empty-boot"), $("#sp-boot")]) {
      el.hidden = !b;
      if (b) el.querySelector("i").style.setProperty("--p", `${Math.round(b.p * 100)}%`);
    }
    if (b) $("#sp-boot .boot-label").textContent = b.text;
    const st = $("#sp-status"), live = !b && session && ["running", "stopping"].includes(session.state);
    st.hidden = !live;
    if (live) {
      const min = Math.floor((session.elapsed_s || 0) / 60);
      st.textContent = session.state === "stopping" ? "Stopping…" : `Running · ${min < 1 ? "under a minute" : min < 60 ? `${min} min` : `${Math.floor(min / 60)} h ${min % 60} min`}`;
    }
    return b;
  }

  function renderEmpty() {
    const boot = renderBoot();
    const anyOnline = [...robots.values()].some(online);
    const show = !anyOnline && !mapInfo;
    $("#empty").hidden = !show;
    if (!show) return;
    const starting = !!boot || (session && session.state === "stopping");
    $("#empty-title").textContent = boot ? "Starting up" : session && session.state === "stopping" ? "Stopping"
      : controlAvailable ? "No session running" : "Waiting for the robots";
    $("#empty-sub").textContent = boot ? boot.text : starting ? "" : controlAvailable
      ? "Start a session to see your robots and give them tasks."
      : "No robot is reporting yet. Check that the robot console is running.";
    $("#empty-go").hidden = starting || !controlAvailable;
  }

  // A session started with demo: true (the Fleet demo preset) runs the task demo by
  // itself, once, as soon as every robot reports and the waypoints are in. Remembered
  // per session id so a reload mid-demo doesn't start it over.
  function maybeAutoDemo() {
    if (!session || session.state !== "running" || !(session.config && session.config.demo) || demo) return;
    const key = `sortbots.autoDemo.${session.session_id}`;
    try { if (localStorage.getItem(key)) return; } catch (e) { /* no storage: may rerun after a reload */ }
    const list = [...robots.values()];
    if (!list.length || !list.every(online) || !waypoints.length) return;
    try { localStorage.setItem(key, "1"); } catch (e) { /* see above */ }
    toast("Starting the task demo");
    runDemo();
  }

  let sessionTimer = null, lastSessionId = null;
  async function pollSession() {
    try {
      const res = await fetch("/api/session", { cache: "no-store" });
      if (res.status === 503) { controlAvailable = false; session = null; } else {
        controlAvailable = true;
        session = await res.json();
        if (session.session_id !== lastSessionId) { // a new run: last run's tasks mean nothing
          if (lastSessionId !== null) { tasks = []; renderTasks(); }
          lastSessionId = session.session_id;
          refreshRoster();
        }
      }
    } catch (e) { /* the console is down; keep the last known state */ }
    renderHeader();
    sessionTimer = setTimeout(pollSession, sessionActive() && session.state !== "running" ? 1000 : 3000);
  }

  function openSession() {
    $("#session").hidden = false;
    if (window.sortbotsScenarios) window.sortbotsScenarios.setView("scenarios");
  }
  function closeSession() {
    $("#session").hidden = true;
    if (window.sortbotsScenarios) window.sortbotsScenarios.setView("live");
  }

  async function refreshRoster() {
    let ids = [];
    try { ids = (await (await fetch("/api/robots")).json()).robots || []; } catch (e) { console.error("failed to load robots", e); }
    if (!ids.length && !robots.size) ids = ["robot_0"];
    ids.forEach(addRobot);
  }

  // -- tools / map input -------------------------------------------------------------------------------------------

  // Sending is not a tool: drag a robot to where it should go, from any tool but the
  // waypoint one. The old "send" tool needed select-robot, pick-tool, click; three
  // steps and a mode to forget you were in.
  const HINTS = {
    select: () => "Drag a robot to send it there (drop on a pin to use it). Click a robot for its camera",
    wp: () => "Click the map to add a waypoint. Drag to set the heading",
  };
  const ROBOT_HIT_PX = 16;
  const robotAt = (px, py) => [...robots.values()].find((b) => {
    if (!b.pose) return false;
    const [a, c] = toPx(b.pose.x, b.pose.y);
    return Math.hypot(a - px, c - py) < ROBOT_HIT_PX;
  });
  const waypointAt = (px, py) => waypoints.find((w) => {
    const [a, c] = toPx(w.x, w.y);
    return Math.hypot(a - px, c - py) < 12;
  });
  let drag = null; // a robot being dragged to a destination: {id, wx, wy, wp}
  function setTool(t) {
    tool = t;
    for (const b of document.querySelectorAll("#tools button")) b.classList.toggle("active", b.dataset.t === t);
    if (!view3d) $("#hint").textContent = HINTS[t]();
    canvas.style.cursor = t === "wp" ? "crosshair" : "grab";
  }
  for (const b of document.querySelectorAll("#tools button")) b.addEventListener("click", () => setTool(b.dataset.t));

  let press = null; // {px, py, wx, wy, moved, panStart}
  canvas.addEventListener("pointerdown", (e) => {
    const r = canvas.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    const [wx, wy] = toWorld(px, py);
    // Pressing on a robot starts a send, not a pan, whatever the tool but waypoints.
    const robot = tool === "wp" ? null : robotAt(px, py);
    press = { px, py, wx, wy, moved: false, cx: view.cx, cy: view.cy, robot: robot ? robot.id : null };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    const r = canvas.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    if (!press) { // hover: say what a press here would do
      if (tool !== "wp") canvas.style.cursor = robotAt(px, py) ? "pointer" : "grab";
      return;
    }
    if (Math.hypot(px - press.px, py - press.py) > 6) press.moved = true;
    if (press.moved && press.robot) {
      const [wx, wy] = toWorld(px, py), wp = waypointAt(px, py);
      drag = { id: press.robot, wx: wp ? wp.x : wx, wy: wp ? wp.y : wy, wp: wp || null };
      canvas.style.cursor = "grabbing";
      dirty = true;
      return;
    }
    if (press.moved && tool === "select") { // drag pans
      view.moved = true;
      view.cx = press.cx - (px - press.px) / view.scale;
      view.cy = press.cy + (py - press.py) / view.scale;
      dirty = true;
    }
  });
  canvas.addEventListener("pointerup", (e) => {
    if (!press) return;
    const r = canvas.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    const p = press, d = drag; press = null; drag = null;
    if (d) {
      dirty = true;
      canvas.style.cursor = "grab";
      // Dropped back on the robot: changed its mind.
      if (robotAt(px, py) && robotAt(px, py).id === d.id) return;
      const r0 = robots.get(d.id);
      // A pin keeps its own heading; anywhere else, arrive facing the way it travelled.
      const yaw = d.wp ? d.wp.yaw || 0 : Math.atan2(d.wy - r0.pose.y, d.wx - r0.pose.x);
      return sendNav(d.id, d.wx, d.wy, yaw, d.wp ? d.wp.name : `(${d.wx.toFixed(1)}, ${d.wy.toFixed(1)})`);
    }
    if (tool === "select") {
      if (p.moved) return;
      const hit = robotAt(px, py);
      // Closed -> open it; open -> give it control; already in control -> close it.
      if (hit) { if (!open.has(hit.id) || selected !== hit.id) openRobot(hit.id); else closeRobot(hit.id); }
      return;
    }
    const [wx, wy] = toWorld(px, py);
    const yaw = p.moved ? Math.atan2(wy - p.wy, wx - p.wx) : 0; // heading from the drag
    if (tool === "wp") {
      const name = nextWaypointName();
      saveWaypoints([...waypoints, { name, x: p.wx, y: p.wy, yaw }]);
      toast(`Waypoint ${name} added`);
    }
  });
  canvas.addEventListener("pointercancel", () => { press = null; if (drag) { drag = null; dirty = true; } });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = canvas.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    const [wx, wy] = toWorld(px, py), k = Math.exp(-e.deltaY * 0.0015);
    view.scale = Math.min(200, Math.max(2, view.scale * k));
    view.cx = wx - (px - cssW / 2) / view.scale; view.cy = wy + (py - cssH / 2) / view.scale; // keep the point under the cursor
    view.moved = true; dirty = true;
  }, { passive: false });

  // -- DOM events ---------------------------------------------------------------------------------------------------

  $("#fleet").addEventListener("click", (e) => {
    const row = e.target.closest(".robot");
    if (!row) return;
    const id = row.dataset.id;
    const act = e.target.closest("[data-act]"), cam = e.target.closest("[data-cam-mode]");
    if (cam) return setCamMode(id, cam.dataset.camMode);
    if (act) {
      if (act.dataset.act === "stop") { stopRobot(id); toast(`${nameOf(id)} stopped`); }
      if (act.dataset.act === "explore") toggleExplore(id);
      if (act.dataset.act === "clear") { publishString(`/${id}/cancel_task`, "all"); toast(`Cleared ${nameOf(id)}'s tasks`); }
      return;
    }
    if (e.target.closest("a")) return;
    // Inside an open card: take control without collapsing it. The header toggles.
    if (e.target.closest(".more")) { if (selected !== id) { selected = id; refreshSelection(); } return; }
    if (open.has(id)) closeRobot(id); else openRobot(id);
  });
  $("#fleet").addEventListener("keydown", (e) => {
    if ((e.key === "Enter" || e.key === " ") && e.target.classList.contains("l1")) { e.preventDefault(); e.target.click(); }
  });
  $("#tasks").addEventListener("click", (e) => {
    const id = e.target.dataset.cancel;
    if (id) cancelTask(id);
  });
  $("#wps").addEventListener("click", (e) => {
    const go = e.target.dataset.go, del = e.target.dataset.del, ren = e.target.dataset.ren;
    if (ren) return startRename(ren);
    if (go) {
      const w = waypoints.find((x) => x.name === go);
      if (!selected) return toast("Select a robot first, then pick a waypoint");
      if (w) sendNav(selected, w.x, w.y, w.yaw || 0, w.name);
    } else if (del && confirm(`Delete waypoint ${del}?`)) {
      saveWaypoints(waypoints.filter((w) => w.name !== del));
    }
  });
  $("#estop").addEventListener("click", () => {
    for (const id of robots.keys()) stopRobot(id);
    toast("Stopped every robot");
  });
  $("#session-chip").addEventListener("click", openSession);
  $("#empty-go").addEventListener("click", openSession);
  $("#sp-x").addEventListener("click", closeSession);
  $("#session").addEventListener("click", (e) => { if (e.target.id === "session") closeSession(); });
  addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      if (drag) { drag = null; press = null; dirty = true; canvas.style.cursor = "grab"; return; } // abandon a send mid-drag
      if (!$("#session").hidden) closeSession(); else if (selected) select(null);
      return;
    }
    if (e.target.closest && e.target.closest("input, select, textarea") || e.metaKey || e.ctrlKey || e.altKey) return;
    const t = { v: "select", f: "wp" }[e.key.toLowerCase()];
    if (t) setTool(t);
  });
  addEventListener("resize", resize);

  // -- 2D map / 3D cloud --------------------------------------------------------------------------------------------
  // The 3D view is look-only: the tools (waypoints, sends) work on the 2D map, so they
  // hide with it. operator_cloud.js subscribes to the cloud only while it is showing.
  let cloudMounted = false;
  function setView(v3) {
    view3d = v3 && !!window.sortbotsCloud;
    for (const b of document.querySelectorAll("#view button")) b.classList.toggle("active", (b.dataset.v === "3d") === view3d);
    // style.display, not [hidden]: #map / #tools set their own display, which beats the attribute.
    canvas.style.display = $("#tools").style.display = view3d ? "none" : "";
    $("#cloud").hidden = !view3d;
    if (view3d) {
      if (!cloudMounted) { window.sortbotsCloud.mount($("#cloud"), ros); cloudMounted = true; }
      window.sortbotsCloud.show(); syncCloudRobots();
      $("#hint").textContent = "Drag to orbit, scroll to zoom, right-drag to pan";
    } else {
      if (cloudMounted) window.sortbotsCloud.hide();
      setTool(tool); dirty = true;
    }
    try { localStorage.setItem("sortbots.mapView", view3d ? "3d" : "2d"); } catch (e) { /* this tab only */ }
  }
  const syncCloudRobots = () => window.sortbotsCloud.setRobots([...robots.values()].map((r) => ({ id: r.id, color: r.color, pose: r.pose })));
  $("#view").addEventListener("click", (e) => { if (e.target.dataset.v) setView(e.target.dataset.v === "3d"); });

  let toastTimer = null;
  function toast(msg) {
    const t = $("#toast");
    t.textContent = msg; t.classList.add("on");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.remove("on"), 2200);
  }

  function render() {
    $("#coverage").hidden = !mapInfo;
    if (mapInfo) $("#cov-n").textContent = `${Math.round(mappedM2).toLocaleString()} m²`;
    dirty = true;
    renderEmpty();
  }

  // -- tick: poses, statuses, redraw -----------------------------------------------------------------------------------------
  setInterval(() => {
    let moved = false;
    for (const r of robots.values()) {
      const p = lookupPose(`${r.id}/base_link`);
      if (p && (!r.pose || Math.hypot(p.x - r.pose.x, p.y - r.pose.y) > 0.01 || Math.abs(p.yaw - r.pose.yaw) > 0.02)) { r.pose = p; moved = true; }
    }
    if (moved) { dirty = true; if (!view.moved) fitView(); if (view3d) syncCloudRobots(); }
    if (worldTruth && !worldLive() && performance.now() - worldTruthAt < WORLD_TRUTH_TTL_MS + 500) dirty = true; // erase stale walkers
    for (const r of robots.values()) {
      if (!r.obstacles || performance.now() - r.obstaclesAt <= OBSTACLE_TTL_MS) continue;
      if (r.obstaclesMap.some((o) => o.moving)) dirty = true;
      r.obstacles = null; r.obstaclesMap = [];
      drawCamObstacles(r.id); // clears it
    }
    renderFleet(); renderEmpty(); maybeAutoDemo();
  }, 250);

  // -- boot ----------------------------------------------------------------------------------------------------------------------
  applyTheme(); resize(); setTool("select"); renderTasks(); renderWaypoints();
  refreshRoster(); loadWaypoints(); pollSession();
  try { if (localStorage.getItem("sortbots.mapView") === "3d") setView(true); } catch (e) { /* default: 2D */ }
  setInterval(loadWaypoints, 10000); // another operator may have added pins
  setInterval(refreshRoster, 15000);

  // Test hook: the offline test (tests/operator_test.mjs) drives state through here.
  window.__operator = { robots, tfTree, lookupPose, onTf, addRobot, statusOf, noteStatus, get tasks() { return tasks; }, get waypoints() { return waypoints; }, select, open, get selected() { return selected; }, cams, loadWaypoints, view, toPx, toWorld, obstaclesInMap, onObstacles, get drag() { return drag; } };
})();
