// SortBots dashboard. Vanilla JS, no build step. Talks to:
//   - rosbridge_websocket (ws://<host>:9090)   — pub/sub for dispatch/status/map/tf
//   - web_video_server    (http://<host>:8080) — MJPEG camera stream
//   - webui/serve.py      (this page's own origin) — /api/waypoints
// All three are brought up together by launch/sortbots_webui.launch.py.

const ROBOT_ID = "robot_0";
const ROSBRIDGE_HOST = window.location.hostname || "localhost";
const ROSBRIDGE_PORT = 9090;
const VIDEO_PORT = 8080;
const MAX_TRAIL_POINTS = 3000;

// -- connection -------------------------------------------------------

const ros = new ROSLIB.Ros({ url: `ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}` });
const connDot = document.getElementById("conn-dot");
const connLabel = document.getElementById("conn-label");

ros.on("connection", () => {
  connDot.classList.add("connected");
  connLabel.textContent = `connected (${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT})`;
});
ros.on("close", () => {
  connDot.classList.remove("connected");
  connLabel.textContent = "disconnected — retrying…";
  setTimeout(() => ros.connect(`ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`), 2000);
});
// Every ROSLIB.Topic below passes reconnect_on_close: true. Without it,
// reconnecting here only reopens the websocket — roslib does NOT resend the
// original subscribe/advertise ops, so every panel silently goes stale (while
// the header still shows "connected") until the page is hard-refreshed. This
// bit us for real: rosbridge_websocket restarting (e.g. as part of a demo
// restart) killed every subscription in an already-open tab.
ros.on("error", (e) => {
  connDot.classList.remove("connected");
  connLabel.textContent = "connection error";
  console.error("rosbridge error", e);
});

// -- camera streams -------------------------------------------------------
// Snapshot polling, NOT MJPEG streams (/stream?type=mjpeg), on purpose: the
// jazzy web_video_server recurrently livelocks while serving long-lived MJPEG
// connections (~90% CPU spin, accepts TCP but never responds, needs SIGKILL —
// observed on 4 instances within 3-20 min, with and without use_sim_time; see
// nodes/web_video_watchdog.sh). Short /snapshot requests open and close
// cleanly, sidestepping the stream-lifecycle bug entirely. ~2.5fps effective,
// which is fine for a dashboard. Self-pacing: the next request only fires
// after the previous one completes, and errors just slow the loop down — so
// this also self-heals across web_video_server restarts with no extra logic.
const CAMERA_POLL_MS = 400;
const CAMERA_ERROR_BACKOFF_MS = 1500;

// A feed that isn't on the stage (neither .stage-main nor .stage-pip) isn't
// worth fetching — in map mode that drops the head cam's polling entirely.
// The timer keeps ticking while hidden rather than being cleared, so the feed
// resumes on its own when the stage swaps, with no restart wiring.
function isFeedVisible(img) {
  return img.classList.contains("stage-main") || img.classList.contains("stage-pip");
}

function wireCameraStream(elId, topic) {
  const img = document.getElementById(elId);
  function tick() {
    if (!isFeedVisible(img)) {
      setTimeout(tick, CAMERA_POLL_MS);
      return;
    }
    const probe = new Image();
    // Swap the visible src only once the new frame is fully loaded, so the
    // panel never flashes blank between polls.
    probe.onload = () => {
      img.src = probe.src;
      setTimeout(tick, CAMERA_POLL_MS);
    };
    probe.onerror = () => setTimeout(tick, CAMERA_ERROR_BACKOFF_MS);
    probe.src =
      `http://${ROSBRIDGE_HOST}:${VIDEO_PORT}/snapshot?topic=/${ROBOT_ID}/${topic}&_=${Date.now()}`;
  }
  tick();
}

wireCameraStream("camera-stream", "camera/rgb");
wireCameraStream("chase-stream", "camera/chase/rgb");

// -- stage mode -----------------------------------------------------------
// The stage box shows exactly one main view, plus at most one picture-in-
// picture inset:
//
//   "chase" -> chase cam large, head cam as PiP   (default)
//   "head"  -> head cam large,  chase cam as PiP  (clicking the PiP swaps)
//   "map"   -> map large,       chase cam as PiP
//
// Clicking the PiP swaps the two cameras; the header's camera/map buttons
// switch between the map and whichever camera mode was last active.

const STAGE_LABELS = {
  chase: "3rd person (chase cam)",
  head: "Head camera",
  map: "Map, trail & nav",
};

const chaseEl = document.getElementById("chase-stream");
const headEl = document.getElementById("camera-stream");
const mapViewEl = document.getElementById("map-view");
const stageLabel = document.getElementById("stage-label");
const pipHint = document.getElementById("pip-hint");
const aimOverlay = document.getElementById("aim-overlay");

let stageMode = "chase";
let lastCameraMode = "chase"; // restored when switching back from the map

// Set by initRecon() below so a stage swap can re-fit the three.js canvas.
let onStageResize = () => {};

function setStageMode(mode) {
  stageMode = mode;
  if (mode !== "map") lastCameraMode = mode;

  const main = { chase: chaseEl, head: headEl, map: mapViewEl }[mode];
  // In map mode the chase cam keeps a corner so you never lose sight of the
  // robot while picking a goal.
  const pip = mode === "chase" ? headEl : chaseEl;

  for (const el of [chaseEl, headEl, mapViewEl]) {
    el.classList.toggle("stage-main", el === main);
    el.classList.toggle("stage-pip", el === pip);
  }

  stageLabel.textContent = STAGE_LABELS[mode];
  pipHint.style.display = pip ? "" : "none";
  // Head pan/tilt has no meaning while looking at the map.
  aimOverlay.style.display = mode === "map" ? "none" : "";

  for (const btn of document.querySelectorAll("#stage-mode button")) {
    btn.classList.toggle("active", (btn.dataset.stage === "map") === (mode === "map"));
  }
  onStageResize();
}

// Clicking either camera while it's the inset promotes it to main.
for (const el of [chaseEl, headEl]) {
  el.addEventListener("click", () => {
    if (el.classList.contains("stage-pip")) {
      setStageMode(el === chaseEl ? "chase" : "head");
    }
  });
}

for (const btn of document.querySelectorAll("#stage-mode button")) {
  btn.addEventListener("click", () => {
    setStageMode(btn.dataset.stage === "map" ? "map" : lastCameraMode);
  });
}

setStageMode("chase");

// -- camera aim (head pan/tilt) -------------------------------------------
// Unlike cmd_vel, head_cmd is a POSITION target (see spawn_warehouse.py's
// HEAD_JOINT_LIMITS), so holding a direction accumulates pan/tilt rather
// than commanding a constant rate. Same held-button/interval pattern as
// manual drive, just with position accumulation instead of a velocity sum.

const HEAD_PAN_LIMITS = [-1.57, 1.57];
const HEAD_TILT_LIMITS = [-0.76, 1.45];
const AIM_STEP_RAD = 0.05;
const AIM_HZ = 10;

const AIM_DELTAS = {
  "pan-left": { pan: AIM_STEP_RAD, tilt: 0 },
  "pan-right": { pan: -AIM_STEP_RAD, tilt: 0 },
  "tilt-up": { pan: 0, tilt: AIM_STEP_RAD },
  "tilt-down": { pan: 0, tilt: -AIM_STEP_RAD },
};

const headCmdTopic = new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/head_cmd`,
  messageType: "sensor_msgs/JointState",
  reconnect_on_close: true,
});

const aimState = { pan: 0, tilt: 0 };
const heldAimCmds = new Set();
const aimStatus = document.getElementById("aim-status");
let aimTimer = null;

function clamp(v, [lo, hi]) {
  return Math.max(lo, Math.min(hi, v));
}

function publishHeadCmd() {
  headCmdTopic.publish(new ROSLIB.Message({
    name: ["head_pan_joint", "head_tilt_joint"],
    position: [aimState.pan, aimState.tilt],
  }));
  aimStatus.textContent = `pan=${aimState.pan.toFixed(2)} tilt=${aimState.tilt.toFixed(2)}`;
}

function aimTick() {
  for (const cmd of heldAimCmds) {
    const d = AIM_DELTAS[cmd];
    if (!d) continue;
    aimState.pan = clamp(aimState.pan + d.pan, HEAD_PAN_LIMITS);
    aimState.tilt = clamp(aimState.tilt + d.tilt, HEAD_TILT_LIMITS);
  }
  if (heldAimCmds.size) publishHeadCmd();
}

function startAim(cmd) {
  if (cmd === "center") {
    heldAimCmds.clear();
    aimState.pan = 0;
    aimState.tilt = 0;
    publishHeadCmd();
    return;
  }
  if (!AIM_DELTAS[cmd] || heldAimCmds.has(cmd)) return;
  heldAimCmds.add(cmd);
  document.querySelector(`#aim-pad [data-aim="${cmd}"]`)?.classList.add("held");
  aimTick();
  if (!aimTimer) aimTimer = setInterval(aimTick, 1000 / AIM_HZ);
}

function stopAim(cmd) {
  if (!heldAimCmds.delete(cmd)) return;
  document.querySelector(`#aim-pad [data-aim="${cmd}"]`)?.classList.remove("held");
  if (heldAimCmds.size === 0 && aimTimer) {
    clearInterval(aimTimer);
    aimTimer = null;
  }
}

const aimPad = document.getElementById("aim-pad");
for (const btn of aimPad.querySelectorAll("button[data-aim]")) {
  const cmd = btn.dataset.aim;
  btn.addEventListener("mousedown", () => startAim(cmd));
  btn.addEventListener("mouseup", () => stopAim(cmd));
  btn.addEventListener("mouseleave", () => stopAim(cmd));
  btn.addEventListener("touchstart", (e) => { e.preventDefault(); startAim(cmd); });
  btn.addEventListener("touchend", (e) => { e.preventDefault(); stopAim(cmd); });
}

// -- manual drive --------------------------------------------------------
// Publishes geometry_msgs/Twist to cmd_vel while a direction is held (mouse
// or keyboard). XLeRobot's base is holonomic — see
// spawn_warehouse.py:_drive_velocities — so linear.x (forward/back) and
// linear.y (strafe) both do something, plus angular.z (rotate in place).
// Multiple directions can be held at once (e.g. forward + strafe) for
// diagonal motion. Republishes at DRIVE_HZ while any direction is held so a
// single dropped rosbridge message can't leave the robot creeping; stops
// immediately (not just on the next tick) the moment nothing is held.

// 0.5 m/s is the base prismatic joints' URDF velocity limit (root_x/root_y,
// vel=0.50) — the sim's hard ceiling on translation, so commanding more just
// saturates. Pure fwd/back/strafe hit it exactly; diagonals (fwd+strafe) clamp
// slightly. To go faster the joint limit must be raised + the USD regenerated.
const DRIVE_LINEAR_MPS = 0.5;
const DRIVE_ANGULAR_RADPS = 1.2; // root_z_rotation has no URDF limit; free to raise
const DRIVE_HZ = 10;

const DRIVE_VECTORS = {
  fwd: { x: DRIVE_LINEAR_MPS, y: 0, yaw: 0 },
  back: { x: -DRIVE_LINEAR_MPS, y: 0, yaw: 0 },
  left: { x: 0, y: DRIVE_LINEAR_MPS, yaw: 0 },
  right: { x: 0, y: -DRIVE_LINEAR_MPS, yaw: 0 },
  "rot-left": { x: 0, y: 0, yaw: DRIVE_ANGULAR_RADPS },
  "rot-right": { x: 0, y: 0, yaw: -DRIVE_ANGULAR_RADPS },
};
const KEY_TO_CMD = {
  w: "fwd", s: "back", a: "left", d: "right", q: "rot-left", e: "rot-right",
};

const cmdVelTopic = new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/cmd_vel`,
  messageType: "geometry_msgs/Twist",
  reconnect_on_close: true,
});

const heldCmds = new Set();
const driveStatus = document.getElementById("drive-status");
let driveTimer = null;

function publishDriveTwist() {
  let x = 0, y = 0, yaw = 0;
  for (const cmd of heldCmds) {
    const v = DRIVE_VECTORS[cmd];
    if (!v) continue;
    x += v.x; y += v.y; yaw += v.yaw;
  }
  cmdVelTopic.publish(new ROSLIB.Message({
    linear: { x, y, z: 0 },
    angular: { x: 0, y: 0, z: yaw },
  }));
  driveStatus.textContent = heldCmds.size
    ? `driving: x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${yaw.toFixed(2)}`
    : "stopped";
}

function startCmd(cmd) {
  if (cmd === "stop") {
    heldCmds.clear();
    publishDriveTwist();
    return;
  }
  if (!DRIVE_VECTORS[cmd] || heldCmds.has(cmd)) return;
  heldCmds.add(cmd);
  document.querySelector(`#drive-pad [data-cmd="${cmd}"]`)?.classList.add("held");
  publishDriveTwist();
  if (!driveTimer) driveTimer = setInterval(publishDriveTwist, 1000 / DRIVE_HZ);
}

function stopCmd(cmd) {
  if (!heldCmds.delete(cmd)) return;
  document.querySelector(`#drive-pad [data-cmd="${cmd}"]`)?.classList.remove("held");
  publishDriveTwist(); // stop immediately, don't wait for the next tick
  if (heldCmds.size === 0 && driveTimer) {
    clearInterval(driveTimer);
    driveTimer = null;
  }
}

// The overlays sit at low opacity until pointed at. Hover covers mouse users;
// touch has no hover, so a tap marks the overlay active (and it fades back a
// few seconds after the last touch).
for (const overlay of document.querySelectorAll(".pad-overlay")) {
  let fadeTimer = null;
  overlay.addEventListener("touchstart", () => {
    overlay.classList.add("pad-active");
    if (fadeTimer) clearTimeout(fadeTimer);
  }, { passive: true });
  overlay.addEventListener("touchend", () => {
    if (fadeTimer) clearTimeout(fadeTimer);
    fadeTimer = setTimeout(() => overlay.classList.remove("pad-active"), 3000);
  }, { passive: true });
}

const drivePad = document.getElementById("drive-pad");
for (const btn of drivePad.querySelectorAll("button[data-cmd]")) {
  const cmd = btn.dataset.cmd;
  btn.addEventListener("mousedown", () => startCmd(cmd));
  btn.addEventListener("mouseup", () => stopCmd(cmd));
  btn.addEventListener("mouseleave", () => stopCmd(cmd));
  btn.addEventListener("touchstart", (e) => { e.preventDefault(); startCmd(cmd); });
  btn.addEventListener("touchend", (e) => { e.preventDefault(); stopCmd(cmd); });
}

// Keyboard is captured at the window, not on the focused pad: the pad is now a
// translucent overlay on the video, and "click the pad before WASD works" was
// the single most confusing thing about the old layout. The form guard below
// is what the old focus-scoping bought us — without it, typing in the dispatch
// selects would drive the robot.
function isTypingTarget(el) {
  if (!el) return false;
  const tag = el.tagName;
  return tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA" || el.isContentEditable;
}

window.addEventListener("keydown", (ev) => {
  if (ev.repeat) return; // browser key-repeat is handled by our own interval
  if (isTypingTarget(ev.target)) return;
  if (ev.ctrlKey || ev.metaKey || ev.altKey) return; // don't eat browser shortcuts
  if (ev.code === "Space") { ev.preventDefault(); startCmd("stop"); return; }
  const cmd = KEY_TO_CMD[ev.key.toLowerCase()];
  if (cmd) { ev.preventDefault(); startCmd(cmd); }
});
window.addEventListener("keyup", (ev) => {
  if (isTypingTarget(ev.target)) return;
  const cmd = KEY_TO_CMD[ev.key.toLowerCase()];
  if (cmd) stopCmd(cmd);
});

// Losing focus mid-drive (alt-tab, switching apps) must not leave the robot
// creeping — the keyup would never arrive. Same safety net as the old pad
// blur handler, moved to the window since that's where keys are read now.
function releaseAllDriveKeys() {
  if (!heldCmds.size) return;
  for (const cmd of heldCmds) {
    document.querySelector(`#drive-pad [data-cmd="${cmd}"]`)?.classList.remove("held");
  }
  heldCmds.clear();
  publishDriveTwist();
  if (driveTimer) { clearInterval(driveTimer); driveTimer = null; }
}
window.addEventListener("blur", releaseAllDriveKeys);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) releaseAllDriveKeys();
});

// -- dispatch form (populated from configs/waypoints.yaml via serve.py) --

const pickupSelect = document.getElementById("pickup-select");
const dropoffSelect = document.getElementById("dropoff-select");

fetch("/api/waypoints")
  .then((r) => r.json())
  .then((stations) => {
    for (const [name, spec] of Object.entries(stations)) {
      const o = document.createElement("option");
      o.value = name;
      o.textContent = name;
      const target = spec.kind === "pickup" ? pickupSelect : dropoffSelect;
      target.appendChild(o);
    }
  })
  .catch((e) => console.error("failed to load waypoints", e));

const dispatchTopic = new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/dispatch_task`,
  messageType: "std_msgs/String",
  reconnect_on_close: true,
});

document.getElementById("dispatch-form").addEventListener("submit", (ev) => {
  ev.preventDefault();
  const payload = { pickup: pickupSelect.value, dropoff: dropoffSelect.value };
  dispatchTopic.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
});

// -- task queue / status --------------------------------------------

const tasks = new Map(); // task_id -> {state, queue_depth, updatedAt}
const queueList = document.getElementById("queue-list");

function renderQueue() {
  queueList.innerHTML = "";
  const entries = [...tasks.entries()].sort((a, b) => b[1].updatedAt - a[1].updatedAt);
  for (const [taskId, info] of entries) {
    const li = document.createElement("li");
    li.innerHTML =
      `<span>${taskId}</span>` +
      `<span class="state-badge state-${info.state}">${info.state}</span>`;
    queueList.appendChild(li);
  }
}

new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/task_status`,
  messageType: "std_msgs/String",
  reconnect_on_close: true,
}).subscribe((msg) => {
  let status;
  try {
    status = JSON.parse(msg.data);
  } catch (e) {
    return;
  }
  if (!status.task_id) return; // idle heartbeat with no active task
  tasks.set(status.task_id, { state: status.state, updatedAt: performance.now() });
  renderQueue();
});

// -- map + robot trail ------------------------------------------------

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
let mapInfo = null;
let mapImageData = null;
const trail = [];
let lastPose = null; // {x, y, yaw}
let plan = []; // Nav2 planned path, world [x, y] points
let goal = null; // {x, y, yaw} — current nav target marker
let costmapCanvas = null; // offscreen render of the global costmap (own grid)

// Pixel<->world use the SLAM map's grid (mapInfo). worldToPixel is also used
// to place the costmap/plan/goal overlays, so everything shares one frame.
function worldToPixel(x, y) {
  const col = (x - mapInfo.origin.position.x) / mapInfo.resolution;
  const row = (y - mapInfo.origin.position.y) / mapInfo.resolution;
  return [col, mapInfo.height - 1 - row];
}

// Inverse of worldToPixel: canvas pixel (map-grid coords) -> world (map frame).
function pixelToWorld(col, row) {
  const x = col * mapInfo.resolution + mapInfo.origin.position.x;
  const y = (mapInfo.height - 1 - row) * mapInfo.resolution + mapInfo.origin.position.y;
  return [x, y];
}

// Planar yaw from a quaternion's z/w (roll/pitch ~0 for the base and 2D goals).
function yawFromQuat(qz, qw) {
  return Math.atan2(2 * qw * qz, 1 - 2 * qz * qz);
}

new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/map`,
  messageType: "nav_msgs/OccupancyGrid",
  reconnect_on_close: true,
}).subscribe((msg) => {
  mapInfo = msg.info;
  const w = msg.info.width;
  const h = msg.info.height;
  canvas.width = w;
  canvas.height = h;
  // Display the canvas at the grid's own aspect ratio (CSS caps it to the
  // stage box). This keeps the map undistorted AND keeps the element's
  // bounding rect equal to the drawn image, which eventToCanvasPixel() below
  // depends on to invert clicks back into world coordinates.
  canvas.style.aspectRatio = `${w} / ${h}`;
  const imgData = ctx.createImageData(w, h);
  const data = msg.data;
  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const v = data[row * w + col];
      const gray = v < 0 ? 128 : Math.round(255 * (1 - v / 100));
      const dstRow = h - 1 - row;
      const i = (dstRow * w + col) * 4;
      imgData.data[i] = gray;
      imgData.data[i + 1] = gray;
      imgData.data[i + 2] = gray;
      imgData.data[i + 3] = 255;
    }
  }
  mapImageData = imgData;
});

// -- Nav2 global costmap overlay (toggleable) -----------------------------
// Its own OccupancyGrid with its own origin/resolution — NOT necessarily the
// same grid as the SLAM map, so it's rendered to a separate offscreen canvas
// (in its own cell space) and blitted in world coords during drawFrame().
// 0..100 cost -> translucent yellow->red tint; 0 and unknown(-1) stay clear
// so the map underneath still shows through.
const costmapToggle = document.getElementById("costmap-toggle");
let costmapInfo = null;

new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/global_costmap/costmap`,
  messageType: "nav_msgs/OccupancyGrid",
  reconnect_on_close: true,
}).subscribe((msg) => {
  costmapInfo = msg.info;
  const w = msg.info.width;
  const h = msg.info.height;
  const off = costmapCanvas || document.createElement("canvas");
  off.width = w;
  off.height = h;
  const octx = off.getContext("2d");
  const img = octx.createImageData(w, h);
  const data = msg.data;
  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const v = data[row * w + col];
      const dstRow = h - 1 - row; // match worldToPixel's row flip
      const i = (dstRow * w + col) * 4;
      if (v <= 0) {
        img.data[i + 3] = 0; // free or unknown -> transparent
        continue;
      }
      // Yellow (low cost) -> red (lethal). Alpha grows with cost but capped
      // so the map underneath stays legible.
      const t = v / 100;
      img.data[i] = 255;
      img.data[i + 1] = Math.round(200 * (1 - t));
      img.data[i + 2] = 0;
      img.data[i + 3] = Math.round(40 + 120 * t);
    }
  }
  octx.putImageData(img, 0, 0);
  costmapCanvas = off;
});

// -- Nav2 planned path ----------------------------------------------------
// The path's final pose is the current nav goal — for BOTH click-to-nav and
// dispatched pickup/dropoff tasks — so the goal marker is driven from here,
// keeping it correct no matter who issued the goal. (task_status carries no
// station coords, so this is the only reliable source for a dispatched goal.)
new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/plan`,
  messageType: "nav_msgs/Path",
  reconnect_on_close: true,
}).subscribe((msg) => {
  plan = msg.poses.map((ps) => [ps.pose.position.x, ps.pose.position.y]);
  if (msg.poses.length) {
    const last = msg.poses[msg.poses.length - 1].pose;
    goal = {
      x: last.position.x,
      y: last.position.y,
      yaw: yawFromQuat(last.orientation.z, last.orientation.w),
    };
  }
});

// -- SLAM status (RTAB-Map Info) ------------------------------------------
// Only two fields are read from the rtabmap_msgs/Info: loop_closure_id (>0
// means a loop just closed this update) and header.stamp (freshness). The
// badge flashes green on a closure and counts total closures; it goes stale
// (red) if Info stops arriving.
const slamBadge = document.getElementById("slam-badge");
let loopClosureCount = 0;
let lastInfoAt = 0;
let loopFlashUntil = 0;

new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/info`,
  messageType: "rtabmap_msgs/Info",
  reconnect_on_close: true,
}).subscribe((msg) => {
  lastInfoAt = performance.now();
  if (msg.loop_closure_id && msg.loop_closure_id > 0) {
    loopClosureCount += 1;
    loopFlashUntil = lastInfoAt + 1500;
  }
});

function updateSlamBadge() {
  if (lastInfoAt === 0) {
    slamBadge.textContent = "SLAM: waiting…";
    slamBadge.className = "";
    return;
  }
  const now = performance.now();
  if (now - lastInfoAt > 3000) {
    slamBadge.textContent = `SLAM: stale · loops ${loopClosureCount}`;
    slamBadge.className = "stale";
  } else if (now < loopFlashUntil) {
    slamBadge.textContent = `SLAM: loop closed · loops ${loopClosureCount}`;
    slamBadge.className = "loop-closed";
  } else {
    slamBadge.textContent = `SLAM: mapping · loops ${loopClosureCount}`;
    slamBadge.className = "";
  }
}
setInterval(updateSlamBadge, 250);

// -- robot pose from /tf ---------------------------------------------------
// This used to be a ROSLIB.TFClient. That does NOT work here and never did:
// the vendored roslib implements TFClient by calling a `/republish_tfs`
// service of type `tf2_web_republisher/RepublishTFs`, and that package is not
// installed, is not provided by rosbridge (see rosbridge_library/capabilities/
// — there is no TF capability), and isn't packaged for Jazzy at all. The
// service call simply never resolved, so `lastPose` stayed null forever and
// the robot marker, the odom trail and the 3D panel's marker were all dead.
//
// Instead: subscribe to /tf and /tf_static directly and compose the chain
//   map -> robot_0/odom      (published by RTAB-Map)
//   robot_0/odom -> robot_0/base_link  (published by the sim)
// ourselves. Frame ids are normalised because ROS 1 bags and some publishers
// keep a leading "/" while ROS 2 does not.

const TF_TARGET_FRAME = `${ROBOT_ID}/base_link`;
const TF_FIXED_FRAME = "map";

// child frame -> { parent, t: {x,y,z}, q: {x,y,z,w} }
const tfTree = new Map();

const normFrame = (f) => (f || "").replace(/^\//, "");

function quatMul(a, b) {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

// Rotate vector v by quaternion q (v' = q * v * q^-1, expanded).
function quatRotate(q, v) {
  const { x, y, z, w } = q;
  const tx = 2 * (y * v.z - z * v.y);
  const ty = 2 * (z * v.x - x * v.z);
  const tz = 2 * (x * v.y - y * v.x);
  return {
    x: v.x + w * tx + (y * tz - z * ty),
    y: v.y + w * ty + (z * tx - x * tz),
    z: v.z + w * tz + (x * ty - y * tx),
  };
}

// Compose parent<-child onto an accumulated child->target transform.
function tfCompose(parent, child) {
  const r = quatRotate(parent.q, child.t);
  return {
    t: { x: parent.t.x + r.x, y: parent.t.y + r.y, z: parent.t.z + r.z },
    q: quatMul(parent.q, child.q),
  };
}

// Walk `frame` up to TF_FIXED_FRAME, composing as we go. Returns null while
// any link in the chain is still missing (e.g. before RTAB-Map publishes
// map->odom), which is why the caller treats null as "no pose yet".
function lookupPose(frame) {
  let acc = { t: { x: 0, y: 0, z: 0 }, q: { x: 0, y: 0, z: 0, w: 1 } };
  let cur = normFrame(frame);
  // Bounded so a malformed tree (a cycle, or a frame reparented mid-run)
  // can't spin forever inside a /tf callback.
  for (let hops = 0; hops < 16; hops++) {
    if (cur === TF_FIXED_FRAME) return acc;
    const link = tfTree.get(cur);
    if (!link) return null;
    acc = tfCompose({ t: link.t, q: link.q }, acc);
    cur = link.parent;
  }
  return null;
}

function onTfMessage(msg) {
  for (const tr of msg.transforms || []) {
    tfTree.set(normFrame(tr.child_frame_id), {
      parent: normFrame(tr.header.frame_id),
      t: tr.transform.translation,
      q: tr.transform.rotation,
    });
  }
  const pose = lookupPose(TF_TARGET_FRAME);
  if (!pose) return;
  const { x, y } = pose.t;
  const yaw = yawFromQuat(pose.q.z, pose.q.w);
  lastPose = { x, y, yaw };
  // Only extend the trail on real movement — /tf arrives at ~60 Hz and a
  // stationary robot would otherwise burn through MAX_TRAIL_POINTS standing
  // still, silently truncating the history that's actually interesting.
  const last = trail[trail.length - 1];
  if (!last || Math.hypot(x - last[0], y - last[1]) > 0.02) {
    trail.push([x, y]);
    if (trail.length > MAX_TRAIL_POINTS) trail.shift();
  }
}

new ROSLIB.Topic({
  ros,
  name: "/tf",
  messageType: "tf2_msgs/TFMessage",
  reconnect_on_close: true,
  // The sim publishes odom at ~60 Hz; the dashboard needs nowhere near that.
  throttle_rate: 50,
}).subscribe(onTfMessage);

new ROSLIB.Topic({
  ros,
  name: "/tf_static",
  messageType: "tf2_msgs/TFMessage",
  reconnect_on_close: true,
}).subscribe(onTfMessage);

function drawFrame() {
  requestAnimationFrame(drawFrame);
  // Nothing to draw to while the map is off-stage. The subscriptions stay
  // live, so the map/trail/plan are current the instant it's swapped back in.
  if (stageMode !== "map") return;
  if (!mapImageData) return;
  ctx.putImageData(mapImageData, 0, 0); // raw pixels; ignores alpha (base layer)
  if (!mapInfo) return;

  // Costmap overlay (own grid): blit its offscreen render into map-frame
  // world coords, scaled by the resolution ratio. drawImage composits the
  // per-pixel alpha over the map, unlike putImageData above.
  if (costmapCanvas && costmapInfo && costmapToggle.checked) {
    const cm = costmapInfo;
    // Offscreen top-left corner = (min x, max y) of the costmap in world.
    const [dx, dy] = worldToPixel(
      cm.origin.position.x,
      cm.origin.position.y + cm.height * cm.resolution
    );
    const dW = (cm.width * cm.resolution) / mapInfo.resolution;
    const dH = (cm.height * cm.resolution) / mapInfo.resolution;
    ctx.drawImage(costmapCanvas, dx, dy, dW, dH);
  }

  // Nav2 planned path (orange), distinct from the cyan odom trail below.
  if (plan.length > 1) {
    ctx.strokeStyle = "#ff9a3c";
    ctx.lineWidth = Math.max(1, 0.06 / mapInfo.resolution);
    ctx.beginPath();
    plan.forEach(([x, y], i) => {
      const [px, py] = worldToPixel(x, y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  if (trail.length > 1) {
    ctx.strokeStyle = "#2dd4ff";
    ctx.lineWidth = Math.max(1, 0.05 / mapInfo.resolution);
    ctx.beginPath();
    trail.forEach(([x, y], i) => {
      const [px, py] = worldToPixel(x, y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  // Nav goal marker (from click-to-nav or dispatch): hollow ring + heading tick.
  if (goal) {
    const [px, py] = worldToPixel(goal.x, goal.y);
    const r = Math.max(3, 0.25 / mapInfo.resolution);
    ctx.strokeStyle = "#ff9a3c";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(px, py, r, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(px, py);
    // Canvas Y is flipped vs world Y, so negate the sin term (as for the pose).
    ctx.lineTo(px + r * 1.8 * Math.cos(goal.yaw), py - r * 1.8 * Math.sin(goal.yaw));
    ctx.stroke();
  }

  if (lastPose) {
    const [px, py] = worldToPixel(lastPose.x, lastPose.y);
    const r = Math.max(2, 0.2 / mapInfo.resolution);
    ctx.fillStyle = "#2d5";
    ctx.beginPath();
    ctx.arc(px, py, r, 0, 2 * Math.PI);
    ctx.fill();
    // heading tick (canvas Y is flipped vs world Y, so negate the sin term)
    ctx.strokeStyle = "#2d5";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px + r * 2 * Math.cos(lastPose.yaw), py - r * 2 * Math.sin(lastPose.yaw));
    ctx.stroke();
  }
}
requestAnimationFrame(drawFrame);

// -- click-to-navigate ----------------------------------------------------
// rviz-style "2D Nav Goal": press on the map sets the goal position, drag sets
// the heading. Sends a NavigateToPose action goal straight to Nav2 via
// rosbridge's send_action_goal op (roslib's ROS2 Action class isn't in the
// vendored 1.4.1 build, but the low-level callOnConnection is). This preempts
// any goal the task_manager is currently running — it's a manual override.

const goalStatus = document.getElementById("goal-status");
let goalPressWorld = null; // [x, y] in the map frame, captured on mousedown

// A mouse event -> canvas backing-store pixel. The backing store is
// mapInfo.width x height cells but is displayed at the panel's CSS size, so
// scale the event's CSS offset by the backing/CSS ratio before inverting.
function eventToCanvasPixel(ev) {
  const rect = canvas.getBoundingClientRect();
  const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
  const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
  return [px, py];
}

function sendNavGoal(x, y, yaw) {
  ros.callOnConnection({
    op: "send_action_goal",
    action: `/${ROBOT_ID}/navigate_to_pose`,
    action_type: "nav2_msgs/action/NavigateToPose",
    args: {
      pose: {
        header: { frame_id: "map" },
        pose: {
          position: { x, y, z: 0 },
          // Yaw-only (planar) -> quaternion, same as task_manager._start_nav.
          orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
        },
      },
    },
  });
  goal = { x, y, yaw }; // instant marker; /plan refreshes it once Nav2 replies
  goalStatus.textContent =
    `nav goal sent: x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${yaw.toFixed(2)}`;
}

canvas.addEventListener("mousedown", (ev) => {
  if (!mapInfo) return; // no grid yet -> can't convert pixels to world
  ev.preventDefault();
  goalPressWorld = pixelToWorld(...eventToCanvasPixel(ev));
});

// mouseup on window (not just the canvas) so a heading drag that ends just
// outside the canvas still commits the goal — the position is the in-canvas
// press point, so it's always a valid map coordinate regardless.
window.addEventListener("mouseup", (ev) => {
  if (!mapInfo || !goalPressWorld) return;
  const [sx, sy] = goalPressWorld;
  goalPressWorld = null;
  const [ux, uy] = pixelToWorld(...eventToCanvasPixel(ev));
  const [dx, dy] = [ux - sx, uy - sy];
  // Near-zero drag = a plain click: keep heading 0 rather than amplify jitter.
  const yaw = Math.hypot(dx, dy) > 0.1 ? Math.atan2(dy, dx) : 0;
  sendNavGoal(sx, sy, yaw);
});

// -- 3D reconstruction (RTAB-Map cloud_map) -------------------------------
// A three.js point-cloud viewer for RTAB-Map's assembled 3D map. The cloud is
// a sensor_msgs/PointCloud2 in the `map` frame (z-up); rosbridge base64-encodes
// its byte payload, which we decode and unpack into a THREE.Points geometry.
// The whole thing is wrapped so a missing/broken three.js can never take down
// the rest of the dashboard.

const POINTCLOUD_TOPIC = `/${ROBOT_ID}/cloud_map`;
const POINTCLOUD_THROTTLE_MS = 1500; // cap rosbridge send rate (cloud can be MBs)
const MAX_RENDER_POINTS = 400000;    // decimate above this to keep the browser snappy

(function initRecon() {
  const container = document.getElementById("recon-canvas");
  const infoEl = document.getElementById("recon-info");
  if (typeof THREE === "undefined" || !container) {
    if (infoEl) infoEl.textContent = "three.js unavailable";
    return;
  }

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0c10);

  const camera = new THREE.PerspectiveCamera(60, 1, 0.05, 500);
  camera.up.set(0, 0, 1); // map frame is z-up (default three.js is y-up)
  camera.position.set(-4, -4, 4);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  container.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.1;
  // Without this, dragging far enough vertically swings the camera below the
  // cloud looking up at its underside — camera.up doesn't change, but the
  // scene reads as "upside down" to the user. Keep it from ever dropping
  // below the horizon; reset view (below) also recovers from this.
  controls.maxPolarAngle = Math.PI / 2 - 0.02;

  // A faint ground grid + origin axes for spatial reference (rotated so the
  // grid lies in the map's XY plane, since GridHelper is XZ by default).
  const grid = new THREE.GridHelper(40, 40, 0x2a2e35, 0x1c1f24);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(1.0));

  // The point cloud itself (geometry replaced on each new message).
  const cloudMat = new THREE.PointsMaterial({
    size: 0.03,
    sizeAttenuation: true,
    vertexColors: true,
  });
  let cloudPoints = null;

  // Robot pose marker: a small green sphere tracking lastPose (from TF).
  const robotMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.15, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0x22dd55 })
  );
  robotMarker.visible = false;
  scene.add(robotMarker);

  let didFitView = false;

  function sizeToContainer() {
    const w = container.clientWidth || 1;
    const h = container.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  sizeToContainer();
  // ResizeObserver rather than a window "resize" listener: the panel also
  // changes size without the window doing so (stage swaps, column reflow),
  // and this is robust to the container being zero-sized at init.
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(sizeToContainer).observe(container);
  } else {
    window.addEventListener("resize", sizeToContainer);
  }
  onStageResize = sizeToContainer; // let setStageMode() re-fit after a swap

  // Frame the camera on the cloud's bounding box (called on first cloud and
  // by the "reset view" button).
  function fitView() {
    if (!cloudPoints) return;
    cloudPoints.geometry.computeBoundingSphere();
    const bs = cloudPoints.geometry.boundingSphere;
    if (!bs || !isFinite(bs.radius) || bs.radius === 0) return;
    controls.target.copy(bs.center);
    const d = bs.radius * 2.2;
    camera.position.set(bs.center.x - d, bs.center.y - d, bs.center.z + d);
    camera.near = Math.max(0.05, bs.radius / 100);
    camera.far = bs.radius * 20;
    camera.updateProjectionMatrix();
    controls.update();
  }
  document.getElementById("recon-reset").addEventListener("click", () => {
    didFitView = false; // let the next fit re-run explicitly
    fitView();
  });

  // rosbridge encodes uint8[] as base64 (jazzy default). Decode to bytes.
  function b64ToBytes(b64) {
    const bin = atob(b64);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  function fieldOffsets(fields) {
    const o = {};
    for (const f of fields) o[f.name] = f.offset;
    return o;
  }

  function onCloud(msg) {
    // data may be a base64 string (default) or a plain byte array.
    const bytes = typeof msg.data === "string" ? b64ToBytes(msg.data)
                : msg.data instanceof Uint8Array ? msg.data
                : Uint8Array.from(msg.data);
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const off = fieldOffsets(msg.fields);
    if (off.x == null || off.y == null || off.z == null) {
      infoEl.textContent = "cloud missing x/y/z fields";
      return;
    }
    const step = msg.point_step;
    const total = msg.width * msg.height;
    // Decimate to keep the render (and allocation) bounded on huge clouds.
    const stride = Math.max(1, Math.ceil(total / MAX_RENDER_POINTS));
    const n = Math.ceil(total / stride);
    const positions = new Float32Array(n * 3);
    const colors = new Float32Array(n * 3);
    const hasRgb = off.rgb != null;
    let k = 0;
    for (let i = 0; i < total; i += stride) {
      const base = i * step;
      const x = dv.getFloat32(base + off.x, true);
      const y = dv.getFloat32(base + off.y, true);
      const z = dv.getFloat32(base + off.z, true);
      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
      positions[k * 3] = x;
      positions[k * 3 + 1] = y;
      positions[k * 3 + 2] = z;
      if (hasRgb) {
        // Packed rgb: the 4 bytes at off.rgb are 0x00RRGGBB (read as uint32).
        const v = dv.getUint32(base + off.rgb, true);
        colors[k * 3] = ((v >> 16) & 0xff) / 255;
        colors[k * 3 + 1] = ((v >> 8) & 0xff) / 255;
        colors[k * 3 + 2] = (v & 0xff) / 255;
      } else {
        colors[k * 3] = colors[k * 3 + 1] = colors[k * 3 + 2] = 0.8;
      }
      k++;
    }

    const geom = new THREE.BufferGeometry();
    // k may be < n if points were skipped — hand three.js only the filled part.
    geom.setAttribute("position", new THREE.BufferAttribute(positions.subarray(0, k * 3), 3));
    geom.setAttribute("color", new THREE.BufferAttribute(colors.subarray(0, k * 3), 3));

    if (cloudPoints) {
      cloudPoints.geometry.dispose();
      cloudPoints.geometry = geom;
    } else {
      cloudPoints = new THREE.Points(geom, cloudMat);
      scene.add(cloudPoints);
    }

    infoEl.textContent =
      `${k.toLocaleString()} pts` + (stride > 1 ? ` (of ${total.toLocaleString()}, 1/${stride})` : "");

    if (!didFitView) { fitView(); didFitView = true; }
  }

  new ROSLIB.Topic({
    ros,
    name: POINTCLOUD_TOPIC,
    messageType: "sensor_msgs/PointCloud2",
    throttle_rate: POINTCLOUD_THROTTLE_MS,
    queue_length: 1,
    reconnect_on_close: true,
  }).subscribe(onCloud);

  function animate() {
    requestAnimationFrame(animate);
    if (lastPose) {
      robotMarker.visible = true;
      robotMarker.position.set(lastPose.x, lastPose.y, 0.15);
    }
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
})();
