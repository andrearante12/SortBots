// SortBots dashboard. Vanilla JS, no build step. Talks to:
//   - rosbridge_websocket (ws://<host>:9090)   — pub/sub for dispatch/status/map/tf
//   - web_video_server    (http://<host>:8080) — MJPEG camera stream
//   - webui/serve.py      (this page's own origin) — /api/waypoints
// All three are brought up together by launch/sortbots_webui.launch.py.

// Which robot this whole page talks to. Read once, synchronously, from
// ?robot=<id> — every subscription below is constructed immediately at
// script-load time against this single ROBOT_ID, so it can't be a value
// that changes later without re-creating ~15 ROSLIB.Topic objects. Switching
// robots (see the header <select id="robot-select">) therefore reloads the
// page with a new query param rather than hot-swapping live — instant and
// correct, just not seamless. Absent the param, defaults to "robot_0" —
// identical to this dashboard's single-robot behavior before robots.yaml
// existed, so a plain http://host:8081/ still works exactly as it always did.
const ROBOT_ID = new URLSearchParams(window.location.search).get("robot") || "robot_0";
const ROSBRIDGE_HOST = window.location.hostname || "localhost";
const ROSBRIDGE_PORT = 9090;
const VIDEO_PORT = 8080;
const MAX_TRAIL_POINTS = 3000;

document.getElementById("page-title").textContent = `SortBots — ${ROBOT_ID}`;
document.title = `SortBots — ${ROBOT_ID}`;

// Populate the robot switcher from configs/robots.yaml (via serve.py). Kept
// separate from ROBOT_ID itself — this only decides what the DROPDOWN
// offers; it never gates which robot the page actually subscribes to.
fetch("/api/robots")
  .then((r) => r.json())
  .then((cfg) => {
    const sel = document.getElementById("robot-select");
    const ids = Array.isArray(cfg.robots) && cfg.robots.length ? cfg.robots : [ROBOT_ID];
    fleetRobotIds = ids;
    for (const id of ids) {
      const o = document.createElement("option");
      o.value = id;
      o.textContent = id;
      sel.appendChild(o);
    }
    sel.value = ROBOT_ID;
    // Every OTHER robot in the roster gets drawn on this page's own map
    // canvas alongside ROBOT_ID (see the "fleet: peer robots" section below)
    // — setupPeer no-ops for ROBOT_ID itself. Function declaration, so it's
    // hoisted and callable here even though it's defined later in the file.
    for (const id of ids) setupPeer(id);
    refreshMapStageLabel();
    // If the current ROBOT_ID isn't in the known list (e.g. a hand-typed
    // ?robot=foo), sel.value silently fails to match any option and the
    // dropdown shows the first entry instead of the truth — add it so what's
    // selected always matches what the page is actually showing.
    if (sel.value !== ROBOT_ID) {
      const o = document.createElement("option");
      o.value = ROBOT_ID;
      o.textContent = `${ROBOT_ID} (unlisted)`;
      sel.insertBefore(o, sel.firstChild);
      sel.value = ROBOT_ID;
    }
    sel.addEventListener("change", () => {
      const url = new URL(window.location.href);
      url.searchParams.set("robot", sel.value);
      window.location.href = url.toString();
    });
  })
  .catch((e) => console.error("failed to load /api/robots", e));

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
//
// The offsetParent check covers the whole live view being hidden (the
// scenarios tab in webui/scenarios.js sets [hidden] on #stage-panel): the
// stage classes are still on the images then, so the class test alone would
// keep both cameras polling web_video_server at 2.5fps behind a hidden panel.
// display:none is exactly the case offsetParent reports as null.
function isFeedVisible(img) {
  if (!img.classList.contains("stage-main") && !img.classList.contains("stage-pip")) return false;
  return img.offsetParent !== null;
}

// Chase cam is optional per robot (explore_fleet: chase_cam_robots: 1 → only
// robot_0). Without this flag the map-mode PiP stays a broken-image icon that
// reads as "dashboard is out of date / web_video_server is dead" even when
// /map and recon are live. Probe via the same Image() path as the feed —
// fetch() to :8080 from :8081 is cross-origin and useless here.
let chaseAvailable = true; // flipped false after a failed probe with no prior frame
let chaseEverOk = false;

function markChaseMissing() {
  if (!chaseAvailable) return;
  chaseAvailable = false;
  const chaseImg = document.getElementById("chase-stream");
  if (chaseImg) {
    chaseImg.removeAttribute("src");
    chaseImg.alt = `no chase cam on ${ROBOT_ID} (fleet keeps chase on robot_0 only)`;
  }
  // Drop the broken PiP / full-stage chase view. Map is the useful default
  // when verifying a fleet run; camera toggle still reaches the head cam.
  setStageMode(stageMode === "chase" ? "map" : stageMode);
}

function wireCameraStream(elId, topic) {
  const img = document.getElementById(elId);
  const isChase = elId === "chase-stream";
  function tick() {
    // Stop hammering web_video_server once we know this robot has no chase
    // product — otherwise every 1.5s error backoff still opens a subscribe
    // that can never succeed.
    if (isChase && !chaseAvailable) return;
    if (!isFeedVisible(img)) {
      setTimeout(tick, CAMERA_POLL_MS);
      return;
    }
    const probe = new Image();
    // Swap the visible src only once the new frame is fully loaded, so the
    // panel never flashes blank between polls.
    probe.onload = () => {
      if (isChase) {
        chaseEverOk = true;
        chaseAvailable = true;
      }
      img.src = probe.src;
      setTimeout(tick, CAMERA_POLL_MS);
    };
    probe.onerror = () => {
      // One miss with no prior frame is enough: missing chase topics fail
      // immediately (curl exit 52 / HTTP empty), while a wedged video server
      // that previously worked keeps chaseEverOk true and just backs off.
      if (isChase && !chaseEverOk) markChaseMissing();
      else setTimeout(tick, CAMERA_ERROR_BACKOFF_MS);
    };
    // qos_profile=sensor_data makes web_video_server subscribe BEST_EFFORT
    // instead of its RELIABLE default. This is not cosmetic: Isaac's image
    // writer publishes RELIABLE, and a RELIABLE subscriber that wedges (which
    // this web_video_server recurrently does — see the header comment) can
    // back-pressure the publisher itself. Measured live 2026-08-01: with a
    // RELIABLE image subscriber attached, Isaac's rgb/depth topics died at
    // sim-t~100-140s on every run; Isaac alone ran indefinitely. A
    // best-effort subscriber can never block the writer, so a wedged video
    // server costs us the camera panes (until the watchdog restarts it), not
    // the whole SLAM pipeline.
    probe.src =
      `http://${ROSBRIDGE_HOST}:${VIDEO_PORT}/snapshot?topic=/${ROBOT_ID}/${topic}` +
      `&qos_profile=sensor_data&_=${Date.now()}`;
  }
  tick();
}

// -- stage mode -----------------------------------------------------------
// The stage box shows exactly one main view, plus at most one picture-in-
// picture inset:
//
//   "chase" -> chase cam large, no PiP                 (default)
//   "head"  -> head cam large,  no PiP                 (reachable only if
//              something still promotes it; camera toggle restores chase)
//   "map"   -> map large,       chase cam as PiP
//
// The header's camera/map buttons switch between the map and whichever
// camera mode was last active.

const STAGE_LABELS = {
  chase: "3rd person (chase cam)",
  head: "Head camera",
  // Updated live once /api/robots + /map arrive — see refreshMapStageLabel().
  map: "Map, trail & nav",
};
let fleetRobotIds = [ROBOT_ID]; // filled from /api/robots; drives the map label
let mapMetaLabel = ""; // e.g. "219×326 · fused" from the /map handler

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
  // No chase product on this robot → never park the stage on a broken <img>.
  // Camera toggle uses head instead; map mode drops the PiP entirely.
  if (!chaseAvailable && mode === "chase") mode = "head";

  stageMode = mode;
  if (mode !== "map") lastCameraMode = mode;

  const main = { chase: chaseEl, head: headEl, map: mapViewEl }[mode];
  // Head-cam PiP intentionally removed: Isaac still renders that camera
  // regardless (RTAB-Map's actual SLAM input, not just a view — it can't be
  // turned off), so hiding it here doesn't reduce server-side load, but it
  // stops the browser from polling/decoding a stream nobody is looking at,
  // and — more importantly — there's no more PiP to click-to-promote, so
  // lastCameraMode can never become "head" and the camera toggle always
  // lands back on chase (or head, when chase is missing). In map mode the
  // chase cam still keeps a corner so you never lose sight of the robot
  // while picking a goal — unless this robot has no chase cam at all.
  const pip = mode === "map" && chaseAvailable ? chaseEl : null;

  for (const el of [chaseEl, headEl, mapViewEl]) {
    el.classList.toggle("stage-main", el === main);
    el.classList.toggle("stage-pip", el === pip);
  }

  if (mode === "map") refreshMapStageLabel();
  else if (mode === "head" && !chaseAvailable) {
    stageLabel.textContent = `Head camera · no chase on ${ROBOT_ID}`;
  } else {
    stageLabel.textContent = STAGE_LABELS[mode];
  }
  pipHint.style.display = pip ? "" : "none";
  // Head pan/tilt has no meaning while looking at the map.
  aimOverlay.style.display = mode === "map" ? "none" : "";

  for (const btn of document.querySelectorAll("#stage-mode button")) {
    btn.classList.toggle("active", (btn.dataset.stage === "map") === (mode === "map"));
  }
  onStageResize();
}

// Map stage label: make it obvious the canvas is nodes/map_merge.py's fused
// /map (every robot's SLAM grid resampled into one world frame), not the
// active robot's private /<id>/map. Without this, a fleet run that looks
// "fine" can still be misread as single-robot when only one marker is
// obvious and the other is off-screen in the padding.
function refreshMapStageLabel() {
  if (stageMode !== "map") return;
  const n = fleetRobotIds.length;
  const base = n > 1
    ? `Fused fleet map (${n} robots)`
    : "Map, trail & nav";
  stageLabel.textContent = mapMetaLabel ? `${base} · ${mapMetaLabel}` : base;
}

// Clicking the chase cam while it's the map-mode inset promotes it to main.
// headEl stays in the listener for back-compat if something re-adds head PiP.
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

// Start snapshot polls only after the stage classes are coherent — otherwise
// markChaseMissing can race setStageMode's first paint, and the HTML default
// (head as .stage-pip) would briefly poll a feed we intentionally hide.
wireCameraStream("camera-stream", "camera/rgb");
wireCameraStream("chase-stream", "camera/chase/rgb");

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

// -- autonomous exploration (nodes/explorer.py) ---------------------------
// explore_cmd/explore_status follow the same "String carrying JSON" pattern
// as dispatch_task/task_status. There's no reliable "is explorer.py even
// running" signal other than message absence, so the status line falls back
// to "not running" if nothing has arrived in a while — the same staleness
// idea as the SLAM badge above.

const exploreCmdTopic = new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/explore_cmd`,
  messageType: "std_msgs/String",
  reconnect_on_close: true,
});

document.getElementById("explore-start").addEventListener("click", () => {
  exploreCmdTopic.publish(new ROSLIB.Message({ data: "start" }));
});
document.getElementById("explore-stop").addEventListener("click", () => {
  exploreCmdTopic.publish(new ROSLIB.Message({ data: "stop" }));
});

const exploreStatusEl = document.getElementById("explore-status");
let lastExploreStatus = null;
let lastExploreStatusAt = 0;
let exploreMsgUntil = 0; // suppresses the regular readout for a transient message (e.g. save confirmation)

new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/explore_status`,
  messageType: "std_msgs/String",
  reconnect_on_close: true,
}).subscribe((msg) => {
  try {
    lastExploreStatus = JSON.parse(msg.data);
  } catch (e) {
    return;
  }
  lastExploreStatusAt = performance.now();
});

function updateExploreStatus() {
  if (performance.now() < exploreMsgUntil) return;
  const now = performance.now();
  if (!lastExploreStatus || now - lastExploreStatusAt > 6000) {
    exploreStatusEl.textContent = "explorer: not running";
    return;
  }
  const s = lastExploreStatus;
  // Prefer coverage_pct: free floor mapped as a fraction of a reference map
  // (configs/explorer.yaml's reference_free_area_m2). explored_pct is the
  // fallback and is NOT coverage — it divides by the current grid's own
  // extent, which grows as RTAB-Map explores, so it can fall while the robot
  // is making progress and reads ~44% on a finished map. Labelled distinctly
  // so the two are never mistaken for each other in a demo.
  const cov =
    s.coverage_pct != null
      ? ` · covered ${s.coverage_pct}%${s.free_area_m2 != null ? ` (${s.free_area_m2} m²)` : ""}`
      : ` · mapped ${s.explored_pct != null ? `${s.explored_pct}%` : "?"}`;
  const goal = s.current_goal
    ? ` -> (${s.current_goal.x.toFixed(1)}, ${s.current_goal.y.toFixed(1)})`
    : "";
  // The explorer is the authority on whether a hint is still live (it expires
  // them), so mirror its view rather than trusting our own optimistic marker.
  steerHint = s.steer_hint || null;
  steerQueue = s.steer_queue || (steerHint ? [steerHint] : []);
  blacklistPoints = s.blacklist_points || [];
  const queued = Math.max(0, steerQueue.length - 1);
  const steer = s.steer_hint
    ? ` · steering (${s.steer_hint.x.toFixed(1)}, ${s.steer_hint.y.toFixed(1)})` +
      (queued ? ` +${queued} queued` : "")
    : "";
  exploreStatusEl.textContent =
    `explorer: ${s.state}${goal}${cov} · blacklisted ${s.blacklisted}${steer}`;
}
setInterval(updateExploreStatus, 500);

// RTAB-Map's database is written continuously as it maps (see docs/running.md
// "Map lifecycle") — this button doesn't create persistence, it triggers
// rtabmap's own `backup` service (std_srvs/Empty) for a labeled, timestamped
// snapshot of the working DB you can point --map at later.
const rtabmapBackup = new ROSLIB.Service({
  ros,
  name: `/${ROBOT_ID}/rtabmap/backup`,
  serviceType: "std_srvs/srv/Empty",
});
document.getElementById("explore-save").addEventListener("click", () => {
  exploreMsgUntil = performance.now() + 3000;
  exploreStatusEl.textContent = "saving…";
  rtabmapBackup.callService(
    new ROSLIB.ServiceRequest({}),
    () => {
      exploreMsgUntil = performance.now() + 3000;
      exploreStatusEl.textContent = "map backed up ✓ (see ~/.ros/*.back)";
    },
    (err) => {
      exploreMsgUntil = performance.now() + 3000;
      exploreStatusEl.textContent = "map backup failed — see console";
      console.error("rtabmap backup failed", err);
    }
  );
});

// Save into the named map library (maps/, see maps/README.md) — a whole entry:
// the fused /map grid, this robot's pose graph, and a manifest tying them
// together, so `library_localize` can load it back on command.
//
// Plain same-origin fetch, NOT rosbridge: this shells out to
// scripts/maps.sh, which needs system ROS 2 sourced, and webui/session.py is
// the only place that knows how to build that shell. rosbridge can't run a
// script; serve.py --control can.
//
// The two artifacts have opposite requirements — the grid needs the stack UP,
// a safe pose-graph copy needs it DOWN — so a mid-run save can legitimately
// come back "pending". That is reported, not treated as a failure; a second
// save after teardown (or `sim_ctl.sh stop --save-map NAME`) completes it.
document.getElementById("explore-save-lib").addEventListener("click", async () => {
  const name = (document.getElementById("map-name").value || "").trim();
  exploreMsgUntil = performance.now() + 3000;
  if (!name) {
    exploreStatusEl.textContent = "name the map first (a-z 0-9 - _)";
    return;
  }
  exploreStatusEl.textContent = `saving ${name}…`;
  try {
    const res = await fetch("/api/map/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, robot_id: ROBOT_ID }),
    });
    const body = await res.json().catch(() => ({}));
    exploreMsgUntil = performance.now() + 3000;
    if (res.status === 503) {
      exploreStatusEl.textContent = "console not running — scripts/maps.sh save";
    } else if (!res.ok) {
      exploreStatusEl.textContent = `save failed: ${body.error || res.statusText}`;
      console.error("map save failed", body);
    } else if (body.db_state !== "complete") {
      exploreStatusEl.textContent = "grid saved ✓ — pose graph pending until stop";
    } else {
      exploreStatusEl.textContent = `saved to maps/${name} ✓`;
    }
  } catch (err) {
    exploreMsgUntil = performance.now() + 3000;
    exploreStatusEl.textContent = "save failed — see console";
    console.error("map save failed", err);
  }
});

// -- map + robot trail ------------------------------------------------

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
let mapInfo = null;
const trail = [];
let lastPose = null; // {x, y, yaw}
let plan = []; // Nav2 planned path, world [x, y] points
let goal = null; // {x, y, yaw} — current nav target marker
let steerHint = null; // {x, y} — active steering hint, mirrored from explore_status
let steerQueue = []; // [{x, y}] head-first; entry 0 is the one being worked
let blacklistPoints = []; // [{x, y, r, strikes, permanent}] from explore_status
const blacklistToggle = document.getElementById("blacklist-toggle");
let showBlacklist = blacklistToggle.checked;
blacklistToggle.addEventListener("change", () => {
  showBlacklist = blacklistToggle.checked;
});
let costmapCanvas = null; // offscreen render of the global costmap (own grid)

// -- fixed view: canvas-pixels-per-metre + a world-space origin -----------
// The canvas backing store is a FIXED size (see MAP_CANVAS_PX below), NOT
// the merged /map grid's own cell dimensions. Earlier this canvas literally
// WAS the grid — `canvas.width/height` and the CSS aspect-ratio were set
// straight from the incoming OccupancyGrid's width/height, so every /map
// message resized the element. That was tolerable for one slow-growing
// single-robot grid; with a fleet fusing two robots' progress into one grid
// (nodes/map_merge.py), the extent grows on nearly every message, and the
// canvas — and everything drawn in it — visibly resized every second or so.
// Read as the whole map continuously zooming in and out.
//
// Instead: the raw grid is rasterized into an OFFSCREEN canvas at 1 cell = 1
// pixel (unchanged), then blitted into this FIXED on-screen canvas via
// drawImage at whatever scale the current `view` says — same technique the
// costmap overlay already used for its own, differently-scaled grid. `view`
// only gets recomputed when the incoming grid no longer fits it (grow-only,
// matching map_merge's own extent — see ensureViewFits), not on every
// message, so the on-screen scale is stable for the length of an
// exploration run instead of drifting continuously.
const MAP_CANVAS_PX = 900;
canvas.width = MAP_CANVAS_PX;
canvas.height = MAP_CANVAS_PX;
canvas.style.aspectRatio = "1 / 1";

let view = null; // {scale (canvas px per metre), originX, originY (world metres, canvas (0, canvasPx))}
let mapOffscreen = null; // 1 cell = 1 px raster of the current /map grid
let mapOffscreenInfo = null; // the info that produced mapOffscreen

function ensureViewFits(info) {
  const minX = info.origin.position.x;
  const minY = info.origin.position.y;
  const maxX = minX + info.width * info.resolution;
  const maxY = minY + info.height * info.resolution;

  if (view) {
    const viewSpan = MAP_CANVAS_PX / view.scale;
    const fits =
      minX >= view.originX && maxX <= view.originX + viewSpan &&
      minY >= view.originY && maxY <= view.originY + viewSpan;
    // Retighten only when the view has become substantially bigger than the
    // data actually needs (not on every minor fluctuation — THAT'S what the
    // original continuous-zoom bug was). Every earlier (re)fit baked in its
    // own PAD_M around whatever the extent was AT THAT TIME and then never
    // shrank, so a map that grew through several fits ended up with padding
    // compounding on padding — the fixed view stayed stable, just at a scale
    // that made the explored area look small in a lot of empty margin.
    const dataSpan = Math.max(maxX - minX, maxY - minY, 1);
    const oversized = fits && viewSpan > dataSpan * 1.4;
    if (fits && !oversized) return;
  }

  // (Re)fit with padding, so exploring right up to the current edge doesn't
  // force another refit on the very next message.
  const PAD_M = 1.5;
  const spanX = (maxX - minX) + PAD_M * 2;
  const spanY = (maxY - minY) + PAD_M * 2;
  const scale = MAP_CANVAS_PX / Math.max(spanX, spanY, 1);
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const halfSpanM = MAP_CANVAS_PX / scale / 2;
  view = { scale, originX: centerX - halfSpanM, originY: centerY - halfSpanM };
}

// Pixel<->world, against the fixed `view` — stable across a run regardless
// of how the underlying grid grows. worldToPixel is also used to place the
// costmap/plan/goal/peer overlays, so everything shares one frame.
function worldToPixel(x, y) {
  const px = (x - view.originX) * view.scale;
  const py = MAP_CANVAS_PX - (y - view.originY) * view.scale;
  return [px, py];
}

// Inverse of worldToPixel: canvas pixel -> world (map frame).
function pixelToWorld(px, py) {
  const x = px / view.scale + view.originX;
  const y = (MAP_CANVAS_PX - py) / view.scale + view.originY;
  return [x, y];
}

// Planar yaw from a quaternion's z/w (roll/pitch ~0 for the base and 2D goals).
function yawFromQuat(qz, qw) {
  return Math.atan2(2 * qw * qz, 1 - 2 * qz * qz);
}

// /map (not /${ROBOT_ID}/map): nodes/map_merge.py's fused, world-anchored
// grid, in the shared "map" frame — the same frame lookupPose() below walks
// TF up to. Each robot's OWN SLAM grid is published at /<rid>/map but lives
// in that robot's <rid>/map frame (a static, per-robot-offset anchor from
// its spawn pose — see launch/sortbots_bringup.launch.py), so drawing THAT
// grid's pixels against a TF-derived world-frame pose would misalign the
// map image against the robot marker by exactly the spawn offset. /map is
// also what makes one canvas showing every robot's fused progress possible.
new ROSLIB.Topic({
  ros,
  name: "/map",
  messageType: "nav_msgs/OccupancyGrid",
  reconnect_on_close: true,
}).subscribe((msg) => {
  mapInfo = msg.info;
  ensureViewFits(msg.info);
  const w = msg.info.width;
  const h = msg.info.height;
  // Rasterize into the OFFSCREEN canvas at 1 cell = 1 px — the visible
  // #map-canvas stays a fixed MAP_CANVAS_PX square (see ensureViewFits'
  // comment above) and is only ever drawn into via drawImage in drawFrame().
  const off = mapOffscreen || document.createElement("canvas");
  off.width = w;
  off.height = h;
  const octx = off.getContext("2d");
  const imgData = octx.createImageData(w, h);
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
  octx.putImageData(imgData, 0, 0);
  mapOffscreen = off;
  mapOffscreenInfo = msg.info;
  // Surface merge health in the stage label — growing WxH is the quickest
  // live check that map_merge is actually fusing (a single-robot /map stays
  // roughly square around that robot; a fleet /map spans both spawn anchors).
  mapMetaLabel = `${w}×${h}`;
  refreshMapStageLabel();
});

// -- Nav2 global costmap overlay (toggleable) -----------------------------
// Its own OccupancyGrid with its own origin/resolution — NOT necessarily the
// same grid as the SLAM map, so it's rendered to a separate offscreen canvas
// (in its own cell space) and blitted in world coords during drawFrame().
// 0..100 cost -> translucent yellow->red tint; 0 (known free) stays fully
// clear. Unknown (-1) gets a faint violet tint of its own — nav2_params.yaml
// sets track_unknown_space: true, so "the planner hasn't seen this yet" is a
// real, distinct state from "the planner has seen this and it's clear",
// which matters a lot once autonomous exploration is driving: a frontier is
// exactly the boundary between these two. They used to render identically
// (both `v <= 0` -> fully transparent), which silently erased that boundary.
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
      if (v < 0) {
        img.data[i] = 140;
        img.data[i + 1] = 110;
        img.data[i + 2] = 220;
        img.data[i + 3] = 26; // faint — a hint, not a competing layer
        continue;
      }
      if (v === 0) {
        img.data[i + 3] = 0; // known free -> transparent
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

// -- frontier markers (nodes/explorer.py, autonomous exploration) ---------
// visualization_msgs/MarkerArray of SPHERE markers, one per frontier cluster
// candidate, already colored/sized by the explorer (the best-scored one is
// bigger and brighter orange — see explorer.py's _publish_frontier_markers).
// Rebuilt wholesale from each message rather than diffed against the
// previous one: the array always carries a DELETEALL first (action 2)
// followed by the current ADD set (action 0), so replacing frontierMarkers
// outright already reflects that without any special-casing here.
let frontierMarkers = []; // [{x, y, r, color}]

new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/frontiers`,
  messageType: "visualization_msgs/MarkerArray",
  reconnect_on_close: true,
}).subscribe((msg) => {
  frontierMarkers = msg.markers
    .filter((m) => m.action === 0) // ADD only; skip the leading DELETEALL
    .map((m) => ({
      x: m.pose.position.x,
      y: m.pose.position.y,
      r: (m.scale.x || 0.14) / 2,
      color: `rgba(${Math.round((m.color.r || 0) * 255)}, ` +
             `${Math.round((m.color.g || 0) * 255)}, ` +
             `${Math.round((m.color.b || 0) * 255)}, ${m.color.a ?? 1})`,
    }));
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
slamBadge.title = `RTAB-Map SLAM status (from /${ROBOT_ID}/info)`;
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

// -- fleet: other robots' pose/trail/plan/frontiers -----------------------
// Camera feeds, drive/aim pads, dispatch, and click-to-nav/steer all stay
// scoped to ROBOT_ID (the "active" robot this page load is for — see the
// header comment on ROBOT_ID for why switching robots reloads the page
// rather than hot-swapping). The fused 2D map and the 3D recon pose
// markers both need the rest of the fleet: with /map world-anchored (see
// the /map subscription above) every robot's pose/trail/plan/frontiers
// already share one frame, so drawing them together is just tracking each
// peer's own small slice of state and giving it its own color.
// Peers are lighter-weight than ROBOT_ID's own state: no goal marker (a
// peer's plan already ends at its goal), no costmap (Nav2's costmap is
// per-robot-namespaced and only useful for the one you're driving), no
// click-to-nav/steer targeting.
const PEER_COLORS = [
  { pose: "#ffb020", trail: "rgba(255,176,32,0.85)", plan: "#c792ea" },
  { pose: "#ff5da2", trail: "rgba(255,93,162,0.85)", plan: "#7ee787" },
];
const peerRobots = new Map(); // robot_id -> {trail, lastPose, plan, frontierMarkers, steerQueue, blacklistPoints, color}
// Peer poses come from /fleet/status (self-reported mesh radio), matching
// autonomy — not silent TF eavesdropping on map -> <peer>/base_link.
const FLEET_STATUS_TTL_MS = 1500;

function setupPeer(id) {
  if (id === ROBOT_ID || peerRobots.has(id)) return;
  const color = PEER_COLORS[peerRobots.size % PEER_COLORS.length];
  const state = {
    id, trail: [], lastPose: null, plan: [],
    frontierMarkers: [], steerQueue: [], blacklistPoints: [],
    color, statusAt: 0,
  };
  peerRobots.set(id, state);

  new ROSLIB.Topic({
    ros, name: `/${id}/plan`, messageType: "nav_msgs/Path", reconnect_on_close: true,
  }).subscribe((msg) => {
    state.plan = msg.poses.map((ps) => [ps.pose.position.x, ps.pose.position.y]);
  });

  new ROSLIB.Topic({
    ros, name: `/${id}/frontiers`, messageType: "visualization_msgs/MarkerArray", reconnect_on_close: true,
  }).subscribe((msg) => {
    state.frontierMarkers = msg.markers
      .filter((m) => m.action === 0)
      .map((m) => ({
        x: m.pose.position.x,
        y: m.pose.position.y,
        r: (m.scale.x || 0.14) / 2,
        color: `rgba(${Math.round((m.color.r || 0) * 255)}, ` +
               `${Math.round((m.color.g || 0) * 255)}, ` +
               `${Math.round((m.color.b || 0) * 255)}, ${m.color.a ?? 1})`,
      }));
  });

  new ROSLIB.Topic({
    ros, name: `/${id}/explore_status`, messageType: "std_msgs/String", reconnect_on_close: true,
  }).subscribe((msg) => {
    let s;
    try {
      s = JSON.parse(msg.data);
    } catch (e) {
      return;
    }
    state.steerQueue = s.steer_queue || (s.steer_hint ? [s.steer_hint] : []);
    state.blacklistPoints = s.blacklist_points || [];
  });
}

// One shared subscription — mesh broadcast, all peers.
new ROSLIB.Topic({
  ros,
  name: "/fleet/status",
  messageType: "std_msgs/String",
  reconnect_on_close: true,
}).subscribe((msg) => {
  let s;
  try {
    s = JSON.parse(msg.data);
  } catch (e) {
    return;
  }
  const id = s.robot_id;
  if (!id || id === ROBOT_ID) return;
  // Peers may appear on radio before /api/robots finishes setupPeer.
  if (!peerRobots.has(id)) setupPeer(id);
  const state = peerRobots.get(id);
  if (!state) return;
  const pose = s.pose || {};
  const x = pose.x, y = pose.y;
  if (typeof x !== "number" || typeof y !== "number") return;
  const yaw = typeof pose.yaw === "number" ? pose.yaw : 0;
  state.lastPose = { x, y, yaw };
  state.statusAt = Date.now();
  const last = state.trail[state.trail.length - 1];
  if (!last || Math.hypot(x - last[0], y - last[1]) > 0.02) {
    state.trail.push([x, y]);
    if (state.trail.length > MAX_TRAIL_POINTS) state.trail.shift();
  }
});

// -- session-boundary reset ------------------------------------------------
//
// The console (rosbridge + serve.py) outlives a sim SESSION by design — only
// Isaac/Nav2/explorer restart on `scripts/sim_ctl.sh stop`/`start`, not the
// WebSocket this page holds open to rosbridge. So nothing here is ever told
// "that was the last session, this is a new one": a restarted session's plan,
// goal, and peer trails just keep whatever they last held until a fresh
// publish overwrites them — which may be a while if the new session hasn't
// picked a Nav2 goal yet. Seen live 2026-08-27 switching explore_fresh (1
// robot) straight to explore_fleet (2 robots) without a reload: the old
// robot's final plan/goal stayed drawn, trailing off toward a position from
// the earlier run.
//
// Poll the same /api/session serve.py --control exposes to scenarios.js, and
// clear everything that only makes sense within one session when its
// session_id changes. 503 (serve.py running WITHOUT --control — the plain
// mode launch/sortbots_webui.launch.py starts) means there's no session
// boundary to detect at all here; leave everything alone rather than treat
// it as an error.
let knownSessionId; // undefined until the first successful read — the first
                    // read must only record a baseline, never clear anything.

function resetSessionOverlayState() {
  plan = [];
  goal = null;
  steerHint = null;
  steerQueue = [];
  blacklistPoints = [];
  trail.length = 0;
  // Client-side counters fed by /<id>/info — RTAB-Map's own loop-closure
  // count resets with the node, but this tally is independent of it and
  // would otherwise keep growing across a session it no longer describes.
  loopClosureCount = 0;
  lastInfoAt = 0;
  loopFlashUntil = 0;
  for (const state of peerRobots.values()) {
    state.trail.length = 0;
    state.plan = [];
    state.frontierMarkers = [];
    state.steerQueue = [];
    state.blacklistPoints = [];
    state.lastPose = null;
  }
}

async function pollSessionBoundary() {
  try {
    const res = await fetch("/api/session", { cache: "no-store" });
    if (!res.ok) return; // 503 without --control, or a transient error either way
    const data = await res.json();
    if (knownSessionId !== undefined && data.session_id !== knownSessionId) {
      resetSessionOverlayState();
    }
    knownSessionId = data.session_id;
  } catch {
    /* offline poll — try again next tick, same tolerance as everything else here */
  }
}
pollSessionBoundary();
setInterval(pollSessionBoundary, 4000);

function onTfMessage(msg) {
  for (const tr of msg.transforms || []) {
    tfTree.set(normFrame(tr.child_frame_id), {
      parent: normFrame(tr.header.frame_id),
      t: tr.transform.translation,
      q: tr.transform.rotation,
    });
  }
  const pose = lookupPose(TF_TARGET_FRAME);
  if (pose) {
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
  // Drop stale peer status so markers vanish if radio goes quiet.
  const now = Date.now();
  for (const state of peerRobots.values()) {
    if (state.lastPose && state.statusAt && now - state.statusAt > FLEET_STATUS_TTL_MS) {
      state.lastPose = null;
    }
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

// nodes/map_merge.py's per-robot map -> <rid>/map anchors — deliberately a
// SEPARATE subscription from /tf, not throttled: those anchors are only
// republished once per second (map_merge's own publish_period_s), and
// sharing /tf's 50ms throttle window with Isaac's ~60Hz-per-robot odometry
// traffic meant an anchor essentially never won that window, so a robot's
// marker could sit permanently missing depending on nothing but luck.
// TRANSIENT_LOCAL on that topic (not requestable from this vendored
// roslib — it has no QoS override) still means whichever anchors already
// exist land immediately on subscribe, same guarantee /map already gets.
new ROSLIB.Topic({
  ros,
  name: "/map_anchors",
  messageType: "tf2_msgs/TFMessage",
  reconnect_on_close: true,
}).subscribe(onTfMessage);

function drawFrame() {
  requestAnimationFrame(drawFrame);
  // Nothing to draw to while the map is off-stage. The subscriptions stay
  // live, so the map/trail/plan are current the instant it's swapped back in.
  if (stageMode !== "map") return;
  if (!mapOffscreen || !mapInfo || !view) return;
  ctx.clearRect(0, 0, MAP_CANVAS_PX, MAP_CANVAS_PX);
  // Blit the raw grid (rasterized 1 cell = 1 px into mapOffscreen by the /map
  // handler above) into the fixed on-screen canvas at the current view scale
  // — same drawImage-into-world-coords technique the costmap overlay below
  // already used for its own, differently-scaled grid.
  {
    const info = mapOffscreenInfo;
    const [dx, dy] = worldToPixel(info.origin.position.x, info.origin.position.y + info.height * info.resolution);
    const dW = info.width * info.resolution * view.scale;
    const dH = info.height * info.resolution * view.scale;
    ctx.imageSmoothingEnabled = false; // keep cells crisp, matching the old 1:1 putImageData look
    ctx.drawImage(mapOffscreen, dx, dy, dW, dH);
  }

  // Costmap overlay (own grid): blit its offscreen render into map-frame
  // world coords at the current view scale — drawImage composits its
  // per-pixel alpha over the map, unlike the base layer's opaque blit above.
  if (costmapCanvas && costmapInfo && costmapToggle.checked) {
    const cm = costmapInfo;
    // Offscreen top-left corner = (min x, max y) of the costmap in world.
    const [dx, dy] = worldToPixel(
      cm.origin.position.x,
      cm.origin.position.y + cm.height * cm.resolution
    );
    const dW = (cm.width * cm.resolution)  * view.scale;
    const dH = (cm.height * cm.resolution)  * view.scale;
    ctx.drawImage(costmapCanvas, dx, dy, dW, dH);
  }

  // Frontier candidates (autonomous exploration). Drawn under the trail/goal/
  // robot marker so those stay legible even with many frontiers on screen.
  for (const fm of frontierMarkers) {
    const [px, py] = worldToPixel(fm.x, fm.y);
    const r = Math.max(2, fm.r  * view.scale);
    ctx.fillStyle = fm.color;
    ctx.beginPath();
    ctx.arc(px, py, r, 0, 2 * Math.PI);
    ctx.fill();
  }

  // Nav2 planned path (orange), distinct from the cyan odom trail below.
  if (plan.length > 1) {
    ctx.strokeStyle = "#ff9a3c";
    ctx.lineWidth = Math.max(1, 0.06  * view.scale);
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
    ctx.lineWidth = Math.max(1, 0.05  * view.scale);
    ctx.beginPath();
    trail.forEach(([x, y], i) => {
      const [px, py] = worldToPixel(x, y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  // Blacklisted areas: where the explorer has given up after failed goals.
  // Off by default — it's a diagnostic overlay, not something you want
  // cluttering the map during a demo. Drawn at each entry's ACTUAL
  // suppression radius (which grows per strike), so what you see is exactly
  // the area frontier selection is excluding. Solid = permanent.
  if (showBlacklist) {
    for (const b of blacklistPoints) {
      const [px, py] = worldToPixel(b.x, b.y);
      const r = Math.max(3, b.r  * view.scale);
      ctx.save();
      ctx.strokeStyle = b.permanent ? "#ff4d4d" : "#ff9a3c";
      ctx.fillStyle = b.permanent ? "rgba(255,77,77,0.14)" : "rgba(255,154,60,0.10)";
      ctx.lineWidth = 1.5;
      if (!b.permanent) ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.arc(px, py, r, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      if (b.strikes > 1 && r > 8) {
        ctx.save();
        ctx.fillStyle = b.permanent ? "#ff8080" : "#ffbe80";
        ctx.font = "10px monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(b.strikes), px, py);
        ctx.restore();
      }
    }
  }

  // Steering queue: dashed rings, deliberately unlike the solid nav-goal ring
  // — these mark regions to explore, not poses to drive to. The head is drawn
  // bright and thick (being worked now); queued ones are dimmer and numbered
  // so the order is readable at a glance. Mirrored from explore_status, so
  // entries vanish as the explorer finishes or expires them.
  steerQueue.forEach((q, i) => {
    const [px, py] = worldToPixel(q.x, q.y);
    const r = Math.max(6, 0.9  * view.scale);
    const head = i === 0;
    ctx.save();
    ctx.strokeStyle = head ? "#5cd0ff" : "rgba(92,208,255,0.45)";
    ctx.lineWidth = head ? 2 : 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.arc(px, py, r, 0, 2 * Math.PI);
    ctx.stroke();
    if (!head) {
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(92,208,255,0.7)";
      ctx.font = "11px monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(String(i), px, py);
    }
    ctx.restore();
  });

  // Nav goal marker (from click-to-nav or dispatch): hollow ring + heading tick.
  if (goal) {
    const [px, py] = worldToPixel(goal.x, goal.y);
    const r = Math.max(3, 0.25  * view.scale);
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
    const r = Math.max(2, 0.2  * view.scale);
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
    drawRobotLabel(px, py, r, ROBOT_ID, "#2d5");
  }

  // Every other robot in the fleet, in its own color — same layering as
  // ROBOT_ID above (frontiers under plan/trail under pose) minus the goal
  // ring, which a peer's plan already implies (see setupPeer's docstring).
  for (const state of peerRobots.values()) {
    for (const fm of state.frontierMarkers) {
      const [px, py] = worldToPixel(fm.x, fm.y);
      const r = Math.max(2, fm.r  * view.scale);
      ctx.fillStyle = fm.color;
      ctx.beginPath();
      ctx.arc(px, py, r, 0, 2 * Math.PI);
      ctx.fill();
    }

    if (state.plan.length > 1) {
      ctx.strokeStyle = state.color.plan;
      ctx.lineWidth = Math.max(1, 0.06  * view.scale);
      ctx.beginPath();
      state.plan.forEach(([x, y], i) => {
        const [px, py] = worldToPixel(x, y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }

    if (state.trail.length > 1) {
      ctx.strokeStyle = state.color.trail;
      ctx.lineWidth = Math.max(1, 0.05  * view.scale);
      ctx.beginPath();
      state.trail.forEach(([x, y], i) => {
        const [px, py] = worldToPixel(x, y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }

    if (showBlacklist) {
      for (const b of state.blacklistPoints) {
        const [px, py] = worldToPixel(b.x, b.y);
        const r = Math.max(3, b.r  * view.scale);
        ctx.save();
        ctx.strokeStyle = state.color.pose;
        ctx.globalAlpha = 0.5;
        ctx.lineWidth = 1.5;
        if (!b.permanent) ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.arc(px, py, r, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.restore();
      }
    }

    state.steerQueue.forEach((q, i) => {
      const [px, py] = worldToPixel(q.x, q.y);
      const r = Math.max(6, 0.9  * view.scale);
      ctx.save();
      ctx.strokeStyle = state.color.pose;
      ctx.globalAlpha = i === 0 ? 0.85 : 0.4;
      ctx.lineWidth = i === 0 ? 2 : 1.5;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.arc(px, py, r, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.restore();
    });

    if (state.lastPose) {
      const [px, py] = worldToPixel(state.lastPose.x, state.lastPose.y);
      const r = Math.max(2, 0.2  * view.scale);
      ctx.fillStyle = state.color.pose;
      ctx.beginPath();
      ctx.arc(px, py, r, 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = state.color.pose;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(
        px + r * 2 * Math.cos(state.lastPose.yaw),
        py - r * 2 * Math.sin(state.lastPose.yaw)
      );
      ctx.stroke();
      drawRobotLabel(px, py, r, state.id, state.color.pose);
    }
  }
}

// Short id label next to a pose marker — without it a fleet map with two
// trails reads as "one robot and some orange noise", and you can't tell
// whether map_anchors + peer TF are actually resolving for the other bot.
function drawRobotLabel(px, py, r, id, color) {
  const label = id.replace(/^robot_/, "r");
  ctx.save();
  ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.textBaseline = "middle";
  ctx.fillStyle = color;
  ctx.strokeStyle = "rgba(0,0,0,0.65)";
  ctx.lineWidth = 3;
  const x = px + r + 4;
  const y = py - r - 2;
  ctx.strokeText(label, x, y);
  ctx.fillText(label, x, y);
  ctx.restore();
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
  // Stop the explorer first, exactly as task_manager.py does before it
  // dispatches (see its _on_dispatch). navigate_to_pose is a SINGLE-GOAL
  // server: without this the explorer keeps issuing its own goals, the two
  // preempt each other every second or so, and — worse — each preemption
  // comes back to the explorer as an ABORTED result, which it scores as a
  // navigation failure and blacklists. Observed live 2026-08-02: manual nav
  // goals were silently poisoning good frontiers and driving spurious
  // escapes, with nothing on either side reporting a conflict.
  // Use `steer` mode to direct exploration without stopping it.
  if (lastExploreStatus && lastExploreStatus.state === "exploring") {
    exploreCmdTopic.publish(new ROSLIB.Message({ data: "stop" }));
  }
  goal = { x, y, yaw }; // instant marker; /plan refreshes it once Nav2 replies
  goalStatus.textContent =
    `nav goal sent: x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${yaw.toFixed(2)}` +
    (lastExploreStatus && lastExploreStatus.state === "exploring"
      ? " — exploration stopped (use steer mode to guide it instead)"
      : "");
}

// What a map click means: "goal" (direct navigate_to_pose, the original
// behaviour) or "steer" (an explore_hint that biases the explorer's frontier
// scoring). They are genuinely different operations, not two ways to do one
// thing: a nav goal competes with the explorer for the single-goal action
// server and loses within about a second, whereas a hint never interrupts
// exploration at all.
// Persisted, because a reload silently reverting to "nav goal" is worse than
// it sounds: the two modes look identical to click but do opposite things,
// and a stray nav goal FIGHTS the explorer (see sendNavGoal) rather than
// steering it. Observed live — a page refresh mid-run turned steer clicks
// into competing Nav2 goals without anything on screen changing.
let mapMode = localStorage.getItem("sortbots.mapMode") || "goal";
const mapModeBar = document.getElementById("map-mode");
const mapHintEl = document.getElementById("map-hint");
const MAP_MODE_HINTS = {
  goal: "Drag to set a Nav2 goal (press = position, drag = heading) — stops autonomous exploration.",
  steer: "Click to send exploration to that area now — shift-click to queue it for after.",
};
function applyMapMode(mode) {
  mapMode = mode;
  localStorage.setItem("sortbots.mapMode", mode);
  for (const b of mapModeBar.querySelectorAll("button")) {
    b.classList.toggle("active", b.dataset.mapmode === mode);
  }
  mapHintEl.textContent = MAP_MODE_HINTS[mode];
}
mapModeBar.addEventListener("click", (ev) => {
  const btn = ev.target.closest("button[data-mapmode]");
  if (btn) applyMapMode(btn.dataset.mapmode);
});
applyMapMode(mapMode);

const exploreHintTopic = new ROSLIB.Topic({
  ros,
  name: `/${ROBOT_ID}/explore_hint`,
  messageType: "std_msgs/String",
  reconnect_on_close: true,
});

// append=true queues the area to visit after the current one instead of
// replacing it (shift-click). The explorer works each region until it's
// mapped, then moves on by itself.
function sendExploreHint(x, y, append) {
  exploreHintTopic.publish(
    new ROSLIB.Message({ data: JSON.stringify(append ? { x, y, append: true } : { x, y }) })
  );
  if (!append) steerHint = { x, y }; // instant marker; explore_status confirms + expires it
  goalStatus.textContent = append
    ? `queued exploration area x=${x.toFixed(2)} y=${y.toFixed(2)}`
    : `steering exploration toward x=${x.toFixed(2)} y=${y.toFixed(2)}`;
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
  if (mapMode === "steer") {
    // Position only — a hint has no heading to express. Shift appends to the
    // queue instead of replacing it.
    sendExploreHint(sx, sy, ev.shiftKey);
    return;
  }
  const [ux, uy] = pixelToWorld(...eventToCanvasPixel(ev));
  const [dx, dy] = [ux - sx, uy - sy];
  // Near-zero drag = a plain click: keep heading 0 rather than amplify jitter.
  const yaw = Math.hypot(dx, dy) > 0.1 ? Math.atan2(dy, dx) : 0;
  sendNavGoal(sx, sy, yaw);
});

// -- 3D reconstruction (RTAB-Map cloud_map, via the relay) ----------------
// A three.js viewer for RTAB-Map's assembled 3D map. The cloud is a
// sensor_msgs/PointCloud2 in the `map` frame (z-up); rosbridge base64-encodes
// its byte payload, which we decode and render either as instanced voxel cubes
// (default — shaded, so vertical structure actually reads) or as flat points.
// The whole thing is wrapped so a missing/broken three.js can never take down
// the rest of the dashboard.
//
// Note we subscribe to /fleet/recon_cloud (nodes/recon_cloud_merge.py), NOT
// per-robot cloud_map: merge transforms each /<id>/recon_cloud into world
// `map` via the same spawn anchors as map_merge, then re-budgets. Per-robot
// recon_cloud still exists for debugging; the panel shows the fused fleet
// cloud so peer markers and voxels share one frame.
//
// Budget note: recon_cloud_relay already caps each robot; merge re-caps the
// union so rosbridge/base64 stay under max_message_size.

const POINTCLOUD_TOPIC = "/fleet/recon_cloud";
const POINTCLOUD_THROTTLE_MS = 3000; // the relay only emits when the pump fires (3s)
const MAX_RENDER_POINTS = 400000;    // decimate above this to keep the browser snappy
// Above this, fall back to plain points — cubes cost 12 triangles each. Set
// deliberately ABOVE the relay's 200k budget so voxels are the normal path and
// this is a genuine safety net (e.g. someone points the panel at raw cloud_map).
const MAX_VOXEL_INSTANCES = 250000;
const VOXEL_SIZE = 0.05;             // matches RTAB-Map's Grid/CellSize

const QS = new URLSearchParams(window.location.search);
const RECON_MODE_INIT = QS.get("recon") === "points" ? "points" : "voxels";
const RECON_COLOR_INIT = QS.get("reconcolor") === "height" ? "height" : "photo";

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

  // A browser with no working WebGL context throws here ("Error creating WebGL
  // context"), which used to take down the whole IIFE — including the
  // subscription at the bottom — leaving #recon-info on its initial "waiting
  // for cloud..." with no canvas and no clue. That reads exactly like a dead
  // ROS topic, and sent us chasing rosbridge and QoS instead of the browser.
  // Chrome drops WebGL on hybrid-graphics/Wayland setups when GPU init fails
  // or hardware acceleration is off; chrome://gpu says which.
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (err) {
    infoEl.textContent = "WebGL unavailable — see chrome://gpu";
    infoEl.title = String((err && err.message) || err);
    container.style.display = "grid";
    container.style.placeItems = "center";
    container.style.color = "#8a939f";
    container.style.font = "0.8rem/1.5 monospace";
    container.style.padding = "12px";
    container.style.textAlign = "center";
    container.textContent =
      "3D panel needs WebGL, which this browser isn't providing.\n" +
      "Check chrome://gpu, or enable hardware acceleration in settings.";
    return;
  }
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

  // Voxels need lighting or they read as one solid silhouette, which defeats
  // the point of drawing them as cubes. Nothing already in the scene cares:
  // GridHelper is LineBasicMaterial, AxesHelper and the robot marker are
  // MeshBasic/LineBasic, and the points path below is unlit too.
  scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x101418, 0.9));
  const sun = new THREE.DirectionalLight(0xffffff, 0.7);
  sun.position.set(1, 1.5, 3);
  scene.add(sun);

  // The point cloud itself (geometry replaced on each new message).
  const cloudMat = new THREE.PointsMaterial({
    size: 0.03,
    sizeAttenuation: true,
    vertexColors: true,
  });
  let cloudPoints = null;

  // -- voxel path --------------------------------------------------------
  // Per-instance colour comes from setColorAt() alone. Do NOT also set
  // vertexColors:true here "to make the colours apply" — it is not needed
  // (verified by rendering both ways under the vendored r137 + swiftshader),
  // and it actively breaks the panel: vertexColors makes color_vertex run
  // `vColor *= color`, BoxGeometry has no color attribute, so the generic
  // vertex-attribute default (0,0,0) applies and every cube renders BLACK.
  // The failure is silent — geometry, lighting and instance count all look
  // fine. If you ever do want vertexColors on this material, you must also
  // give voxelGeom a unit "color" attribute.
  const voxelGeom = new THREE.BoxGeometry(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
  const voxelMat = new THREE.MeshLambertMaterial();
  let voxelMesh = null;
  let voxelCapacity = 0;
  const tmpColor = new THREE.Color();

  let lastCloud = null;                 // decoded payload, kept for re-render
  let reconMode = RECON_MODE_INIT;      // "voxels" | "points"
  let reconColor = RECON_COLOR_INIT;    // "photo"  | "height"

  // Pose markers: active robot is green (matches the 2D #2d5 marker);
  // peers reuse PEER_COLORS from the fleet map. Until peers were added
  // here, the 3D panel only ever showed one sphere — easy to misread as
  // "the other robot isn't in the scene" when it's just not drawn.
  const markerGeom = new THREE.SphereGeometry(0.15, 16, 12);
  const robotMarker = new THREE.Mesh(
    markerGeom,
    new THREE.MeshBasicMaterial({ color: 0x22dd55 })
  );
  robotMarker.visible = false;
  scene.add(robotMarker);
  const peerMarkers = new Map(); // robot_id -> Mesh

  function peerMarkerFor(id, colorHex) {
    let mesh = peerMarkers.get(id);
    if (mesh) return mesh;
    mesh = new THREE.Mesh(
      markerGeom,
      new THREE.MeshBasicMaterial({ color: new THREE.Color(colorHex) })
    );
    mesh.visible = false;
    scene.add(mesh);
    peerMarkers.set(id, mesh);
    return mesh;
  }

  function syncPoseMarker(mesh, pose) {
    if (!pose) return false;
    const moved = !mesh.visible
      || Math.abs(mesh.position.x - pose.x) > 1e-3
      || Math.abs(mesh.position.y - pose.y) > 1e-3;
    if (!moved) return false;
    mesh.visible = true;
    mesh.position.set(pose.x, pose.y, 0.15);
    return true;
  }

  let didFitView = false;
  let lastFitRadius = 0;
  // Declared up here, not next to animate(), because sizeToContainer() runs
  // during setup below and would hit the temporal dead zone otherwise.
  let needsRender = true;

  function sizeToContainer() {
    const w = container.clientWidth || 1;
    const h = container.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    needsRender = true;
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

  // Frame the camera on the cloud's bounds (called on first cloud and by the
  // "reset view" button). Derived from the decoded payload rather than from
  // cloudPoints.geometry.boundingSphere, because in voxel mode there is no
  // cloudPoints — reading it there would silently leave the camera at its
  // (-4,-4,4) init, staring at nothing.
  function cloudRadius() {
    const { min, max } = lastCloud;
    return 0.5 * Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
  }

  function fitView() {
    if (!lastCloud) return;
    const { min, max } = lastCloud;
    const center = new THREE.Vector3(
      (min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2
    );
    const radius = cloudRadius();
    if (!isFinite(radius) || radius === 0) return;
    lastFitRadius = radius;
    controls.target.copy(center);
    // 1.6 rather than a looser factor: the panel is a narrow column, and the
    // bounding sphere of a wide, flat map is dominated by its x/y diagonal, so
    // a generous margin leaves the cloud as a speck in the middle of the grid.
    const d = radius * 1.6;
    camera.position.set(center.x - d, center.y - d, center.z + d);
    camera.near = Math.max(0.05, radius / 100);
    camera.far = radius * 20;
    camera.updateProjectionMatrix();
    controls.update();
    needsRender = true;
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
    // Tracked in the same pass as the decode — six compares per point, and
    // both the height ramp and fitView() need them.
    const min = [Infinity, Infinity, Infinity];
    const max = [-Infinity, -Infinity, -Infinity];
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
      if (x < min[0]) min[0] = x; if (x > max[0]) max[0] = x;
      if (y < min[1]) min[1] = y; if (y > max[1]) max[1] = y;
      if (z < min[2]) min[2] = z; if (z > max[2]) max[2] = z;
      k++;
    }
    if (k === 0) { infoEl.textContent = "cloud empty"; return; }

    // k may be < n if points were skipped — keep only the filled part.
    lastCloud = {
      positions: positions.subarray(0, k * 3),
      colors: colors.subarray(0, k * 3),
      n: k, total, stride, min, max,
    };
    rebuild();
    // Re-frame while the map is still growing substantially. Fitting only once
    // used to be fine when the first cloud was already most of the map; now the
    // first cloud is often a few seconds of floor and the map ends up 20x
    // larger, which would leave the camera parked on a speck. Once growth
    // settles below the threshold the view stops moving under the user.
    if (!didFitView || cloudRadius() > lastFitRadius * 1.5) {
      fitView();
      didFitView = true;
    }
  }

  // -- render paths ------------------------------------------------------
  // Per-point colour for the current colour mode. "photo" is the cloud's own
  // rgb; "height" ramps blue (floor) -> cyan -> yellow -> red (shelf top),
  // which is what actually makes vertical structure legible when the cloud's
  // real colours are all warehouse-grey.
  function colorAt(c, i) {
    if (reconColor === "photo") {
      return c.setRGB(
        lastCloud.colors[i * 3], lastCloud.colors[i * 3 + 1], lastCloud.colors[i * 3 + 2]
      );
    }
    const zMin = lastCloud.min[2];
    const span = lastCloud.max[2] - zMin || 1;
    const t = (lastCloud.positions[i * 3 + 2] - zMin) / span;
    return c.setHSL(0.68 * (1 - t), 0.85, 0.25 + 0.35 * t);
  }

  function buildVoxels() {
    const n = lastCloud.n;
    if (n > voxelCapacity) {
      // InstancedMesh buffers are fixed at construction, so grow in 50k steps
      // and reuse — a steadily-growing map would otherwise reallocate on
      // every single message.
      if (voxelMesh) { scene.remove(voxelMesh); voxelMesh.dispose(); }
      voxelCapacity = Math.ceil(n / 50000) * 50000;
      voxelMesh = new THREE.InstancedMesh(voxelGeom, voxelMat, voxelCapacity);
      // The bounding sphere comes from the GEOMETRY (one 5 cm cube at the
      // origin), not from the instances, so three.js would cull the entire
      // mesh the moment the origin left frame.
      voxelMesh.frustumCulled = false;
      scene.add(voxelMesh);
    }

    // Write translations straight into the instance matrix. Every instance is
    // an axis-aligned unit-scale translation, so the basis is the identity and
    // only elements 12/13/14 vary — much cheaper than n Object3D.updateMatrix()
    // calls, and this runs on the main thread every time a cloud lands.
    const m = voxelMesh.instanceMatrix.array;
    for (let i = 0; i < n; i++) {
      const o = i * 16;
      m[o] = m[o + 5] = m[o + 10] = m[o + 15] = 1;
      m[o + 1] = m[o + 2] = m[o + 3] = m[o + 4] = 0;
      m[o + 6] = m[o + 7] = m[o + 8] = m[o + 9] = 0;
      m[o + 11] = 0;
      m[o + 12] = lastCloud.positions[i * 3];
      m[o + 13] = lastCloud.positions[i * 3 + 1];
      m[o + 14] = lastCloud.positions[i * 3 + 2];
      voxelMesh.setColorAt(i, colorAt(tmpColor, i));
    }
    voxelMesh.count = n;
    voxelMesh.instanceMatrix.needsUpdate = true;
    if (voxelMesh.instanceColor) voxelMesh.instanceColor.needsUpdate = true;
  }

  function buildPoints() {
    const n = lastCloud.n;
    const colors = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      colorAt(tmpColor, i);
      colors[i * 3] = tmpColor.r;
      colors[i * 3 + 1] = tmpColor.g;
      colors[i * 3 + 2] = tmpColor.b;
    }
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(lastCloud.positions, 3));
    geom.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    if (cloudPoints) {
      cloudPoints.geometry.dispose();
      cloudPoints.geometry = geom;
    } else {
      cloudPoints = new THREE.Points(geom, cloudMat);
      scene.add(cloudPoints);
    }
  }

  function rebuild() {
    if (!lastCloud) return;
    const useVoxels = reconMode === "voxels" && lastCloud.n <= MAX_VOXEL_INSTANCES;
    if (useVoxels) buildVoxels(); else buildPoints();
    // Toggle visibility rather than disposing, so flipping modes is instant.
    if (voxelMesh) voxelMesh.visible = useVoxels;
    if (cloudPoints) cloudPoints.visible = !useVoxels;

    const { n, total, stride, min, max } = lastCloud;
    const capped = reconMode === "voxels" && !useVoxels ? " — over cap, points" : "";
    // Fused fleet cloud in `map` (nodes/recon_cloud_merge.py) — not one
    // robot's private recon. Peer pose markers use the same frame.
    infoEl.textContent =
      `fleet · ${n.toLocaleString()} ${useVoxels ? "vox" : "pts"} · ` +
      `z ${min[2].toFixed(1)}–${max[2].toFixed(1)} m` +
      (stride > 1 ? ` (of ${total.toLocaleString()}, 1/${stride})` : "") + capped;
    needsRender = true;
  }

  function wireSelect(id, initial, apply) {
    const el = document.getElementById(id);
    if (!el) return;
    el.value = initial;
    el.addEventListener("change", () => { apply(el.value); rebuild(); });
  }
  wireSelect("recon-mode", RECON_MODE_INIT, (v) => { reconMode = v; });
  wireSelect("recon-color", RECON_COLOR_INIT, (v) => { reconColor = v; });

  new ROSLIB.Topic({
    ros,
    name: POINTCLOUD_TOPIC,
    messageType: "sensor_msgs/PointCloud2",
    throttle_rate: POINTCLOUD_THROTTLE_MS,
    queue_length: 1,
    reconnect_on_close: true,
  }).subscribe(onCloud);

  // Render on demand, not every frame.
  //
  // This panel shares a GPU with Isaac Sim. Points were cheap enough to redraw
  // at 60 fps forever, but a voxel build is ~100k instanced cubes (1.2M
  // triangles), and burning that continuously while nothing changes is enough
  // to starve Isaac's render products — the sim keeps stepping while its
  // camera image topics silently stop (camera_info, which needs no rendering,
  // keeps going). So draw only when something actually changed: a new cloud, a
  // camera move, a resize, or the robot marker relocating.
  controls.addEventListener("change", () => { needsRender = true; });

  function animate() {
    requestAnimationFrame(animate);
    if (syncPoseMarker(robotMarker, lastPose)) needsRender = true;
    for (const [id, state] of peerRobots) {
      if (syncPoseMarker(peerMarkerFor(id, state.color.pose), state.lastPose)) {
        needsRender = true;
      }
    }
    // Cheap on CPU and emits "change" only when the camera really moves, so
    // damping still settles smoothly without pinning the GPU.
    controls.update();
    if (!needsRender) return;
    needsRender = false;
    renderer.render(scene, camera);
  }
  animate();
})();
