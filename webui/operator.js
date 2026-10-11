// SortBots operator view (webui/operator.html). Fleet-first: every robot, one map,
// the task list and the waypoints, with one robot's live view a click away. The
// per-robot engineering dashboard (app.js) is untouched and stays at /?robot=ID.
//
// Talks to two things, like the rest of the webui:
//   - rosbridge (9090)   live topics: /map, /tf, per-robot plan / explore_status /
//                        task_status / action status, and the commands this page
//                        sends (dispatch_task, explore_cmd, navigate_to_pose, cmd_vel)
//   - serve.py (HTTP)    /api/robots, /api/nav_waypoints, /api/waypoints (task
//                        stations), /api/session (the console's control API)
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
  const GOAL_STATES = { 1: "accepted", 2: "navigating", 3: "cancelling", 4: "arrived", 5: "cancelled", 6: "failed" };

  const $ = (s) => document.querySelector(s);
  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const nameOf = (id) => id.replace(/^robot_/, "Robot ");

  // -- state ------------------------------------------------------------------

  const robots = new Map(); // id -> {id, color, pose, linkAt, plan, explore, exploreAt, task, taskAt, queueDepth, nav, navStamp}
  let selected = null;
  let tool = "select";
  let waypoints = []; // [{name, x, y, yaw}], the server's copy
  let stations = {}; // task stations from configs/waypoints.yaml: name -> {kind, ...}
  let tasks = []; // [{id, pickup, dropoff, robot, state, at}]
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

  const UNKNOWN = 0xfffcfbfb, FREE = 0xffedeae7, OCCUPIED = 0xff8f827a; // ABGR (little-endian RGBA)

  topic("/map", "nav_msgs/OccupancyGrid").subscribe((msg) => {
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
        if (v < 0) { px[dst + col] = UNKNOWN; continue; }
        px[dst + col] = v >= 50 ? OCCUPIED : FREE;
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
  });

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

  // -- drawing ----------------------------------------------------------------------

  function drawFrame() {
    requestAnimationFrame(drawFrame);
    if (!dirty || !cssW) return;
    dirty = false;
    ctx.fillStyle = "#f2f3f4";
    ctx.fillRect(0, 0, cssW, cssH);

    if (mapCanvas && mapInfo) {
      const { width: w, height: h, resolution: res, origin } = mapInfo;
      const [dx, dy] = toPx(origin.position.x, origin.position.y + h * res);
      ctx.imageSmoothingEnabled = false; // crisp cells
      ctx.drawImage(mapCanvas, dx, dy, w * res * view.scale, h * res * view.scale);
    }

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
      ctx.fillStyle = "#f2b705"; ctx.strokeStyle = "#15191e"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(a, b - 9); ctx.lineTo(a + 6, b); ctx.lineTo(a, b + 9); ctx.lineTo(a - 6, b); ctx.closePath();
      ctx.fill(); ctx.stroke();
      ctx.fillStyle = "#15191e"; ctx.fillText(w.name, a, b - 14);
    }

    for (const r of robots.values()) {
      if (!r.pose) continue;
      const [a, b] = toPx(r.pose.x, r.pose.y), live = online(r);
      ctx.globalAlpha = live ? 1 : 0.4;
      if (selected === r.id) { ctx.strokeStyle = r.color; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(a, b, 17, 0, 7); ctx.stroke(); }
      ctx.fillStyle = r.color; ctx.beginPath(); ctx.arc(a, b, 10, 0, 7); ctx.fill();
      const yaw = -r.pose.yaw; // canvas y points down
      ctx.fillStyle = "#fff"; ctx.beginPath();
      ctx.moveTo(a + Math.cos(yaw) * 13, b + Math.sin(yaw) * 13);
      ctx.lineTo(a + Math.cos(yaw + 2.5) * 6, b + Math.sin(yaw + 2.5) * 6);
      ctx.lineTo(a + Math.cos(yaw - 2.5) * 6, b + Math.sin(yaw - 2.5) * 6);
      ctx.fill();
      ctx.fillStyle = "#15191e"; ctx.font = '600 11px "IBM Plex Sans", system-ui, sans-serif';
      ctx.fillText(nameOf(r.id), a, b + 28);
      ctx.globalAlpha = 1;
    }
  }
  requestAnimationFrame(drawFrame);

  // -- robots --------------------------------------------------------------------------

  const online = (r) => !!r.pose && performance.now() - r.linkAt < ONLINE_TTL_MS;

  function addRobot(id) {
    if (robots.has(id)) return;
    const r = {
      id, color: COLORS[robots.size % COLORS.length], pose: null, linkAt: -Infinity, plan: [],
      explore: null, exploreAt: -Infinity, task: null, taskAt: -Infinity, queueDepth: 0,
      nav: null, seenStamp: {},
    };
    robots.set(id, r);

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
      if (s.task_id) noteTask(id, s);
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
    const busy = r.task && r.task.state && !["IDLE", "DONE", "FAILED"].includes(r.task.state);
    if (busy && now - r.taskAt < EXPLORE_TTL_MS * 4) {
      return { tag: "on a task", detail: TASK_PHASE[r.task.state] || r.task.state };
    }
    if (r.nav && !r.nav.done) return { tag: "navigating", detail: `Going to ${r.nav.label}` };
    if (r.explore && r.explore.state === "exploring" && now - r.exploreAt < EXPLORE_TTL_MS) return { tag: "exploring", detail: "Exploring" };
    return { tag: "idle", detail: r.nav && r.nav.state === "failed" ? "Last goal failed" : "Idle" };
  }

  const isFree = (r) => online(r) && statusOf(r).tag === "idle" && !r.queueDepth;

  // -- tasks ---------------------------------------------------------------------------
  // task_manager keeps one queue per robot and reports only the CURRENT task
  // ({task_id, state, queue_depth}) on /<id>/task_status. The list here is therefore:
  // tasks this page dispatched (waiting until a status mentions their id) plus any task
  // a status mentions, so tasks started elsewhere still show up. There is no cancel.

  const TASK_PHASE = {
    NAV_TO_PICKUP: "Heading to pick-up", PICKING: "Picking up", NAV_TO_DROPOFF: "Heading to drop-off",
    PLACING: "Placing", DONE: "Done", FAILED: "Failed",
  };

  function noteTask(robotId, s) {
    let t = tasks.find((x) => x.id === s.task_id);
    if (!t) { t = { id: s.task_id, pickup: null, dropoff: null, robot: robotId, state: s.state, at: Date.now() }; tasks.push(t); }
    t.robot = robotId; t.state = s.state; t.at = Date.now();
    trimTasks(); renderTasks(); renderFleet();
  }
  function trimTasks() {
    const finished = tasks.filter((t) => t.state === "DONE" || t.state === "FAILED").sort((a, b) => b.at - a.at);
    for (const t of finished.slice(8)) tasks.splice(tasks.indexOf(t), 1);
  }

  const TASK_LABEL = (t) => ({
    QUEUED: ["waiting", ""], NAV_TO_PICKUP: ["en route", "active"], PICKING: ["picking up", "active"],
    NAV_TO_DROPOFF: ["en route", "active"], PLACING: ["placing", "active"], DONE: ["done", "done"], FAILED: ["failed", "failed"],
  }[t.state] || [t.state.toLowerCase(), "active"]);

  function renderTasks() {
    const rank = (t) => (t.state === "DONE" || t.state === "FAILED" ? 2 : t.state === "QUEUED" ? 1 : 0);
    const list = [...tasks].sort((a, b) => rank(a) - rank(b) || a.at - b.at);
    $("#tasks").innerHTML = list.length ? list.map((t) => {
      const [label, cls] = TASK_LABEL(t), r = robots.get(t.robot);
      const title = t.pickup ? `${esc(t.pickup)} to ${esc(t.dropoff)}` : `Task ${esc(t.id)}`;
      return `<div class="tk ${rank(t) === 2 ? "finished" : ""}"><span class="t1">${title}</span><span class="st ${cls}">${label}</span>
        <span class="t2">${esc(t.id)}</span><span class="who">${r ? `<i style="--c:${r.color}"></i>${esc(nameOf(r.id))}` : ""}</span></div>`;
    }).join("") : `<div class="none">No tasks yet. Add one below.</div>`;
    const active = tasks.filter((t) => rank(t) === 0).length, waiting = tasks.filter((t) => t.state === "QUEUED").length;
    $("#tk-count").textContent = tasks.length ? `${active} running, ${waiting} waiting` : "";
  }

  // -- fleet list ---------------------------------------------------------------------------

  let lastFleetSig = "";
  const camEl = document.createElement("div");
  camEl.className = "cam";
  camEl.innerHTML = "<span>Connecting to camera…</span>";

  function renderFleet() {
    const list = [...robots.values()];
    const rows = list.map((r) => {
      const st = statusOf(r), open = selected === r.id;
      return { r, st, open, pose: r.pose ? `${r.pose.x.toFixed(1)}, ${r.pose.y.toFixed(1)}` : "", exploring: st.tag === "exploring" };
    });
    // The position readout changes every few centimetres; patch it in place rather than
    // rebuilding the list under the operator's cursor (a rebuild between mousedown and
    // mouseup eats the click).
    const facts = $(".more .facts span");
    if (facts) { const o = rows.find((x) => x.open); if (o) facts.textContent = o.pose ? `${o.pose} m` : "no position yet"; }
    const sig = JSON.stringify(rows.map((x) => [x.r.id, x.st, x.open, online(x.r)]));
    if (sig === lastFleetSig) return;
    lastFleetSig = sig;
    const onlineN = list.filter(online).length;
    $("#n-robots").textContent = list.length ? `${onlineN} of ${list.length} online` : "";
    $("#fleet").innerHTML = list.length ? rows.map(({ r, st, open, pose, exploring }) => `
      <div class="robot ${open ? "sel" : ""} ${online(r) ? "" : "offline"}" data-id="${esc(r.id)}" style="--c:${r.color}" tabindex="0" role="button" aria-expanded="${open}">
        <div class="l1"><span class="sw"></span><span class="name">${esc(nameOf(r.id))}</span><span class="tag">${esc(st.tag)}</span></div>
        <div class="l2">${esc(st.detail)}</div>
        ${open ? `<div class="more">
          <div id="cam-slot"></div>
          <div class="facts"><span>${pose ? `${pose} m` : "no position yet"}</span></div>
          <div class="acts"><button class="btn small" data-act="stop">Stop</button>
            <button class="btn small" data-act="explore">${exploring ? "Stop exploring" : "Explore"}</button></div>
          <a class="open" href="/?robot=${encodeURIComponent(r.id)}" target="_blank" rel="noopener" title="The full per-robot dashboard: cameras, SLAM, drive pad">Open live view</a>
        </div>` : ""}
      </div>`).join("") : `<div class="none">No robots found.</div>`;
    const slot = $("#cam-slot");
    if (slot) slot.replaceWith(camEl);
    renderTargets();
  }

  // Camera thumbnail for the selected robot: short /snapshot polls, never MJPEG, and
  // always qos_profile=sensor_data (see app.js for the failure mode both avoid).
  let camTimer = null, camToken = 0;
  function startCam() {
    stopCam();
    if (!selected) return;
    const id = selected, token = ++camToken;
    const tick = () => {
      if (token !== camToken) return;
      const probe = new Image();
      probe.onload = () => {
        if (token !== camToken) return;
        camEl.style.backgroundImage = `url(${probe.src})`;
        camEl.innerHTML = "";
        camTimer = setTimeout(tick, 400);
      };
      probe.onerror = () => {
        if (token !== camToken) return;
        camEl.style.backgroundImage = "";
        camEl.innerHTML = "<span>No camera feed</span>";
        camTimer = setTimeout(tick, 3000);
      };
      probe.src = `http://${HOST}:${VIDEO_PORT}/snapshot?topic=/${id}/camera/rgb&qos_profile=sensor_data&_=${Date.now()}`;
    };
    camEl.style.backgroundImage = ""; camEl.innerHTML = "<span>Connecting to camera…</span>";
    tick();
  }
  function stopCam() { camToken++; clearTimeout(camTimer); }

  function select(id) {
    selected = id;
    lastFleetSig = "";
    renderFleet();
    startCam();
    setTool(tool);
    dirty = true;
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
  function renderWaypoints() {
    $("#wps").innerHTML = waypoints.length ? waypoints.map((w) =>
      `<span class="wp"><button class="go" data-go="${esc(w.name)}" title="Send the selected robot here">${esc(w.name)}</button><button class="del" data-del="${esc(w.name)}" title="Delete ${esc(w.name)}" aria-label="Delete ${esc(w.name)}">&times;</button></span>`
    ).join("") : `<div class="none" style="padding:0">None yet. Pick the waypoint tool and click the map.</div>`;
  }

  // -- task form -----------------------------------------------------------------------------------------

  async function loadStations() {
    try { stations = await (await fetch("/api/waypoints")).json(); } catch (e) { console.error("failed to load stations", e); stations = {}; }
    const opts = (kind) => Object.entries(stations).filter(([, s]) => s.kind === kind).map(([n]) => `<option>${esc(n)}</option>`).join("");
    const from = opts("pickup"), to = opts("dropoff");
    $("#nt-from").innerHTML = from || "<option value=''>(no pick-up stations)</option>";
    $("#nt-to").innerHTML = to || "<option value=''>(no drop-off stations)</option>";
    $("#nt-add").disabled = !from || !to;
  }
  function renderTargets() {
    const keep = $("#nt-who").value;
    $("#nt-who").innerHTML = `<option value="auto">Any free robot</option>` +
      [...robots.values()].map((r) => `<option value="${esc(r.id)}">${esc(nameOf(r.id))}</option>`).join("");
    if ([...$("#nt-who").options].some((o) => o.value === keep)) $("#nt-who").value = keep;
  }
  $("#task-form").addEventListener("submit", (ev) => {
    ev.preventDefault();
    const pickup = $("#nt-from").value, dropoff = $("#nt-to").value;
    if (!pickup || !dropoff) return;
    let who = $("#nt-who").value;
    if (who === "auto") {
      const free = [...robots.values()].find(isFree);
      if (!free) return toast("No robot is free. Pick one to queue the task on it.");
      who = free.id;
    }
    const id = Math.random().toString(16).slice(2, 10);
    publishString(`/${who}/dispatch_task`, JSON.stringify({ pickup, dropoff, task_id: id }));
    tasks.push({ id, pickup, dropoff, robot: who, state: "QUEUED", at: Date.now() });
    renderTasks();
    toast(`Task queued for ${nameOf(who)}`);
  });

  // -- session / header ----------------------------------------------------------------------------------------

  const ACTIVE = ["starting", "running", "stopping"];
  const sessionActive = () => !!session && ACTIVE.includes(session.state);

  function renderHeader() {
    const s = session, cfg = s && sessionActive() ? s.config : null;
    const premapped = !!cfg && cfg.map && cfg.map !== "new" && cfg.mode === "readonly";
    for (const b of document.querySelectorAll("#mode button")) {
      b.classList.toggle("active", !!cfg && (b.dataset.m === "loc") === premapped);
    }
    $("#map-name").textContent = cfg && cfg.map && cfg.map !== "new" ? (cfg.map === "working" ? "last map" : cfg.map) : "";
    const label = !s ? "No session" : { starting: `Starting: ${s.phase_label || "…"}`, running: "Session running", stopping: "Stopping…", failed: "Session failed" }[s.state] || "No session";
    $("#s-label").textContent = controlAvailable ? label : "Session";
    $("#s-dot").className = "dot " + (s && s.state === "running" ? "" : s && ["starting", "stopping"].includes(s.state) ? "busy" : "idle");
    renderEmpty();
  }

  function renderEmpty() {
    const anyOnline = [...robots.values()].some(online);
    const show = !anyOnline && !mapInfo;
    $("#empty").hidden = !show;
    if (!show) return;
    const starting = session && ["starting", "stopping"].includes(session.state);
    $("#empty-title").textContent = starting ? "Starting up" : controlAvailable ? "No session running" : "Waiting for the robots";
    $("#empty-sub").textContent = starting ? (session.phase_label || "") : controlAvailable
      ? "Start a session to see your robots and give them tasks."
      : "No robot is reporting yet. Check that the robot console is running.";
    $("#empty-go").hidden = starting || !controlAvailable;
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

  const HINTS = {
    select: () => (selected ? "Click the map for waypoints and sends, or pick another robot" : "Click a robot to see its camera"),
    wp: () => "Click the map to add a waypoint. Drag to set the heading",
    goto: () => (selected ? `Click the map to send ${nameOf(selected)} there. Drag to set the heading` : "Select a robot first, then click a destination"),
  };
  function setTool(t) {
    tool = t;
    for (const b of document.querySelectorAll("#tools button")) b.classList.toggle("active", b.dataset.t === t);
    $("#hint").textContent = HINTS[t]();
    canvas.style.cursor = t === "select" ? "grab" : "crosshair";
  }
  for (const b of document.querySelectorAll("#tools button")) b.addEventListener("click", () => setTool(b.dataset.t));

  let press = null; // {px, py, wx, wy, moved, panStart}
  canvas.addEventListener("pointerdown", (e) => {
    const r = canvas.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    const [wx, wy] = toWorld(px, py);
    press = { px, py, wx, wy, moved: false, cx: view.cx, cy: view.cy };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!press) return;
    const r = canvas.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    if (Math.hypot(px - press.px, py - press.py) > 6) press.moved = true;
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
    const p = press; press = null;
    if (tool === "select") {
      if (p.moved) return;
      const hit = [...robots.values()].find((b) => {
        if (!b.pose) return false;
        const [a, c] = toPx(b.pose.x, b.pose.y);
        return Math.hypot(a - px, c - py) < 16;
      });
      select(hit ? (hit.id === selected ? null : hit.id) : selected);
      return;
    }
    const [wx, wy] = toWorld(px, py);
    const yaw = p.moved ? Math.atan2(wy - p.wy, wx - p.wx) : 0; // heading from the drag
    if (tool === "wp") {
      const name = nextWaypointName();
      saveWaypoints([...waypoints, { name, x: p.wx, y: p.wy, yaw }]);
      toast(`Waypoint ${name} added`);
    } else if (tool === "goto") {
      if (!selected) return toast("Select a robot first");
      sendNav(selected, p.wx, p.wy, yaw, `(${p.wx.toFixed(1)}, ${p.wy.toFixed(1)})`);
    }
  });
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
    const act = e.target.closest("[data-act]");
    if (act && selected) {
      if (act.dataset.act === "stop") { stopRobot(selected); toast(`${nameOf(selected)} stopped`); }
      if (act.dataset.act === "explore") toggleExplore(selected);
      return;
    }
    if (e.target.closest("a")) return;
    const row = e.target.closest(".robot");
    if (row) select(selected === row.dataset.id && !e.target.closest(".more") ? null : row.dataset.id);
  });
  $("#fleet").addEventListener("keydown", (e) => {
    if ((e.key === "Enter" || e.key === " ") && e.target.classList.contains("robot")) { e.preventDefault(); e.target.click(); }
  });
  $("#wps").addEventListener("click", (e) => {
    const go = e.target.dataset.go, del = e.target.dataset.del;
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
  $("#mode").addEventListener("click", (e) => {
    if (e.target.dataset.m && !e.target.classList.contains("active")) openSession(); // switching restarts the stack
  });
  $("#sp-x").addEventListener("click", closeSession);
  $("#session").addEventListener("click", (e) => { if (e.target.id === "session") closeSession(); });
  addEventListener("keydown", (e) => {
    if (e.key === "Escape") { if (!$("#session").hidden) closeSession(); else if (selected) select(null); return; }
    if (e.target.closest && e.target.closest("input, select, textarea") || e.metaKey || e.ctrlKey || e.altKey) return;
    const t = { v: "select", f: "wp", g: "goto" }[e.key.toLowerCase()];
    if (t) setTool(t);
  });
  addEventListener("resize", resize);

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
    if (moved) { dirty = true; if (!view.moved) fitView(); }
    renderFleet(); renderEmpty();
  }, 250);

  // -- boot ----------------------------------------------------------------------------------------------------------------------
  resize(); setTool("select"); renderTasks(); renderWaypoints();
  refreshRoster(); loadWaypoints(); loadStations(); pollSession();
  setInterval(loadWaypoints, 10000); // another operator may have added pins
  setInterval(refreshRoster, 15000);

  // Test hook: the offline test (tests/operator_test.mjs) drives state through here.
  window.__operator = { robots, tfTree, lookupPose, onTf, addRobot, statusOf, noteTask, get tasks() { return tasks; }, get waypoints() { return waypoints; }, select, view, toPx, toWorld };
})();
