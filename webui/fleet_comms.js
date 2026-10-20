// SortBots dashboard — Fleet mesh tab. Monitors /fleet/status + /fleet/intent
// and renders the gateway's /fleet/task_board as the operator's annunciator.
//
// Own rosbridge connection (same pattern as arm.js): independent of app.js's
// ~15 topic subscriptions, so opening this tab does not require the live
// view's robot picker state. Autonomy nodes use this same radio — no TF
// peer eavesdropping — so what you see here is what explorers hear.
//
// Liveness, battery and task ownership come from the BOARD, not from this
// page's own heartbeat ages: the board is the gateway's protocol state
// (nodes/fleet_tasks.py --gateway, same status_timeout_s the robots apply to
// each other), so the GUI never disagrees with the fleet about who is gone.
// The heartbeat age is still shown, as a secondary hint. Requirement
// 2026-10-20: these three facts must reach the operator no matter what.
//
// View switching lives in scenarios.js (it already owns #view-mode); this
// file only renders when #view-fleet is shown and keeps a short event log.

(function () {
  const ROSBRIDGE_HOST = window.location.hostname || "localhost";
  const ROSBRIDGE_PORT = 9090;
  const STATUS_TOPIC = "/fleet/status";
  const INTENT_TOPIC = "/fleet/intent";
  const BOARD_TOPIC = "/fleet/task_board";
  const ACK_TOPIC = "/fleet/alarm_ack";
  const STATUS_FRESH_MS = 1000;
  const STATUS_AGING_MS = 2500;
  // The gateway publishes the board at 1 Hz; older than this and the gateway
  // itself is the thing to worry about (its alarms can no longer arrive).
  const BOARD_STALE_MS = 5000;
  const LOW_BATTERY_PCT = 20; // protocol battery_reserve_pct: below it a robot stops bidding
  const INTENT_TTL_FALLBACK_S = 90;
  const MAX_LOG = 80;
  const TICK_MS = 250;

  const viewEl = document.getElementById("view-fleet");
  const statusList = document.getElementById("fleet-status-list");
  const tasksList = document.getElementById("fleet-tasks-list");
  const intentList = document.getElementById("fleet-intent-list");
  const alarmsEl = document.getElementById("fleet-alarms");
  const alarmCountEl = document.getElementById("fleet-alarm-count");
  const boardAgeEl = document.getElementById("fleet-board-age");
  const logEl = document.getElementById("fleet-log");
  const connEl = document.getElementById("fleet-conn");
  if (!viewEl || typeof ROSLIB === "undefined") return;

  /** @type {Map<string, {raw: object, recvAt: number}>} */
  const statuses = new Map();
  /** @type {Map<string, {raw: object, recvAt: number}>} */
  const intentMap = new Map();
  /** The last /fleet/task_board, or null before the gateway has spoken. */
  let board = null;
  let boardAt = 0;
  const seenAlarms = new Set();
  const logLines = [];
  let connected = false;
  let tickTimer = null;

  function fmt(n, digits = 2) {
    return typeof n === "number" && Number.isFinite(n) ? n.toFixed(digits) : "—";
  }

  function ageClass(ageMs) {
    if (ageMs <= STATUS_FRESH_MS) return "fresh";
    if (ageMs <= STATUS_AGING_MS) return "aging";
    return "stale";
  }

  function pushLog(kind, text) {
    const t = new Date().toLocaleTimeString();
    logLines.push(`[${t}] ${kind} ${text}`);
    while (logLines.length > MAX_LOG) logLines.shift();
    if (logEl) {
      logEl.textContent = logLines.join("\n");
      logEl.scrollTop = logEl.scrollHeight;
    }
  }

  function boardRobots() {
    const out = new Map();
    for (const r of (board && board.robots) || []) if (r && r.id) out.set(r.id, r);
    return out;
  }

  function renderStatus() {
    const now = Date.now();
    const fleet = boardRobots();
    // Every robot either this page or the gateway has heard. A robot on the
    // board with no /fleet/status here is still a robot (e.g. the bridge is
    // forwarding only the board), and a silent one must stay visible: an
    // annunciator that drops the dead robot's card has hidden the alarm.
    const ids = [...new Set([...statuses.keys(), ...fleet.keys()])].sort();
    if (!ids.length) {
      statusList.innerHTML = '<div id="fleet-empty-status">No status yet.</div>';
      return;
    }
    statusList.innerHTML = "";
    for (const id of ids) {
      const entry = statuses.get(id);
      const s = entry ? entry.raw : {};
      const fr = fleet.get(id);
      const age = entry ? now - entry.recvAt : null;
      const pose = s.pose || {};
      const twist = s.twist || {};
      const mode = (fr && fr.mode) || s.mode || "idle";
      const dead = fr ? fr.alive === false : false;
      const batt = fr && fr.battery_pct != null ? fr.battery_pct
        : (typeof s.battery_pct === "number" ? s.battery_pct : null);
      const card = document.createElement("div");
      card.dataset.robot = id;
      card.className = `fleet-card ${dead ? "dead" : (age == null ? "stale" : ageClass(age))}`;
      const owned = fr ? fr.owned || [] : (s.owned || []).map((t) => t[0]);
      const carrying = new Set(fr ? fr.carrying || [] : []);
      const tasksStr = owned.length
        ? owned.map((t) => `${escapeHtml(String(t))}${carrying.has(t) ? " (carrying)" : ""}`).join(", ")
        : "—";
      card.innerHTML =
        `<h4>${escapeHtml(id)} ` +
        `<span class="fleet-mode ${dead ? "dead" : escapeHtml(mode)}">${dead ? "SILENT" : escapeHtml(mode)}</span>` +
        `<span class="fleet-batt ${batt != null && batt < LOW_BATTERY_PCT ? "low" : ""}">` +
        `${batt != null ? "⚡ " + fmt(batt, 1) + " %" : "⚡ —"}</span></h4>` +
        `<dl>` +
        `<dt>fleet</dt><dd>${fr ? (dead ? "silent" : "alive") +
          (fr.last_heard_s != null ? ` · heard ${fmt(fr.last_heard_s, 1)} s ago` : "") : "no board"}</dd>` +
        `<dt>tasks</dt><dd>${tasksStr}</dd>` +
        `<dt>pose</dt><dd>${entry ? `x=${fmt(pose.x)} y=${fmt(pose.y)} yaw=${fmt(pose.yaw)}` : "—"}</dd>` +
        `<dt>twist</dt><dd>${entry ? `vx=${fmt(twist.vx, 3)} wz=${fmt(twist.wz, 3)}` : "—"}</dd>` +
        `<dt>age</dt><dd>${age != null ? (age / 1000).toFixed(2) + " s" : "no status here"}</dd>` +
        `</dl>`;
      statusList.appendChild(card);
    }
  }

  function renderTasks() {
    const tasks = (board && board.tasks) || [];
    if (!tasks.length) {
      tasksList.innerHTML = '<div id="fleet-empty-tasks">No tasks yet.</div>';
      return;
    }
    const rows = tasks.map((t) => {
      const cls = t.committed ? "carrying" : escapeHtml(String(t.state || ""));
      const state = t.committed ? "carrying" : (t.state || "?");
      return `<tr class="${cls}" data-task="${escapeHtml(String(t.task_id))}">` +
        `<td>${escapeHtml(String(t.task_id))}</td>` +
        `<td>${escapeHtml(t.pickup || "?")} → ${escapeHtml(t.dropoff || "?")}</td>` +
        `<td class="state">${escapeHtml(state)}</td>` +
        `<td>${escapeHtml(t.owner || "—")}</td>` +
        `<td>${t.age_s != null ? fmt(t.age_s, 0) + " s" : "—"}</td></tr>`;
    });
    tasksList.innerHTML =
      `<table id="fleet-tasks"><thead><tr><th>task</th><th>route</th><th>state</th>` +
      `<th>owner</th><th>age</th></tr></thead><tbody>${rows.join("")}</tbody></table>`;
  }

  function renderAlarms() {
    const alarms = (board && board.alarms) || [];
    for (const el of alarmsEl.querySelectorAll(".fleet-alarm")) el.remove();
    const active = alarms.filter((a) => a.active).length;
    alarmCountEl.textContent = alarms.length
      ? `${active} active, ${alarms.length - active} unacknowledged` : "none";
    alarmCountEl.classList.toggle("none", !alarms.length);
    // Active first, newest first within each group: the operator reads the
    // top row.
    const sorted = [...alarms].sort((a, b) => (b.active - a.active) || (b.t - a.t));
    const t0 = board ? board.t : 0;
    for (const a of sorted) {
      const row = document.createElement("div");
      row.className = `fleet-alarm ${a.active ? "" : "cleared"}`;
      row.dataset.alarm = String(a.id);
      row.innerHTML =
        `<span class="kind">${escapeHtml(a.kind)}</span>` +
        `<span>${escapeHtml(a.text || "")}</span>` +
        `<span class="age">${fmt(Math.max(0, t0 - a.t), 0)} s ago${a.active ? "" : " · cleared"}</span>` +
        `<button type="button" title="Acknowledge: the row leaves once the condition has also cleared.">ack</button>`;
      row.querySelector("button").addEventListener("click", () => ackAlarm(a.id));
      alarmsEl.appendChild(row);
    }
  }

  function renderBoardAge() {
    if (!board) {
      boardAgeEl.textContent = "no board yet";
      boardAgeEl.classList.add("stale");
      return;
    }
    const age = Date.now() - boardAt;
    boardAgeEl.textContent = `board ${(age / 1000).toFixed(1)} s ago`;
    boardAgeEl.classList.toggle("stale", age > BOARD_STALE_MS);
  }

  let ackTopic = null;
  function ackAlarm(id) {
    if (!ackTopic) return;
    ackTopic.publish(new ROSLIB.Message({ data: JSON.stringify({ id }) }));
    pushLog("ack", `alarm ${id}`);
  }

  function renderIntent() {
    const now = Date.now() / 1000;
    const rows = [...intentMap.entries()].sort((a, b) => a[0].localeCompare(b[0]));
    const live = rows.filter(([, e]) => {
      if (e.raw.released) return false;
      const exp = e.raw.expires_at;
      if (typeof exp === "number" && exp < now) return false;
      return true;
    });
    if (!live.length) {
      intentList.innerHTML = '<div id="fleet-empty-intent">No active intents.</div>';
      return;
    }
    intentList.innerHTML = "";
    for (const [id, entry] of live) {
      const it = entry.raw;
      const goal = it.goal || { x: it.x, y: it.y };
      const corridor = it.corridor || [];
      const exp = typeof it.expires_at === "number" ? it.expires_at : now + INTENT_TTL_FALLBACK_S;
      const card = document.createElement("div");
      card.className = "fleet-card fresh";
      const corrStr = corridor.length
        ? corridor.map((p) => `(${fmt(p[0])},${fmt(p[1])})`).join(" → ")
        : "—";
      card.innerHTML =
        `<h4>${escapeHtml(id)} <span class="fleet-mode">pri ${escapeHtml(String(it.priority ?? 0))}</span></h4>` +
        `<dl>` +
        `<dt>goal</dt><dd>x=${fmt(goal.x)} y=${fmt(goal.y)}</dd>` +
        `<dt>corridor</dt><dd>${escapeHtml(corrStr)}</dd>` +
        `<dt>expires</dt><dd>in ${Math.max(0, exp - now).toFixed(0)} s</dd>` +
        `</dl>`;
      intentList.appendChild(card);
    }
  }

  function escapeHtml(s) {
    return String(s)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function setConnLabel(state) {
    connEl.classList.remove("stale", "dead");
    if (state === "ok") {
      connEl.textContent = `rosbridge: ${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`;
    } else if (state === "retry") {
      connEl.textContent = "rosbridge: reconnecting…";
      connEl.classList.add("stale");
    } else {
      connEl.textContent = "rosbridge: disconnected";
      connEl.classList.add("dead");
    }
  }

  function onStatusMsg(msg) {
    let s;
    try {
      s = JSON.parse(msg.data);
    } catch (e) {
      return;
    }
    if (!s.robot_id) return;
    statuses.set(s.robot_id, { raw: s, recvAt: Date.now() });
    const p = s.pose || {};
    pushLog("status", `${s.robot_id} @ (${fmt(p.x)},${fmt(p.y)}) mode=${s.mode || "?"}`);
    if (!viewEl.hidden) renderStatus();
  }

  function onIntentMsg(msg) {
    let it;
    try {
      it = JSON.parse(msg.data);
    } catch (e) {
      return;
    }
    if (!it.robot_id) return;
    if (it.released) {
      intentMap.delete(it.robot_id);
      pushLog("intent", `${it.robot_id} RELEASED`);
    } else {
      // Normalize goal shape for cards that expect .goal
      if (!it.goal && (it.x != null || (it.goal && it.goal.x != null))) {
        /* already fine */
      }
      if (!it.goal && typeof it.x === "number") {
        it.goal = { x: it.x, y: it.y };
      }
      intentMap.set(it.robot_id, { raw: it, recvAt: Date.now() });
      const g = it.goal || {};
      pushLog(
        "intent",
        `${it.robot_id} → (${fmt(g.x)},${fmt(g.y)}) pri=${it.priority ?? 0}` +
          (it.corridor && it.corridor.length ? ` corridor[${it.corridor.length}]` : "")
      );
    }
    if (!viewEl.hidden) renderIntent();
  }

  function onBoardMsg(msg) {
    let b;
    try {
      b = JSON.parse(msg.data);
    } catch (e) {
      return;
    }
    if (!b || !Array.isArray(b.robots) || !Array.isArray(b.tasks)) return;
    board = b;
    boardAt = Date.now();
    for (const a of b.alarms || []) {
      if (!seenAlarms.has(a.id)) {
        seenAlarms.add(a.id);
        pushLog("ALARM", `${a.kind}: ${a.text || ""}`);
      }
    }
    if (!viewEl.hidden) {
      renderStatus();
      renderTasks();
      renderAlarms();
      renderBoardAge();
    }
  }

  const ros = new ROSLIB.Ros({ url: `ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}` });
  ros.on("connection", () => {
    connected = true;
    setConnLabel("ok");
    pushLog("sys", "connected");
  });
  ros.on("close", () => {
    connected = false;
    setConnLabel("retry");
    setTimeout(() => ros.connect(`ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`), 2000);
  });
  ros.on("error", () => {
    connected = false;
    setConnLabel("dead");
  });

  new ROSLIB.Topic({
    ros,
    name: STATUS_TOPIC,
    messageType: "std_msgs/String",
    reconnect_on_close: true,
  }).subscribe(onStatusMsg);

  new ROSLIB.Topic({
    ros,
    name: INTENT_TOPIC,
    messageType: "std_msgs/String",
    reconnect_on_close: true,
  }).subscribe(onIntentMsg);

  // Latched at the gateway, so the current board arrives on subscribe.
  new ROSLIB.Topic({
    ros,
    name: BOARD_TOPIC,
    messageType: "std_msgs/String",
    reconnect_on_close: true,
  }).subscribe(onBoardMsg);

  ackTopic = new ROSLIB.Topic({ ros, name: ACK_TOPIC, messageType: "std_msgs/String" });
  if (typeof ackTopic.advertise === "function") ackTopic.advertise();

  function renderAll() {
    renderStatus();
    renderTasks();
    renderAlarms();
    renderBoardAge();
    renderIntent();
  }

  function tick() {
    if (viewEl.hidden) return;
    // Drop very old status so cards don't linger forever after a crash —
    // unless the board still lists the robot, in which case the fleet, not
    // this page, decides when it is gone (see renderStatus).
    const now = Date.now();
    for (const [id, e] of statuses) {
      if (now - e.recvAt > 15000) statuses.delete(id);
    }
    renderAll();
  }

  // scenarios.js toggles [hidden] on #view-fleet; observe that.
  const obs = new MutationObserver(() => {
    if (!viewEl.hidden) {
      renderAll();
      if (!tickTimer) tickTimer = setInterval(tick, TICK_MS);
    } else if (tickTimer) {
      clearInterval(tickTimer);
      tickTimer = null;
    }
  });
  obs.observe(viewEl, { attributes: true, attributeFilter: ["hidden"] });

  // Expose for scenarios.js / tests. onBoard takes a {data: json} message so
  // a headless test can feed a board without rosbridge.
  window.__fleetComms = {
    statuses,
    intents: intentMap,
    board: () => board,
    onBoard: onBoardMsg,
    connected: () => connected,
  };
})();
