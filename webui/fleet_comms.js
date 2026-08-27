// SortBots dashboard — Fleet mesh tab. Monitors /fleet/status + /fleet/intent.
//
// Own rosbridge connection (same pattern as arm.js): independent of app.js's
// ~15 topic subscriptions, so opening this tab does not require the live
// view's robot picker state. Autonomy nodes use this same radio — no TF
// peer eavesdropping — so what you see here is what explorers hear.
//
// View switching lives in scenarios.js (it already owns #view-mode); this
// file only renders when #view-fleet is shown and keeps a short event log.

(function () {
  const ROSBRIDGE_HOST = window.location.hostname || "localhost";
  const ROSBRIDGE_PORT = 9090;
  const STATUS_TOPIC = "/fleet/status";
  const INTENT_TOPIC = "/fleet/intent";
  const STATUS_FRESH_MS = 1000;
  const STATUS_AGING_MS = 2500;
  const INTENT_TTL_FALLBACK_S = 90;
  const MAX_LOG = 80;
  const TICK_MS = 250;

  const viewEl = document.getElementById("view-fleet");
  const statusList = document.getElementById("fleet-status-list");
  const intentList = document.getElementById("fleet-intent-list");
  const logEl = document.getElementById("fleet-log");
  const connEl = document.getElementById("fleet-conn");
  if (!viewEl || typeof ROSLIB === "undefined") return;

  /** @type {Map<string, {raw: object, recvAt: number}>} */
  const statuses = new Map();
  /** @type {Map<string, {raw: object, recvAt: number}>} */
  const intentMap = new Map();
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

  function renderStatus() {
    const now = Date.now();
    const rows = [...statuses.entries()].sort((a, b) => a[0].localeCompare(b[0]));
    if (!rows.length) {
      statusList.innerHTML = '<div id="fleet-empty-status">No status yet.</div>';
      return;
    }
    statusList.innerHTML = "";
    for (const [id, entry] of rows) {
      const s = entry.raw;
      const age = now - entry.recvAt;
      const pose = s.pose || {};
      const twist = s.twist || {};
      const mode = s.mode || "idle";
      const card = document.createElement("div");
      card.className = `fleet-card ${ageClass(age)}`;
      card.innerHTML =
        `<h4>${escapeHtml(id)} <span class="fleet-mode ${escapeHtml(mode)}">${escapeHtml(mode)}</span></h4>` +
        `<dl>` +
        `<dt>pose</dt><dd>x=${fmt(pose.x)} y=${fmt(pose.y)} yaw=${fmt(pose.yaw)}</dd>` +
        `<dt>twist</dt><dd>vx=${fmt(twist.vx, 3)} wz=${fmt(twist.wz, 3)}</dd>` +
        `<dt>age</dt><dd>${(age / 1000).toFixed(2)} s</dd>` +
        `<dt>stamp</dt><dd>${fmt(s.stamp, 1)}</dd>` +
        `</dl>`;
      statusList.appendChild(card);
    }
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

  function tick() {
    if (viewEl.hidden) return;
    // Drop very old status so cards don't linger forever after a crash.
    const now = Date.now();
    for (const [id, e] of statuses) {
      if (now - e.recvAt > 15000) statuses.delete(id);
    }
    renderStatus();
    renderIntent();
  }

  // scenarios.js toggles [hidden] on #view-fleet; observe that.
  const obs = new MutationObserver(() => {
    if (!viewEl.hidden) {
      renderStatus();
      renderIntent();
      if (!tickTimer) tickTimer = setInterval(tick, TICK_MS);
    } else if (tickTimer) {
      clearInterval(tickTimer);
      tickTimer = null;
    }
  });
  obs.observe(viewEl, { attributes: true, attributeFilter: ["hidden"] });

  // Expose for scenarios.js / tests.
  window.__fleetComms = {
    statuses,
    intents: intentMap,
    connected: () => connected,
  };
})();
