/* Manual arm teleop pad + pose capture.
 *
 * This is the instrument that produces the hardcoded numbers in
 * configs/arm_poses.yaml: jog the arm until the gripper is where you want it,
 * hit `cap`, and paste the emitted YAML block into a `sequences:` list. There
 * is no IK anywhere in this path — the captured values ARE the recorded joint
 * angles, which is exactly why nodes/scripted_pick.py can play them back
 * without torch.
 *
 * Wire contract (spawn_warehouse.py:_read_arm_cmd enforces it):
 *   /<robot>/arm_joint_cmd  sensor_msgs/JointState
 *     name:     all six of ARM_JOINT_NAMES, in order, ALWAYS — _read_arm_cmd
 *               returns None and silently drops the command if any is missing.
 *     position: six floats, radians, within the URDF limits below.
 *
 * SELF-MOUNTING, and deliberately decoupled from app.js: it injects its own
 * <style>, builds its own DOM under #stage-panel, re-derives ROBOT_ID, and
 * opens its own rosbridge connection. app.js's `ros` and `ROBOT_ID` are
 * top-level consts, not window properties, so they aren't reachable anyway —
 * but the bigger win is that this file needed a ONE-LINE diff to index.html
 * and zero lines of app.js while two other branches were editing both. Fold
 * the CSS into index.html's style block once subsystem/manipulation merges.
 *
 * Open-loop, like the head-aim pad it's modelled on (app.js:227-305): nothing
 * publishes /joint_states, so the commanded pose is held client-side. The
 * starting all-zeros is not a guess — configs/physics_overrides/xlerobot.json
 * sets target_value: 0.0 for every arm joint, so that IS the pose at t=0.
 */
(() => {
  "use strict";

  // Order is the contract with spawn_warehouse.ARM_JOINT_NAMES. Limits are
  // from third_party/XLeRobot/.../xlerobot.urdf, duplicated (rather than
  // fetched) so the pad clamps correctly with nothing else running.
  const JOINTS = [
    { name: "Rotation",    label: "Rot",     lo: -2.1,     hi: 2.1 },
    { name: "Pitch",       label: "Pitch",   lo: -0.1,     hi: 3.45 },
    { name: "Elbow",       label: "Elbow",   lo: -0.2,     hi: 3.14159 },
    { name: "Wrist_Pitch", label: "W.Pit",   lo: -1.8,     hi: 1.8 },
    { name: "Wrist_Roll",  label: "W.Rol",   lo: -3.14159, hi: 3.14159 },
    { name: "Jaw",         label: "Jaw",     lo: 0.0,      hi: 1.7 },
  ];
  const JOINT_NAMES = JOINTS.map((j) => j.name);

  const STEP_RAD = 0.05;
  const JOG_HZ = 10;
  // Hold-to-accelerate. A flat 0.05 rad/tick at 10 Hz is 0.5 rad/s, which is
  // right for nudging the gripper onto a box but means ~13 s of held button to
  // sweep Wrist_Roll across its full +/-pi range. Ramping after RAMP_TICKS
  // keeps taps fine-grained (the first half-second is always 1x) while making
  // a full-range move a couple of seconds.
  const RAMP_TICKS = 5;
  const MAX_MULT = 6;
  const ROSBRIDGE_PORT = 9090;

  const ROBOT_ID =
    new URLSearchParams(window.location.search).get("robot") || "robot_0";
  const STORE_KEY = `sortbots.armCaptures.${ROBOT_ID}`;

  const stagePanel = document.getElementById("stage-panel");
  const stageMode = document.getElementById("stage-mode");
  if (!stagePanel || !stageMode || typeof ROSLIB === "undefined") return;

  // -- styles ------------------------------------------------------------
  // Injected rather than added to index.html's <style> block — see the module
  // comment. `!important` on the display rules is load-bearing: app.js's
  // setStageMode writes `aimOverlay.style.display` INLINE, and an inline style
  // beats a plain rule. !important wins in both directions, so the head/arm
  // toggle survives every stage-mode change without arm.js having to observe
  // or re-apply anything.
  const style = document.createElement("style");
  style.textContent = `
  #arm-overlay { right: 12px; width: 150px; }
  #stage-panel:not(.arm-mode) #arm-overlay { display: none !important; }
  #stage-panel.arm-mode #aim-overlay { display: none !important; }
  #stage-panel.stage-is-map #arm-overlay { display: none !important; }
  #arm-toggle {
    position: absolute; right: 12px; z-index: 11;
    display: flex; gap: 2px; opacity: 0.4; transition: opacity 0.15s;
  }
  #stage-panel:hover #arm-toggle, #arm-toggle:hover, #arm-toggle:focus-within { opacity: 1; }
  #stage-panel.stage-is-map #arm-toggle { display: none; }
  #arm-toggle button {
    background: rgba(20,22,26,0.75); color: #9aa;
    border: 1px solid rgba(120,130,145,0.3);
    border-radius: 5px; font-size: 0.65rem; padding: 2px 8px;
    cursor: pointer; user-select: none; text-transform: uppercase;
    letter-spacing: 0.05em;
  }
  #arm-toggle button.active { background: #2d5; color: #111; border-color: #2d5; }
  #arm-rows { display: grid; grid-template-columns: auto 22px 1fr 22px; gap: 3px 4px; align-items: center; }
  #arm-rows .arm-label { font-size: 0.62rem; color: #9aa; white-space: nowrap; }
  #arm-rows .arm-val {
    font-family: monospace; font-size: 0.68rem; color: #7bf; text-align: center;
  }
  #arm-rows .arm-val.at-limit { color: #fb7; }
  #arm-rows button {
    background: rgba(20,22,26,0.75); color: #e6e6e6;
    border: 1px solid rgba(120,130,145,0.3);
    border-radius: 4px; font-size: 0.7rem; line-height: 1; padding: 3px 0;
    cursor: pointer; user-select: none;
  }
  #arm-rows button:active, #arm-rows button.held { background: #2d5; color: #111; }
  #arm-actions { display: flex; gap: 4px; margin-top: 6px; }
  #arm-actions button {
    flex: 1; background: rgba(20,22,26,0.75); color: #e6e6e6;
    border: 1px solid rgba(120,130,145,0.3);
    border-radius: 4px; font-size: 0.62rem; padding: 3px 0;
    cursor: pointer; user-select: none;
  }
  #arm-actions button:active { background: #2d5; color: #111; }
  #arm-capture-name {
    width: 100%; margin-top: 5px; box-sizing: border-box;
    background: rgba(20,22,26,0.75); color: #e6e6e6;
    border: 1px solid rgba(120,130,145,0.3); border-radius: 4px;
    font-size: 0.65rem; font-family: monospace; padding: 3px 5px;
  }
  #arm-captures {
    width: 100%; margin-top: 5px; box-sizing: border-box; resize: vertical;
    background: rgba(20,22,26,0.85); color: #2d5;
    border: 1px solid rgba(120,130,145,0.3); border-radius: 4px;
    font-size: 0.6rem; font-family: monospace; padding: 4px; height: 74px;
    white-space: pre; overflow-x: auto;
  }
  #arm-captures:empty, #arm-captures.empty { display: none; }
  #arm-status { font-size: 0.6rem; font-family: monospace; color: #9aa; margin-top: 4px; }
  @media (max-width: 560px) {
    #arm-overlay { width: 132px; }
    #arm-rows { grid-template-columns: auto 20px 1fr 20px; }
  }`;
  document.head.appendChild(style);

  // -- DOM ---------------------------------------------------------------

  const toggle = document.createElement("div");
  toggle.id = "arm-toggle";
  toggle.innerHTML = `
    <button type="button" data-pad="head" class="active" title="Head pan/tilt pad">head</button>
    <button type="button" data-pad="arm" title="Arm teleop pad">arm</button>`;
  stagePanel.appendChild(toggle);

  const overlay = document.createElement("div");
  overlay.className = "pad-overlay";
  overlay.id = "arm-overlay";
  overlay.innerHTML =
    `<h3>Arm</h3>
     <div id="arm-rows" title="Jog the arm one joint at a time and capture the pose.
Publishes straight to arm_joint_cmd — scripted_pick.py publishes the same topic, so don't jog while a task is picking, it will fight you for it."></div>
     <div id="arm-actions">
       <button type="button" id="arm-home" title="All joints to 0 (the pose the sim starts in)">home</button>
       <button type="button" id="arm-capture" title="Append the current pose to the capture log">cap</button>
       <button type="button" id="arm-copy" title="Copy the capture log to the clipboard">copy</button>
     </div>
     <input type="text" id="arm-capture-name" value="pose_1" spellcheck="false"
            title="Name for the next captured pose" />
     <textarea id="arm-captures" readonly spellcheck="false" class="empty"
               title="Paste this under a sequences: key in configs/arm_poses.yaml"></textarea>
     <div id="arm-status">idle</div>`;
  stagePanel.appendChild(overlay);

  const rowsEl = overlay.querySelector("#arm-rows");
  const statusEl = overlay.querySelector("#arm-status");
  const nameEl = overlay.querySelector("#arm-capture-name");
  const capturesEl = overlay.querySelector("#arm-captures");

  const valEls = [];
  JOINTS.forEach((j, i) => {
    const label = document.createElement("span");
    label.className = "arm-label";
    label.textContent = j.label;

    const dec = document.createElement("button");
    dec.type = "button";
    dec.dataset.jog = `${i}:-1`;
    dec.title = `${j.name} − (limit ${j.lo})`;
    dec.innerHTML = "&#9664;";

    const val = document.createElement("span");
    val.className = "arm-val";
    val.dataset.joint = j.name;

    const inc = document.createElement("button");
    inc.type = "button";
    inc.dataset.jog = `${i}:1`;
    inc.title = `${j.name} + (limit ${j.hi})`;
    inc.innerHTML = "&#9654;";

    rowsEl.append(label, dec, val, inc);
    valEls.push(val);
  });

  // -- ROS ---------------------------------------------------------------

  const ros = new ROSLIB.Ros({
    url: `ws://${window.location.hostname || "localhost"}:${ROSBRIDGE_PORT}`,
  });
  ros.on("error", () => {});
  ros.on("close", () => setTimeout(() => {
    try { ros.connect(`ws://${window.location.hostname || "localhost"}:${ROSBRIDGE_PORT}`); } catch (e) { /* retried on the next close */ }
  }, 2000));

  const armCmdTopic = new ROSLIB.Topic({
    ros,
    name: `/${ROBOT_ID}/arm_joint_cmd`,
    messageType: "sensor_msgs/JointState",
    reconnect_on_close: true,
  });

  // -- state -------------------------------------------------------------

  const armState = JOINTS.map(() => 0.0);
  const held = new Map();   // "<idx>:<dir>" -> ticks held, for the ramp
  let jogTimer = null;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  function render() {
    JOINTS.forEach((j, i) => {
      valEls[i].textContent = armState[i].toFixed(2);
      const atLimit = armState[i] <= j.lo + 1e-6 || armState[i] >= j.hi - 1e-6;
      valEls[i].classList.toggle("at-limit", atLimit);
    });
  }

  function publish() {
    armCmdTopic.publish(new ROSLIB.Message({
      name: JOINT_NAMES,
      position: armState.slice(),
    }));
    render();
  }

  // Resync from the wire. There is no /joint_states, but scripted_pick.py
  // publishes the same topic we do, so echoing it back is the only feedback
  // channel available — without it, jogging right after a scripted pick would
  // snap the arm back to this pad's stale idea of the pose. Ignored while a
  // button is held so a round-tripped echo of our own publish can't stutter an
  // in-progress jog.
  armCmdTopic.subscribe((msg) => {
    if (held.size || !msg || !msg.name || !msg.position) return;
    const byName = new Map(msg.name.map((n, i) => [n, msg.position[i]]));
    if (!JOINT_NAMES.every((n) => typeof byName.get(n) === "number")) return;
    JOINT_NAMES.forEach((n, i) => { armState[i] = byName.get(n); });
    render();
    statusEl.textContent = "synced";
  });

  // -- jogging -----------------------------------------------------------

  function tick() {
    for (const [key, ticks] of held) {
      const [idx, dir] = key.split(":").map(Number);
      const j = JOINTS[idx];
      const mult = Math.min(MAX_MULT, 1 + Math.floor(ticks / RAMP_TICKS));
      armState[idx] = clamp(armState[idx] + dir * STEP_RAD * mult, j.lo, j.hi);
      held.set(key, ticks + 1);
    }
    if (held.size) {
      publish();
      statusEl.textContent = "jogging";
    }
  }

  function startJog(key) {
    if (held.has(key)) return;
    held.set(key, 0);
    rowsEl.querySelector(`[data-jog="${key}"]`)?.classList.add("held");
    tick();
    if (!jogTimer) jogTimer = setInterval(tick, 1000 / JOG_HZ);
  }

  function stopJog(key) {
    if (!held.delete(key)) return;
    rowsEl.querySelector(`[data-jog="${key}"]`)?.classList.remove("held");
    if (held.size === 0 && jogTimer) {
      clearInterval(jogTimer);
      jogTimer = null;
      statusEl.textContent = "idle";
    }
  }

  for (const btn of rowsEl.querySelectorAll("button[data-jog]")) {
    const key = btn.dataset.jog;
    btn.addEventListener("mousedown", () => startJog(key));
    btn.addEventListener("mouseup", () => stopJog(key));
    btn.addEventListener("mouseleave", () => stopJog(key));
    btn.addEventListener("touchstart", (e) => { e.preventDefault(); startJog(key); });
    btn.addEventListener("touchend", (e) => { e.preventDefault(); stopJog(key); });
  }

  // A jog left running because the tab lost focus mid-press would keep
  // driving the arm into a limit unseen — same safety net app.js keeps for
  // the drive keys.
  const releaseAll = () => { for (const key of [...held.keys()]) stopJog(key); };
  window.addEventListener("blur", releaseAll);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) releaseAll();
  });

  overlay.querySelector("#arm-home").addEventListener("click", () => {
    releaseAll();
    JOINTS.forEach((j, i) => { armState[i] = clamp(0, j.lo, j.hi); });
    publish();
    statusEl.textContent = "home";
  });

  // -- capture log -------------------------------------------------------
  // The emitted format must parse as-is via nodes/arm_poses.load_pose_book —
  // webui/tests/arm_test.mjs round-trips this text through that loader, which
  // is what keeps the two in sync.

  let captures = [];
  try {
    const saved = JSON.parse(window.localStorage.getItem(STORE_KEY) || "[]");
    if (Array.isArray(saved)) captures = saved;
  } catch (e) { /* corrupt entry; start clean */ }

  function formatCapture(entry) {
    return `    - {name: ${entry.name}, q: [${entry.q.map((v) => v.toFixed(3)).join(", ")}]}`;
  }

  function renderCaptures() {
    if (!captures.length) {
      capturesEl.value = "";
      capturesEl.classList.add("empty");
      return;
    }
    const header =
      `    # captured ${captures[0].at} ${ROBOT_ID} [${JOINT_NAMES.join(",")}]`;
    capturesEl.value = [header, ...captures.map(formatCapture)].join("\n");
    capturesEl.classList.remove("empty");
    try { window.localStorage.setItem(STORE_KEY, JSON.stringify(captures)); } catch (e) { /* quota/private mode */ }
  }

  // `pose_1` -> `pose_2`, but also `approach` -> `approach2`, so capturing a
  // run of named poses never silently reuses the previous name.
  function nextName() {
    const base = nameEl.value.trim();
    if (!base) return "";
    const m = /^(.*?)(\d+)$/.exec(base);
    return m ? `${m[1]}${Number(m[2]) + 1}` : `${base}2`;
  }

  overlay.querySelector("#arm-capture").addEventListener("click", () => {
    const name = nameEl.value.trim() || `pose_${captures.length + 1}`;
    captures.push({ name, q: armState.slice(), at: new Date().toISOString() });
    renderCaptures();
    const next = nextName();
    if (next) nameEl.value = next;
    statusEl.textContent = `captured ${name}`;
  });

  overlay.querySelector("#arm-copy").addEventListener("click", async () => {
    if (!captures.length) { statusEl.textContent = "nothing captured"; return; }
    capturesEl.classList.remove("empty");
    // navigator.clipboard is UNDEFINED here whenever the dashboard is reached
    // over the tailnet: it's served plain http:// to a non-localhost origin,
    // which is not a secure context. The execCommand fallback is the path that
    // actually runs in normal use, not a defensive afterthought.
    capturesEl.select();
    try {
      if (navigator.clipboard && window.isSecureContext) {
        await navigator.clipboard.writeText(capturesEl.value);
        statusEl.textContent = "copied";
        return;
      }
      statusEl.textContent = document.execCommand("copy") ? "copied" : "select & copy";
    } catch (e) {
      statusEl.textContent = "select & copy";
    }
  });

  // Long-press / double-click to clear: a plain button would be a fourth
  // action in a row already at the overlay's width budget.
  capturesEl.addEventListener("dblclick", () => {
    captures = [];
    renderCaptures();
    try { window.localStorage.removeItem(STORE_KEY); } catch (e) { /* ignore */ }
    statusEl.textContent = "log cleared";
  });

  renderCaptures();

  // -- head/arm switching ------------------------------------------------
  // Map mode is read off #stage-mode's .active class rather than tracked
  // independently: app.js owns stage mode, and every path into it (the
  // segmented buttons, clicking the pip camera, the initial setStageMode call)
  // goes through the same class toggle. Observing it means arm.js never has to
  // hook setStageMode or duplicate its logic.
  function syncStageClass() {
    const mapBtn = stageMode.querySelector('button[data-stage="map"]');
    stagePanel.classList.toggle("stage-is-map", !!mapBtn?.classList.contains("active"));
  }
  new MutationObserver(syncStageClass).observe(stageMode, {
    subtree: true, attributes: true, attributeFilter: ["class"],
  });
  syncStageClass();

  for (const btn of toggle.querySelectorAll("button[data-pad]")) {
    btn.addEventListener("click", () => {
      const armMode = btn.dataset.pad === "arm";
      stagePanel.classList.toggle("arm-mode", armMode);
      for (const b of toggle.querySelectorAll("button[data-pad]")) {
        b.classList.toggle("active", b === btn);
      }
      if (armMode) releaseAll();
    });
  }

  // The toggle chip sits directly above whichever corner pad is showing. Both
  // pads are bottom-anchored and differ in height, so measure rather than
  // guess — and remeasure when the pad's own content (the capture log) grows.
  function placeToggle() {
    const pad = stagePanel.classList.contains("arm-mode")
      ? overlay
      : document.getElementById("aim-overlay");
    if (!pad) return;
    toggle.style.bottom = `${pad.offsetHeight + 18}px`;
  }
  new ResizeObserver(placeToggle).observe(overlay);
  toggle.addEventListener("click", placeToggle);
  placeToggle();

  render();
})();
