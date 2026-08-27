/* All prose, tables, diagrams and field notes for the docs site.
 *
 * Kept as data in one file so the writing can be edited without touching the
 * renderers. site/tests/site_test.mjs asserts the shape of every entry, and
 * that each `parts` id resolves against site/js/parts.js.
 *
 * Editorial rules:
 *   - `lede` is plain language. No topic names, no constants, no acronyms that
 *     aren't expanded. Someone who has never used ROS should finish it knowing
 *     what the subsystem is for.
 *   - `deep` is where the real interfaces and numbers live, behind a
 *     disclosure. Assume a robotics-literate reader there.
 *   - `notes` are dated field notes: things that actually broke during live
 *     runs, and what the code does now because of it. These are the most
 *     valuable content on the page -- they're why the design is what it is.
 *
 * Architecture source of truth is docs/perception_exploration.md; the runbook
 * is docs/running.md. When those change, change this too.
 */

export const SUBSYSTEMS = [
  {
    id: 'perception',
    ns: '/robot_0/camera/depth_static',
    eyebrow: 'Perception',
    title: 'One camera, two kinds of truth',
    parts: ['head-camera'],
    lede: [
      'No lidar. One depth camera in the head has to both map the warehouse and notice what is moving in front of it.',
      'Movers are erased from the mapper’s copy and published on their own. The map keeps the building; the navigator keeps the traffic.',
    ],
    facts: [
      { v: '1', k: 'depth camera' },
      { v: '0', k: 'lidars' },
      { v: '4.5 m', k: 'usable range' },
      { v: '2', k: 'output streams' },
    ],
    diagram: {
      caption: 'One depth image in, two streams out — each to the consumer that wants that version of reality.',
      nodes: [
        { id: 'cam', label: 'camera/depth', sub: 'RealSense D435', x: 3, y: 38, kind: 'sensor' },
        { id: 'filt', label: 'dynamic_obstacle_filter', sub: 'frame differencing + blob labelling', x: 27, y: 38, kind: 'node' },
        { id: 'static', label: 'camera/depth_static', sub: 'movers erased (NaN)', x: 60, y: 8, kind: 'topic' },
        { id: 'dyn', label: 'dynamic_obstacles', sub: 'movers only, ephemeral', x: 60, y: 62, kind: 'topic' },
        { id: 'rtab', label: 'RTAB-Map', sub: 'builds the map that persists', x: 84, y: 8, kind: 'consumer' },
        { id: 'nav', label: 'Nav2 costmaps', sub: 'clears as soon as they leave', x: 84, y: 62, kind: 'consumer' },
      ],
      edges: [
        { from: 'cam', to: 'filt', kind: 'data' },
        { from: 'filt', to: 'static', kind: 'data' },
        { from: 'filt', to: 'dyn', kind: 'data' },
        { from: 'static', to: 'rtab', kind: 'data' },
        { from: 'dyn', to: 'nav', kind: 'data' },
      ],
    },
    deep: {
      blurb:
        '<code>nodes/dynamic_obstacle_filter.py</code> runs before RTAB-Map in the bringup, so <code>depth_static</code> already exists when mapping starts. It is the <em>only</em> depth source the mapper is given. Every subscription here uses <code>qos_profile_sensor_data</code>.',
      tables: [
        {
          caption: 'Interfaces',
          head: ['Direction', 'Topic', 'Type', 'Purpose'],
          rows: [
            ['sub', '/{id}/camera/depth', 'sensor_msgs/Image', 'raw depth from the head camera'],
            ['sub', '/{id}/camera/camera_info', 'sensor_msgs/CameraInfo', 'pinhole intrinsics for unprojection'],
            ['pub', '/{id}/camera/depth_static', 'sensor_msgs/Image', 'always re-encoded 32FC1, mover pixels NaN'],
            ['pub', '/{id}/dynamic_obstacles', 'sensor_msgs/PointCloud2', 'xyz only; empty cloud published so costmaps can clear'],
          ],
        },
        {
          caption: 'Filter tuning',
          head: ['Key', 'Value', 'What it controls'],
          rows: [
            ['change_m', '0.12', 'depth delta between frames that counts as motion'],
            ['min_range_m / max_range_m', '0.35 / 4.5', 'range band outside which depth is ignored'],
            ['min_blob_px / max_blob_px', '40 / 25000', 'blob sizes kept as real movers'],
            ['morph_close', '2', 'dilate/erode passes that close speckle holes'],
            ['min_height_m / max_height_m', '0.08 / 1.35', 'reserved — no camera-tilt model yet, currently unused'],
          ],
        },
        {
          caption: 'Occupancy grid tuning (RTAB-Map)',
          head: ['Parameter', 'Value', 'Why'],
          rows: [
            ['Grid/RayTracing', 'true', 'without it only cells where a depth point landed are free, and frontier exploration has nothing to chase'],
            ['Grid/3D', 'true', 'makes cloud_map real 3D geometry instead of a 15 cm pancake'],
            ['Grid/RangeMax', '5.0', 'matches the costmap raytrace range'],
            ['GridGlobal/ProbHit / ProbMiss', '0.8 / 0.45', 'reweighted — 3D ray tracing floods free evidence into the projected 2D map'],
            ['Mem/InitWMWithAllNodes', 'true', 'without it a resumed run loads 706 nodes but activates 3, and exploration instantly declares itself done'],
          ],
        },
      ],
    },
    notes: [
      {
        date: '2026-08-09',
        title: 'The blob labeller that blinded both costmaps',
        body:
          'Connected-component labelling started life as a per-pixel flood fill. At 848×480 that is 407,000 interpreted iterations per frame, which throttled <code>depth_static</code> to about 0.3 Hz. The knock-on effect was the expensive part: <code>depth_image_proc</code> keeps a <code>camera_info</code> queue only five messages deep, so it never synced an image/info pair, published no point cloud, and left <em>both</em> costmaps and the collision monitor with no obstacle data at all. Rewritten as row-runs plus union-find, with blob sizes from a single <code>bincount</code> over run lengths.',
      },
      {
        date: '2026-08-01',
        title: 'Free space floods the map when ray tracing is on',
        body:
          'Turning on 3D ray tracing pushed far more "this cell is empty" evidence into the projected 2D grid than the hit/miss probabilities were tuned for. Reweighting <code>ProbHit/ProbMiss</code> from 0.7/0.4 to 0.8/0.45 took occupied cells from 7,361 down to 659 while free space roughly doubled.',
      },
    ],
  },

  {
    id: 'navigation',
    ns: '/robot_0/navigate_to_pose',
    eyebrow: 'Navigation',
    title: 'Turn, then go',
    parts: ['chassis', 'wheels'],
    lede: [
      'The cart can strafe. The navigator will not let it — the only camera points where the robot faces, so travelling sideways would be driving blind.',
      'Two thousand trajectories scored every cycle, against a live map. No map file, no separate localiser.',
    ],
    facts: [
      { v: '2,000', k: 'trajectories/cycle' },
      { v: '2.8 s', k: 'prediction horizon' },
      { v: '20 Hz', k: 'control rate' },
      { v: '0.5 m/s', k: 'top speed' },
    ],
    diagram: {
      caption: 'Velocity flows through four hops before it reaches the wheels. Every one of them is a lifecycle node, and a node left unmanaged silently publishes nothing.',
      nodes: [
        { id: 'bt', label: 'bt_navigator', sub: 'behaviour tree + recoveries', x: 2, y: 40, kind: 'node' },
        { id: 'ctrl', label: 'controller_server', sub: 'MPPI · DiffDrive', x: 24, y: 40, kind: 'node' },
        { id: 'smooth', label: 'velocity_smoother', sub: 'accel limits', x: 47, y: 40, kind: 'node' },
        { id: 'coll', label: 'collision_monitor', sub: 'last-chance stop', x: 68, y: 40, kind: 'node' },
        { id: 'base', label: 'cmd_vel', sub: 'to the wheels', x: 89, y: 40, kind: 'consumer' },
      ],
      edges: [
        { from: 'bt', to: 'ctrl', label: 'path', kind: 'intent' },
        { from: 'ctrl', to: 'smooth', label: 'cmd_vel_nav', kind: 'intent' },
        { from: 'smooth', to: 'coll', label: 'cmd_vel_smoothed', kind: 'intent' },
        { from: 'coll', to: 'base', kind: 'intent' },
      ],
    },
    deep: {
      blurb:
        'Nav2 runs with no <code>map_server</code> and no <code>amcl</code> — RTAB-Map and the fleet map merger own the map between them. Everything is namespaced under the robot id via <code>PushRosNamespace</code>, the only place in the repo that uses real ROS namespacing rather than f-string topic prefixes.',
      tables: [
        {
          caption: 'Controller — MPPI',
          head: ['Parameter', 'Value', 'Note'],
          rows: [
            ['motion_model', 'DiffDrive', 'base is physically holonomic; strafing points the sensor away from travel'],
            ['time_steps × model_dt', '56 × 0.05 s', '2.8 s lookahead'],
            ['batch_size', '2000', 'candidate rollouts per cycle'],
            ['vx_max / vx_min', '0.5 / −0.15 m/s', 'brief reverse allowed for corner escape and BackUp'],
            ['wz_max', '1.0 rad/s', ''],
            ['critics', 'block list, not flow seq', 'a bracketed list with a trailing comma parses in pyyaml but silently loads zero critics in ROS'],
          ],
        },
        {
          caption: 'Costmaps',
          head: ['Layer', 'Setting', 'Why'],
          rows: [
            ['global_costmap', '4 Hz update / 2 Hz publish', 'was 1/1 — cuts obstacle-to-planner latency from 1 s to 250 ms'],
            ['static_layer.map_topic', '/map (absolute)', 'a relative topic resolves into the costmap sub-namespace and silently gets no map'],
            ['obstacle_layer sources', 'depth_scan + dynamic_scan', 'persistent geometry and ephemeral movers, marked separately'],
            ['dynamic_scan persistence', '0.0 s', 'peer marks clear the moment they leave the frustum'],
            ['inflation_radius', '0.65 m', 'the cost critic scores the robot as a point, so inflation is the only thing keeping it off obstacles'],
            ['local_costmap', 'rolling 6×6 m @ 0.05', 'footprint 0.5 × 0.4 m, corner reach 0.320 m'],
          ],
        },
        {
          caption: 'Task dispatch — nodes/task_manager.py',
          head: ['State', 'Leaves on', 'Notes'],
          rows: [
            ['IDLE', 'a task dispatched and the explorer idle', 'stops the explorer first; both drive the same single-goal action server'],
            ['NAV_TO_PICKUP → PICKING', 'arrival, then pick_result', '30 s safety timeout in case nothing is listening'],
            ['NAV_TO_DROPOFF → PLACING', 'arrival, then pick_result', ''],
            ['FAILED', 'nav abort/reject or pick failure', 'returns to IDLE rather than stalling the queue'],
          ],
        },
      ],
    },
    notes: [
      {
        date: '2026-08-09',
        title: 'Nav2 was severed from the robot and still looked healthy',
        body:
          'The collision monitor is the final velocity hop — it republishes <code>cmd_vel_smoothed</code> as <code>cmd_vel</code>. It was missing from the lifecycle manager\'s node list, and an unmanaged Nav2 lifecycle node stays unconfigured forever, publishing nothing. So the robot never moved on a navigation command, while the controller cheerfully logged "passing new path to controller" at 4 Hz all run. What masked it: the recovery behaviours publish <code>cmd_vel</code> directly and unremapped, so recovery spins <em>did</em> visibly rotate the robot. It must be listed, and listed last.',
      },
      {
        date: '2026-08-02',
        title: 'The goal checker, not the progress checker',
        body:
          'Goals kept timing out within a metre of the target. The suspicion was the progress checker; the actual cause was the goal tolerance. The robot settled 0.234 m from a frontier goal against an xy tolerance of 0.15 m, decided it had not arrived, and sat there until the timeout. Tolerances went to 0.25 m in both position and yaw.',
      },
    ],
  },

  {
    id: 'exploration',
    ns: '/robot_0/explore_status',
    eyebrow: 'Exploration',
    title: 'Chasing the edge of the known',
    parts: ['head-pantilt', 'head-camera'],
    lede: [
      'Nobody hands it a map. It drives at the edge of what it has already seen, stands back, and looks.',
      'Most goals never formally arrive — about 85% are abandoned early because the frontier was consumed on the way there. Giving up well is the job.',
    ],
    facts: [
      { v: '~85%', k: 'goals consumed early' },
      { v: '1 Hz', k: 'replan rate' },
      { v: '1.5 m', k: 'goal standoff' },
      { v: '3', k: 'strikes to blacklist' },
    ],
    scrubber: true,
    diagram: {
      caption: 'Every tick either keeps the current goal alive or replaces it. The blacklist is what stops a failure from being retried forever.',
      nodes: [
        { id: 'map', label: '/map', sub: 'fused occupancy grid', x: 2, y: 40, kind: 'topic' },
        { id: 'find', label: 'find frontiers', sub: 'free cell touching unknown', x: 22, y: 40, kind: 'node' },
        { id: 'score', label: 'score + filter', sub: 'size · openness · distance', x: 45, y: 40, kind: 'node' },
        { id: 'goal', label: 'navigate_to_pose', sub: 'standoff goal, facing in', x: 69, y: 12, kind: 'consumer' },
        { id: 'black', label: 'blacklist', sub: 'escalating strikes', x: 69, y: 66, kind: 'node' },
      ],
      edges: [
        { from: 'map', to: 'find', kind: 'data' },
        { from: 'find', to: 'score', kind: 'data' },
        { from: 'score', to: 'goal', label: 'best', kind: 'intent' },
        { from: 'goal', to: 'black', label: 'stuck / timeout', kind: 'intent' },
        { from: 'black', to: 'score', label: 'suppress', kind: 'intent' },
      ],
    },
    deep: {
      blurb:
        '<code>nodes/explorer.py</code> subscribes to the <em>fused</em> <code>/map</code> by default, not its own, so each robot\'s frontier search sees space any robot has mapped. Frontier clustering is 8-connected BFS in pure Python — about 7.5 ms on a 380×746 grid, once per replan tick.',
      tables: [
        {
          caption: 'Goal selection, in priority order',
          head: ['Mode', 'Triggers when', 'Picks'],
          rows: [
            ['Steer lock', 'an operator hint is live', 'candidate nearest the hint, ignoring score'],
            ['Escape', '2 consecutive failures', 'the farthest valid frontier, ignoring the distance cap'],
            ['Backtrack', 'no candidates in range, some beyond it', 'nearest of the far pool'],
            ['Normal', 'otherwise', 'highest scored candidate'],
          ],
        },
        {
          caption: 'Scoring and filtering',
          head: ['Key', 'Value', 'Effect'],
          rows: [
            ['score', 'size · openness^1.0 / dist^1.0', 'classic frontier scoring; alpha was 1.5, which let an 8-cell frontier at 1 m tie a 200-cell one at 8 m'],
            ['min_frontier_cells', '12', '0.60 m of boundary at 5 cm resolution'],
            ['goal_standoff_max_m', '1.5', 'stand back from the frontier and look at it'],
            ['goal_clearance_m', '0.6', 'raised alongside inflation from 0.4 to 0.65'],
            ['max_goal_distance_m', '12.0', 'on a roughly 19 × 37 m map'],
            ['blacklist radius / ttl', '0.5 m / 120 s', 'first strike; grows to 1.5 m and 360 s, then permanent'],
          ],
        },
        {
          caption: 'Status published on /{id}/explore_status',
          head: ['Field', 'Meaning'],
          rows: [
            ['state', 'exploring | stopped | paused | done'],
            ['phase', 'pending | running | done — the startup spin, deliberately not a state value'],
            ['explored_pct', 'fraction of the current grid that is known — not coverage of the building'],
            ['goals_consumed', 'goals abandoned early because the frontier was already mapped'],
            ['blacklist_points[]', 'x, y, radius, strikes, permanent'],
          ],
        },
      ],
    },
    notes: [
      {
        date: '2026-08-02',
        title: 'Every timeout cancelled its own replacement',
        body:
          'The navigation action takes one goal at a time, so sending a new goal already preempts the old one. The explorer was also explicitly cancelling first — and the cancel is processed asynchronously, so it reliably landed <em>after</em> the replacement became active and cancelled that instead, about 20 ms later. Compounding it: a preempted goal\'s late "aborted" result was read against whichever goal was current by the time it arrived, so the explorer blacklisted a target it was actively driving to. Fixed with a generation counter that lets stale callbacks no-op, and by never cancelling a goal that is about to be replaced.',
      },
      {
        date: '2026-08-01',
        title: 'A flat blacklist made finishing structurally impossible',
        body:
          'Blacklist entries used to expire on a fixed timer, so a genuinely unreachable frontier came back every two minutes forever. That burned goal cycles, and worse, it meant there was always one more candidate — the "no frontiers left" counter never incremented, and the run could never declare itself done. Entries now escalate: re-blacklisting near an existing entry bumps a strike count, which grows both the radius and the lifetime, and pins the entry permanently at three strikes.',
      },
      {
        date: null,
        title: 'explored_pct is not coverage',
        body:
          'The number the dashboard shows is the known fraction of the grid the robot has allocated, not of the warehouse. The most complete map this project has built reads 43.8%. Real coverage is computed separately against a reference free area, and only where a reference exists.',
      },
    ],
  },

  {
    id: 'communication',
    ns: '/fleet/status',
    eyebrow: 'Communication',
    title: 'Only what they’re told',
    parts: ['head-pantilt'],
    lede: [
      'Autonomy never looks up a peer on the transform tree. That would work in simulation and be a lie about the real warehouse.',
      'Each robot broadcasts where it is and where it is going. That radio is the only peer information any decision may use; when two intents cross, the lower-priority robot waits.',
    ],
    facts: [
      { v: '5 Hz', k: 'status broadcast' },
      { v: '1 s', k: 'position lifetime' },
      { v: '90 s', k: 'intent lifetime' },
      { v: '0', k: 'peer lookups on /tf' },
    ],
    diagram: {
      caption: 'A shared broadcast bus standing in for a mesh radio. Nothing on it is authoritative — it is what each robot says about itself.',
      nodes: [
        { id: 'r0', label: 'robot_0', sub: 'fleet_radio + explorer', x: 4, y: 8, kind: 'node' },
        { id: 'r1', label: 'robot_1', sub: 'fleet_radio + explorer', x: 4, y: 66, kind: 'node' },
        { id: 'status', label: '/fleet/status', sub: 'BEST_EFFORT · volatile', x: 40, y: 8, kind: 'topic' },
        { id: 'intent', label: '/fleet/intent', sub: 'RELIABLE · transient local', x: 40, y: 66, kind: 'topic' },
        { id: 'yield', label: 'yield decision', sub: 'corridor intersection test', x: 76, y: 37, kind: 'consumer' },
      ],
      edges: [
        { from: 'r0', to: 'status', kind: 'data' },
        { from: 'r1', to: 'status', kind: 'data' },
        { from: 'r0', to: 'intent', kind: 'intent' },
        { from: 'r1', to: 'intent', kind: 'intent' },
        { from: 'status', to: 'yield', kind: 'data' },
        { from: 'intent', to: 'yield', kind: 'intent' },
      ],
    },
    deep: {
      blurb:
        '<code>nodes/fleet_radio.py</code> is both a runnable broadcaster and an importable protocol module — the explorer and the offline tests use the same encode/parse helpers. Topics are deliberately <em>not</em> namespaced: this is a shared bus, standing in for a real radio. Messages are <code>std_msgs/String</code> carrying JSON; the repo defines no custom message types anywhere.',
      code: {
        caption: 'Wire format',
        text: `// /fleet/status — 5 Hz, best effort
{"robot_id": "robot_0",
 "pose":  {"x": -7.15, "y": 11.62, "yaw": 0.31},
 "twist": {"vx": 0.42, "wz": 0.0},
 "mode":  "explore",          // idle | explore | stuck
 "stamp": 1754...}

// /fleet/intent — on change, replayed to late joiners
{"robot_id": "robot_1",
 "goal":     {"x": 2.4, "y": 15.1},
 "corridor": [[1.2, 13.7], [1.9, 14.4]],   // optional polyline
 "priority": 1,                             // lower outranks
 "expires_at": 1754...,
 "released": true}                          // release carries only id + flag`,
      },
      tables: [
        {
          caption: 'Why the two channels differ',
          head: ['Channel', 'Reliability', 'Durability', 'Reasoning'],
          rows: [
            ['/fleet/status', 'BEST_EFFORT', 'volatile', 'a stale pose is worse than none — never resend an old position'],
            ['/fleet/intent', 'RELIABLE', 'transient local', 'a robot joining late must still learn which corridors are claimed'],
          ],
        },
        {
          caption: 'Yield logic',
          head: ['Step', 'Behaviour'],
          rows: [
            ['Priority', 'derived from the robot id; peers ranked below us are ignored entirely'],
            ['Conflict test', 'segment intersection between our start→goal leg and the peer\'s corridor polyline, with a 0.6 m pad'],
            ['Degenerate case', 'a peer intent with no corridor falls back to a disc test around its goal'],
            ['On conflict', 'suppress our own goal for 3 s and re-plan; peers also act as soft keepouts during scoring'],
            ['Expiry', 'double TTL — wall-clock expires_at and monotonic receive age must both hold'],
          ],
        },
      ],
    },
    notes: [
      {
        date: null,
        title: 'The pose oracle we refused to build',
        body:
          'This is the standing invariant of the whole fleet design: autonomy must never look up another robot\'s frame on the transform tree. It would work perfectly in simulation and be unimplementable on real hardware, and it would quietly invalidate every multi-robot result the project produces. Peers are visible in exactly two ways — as anonymous moving obstacles when the depth camera happens to see them, and as self-reported broadcasts on the radio.',
      },
      {
        date: null,
        title: 'This is not multi-agent path planning',
        body:
          'No robot plans around another robot\'s route. Intent creates soft keepouts that bias frontier scoring away from a claimed corridor, and a yield gate that makes the lower-priority robot wait. Actual collision avoidance is local and anonymous: whatever the depth camera sees, the costmap treats as an obstacle, robot or not.',
      },
    ],
  },

  {
    id: 'fusion',
    ns: '/map',
    eyebrow: 'Fleet fusion',
    title: 'Two maps of one warehouse',
    parts: ['chassis'],
    lede: [
      'Each robot maps alone — own database, own pose graph, no loop closure between them.',
      'The grids are pinned together from known spawn poses and merged cell by cell. Occupied beats free, free beats unknown. Collaborative fusion, not collaborative SLAM.',
    ],
    facts: [
      { v: '1 Hz', k: 'grid merge' },
      { v: '200k', k: 'point budget' },
      { v: '5 cm', k: 'voxel size' },
      { v: '0', k: 'shared pose graphs' },
    ],
    diagram: {
      caption: 'Private maps, shared product. The anchors are static transforms built from each robot’s known spawn pose.',
      nodes: [
        { id: 'm0', label: 'robot_0/map', sub: 'own database', x: 3, y: 8, kind: 'topic' },
        { id: 'm1', label: 'robot_1/map', sub: 'own database', x: 3, y: 66, kind: 'topic' },
        { id: 'anchor', label: 'spawn anchors', sub: 'map → {id}/map', x: 31, y: 37, kind: 'node' },
        { id: 'merge', label: 'map_merge', sub: 'occupied > free > unknown', x: 55, y: 37, kind: 'node' },
        { id: 'out', label: '/map', sub: 'fused, frame: map', x: 81, y: 37, kind: 'consumer' },
      ],
      edges: [
        { from: 'm0', to: 'anchor', kind: 'data' },
        { from: 'm1', to: 'anchor', kind: 'data' },
        { from: 'anchor', to: 'merge', kind: 'data' },
        { from: 'merge', to: 'out', kind: 'data' },
      ],
    },
    deep: {
      blurb:
        'Two merge nodes run once per bringup, not once per robot: <code>nodes/map_merge.py</code> for the 2D occupancy grid and <code>nodes/recon_cloud_merge.py</code> for the 3D cloud. Both split their algorithm into a ROS-free core (<code>map_fuse.py</code>, <code>recon_fuse.py</code>) that imports no rclpy, so the fusion maths is unit-testable offline in seconds.',
      code: {
        caption: 'Cell resolution, per output cell',
        text: `tier      = 0 unknown | 1 free | 2 occupied
best_tier = max(tier across contributing robots)

# among robots at the winning tier, prefer the more confident reading:
#   occupied -> highest value, free -> lowest value
score     = value if tier == 2 else -value if tier == 1 else -1000
winner    = argmax(score masked to best_tier)`,
      },
      tables: [
        {
          caption: 'Shared and not shared',
          head: ['Property', 'Shared?', 'Note'],
          rows: [
            ['Occupancy grid', 'yes', 'fused via known spawn anchors'],
            ['3D reconstruction', 'yes', 'voxel-downsampled to a 200k point budget'],
            ['Pose graph', 'no', 'each robot owns its own database and optimisation'],
            ['Inter-robot loop closure', 'no', 'would require a fundamentally different stack'],
            ['Drift correction between robots', 'no', 'anchors are static; map-to-map ICP is noted as future work'],
          ],
        },
      ],
    },
    notes: [
      {
        date: null,
        title: 'Why anchors get their own topic instead of using /tf',
        body:
          'The anchor transforms were originally published on the standard transform topics. The dashboard throttles its transform subscription to one message per 50 ms, and the simulator publishes odometry transforms at around 60 Hz per robot — so a once-per-second anchor essentially never won the throttle window, and the browser silently dropped it forever. Native consumers never saw the problem. Anchors now publish on their own topic, held and replayed, so a browser tab opened at any moment receives every current anchor immediately.',
      },
      {
        date: null,
        title: 'Silent fusion hid a broken camera for a whole run',
        body:
          'The merge used to log nothing when it succeeded. A camera remap on the second robot meant it contributed nothing to the fused map, and the merge dutifully published a perfectly valid single-robot map for an entire run without complaint. It now logs whenever the set of contributing robots or the output extent changes.',
      },
    ],
  },

  {
    id: 'manipulation',
    ns: '/robot_0/arm_joint_cmd',
    eyebrow: 'Manipulation',
    title: 'Six joints, played back',
    parts: ['arm-right', 'arm-left', 'grippers'],
    lede: [
      'Two arms, six joints, replayed from a recording. Getting to the shelf is a navigation problem; the demo does not solve inverse kinematics on the way.',
      'Shoulder at 0.83&nbsp;m, reach about 0.45&nbsp;m; it docks 0.30&nbsp;m short and a little off-centre.',
    ],
    facts: [
      { v: '2', k: 'arms' },
      { v: '6', k: 'joints commanded' },
      { v: '0.83 m', k: 'shoulder height' },
      { v: '~0.45 m', k: 'reach' },
    ],
    deep: {
      blurb:
        '<code>nodes/scripted_pick.py</code> streams recorded poses at 20 Hz through a fixed sequence: stow, approach, descend, close, attach, lift, carry. Pose books live in <code>configs/arm_poses.yaml</code>; the dashboard\'s arm pad emits the same format, round-tripped by an offline test.',
      tables: [
        {
          caption: 'Pick protocol',
          head: ['From', 'Topic', 'Payload'],
          rows: [
            ['task_manager', '/{id}/pick_request', 'attach | detach'],
            ['scripted_pick', '/{id}/arm_joint_cmd', 'JointState — all six joints, in order'],
            ['scripted_pick', '/{id}/pick_cmd', 'attach | detach (simulated weld)'],
            ['scripted_pick', '/{id}/pick_result', 'picked | placed | failed'],
          ],
        },
        {
          caption: 'Station docking',
          head: ['Key', 'Value', 'Derivation'],
          rows: [
            ['deck_height_m', '0.68', 'reach forward and slightly down from a 0.83 m shoulder'],
            ['dock_offset_m', '0.30', 'horizontal reach at that height is √(0.45² − 0.15²) ≈ 0.42 m'],
            ['dock_lateral_m', 'offset +90° from facing', 'the arm is mounted 0.045 m off the chassis centreline'],
            ['stations', 'shelf_a, shelf_b, zone_a, zone_b', 'placed near spawn for short, provable navigation legs'],
          ],
        },
      ],
    },
    notes: [
      {
        date: null,
        title: 'A top-level import that made the node unlaunchable',
        body:
          'The arm node imported PyTorch at module scope, for the inverse kinematics solver. PyTorch only exists in the manipulation project\'s separate conda environment, so in the simulation environment the import failed instantly and <em>no launch file could ever start the node</em>. Nothing was subscribed to pick requests, so every dispatched task sat in the picking state until it timed out, with no error pointing at the cause. The solver now lives behind a flag with lazy imports, and the default path is pure playback.',
      },
      {
        date: null,
        title: 'Joint limits are not enforced by the solver',
        body:
          'The inverse kinematics library accepts joint limits but treats them as a hint, not a constraint — it returned a 4.33 rad solution for a joint whose range ends at 3.45. Every solution is re-checked against the limits on the caller\'s side before it is accepted.',
      },
    ],
  },

  {
    id: 'simulation',
    ns: 'scripts/sim_ctl.sh',
    eyebrow: 'Simulation & infrastructure',
    title: 'The parts that aren’t the robot',
    parts: ['chassis', 'head-camera'],
    lede: [
      'Isaac Sim and ROS&nbsp;2 must never share a shell. The control plane is a script with exit codes; a dashboard drives the same operations.',
      'Tests replay a recorded bag against the interface. Nothing here is pointed at a live sim.',
    ],
    facts: [
      { v: '0.45×', k: 'real-time factor' },
      { v: '3', k: 'exit codes with meaning' },
      { v: '~0 s', k: 'offline test setup' },
      { v: '2', k: 'isolated environments' },
    ],
    deep: {
      blurb:
        'Two independent simulation tracks share no code: the warehouse mobility stack, and a separate manipulation environment. The repository is not a standard robotics workspace — there are no packages, and every node is a loose script launched by path.',
      tables: [
        {
          caption: 'sim_ctl.sh exit codes',
          head: ['Code', 'Meaning'],
          rows: [
            ['0', 'success'],
            ['1', 'command failed'],
            ['3', 'no console running'],
            ['4', 'no session running'],
            ['124', 'timed out waiting for a phase'],
          ],
        },
        {
          caption: 'Environment invariants',
          head: ['Rule', 'Failure it prevents'],
          rows: [
            ['Never source the robotics stack in a simulator shell', 'the simulator\'s bundled Python and the system stack corrupt each other'],
            ['Put the system Python first on PATH', 'conda\'s interpreter is picked up by shebang-launched robotics tools, which then cannot import their bindings'],
            ['Namespaced parameter files need a wildcard node key', 'a bare top-level node name silently applies to nothing'],
            ['Image snapshot requests must use sensor-data QoS', 'reliable image subscribers kill the simulator\'s image writers a couple of minutes in'],
            ['Liveness is a process check, not an exit status', 'the launcher returns as soon as it has spawned the run, not when the run ends'],
          ],
        },
        {
          caption: 'Offline checks',
          head: ['Command', 'Covers'],
          rows: [
            ['node webui/tests/dashboard_test.mjs', 'dashboard against a recorded fixture, five viewports'],
            ['node webui/tests/scenarios_test.mjs', 'scenario tab, with and without control enabled'],
            ['python3 -m pytest tests/', 'fusion maths, protocol parsing, pose books'],
            ['node site/tests/site_test.mjs', 'this site — content shape, part ids, no external requests'],
          ],
        },
      ],
    },
    notes: [
      {
        date: null,
        title: 'Reliable image subscribers kill the simulator',
        body:
          'Repeatedly subscribing to camera topics with reliable delivery — which is the default, and which an ad-hoc command-line subscription will happily do — silently kills the simulator\'s image writers about two minutes into a run. The cameras simply stop, with no error. Every snapshot request must ask for sensor-data quality of service explicitly. This one was bisected live.',
      },
      {
        date: null,
        title: 'Measure speed in simulated time',
        body:
          'The simulation runs at roughly 0.45× real time, and not consistently. Every speed and duration figure in this project is measured against the simulated clock; a wall-clock stopwatch reports numbers that are wrong by more than a factor of two.',
      },
    ],
  },
];

export const HERO = {
  title: 'SortBots',
  tagline: 'A fleet of warehouse robots that map a building they have never seen, agree on what they found, and stay out of each other’s way.',
  meta: [
    { k: 'Platform', v: 'XLeRobot' },
    { k: 'Simulator', v: 'Isaac Sim' },
    { k: 'Middleware', v: 'ROS 2 Jazzy' },
    { k: 'Sensing', v: 'One depth camera' },
  ],
};
