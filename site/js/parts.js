/* Robot part groups: glTF node names -> clickable anatomy hotspots.
 *
 * The node names come straight out of site/build/urdf_to_glb.py, which encodes
 * them as `{link}__{visual}` (plus a trailing index where one link carries two
 * visuals with the same name). If the GLB is rebuilt and a name here stops
 * matching, site/tests/site_test.mjs fails rather than the page silently
 * losing a hotspot.
 *
 * Two things about this robot that shape the list:
 *   - `head_camera_link` has NO geometry of its own. The depth camera's body is
 *     baked into head_tilt_link's two meshes, so the "camera" hotspot targets
 *     those. There is nothing else to point at.
 *   - The wheels are separate visuals on base_link rather than separate links,
 *     which is why they are addressable at all.
 */

export const PART_GROUPS = {
  'head-camera': {
    label: 'Depth camera',
    detail: 'RealSense D435 — the only exteroceptive sensor',
    nodes: ['head_tilt_link__top_head_3', 'head_tilt_link__top_head_3__1'],
  },
  'head-pantilt': {
    label: 'Head and mast',
    detail: 'Pan and tilt, aiming the one camera',
    nodes: [
      'head_pan_link__top_head_1',
      'head_pan_link__top_head_2',
      'top_base_link__top_base_link_visual1',
      'top_base_link__top_base_link_visual2',
    ],
  },
  chassis: {
    label: 'Chassis',
    detail: 'The cart everything else rides on',
    nodes: ['base_link__raskog'],
  },
  wheels: {
    label: 'Omni wheels',
    detail: 'Can strafe; the navigator will not',
    nodes: ['base_link__raskogwheel1', 'base_link__raskogwheel2'],
  },
  'arm-right': {
    label: 'Right arm',
    detail: 'Five joints plus a gripper',
    nodes: [
      'Base__base_link_chassis', 'Base__base_link_motor',
      'Rotation_Pitch__rotation_pitch_chassis', 'Rotation_Pitch__rotation_pitch_motor',
      'Upper_Arm__upper_arm_chassis', 'Upper_Arm__upper_arm_motor',
      'Lower_Arm__lower_arm_chassis', 'Lower_Arm__lower_arm_motor',
      'Wrist_Pitch_Roll__wrist_pitch_roll_chassis', 'Wrist_Pitch_Roll__wrist_pitch_roll_motor',
      'Fixed_Jaw__fixed_jaw_chassis', 'Fixed_Jaw__fixed_jaw_motor',
      'Moving_Jaw__moving_jaw_chassis',
      'Right_Arm_Camera__Right_Arm_Camera_visual', 'Right_Arm_Camera__Right_Arm_Camera_visual_2',
    ],
  },
  'arm-left': {
    label: 'Left arm',
    detail: 'Mirror of the right, same limits',
    nodes: [
      'Base_2__base_link_chassis', 'Base_2__base_link_motor',
      'Rotation_Pitch_2__rotation_pitch_chassis', 'Rotation_Pitch_2__rotation_pitch_motor',
      'Upper_Arm_2__upper_arm_chassis', 'Upper_Arm_2__upper_arm_motor',
      'Lower_Arm_2__lower_arm_chassis', 'Lower_Arm_2__lower_arm_motor',
      'Wrist_Pitch_Roll_2__wrist_pitch_roll_chassis', 'Wrist_Pitch_Roll_2__wrist_pitch_roll_motor',
      'Fixed_Jaw_2__fixed_jaw_chassis', 'Fixed_Jaw_2__fixed_jaw_motor',
      'Moving_Jaw_2__moving_jaw_chassis',
      'Left_Arm_Camera__Left_Arm_Camera_visual', 'Left_Arm_Camera__Left_Arm_Camera_visual_2',
    ],
  },
  grippers: {
    label: 'Grippers',
    detail: 'One driven jaw each; closing is a recorded joint position',
    nodes: [
      'Fixed_Jaw__fixed_jaw_chassis', 'Moving_Jaw__moving_jaw_chassis',
      'Fixed_Jaw_2__fixed_jaw_chassis', 'Moving_Jaw_2__moving_jaw_chassis',
    ],
  },
};

/* The anatomy section's clickable dots. Deliberately a subset of PART_GROUPS:
 * `grippers` overlaps both arms, so it is addressable by a subsystem section
 * but is not its own dot -- two overlapping hotspots would fight over the same
 * meshes under the cursor. Order is top-of-robot down, which is also DOM and
 * tab order. */
/* Labels open to the right of the dot. The stage is the right half of the
 * page, so opening left would point them at the copy. */
export const HOTSPOTS = [
  { part: 'head-camera', subsystem: 'perception' },
  { part: 'head-pantilt', subsystem: 'exploration' },
  { part: 'arm-right', subsystem: 'manipulation' },
  { part: 'arm-left', subsystem: 'manipulation' },
  { part: 'chassis', subsystem: 'navigation' },
  { part: 'wheels', subsystem: 'navigation' },
];

/** Reverse index: glTF node name -> part id. Built once at module load. */
export const NODE_TO_PART = (() => {
  const map = new Map();
  for (const [id, group] of Object.entries(PART_GROUPS)) {
    // `grippers` is intentionally not in this index: its nodes belong to the
    // arms, and hover picking must resolve them to the arm that owns them.
    if (id === 'grippers') continue;
    for (const node of group.nodes) map.set(node, id);
  }
  return map;
})();
