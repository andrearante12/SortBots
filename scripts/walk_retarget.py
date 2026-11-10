"""Retarget one stride of Isaac's walk-clip library onto a dressed character.

Isaac/People/Animations/stand_walk_N.skelanim.usd is authored against an
81-joint rig (Isaac/People/Characters/biped_demo — Root/Pelvis/L_UpLeg/...,
Y-up, facing +Z). The dressed characters (male_adult_construction_01_new,
_05_new, F_Business_02, ...) all share a different 101-joint CC-style rig
(RL_BoneRoot/Hip/Pelvis/L_Thigh/..., Z-up, facing -Y), with zero joint names
in common, and the retargeting omni.anim.people would do isn't in this pip
install. Both rigs rest in a T-pose, so a per-bone retarget is enough for a
walk:

  - every mapped target joint gets the source joint's WORLD rotation change
    since its rest pose, carried into the target's frame (rotateX +90 turns
    Y-up/+Z-forward into Z-up/-Y-forward), after a fixed per-bone alignment
    that turns the target's rest bone onto the source's — so a thigh points
    where the source thigh points even where the two T-poses differ a little;
  - unmapped joints (twist/share bones, fingers, face) keep their rest LOCAL
    transform and follow their parent;
  - the hip gets the source pelvis's bob and sway, scaled by hip height.

Pure pxr (no Isaac imports), so it runs, and is checked, outside the sim.
"""

from __future__ import annotations

# Every stand_walk_N clip is a long motion-matching CAPTURE, not an authored
# walk loop: the character wanders a ~3 m circle, turning, stopping and
# changing pace (stand_walk_7: 50 s, root yaw sweeping through 360 deg).
# Looping "the first few seconds" looped a TURN, which read live as walking
# backwards (2026-11-09). Each entry is a single straight stride —
# (start_s, end_s, speed_mps) in the clip's own seconds — found offline by
# searching every clip for the window of 0.8-2.6 s whose end pose best
# matches its start (summed per-joint quaternion distance, root excluded),
# with root speed >= 0.9 m/s and root yaw steady within 12 deg. All walk
# along the root joint's +Z (motion vs facing within 5 deg). stand_walk_5
# has no clean stride (best seam 30x worse, 12 deg of wobble) and
# stand_walk_6 doesn't exist.
WALK_CLIPS = {
    "stand_walk_1": (24.33, 25.67, 1.00),
    "stand_walk_2": (49.13, 50.47, 1.01),
    "stand_walk_3": (28.00, 29.27, 1.16),
    "stand_walk_4": (26.00, 27.37, 1.03),
    "stand_walk_7": (0.20, 1.53, 1.04),
}
WALK_SRC_SKEL = "Isaac/People/Characters/biped_demo/biped_demo_meters.usd"
WALK_SRC_CLIP = "Isaac/People/Animations/{}.skelanim.usd"

# target (CC rig) joint <- source (walk-library rig) joint. Hip, not Pelvis:
# on the CC rig Hip parents both the legs' Pelvis and the spine's Waist, the
# role the source Pelvis plays. Spine3 has no slot; mapping by WORLD rotation
# means Chest -> Spine02 still lands the chest where the source's is.
RETARGET_MAP = {
    "Hip": "Pelvis",
    "Waist": "Spine1", "Spine01": "Spine2", "Spine02": "Chest",
    "NeckTwist01": "Neck1", "NeckTwist02": "Neck2", "Head": "Head",
}
for _s in ("L", "R"):
    RETARGET_MAP.update({
        f"{_s}_Thigh": f"{_s}_UpLeg", f"{_s}_Calf": f"{_s}_LoLeg",
        f"{_s}_Foot": f"{_s}_Ankle", f"{_s}_ToeBase": f"{_s}_Ball",
        f"{_s}_Clavicle": f"{_s}_Clavicle", f"{_s}_Upperarm": f"{_s}_UpArm",
        f"{_s}_Forearm": f"{_s}_LoArm", f"{_s}_Hand": f"{_s}_Wrist",
    })
# The bone each joint's alignment is measured along (joint -> this child).
BONE_CHILD = {}
for _s in ("L", "R"):
    BONE_CHILD.update({
        f"{_s}_Thigh": f"{_s}_Calf", f"{_s}_Calf": f"{_s}_Foot",
        f"{_s}_Foot": f"{_s}_ToeBase", f"{_s}_Clavicle": f"{_s}_Upperarm",
        f"{_s}_Upperarm": f"{_s}_Forearm", f"{_s}_Forearm": f"{_s}_Hand",
    })


def _leaf(token) -> str:
    return str(token).rsplit("/", 1)[-1]


def _rot3(m):
    """The orthonormal rotation part of a Matrix4d, as a Matrix3d."""
    from pxr import Gf

    return Gf.Matrix3d(m.ExtractRotationMatrix()).GetOrthonormalized()


def _skeleton_rest(skel):
    """(joint tokens, Topology, rest local Matrix4ds, rest world Matrix4ds)."""
    from pxr import UsdSkel

    joints = list(skel.GetJointsAttr().Get())
    topo = UsdSkel.Topology(joints)
    rest = list(skel.GetRestTransformsAttr().Get())
    return joints, topo, rest, list(UsdSkel.ConcatJointTransforms(topo, rest))


def retarget_stride(src_skel, src_anim, window, tgt_skel):
    """One straight stride of src_anim (a UsdSkel.Animation on the walk-library
    rig, opened on its OWN stage so time codes are its native frames), mapped
    onto tgt_skel (a UsdSkel.Skeleton on the CC rig).

    Returns (frames, stride_s, seam): frames is a list of (QuatfArray,
    Vec3fArray, Vec3hArray) in tgt_skel's joint order, ready for a
    UsdSkel.Animation's rotations/translations/scales; stride_s the clip
    seconds the stride lasts; seam how far its last pose is from its first.
    """
    from pxr import Gf, UsdSkel, Vt

    s_joints, s_topo, s_rest, s_w0 = _skeleton_rest(src_skel)
    t_joints, t_topo, t_rest, t_w0 = _skeleton_rest(tgt_skel)
    s_idx = {_leaf(j): i for i, j in enumerate(s_joints)}
    t_idx = {_leaf(j): i for i, j in enumerate(t_joints)}
    a_joints = [_leaf(j) for j in src_anim.GetJointsAttr().Get()]
    a_to_s = [s_idx[n] for n in a_joints]
    mapped = {t_idx[t]: s_idx[s] for t, s in RETARGET_MAP.items() if t in t_idx and s in s_idx}
    missing = sorted(set(RETARGET_MAP) - {_leaf(t_joints[i]) for i in mapped})
    if missing:
        raise ValueError(f"target rig lacks {missing}")

    # Source frame -> target frame: rotateX +90, (x,y,z) -> (x,-z,y).
    # Row-vector convention throughout, as Gf's: p_world = p_local * M.
    C = Gf.Matrix3d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0))
    C_inv = C.GetInverse()

    def pos(ws, i):
        return ws[i].ExtractTranslation()

    s_r0 = {i: _rot3(m) for i, m in enumerate(s_w0)}
    t_r0 = {i: _rot3(m) for i, m in enumerate(t_w0)}
    align = {}
    for ti, si in mapped.items():
        child = BONE_CHILD.get(_leaf(t_joints[ti]))
        if child is None:
            align[ti] = Gf.Matrix3d(1.0)
            continue
        tc, sc = t_idx[child], s_idx[RETARGET_MAP[child]]
        t_dir = (pos(t_w0, tc) - pos(t_w0, ti)).GetNormalized()
        s_dir = ((pos(s_w0, sc) - pos(s_w0, si)) * C).GetNormalized()
        align[ti] = Gf.Matrix3d().SetRotate(Gf.Rotation(t_dir, s_dir))

    hip_t, pelvis_s = t_idx["Hip"], s_idx["Pelvis"]
    hip_scale = pos(t_w0, hip_t)[2] / max(1e-6, pos(s_w0, pelvis_s)[1])
    pelvis_rest = pos(s_w0, pelvis_s)
    parents = list(t_topo.GetParentIndices())
    t_rest_trans = [m.ExtractTranslation() for m in t_rest]
    t_rest_rot = [_rot3(m) for m in t_rest]
    s_roots = {i for i, j in enumerate(s_joints) if "/" not in str(j)}

    rot_attr, trans_attr = src_anim.GetRotationsAttr(), src_anim.GetTranslationsAttr()
    times = sorted(rot_attr.GetTimeSamples())
    if not times:
        raise ValueError("clip has no rotation samples")
    tcps = src_anim.GetPrim().GetStage().GetTimeCodesPerSecond()
    a_tc, b_tc = (times[0] + s * tcps for s in window[:2])
    picked = [t for t in times if a_tc <= t < b_tc]
    if len(picked) < 4:
        raise ValueError(f"window {window} holds {len(picked)} frames")

    def source_world(t):
        local = list(s_rest)
        rots, trans = rot_attr.Get(t), trans_attr.Get(t)
        for k, si in enumerate(a_to_s):
            if si in s_roots:
                continue       # root pinned: the capture's wandering, see WALK_CLIPS
            m = Gf.Matrix4d().SetRotate(Gf.Quatd(rots[k]))
            m.SetTranslateOnly(Gf.Vec3d(trans[k]) if trans else s_rest[si].ExtractTranslation())
            local[si] = m
        return list(UsdSkel.ConcatJointTransforms(s_topo, local))

    def pose(t):
        sw = source_world(t)
        world = [None] * len(t_joints)
        rots = Vt.QuatfArray(len(t_joints))
        trans = Vt.Vec3fArray(len(t_joints))
        for k, p in enumerate(parents):
            parent = world[p] if p >= 0 else Gf.Matrix3d(1.0)
            if k in mapped:
                si = mapped[k]
                delta = C_inv * s_r0[si].GetInverse() * _rot3(sw[si]) * C
                world[k] = t_r0[k] * align[k] * delta
                local = (world[k] * parent.GetInverse()).GetOrthonormalized()
            else:
                local = t_rest_rot[k]
                world[k] = local * parent
            rots[k] = Gf.Quatf(local.ExtractRotation().GetQuat())
            tr = Gf.Vec3d(t_rest_trans[k])
            if k == hip_t:
                tr += ((pos(sw, pelvis_s) - pelvis_rest) * C) * hip_scale
            trans[k] = Gf.Vec3f(tr)
        return rots, trans

    scales = Vt.Vec3hArray([Gf.Vec3h(1.0, 1.0, 1.0)] * len(t_joints))
    frames = [(*pose(t), scales) for t in picked]
    a0, a1 = rot_attr.Get(picked[0]), rot_attr.Get(b_tc)
    seam = sum(1.0 - min(1.0, abs(Gf.Dot(Gf.Quatd(x), Gf.Quatd(y)))) for x, y in list(zip(a0, a1))[1:])
    return frames, (b_tc - a_tc) / tcps, seam
