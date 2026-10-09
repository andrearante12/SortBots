"""The magnet effector on the whole SO-101 follower arm.

Loads the upstream full-arm assembly (reference/SO101_Assembly.step, 20 MB,
fetched by setup_cad.sh, gitignored), strips the stock gripper, and bolts our
parts onto motor 5's horn.

Placement is REGISTERED, not hand-typed: the assembly's own fixed-jaw solid is
matched against reference/Wrist_Roll_Follower_SO101.step (same volume, same
inertia; principal axes give the rotation up to sign, and the one sign combo
that lands every vertex on top of the other is the answer — 0.000 mm residual
on 2026-10-05). Our frame IS that part's frame (model.reference_wrist()), so
the same transform places our parts.
"""
import itertools
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.gp import gp_Trsf
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

import model
from params import DEFAULT, Params

ARM_STEP = model.REF / "SO101_Assembly.step"


def load_parts(path: Path = ARM_STEP) -> list:
    """Every leaf of the STEP assembly as (top-level component name, located shape).

    cq.importers.importStep flattens names away; we need them to know which
    solid is the fixed jaw, so walk the XCAF tree ourselves.
    """
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"can't read {path}")
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
            return attr.Get().ToExtString()
        return "?"

    out = []

    def walk(label, loc, top):
        ref = label
        if st.IsReference_s(label):
            ref = TDF_Label()
            st.GetReferredShape_s(label, ref)
        top = top or name(label)
        if st.IsAssembly_s(ref):
            kids = TDF_LabelSequence()
            st.GetComponents_s(ref, kids)
            for i in range(1, kids.Length() + 1):
                k = kids.Value(i)
                walk(k, loc.Multiplied(st.GetLocation_s(k)), top)
            return
        shape = cq.Shape.cast(st.GetShape_s(ref).Moved(loc))
        if shape.Solids():  # wiring holders etc. come through as empty shells
            out.append((top, shape))

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        root = roots.Value(i)
        kids = TDF_LabelSequence()
        st.GetComponents_s(root, kids)
        for j in range(1, kids.Length() + 1):  # skip the root, name by its children
            k = kids.Value(j)
            walk(k, st.GetLocation_s(k), "")
    return out


def _mass_props(shape: cq.Shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, g)
    c = g.CentreOfMass()
    m = g.MatrixOfInertia()
    inertia = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    _, axes = np.linalg.eigh(inertia)
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()]), axes


def register(ref: cq.Shape, target: cq.Shape, tol: float = 0.01) -> gp_Trsf:
    """Rigid transform mapping `ref` onto `target` (same part, different placement)."""
    vr, cr, ar = _mass_props(ref)
    vt, ct, at = _mass_props(target)
    if abs(vr - vt) > 1e-3 * vr:
        raise ValueError(f"not the same part: volume {vr:.0f} vs {vt:.0f} mm3")
    pts = np.array([v.toTuple() for v in ref.Vertices()])
    tgt = np.array([v.toTuple() for v in target.Vertices()])
    best = None
    for signs in itertools.product((1, -1), repeat=3):
        rot = at @ np.diag(signs) @ ar.T
        if np.linalg.det(rot) < 0:
            continue
        moved = pts @ rot.T + (ct - rot @ cr)
        err = max(np.min(np.linalg.norm(tgt - q, axis=1)) for q in moved)
        if best is None or err < best[0]:
            best = (err, rot, ct - rot @ cr)
    err, rot, t = best
    if err > tol:
        raise ValueError(f"registration residual {err:.3f} mm > {tol} — upstream part changed?")
    trsf = gp_Trsf()
    trsf.SetValues(*rot[0], t[0], *rot[1], t[1], *rot[2], t[2])
    return trsf


def _local_z_range(shape: cq.Shape, inv: gp_Trsf) -> tuple:
    bb = shape.moved(cq.Location(inv)).BoundingBox()
    c = bb.center
    return bb.zmin, bb.zmax, c.z, float(np.hypot(c.x, c.y))


def classify(name: str, zc: float, rc: float) -> str:
    """'gripper' = stock gripper hardware, dropped: anything centred beyond motor
    5's horn face (local z>0).
    'horn' = the 4 horn screws + centre screw (r~7 / r~0, z~2): kept and reused,
    and they turn WITH the effector, so the roll sweep must not test against them.
    'arm' = everything else."""
    if name.startswith("Component6"):
        # A 6-face body routed from motor 5 into the jaw; the stock jaw itself
        # overlaps it by 5 mm3, so it's motor 6's cable, and it goes with motor 6.
        return "gripper"
    if rc < 9.0 and -6.0 < zc < 6.0 and "Screw" in name:
        return "horn"
    return "gripper" if zc > 0.5 else "arm"


class Arm:
    """Loaded once; the 20 MB STEP takes ~7 s to read."""

    def __init__(self, path: Path = ARM_STEP):
        parts = load_parts(path)
        jaw = [s for n, s in parts if n.startswith("Wrist_Roll_Follower")]
        if len(jaw) != 1:
            raise RuntimeError(f"expected one Wrist_Roll_Follower in {path.name}, got {len(jaw)}")
        self.trsf = register(model.reference_wrist().val(), jaw[0])
        inv = self.trsf.Inverted()
        self.groups = {"arm": [], "horn": [], "gripper": []}
        for n, s in parts:
            _, _, zc, rc = _local_z_range(s, inv)
            self.groups[classify(n, zc, rc)].append((n, s))
        self.kept = self.groups["arm"] + self.groups["horn"]
        self.removed = self.groups["gripper"]

    def place(self, shape: cq.Shape, roll_deg: float = 0.0) -> cq.Shape:
        """Our frame -> arm frame, optionally rolled about the wrist-roll axis."""
        return shape.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), roll_deg).moved(
            cq.Location(self.trsf))

    def neighbours(self, group: str = "arm") -> dict:
        """Parts of `group` that sit near the effector, merged per component."""
        out = {}
        inv = self.trsf.Inverted()
        for n, s in self.groups[group]:
            zmin, zmax, _, _ = _local_z_range(s, inv)
            if zmax > -15:
                out.setdefault(n, []).append(s)
        return {n: cq.Compound.makeCompound(v) for n, v in out.items()}

    def roll_clearance(self, p: Params = DEFAULT, step: float = 15.0,
                       eps: float = 0.05, angles=None) -> list:
        """[(roll_deg, our part, arm part, overlap mm3)] for every collision.

        The fixed arm is swept over the full roll range; the horn screws turn
        with us, so they're only checked at roll 0. `eps` absorbs OCC noise on
        tangent faces (screw heads sitting in their counterbores read ~0.006).
        """
        ours = {"adapter_body": model.adapter_body(p).val(),
                "plunger": model.plunger(p).val(),
                "magnet": model.magnet(p).val()}
        hits = []

        def check(a, near):
            for on, o in ours.items():
                placed = self.place(o, a)
                for nn, sh in near.items():
                    v = placed.intersect(sh).Volume()
                    if v > eps:
                        hits.append((float(a), on, nn, v))

        check(0.0, self.neighbours("horn"))
        arm = self.neighbours("arm")
        for a in (np.arange(0.0, 360.0, step) if angles is None else angles):
            check(float(a), arm)
        return hits

    def assembly(self, p: Params = DEFAULT, roll_deg: float = 0.0) -> cq.Assembly:
        """so101_magnet_arm
             |- SO101_arm_stock    upstream arm, gripper removed, untouched
             |- magnet_effector    model.effector_assembly(), PLACED by its
                                   group location (not by moving geometry), so
                                   FreeCAD's dragger sits on motor 5's horn
                                   with W along the wrist
        """
        a = cq.Assembly(name="so101_magnet_arm")
        arm = cq.Assembly(name="SO101_arm_stock")
        groups = {}
        for n, s in self.groups["arm"]:
            groups.setdefault(n, []).append(s)
        for n, ss in groups.items():
            dark = "Servo" in n or "Screw" in n or "Nut" in n
            col = cq.Color(0.2, 0.2, 0.2) if dark else cq.Color(0.92, 0.92, 0.9)
            arm.add(cq.Compound.makeCompound(ss), name=_safe(n), color=col)
        a.add(arm)
        # The arm's own horn screws (reused) move into our hardware group,
        # expressed in our frame so they ride along with the effector.
        inv = cq.Location(self.trsf.Inverted())
        horn = cq.Compound.makeCompound([s.moved(inv) for _, s in self.groups["horn"]])
        loc = cq.Location(self.trsf) * cq.Location(cq.Vector(), cq.Vector(0, 0, 1), roll_deg)
        a.add(model.effector_assembly(p, horn=horn), loc=loc)
        return a


def _safe(name: str) -> str:
    # Assembly names become STEP product names and must be unique; ':' and
    # '/' upset some importers.
    return name.replace(":", "_").replace("/", "_")
