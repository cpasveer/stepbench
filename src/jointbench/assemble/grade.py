"""Grading a level 4a (assembles) answer: two parts, their joint, the way they go in, the load.

Everything is the existing grader, applied part by part, plus four checks of
the assembly as a whole:

  part1 ... / part2 ...  each part judged as a level 3 answer is judged: to
              the one fixed part it bolts to, and to the other part, which
              it sees as one more fixed part with the joint bolts in it
              (HOLES, SEATING, SEAT_INSIDE, NO_INTERFERENCE, CLEAR_PATH,
              TOOL_ACCESS, CLEAN, MANUFACTURING). The fixed part it does not
              bolt to counts as an obstacle.
  ASSEMBLY_FILE  the assembly file is complete and consistent.
  INSERTION   each part, in the stated order, comes in along its stated
              direction from outside the scene without running into a fixed
              part, an obstacle, the part already in place or that part's
              bolt heads (``sweep``).
  JOINT_ROOM  the joint's heads and nuts have room clear of the fixed parts
              and obstacles, and the two parts touch only at the joint face.
  JOINT_BOLTS no joint bolt carries more than its catalogue capacity under
              the stated load, in whichever direction it pulls (``joint``).
  STIFFNESS   the two parts analysed together, joined at the joint face,
              meet the stated load's stiffness (level 3's check, unchanged).

Deterministic: the same files give the same verdict and the same numbers.
"""
from __future__ import annotations

import os
import tempfile

import build123d as bd
import numpy as np

from ..catalogue import BOLTS, NUTS, nut_side
from ..checks.base import CheckResult, Context, area, volume
from ..geometry import Placement, read_step, write_step
from ..grade import Report, grade as grade_part
from ..task import ResolvedBolt, Task
from . import joint as J
from .sweep import collisions, describe, path_collisions

ASSEMBLY_FILE = "assembly.json"
CONTACT_ELSEWHERE_MAX = 0.5  # mm2: touching outside the joint face beyond this is touching (numerical noise below)


class _SolidObstacle:
    """A fixed part the part being judged does not bolt to: to that part, an obstacle."""

    def __init__(self, name, solid):
        self.name, self._s = name, solid

    def solid(self):
        return self._s


def subtask(task: Task, asm: J.Assembly, k: int, solids: list) -> Task:
    """Part ``k``'s own judgement: its fixed part, and the other part as a
    fixed part at the joint (see ``joint.joint_interface``)."""
    me = asm.parts[k]
    itf, pl = J.joint_interface(asm, k, solids[1 - k], Placement)
    parts = {me.bolts_to: task.parts[me.bolts_to], J.JOINT: itf}
    pose = {me.bolts_to: task.poses[0][me.bolts_to], J.JOINT: pl}
    obstacles = list(task.obstacles_at(0)) + [_SolidObstacle(f"{n} (fixed part)", s)
                                              for n, s in task.fixed_solids(0).items() if n != me.bolts_to]
    tol = dict(task.tolerances)
    tol["required_checks"] = [c for c in tol.get("required_checks", []) if c != "STIFFNESS"]
    t = Task(f"{task.id}/{me.name}", 3, task.seed, parts, [pose], tol, {}, [obstacles], task.generator_version)
    t.root = task.root
    return t


def _grips(rep: Report) -> dict:
    hol = next((c for c in rep.checks if c.name.split(" ")[-1] == "HOLES"), None)  # named "part1 HOLES" once reported
    return {r["bolt"]: r["grip"] for r in ((hol.details or {}).get("holes") or [])} if hol else {}


def grade(task: Task, assembly_path: str, *, stiffness: bool = True) -> Report:
    """Grade the answer whose assembly file is ``assembly_path`` (its part
    files are found next to it)."""
    checks: list[CheckResult] = []
    folder = os.path.dirname(os.path.abspath(assembly_path))
    try:
        asm = J.load(assembly_path, list(task.parts), task.tolerances.get("joint"))
    except J.AssemblyError as e:
        checks.append(CheckResult("ASSEMBLY_FILE", False, str(e)))
        return Report(task.id, False, checks)
    checks.append(CheckResult("ASSEMBLY_FILE", True,
                              f"{asm.parts[0].name} bolts to {asm.parts[0].bolts_to}, {asm.parts[1].name} to "
                              f"{asm.parts[1].bolts_to}; joint: {len(asm.joint.positions)} x {asm.joint.bolt}, "
                              f"heads on {asm.joint.head_on}"))
    paths = [os.path.join(folder, p.file) for p in asm.parts]
    solids = []
    for p, path in zip(asm.parts, paths):
        try:
            s = read_step(path).solids() if os.path.exists(path) else []
        except Exception:  # noqa: BLE001
            s = []
        solids.append(s[0] if len(s) == 1 else None)
    if any(s is None for s in solids):
        for p, path, s in zip(asm.parts, paths, solids):
            if s is None:
                from ..checks import file_valid

                r = file_valid.run(Context(task, path)) if os.path.exists(path) else \
                    CheckResult("FILE_VALID", False, f"no file {p.file}")
                if r.ok:
                    r = CheckResult("FILE_VALID", False, "not exactly one solid")
                r.name = f"{p.name} FILE_VALID"
                checks.append(r)
        return Report(task.id, False, checks)

    # ---- each part, by the grader's own checks
    reps = []
    for k, (p, path) in enumerate(zip(asm.parts, paths)):
        rep = grade_part(subtask(task, asm, k, solids), path, optional=False, score=False)
        reps.append(rep)
        for c in rep.checks:
            c.name = f"{p.name} {c.name}"
            checks.append(c)
    grips = [_grips(r) for r in reps]

    checks.append(_insertion(task, asm, solids, grips))
    room, contact = _joint_room(task, asm, solids, grips)
    checks.append(room)
    checks.append(_joint_reach(task, asm, solids, grips))
    if asm.joint.length is not None:
        checks.append(_joint_length(asm, grips))
    try:
        sides = joint_sides(asm, solids, reps)
    except Exception as e:  # noqa: BLE001 - a part that cannot be measured fails
        sides = None
        checks.append(CheckResult("JOINT_BOLTS", False, f"could not be measured: {type(e).__name__}: {e}",
                                  group="sturdy"))
    if sides is not None:
        checks.append(_joint_bolts(task, asm, contact, sides))
    if task.stiffness:
        if not stiffness:
            checks.append(CheckResult("STIFFNESS", None, "not run (asked to leave it out)", required=False, group="sturdy"))
        elif not all(c.ok for c in checks if c.required):
            checks.append(CheckResult("STIFFNESS", None, "skipped: run once every other requirement is met "
                                                         "(the analysis takes minutes)", group="sturdy"))
        else:
            checks.append(_stiffness(task, solids))
    passed = all(c.ok for c in checks if c.required)
    info = {}
    from .. import material_score

    if passed and material_score.asked(task):
        info["material_used"] = _printed(task, asm, paths, reps)
    info.update({"volume": round(sum(float(s.volume) for s in solids), 3),
            "parts": {p.name: round(float(s.volume), 3) for p, s in zip(asm.parts, solids)}})
    bb = bd.Compound(solids).bounding_box()
    info.update(bbox_min=[round(v, 3) for v in bb.min], bbox_max=[round(v, 3) for v in bb.max],
                bbox_size=[round(v, 3) for v in bb.size])
    return Report(task.id, passed, checks, info)


def _printed(task: Task, asm: J.Assembly, paths, reps) -> dict:
    """The score: the plastic the printer lays down for both parts, each in the
    build direction its printing check chose (level 3's measure, unchanged)."""
    from .. import material_score

    per, total, grams = {}, 0.0, 0.0
    for p, path, rep in zip(asm.parts, paths, reps):
        fdm = next((c for c in rep.checks if c.name.split(" ")[-1].startswith("FDM_")), None)
        direction = (fdm.details or {}).get("direction") if fdm else None
        try:
            m = material_score.material_used(task, path, direction)
        except Exception as e:  # noqa: BLE001 - no figure rather than a wrong one
            return {"error": f"{p.name}: {type(e).__name__}: {e}"}
        per[p.name] = m
        total += m["used_cm3"]
        grams += m["grams"]
    return {"used_cm3": round(total, 3), "grams": round(grams, 2),
            "measure": material_score.MEASURE[material_score.process_of(task)] + ", both parts", "parts": per}


def grade_folder(task_dir: str, assembly: str | None = None, **kw) -> Report:
    task = Task.load(os.path.join(task_dir, "task.json"))
    return grade(task, assembly or os.path.join(task_dir, ASSEMBLY_FILE), **kw)


# --------------------------------------------------------------------------


def _bolt_heads(task: Task, fixture: str, grips: dict) -> list:
    """The heads of a fixed part's bolts, as they sit once that part's bolts are in."""
    out = []
    for b in task.bolts(0):
        if b.part != fixture:
            continue
        bb = BOLTS[b.size]
        g = grips.get(b.id, bb.grip_min)
        out.append((b.id, bd.Plane(origin=b.position, z_dir=b.axis) * bd.Pos(0, 0, g + bb.head_h / 2)
                    * bd.Cylinder(bb.head_d / 2, bb.head_h)))
    return out


def _insertion(task: Task, asm: J.Assembly, solids, grips) -> CheckResult:
    lim = task.tolerances["overlap_volume_max"]
    fixed = task.fixed_solids(0)
    rows, problems = [], []
    placed: dict = {}
    for k, p in enumerate(asm.parts):
        bodies = {f"fixed part {n}": s for n, s in fixed.items()}
        bodies.update({f"obstacle {n}": s for n, s in task.obstacle_solids(0).items()})
        bodies.update(placed)
        from .path import sweep_path

        hits, stretches, why = sweep_path(solids[k], p.path or [], bodies, direction=p.direction,
                                          box=list(task.obstacle_solids(0).values()))
        if why:
            problems.append(f"{p.name}: {why}")
        for what, v in hits.items():
            rows.append({"part": p.name, "against": what, "volume": v})
        bad = sorted(((v, w) for w, v in hits.items() if v > lim), reverse=True)
        if bad:
            where = next((st["stretch"] for st in stretches if st["hits"].get(bad[0][1], 0) > lim), "")
            problems.append(f"{p.name} runs into {bad[0][1]} on its way in ({where}; {bad[0][0]:.3f} mm3)"
                            + (f" and {len(bad) - 1} more" if len(bad) > 1 else ""))
        placed[p.name] = solids[k]
        for bid, head in _bolt_heads(task, p.bolts_to, grips[k]):
            placed[f"bolt head {bid} (on {p.name})"] = head
    if problems:
        return CheckResult("INSERTION", False, "; ".join(problems),
                           max(r["volume"] for r in rows), "mm3", {"sweeps": rows})
    from .path import describe_way

    order = ", then ".join(f"{p.name} {describe_way(p.direction, p.path)}" for p in asm.parts)
    return CheckResult("INSERTION", True, f"each part goes in clear of everything: {order}",
                       max((r["volume"] for r in rows), default=0.0), "mm3", {"sweeps": rows})


def _plane_faces(solid, origin, normal):
    from ..checks.seating import _onto_plane, _planar_faces

    got, _ = _planar_faces(solid, tuple(normal), tuple(origin), -1e-3, 1e-3)
    return [_onto_plane(f, tuple(-np.asarray(normal)), -dist) for f, dist in got]


def contact_faces(asm: J.Assembly, solids) -> list:
    """Where the two parts touch at the joint plane: the overlap of the head
    side's faces lying on it (facing the nut side, -axis) with the nut side's
    (facing +axis)."""
    j = asm.joint
    k_head = 0 if asm.parts[0].name == j.head_on else 1
    c = j.centre
    head = _plane_faces(solids[k_head], c, -j.axis)
    nut = _plane_faces(solids[1 - k_head], c, j.axis)
    out = []
    for a in head:
        for b in nut:
            x = a & b
            for f in (x.faces() if x is not None else []):
                if f.area > 1e-9:
                    out.append(f)
    return out


def _signed(v, n):
    return float(np.asarray(tuple(v)) @ n)


def shear_keys(asm: J.Assembly, solids) -> tuple[list, float]:
    """Pairs of facing flanks near the joint face: a flat face of one part,
    square to the joint face and within ``KEY_DEPTH`` of it, facing a flat
    face of the other part at most ``KEY_GAP`` away, overlapping it. Returns
    ([{"normal", "arm", "area_mm2", "gap_mm"}], area of the pairs that touch)."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane

    j = asm.joint
    n, c = j.axis, j.centre

    def flanks(solid):
        out = []
        for f in solid.faces():
            if BRepAdaptor_Surface(f.wrapped).GetType() != GeomAbs_Plane:
                continue
            fn = np.array(tuple(f.normal_at()))
            if abs(float(fn @ n)) > 1e-6:
                continue
            hs = [_signed(v, n) - float(c @ n) for v in f.vertices()]
            if max(abs(h) for h in hs) > J.KEY_DEPTH + 1e-6:
                continue
            out.append((f, fn))
        return out

    keys, touching = [], 0.0
    fa, fb = flanks(solids[0]), flanks(solids[1])
    for f1, n1 in fa:
        o1 = np.array(tuple(f1.center()))
        for f2, n2 in fb:
            if float(n1 @ n2) > -1 + 1e-6:
                continue
            gap = float((np.array(tuple(f2.center())) - o1) @ n1)
            if not (-1e-4 <= gap <= J.KEY_GAP + 1e-6):
                continue
            shifted = f2.moved(bd.Location(tuple(-n1 * gap)))
            x = f1 & shifted
            a = sum(g.area for g in (x.faces() if x is not None else []))
            if a < 1.0:
                continue
            # the key's reach along its flank, from the bolts' centre: how well it stops a twist
            along = np.cross(n, n1)
            arm = max(abs(float((np.array(tuple(v)) - c) @ along)) for g in x.faces() for v in g.vertices())
            keys.append({"normal": [round(float(v), 6) for v in n1], "arm": round(arm, 3), "area_mm2": round(a, 3),
                         "gap_mm": round(gap, 4)})
            if gap <= 1e-4:
                touching += a
    return keys, touching


def _joint_room(task: Task, asm: J.Assembly, solids, grips):
    lim = task.tolerances["overlap_volume_max"]
    tol = task.tolerances
    j = asm.joint
    from ..checks.interference import keepout

    k_head = 0 if asm.parts[0].name == j.head_on else 1
    others = {f"fixed part {n}": s for n, s in task.fixed_solids(0).items()}
    others.update({f"obstacle {n}": s for n, s in task.obstacle_solids(0).items()})
    rows, problems = [], []
    for i, pos in enumerate(j.positions):
        bid = f"{J.JOINT}{i + 1}"
        for side, size, axis, k in (("head", j.bolt, j.axis, k_head), ("nut", nut_side(j.bolt), -j.axis, 1 - k_head)):
            g = grips[k].get(bid, BOLTS[size].grip_min)
            ko = keepout(ResolvedBolt(bid, J.JOINT, size, tuple(pos), tuple(axis)), g, tol)
            for what, s in others.items():
                v = volume(ko & s)
                rows.append({"bolt": bid, "side": side, "against": what, "volume": round(v, 4)})
                if v > lim:
                    problems.append(f"{bid}: the {side} has no room, {what} is in the way ({v:.3f} mm3)")
    faces = contact_faces(asm, solids)
    joint_area = sum(f.area for f in faces)
    fused = solids[0].fuse(solids[1]).clean()
    total = max(0.0, (area(solids[0]) + area(solids[1]) - area(fused)) / 2)
    keys, key_touch = shear_keys(asm, solids)
    elsewhere = max(0.0, total - joint_area - key_touch - _key_floor(asm, solids))
    details = {"keepouts": rows, "contact_joint_mm2": round(joint_area, 3), "contact_elsewhere_mm2": round(elsewhere, 3),
               "shear_keys": keys}
    if joint_area <= 0:
        problems.append("the two parts do not touch at the joint face")
    if elsewhere > CONTACT_ELSEWHERE_MAX:
        problems.append(f"the two parts also touch outside the joint face and its shear keys ({elsewhere:.1f} mm2; "
                        f"limit {CONTACT_ELSEWHERE_MAX:g}): only the joint may carry load between them")
    contact = {"faces": faces, "area": joint_area, "keys": keys}
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult("JOINT_ROOM", False, problems[0] + more, round(elsewhere, 3), "mm2",
                           dict(details, problems=problems)), contact
    kt = f", {len(keys)} shear key flank pair{'s' if len(keys) != 1 else ''}" if keys else ""
    return CheckResult("JOINT_ROOM", True, f"heads and nuts have room; the parts touch only at the joint "
                                           f"({joint_area:.0f} mm2 of joint face{kt})", round(elsewhere, 3), "mm2",
                       details), contact


def _key_floor(asm: J.Assembly, solids) -> float:
    """Contact on faces parallel to the joint face but off it, within the key
    depth (a key's top on its groove's floor): part of the joint too."""
    j = asm.joint
    tot = 0.0
    k_head = 0 if asm.parts[0].name == j.head_on else 1

    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane

    def faces(solid, normal):
        out = []
        for f in solid.faces():
            if BRepAdaptor_Surface(f.wrapped).GetType() != GeomAbs_Plane:
                continue
            try:
                fn = np.array(tuple(f.normal_at()))
            except Exception:  # noqa: BLE001
                continue
            if float(fn @ normal) < 1 - 1e-6:
                continue
            h = _signed(f.center(), j.axis) - float(j.centre @ j.axis)
            if 1e-3 < abs(h) <= J.KEY_DEPTH + 1e-6:
                out.append((f, h))
        return out

    for fh, hh in faces(solids[k_head], -j.axis):
        for fn_, hn in faces(solids[1 - k_head], j.axis):
            if abs(hh - hn) < 1e-4:
                x = fh & fn_
                tot += sum(g.area for g in (x.faces() if x is not None else []))
    return tot


def _joint_reach(task: Task, asm: J.Assembly, solids, grips) -> CheckResult:
    """JOINT_REACH: a key reaches every joint bolt from outside. Along each
    head's axis, from the top of the head out past everything there is, a
    cylinder of the head's diameter is free of both parts, the fixed parts
    and every obstacle: a straight key comes in from outside -- here, through
    the box's opening -- and finds the head."""
    from .sweep import travel

    j = asm.joint
    lim = task.tolerances["overlap_volume_max"]
    k_head = 0 if asm.parts[0].name == j.head_on else 1
    b = BOLTS[j.bolt]
    bodies = {p.name: s for p, s in zip(asm.parts, solids)}
    bodies.update({f"fixed part {n}": s for n, s in task.fixed_solids(0).items()})
    bodies.update({f"obstacle {n}": s for n, s in task.obstacle_solids(0).items()})
    rows, problems = [], []
    for i, pos in enumerate(j.positions):
        bid = f"{J.JOINT}{i + 1}"
        top = np.asarray(pos, float) + j.axis * (grips[k_head].get(bid, b.grip_min) + b.head_h)
        start = bd.Pos(*top) * bd.Sphere(0.01)
        L = travel(start, -j.axis, list(bodies.values()))  # out past everything along the axis
        key = bd.Plane(origin=tuple(top), z_dir=tuple(j.axis)) * bd.Pos(0, 0, L / 2) * bd.Cylinder(b.head_d / 2, L)
        hits = {n: round(volume(key & s), 4) for n, s in bodies.items()}
        rows.append({"bolt": bid, "length_mm": round(L, 2), "hits": hits})
        bad = sorted(((v, n) for n, v in hits.items() if v > lim), reverse=True)
        if bad:
            problems.append(f"{bid}: no key reaches it from outside: along its axis {bad[0][1]} is in the way "
                            f"({bad[0][0]:.1f} mm3)")
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult("JOINT_REACH", False, problems[0] + more, None, "", {"keys": rows})
    return CheckResult("JOINT_REACH", True, f"a straight key reaches all {len(rows)} joint bolts from outside",
                       None, "", {"keys": rows})


def _joint_length(asm: J.Assembly, grips) -> CheckResult:
    """JOINT_LENGTH: the stated bolt length suits what it clamps: past the
    head-side part and the nut-side part (their measured grips) and the nut,
    it stands out at least one thread pitch (the nut fully engaged) and at
    most two (the nut's room)."""
    from ..catalogue import NUTS, stick_out

    j = asm.joint
    k_head = 0 if asm.parts[0].name == j.head_on else 1
    P = NUTS[j.bolt].pitch
    rows, problems = [], []
    for i, _ in enumerate(j.positions):
        bid = f"{J.JOINT}{i + 1}"
        clamped = grips[k_head].get(bid, 0.0) + grips[1 - k_head].get(bid, 0.0)
        s = stick_out(j.bolt, j.length, clamped)
        rows.append({"bolt": bid, "clamped_mm": round(clamped, 3), "stick_out_mm": round(s, 3)})
        if not (P - 1e-3 <= s <= 2 * P + 1e-3):
            problems.append(f"{bid}: a {j.bolt} x {j.length:g} through {clamped:.2f} mm of parts and its nut "
                            f"({NUTS[j.bolt].m:g} mm) stands out {s:.2f} mm; it must stand out {P:g} to {2 * P:g} mm "
                            "(one to two thread pitches)")
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult("JOINT_LENGTH", False, problems[0] + more, None, "", {"bolts": rows})
    return CheckResult("JOINT_LENGTH", True, f"{j.bolt} x {j.length:g}: stands out of every nut by one to two thread "
                                             f"pitches", None, "", {"bolts": rows})


def _outline_points(faces) -> np.ndarray:
    pts = []
    for f in faces:
        for e in f.edges():
            for t in np.linspace(0.0, 1.0, 9):
                pts.append(tuple(e.position_at(float(t))))
    return np.array(pts) if pts else np.zeros((0, 3))


def _holes(rep: Report) -> dict:
    hol = next((c for c in rep.checks if c.name.split(" ")[-1] == "HOLES"), None)
    return {r["bolt"]: r for r in ((hol.details or {}).get("holes") or [])} if hol else {}


def edge_distances(solid, point, axis, depth: float, n_dirs: int = J.EDGE_DIRECTIONS) -> np.ndarray:
    """From a hole's axis, at ``depth`` along ``axis`` (the middle of the
    part's thickness there), the distance to where the part's material ends
    in each of ``n_dirs`` in-plane directions (the joint's own in-plane axes,
    from x): the first face the line leaves the material through, after it
    enters at the hole wall. inf if the material does not end within 500 mm."""
    from OCP.gp import gp_Dir, gp_Lin, gp_Pnt
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector

    ex, ey = J.frame(axis)
    o = np.asarray(point, float) + np.asarray(axis, float) * depth
    hit = IntCurvesFace_ShapeIntersector()
    hit.Load(solid.wrapped, 1e-6)
    out = np.full(n_dirs, np.inf)
    for i in range(n_dirs):
        a = 2 * np.pi * i / n_dirs
        u = np.cos(a) * ex + np.sin(a) * ey
        hit.Perform(gp_Lin(gp_Pnt(*map(float, o)), gp_Dir(*map(float, u))), 0.0, 500.0)
        ws = sorted(hit.WParameter(j) for j in range(1, hit.NbPnt() + 1))
        if not ws:
            continue
        inside = solid.is_inside(bd.Vector(*(o + u * 1e-3)))  # true when the hole is not there (no hole found)
        rest = ws if inside else ws[1:]  # skip the hole wall, where the line enters the material
        if rest:
            out[i] = round(float(rest[0]), 4)
    return out


def joint_sides(asm: J.Assembly, solids, reps) -> dict:
    """The plastic round each joint bolt, as measured on the two parts."""
    j = asm.joint
    k_head = 0 if asm.parts[0].name == j.head_on else 1
    b, nut = BOLTS[j.bolt], NUTS[j.bolt]
    out = {"head": [], "nut": []}
    for side, k, outer, sgn in (("head", k_head, b.head_d, 1.0), ("nut", 1 - k_head, nut.dw, -1.0)):
        rows = _holes(reps[k])
        for i, pos in enumerate(j.positions):
            r = rows.get(f"{J.JOINT}{i + 1}") or {}
            t = float(r.get("grip", b.grip_min))
            hole = float(r.get("diameter", b.clear_coarse))
            out[side].append({"t": t, "hole": hole, "outer": outer,
                              "edge": edge_distances(solids[k], pos, j.axis * sgn, t / 2)})
    return out


def _joint_bolts(task: Task, asm: J.Assembly, contact, sides=None) -> CheckResult:
    st = task.stiffness or {}
    per = st.get("per_mount") or []
    if not per:
        return CheckResult("JOINT_BOLTS", None, "no stated load", group="sturdy")
    if not contact["faces"]:
        return CheckResult("JOINT_BOLTS", False, "the parts do not touch at the joint face, so the joint cannot "
                                                 "carry anything", group="sturdy")
    worst_rows, worst = [], None
    for m in per:
        mount = m["mount"]
        k = next(i for i, p in enumerate(asm.parts) if p.bolts_to == mount)
        load_on_head = asm.parts[k].name == asm.joint.head_on
        res = J.bolt_forces(asm, _outline_points(contact["faces"]), np.array(task.poses[0][mount].origin, float),
                            float(m["design_load_N"]), load_on_head, contact.get("keys"), sides)
        for r in res["bolts"]:
            worst_rows.append(dict(r, mount=mount))
        w = max(res["bolts"], key=lambda r: r["utilisation"])
        if worst is None or w["utilisation"] > worst[0]["utilisation"]:
            worst = (w, res, m)
    w, res, m = worst
    al = res["allowables"]
    passed = w["utilisation"] <= 1.0
    reason = (f"worst: bolt {w['bolt']} in {w['part']}, {w['mode']}, at {w['utilisation']:.0%} of what the plastic "
              f"takes (pull {w['tension_N']:.0f} N, slide {w['shear_N']:.0f} N; {al['material']}, bearing "
              f"{al['bearing_MPa']:.3g} MPa, shear {al['shear_MPa']:.3g} MPa allowed), with {m['mass_kg']:g} kg on "
              f"{m['mount']} pulling along {describe(w['load_direction'])}"
              + (f"; shear keys take the slide ({res['keys']['slide_taken']})"
                 + (" and the twist" if res["keys"]["twist_taken"] else "") if res["keys"]["count"] else ""))
    if not passed:
        reason = "the plastic gives way: " + reason
    return CheckResult("JOINT_BOLTS", passed, reason, w["utilisation"], "ratio",
                       {"bolts": worst_rows, "allowables": al, "directions": res["directions"], "keys": res["keys"]},
                       group="sturdy")


def _stiffness(task: Task, solids) -> CheckResult:
    from ..checks import stiffness as S

    fused = solids[0].fuse(solids[1]).clean()
    if len(fused.solids()) != 1:
        return CheckResult("STIFFNESS", False, "the two parts do not join into one load path at the joint face",
                           group="sturdy")
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, "assembly.step")
        write_step(fused.solids()[0], p)
        try:
            r = S.run(Context(task, p))
        except Exception as e:  # noqa: BLE001 - a part that cannot be measured fails
            r = CheckResult("STIFFNESS", False, f"could not be measured: {type(e).__name__}: {e}")
    r.group = "sturdy"
    r.reason = "the two parts together, joined at the joint face: " + r.reason
    return r
