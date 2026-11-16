"""Level 2: the two mounting faces are at an angle to each other and offset
sideways, and the whole scene has an arbitrary position and orientation.

Nothing stands between the parts yet (that is level 3), but nothing is
axis-aligned any more: no face is parallel to a coordinate plane, the faces
are not parallel to each other, and the bolt patterns do not overlap in plan.
A memorised level-1 answer (two plates joined by a straight wall) does not
transfer; the connector has to be built in each face's own frame and bridged
through space.

Generation is answer-first, as at level 1. The reference connector is:

* a plate on each mounting face, covering its bolts, extended by a tab on
  the side facing the other part;
* a bridge from the top of one tab to the top of the other: the convex
  hull of the two tab rectangles.

It is built entirely from what ``task.json`` says -- face origins, normals,
bolt positions -- so the same code would work for any placement. A draw is
kept only if the reference and an alternative body (thicker plates, longer
tabs, narrower bridge) both pass the grader, so no task hangs on a knife edge.
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import build123d as bd

from ..catalogue import GENERATOR_VERSION, BOLTS, Interface, edge_margin, make_interface, sample_interface
from ..geometry import Placement, convex_hull, matmul, rot_axis_angle, rot_from_quaternion, write_step
from ..task import DEFAULT_TOLERANCES, Task

LEVEL = 2
KNOBS_DEFAULT = {"process": "pbf", "stiffness": 0.0}
TAB = 14.0
SPREAD = 24.0  # mm, side of the truss's chord square

SITUATION = (
    "Two parts are fixed in space. Their mounting faces are at an angle to each\n"
    "other and offset sideways, and the whole arrangement has an arbitrary\n"
    "position and orientation: no face is parallel to a coordinate plane.\n"
    "Nothing stands between them."
)


def _rz(deg):
    return rot_axis_angle((0.0, 0.0, 1.0), deg)


FLIP = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def _draw(rng: random.Random) -> dict:
    a, b = sample_interface(rng), sample_interface(rng)
    ba, bb = BOLTS[a.bolt], BOLTS[b.bolt]
    t_choices = [4.0, 5.0, 6.0, 8.0, 10.0]
    q = [rng.gauss(0.0, 1.0) for _ in range(4)]
    return {
        "A": {"kind": a.kind, "params": a.params, "yaw": 5.0 * rng.randrange(72)},
        "B": {"kind": b.kind, "params": b.params, "yaw": 5.0 * rng.randrange(72)},
        "t": {"A": rng.choice([t for t in t_choices if t >= ba.grip_min]),
              "B": rng.choice([t for t in t_choices if t >= bb.grip_min])},
        # B relative to A: height above A's face, sideways offset and its direction
        "height": float(rng.randrange(70, 161)) / 2,  # 35 .. 80 mm
        "lateral": float(rng.randrange(50, 141)) / 2,  # 25 .. 70 mm
        "lateral_dir": 5.0 * rng.randrange(72),
        # tilt of B's face: angle and the horizontal axis it tilts about
        "tilt": 5.0 * rng.randrange(4, 15),  # 20 .. 70 deg
        "tilt_axis": 5.0 * rng.randrange(72),
        # placement of the whole scene
        "scene_quat": q,
        "scene_origin": [float(rng.randrange(-300, 301)) / 2 for _ in range(3)],
    }


def task_from_draw(seed: int, d: dict) -> Task:
    parts = {n: make_interface(d[n]["kind"], d[n]["params"]) for n in ("A", "B")}
    g = Placement(tuple(d["scene_origin"]), rot_from_quaternion(*d["scene_quat"]))
    psi = math.radians(d["lateral_dir"])
    phi = math.radians(d["tilt_axis"])
    tilt = rot_axis_angle((math.cos(phi), math.sin(phi), 0.0), d["tilt"])
    a = Placement((0.0, 0.0, 0.0), _rz(d["A"]["yaw"]))
    b = Placement((d["lateral"] * math.cos(psi), d["lateral"] * math.sin(psi), d["height"]),
                  matmul(tilt, matmul(FLIP, _rz(d["B"]["yaw"]))))
    pose = {"A": a.then(g), "B": b.then(g)}
    return Task(f"L2-s{seed}", LEVEL, seed, parts, [pose], dict(DEFAULT_TOLERANCES), {"generator": "level2"}, generator_version=GENERATOR_VERSION)


# --------------------------------------------------------------------------
# the reference connector (generic: reads only faces and bolts)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _plate_frame(task: Task, part: str, other: str) -> bd.Plane:
    faces = {f.part: f for f in task.faces(0)}
    f, g = faces[part], faces[other]
    w = [g.origin[i] - f.origin[i] for i in range(3)]
    k = _dot(w, f.normal)
    u = [w[i] - k * f.normal[i] for i in range(3)]
    if math.sqrt(_dot(u, u)) < 1e-6:
        return bd.Plane(origin=f.origin, z_dir=f.normal)
    return bd.Plane(origin=f.origin, x_dir=u, z_dir=f.normal)


def build_connector(task: Task, d: dict, *, style: str = "reference", hole_shift: float = 0.0,
                    hole_d: float | None = None, push: float = 0.0, lift: float = 0.0,
                    member_area: float | None = None, thin_plates: bool = False,
                    fin: bool = False, spread: float | None = None, shape: str | None = None) -> bd.Part:
    """The reference: a plate on each face with a tab beside its bolts,
    joined by a truss (see ``levels.truss``): four round chords leave each
    tab square to its face at the corners of a square, run straight across
    to the other tab, and are braced by a diagonal in every bay of every
    side panel, with a sphere at each node.

    style        "alt": thicker plates, a wider truss, chords 10% thicker
    hole_shift, hole_d, push, lift   as at level 1
    member_area  chords sized to exactly this section in total (instead of
                 1.25 x the requirement)
    thin_plates  plates only as thick as the bolts allow
    fin          a 0.4 mm fin on plate A: fits, but cannot be made
    spread       the chord square's side, mm (default ``SPREAD``)
    shape        one of ``levels.weak``'s shapes instead (fits, not stiff)
    """
    if shape:
        from . import weak

        faces = {f.part: bd.Vector(*f.origin) for f in task.faces(0)}
        return weak.build(task, shape, {"A": faces["B"], "B": faces["A"]}, lift=lift)
    from ..checks.section_min import required_area
    from . import construct as C
    from . import rods as R
    from . import truss as T

    alt = style == "alt"
    need = 1.25 * required_area(task, "A", "B")
    area = member_area if member_area is not None else need * (1.21 if alt else 1.0)
    # an exact member_area (self-test, par) is built as asked; the
    # reference's own chords are never thinner than R_MIN
    r = math.sqrt(area / 4 / math.pi)
    r = min(R.R_MAX, max(1.0, r) if member_area is not None else max(R.R_MIN, r))
    a = (spread if spread is not None else SPREAD) + (6.0 if alt else 0.0)
    a = max(a, 2 * r + R.GAP)
    rho = a / math.sqrt(2)  # the square's corners lie on this circle, however it ends up turned
    faces = {f.part: f for f in task.faces(0)}
    gap_in = r + 2.0  # chords clear of the bolt rings
    tab = gap_in + 2 * rho + r + 3.0
    plates, centre = {}, {}
    for part, other in (("A", "B"), ("B", "A")):
        bt = BOLTS[task.parts[part].bolt]
        t = max(R.PLATE_T, bt.grip_min) + (1.0 if alt else 0.0)
        if thin_plates:  # as thin as the bolts allow, but never under a third of the rod
            t = max(bt.grip_min, 2 * r / 3 + 0.1)
        z0 = lift if part == "A" else 0.0
        P = C.plate(task, part, C._v(faces[other].origin), t, tab=tab, z0=z0)
        yc = (P["y0"] + P["y1"]) / 2
        width = 2 * (rho + r + 3.0)
        if P["y1"] - P["y0"] < width:  # widen the plate to carry the chords
            P["y0"], P["y1"] = yc - width / 2, yc + width / 2
        diag = math.hypot(P["x1"] + tab - P["x0"], P["y1"] - P["y0"])
        P["t"] = t = min(8.0, max(t, diag / 38.0))  # a long thin plate warps when made
        P["solid"] = P["frame"] * bd.Pos((P["x0"] + P["x1"] + tab) / 2, yc, z0) * bd.extrude(
            bd.Rectangle(P["x1"] + tab - P["x0"], P["y1"] - P["y0"]), t)
        if fin and part == "A":
            P["solid"] = P["solid"] + P["frame"] * bd.Pos(P["x0"] - 3.0, yc, z0 + t / 2) * bd.Box(8.0, 8.0, 0.4)
        plates[part] = P
        centre[part] = P["frame"].from_local_coords(bd.Vector(P["x1"] + gap_in + rho, yc, z0 + t / 2))
    pa, pb = plates["A"], plates["B"]
    na, nb = pa["frame"].z_dir, pb["frame"].z_dir
    # columns square out of each plate, just tall enough that every chord
    # leaves its plate square and turns only beyond the plate's clearance,
    # and that no rod of the run skims either plate
    xdir = pa["frame"].x_dir
    ha = hb = 1.0
    need = r + T.CLEAR
    rd = max(T.R_DIAG_MIN, 0.7 * r)  # thinner than the chord: the joint keeps an open wedge
    lay = None
    for _ in range(16):
        p1, p2 = centre["A"] + na * (pa["t"] / 2 + ha), centre["B"] + nb * (pb["t"] / 2 + hb)
        dvec = (p2 - p1).normalized()
        # square's sides along the run's direction as seen on plate A: the
        # mitre at A's bend then shifts each corner by only half a side
        xdir = dvec - na * dvec.dot(na) if (dvec - na * dvec.dot(na)).length > 1e-6 else pa["frame"].x_dir
        ch = T._mitre([centre["A"], p1, p2, centre["B"]], T.square_offsets(na, xdir, a))
        da = min((c[1] - c[0]).length for c in ch) - pa["t"] / 2
        db = min((c[-1] - c[-2]).length for c in ch) - pb["t"] / 2
        if da < need - 0.05 or db < need - 0.05:  # converges from below: overshoot a little
            ha += max(0.0, need - da) + (0.3 if da < need - 0.05 else 0.0)
            hb += max(0.0, need - db) + (0.3 if db < need - 0.05 else 0.0)
            continue
        lay = T.prune(T.path_layout([centre["A"], p1, p2, centre["B"]], xdir, a, r,
                                    t_start=pa["t"], t_end=pb["t"]), r, rd)
        ca, cb = T.plate_clearance(lay, pa, r, rd), T.plate_clearance(lay, pb, r, rd)
        if ca >= T.PLATE_CLEAR and cb >= T.PLATE_CLEAR:
            break
        ha += max(0.0, T.PLATE_CLEAR - ca) + 0.5 if ca < T.PLATE_CLEAR else 0.0
        hb += max(0.0, T.PLATE_CLEAR - cb) + 0.5 if cb < T.PLATE_CLEAR else 0.0
        lay = None
    if lay is None:
        raise ValueError("no column height keeps the truss clear of both plates")
    d.setdefault("truss", {}).update(columns=[round(ha, 1), round(hb, 1)])
    ok, worst = T.angles_ok(lay)
    bends_ok = all(R.min_bend_ok(ch) for ch in lay["bends"])
    d.setdefault("truss", {}).update(bays=lay["bays"], min_angle=round(worst, 1), spread=a,
                                     diagonals=len(lay["diagonals"]), dropped=lay["dropped"])
    if not ok or not bends_ok:
        raise ValueError(f"truss rods meet at {worst:.0f} deg (at least {T.MIN_ANGLE:g} needed)"
                         + ("" if bends_ok else ", or a chord bends too sharply"))
    body = pa["solid"] + pb["solid"] + T.solid(lay, r, rd)
    d["truss"]["blend"] = lay.get("blend")
    body = C.cut_holes(body, task, plates, hole_shift=hole_shift, hole_d=hole_d)
    body = C.cut_protrusions(body, task, plates=plates)
    if push:
        na = plates["A"]["frame"].z_dir
        body = body.moved(bd.Location(tuple(-push * v for v in na)))
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


# --------------------------------------------------------------------------


def _faces_see_each_other(task: Task) -> bool:
    fa, fb = task.faces(0)
    w = [fb.origin[i] - fa.origin[i] for i in range(3)]
    L = math.sqrt(_dot(w, w))
    return _dot(w, fa.normal) > 0.3 * L and -_dot(w, fb.normal) > 0.3 * L


def generate(seed: int, knobs: dict | None = None, max_tries: int = 200):
    """Return (task, draw, reference solid). Deterministic in ``seed``."""
    from ..grade import grade

    rng = random.Random(f"L2-{seed}")
    rej: dict = {}

    def reject(why):
        rej[why] = rej.get(why, 0) + 1

    for _ in range(max_tries):
        d = _draw(rng)
        task = task_from_draw(seed, d)
        from ..requirements import apply_process

        process = (knobs or {}).get("process", KNOBS_DEFAULT["process"])
        apply_process(task, process)  # the reference must be makeable by it, or the draw is rejected
        from ..requirements import apply_key_turn

        apply_key_turn(task)  # every bolt turnable with its hex key, or the draw is rejected
        if process != KNOBS_DEFAULT["process"]:
            task.id += f"-{process}"
        stiff = float((knobs or {}).get("stiffness", KNOBS_DEFAULT["stiffness"]))
        if stiff not in (0.0, 1.0):
            raise ValueError("knob stiffness is 1 (required, from the task's bolts and span) or 0 (off)")
        if stiff:
            from ..requirements import apply_stiffness

            apply_stiffness(task)
            task.meta = dict(task.meta, knobs=dict(task.meta.get("knobs", {}), stiffness=stiff))
        if stiff != KNOBS_DEFAULT["stiffness"]:
            task.id += "-stiff" if stiff else "-nostiff"
        if not _faces_see_each_other(task):
            continue
        fx = task.fixed_solids(0)
        if fx["A"].distance_to(fx["B"]) < 1.0:
            continue
        try:
            ref = build_connector(task, d)
        except Exception as e:  # noqa: BLE001 - a degenerate bridge: draw again
            reject(f"reference not built: {str(e)[:60]}")
            continue
        if len(ref.solids()) != 1:
            reject("reference not one solid")
            continue
        from ..geometry import BUILD_ROOM, fits_build_volume

        if not fits_build_volume(ref, BUILD_ROOM):
            reject("reference does not fit a Prusa MINI with room to spare")
            continue
        # the reference first (the stiffness analysis runs only if all else
        # passes); the alternative body only for a reference that passed
        ok = True
        with tempfile.TemporaryDirectory() as tmp:
            for which in ("reference", "alternative"):
                if which == "alternative":
                    try:
                        part = build_connector(task, d, style="alt")
                    except Exception:  # noqa: BLE001
                        part = None
                    if part is None or len(part.solids()) != 1:
                        reject("alternative body not built")
                        ok = False
                        break
                    skip = frozenset({"MANUFACTURING", "STIFFNESS"})
                else:
                    part, skip = ref, frozenset()
                p = os.path.join(tmp, "candidate.step")
                write_step(part, p)
                rep = grade(task, p, optional=False, score=False, skip=skip)
                if not rep.passed:
                    reject(f"{which} fails: {','.join(rep.failed())}")
                    ok = False
                    break
                st = next((c for c in rep.checks if c.name == "STIFFNESS" and c.ok), None)
                if st is not None and not skip:
                    d["reference_stiffness_ratio"] = st.value  # weakest mount's figure / requirement
        if ok:
            from ..requirements import par as par_for

            task.par = par_for(task, d, build_connector)
            d["rejections"] = rej
            d["summary"] = (f"tilt {d['tilt']:g} deg, offset {d['lateral']:g} mm, "
                            f"height {d['height']:g} mm" + ("" if task.par else ", no par"))
            return task, d, ref
    raise RuntimeError(f"seed {seed}: no valid level-2 draw in {max_tries} tries ({rej})")


def _undersized(task):
    return {"hole_d": BOLTS[task.parts["A"].bolt].clear_fine - 0.2}


SELFTEST_VARIANTS = [
    ("reference answer", lambda t: {}, None),
    ("different body, same interfaces", lambda t: {"style": "alt"}, None),
    ("holes shifted 0.05 mm (inside tol)", lambda t: {"hole_shift": 0.05}, None),
    ("holes shifted 0.25 mm", lambda t: {"hole_shift": 0.25}, "HOLES"),
    ("holes shifted 0.5 mm", lambda t: {"hole_shift": 0.5}, "HOLES"),
    ("holes shifted 1 mm", lambda t: {"hole_shift": 1.0}, "HOLES"),
    ("holes shifted 2 mm", lambda t: {"hole_shift": 2.0}, "HOLES"),
    ("holes 0.2 mm under clearance", _undersized, "HOLES"),
    ("body pushed 1 mm into part A", lambda t: {"push": 1.0}, "NO_INTERFERENCE"),
    ("seat on A lifted 1 mm", lambda t: {"lift": 1.0}, "SEATING"),
]


def _need(task):
    from ..checks.section_min import required_area

    names = list(task.parts)
    return max(required_area(task, a, b) for i, a in enumerate(names) for b in names[i + 1:])


SELFTEST_VARIANTS += [
    # a sound member, well clear of the limit, must pass (at 1.1 x the variant's
    # plates or slanted segments can themselves be the thinnest section)
    ("member at 1.5 x the required section", lambda t: {"member_area": 1.5 * _need(t)}, ("ok", "SECTION_MIN")),
    ("member at 0.9 x the required section", lambda t: {"member_area": 0.9 * _need(t)}, "SECTION_MIN"),
    ("0.4 mm fin: fits, cannot be made", lambda t: {"fin": True}, "*_WALL_MIN"),  # the task's process
]


def PAR_CANDIDATES(task):
    """Lean answers par is chosen from (the smallest volume that passes
    every required check wins): trusses 14 to 24 mm wide with chords 3 or
    3.5 mm in radius, never under 1.15 x the required section, on plates of
    the reference's thickness. Plates thinned to the bolts' minimum grip
    bend too much here (measured: 0.70-0.75 of the requirement whatever the
    truss), so at this level stiffness, not the section, sets the plates."""
    import math

    need = _need(task)
    return [{"member_area": round(max(1.15 * need, 4 * math.pi * rc * rc), 3), "spread": a}
            for a in (14.0, 19.0, SPREAD) for rc in (3.0, 3.5)]


# stiffness rows (they run the stiffness analysis; see selftest): the lean
# answer par was taken from must pass, last round's slender shapes must fail
def _par_build(task):
    return dict((task.par or {}).get("build") or {"thin_plates": True})


SELFTEST_VARIANTS.append(("lean answer par was taken from (just stiff enough)", _par_build, None))
from .weak import SELFTEST_ROWS as _WEAK  # noqa: E402

SELFTEST_VARIANTS += [(label, (lambda s: lambda t: {"shape": s})(shape), "STIFFNESS") for label, shape in _WEAK]
