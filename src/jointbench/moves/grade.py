"""Grading a "moves" answer: a planar linkage that carries fixture B through stated poses.

Everything is measured on exact geometry, as in every other level:

  <part> ...     each part bolted to a fixture is judged by the existing grader
                 against that fixture: HOLES, SEATING, SEAT_INSIDE,
                 NO_INTERFERENCE, CLEAN and the filament printing rules. A link
                 bolted to neither is judged for FILE_VALID, CLEAN and printing.
  ASSEMBLY_FILE  the assembly file is complete and consistent.
  FASTENERS      every catalogue screw is long enough for its grip and nut, and
                 its head and nut bear on the body it is clamped to.
  ASSEMBLED      at the start nothing overlaps anything else.
  INSERTION      each part, in order, then each screw and nut, goes in along its
                 straight path without running into anything already there.
  PIVOTS         every declared pivot is a real pin in a real bore: a round pin
                 (a printed one, or a catalogue screw) on one body and a round
                 bore on the other, both on the declared axis, overlapping along
                 it, the bore at least the printing gap larger.
  MOBILITY       the bodies and pivots leave exactly one way to move, and the
                 declared drive controls it.
  MOTION         driven through its declared range in small steps, the
                 mechanism closes at every step (no dead point, no jump), and no
                 two bodies overlap.
  GAP            at every step, bodies that move relative to each other stay at
                 least the printing gap apart.
  POSES          fixture B passes through every required pose, in order,
                 within the stated tolerance.
  INTERLOCK      no pivot comes apart: at the start, each pivot's two bodies,
                 shifted 5 mm in any of 26 directions or tilted 15 deg, meet.
  PLAY           at each pivot the free shift and tilt are within the limits.

Deterministic: the same files give the same verdict and the same numbers.
"""
from __future__ import annotations

import math
import os

import build123d as bd
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

from ..catalogue import BOLTS
from ..checks.base import CheckResult, Context, volume
from ..geometry import Placement, read_step
from ..grade import Report, grade as grade_part
from ..task import Task
from . import assembly as AS
from . import linkage as LK
from . import motion as MO
from .sweep import collisions

SEAT_SHARE = 0.75  # share of a head's or nut's bearing ring that must bear on its own body's parts
OVERLAP_MIN = 1.0  # mm: a pin and its bore overlap along the axis by at least this
FINE_DEG = 0.25  # deg of drive between the kinematic samples


def settings(task: Task) -> dict:
    return task.tolerances["moves"]


def targets(task: Task) -> list[Placement]:
    return [Placement.from_json(t["placement"]) for t in task.meta["motion"]["targets"]]


class _Solid:
    """A body the part being judged must keep its tools clear of, as an obstacle of its own task."""

    def __init__(self, name, solid):
        self.name, self._s = name, solid

    def solid(self):
        return self._s


def subtask(task: Task, part: AS.PartEntry, before: list | None = None) -> Task:
    """The part's own judgement, as a one-fixture task at the start (or no fixture for a link), judged as
    at level 3 for bolt access: the other fixture and the parts put in before it (``before``: [(name,
    solid)]) are obstacles, so every bolt must be reachable when the part is bolted on arrival."""
    fx = part.bolts_to
    tol = {k: v for k, v in task.tolerances.items() if k != "moves"}
    parts = {fx: task.parts[fx]} if fx else {}
    pose = {fx: task.poses[0][fx]} if fx else {}
    obs = []
    if fx:
        fixed = task.fixed_solids(0)
        obs = [_Solid(f"fixture {n}", s) for n, s in fixed.items() if n != fx]
        obs += [_Solid(n, s) for n, s in (before or [])]
        obs += [_Solid(f"box {o.name}", o.solid()) for o in task.obstacles_at(0)]  # a closed box (level 6)
    t = Task(f"{task.id}/{part.name}", 3 if fx else 2, task.seed, parts, [pose], tol, {}, [obs] if obs else [],
             generator_version=task.generator_version)
    t.root = task.root
    return t


def _grips(rep: Report) -> dict:
    hol = next((c for c in rep.checks if c.name == "HOLES"), None)
    return {r["bolt"]: r["grip"] for r in ((hol.details or {}).get("holes") or [])} if hol else {}


def fixture_heads(task: Task, fixture: str, grips: dict) -> list:
    out = []
    for b in task.bolts(0):
        if b.part != fixture:
            continue
        bb = BOLTS[b.size]
        g = grips.get(b.id, bb.grip_min)
        out.append((b.id, bd.Plane(origin=b.position, z_dir=b.axis) * bd.Pos(0, 0, g + bb.head_h / 2)
                    * bd.Cylinder(bb.head_d / 2, bb.head_h)))
    return out


def _link_checks(task: Task, part: AS.PartEntry, path: str, manufacturing: bool) -> list[CheckResult]:
    """A part bolted to no fixture: valid file, clean modelling, printable."""
    from ..checks import clean, file_valid
    from ..checks import manufacturing as mfg

    ctx = Context(subtask(task, part), path)
    out = [file_valid.run(ctx)]
    if not out[0].ok:
        return out
    for mod, group in ((clean, "clean"), (mfg, "makeable")):
        if mod is mfg and not manufacturing:
            continue
        try:
            res = mod.run(ctx)
        except Exception as e:  # noqa: BLE001
            res = CheckResult(mod.NAME, False, f"could not be measured: {type(e).__name__}: {e}")
        for r in res if isinstance(res, list) else [res]:
            r.group = group
            out.append(r)
    return out


class Answer:
    """The parsed answer: parts, bodies and their pieces at the start."""

    def __init__(self, task, asm, solids, heads):
        self.task, self.asm = task, asm
        self.solids = dict(zip([p.name for p in asm.parts], solids))
        self.heads = heads
        fx = task.fixed_solids(0)
        self.fixtures = fx
        self.pieces = {b: [] for b in asm.bodies()}  # body -> [(name, shape)]
        self.pieces["A"].append(("fixture A", fx["A"]))
        self.pieces["B"].append(("fixture B", fx["B"]))
        # a closed box round everything (level 6): its walls stand still with the fixed fixture
        self.walls = [(f"box {o.name}", o.solid()) for o in task.obstacles_at(0)]
        self.pieces[task.meta["motion"]["fixed"]] += self.walls
        for p in asm.parts:
            hs = heads.get(p.name, [])
            s = self.solids[p.name]
            body = bd.Compound([s] + [h for _, h in hs]) if hs else s
            self.pieces[asm.body_of(p.name)].append((p.name + (" (with its bolt heads)" if hs else ""), body))
        for f in asm.fasteners:
            self.pieces[asm.body_of(f.held_by)].append((f.label, bd.Compound(f.bodies())))
        # a pivot on a ball bearing: the bearing rides with the part it is pressed into; the pin of the other body
        # runs in its bore without clearance (the bearing's own job), so that pair is not measured for gap or clash
        self.bearings, self.bearing_pin_body = {}, {}
        for pv in asm.pivots:
            if pv.bearing:
                name = f"bearing {pv.bearing} at {pv.name}"
                ring = pv.bearing_ring()
                self.pieces[asm.body_of(pv.seat_on)].append((name, ring))
                self.bearings.setdefault(pv.seat_on, []).append(ring)
                other = pv.joins[1] if pv.joins[0] == pv.seat_on else pv.joins[0]
                self.bearing_pin_body[name] = asm.body_of(other)


def grade(task: Task, assembly_path: str, *, manufacturing: bool = True, progress=None, extra=None) -> Report:
    """Grade the answer whose assembly file is ``assembly_path`` (part files next to it).
    ``manufacturing=False`` leaves the printing rules out (quicker, for tests)."""
    say = progress or (lambda s: None)
    mo = task.meta["motion"]
    checks: list[CheckResult] = []
    folder = os.path.dirname(os.path.abspath(assembly_path))
    try:
        asm = AS.load(assembly_path, mo["fixed"], mo["moving"])
    except AS.AssemblyError as e:
        checks.append(CheckResult("ASSEMBLY_FILE", False, str(e)))
        return Report(task.id, False, checks)
    checks.append(CheckResult("ASSEMBLY_FILE", True, "; ".join(
        [f"{p.name} bolts to {p.bolts_to or 'nothing (a link)'}" for p in asm.parts]
        + [f"{len(asm.pivots)} pivot(s), drive at {asm.drive} over {asm.range_deg:g} deg",
           f"{len(asm.fasteners)} fastener(s)"])))

    solids = []
    for p in asm.parts:
        path = os.path.join(folder, p.file)
        s = None
        if os.path.exists(path):
            try:
                got = read_step(path).solids()
                s = got[0] if len(got) == 1 else None
            except Exception:  # noqa: BLE001
                s = None
        solids.append(s)
    if any(s is None for s in solids):
        from ..checks import file_valid

        for p, s in zip(asm.parts, solids):
            if s is None:
                path = os.path.join(folder, p.file)
                r = file_valid.run(Context(task, path)) if os.path.exists(path) else \
                    CheckResult("FILE_VALID", False, f"no file {p.file}")
                if r.ok:
                    r = CheckResult("FILE_VALID", False, "not exactly one solid")
                r.name = f"{p.name} FILE_VALID"
                checks.append(r)
        return Report(task.id, False, checks)

    # ---- each part on its own
    heads, before = {}, []
    for p, s in zip(asm.parts, solids):
        say(f"grading {p.name}")
        path = os.path.join(folder, p.file)
        if p.bolts_to:
            sub = subtask(task, p, before)
            if not manufacturing:
                sub.tolerances["required_checks"] = [c for c in sub.tolerances.get("required_checks", [])
                                                     if c != "MANUFACTURING"]
            rep = grade_part(sub, path, optional=False, score=False)
            heads[p.name] = fixture_heads(task, p.bolts_to, _grips(rep))
            got = rep.checks
        else:
            got = _link_checks(task, p, path, manufacturing)
        before.append((p.name, s))
        for c in got:
            c.name = f"{p.name} {c.name}"
            checks.append(c)

    ans = Answer(task, asm, solids, heads)
    checks.append(_fasteners(ans))
    say("assembled state")
    checks.append(_assembled(ans))
    say("insertion")
    checks.append(_insertion(ans))
    checks.append(_pivots(ans))
    lk, mob = _mobility(ans)
    checks.append(mob)
    if lk is None:
        for n in ("MOTION", "GAP", "POSES"):
            checks.append(CheckResult(n, None, "not judged: the mechanism has no single motion (see MOBILITY)"))
    else:
        say("driving the mechanism")
        checks += _motion(ans, lk)
    if extra is not None:  # a level's own checks (level 6: the latch, its strength, the pivots' seats)
        checks += extra(ans, lk, checks)
    say("pivots: small moves")
    checks += _interlock(ans)
    passed = all(c.ok for c in checks if c.required)
    info = {"parts": {p.name: round(float(s.volume), 3) for p, s in zip(asm.parts, solids)},
            "volume": round(sum(float(s.volume) for s in solids), 3)}
    bb = bd.Compound(solids).bounding_box()
    info.update(bbox_min=[round(v, 3) for v in bb.min], bbox_max=[round(v, 3) for v in bb.max],
                bbox_size=[round(v, 3) for v in bb.size])
    from ..geometry import build_box, fits_build_volume

    info["build_box"] = {p.name: build_box(s) for p, s in zip(asm.parts, solids)}
    info["fits_build_volume"] = {p.name: fits_build_volume(s) for p, s in zip(asm.parts, solids)}
    return Report(task.id, passed, checks, info)


def grade_folder(task_dir: str, assembly: str | None = None, **kw) -> Report:
    task = Task.load(os.path.join(task_dir, "task.json"))
    return grade(task, assembly or os.path.join(task_dir, AS.ASSEMBLY_FILE), **kw)


# --------------------------------------------------------------------------
# the assembled answer, standing still


def _bearing(f: AS.Fastener, which: str, solids) -> tuple[float, float]:
    b = BOLTS[f.size]
    r_in = b.clear_coarse / 2
    if which == "head":
        r_out, z0 = b.head_d / 2, 0.0
    else:
        r_out, z0 = AS.NUTS[f.size][0] / 2, f.grip - AS.SEAT_SLAB
    ring = f.plane() * bd.Pos(0, 0, z0 + AS.SEAT_SLAB / 2) * (
        bd.Cylinder(r_out, AS.SEAT_SLAB) - bd.Cylinder(r_in, AS.SEAT_SLAB + 1))
    got = sum(volume(ring & s) for s in solids) / AS.SEAT_SLAB
    return got, math.pi * (r_out ** 2 - r_in ** 2)


def _fasteners(ans: Answer) -> CheckResult:
    asm = ans.asm
    if not asm.fasteners:
        return CheckResult("FASTENERS", True, "no fasteners: the parts hold by their own shape")
    problems, rows = [], []
    for f in asm.fasteners:
        _, m = AS.NUTS[f.size]
        if f.grip + m > f.length + 1e-6:
            problems.append(f"{f.label}: too short -- grip {f.grip:g} mm plus the {m:g} mm nut needs at least "
                            f"{f.grip + m:g} mm")
            continue
        body = asm.body_of(f.held_by)
        mine = [ans.solids[p.name] for p in asm.parts if asm.body_of(p.name) == body]
        for which in ("head", "nut"):
            got, ring = _bearing(f, which, mine)
            rows.append({"fastener": f.label, "end": which, "bearing_mm2": round(got, 3), "ring_mm2": round(ring, 3)})
            if got < SEAT_SHARE * ring:
                problems.append(f"{f.label}: its {which} bears on only {got / ring:.0%} of its ring on the parts that "
                                f"move with {f.held_by} (at least {SEAT_SHARE:.0%})")
    if problems:
        return CheckResult("FASTENERS", False, "; ".join(problems), details={"bearing": rows})
    return CheckResult("FASTENERS", True, f"{len(asm.fasteners)} screw(s): long enough, head and nut bear on the "
                       "body each is clamped to", details={"bearing": rows})


def _assembled(ans: Answer) -> CheckResult:
    task, asm = ans.task, ans.asm
    lim = task.tolerances["overlap_volume_max"]
    bodies = [(f"fixture {n}", s) for n, s in ans.fixtures.items()] + list(ans.walls)
    for p in asm.parts:
        bodies.append((p.name, ans.solids[p.name]))
        bodies += [(f"bolt head {bid} (on {p.name})", h) for bid, h in ans.heads.get(p.name, [])]
    for f in asm.fasteners:
        bodies += [(f"{f.label} screw", f.screw()), (f"{f.label} nut", f.nut())]
    worst, bad = 0.0, []
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            (na, a), (nb, b) = bodies[i], bodies[j]
            if na.startswith("fixture") and nb.startswith("fixture"):
                continue
            if MO.distance(a, b) > MO.CONTACT:
                continue
            v = volume(a & b)
            worst = max(worst, v)
            if v > lim:
                bad.append((v, na, nb))
    if bad:
        bad.sort(reverse=True)
        v, na, nb = bad[0]
        more = f" (and {len(bad) - 1} more pairs)" if len(bad) > 1 else ""
        return CheckResult("ASSEMBLED", False, f"assembled, {na} and {nb} overlap by {v:.3f} mm3{more}",
                           round(v, 4), "mm3")
    return CheckResult("ASSEMBLED", True, f"assembled, nothing overlaps ({len(bodies)} bodies)", round(worst, 4), "mm3")


def _insertion(ans: Answer) -> CheckResult:
    task, asm = ans.task, ans.asm
    lim = task.tolerances["overlap_volume_max"]
    placed = {f"fixture {n}": s for n, s in ans.fixtures.items()}
    rows, problems = [], []

    def go(what, body, d, length=None):
        hits = collisions(body, d, placed, length=length)
        for n, v in hits.items():
            rows.append({"moving": what, "against": n, "volume": v})
        bad = sorted(((v, n) for n, v in hits.items() if v > lim), reverse=True)
        if bad:
            problems.append(f"{what}, coming in along {AS.describe(d)}, runs into {bad[0][1]} "
                            f"({bad[0][0]:.3f} mm3 swept through)" + (f" and {len(bad) - 1} more" if len(bad) > 1 else ""))

    for n, s in ans.walls:  # parts come in through the box's opening: the walls are in their way
        placed[n] = s
    for p in asm.parts:
        s = ans.solids[p.name]
        if ans.bearings.get(p.name):  # a part goes in with its bearings pressed in
            s = bd.Compound([s] + ans.bearings[p.name])
        if p.path is not None:
            _go_path(ans, p.name, s, p.path, placed, rows, problems, lim)
        else:
            go(p.name, s, p.direction)
        placed[p.name] = s
        for bid, h in ans.heads.get(p.name, []):
            placed[f"bolt head {bid} (on {p.name})"] = h
    # screws and nuts are brought to their line inside the box: each is held on its line, in the room the brief
    # states (a screw its length plus the room behind where its head ends up, a nut the shank's end plus the room
    # beyond it), and goes in from there; that room must be clear of everything, and inside the box's walls
    room = (task.tolerances.get("moves") or {}).get("screw_room", 5.0)
    for f in asm.fasteners:
        ls = f.length + room if ans.walls else None
        ln = f.length - f.grip + room if ans.walls else None
        go(f"{f.label} screw", f.screw(), f.axis, ls)
        placed[f"{f.label} screw"] = f.screw()
        go(f"{f.label} nut", f.nut(), -f.axis, ln)
        placed[f"{f.label} nut"] = f.nut()
        if ans.walls:
            b = BOLTS[f.size]
            pl = f.plane()
            need = f.length + b.head_h + room
            behind = pl * bd.Pos(0, 0, -need / 2) * bd.Cylinder(b.head_d / 2, need)
            m = AS.NUTS[f.size][1]
            beyond = pl * bd.Pos(0, 0, f.length + (m + room) / 2) * bd.Cylinder(AS.nut_corner_d(f.size) / 2, m + room)
            for what, zone in (("screw", behind), ("nut", beyond)):
                hit = max((volume(zone & s) for _, s in ans.walls), default=0.0)
                if hit > lim:
                    problems.append(f"{f.label}: no room inside the box to bring its {what} to its line (the box's "
                                    f"wall is within {need if what == 'screw' else m + room:g} mm along its axis)")
    worst = max((r["volume"] for r in rows), default=0.0)
    if problems:
        return CheckResult("INSERTION", False, "; ".join(problems), worst, "mm3", {"sweeps": rows})
    order = ", then ".join([(f"{p.name} along {AS.describe(p.direction)}" if p.path is None else
                             f"{p.name} along its path of {len(p.path)} step(s)") for p in asm.parts]
                           + ([f"{len(asm.fasteners)} screws and nuts"] if asm.fasteners else []))
    return CheckResult("INSERTION", True, f"everything goes in clear: {order}", worst, "mm3", {"sweeps": rows})


def _step_motion(st) -> Placement:
    if st[0] == "move":
        return MO.shift(st[1])
    return MO.about(tuple(st[1]), tuple(st[2]), st[3])


def _inverse(pl: Placement) -> Placement:
    R = np.array(pl.rot)
    o = -R.T @ np.asarray(pl.origin)
    return Placement(tuple(float(v) for v in o), tuple(tuple(float(v) for v in r) for r in R.T))


TURN_STEP_MM = 1.0  # a turn of a path is checked in steps that move no point of the part more than this


def _go_path(ans: Answer, name, body, steps, placed, rows, problems, lim) -> None:
    """A part's way in along a path of straight moves and turns (world-fixed), from a start outside the box to
    its place: each move swept exactly (as a straight insertion over the move's length), each turn in small
    steps; the start must lie outside the box."""
    total = Placement((0.0, 0.0, 0.0), ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)))
    motions = [_step_motion(st) for st in steps]
    for m in motions:
        total = total.then(m)
    start = body.moved(_inverse(total).location())
    box = ans.task.meta.get("motion", {}).get("box") or {}
    if box.get("axes"):
        ax = np.array(box["axes"], float)
        lo = np.array([box["inside_s"][0], box["inside_a"][0], box["inside_t"][0]])
        hi = np.array([box["inside_s"][1], box["inside_a"][1], box["inside_t"][1]])
        c = np.asarray(box["origin"], float) + ax.T @ ((lo + hi) / 2)
        rot = tuple(tuple(float(v) for v in r) for r in ax.T)
        inner = bd.Box(*(hi - lo)).moved(Placement(tuple(float(v) for v in c), rot).location())
        v = volume(start & inner)
        if v > lim:
            problems.append(f"{name}'s path does not start outside the box ({v:.1f} mm3 of it inside)")
    cur = start
    for k, (st, m) in enumerate(zip(steps, motions)):
        if st[0] == "move":
            nxt = cur.moved(m.location())
            hits = collisions(nxt, st[1], placed, length=float(np.linalg.norm(st[1])))
        else:
            bb = cur.bounding_box()
            pts = np.array([[x, y, z] for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y) for z in (bb.min.Z, bb.max.Z)])
            a = np.asarray(st[2]) / np.linalg.norm(st[2])
            v_ = pts - np.asarray(st[1])
            reach = float(np.max(np.linalg.norm(v_ - np.outer(v_ @ a, a), axis=1)))
            n = max(1, math.ceil(abs(st[3]) * math.pi / 180 * reach / TURN_STEP_MM))
            hits = {nm: 0.0 for nm in placed}
            for i in range(1, n + 1):
                inter = cur.moved(MO.about(tuple(st[1]), tuple(st[2]), st[3] * i / n).location())
                for nm, s_ in placed.items():
                    if MO.distance(inter, s_) <= MO.CONTACT:
                        hits[nm] = max(hits[nm], volume(inter & s_))
            nxt = cur.moved(m.location())
        for nm, v in hits.items():
            rows.append({"moving": f"{name} (path step {k + 1})", "against": nm, "volume": round(v, 4)})
        bad = sorted(((v, nm) for nm, v in hits.items() if v > lim), reverse=True)
        if bad:
            problems.append(f"{name}, at step {k + 1} of its path ({st[0]}), runs into {bad[0][1]} ({bad[0][0]:.3f} mm3)")
            return
        cur = nxt


# --------------------------------------------------------------------------
# pivots


def cylinders(shape) -> list[dict]:
    """Every cylindrical face: axis point and direction, radius, whether it is a bore
    (concave: the material outside) and its extent along its axis."""
    out = []
    for f in shape.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cylinder:
            continue
        cy = ad.Cylinder()
        ax = cy.Axis()
        loc = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
        dr = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
        p = np.array(tuple(f.position_at(0.5, 0.5)))
        nrm = np.array(tuple(f.normal_at(0.5, 0.5)))
        radial = (p - loc) - dr * float((p - loc) @ dr)
        concave = float(nrm @ radial) < 0
        ts = [float((np.array(tuple(f.position_at(u, v))) - loc) @ dr) for u in (0.0, 0.5, 1.0) for v in (0.0, 1.0)]
        out.append({"loc": loc, "dir": dr, "r": float(cy.Radius()), "bore": concave, "span": (min(ts), max(ts))})
    return out


def _coaxial(cyl, point, axis, off_max, ang_max) -> tuple[float, float] | None:
    """(t0, t1) extent along the declared axis if the cylinder lies on it, else None."""
    a = np.asarray(axis, float)
    cosang = abs(float(cyl["dir"] @ a))
    if math.degrees(math.acos(min(1.0, cosang))) > ang_max:
        return None
    v = cyl["loc"] - np.asarray(point, float)
    if float(np.linalg.norm(v - a * float(v @ a))) > off_max:
        return None
    t = float(v @ a)
    s = 1.0 if float(cyl["dir"] @ a) > 0 else -1.0
    e = sorted((t + s * cyl["span"][0], t + s * cyl["span"][1]))
    return e[0], e[1]


def _pivot_features(ans: Answer, body: str, pv: AS.PivotEntry, st: dict, cache: dict):
    """(pins, bores) of ``body`` on the pivot's axis: [(radius, (t0, t1), what)]."""
    pins, bores = [], []
    for p in ans.asm.parts:
        if ans.asm.body_of(p.name) != body:
            continue
        if p.name not in cache:
            cache[p.name] = cylinders(ans.solids[p.name])
        for c in cache[p.name]:
            got = _coaxial(c, pv.point, pv.axis, st["pivot_offset_max"], st["pivot_angle_max_deg"])
            if got:
                (bores if c["bore"] else pins).append((c["r"], got, f"{'bore' if c['bore'] else 'pin'} on {p.name}"))
    for f in ans.asm.fasteners:
        if ans.asm.body_of(f.held_by) != body:
            continue
        cyl = {"loc": f.head_at, "dir": f.axis, "r": BOLTS[f.size].d / 2, "bore": False, "span": (0.0, f.length)}
        got = _coaxial(cyl, pv.point, pv.axis, st["pivot_offset_max"], st["pivot_angle_max_deg"])
        if got:
            pins.append((cyl["r"], got, f.label))
    return pins, bores


def _pivots(ans: Answer) -> CheckResult:
    st = settings(ans.task)
    cache, rows, problems = {}, [], []
    for pv in ans.asm.pivots:
        b1, b2 = ans.asm.body_of(pv.joins[0]), ans.asm.body_of(pv.joins[1])
        f1, f2 = _pivot_features(ans, b1, pv, st, cache), _pivot_features(ans, b2, pv, st, cache)
        if pv.bearing:
            rows_, prob = _bearing_pivot(ans, pv, f1 if ans.asm.body_of(pv.seat_on) == b2 else f2)
            rows += rows_
            problems += prob
            continue
        best = None
        for (pins, _), (_, bores) in ((f1, f2), (f2, f1)):
            for rp, sp, wp in pins:
                for rb, sb, wb in bores:
                    ov = min(sp[1], sb[1]) - max(sp[0], sb[0])
                    if rb > rp and ov >= OVERLAP_MIN and (best is None or rb - rp < best[0]):
                        best = (rb - rp, wp, wb, ov)
        if best is None:
            problems.append(f"pivot {pv.name}: no round pin in a round bore on its axis between {pv.joins[0]} and "
                            f"{pv.joins[1]} (a pin on one body inside a bore of the other, overlapping by at least "
                            f"{OVERLAP_MIN:g} mm along the axis)")
            continue
        g, wp, wb, ov = best
        rows.append({"pivot": pv.name, "gap": round(g, 4), "pin": wp, "bore": wb, "overlap": round(ov, 3)})
        if g < st["gap_min"] - 1e-6:
            problems.append(f"pivot {pv.name}: the {wp} is only {g:.3f} mm smaller than the {wb} all round; "
                            f"it needs the {st['gap_min']:g} mm printing gap")
    if problems:
        return CheckResult("PIVOTS", False, "; ".join(problems), details={"pivots": rows})
    return CheckResult("PIVOTS", True, "every pivot is a pin in a bore: " + "; ".join(
        f"{r['pivot']} {r['pin']} in {r['bore']}" + (f", {r['gap']:.2f} mm gap" if "gap" in r else "") for r in rows),
        details={"pivots": rows})


PIN_FIT = 0.10  # mm: a pin in a bearing's bore may be this much under the bore's diameter (a slip fit)


def _bearing_pivot(ans: Answer, pv: AS.PivotEntry, other_features) -> tuple[list, list]:
    """A pivot on a catalogue ball bearing: the part holding it is judged by BEARING_SEAT's own measures (bore in
    the fit band covering the width, wall all round, a shoulder at the far face, the inner race free, a straight way
    in), and the other body needs a pin on the axis that fills the bearing's bore (d - 0.1 to d) over 90 % of its
    width."""
    from ..catalogue import BEARINGS
    from ..checks.bearing_seat import WHY, measure

    d, D, B = BEARINGS[pv.bearing]
    pl = Placement(tuple(float(v) for v in pv.point), _frame_rot(pv.axis))
    m = measure(ans.solids[pv.seat_on], pl, d, D, B, (), ans.task.tolerances)
    problems = []
    for key in ("bore", "wall", "shoulder", "inner_race_free", "way_in"):
        if not m[key]["ok"]:
            x = m[key]
            problems.append(f"pivot {pv.name}: bearing {pv.bearing} in {pv.seat_on}: " + WHY[key].format(
                D=D, Dmax=D + 0.1, covered=x.get("covered_mm", 0), B=B, wall=x.get("min_mm", 0), share=x.get("share", 0),
                r=x.get("radius_mm", 0), what="the part is in the way"))
    pins, _ = other_features
    fit = [(r, sp, w) for r, sp, w in pins if d / 2 - PIN_FIT / 2 - 1e-6 <= r <= d / 2 + 1e-6]
    best = max((min(sp[1], B) - max(sp[0], 0.0) for _, sp, _ in fit), default=0.0)
    if best < 0.9 * B - 1e-6:
        problems.append(f"pivot {pv.name}: no pin of {d - PIN_FIT:g}-{d:g} mm on the axis fills bearing {pv.bearing}'s "
                        f"bore over 90 % of its {B:g} mm width ({best:.1f} mm)")
    row = {"pivot": pv.name, "pin": f"pin in bearing {pv.bearing}", "bore": f"{pv.seat_on}'s seat", "bearing": pv.bearing,
           "seat": {k: m[k]["ok"] for k in ("bore", "wall", "shoulder", "inner_race_free", "way_in")}}
    return [row], problems


def _frame_rot(axis):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    x = np.cross(a, [0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [1.0, 0.0, 0.0])
    x = x / np.linalg.norm(x)
    y = np.cross(a, x)
    return tuple(tuple(float(v) for v in row) for row in np.column_stack([x, y, a]))


# --------------------------------------------------------------------------
# the mechanism


def build_linkage(ans: Answer) -> LK.Linkage:
    asm = ans.asm
    return LK.build(asm.bodies(), [(v.name, asm.body_of(v.joins[0]), asm.body_of(v.joins[1]), v.point, v.axis)
                                   for v in asm.pivots], asm.drive)


def _mobility(ans: Answer):
    try:
        lk = build_linkage(ans)
    except LK.LinkageError as e:
        return None, CheckResult("MOBILITY", False, str(e))
    free, with_drive = lk.mobility()
    nb, nj = len(lk.bodies), len(lk.pivots)
    if free != 1:
        word = "cannot move at all" if free == 0 else f"can move {free} independent ways"
        return None, CheckResult("MOBILITY", False, f"the mechanism {word}: {nb} bodies and {nj} pivots leave {free} "
                                 "free motion(s); it must have exactly 1", float(free), "motions")
    if with_drive != lk.n:
        return None, CheckResult("MOBILITY", False, f"the drive at {ans.asm.drive} does not control the motion",
                                 float(free), "motions")
    return lk, CheckResult("MOBILITY", True, f"exactly one way to move ({nb} bodies, {nj} pivots), driven at "
                           f"{ans.asm.drive}", 1.0, "motions")


def _corners(shape) -> np.ndarray:
    bb = shape.bounding_box()
    return np.array([[x, y, z] for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y) for z in (bb.min.Z, bb.max.Z)])


def pose_error(pl: Placement, tg: Placement) -> tuple[float, float]:
    d = float(np.linalg.norm(np.asarray(pl.origin) - np.asarray(tg.origin)))
    R = np.array(pl.rot) @ np.array(tg.rot).T
    return d, math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1) / 2))))


def check_steps(ans: Answer, lk: LK.Linkage, path, st) -> list[int]:
    """Indices into ``path`` at which collisions are checked: between two of them no corner of any
    body's box moves more than ``step_move_max`` mm, and the drive turns at most ``step_max_deg``."""
    corners = {k: np.vstack([_corners(s) for _, s in ans.pieces[b]]) for k, b in enumerate(lk.bodies) if k}

    def at(i):
        out = {}
        for k, c in corners.items():
            pl = lk.placement(path[i][1], k)
            out[k] = c @ np.array(pl.rot).T + np.array(pl.origin)
        return out

    idx, ref = [0], at(0)
    for i in range(1, len(path)):
        now = at(i)
        span = max(float(np.max(np.linalg.norm(now[k] - ref[k], axis=1))) for k in now)
        if span > st["step_move_max"] or abs(path[i][0] - path[idx[-1]][0]) > st["step_max_deg"] + 1e-9:
            j = i - 1 if i - 1 > idx[-1] else i
            idx.append(j)
            ref = at(j)
    if idx[-1] != len(path) - 1:
        idx.append(len(path) - 1)
    return idx


def _motion(ans: Answer, lk: LK.Linkage) -> list[CheckResult]:
    task, st = ans.task, settings(ans.task)
    lim = task.tolerances["overlap_volume_max"]
    path, stop = lk.sweep(ans.asm.range_deg, FINE_DEG)
    b_idx = lk.bodies.index("B")
    b0 = task.poses[0]["B"]
    steps = check_steps(ans, lk, path, st)
    clashes, rows = [], []
    closest = (math.inf, 0.0, "", "")
    for i in steps:
        deg, q = path[i]
        placed = {}
        for k, b in enumerate(lk.bodies):
            pl = lk.placement(q, k)
            placed[b] = [(n, s if k == 0 else s.moved(pl.location())) for n, s in ans.pieces[b]]
        step_min = (math.inf, "", "")
        for x in range(len(lk.bodies)):
            for y in range(x + 1, len(lk.bodies)):
                for na, sa in placed[lk.bodies[x]]:
                    for nb_, sb in placed[lk.bodies[y]]:
                        if na.startswith("fixture") and nb_.startswith("fixture"):
                            continue
                        if ans.bearing_pin_body.get(na) == lk.bodies[y] or ans.bearing_pin_body.get(nb_) == lk.bodies[x]:
                            continue  # a bearing and the pin that runs in it
                        dd = MO.distance(sa, sb)
                        if dd <= MO.CONTACT:
                            v = volume(sa & sb)
                            if v > lim:
                                clashes.append((deg, v, na, nb_))
                            dd = 0.0
                        if dd < step_min[0]:
                            step_min = (dd, na, nb_)
        rows.append({"deg": round(deg, 4), "gap": round(step_min[0], 4), "a": step_min[1], "b": step_min[2]})
        if step_min[0] < closest[0]:
            closest = (step_min[0], deg, step_min[1], step_min[2])
    n = len(steps)
    out = []
    if stop is not None:
        out.append(CheckResult("MOTION", False, stop[1] + f" (declared range {ans.asm.range_deg:g} deg)",
                               round(stop[0], 3), "deg", {"steps": rows}))
    elif clashes:
        a0, v, na, nb_ = clashes[0]
        degs = sorted({c[0] for c in clashes}, key=abs)
        out.append(CheckResult("MOTION", False, f"driven, {na} runs into {nb_} at {a0:.1f} deg of drive ({v:.3f} mm3 "
                               f"overlap); clashes at {len(degs)} of {n} steps ({degs[0]:.1f} to {degs[-1]:.1f} deg)",
                               round(max(c[1] for c in clashes), 4), "mm3",
                               {"steps": rows, "clashes": [[round(c[0], 3), round(c[1], 4), c[2], c[3]] for c in clashes]}))
    else:
        out.append(CheckResult("MOTION", True, f"driven from 0 to {ans.asm.range_deg:g} deg in {n} steps, the "
                               "mechanism closes at every step and nothing collides", 0.0, "mm3", {"steps": rows}))
    if clashes:
        out.append(CheckResult("GAP", None, "not judged: bodies collide (see MOTION)"))
    else:
        g, ang, na, nb_ = closest
        if g < st["gap_min"] - 1e-6:
            low = [r for r in rows if r["gap"] < st["gap_min"] - 1e-6]
            out.append(CheckResult("GAP", False, f"{na} comes within {g:.3f} mm of {nb_} at {ang:.1f} deg of drive; "
                                   f"parts that move against each other need at least {st['gap_min']:g} mm (too close "
                                   f"at {len(low)} of {n} steps)", round(g, 4), "mm"))
        else:
            out.append(CheckResult("GAP", True, f"the closest any two bodies come is {g:.3f} mm ({na} to {nb_} at "
                                   f"{ang:.1f} deg); at least {st['gap_min']:g} mm", round(g, 4), "mm"))
    out.append(_poses(ans, lk, path, stop, b0, b_idx, st))
    return out


def _poses(ans, lk, path, stop, b0, b_idx, st) -> CheckResult:
    """B through the required poses, in order: the best sample, then refined between its neighbours."""

    def score(e):
        return max(e[0] / st["pose_position"], e[1] / st["pose_angle_deg"])

    bpl = [b0.then(lk.placement(q, b_idx)) for _, q in path]
    rows, problems, after = [], [], -1
    for k, tg in enumerate(targets(ans.task)):
        cand = list(range(after + 1, len(path)))
        if not cand:
            problems.append(f"pose {k + 1}: the motion has ended before it")
            continue
        errs = {i: pose_error(bpl[i], tg) for i in cand}
        i = min(cand, key=lambda i: score(errs[i]))
        lo, hi = path[max(i - 1, 0)][0], path[min(i + 1, len(path) - 1)][0]
        e_best, deg_best = errs[i], path[i][0]
        for _ in range(40):
            res = []
            for mdeg in (lo + (hi - lo) / 3, hi - (hi - lo) / 3):
                qq, r, _ = lk.solve(math.radians(mdeg), path[i][1])
                ee = pose_error(b0.then(lk.placement(qq, b_idx)), tg) if r < 1e-6 else (math.inf, math.inf)
                res.append((score(ee), ee, mdeg))
            if res[0][0] < res[1][0]:
                hi = lo + 2 * (hi - lo) / 3
            else:
                lo = lo + (hi - lo) / 3
            for sc, ee, mdeg in res:
                if sc < score(e_best):
                    e_best, deg_best = ee, mdeg
        rows.append({"pose": k + 1, "at_deg": round(deg_best, 3), "position_mm": round(e_best[0], 4),
                     "angle_deg": round(e_best[1], 4)})
        if e_best[0] > st["pose_position"] or e_best[1] > st["pose_angle_deg"]:
            problems.append(f"pose {k + 1}: B comes no closer than {e_best[0]:.2f} mm and {e_best[1]:.2f} deg (at "
                            f"{deg_best:.1f} deg of drive); it must come within {st['pose_position']:g} mm and "
                            f"{st['pose_angle_deg']:g} deg")
        after = i
    if stop is not None and problems:
        problems.append(f"the motion stopped at {path[-1][0]:.1f} deg")
    if problems:
        return CheckResult("POSES", False, "; ".join(problems), details={"poses": rows})
    return CheckResult("POSES", True, "B passes through every pose in order: " + "; ".join(
        f"pose {r['pose']} at {r['at_deg']:.1f} deg of drive, within {r['position_mm']:.3f} mm and "
        f"{r['angle_deg']:.3f} deg" for r in rows), details={"poses": rows})


# --------------------------------------------------------------------------
# each pivot, pulled and tilted


def _free(sides, f, upto, step, start=0.0):
    t = start
    while t < upto - 1e-9:
        t = min(upto, t + step)
        if sides.touching(f(t)):
            return t
    return None


def _bisect(sides, f, hi, n=6, lo=0.0):
    for _ in range(n):
        mid = (lo + hi) / 2
        if sides.touching(f(mid)):
            hi = mid
        else:
            lo = mid
    return lo


def _interlock(ans: Answer) -> list[CheckResult]:
    st = settings(ans.task)
    lim = {"mm": st["play_max"], "deg": st["tilt_max_deg"]}
    loose = {"mm": st["loose_shift"], "deg": st["loose_tilt"]}
    probe = {"mm": 0.25, "deg": 1.0}
    escapes, too_free, rows = [], [], []
    for pv in ans.asm.pivots:
        b1, b2 = ans.asm.body_of(pv.joins[0]), ans.asm.body_of(pv.joins[1])
        walls = {n for n, _ in ans.walls}  # a pivot must hold by itself: the box's walls do not count
        sides = MO.Sides([x for x in ans.pieces[b1] if x[0] not in walls], [x for x in ans.pieces[b2] if x[0] not in walls],
                         np.asarray(pv.point), np.asarray(pv.axis),
                         frozenset({("fixture A", "fixture B"), ("fixture B", "fixture A")}))
        for name, unit, vec, f in MO.moves(pv.axis, pv.point, "all", 1.0):
            L = loose[unit]
            if sides.touching(f(L)):
                continue
            if _free(sides, f, L, probe[unit]) is None:
                escapes.append((pv.name, name, unit, vec, L))
        for name, unit, vec, f in MO.moves(pv.axis, pv.point, "principal", 1.0):
            L = lim[unit]
            if sides.touching(f(L)):
                rows.append({"pivot": pv.name, "move": name, "unit": unit, "free": round(_bisect(sides, f, L), 4)})
                continue
            hit = _free(sides, f, loose[unit], probe[unit], start=L)
            if hit is not None:
                hit = round(_bisect(sides, f, hit, n=6, lo=max(L, hit - probe[unit])), 3)
                too_free.append((pv.name, name, unit, vec, hit))
                rows.append({"pivot": pv.name, "move": name, "unit": unit, "free": hit})
    if escapes:
        escapes.sort(key=lambda e: (e[2] == "deg", e[1].count("+1") + e[1].count("-1")))
        pv, name, unit, vec, L = escapes[0]
        what = "tilted" if unit == "deg" else "moved"
        il = CheckResult("INTERLOCK", False, f"pivot {pv} comes apart: its two bodies can be {what} {L:g} {unit} "
                         f"({name}, world {AS.describe(vec)}) without meeting"
                         + (f"; {len(escapes) - 1} more ways apart" if len(escapes) > 1 else ""),
                         float(len(escapes)), "ways apart", {"escapes": [[e[0], e[1]] for e in escapes]})
    else:
        il = CheckResult("INTERLOCK", True, f"no pivot comes apart: each pivot's two bodies, shifted {loose['mm']:g} mm "
                         f"in any of 26 directions or tilted {loose['deg']:g} deg, meet", 0.0, "ways apart")
    if too_free:
        pv, name, unit, vec, hit = too_free[0]
        pl = CheckResult("PLAY", False, f"too much play at pivot {pv}: its bodies move {hit:.2f} {unit} ({name}, world "
                         f"{AS.describe(vec)}) before they meet; at most {lim[unit]:g} {unit}"
                         + (f" ({len(too_free) - 1} more)" if len(too_free) > 1 else ""), hit, unit, {"play": rows})
    elif escapes:
        pl = CheckResult("PLAY", None, "not judged: a pivot comes apart (see INTERLOCK)", details={"play": rows})
    else:
        sh = max((r["free"] for r in rows if r["unit"] == "mm"), default=0.0)
        tl = max((r["free"] for r in rows if r["unit"] == "deg"), default=0.0)
        pl = CheckResult("PLAY", True, f"play within limits at every pivot: shift at most {sh:.2f} mm (limit "
                         f"{lim['mm']:g}), tilt at most {tl:.2f} deg (limit {lim['deg']:g})", sh, "mm", {"play": rows})
    return [il, pl]
