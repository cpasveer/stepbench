"""Grading a "locks" answer: B goes in along the axis, turns, and is then locked.

The answer is two STEP files, ``part_A.step`` (bolted to A) and
``part_B.step`` (bolted to B), modelled with B at its start pose: pushed in,
not yet turned. The *fixed side* is A, part_A and its bolt heads; the
*moving side* is B, part_B and its bolt heads. Everything is exact:

  part_A ... / part_B ...  each part against its own fixture, as at levels 1
            and 2: holes, seating, seat inside, no interference, clean
            modelling, filament printing, and bolts reachable by a straight
            tool (each part is bolted to its fixture before B comes in, so
            only its own fixture is in the tool's way)
  SEATED    at the start pose the two sides do not overlap
  CONNECT   the moving side, brought in from outside along the insertion
            direction to the start pose, sweeps through nothing of the fixed
            side (exact swept volume)
  TURN      turned about the axis from 0 to the stated angle in small steps,
            the moving side never overlaps the fixed side
  GAP       at every step of the turn the two sides stay the printing gap apart
  LOCKED    turned, the moving side pulled back along the way it came meets
            the fixed side within the stated play
  UNLOCKS   not yet turned, the moving side pulled back the same way meets
            nothing: only the turn locks it
  INTERLOCK turned, the moving side cannot come loose in any of 26
            directions or by tilting
  PLAY      turned, its free shift along the axis and square to it, and its
            free tilt, are within the stated limits
  LOCK_STRENGTH  turned and pulled back with the stated load, the plastic
            where the two parts bear on each other carries it (4a's rules and
            allowables, ``catalogue.plastic_allowables``): see ``_strength``

Score: the printed weight of both parts (level 3's measure), among answers
that pass.
"""
from __future__ import annotations

import math
import os

import build123d as bd
import numpy as np

from ..checks.base import CheckResult, Context, volume
from ..geometry import read_step
from ..grade import Report, grade as grade_part
from ..moves import motion as MO
from ..moves.grade import _bisect, _free, _grips, fixture_heads
from ..moves.sweep import collisions
from ..task import Task
from .generate import FILES


def settings(task: Task) -> dict:
    return task.tolerances["locks"]


def subtask(task: Task, fx: str) -> Task:
    """One part's judgement against its own fixture, as at level 3 for bolt access (no obstacles:
    each part is bolted to its fixture before the two meet)."""
    tol = {k: v for k, v in task.tolerances.items() if k != "locks"}
    t = Task(f"{task.id}/part_{fx}", 3, task.seed, {fx: task.parts[fx]}, [{fx: task.poses[0][fx]}], tol, {},
             generator_version=task.generator_version)
    t.root = task.root
    return t


def _reach(pieces, point, axis) -> float:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    far = 0.0
    for _, s in pieces:
        bb = s.bounding_box()
        for x in (bb.min.X, bb.max.X):
            for y in (bb.min.Y, bb.max.Y):
                for z in (bb.min.Z, bb.max.Z):
                    v = np.array([x, y, z]) - np.asarray(point, float)
                    far = max(far, float(np.linalg.norm(v - a * float(v @ a))))
    return far


def grade(task: Task, folder: str, *, manufacturing: bool = True, progress=None) -> Report:
    """Grade ``part_A.step`` and ``part_B.step`` in ``folder``."""
    say = progress or (lambda s: None)
    st, lk = settings(task), task.meta["lock"]
    checks: list[CheckResult] = []
    solids = {}
    from ..checks import file_valid

    for fx, fn in FILES.items():
        path = os.path.join(folder, fn)
        r = file_valid.run(Context(task, path)) if os.path.exists(path) else CheckResult("FILE_VALID", False, f"no file {fn}")
        if r.ok:
            got = read_step(path).solids()
            solids[fx] = got[0] if len(got) == 1 else None
            if solids[fx] is None:
                r = CheckResult("FILE_VALID", False, "not exactly one solid")
        if not r.ok:
            r.name = f"part_{fx} FILE_VALID"
            checks.append(r)
    if len(solids) < 2 or any(s is None for s in solids.values()):
        return Report(task.id, False, checks)

    heads, reps = {}, {}
    for fx, fn in FILES.items():
        say(f"grading part_{fx}")
        sub = subtask(task, fx)
        if not manufacturing:
            sub.tolerances["required_checks"] = [c for c in sub.tolerances.get("required_checks", []) if c != "MANUFACTURING"]
        rep = grade_part(sub, os.path.join(folder, fn), optional=False, score=False)
        heads[fx] = fixture_heads(task, fx, _grips(rep))
        reps[fx] = rep
        for c in rep.checks:
            c.name = f"part_{fx} {c.name}"
            checks.append(c)

    fixed, moving = lk["fixed"], lk["moving"]
    fxs = task.fixed_solids(0)

    def side(n):
        hs = [h for _, h in heads[n]]
        return [(f"fixture {n}", fxs[n]), (f"part_{n}" + (" (with its bolt heads)" if hs else ""),
                                            bd.Compound([solids[n]] + hs) if hs else solids[n])]

    sides = MO.Sides(side(fixed), side(moving), np.asarray(lk["axis_point"], float), np.asarray(lk["axis_direction"], float),
                     frozenset({(f"fixture {moving}", f"fixture {fixed}")}))
    say("seated")
    checks.append(_seated(task, sides))
    say("connect")
    checks.append(_connect(task, sides, lk))
    say("turn")
    checks += _turn(task, sides, lk, st)
    say("locked")
    checks += _locked(task, sides, lk, st)
    say("strength")
    checks.append(_strength(task, solids, lk, next(c for c in checks if c.name == "LOCKED")))
    passed = all(c.ok for c in checks if c.required)
    info = {"parts": {f"part_{k}": round(float(s.volume), 3) for k, s in solids.items()},
            "volume": round(sum(float(s.volume) for s in solids.values()), 3)}
    bb = bd.Compound(list(solids.values())).bounding_box()
    info.update(bbox_min=[round(v, 3) for v in bb.min], bbox_max=[round(v, 3) for v in bb.max],
                bbox_size=[round(v, 3) for v in bb.size])
    from .. import material_score

    if passed and material_score.asked(task):
        info["material_used"] = _printed(task, folder, reps)
    return Report(task.id, passed, checks, info)


def _printed(task: Task, folder: str, reps: dict) -> dict:
    """The score: the plastic the printer lays down for both parts, each in the build direction its
    printing check chose (level 3's measure; as 4a scores its two parts)."""
    from .. import material_score

    per, total, grams = {}, 0.0, 0.0
    for fx, fn in FILES.items():
        fdm = next((c for c in reps[fx].checks if c.name.split(" ")[-1].startswith("FDM_")), None)
        direction = (fdm.details or {}).get("direction") if fdm else None
        try:
            m = material_score.material_used(task, os.path.join(folder, fn), direction)
        except Exception as e:  # noqa: BLE001 - no figure rather than a wrong one
            return {"error": f"part_{fx}: {type(e).__name__}: {e}"}
        per[f"part_{fx}"] = m
        total += m["used_cm3"]
        grams += m["grams"]
    return {"used_cm3": round(total, 3), "grams": round(grams, 2),
            "measure": material_score.MEASURE[material_score.process_of(task)] + ", both parts", "parts": per}


CONTACT_DEPTH = 0.05  # mm: how far past first touch the parts are pressed to find where they bear
ROD = 0.05  # mm: radius of the probe that measures the plastic behind a contact


def _behind(solid, start, direction) -> float:
    """How much plastic lies behind ``start`` in ``direction``, straight on, before the first gap (mm)."""
    rod = bd.Plane(origin=tuple(start), z_dir=tuple(direction)) * bd.Pos(0, 0, 100.0) * bd.Cylinder(ROD, 200.0)
    got = solid & rod
    try:
        pieces = got.solids()
    except Exception:  # noqa: BLE001
        return 0.0
    d = np.asarray(direction, float)
    best = 0.0
    for p in pieces:
        bb = p.bounding_box()
        ts = [float((np.array([x, y, z]) - start) @ d) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)
              for z in (bb.min.Z, bb.max.Z)]
        if min(ts) <= 0.2:  # the piece the probe starts in
            best = max(best, max(ts))
    return best


def _on(patch) -> np.ndarray:
    """A point of the patch near its middle: its centre where that lies in it, else the patch's point
    nearest the centre (a patch curved round an axis has its centre off it, in the empty space)."""
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape

    c = np.array(tuple(patch.center()))
    d = BRepExtrema_DistShapeShape(bd.Vertex(*c).wrapped, patch.wrapped)
    if not d.IsDone() or d.Value() < 1e-9:
        return c
    q = d.PointOnShape2(1)
    return np.array([q.X(), q.Y(), q.Z()])


def _strength(task: Task, solids: dict, lk: dict, locked: CheckResult) -> CheckResult:
    """LOCK_STRENGTH. Turned, B is pulled back with the stated load. Where the two parts bear on
    each other is found exactly: part_B is pulled back past the first touch (the measured play) by
    ``CONTACT_DEPTH`` and its overlap with part_A is each contact patch. Judged as 4a judges its joint,
    in the printed plastic, with the same allowables (``catalogue.plastic_allowables``):

      bearing   the load over the patches' area projected square to the pull, against the allowable
                bearing stress (pressure on an inclined face is taken on its projection);
      shear     on each part, each patch's share of the load punching a plug of the patch's area out
                through the plastic behind it, straight along the pull (as a bolt head punches through
                at 4a), against the allowable shear stress. The plug's outline is taken as round, the
                shortest outline for its area: no credit for a long thin patch.

    The largest ratio is the utilisation; at most 1 passes."""
    from ..catalogue import plastic_allowables

    F = float(lk["pull_load_N"])
    if locked.ok is not True:
        return CheckResult("LOCK_STRENGTH", None, "not judged: the turn does not lock (see LOCKED)", group="sturdy")
    al = plastic_allowables()
    back = -np.asarray(lk["insert_direction"], float)
    turned = MO.about(lk["axis_point"], lk["axis_direction"], float(lk["turn_deg"]))
    pl = turned.then(MO.shift((float(locked.value) + CONTACT_DEPTH) * back))
    a_part, b_part = solids["A"], solids["B"].moved(pl.location())
    try:
        patches = [p for p in (a_part & b_part).solids() if p.volume > 1e-9]
    except Exception:  # noqa: BLE001
        patches = []
    rows = []
    for p in patches:
        proj = 0.0
        for f in p.faces():
            nrm = np.array(tuple(f.normal_at(f.center())))
            c = float(nrm @ back)
            if c > 0:
                proj += float(f.area) * c
        if proj > 1e-6:
            rows.append({"centre": _on(p), "area": proj})
    area = sum(r["area"] for r in rows)
    if area <= 0:
        return CheckResult("LOCK_STRENGTH", False, "turned and pulled back, the two parts do not bear on each other "
                           "over any area (they meet only at a point or edge)", None, group="sturdy")
    u_bear = F / (al["bearing_MPa"] * area)
    worst = (u_bear, f"bearing on {len(rows)} contact patch(es), {area:.1f} mm2 projected")
    out = []
    for r in rows:
        share = F * r["area"] / area
        ring = 2.0 * math.sqrt(math.pi * r["area"])
        # the pull drags B's plastic back and pushes A's plastic on: each part's plastic behind its contact
        for part, solid, d in (("part_A", a_part, back), ("part_B", b_part, -back)):
            t = _behind(solid, r["centre"], d)
            u = share / (al["shear_MPa"] * ring * t) if t > 0 else math.inf
            out.append({"part": part, "patch_mm2": round(r["area"], 3), "behind_mm": round(t, 3), "utilisation": round(u, 4)})
            if u > worst[0]:
                worst = (u, f"{part}: a {r['area']:.1f} mm2 patch punching through {t:.2f} mm of plastic")
    u, what = worst
    msg = (f"{F:g} N pull ({lk['pull_load_kg']:g} kg): utilisation {u:.3f}, worst {what} (allowables "
           f"{al['bearing_MPa']:g} MPa bearing, {al['shear_MPa']:g} MPa shear)")
    return CheckResult("LOCK_STRENGTH", u <= 1.0, msg if u <= 1.0 else "the lock is too weak: " + msg, round(u, 4), "",
                       {"patches": [{"area_mm2": round(r["area"], 3)} for r in rows], "shear": out,
                        "bearing_utilisation": round(u_bear, 4), "allowables": al}, group="sturdy")


def grade_folder(task_dir: str, answer: str | None = None, **kw) -> Report:
    task = Task.load(os.path.join(task_dir, "task.json"))
    return grade(task, answer or task_dir, **kw)


def _seated(task, sides) -> CheckResult:
    lim = task.tolerances["overlap_volume_max"]
    worst = (0.0, "", "")
    for mn, ms, fn, fs in sides.pairs(None):
        if MO.distance(ms, fs) <= MO.CONTACT:
            v = volume(ms & fs)
            if v > worst[0]:
                worst = (v, mn, fn)
    if worst[0] > lim:
        return CheckResult("SEATED", False, f"at the start pose {worst[1]} and {worst[2]} overlap by {worst[0]:.3f} mm3",
                           round(worst[0], 4), "mm3")
    return CheckResult("SEATED", True, "at the start pose the two sides do not overlap", round(worst[0], 4), "mm3")


def _connect(task, sides, lk) -> CheckResult:
    lim = task.tolerances["overlap_volume_max"]
    d = np.asarray(lk["insert_direction"], float)
    body = bd.Compound([s for _, s in sides.moving])
    hits = collisions(body, d, dict(sides.fixed))
    bad = sorted(((v, n) for n, v in hits.items() if v > lim), reverse=True)
    if bad:
        return CheckResult("CONNECT", False, f"coming in along ({', '.join(f'{x:g}' for x in d)}), B's side sweeps "
                           f"through {bad[0][1]} ({bad[0][0]:.3f} mm3)", bad[0][0], "mm3", {"sweeps": hits})
    return CheckResult("CONNECT", True, "B and its part come in straight along the axis clear of A's side",
                       max(hits.values(), default=0.0), "mm3", {"sweeps": hits})


def _turn(task, sides, lk, st) -> list[CheckResult]:
    lim = task.tolerances["overlap_volume_max"]
    th = float(lk["turn_deg"])
    angles = MO.steps(abs(th), _reach(sides.moving, lk["axis_point"], lk["axis_direction"]), st["step_max_deg"],
                      st["step_move_max"])
    clashes, rows = [], []
    closest = (math.inf, 0.0, "", "")
    for a in angles:
        deg = math.copysign(a, th)
        pl = MO.about(lk["axis_point"], lk["axis_direction"], deg)
        dd, mn, fn = sides.closest(pl)
        if dd <= MO.CONTACT:
            v, mn2, fn2 = 0.0, mn, fn
            for m_, ms, f_, fs in sides.pairs(pl):
                if MO.distance(ms, fs) <= MO.CONTACT:
                    vv = volume(ms & fs)
                    if vv > v:
                        v, mn2, fn2 = vv, m_, f_
            if v > lim:
                clashes.append((deg, v, mn2, fn2))
            dd = 0.0
        rows.append({"deg": round(deg, 4), "gap": round(dd, 4)})
        if dd < closest[0]:
            closest = (dd, deg, mn, fn)
    n = len(angles)
    if clashes:
        d0, v, mn, fn = clashes[0]
        return [CheckResult("TURN", False, f"turning, {mn} runs into {fn} at {d0:.1f} deg ({v:.3f} mm3 overlap); clashes "
                            f"at {len(clashes)} of {n} steps", round(max(c[1] for c in clashes), 4), "mm3", {"steps": rows}),
                CheckResult("GAP", None, "not judged: the turn clashes (see TURN)")]
    out = [CheckResult("TURN", True, f"B turns through {th:g} deg clear of A's side ({n} steps)", 0.0, "mm3", {"steps": rows})]
    g, deg, mn, fn = closest
    if g < st["gap_min"] - 1e-6:
        out.append(CheckResult("GAP", False, f"{mn} comes within {g:.3f} mm of {fn} at {deg:.1f} deg; surfaces that move "
                               f"against each other need at least {st['gap_min']:g} mm", round(g, 4), "mm"))
    else:
        out.append(CheckResult("GAP", True, f"the closest the two sides come over the turn is {g:.3f} mm (at {deg:.1f} deg); "
                               f"at least {st['gap_min']:g} mm", round(g, 4), "mm"))
    return out


def _locked(task, sides, lk, st) -> list[CheckResult]:
    out = []
    back = -np.asarray(lk["insert_direction"], float)
    turned = MO.about(lk["axis_point"], lk["axis_direction"], float(lk["turn_deg"]))
    pull = lambda base: (lambda t: base.then(MO.shift(t * back)))  # noqa: E731
    f = pull(turned)
    L, loose = st["play_max"], st["loose_shift"]
    if sides.touching(f(L)):
        free = _bisect(sides, f, L, n=7)
        out.append(CheckResult("LOCKED", True, f"turned, B pulled back meets A's side after {free:.2f} mm (at most "
                               f"{L:g} mm)", round(free, 3), "mm"))
    else:
        hit = _free(sides, f, loose, 0.25, start=L)
        if hit is None:
            out.append(CheckResult("LOCKED", False, f"turned, B can still be pulled back {loose:g} mm without meeting A's "
                                   "side: the turn does not lock it", None, "mm"))
        else:
            hit = _bisect(sides, f, hit, n=6, lo=max(L, hit - 0.25))
            out.append(CheckResult("LOCKED", False, f"turned, B pulled back moves {hit:.2f} mm before it meets A's side; "
                                   f"at most {L:g} mm", round(hit, 3), "mm"))
    g0 = pull(MO.shift((0.0, 0.0, 0.0)))
    hit = _free(sides, g0, loose, 0.25)
    if hit is None:
        out.append(CheckResult("UNLOCKS", True, f"not yet turned, B pulls straight back {loose:g} mm without meeting "
                               "anything: only the turn locks it", None, "mm"))
    else:
        hit = _bisect(sides, g0, hit, n=6, lo=max(0.0, hit - 0.25))
        out.append(CheckResult("UNLOCKS", False, f"not yet turned, B pulled back already meets A's side after {hit:.2f} mm: "
                               "it is locked before the turn", round(hit, 3), "mm"))
    # turned: every other direction
    lim = {"mm": st["play_max"], "deg": st["tilt_max_deg"]}
    lo_ = {"mm": loose, "deg": st["loose_tilt"]}
    probe = {"mm": 0.25, "deg": 1.0}
    escapes, too_free, rows = [], [], []
    P, A = lk["axis_point"], lk["axis_direction"]
    for name, unit, vec, mv in MO.moves(A, P, "all", 1.0):
        h = lambda t, mv=mv: turned.then(mv(t))  # noqa: E731
        if sides.touching(h(lo_[unit])):
            continue
        if _free(sides, h, lo_[unit], probe[unit]) is None:
            escapes.append((name, unit, vec))
    for name, unit, vec, mv in MO.moves(A, P, "principal", 1.0):
        h = lambda t, mv=mv: turned.then(mv(t))  # noqa: E731
        if sides.touching(h(lim[unit])):
            rows.append({"move": name, "unit": unit, "free": round(_bisect(sides, h, lim[unit]), 4)})
            continue
        hit = _free(sides, h, lo_[unit], probe[unit], start=lim[unit])
        if hit is not None:
            hit = round(_bisect(sides, h, hit, n=6, lo=max(lim[unit], hit - probe[unit])), 3)
            too_free.append((name, unit, vec, hit))
    if escapes:
        escapes.sort(key=lambda e: (e[1] == "deg", e[0].count("+1") + e[0].count("-1")))
        name, unit, vec = escapes[0]
        out.append(CheckResult("INTERLOCK", False, f"turned, B comes loose: {'tilted' if unit == 'deg' else 'moved'} "
                               f"{lo_[unit]:g} {unit} ({name}, world ({', '.join(f'{x:.3f}' for x in vec)})) it meets "
                               "nothing", float(len(escapes)), "ways out"))
    else:
        out.append(CheckResult("INTERLOCK", True, f"turned, B cannot come loose: shifted {loose:g} mm in any of 26 "
                               f"directions or tilted {st['loose_tilt']:g} deg, it meets A's side", 0.0, "ways out"))
    if too_free:
        name, unit, vec, hit = too_free[0]
        out.append(CheckResult("PLAY", False, f"turned, B moves {hit:.2f} {unit} ({name}) before it meets A's side; at most "
                               f"{lim[unit]:g} {unit}", hit, unit, {"play": rows}))
    elif escapes:
        out.append(CheckResult("PLAY", None, "not judged: B comes loose (see INTERLOCK)"))
    else:
        sh = max((r["free"] for r in rows if r["unit"] == "mm"), default=0.0)
        tl = max((r["free"] for r in rows if r["unit"] == "deg"), default=0.0)
        out.append(CheckResult("PLAY", True, f"turned, B shifts at most {sh:.2f} mm (limit {lim['mm']:g}) and tilts at most "
                               f"{tl:.2f} deg (limit {lim['deg']:g})", sh, "mm", {"play": rows}))
    return out
