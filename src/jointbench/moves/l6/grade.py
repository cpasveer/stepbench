"""Grading level 6: the moves grader (``jointbench.moves.grade``: parts, assembly, pivots, mobility,
motion, gaps, poses, pivots pulled and tilted -- with the closed box's walls standing with the fixed
fixture, in every collision, gap, insertion and key-access check), plus, at B's home pose:

  (The answer is modelled and assembled with B at home; the drive takes B from there out to the start.)

  LATCH          pulled straight down (the stated direction) on its own -- B, the parts bolted to it and
                 their screws, without the links -- B's side meets the fixed side within the stated play:
                 the linkage's path is the only way out.
  LATCH_STRENGTH the stated load, pulling B's side down there, carried by the plastic where the two sides
                 then bear on each other (4d's rule, 4a's allowables: bearing on the patches' projected area,
                 and a plug of each patch's area punched out through the plastic behind it).
  PIVOT_SEATS    the same load through every pivot: the pin bearing on each body's bore (the load over the
                 pin's diameter times the bore's length, against the allowable bearing stress, as 4a judges a
                 bolt bearing on a hole wall).
"""
from __future__ import annotations

import math
import os

import numpy as np

from ...catalogue import BOLTS, plastic_allowables
from ...checks.base import CheckResult
from ...task import Task
from ...locks.grade import CONTACT_DEPTH, _behind
from .. import motion as MO
from ..grade import _bisect, _free, _pivot_features, grade as grade_moves, settings


def grade(task: Task, assembly_path: str, *, manufacturing: bool = True, progress=None):
    return grade_moves(task, assembly_path, manufacturing=manufacturing, progress=progress, extra=_home_checks)


def grade_folder(task_dir: str, assembly: str | None = None, **kw):
    from ..assembly import ASSEMBLY_FILE

    task = Task.load(os.path.join(task_dir, "task.json"))
    return grade(task, assembly or os.path.join(task_dir, ASSEMBLY_FILE), **kw)


def _home_checks(ans, lk, checks) -> list[CheckResult]:
    task = ans.task
    mo = task.meta["motion"]
    # the answer is assembled at home: the latch is judged there, as assembled
    fixed, moving = mo["fixed"], mo["moving"]
    moving_pieces = list(ans.pieces[moving])
    sides = MO.Sides(ans.pieces[fixed], moving_pieces, np.zeros(3), np.array([0.0, 0.0, 1.0]),
                     frozenset({(f"fixture {moving}", f"fixture {fixed}")}))
    down = np.asarray(mo["latch"]["pull_direction"], float)
    st = settings(task)
    lim = st["latch_play_max"]
    pull = lambda t: MO.shift(t * down)  # noqa: E731
    out = []
    free = None
    if sides.touching(pull(lim)):
        free = _bisect(sides, pull, lim, n=7)
        out.append(CheckResult("LATCH", True, f"at home, B's side pulled straight down meets the fixed side after "
                               f"{free:.2f} mm (at most {lim:g} mm)", round(free, 3), "mm"))
    else:
        hit = _free(sides, pull, st["loose_shift"], 0.25, start=lim)
        msg = (f"at home, B's side pulled straight down moves {hit:.2f} mm before it meets the fixed side; at most "
               f"{lim:g} mm") if hit is not None else (f"at home, B's side pulled straight down {st['loose_shift']:g} "
                                                      "mm meets nothing: nothing latches it")
        out.append(CheckResult("LATCH", False, msg, hit, "mm"))
    F = float(mo["latch"]["load_N"])
    al = plastic_allowables()
    if free is None:
        out.append(CheckResult("LATCH_STRENGTH", None, "not judged: nothing latches B (see LATCH)", group="sturdy"))
    else:
        out.append(_latch_strength(sides, pull, free, F, al, mo))
    out.append(_pivot_seats(ans, F, al))
    return out


def _latch_strength(sides, pull, free, F, al, mo) -> CheckResult:
    pl = pull(free + CONTACT_DEPTH)
    down = np.asarray(mo["latch"]["pull_direction"], float)
    rows = []
    for mn, ms, fn, fs in sides.pairs(pl):
        if MO.distance(ms, fs) > MO.CONTACT:
            continue
        try:
            pieces = [p for p in (ms & fs).solids() if p.volume > 1e-9]
        except Exception:  # noqa: BLE001
            pieces = []
        for p in pieces:
            proj = 0.0
            for f in p.faces():
                c = float(np.array(tuple(f.normal_at(f.center()))) @ -down)
                if c > 0:
                    proj += float(f.area) * c
            if proj > 1e-6:
                rows.append({"centre": np.array(tuple(p.center())), "area": proj, "moving": (mn, ms), "fixed": (fn, fs)})
    area = sum(r["area"] for r in rows)
    if area <= 0:
        return CheckResult("LATCH_STRENGTH", False, "pulled down, the two sides do not bear on each other over any area "
                           "(they meet only at a point or an edge)", None, group="sturdy")
    u_bear = F / (al["bearing_MPa"] * area)
    worst = (u_bear, f"bearing on {len(rows)} contact patch(es), {area:.1f} mm2 projected")
    for r in rows:
        share = F * r["area"] / area
        ring = 2.0 * math.sqrt(math.pi * r["area"])
        # the pull drags B's plastic down and pushes the fixed side's plastic on: each side's plastic behind it
        for (nm, solid), dvec in ((r["fixed"], down), (r["moving"], -down)):
            t = _behind(solid, r["centre"], dvec)
            u = share / (al["shear_MPa"] * ring * t) if t > 0 else math.inf
            if u > worst[0]:
                worst = (u, f"{nm}: a {r['area']:.1f} mm2 patch punching through {t:.2f} mm of plastic")
    u, what = worst
    msg = (f"{F:g} N ({mo['latch']['load_kg']:g} kg) down at home: utilisation {u:.3f}, worst {what} (allowables "
           f"{al['bearing_MPa']:g} MPa bearing, {al['shear_MPa']:g} MPa shear)")
    return CheckResult("LATCH_STRENGTH", u <= 1.0, msg if u <= 1.0 else "the latch is too weak: " + msg, round(u, 4), "",
                       {"bearing_utilisation": round(u_bear, 4), "allowables": al}, group="sturdy")


def _pivot_seats(ans, F, al) -> CheckResult:
    st = settings(ans.task)
    cache, rows, worst = {}, [], (0.0, "")
    for pv in ans.asm.pivots:
        b1, b2 = ans.asm.body_of(pv.joins[0]), ans.asm.body_of(pv.joins[1])
        if pv.bearing:  # the bearing's outside on its seat, and the pin on the bearing's bore, each over its width
            from ...catalogue import BEARINGS

            dd, DD, BB = BEARINGS[pv.bearing]
            for body, dia, what in ((ans.asm.body_of(pv.seat_on), DD, "seat"), (None, dd, "pin")):
                u = F / (al["bearing_MPa"] * dia * BB)
                rows.append({"pivot": pv.name, "body": body or "pin", "diameter": dia, "length": BB, "utilisation": round(u, 4)})
                if u > worst[0]:
                    worst = (u, f"pivot {pv.name}, bearing {pv.bearing}'s {what}: {dia:g} mm over {BB:g} mm")
            continue
        f1, f2 = _pivot_features(ans, b1, pv, st, cache), _pivot_features(ans, b2, pv, st, cache)
        pins = f1[0] + f2[0]
        if not pins:
            continue
        r_pin = min(p[0] for p in pins)
        for body, (_, bores) in ((b1, f1), (b2, f2)):
            Ln = sum(sp[1] - sp[0] for _, sp, _ in bores)
            if Ln <= 0:
                continue
            u = F / (al["bearing_MPa"] * 2 * r_pin * Ln)
            rows.append({"pivot": pv.name, "body": body, "pin_d": round(2 * r_pin, 3), "bore_length": round(Ln, 3),
                         "utilisation": round(u, 4)})
            if u > worst[0]:
                worst = (u, f"pivot {pv.name}, {body}: {2 * r_pin:.1f} mm pin on {Ln:.1f} mm of bore")
    if not rows:
        return CheckResult("PIVOT_SEATS", None, "not judged: no pin found in a bore (see PIVOTS)", group="sturdy")
    u, what = worst
    msg = f"{F:g} N through each pivot: utilisation {u:.3f}, worst {what} (bearing {al['bearing_MPa']:g} MPa)"
    return CheckResult("PIVOT_SEATS", u <= 1.0, msg if u <= 1.0 else "a pivot's seat is too weak: " + msg, round(u, 4), "",
                       {"pivots": rows}, group="sturdy")
