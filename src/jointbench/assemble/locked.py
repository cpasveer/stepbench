"""Grading an "assembles with a lock" answer (the pipe's level 5): two parts, each
bolted to its fixed part, put in one after the other through the box's opening,
joined to each other not by bolts but by a push-and-turn lock.

The assembly file is level 4's (``parts``: name, file, bolts_to,
insert_direction, path) with, in place of ``joint``, a ``lock``:

    "lock": {"moving": "<part name>", "push": [dx, dy, dz],
             "turn": {"point": [x, y, z], "axis": [ax, ay, az], "degrees": a}}

The part files show both parts in their final, locked place. The moving part
goes in along its way to where it waits (its final place with the turn and then
the push undone); the other part goes in; then the moving part is pushed (along
the turn's axis) and turned; then both are bolted.

  part ... (each)  the part judged as at level 3, against its own fixed part, with
              the box, the other fixed part and the other part, in place, as
              obstacles
  ASSEMBLY_FILE  complete and consistent: the push along the turn's axis
  INSERTION   each part along its way, in order, clear of everything there then
  SEATED / PUSH / TURN / GAP / LOCKED / UNLOCKS / INTERLOCK / PLAY / LOCK_STRENGTH
              level 4d's lock checks (``locks.grade``), the moving part against
              the other part, both fixed parts and the box. The
              moving part slides on its own fixed part as it pushes and turns: that
              pair is left out of the gap and of pulling back (they touch by
              design), and kept for coming loose (it holds the part there).

No stiffness of the two as one body (they are not bonded at a face): the load
through the lock is judged in its plastic (LOCK_STRENGTH). Score: the printed
weight of both parts, as at level 4.
"""
from __future__ import annotations

import json
import os


import build123d as bd
import numpy as np

from ..checks.base import CheckResult, Context
from ..geometry import Placement, read_step
from ..grade import Report, grade as grade_part
from ..moves import motion as MO
from ..task import Task
from . import joint as J
from .grade import _SolidObstacle, _printed
from .path import describe_way, sweep_path
from .sweep import collisions

ASSEMBLY_FILE = "assembly.json"
AXIS_TOL_DEG = 1.0  # the push must lie along the turn's axis within this


class Lock:
    def __init__(self, moving, push, point, axis, degrees):
        self.moving, self.push, self.point, self.axis, self.degrees = moving, push, point, axis, degrees

    def turned(self) -> Placement:
        return MO.about(self.point, self.axis, self.degrees)

    def back(self) -> Placement:
        """Final place -> where the moving part waits: the turn undone, then the push."""
        return MO.about(self.point, self.axis, -self.degrees).then(MO.shift(-self.push))


class Assembly:
    def __init__(self, parts, lock):
        self.parts, self.lock = parts, lock


def load(path: str, fixed_parts) -> Assembly:
    if not os.path.exists(path):
        raise J.AssemblyError(f"no assembly file {os.path.basename(path)}")
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except Exception as e:  # noqa: BLE001
        raise J.AssemblyError(f"the assembly file is not valid JSON: {e}") from None
    ps = d.get("parts")
    if not isinstance(ps, list) or len(ps) != 2:
        raise J.AssemblyError("the assembly file must list exactly two parts")
    parts = []
    for i, p in enumerate(ps):
        for key in ("name", "file", "bolts_to"):
            if key not in p:
                raise J.AssemblyError(f"part {i + 1} has no {key!r}")
        if "insert_direction" not in p and not p.get("path"):
            raise J.AssemblyError(f"part {i + 1} has neither an 'insert_direction' nor a 'path' (a way in)")
        if p["bolts_to"] not in fixed_parts:
            raise J.AssemblyError(f"part {p['name']} bolts to {p['bolts_to']!r}, which is not a fixed part")
        parts.append(J.PartEntry(str(p["name"]), str(p["file"]), p["bolts_to"],
                                 J._vec(p["insert_direction"], f"{p['name']}'s insert_direction")
                                 if "insert_direction" in p else None,
                                 J._path(p.get("path") or [], p["name"])))
    if parts[0].name == parts[1].name or parts[0].bolts_to == parts[1].bolts_to:
        raise J.AssemblyError("two differently named parts, each bolting to a different fixed part")
    lk = d.get("lock")
    if not isinstance(lk, dict) or "turn" not in lk or "push" not in lk or "moving" not in lk:
        raise J.AssemblyError("the assembly file has no complete 'lock' (moving, push, turn)")
    if lk["moving"] not in (parts[0].name, parts[1].name):
        raise J.AssemblyError("the lock's moving part must be one of the two parts")
    try:
        push = np.array([float(x) for x in lk["push"]])
        tu = lk["turn"]
        point = np.array([float(x) for x in tu["point"]])
        axis = J._vec(tu["axis"], "the turn's axis")
        deg = float(tu["degrees"])
    except J.AssemblyError:
        raise
    except Exception:  # noqa: BLE001
        raise J.AssemblyError("the lock's push and turn must be numbers: push [dx, dy, dz], turn point, axis, "
                              "degrees") from None
    n = float(np.linalg.norm(push))
    if n < 1e-6 or abs(deg) < 1e-6:
        raise J.AssemblyError("the lock needs a push and a turn")
    if abs(float(push @ axis)) / n < np.cos(np.radians(AXIS_TOL_DEG)):
        raise J.AssemblyError("the lock's push must run along the turn's axis")
    return Assembly(parts, Lock(lk["moving"], push, point, axis, deg))


def subtask(task: Task, asm: Assembly, k: int, there: list) -> Task:
    """Part k against its own fixed part; the box, the other fixed part and the other part (``there``:
    where it is while part k is bolted) as obstacles."""
    me = asm.parts[k]
    obstacles = list(task.obstacles_at(0)) + [_SolidObstacle(f"{n} (fixed part)", s)
                                              for n, s in task.fixed_solids(0).items() if n != me.bolts_to]
    obstacles.append(_SolidObstacle(asm.parts[1 - k].name, there[1 - k]))
    tol = {k_: v for k_, v in task.tolerances.items() if k_ not in ("locks", "joint")}
    tol["required_checks"] = [c for c in tol.get("required_checks", []) if c != "STIFFNESS"]
    t = Task(f"{task.id}/{me.name}", 3, task.seed, {me.bolts_to: task.parts[me.bolts_to]},
             [{me.bolts_to: task.poses[0][me.bolts_to]}], tol, {}, [obstacles], task.generator_version)
    t.root = task.root
    return t


def grade(task: Task, assembly_path: str, *, load_kg: float | None = None, manufacturing: bool = True) -> Report:
    """``manufacturing=False`` leaves out each part's printing check (minutes; for a broken copy whose
    fault lies elsewhere)."""
    from ..locks import grade as LG

    checks: list[CheckResult] = []
    folder = os.path.dirname(os.path.abspath(assembly_path))
    try:
        asm = load(assembly_path, list(task.parts))
    except J.AssemblyError as e:
        return Report(task.id, False, [CheckResult("ASSEMBLY_FILE", False, str(e))])
    lk = asm.lock
    km = 0 if asm.parts[0].name == lk.moving else 1
    checks.append(CheckResult("ASSEMBLY_FILE", True, f"{asm.parts[0].name} bolts to {asm.parts[0].bolts_to}, "
                              f"{asm.parts[1].name} to {asm.parts[1].bolts_to}; {lk.moving} is pushed "
                              f"{np.linalg.norm(lk.push):.1f} mm and turned {lk.degrees:g} deg into the lock"))
    paths = [os.path.join(folder, p.file) for p in asm.parts]
    solids = []
    for p, path in zip(asm.parts, paths):
        try:
            s = read_step(path).solids() if os.path.exists(path) else []
        except Exception:  # noqa: BLE001
            s = []
        solids.append(s[0] if len(s) == 1 else None)
    if any(s is None for s in solids):
        from ..checks import file_valid

        for p, path, s in zip(asm.parts, paths, solids):
            if s is None:
                r = file_valid.run(Context(task, path)) if os.path.exists(path) else \
                    CheckResult("FILE_VALID", False, f"no file {p.file}")
                if r.ok:
                    r = CheckResult("FILE_VALID", False, "not exactly one solid")
                r.name = f"{p.name} FILE_VALID"
                checks.append(r)
        return Report(task.id, False, checks)
    waiting = solids[km].moved(lk.back().location())
    started = solids[km].moved(MO.about(lk.point, lk.axis, -lk.degrees).location())  # pushed, not yet turned

    # ---- each part
    reps = []
    for k, (p, path) in enumerate(zip(asm.parts, paths)):  # both bolted once locked: the other in place
        sub = subtask(task, asm, k, solids)
        if not manufacturing:
            sub.tolerances["required_checks"] = [c for c in sub.tolerances["required_checks"] if c != "MANUFACTURING"]
        rep = grade_part(sub, path, optional=False, score=False)
        reps.append(rep)
        for c in rep.checks:
            c.name = f"{p.name} {c.name}"
            checks.append(c)


    # ---- the ways in, in order: the moving part to where it waits
    lim = task.tolerances["overlap_volume_max"]
    fixed = task.fixed_solids(0)
    placed, problems, rows = {}, [], []
    for k, p in enumerate(asm.parts):
        body = waiting if k == km else solids[k]
        bodies = {f"fixed part {n}": s for n, s in fixed.items()}
        bodies.update({f"obstacle {n}": s for n, s in task.obstacle_solids(0).items()})
        bodies.update(placed)
        hits, stretches, why = sweep_path(body, p.path or [], bodies, direction=p.direction,
                                          box=list(task.obstacle_solids(0).values()))
        if why:
            problems.append(f"{p.name}: {why}")
        rows += [{"part": p.name, "against": w, "volume": v} for w, v in hits.items()]
        bad = sorted(((v, w) for w, v in hits.items() if v > lim), reverse=True)
        if bad:
            where = next((st["stretch"] for st in stretches if st["hits"].get(bad[0][1], 0) > lim), "")
            problems.append(f"{p.name} runs into {bad[0][1]} on its way in ({where}; {bad[0][0]:.3f} mm3)")
        placed[p.name] = body  # (no bolt heads yet: both are bolted once locked)
    checks.append(CheckResult("INSERTION", not problems, "; ".join(problems) if problems else
                              "each part goes in clear of everything: " + ", then ".join(
                                  f"{p.name} {describe_way(p.direction, p.path)}" for p in asm.parts)
                              + f"; {lk.moving} waits for the lock",
                              max((r["volume"] for r in rows), default=0.0), "mm3", {"sweeps": rows}))

    # ---- the lock: level 4d's checks, the moving part against everything else
    ko = 1 - km
    other = asm.parts[ko]
    mine = asm.parts[km].bolts_to
    fixed_side = [(f"fixture {n}", s) for n, s in fixed.items()]
    fixed_side.append((other.name, solids[ko]))
    fixed_side += [(f"obstacle {n}", s) for n, s in task.obstacle_solids(0).items()]
    moving_side = [(lk.moving, started)]
    skip = frozenset({(lk.moving, f"fixture {mine}")})  # it slides on its own fixed part by design
    sliding = MO.Sides(fixed_side, moving_side, lk.point, lk.axis, skip)
    holding = MO.Sides(fixed_side, moving_side, lk.point, lk.axis)
    kg = load_kg if load_kg is not None else float((task.meta.get("lock") or {}).get("pull_load_kg", 1.0))
    lkd = {"axis_point": lk.point, "axis_direction": lk.axis, "turn_deg": lk.degrees,
           "insert_direction": lk.push / np.linalg.norm(lk.push), "pull_load_kg": kg, "pull_load_N": round(kg * 9.81, 3)}
    st = task.tolerances["locks"]
    checks.append(LG._seated(task, sliding))
    hits = collisions(started, lk.push, {n: s for n, s in fixed_side if n != f"fixture {mine}"},
                      float(np.linalg.norm(lk.push)))
    bad = sorted(((v, n) for n, v in hits.items() if v > lim), reverse=True)
    checks.append(CheckResult("PUSH", not bad, f"pushed {np.linalg.norm(lk.push):.1f} mm along the axis, "
                              + (f"{lk.moving} runs into {bad[0][1]} ({bad[0][0]:.3f} mm3)" if bad else
                                 f"{lk.moving} meets nothing"), max(hits.values(), default=0.0), "mm3"))
    checks += [_say(c, lk.moving, other.name) for c in LG._turn(task, sliding, lkd, st)]
    pulled = LG._locked(task, sliding, lkd, st)
    held = LG._locked(task, holding, lkd, st)
    by = {c.name: c for c in pulled}
    by.update({c.name: c for c in held if c.name in ("INTERLOCK", "PLAY")})
    for n in ("LOCKED", "UNLOCKS", "INTERLOCK", "PLAY"):
        checks.append(_say(by[n], lk.moving, other.name))
    s = LG._strength(task, {"A": solids[ko], "B": started}, lkd, by["LOCKED"])
    checks.append(_say(s, lk.moving, other.name))

    passed = all(c.ok for c in checks if c.required)
    info = {"volume": round(sum(float(x.volume) for x in solids), 3),
            "parts": {p.name: round(float(x.volume), 3) for p, x in zip(asm.parts, solids)}}
    bb = bd.Compound(solids).bounding_box()  # (as level 4's grader: the report's text prints it)
    info.update(bbox_min=[round(v, 3) for v in bb.min], bbox_max=[round(v, 3) for v in bb.max],
                bbox_size=[round(v, 3) for v in bb.size])
    from .. import material_score

    if passed and material_score.asked(task):
        info["material_used"] = _printed(task, asm, paths, reps)
    return Report(task.id, passed, checks, info)


def _say(c: CheckResult, moving: str, other: str) -> CheckResult:
    """level 4d's wording (B moves, A stays) in this answer's names."""
    c.reason = (c.reason.replace("B's side", f"{moving}").replace("A's side", other)
                .replace("B and its part", moving).replace("B pulled", f"{moving} pulled")
                .replace("B pulls", f"{moving} pulls").replace("B comes", f"{moving} comes")
                .replace("B turns", f"{moving} turns").replace("B shifts", f"{moving} shifts")
                .replace("B moves", f"{moving} moves").replace("B can", f"{moving} can")
                .replace("B cannot", f"{moving} cannot").replace("turned, B ", f"turned, {moving} "))
    return c


def grade_folder(task_dir: str, assembly: str | None = None, **kw) -> Report:
    task = Task.load(os.path.join(task_dir, "task.json"))
    return grade(task, assembly or os.path.join(task_dir, ASSEMBLY_FILE), **kw)
