"""Grading one part in several arrangements.

The answer is ``output.step`` plus ``placement.json`` beside it: per
arrangement, a rotation and a translation that carry the part from its own
coordinates into that arrangement's coordinates (a point p lands at R p + t).
Only rigid moves count: R must be a rotation -- columns of unit length and
square to each other, within 1e-4 -- and not a mirror image.

The grader places the solid into each arrangement as declared and grades it
there with the core grader, exactly as a one-arrangement task: the
arrangement's own fixed parts, bolts, stated load and requirements. The part
passes only if it passes in every arrangement.

What belongs to the part itself is judged once, on ``output.step`` as handed
in: clean modelling, and printing -- the same part is printed once, in its
own print direction, whatever arrangement it later goes into (the printing
checker tries the part's own axis directions, so placing it first would
change its verdict). The material score (printed weight) is measured the
same way.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass, field

from ...checks.base import CheckResult
from ...geometry import Placement, read_step, write_step
from ...grade import Report
from ...task import Task

PLACEMENT_FILE = "placement.json"
ORTHO_MAX = 1e-4          # largest error in R^T R = I accepted (typed-in numbers)
SOLID_ONLY = {"CLEAN", "MANUFACTURING"}  # judged once, on the part as handed in


def arrangements(task: Task) -> list[dict]:
    """[{"name", "pose", "parts"}], from task.json's meta or, failing that,
    one arrangement per stored pose."""
    arr = (task.meta or {}).get("arrangements")
    if arr:
        return arr
    return [{"name": str(k + 1), "pose": k, "parts": list(p)} for k, p in enumerate(task.poses)]


def parse_placement(data, names: list[str]) -> tuple[dict | None, CheckResult]:
    """Read the placement file's content. Returns ({name: Placement}, result)."""
    if not isinstance(data, dict):
        return None, CheckResult("PLACEMENT", False, f"{PLACEMENT_FILE} must hold one JSON object, keyed by "
                                 f"arrangement ({', '.join(repr(n) for n in names)})")
    missing = [n for n in names if n not in data]
    if missing:
        return None, CheckResult("PLACEMENT", False, f"{PLACEMENT_FILE} gives no placement for arrangement "
                                 + ", ".join(missing))
    out, notes = {}, []
    for n in names:
        e = data[n]
        try:
            R = [[float(v) for v in row] for row in e["rotation"]]
            t = [float(v) for v in e["translation"]]
        except (KeyError, TypeError, ValueError):
            return None, CheckResult("PLACEMENT", False, f"arrangement {n}: give \"rotation\" (3 rows of 3 numbers) "
                                     "and \"translation\" (3 numbers)")
        if len(R) != 3 or any(len(r) != 3 for r in R) or len(t) != 3 \
                or not all(math.isfinite(v) for v in t + [x for r in R for x in r]):
            return None, CheckResult("PLACEMENT", False, f"arrangement {n}: the rotation must be 3 rows of 3 numbers "
                                     "and the translation 3 numbers")
        err = max(abs(sum(R[k][i] * R[k][j] for k in range(3)) - (1.0 if i == j else 0.0))
                  for i in range(3) for j in range(3))
        det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
               + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
        if err > ORTHO_MAX:
            return None, CheckResult("PLACEMENT", False, f"arrangement {n}: the rotation is not a rotation: its "
                                     f"columns are not unit length and square to each other (off by {err:.2g}, "
                                     f"limit {ORTHO_MAX:g})", err)
        if det < 0:
            return None, CheckResult("PLACEMENT", False, f"arrangement {n}: the rotation is a mirror image "
                                     "(determinant -1); only rigid moves are allowed, no mirroring")
        out[n] = Placement(tuple(t), _nearest_rotation(R))
        if err > 1e-9:
            notes.append(f"{n} squared up from {err:.1g}")
    why = "; ".join(notes)
    return out, CheckResult("PLACEMENT", True, f"{PLACEMENT_FILE}: a rigid move for arrangement "
                            + " and ".join(names) + (f" ({why})" if why else ""))


def _nearest_rotation(R):
    """The rotation nearest to an almost-rotation (polar decomposition)."""
    import numpy as np

    u, _, vt = np.linalg.svd(np.array(R, dtype=float))
    q = u @ vt
    return tuple(tuple(float(v) for v in row) for row in q)


def read_placement(step_path: str, task: Task):
    """The placement file next to ``step_path`` (else in the task folder)."""
    names = [a["name"] for a in arrangements(task)]
    cands = [os.path.join(os.path.dirname(os.path.abspath(step_path)), PLACEMENT_FILE)]
    if task.root:
        cands.append(os.path.join(task.root, PLACEMENT_FILE))
    path = next((p for p in cands if os.path.exists(p)), None)
    if path is None:
        return None, CheckResult("PLACEMENT", False, f"no {PLACEMENT_FILE} next to the part: it must say, per "
                                 "arrangement, how the part is placed")
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        return None, CheckResult("PLACEMENT", False, f"{PLACEMENT_FILE} could not be read as JSON: {e}")
    return parse_placement(data, names)


def sub_task(task: Task, arr: dict) -> Task:
    """One arrangement as a plain one-pose task of the connecting-part family."""
    parts = {n: task.parts[n] for n in arr["parts"]}
    pose = {n: task.poses[arr["pose"]][n] for n in arr["parts"]}
    tol = dict(task.tolerances)
    tol.pop("score", None)  # the score is the part's, measured once
    st = arr.get("stiffness")
    tol["required_checks"] = [c for c in tol.get("required_checks", []) if c != "STIFFNESS" or st]
    t = Task(f"{task.id} arrangement {arr['name']}", 1, task.seed, parts, [pose], tol,
             {}, generator_version=task.generator_version, stiffness=st)
    t.root = task.root
    return t


@dataclass
class AdaptReport(Report):
    """A report over several arrangements: the placement check, then every
    arrangement's own checks, each reason saying which arrangement it is."""

    arrangements: list = field(default_factory=list)  # [{"name", "parts", "outcome", "failed"}]

    def text(self) -> str:
        req = [c for c in self.checks if c.required]
        n_ok = sum(1 for c in req if c.ok)
        head = {"pass": "PASS", "fail": "FAIL", "error": "ERROR"}[self.outcome()]
        over = f" over {len(self.arrangements)} arrangements" if self.arrangements else ""
        lines = [f"{head}  ({n_ok}/{len(req)} required checks{over})  {self.task_id}"]
        if head == "ERROR":
            lines.append("  The grader could not measure this part (a geometry-kernel failure, not a fault"
                         " of the part). This is no verdict and does not count as a try.")

        def row(c, indent="  "):
            mark = "err " if c.error else ("ok  " if c.ok else ("--  " if c.ok is None else ("FAIL" if c.required else "warn")))
            reason = c.reason
            a = c.details.get("arrangement")
            if a is not None:  # under its arrangement's heading the prefix says nothing new
                reason = reason.removeprefix(f"in arrangement {a}: ")
            return f"{indent}{mark} {c.name:<16} {reason}"

        for c in self.checks:
            if "arrangement" not in c.details and c.details.get("scope") != "part":
                lines.append(row(c))
        part = [c for c in self.checks if c.details.get("scope") == "part"]
        if part:
            bad = sorted({c.name for c in part if c.ok is False and c.required})
            lines.append("  the part itself, as handed in (printed once, in its own print direction): "
                         + (f"fails: {', '.join(bad)}" if bad else "passes"))
            lines += [row(c, "    ") for c in part]
        for a in self.arrangements:
            res = {"pass": "passes", "fail": "fails", "error": "could not be measured"}[a["outcome"]]
            tail = f": {', '.join(a['failed'])}" if a["failed"] else ""
            lines.append(f"  arrangement {a['name']} (fixed parts {', '.join(a['parts'])}): the part {res}{tail}")
            lines += [row(c, "    ") for c in self.checks if c.details.get("arrangement") == a["name"]]
        if any(not c.required for c in self.checks):
            lines.append("  (warn: reported for information, not required for a pass in this task)")
        mu = self.info.get("material_used")
        if mu and "used_cm3" in mu:
            lines.append(f"  score {mu['grams']:.0f} g printed ({mu['used_cm3']:.1f} cm3): {mu['measure']}; "
                         "lower is better")
        if self.info.get("volume") is not None:
            bb = self.info.get("bbox_size")
            lines.append(f"  info volume {self.info['volume']:.1f} mm3, "
                         f"bounding box {bb[0]:.1f} x {bb[1]:.1f} x {bb[2]:.1f} mm (in the part's own coordinates)")
        return "\n".join(lines)

    def to_json(self) -> dict:
        return dict(super().to_json(), arrangements=self.arrangements)


def _prefixed(c: CheckResult, name: str) -> CheckResult:
    reason = c.reason
    return CheckResult(c.name, c.ok, f"in arrangement {name}: {reason}", c.value, c.unit,
                       dict(c.details, arrangement=name), c.required, c.group, c.error)


def grade_with_placement(task: Task, step_path: str, placement, *, manufacturing: bool = True, optional=True,
                         score: bool = True, skip: frozenset = frozenset()) -> AdaptReport:
    """Grade ``step_path`` with ``placement``: a {name: Placement} (already
    checked) or the placement file's content (a dict, checked here)."""
    from ...grade import grade

    arrs = arrangements(task)
    names = [a["name"] for a in arrs]
    if placement and not all(isinstance(v, Placement) for v in placement.values()):
        placement, pres = parse_placement(placement, names)
    else:
        pres = CheckResult("PLACEMENT", True, "a rigid move for arrangement " + " and ".join(names))
    pres.group = "fits"
    if placement is None:
        return AdaptReport(task.id, False, [pres], {}, arrangements=[])
    try:
        shape = read_step(step_path)
    except Exception:  # noqa: BLE001 - FILE_VALID on the file itself says what is wrong
        shape = None
    checks, subs, info = [pres], [], {}
    own, ctx = _as_handed_in(task, step_path, optional, skip)
    checks += own
    if own and own[0].ok is False:  # no valid solid: nothing to place
        return AdaptReport(task.id, False, checks, info, arrangements=[])
    with tempfile.TemporaryDirectory() as tmp:
        for i, a in enumerate(arrs):
            sub = sub_task(task, a)
            path = step_path
            if shape is not None:
                path = os.path.join(tmp, f"placed-{i}.step")
                moved = shape.moved(placement[a["name"]].location())
                write_step(moved, path)
            rep = grade(sub, path, manufacturing=manufacturing, optional=optional, score=False,
                        skip=frozenset(skip) | SOLID_ONLY)
            checks += [_prefixed(c, a["name"]) for c in rep.checks]
            subs.append({"name": a["name"], "parts": list(a["parts"]), "outcome": rep.outcome(),
                         "failed": rep.failed()})
            if i == 0 and shape is not None:
                info = _own_frame_info(shape)
    passed = all(c.ok for c in checks if c.required)
    info["placement"] = {n: p.to_json() for n, p in placement.items()}
    from ... import material_score

    if passed and score and material_score.asked(task):
        dfm = getattr(ctx, "dfm", None)
        try:
            info["material_used"] = material_score.material_used(task, step_path, getattr(dfm, "direction", None) or None)
        except Exception as e:  # noqa: BLE001 - no figure rather than a wrong one
            info["material_used"] = {"error": f"{type(e).__name__}: {e}"}
    return AdaptReport(task.id, passed, checks, info, arrangements=subs)


def _as_handed_in(task: Task, step_path: str, optional, skip) -> tuple[list, object]:
    """The checks of the part itself, on output.step as handed in: a valid
    solid, clean modelling, printing (where the task requires or asks for them)."""
    from ...checks import GROUP_OF, clean, file_valid, manufacturing
    from ...checks.base import Context
    from ...dfm import MeasurementError

    ctx = Context(sub_task(task, arrangements(task)[0]), step_path)
    req = set(task.tolerances.get("required_checks", []))
    out = [file_valid.run(ctx)]
    if out[0].ok:
        for mod in (clean, manufacturing):
            name = mod.NAME
            wanted = name in req or optional == "all" or (isinstance(optional, (set, frozenset)) and name in optional)
            if name in skip or not wanted:
                continue
            try:
                res = mod.run(ctx)
            except MeasurementError as e:
                res = CheckResult(name, None, f"could not be measured (grader error, not a verdict): {e}", error=True)
            except Exception as e:  # noqa: BLE001 - a crash in a check is a fail, with the reason
                res = CheckResult(name, False, f"could not be measured: {type(e).__name__}: {e}")
            for r in (res if isinstance(res, list) else [res]):
                r.required, r.group = name in req, GROUP_OF[name]
                out.append(r)
    for r in out:
        r.details = dict(r.details, scope="part")
    return out, ctx


def _own_frame_info(shape) -> dict:
    solids = shape.solids()
    s = solids[0] if len(solids) == 1 else shape
    try:
        bb = s.bounding_box()
        return {"volume": round(float(s.volume), 3), "bbox_size": [round(v, 3) for v in bb.size]}
    except Exception:  # noqa: BLE001 - information only
        return {}


def grade_adapt(task: Task, step_path: str, *, manufacturing: bool = True, optional=True, score: bool = True,
                skip: frozenset = frozenset()) -> AdaptReport:
    """The family's grader: reads the placement file next to the part."""
    placement, pres = read_placement(step_path, task)
    if placement is None:
        pres.group = "fits"
        return AdaptReport(task.id, False, [pres], {}, arrangements=[])
    rep = grade_with_placement(task, step_path, placement, manufacturing=manufacturing, optional=optional,
                               score=score, skip=skip)
    rep.checks[0] = pres
    return rep
