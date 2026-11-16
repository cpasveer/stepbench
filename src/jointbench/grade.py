"""Grading one STEP file against one task.

The grader sees only the file. It does not know or care which program made
it. Same file in, same verdict and numbers out: every number is rounded
before it is reported, and nothing random or time-dependent enters.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import os
from dataclasses import dataclass, field

from .checks import CHECKS, GROUP_OF, GROUP_SWITCH, GROUPS, OPTIONAL, CheckResult, Context
from .task import Task


@dataclass
class Report:
    task_id: str
    passed: bool
    checks: list[CheckResult]
    info: dict = field(default_factory=dict)

    def failed(self) -> list[str]:
        """Checks that failed and decide the verdict."""
        return [c.name for c in self.checks if c.ok is False and c.required]

    def warnings(self) -> list[str]:
        """Checks that failed but are only reported for this task."""
        return [c.name for c in self.checks if c.ok is False and not c.required]

    def errors(self) -> list[str]:
        """Checks the grader could not measure (kernel failures, not verdicts)."""
        out = [c.name for c in self.checks if c.error]
        wall = ((self.info.get("material") or {}).get("wall") or {})
        if wall.get("error"):
            out.append("material score: minimum wall")
        return out

    def outcome(self) -> str:
        """pass, fail or error. Error only when a required check could not be
        measured and nothing required failed on its own: the verdict then
        rests on a measurement the grader could not make, so it is no
        verdict, and the attempt does not count against the agent."""
        if self.passed:
            return "pass"
        if self.failed():
            return "fail"
        if any(c.error and c.required for c in self.checks):
            return "error"
        return "fail"

    def counts_as_attempt(self) -> bool:
        return self.outcome() != "error"

    def text(self) -> str:
        req = [c for c in self.checks if c.required]
        n_ok = sum(1 for c in req if c.ok)
        head = {"pass": "PASS", "fail": "FAIL", "error": "ERROR"}[self.outcome()]
        lines = [f"{head}  ({n_ok}/{len(req)} required checks)  {self.task_id}"]
        if head == "ERROR":
            lines.append("  The grader could not measure this part (a geometry-kernel failure, not a fault"
                         " of the part). This is no verdict and does not count as a try.")
        for c in self.checks:
            mark = "err " if c.error else ("ok  " if c.ok else ("--  " if c.ok is None else ("FAIL" if c.required else "warn")))
            lines.append(f"  {mark} {c.name:<16} {c.reason}")
        if any(not c.required for c in self.checks):
            lines.append("  (warn: reported for information, not required for a pass in this task)")
        v = self.verdict()["requirements"]
        lines.append("  requirements: " + ", ".join(
            f"{g} {r['result']}{'' if r['required'] else ' (not required)'}" for g, r in v.items()))
        if self.info:
            bb = self.info.get("bbox_size")
            lines.append(f"  info volume {self.info['volume']:.1f} mm3, "
                         f"bounding box {bb[0]:.1f} x {bb[1]:.1f} x {bb[2]:.1f} mm")
            mu = self.info.get("material_used")
            if mu and "used_cm3" in mu:
                lines.append(f"  score {mu['used_cm3']:.1f} cm3 ({mu['grams']:.0f} g): {mu['measure']}; "
                             "lower is better")
            m = self.info.get("material")
            if m:
                tail = "" if m["counts"] else f" -- does not count: {m['why_not']}"
                lines.append(f"  material {m['ratio']:.2f} x par (par {m['par']:.0f} mm3 / volume "
                             f"{m['volume']:.0f} mm3; higher is leaner){tail}")
        return "\n".join(lines)

    def verdict(self) -> dict:
        """The judgement in the maintainer's shape: every requirement switched on
        for the task is pass or fail; material is the one score."""
        req = {}
        for g in GROUPS:
            cs = [c for c in self.checks if c.group == g]
            if not cs:
                continue
            on = any(c.required for c in cs)
            if any(c.ok is None for c in cs) and not any(c.ok is False for c in cs):  # skipped or error
                state = "not measured"
            else:
                state = "pass" if all(c.ok for c in cs if c.ok is not None) else "fail"
            req[g] = {"required": on, "result": state,
                      "failed": [c.name for c in cs if c.ok is False]}
        m = self.info.get("material")
        score = None
        if m:
            score = {"material": m["ratio"] if m["counts"] else None, "ratio_uncounted": m["ratio"],
                     "counts": m["counts"], "why_not": m.get("why_not")}
        mu = self.info.get("material_used")
        if mu and "used_cm3" in mu:
            score = dict(score or {}, material_used_cm3=mu["used_cm3"], material_measure=mu["measure"])
        return {"passed": self.passed, "outcome": self.outcome(), "counts_as_attempt": self.counts_as_attempt(),
                "errors": self.errors(), "requirements": req, "score": score}

    def to_json(self) -> dict:
        return {"task": self.task_id, "passed": self.passed, "outcome": self.outcome(),
                "counts_as_attempt": self.counts_as_attempt(), "verdict": self.verdict(),
                "checks": [c.to_json() for c in self.checks], "info": self.info}


def grade(task: Task, step_path: str, *, manufacturing: bool = True, optional=True, score: bool = True,
          skip: frozenset = frozenset()) -> Report:
    """Grade one file. ``manufacturing=False`` skips the manufacturing rules
    when they are only reported (the self-test uses it for speed);
    Which reported-only checks run (required ones always do):
      optional=True   (default) only what the material score needs:
                      SECTION_MIN, and the wall rule inside the score
      optional="all"  every reported group (clean, makeable, sturdy)
      optional={...}  these check names
      optional=False  none; once a required check fails, slow checks stop
    A group that is neither required nor asked for is not computed.
    ``skip`` leaves out these checks entirely (generator use only: the
    alternative body is graded for fit, not makeability).
    ``score=False`` leaves out the material score (its wall check runs the
    manufacturing checker and takes seconds); the verdict is unaffected."""
    from . import families
    from .dfm import MeasurementError

    fam = families.of(task)
    if fam.grade is not None:
        return fam.grade(task, step_path, manufacturing=manufacturing, optional=optional, score=score, skip=skip)
    ctx = Context(task, step_path)
    ctx.verdict_only = optional is False
    results: list[CheckResult] = []
    required_extra = set(task.tolerances.get("required_checks", []))
    if fam.check_poses is not None:
        ctx.n_check_poses = int(fam.check_poses(task))
    ctx.scene = fam.pose_scene
    opt_names = set(OPTIONAL) | {m.NAME for m, _ in fam.checks if getattr(m, "OPTIONAL", False)}
    group_of = dict(GROUP_OF, **{m.NAME: getattr(m, "GROUP", "fits") for m, _ in fam.checks})
    core = [(m, lv) for m, lv in CHECKS[1:] if fam.core_checks is None or m.NAME in fam.core_checks]
    for mod, first_level in list(CHECKS[:1]) + _POSES + core + list(fam.checks):
        if mod is _PoseGate:
            if fam.pose_answer is None or results[0].ok is False:
                continue
            r = _PoseGate.run(ctx, fam)
            results.append(r)
            if r.ok is False:
                break
            continue
        if task.level < first_level:
            continue
        if mod.NAME in skip:
            continue
        if mod.NAME in ("SEAT_INSIDE", "BEARING_SEAT", "COMPLIANCE", "KEY_TURN") and mod.NAME not in required_extra:
            continue  # judged only where a task asks for it: older tasks read exactly as before
        if results and results[0].ok is False:
            results.append(CheckResult(mod.NAME, None, "skipped: file not valid"))
            continue
        is_opt = mod.NAME in opt_names and mod.NAME not in required_extra
        if optional is True:
            wanted = mod.NAME == "SECTION_MIN"
        elif optional == "all":
            wanted = True
        elif optional is False:
            wanted = False
        else:
            wanted = mod.NAME in optional
        if is_opt and (not wanted or (mod.NAME == "MANUFACTURING" and not manufacturing and optional != "all")):
            continue
        slow = mod.NAME in ("SECTION_MIN", "STIFFNESS", "MANUFACTURING", "CLEAN", "COMPLIANCE")
        # a check the grader could not measure decides nothing yet: the later
        # checks still run, and a real failure among them decides the verdict
        fit_so_far = all(c.ok for c in results if c.required and not c.error)
        if optional is False and (mod.NAME in opt_names or slow) and not fit_so_far:
            # verdict-only grading: the verdict is already fail, so the slow
            # measurements cannot change it
            results.append(CheckResult(mod.NAME, None, "skipped: verdict already decided",
                                       group=group_of.get(mod.NAME, "fits")))
            continue
        if is_opt and not fit_so_far:
            # reported-only checks measure parts that already fit; this also
            # keeps the slower measurements off failing attempts
            results.append(CheckResult(mod.NAME, None, "skipped: the part does not fit yet", required=False,
                                       group=group_of.get(mod.NAME, "fits")))
            continue
        try:
            res = mod.run(ctx)
        except MeasurementError as e:  # the kernel could not measure: no verdict either way
            res = CheckResult(mod.NAME, None, f"could not be measured (grader error, not a verdict): {e}",
                              error=True)
        except Exception as e:  # noqa: BLE001 - a crash in a check is a fail, with the reason
            res = CheckResult(mod.NAME, False, f"could not be measured: {type(e).__name__}: {e}")
        for r in (res if isinstance(res, list) else [res]):
            r.required = mod.NAME not in opt_names or mod.NAME in required_extra
            r.group = group_of.get(mod.NAME, "fits")
            results.append(r)
    info = {}
    if ctx.solid is not None:
        bb = ctx.solid.bounding_box()
        info = {
            "volume": round(float(ctx.solid.volume), 3),
            "bbox_min": [round(v, 3) for v in bb.min],
            "bbox_max": [round(v, 3) for v in bb.max],
            "bbox_size": [round(v, 3) for v in bb.size],
        }
        from .geometry import BUILD_VOLUME, build_box

        try:
            box = build_box(ctx.solid)
            info["build_box"] = box  # sides of a tight box in the part's best orientation, smallest first
            info["fits_build_volume"] = all(s <= v + 1e-6 for s, v in zip(box, sorted(BUILD_VOLUME)))
        except Exception:  # noqa: BLE001 - information only: never fails a grade
            pass
    passed = all(c.ok for c in results if c.required)
    if ctx.solid is not None and task.par and score:
        info["material"] = material(task, ctx, results, passed)
    from . import material_score

    if ctx.solid is not None and score and passed and material_score.asked(task):
        # level 3: the material the stated process uses (lower is better), in the
        # build direction the manufacturing checker chose
        dfm = getattr(ctx, "dfm", None)
        try:
            info["material_used"] = material_score.material_used(
                task, step_path, getattr(dfm, "direction", None) or None)
        except Exception as e:  # noqa: BLE001 - no figure rather than a wrong one
            info["material_used"] = {"error": f"{type(e).__name__}: {e}"}
    return Report(task.id, passed, results, info)


class _PoseGate:
    """POSES: for a moving family, the answer must reach every pose. The
    family's ``pose_answer`` places it; the still checks then run on the
    answer as placed at each pose."""

    NAME = "POSES"

    @staticmethod
    def run(ctx, fam) -> CheckResult:
        places = []
        for k in ctx.poses():
            try:
                places.append(fam.pose_answer(ctx.task, ctx, k))
            except Exception as e:  # noqa: BLE001
                return CheckResult("POSES", False, f"pose {k}: could not place the answer: {e}")
            if places[-1] is None:
                return CheckResult("POSES", False, f"the answer cannot reach pose {k}", k, "pose")
        ctx.answer_poses = places
        return CheckResult("POSES", True, f"the answer reaches all {len(places)} poses", len(places), "poses")


_POSES = [(_PoseGate, 1)]


def material(task: Task, ctx, results, passed: bool) -> dict:
    """The material figure: par volume / part volume (1.0 = par, higher =
    leaner). Pass or fail is not touched. A part counts for the score only
    if it passes, keeps the minimum section (SECTION_MIN) and the minimum
    wall of the stated process (the manufacturing checker's wall rule)."""
    vol = float(ctx.solid.volume)
    par = float(task.par["volume"])
    out = {"volume": round(vol, 3), "par": round(par, 3), "ratio": round(par / vol, 4), "counts": False}
    sec = next((c for c in results if c.name == "SECTION_MIN"), None)
    wall = next((c for c in results if c.name.endswith("_WALL_MIN")), None)
    wall_error = None
    if wall is None and passed:
        from .dfm import MeasurementError

        try:
            wall = _wall_only(task, ctx)
        except MeasurementError as e:
            wall_error = str(e)
        except Exception as e:  # noqa: BLE001 -- the direct rule call is not wrapped by the checker
            wall_error = f"{type(e).__name__}: {e}"
    why = []
    if not passed:
        why.append("the part does not pass")
    if sec is None or sec.ok is not True:
        why.append("SECTION_MIN not met" if sec is not None and sec.ok is False else "SECTION_MIN not measured")
    if wall is None or wall.ok is not True:
        why.append("minimum wall not met" if wall is not None and wall.ok is False else
                   ("wall could not be measured (grader error)" if wall_error else "wall not measured"))
    out["wall"] = None if wall is None else {"ok": wall.ok, "value": wall.value, "reason": wall.reason}
    if wall_error:
        out["wall"] = {"ok": None, "value": None, "reason": f"could not be measured: {wall_error}", "error": True}
    out["counts"] = not why
    if why:
        out["why_not"] = "; ".join(why)
    from .par import SUSPICIOUS

    if out["counts"] and out["ratio"] > SUSPICIOUS:
        # par is meant to be an honest lean answer: beating it by this much
        # says par, or a requirement, is wrong for this task
        out["beats_par_by"] = out["ratio"]
        out["note"] = (f"beats par by {out['ratio']:.2f} x (more than {SUSPICIOUS:g} x): "
                       "par or a requirement may be wrong for this task")
    return out


def _wall_only(task: Task, ctx):
    """Only the manufacturing checker's wall rule, through its own functions."""
    from .checks.manufacturing import settings
    from .dfm import PROCESSES, Part, load_process

    m = settings(task)
    proc = load_process(m["process"])
    part = Part(ctx.solid, proc.settings["sample_spacing"])
    for r in PROCESSES[m["process"]].independent(part, proc):
        if r.id.endswith("_WALL_MIN"):
            return CheckResult(r.id, bool(r.ok), r.reason, r.value, r.unit, required=False)
    return None


def sha256(path: str) -> str | None:
    if not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def log_attempt(task_dir: str, step_path: str, report: Report, who: str = "verify", **kw) -> int:
    """Archive the attempt under <task>/attempts/NNNN/ and append its line to
    <task>/attempts.jsonl; returns the attempt number. Keyword options
    (script, feedback, feedback_format, note, tool, reported) are passed to
    ``archive.record_attempt``."""
    from .archive import record_attempt

    return record_attempt(task_dir, step_path, report, by=who, **kw)
