"""COMPLIANCE: the part lets mount B make the task's stated small motion
easily and resists every other (a flexure, test E1). Only where a task
states a travel (``tolerances.compliance``) and lists the check.

One analysis in the printed material (``jointbench.stiffness.compliance``)
gives three figures, each judged against the task's limits: the stiffness
along the travel (within a band), every other way of loading B at least a
stated multiple stiffer, and the stress when B is pushed the full travel
(STRENGTH's rules: loaded inside corners rounded, highest stress under the
allowable). The same analysis gives STIFFNESS with the travel left out, so
the two checks share it (cached on the grading context).
"""
from __future__ import annotations

import os
import shutil
import tempfile

from .base import CheckResult, Context

NAME = "COMPLIANCE"
GROUP = "sturdy"


def analysed(ctx: Context):
    """The compliance analysis of the answer, once per grading."""
    rep = getattr(ctx, "_compliance", None)
    if rep is None:
        from ..stiffness.compliance import screened

        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, "task.json"), "w", encoding="utf-8") as f:
                f.write(ctx.task.dumps())
            p = os.path.join(tmp, "output.step")
            shutil.copyfile(ctx.step_path, p)
            rep = screened(tmp, p, average_radius=0.0)
        ctx._compliance = rep
    return rep


def run(ctx: Context) -> CheckResult:
    if not ctx.task.tolerances.get("compliance"):
        return CheckResult(NAME, None, "no travel stated for this task")
    rep = analysed(ctx)
    if not rep.ok:
        return CheckResult(NAME, False, f"could not be analysed: {rep.error}")
    lo, hi = rep.band
    unit = "mm" if rep.kind == "slide" else "deg"
    parts = [f"along the travel {rep.stiffness:.3g} {rep.unit} (band {lo:g} to {hi:g}: {rep.band_verdict})",
             f"every other way {rep.ratio:.3g} x stiffer (need {rep.ratio_required:g}: {rep.ratio_verdict}; "
             f"softest: {rep.other_description})",
             f"at the full {rep.travel:g} {unit}: {rep.stress_verdict}, {rep.stress['reason'] if rep.stress else ''}"]
    flag = f" -- {rep.fallback}" if rep.fallback else ""
    worst = min(rep.ratio / rep.ratio_required, rep.stiffness / lo, hi / rep.stiffness,
                1 / rep.stress["stress_ratio"] if rep.stress and rep.stress.get("stress_ratio") else 1e9)
    return CheckResult(NAME, rep.passed, "; ".join(parts) + flag, round(worst, 4), "ratio", rep.to_json())
