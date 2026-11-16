"""CLEAN: is the part modelled cleanly (no slivers, tiny steps, knife edges)?

Calls the project's modelling-quality checker, ``jointbench.quality``,
through its public function ``check_solid``; nothing of it is copied here.
Each of its rules is reported as a named check of its own. Whether they
decide the verdict is a knob: they are required only where the task lists
``"CLEAN"`` in ``tolerances["required_checks"]``; otherwise reported.
"""
from __future__ import annotations

from .base import CheckResult, Context

NAME = "CLEAN"


def run(ctx: Context) -> list[CheckResult]:
    from ..quality import check_solid

    rep = check_solid(ctx.solid, "output.step")
    return [CheckResult(r.id, bool(r.ok), r.reason, r.value, r.unit,
                        {"rule": r.title, "limit": r.limit, "where": r.where[:3]}) for r in rep.rules]
