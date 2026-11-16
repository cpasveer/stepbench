"""Manufacturing rules: can the part be made by the stated process?

This calls the project's own rule checker in ``jointbench.dfm`` through its
public function ``check_solid``; nothing of it is copied here. Every rule it
reports becomes a named check of its own (``FDM_WALL_MIN``,
``FDM_OVERHANG``, ...; ``IM_*`` for moulding), with the checker's reason and
number, so the agent sees exactly which rule failed and where.

The task states the process and quantity in ``tolerances["manufacturing"]``
(default: filament printing, one off). The checker chooses the build or pull
direction itself unless the task states one. Whether the rules decide the
verdict is a knob: they are required only where the task lists
``"MANUFACTURING"`` in ``tolerances["required_checks"]``; otherwise they are
reported.
"""
from __future__ import annotations

from .base import CheckResult, Context

NAME = "MANUFACTURING"
DEFAULT = {"process": "fdm", "quantity": 1, "direction": None}


def settings(task) -> dict:
    return dict(DEFAULT, **task.tolerances.get("manufacturing", {}))


def run(ctx: Context) -> list[CheckResult]:
    from ..dfm import check_solid

    m = settings(ctx.task)
    rep = check_solid(ctx.solid, m["process"], m.get("direction"), "output.step")
    ctx.dfm = rep
    out = []
    for r in rep.rules:
        out.append(CheckResult(r.id, bool(r.ok), r.reason, r.value, r.unit,
                               {"rule": r.title, "limit": r.limit, "where": r.where[:3],
                                "process": rep.process, "direction": rep.direction,
                                "direction_chosen": rep.direction_chosen}))
    return out
