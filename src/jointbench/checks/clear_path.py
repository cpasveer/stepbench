"""CLEAR_PATH: the part does not touch any obstacle (level 3 on).

Obstacles are fixed keep-out bodies listed in ``task.json``: other
equipment, a cable run, a guard. Touching is measured as overlap volume,
with the same numerical limit as NO_INTERFERENCE. The obstacle is named in
the reason, so the agent knows which one it hit.
"""
from __future__ import annotations

from .base import CheckResult, Context, volume

NAME = "CLEAR_PATH"


def run(ctx: Context) -> CheckResult:
    t = ctx.task
    lim = t.tolerances["overlap_volume_max"]
    rows, worst, worst_name = [], 0.0, ""
    for k in ctx.poses():
        for name, ob in ctx.obstacles(k).items():
            v = volume(ctx.solid_at(k) & ob)
            rows.append({"obstacle": name, "pose": k, "volume": round(v, 4)})
            if v > worst:
                worst, worst_name = v, name
    worst = round(worst, 4)
    if not rows:
        return CheckResult(NAME, True, "no obstacles in this task", 0.0, "mm3", {"overlaps": rows})
    if worst > lim:
        n_bad = sum(1 for r in rows if r["volume"] > lim)
        extra = f" ({n_bad} obstacles touched)" if n_bad > 1 else ""
        return CheckResult(NAME, False, f"part runs into obstacle {worst_name}: {worst:.3f} mm3 overlap{extra}",
                           worst, "mm3", {"overlaps": rows})
    return CheckResult(NAME, True, f"clear of all {len(rows)} obstacles (worst {worst:.4f} mm3, limit {lim:g})",
                       worst, "mm3", {"overlaps": rows})
