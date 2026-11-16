"""TOOL_ACCESS: a tool can reach every bolt head (level 3 on).

A socket head cap screw is turned with a hex key or a bit driver brought in
straight along the bolt axis. For each bolt we take a straight cylinder from
the top of the head (at the measured grip) outward along the axis: diameter
= head diameter + ``tool_diameter_extra``, length ``tool_length``. It must
be free of the part itself, of every fixed part and of every obstacle.
"""
from __future__ import annotations

import build123d as bd

from ..catalogue import BOLTS, is_nut
from .base import CheckResult, Context, volume

NAME = "TOOL_ACCESS"


def envelope(bolt, grip: float, tol: dict) -> bd.Part:
    b = BOLTS[bolt.size]
    r = (b.head_d + tol["tool_diameter_extra"]) / 2
    L = tol["tool_length"]
    pl = bd.Plane(origin=bolt.position, z_dir=bolt.axis)
    return pl * bd.Pos(0, 0, grip + b.head_h + L / 2) * bd.Cylinder(r, L)


def run(ctx: Context) -> CheckResult:
    t = ctx.task
    lim = t.tolerances["overlap_volume_max"]
    rows, problems = [], []
    for k in ctx.poses():
        others = {f"fixed part {n}": s for n, s in ctx.fixed(k).items()}
        others.update({f"obstacle {n}": s for n, s in ctx.obstacles(k).items()})
        for bolt in t.bolts(ctx.stored(k)):
            if is_nut(bolt.size):
                continue  # level 4: a nut is held with a spanner from the side, not turned along the axis
            grip = ctx.grips.get((k, bolt.id), BOLTS[bolt.size].grip_min)
            env = envelope(bolt, grip, t.tolerances)
            hits = [("the part itself", volume(ctx.solid_at(k) & env))]
            hits += [(n, volume(env & s)) for n, s in others.items()]
            for what, v in hits:
                rows.append({"bolt": bolt.id, "blocked_by": what, "volume": round(v, 4)})
            bad = [(w, v) for w, v in hits if v > lim]
            if bad:
                w, v = max(bad, key=lambda x: x[1])
                problems.append(f"{bolt.id}: tool path blocked by {w} ({v:.3f} mm3)")
    worst = round(max((r["volume"] for r in rows), default=0.0), 4)
    L, extra = t.tolerances["tool_length"], t.tolerances["tool_diameter_extra"]
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult(NAME, False, problems[0] + more, worst, "mm3", {"envelopes": rows, "problems": problems})
    return CheckResult(NAME, True, f"a straight tool ({L:g} mm long, head dia + {extra:g}) reaches every bolt",
                       worst, "mm3", {"envelopes": rows})
