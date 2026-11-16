"""NO_INTERFERENCE: nothing occupies the same space, bolts included.

Three exact boolean intersections, all measured as volume:

* the submitted part against each fixed part (minus any region the task
  declares as intended overlap -- an interference fit -- on that part);
* each bolt's keep-out against the submitted part;
* each bolt's keep-out against the fixed parts.

A bolt's keep-out is its shank (nominal diameter, from the mounting face up
to the measured grip) plus its head with clearance (head diameter plus
``head_clearance_radial`` all round, head height plus
``head_clearance_axial`` above). Level 1 uses tapped holes, so there is no
nut; a nut or washer would be one more cylinder in ``keepout``.
"""
from __future__ import annotations

import build123d as bd

from ..catalogue import BOLTS
from .base import CheckResult, Context, volume

NAME = "NO_INTERFERENCE"


def keepout(bolt, grip: float, tol: dict) -> bd.Part:
    b = BOLTS[bolt.size]
    pl = bd.Plane(origin=bolt.position, z_dir=bolt.axis)
    head_r = b.head_d / 2 + tol["head_clearance_radial"]
    head_h = b.head_h + tol["head_clearance_axial"]
    shank = pl * bd.Pos(0, 0, grip / 2) * bd.Cylinder(b.d / 2, grip)
    head = pl * bd.Pos(0, 0, grip + head_h / 2) * bd.Cylinder(head_r, head_h)
    return shank + head


def run(ctx: Context) -> CheckResult:
    t = ctx.task
    lim = t.tolerances["overlap_volume_max"]
    rows, worst, worst_what = [], 0.0, ""
    for k in ctx.poses():
        fixed = ctx.fixed(k)
        part = ctx.solid_at(k)
        if ctx.scene is not None:
            # the family moves the neighbours with the answer: they must not
            # foul each other either (a wrong answer can drive a crank into a rocker)
            names = list(fixed)
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    v = volume(fixed[a] & fixed[b])
                    rows.append({"what": f"fixed {a} vs fixed {b} at pose {k}", "volume": round(v, 4)})
                    if v > worst:
                        worst, worst_what = v, f"fixed parts {a} and {b} collide at pose {k}"
        for name, fx in fixed.items():
            ov = part & fx
            regions = t.overlap_regions(name)
            if regions and ov is not None:
                meant = volume(ov)
                for reg in regions:
                    ov = ov - reg
                v = volume(ov)
                rows.append({"what": f"intended overlap with fixed {name} (not counted)",
                             "volume": round(meant - v, 4)})
            else:
                v = volume(ov)
            rows.append({"what": f"part vs fixed {name}", "volume": round(v, 4)})
            if v > worst:
                worst, worst_what = v, f"part overlaps fixed part {name}"
        for bolt in t.bolts(ctx.stored(k)):
            ko = keepout(bolt, ctx.grips.get((k, bolt.id), BOLTS[bolt.size].grip_min), t.tolerances)
            v = volume(part & ko)
            rows.append({"what": f"bolt {bolt.id} vs part", "volume": round(v, 4)})
            if v > worst:
                worst, worst_what = v, f"part blocks bolt {bolt.id} (shank or head space)"
            for name, fx in fixed.items():
                v = volume(ko & fx)
                rows.append({"what": f"bolt {bolt.id} vs fixed {name}", "volume": round(v, 4)})
                if v > worst:
                    worst, worst_what = v, f"bolt {bolt.id} head would hit fixed part {name}"
    worst = round(worst, 4)
    if worst > lim:
        n_bad = sum(1 for r in rows if r["volume"] > lim)
        extra = f" ({n_bad} clashes)" if n_bad > 1 else ""
        return CheckResult(NAME, False, f"{worst_what}: {worst:.3f} mm3 overlap{extra}", worst, "mm3",
                           {"overlaps": rows})
    return CheckResult(NAME, True, f"no overlap with fixed parts or bolt space "
                       f"(worst {worst:.4f} mm3, limit {lim:g})", worst, "mm3", {"overlaps": rows})
