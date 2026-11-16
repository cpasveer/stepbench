"""Material actually used to make a part, per process: for a material score.

    from jointbench.dfm.material import material
    m = material("part.step", "fdm")      # grams and a breakdown

Settings (perimeters, infill, densities...) live in the process file's
``[material]`` table, so a brief can state them.

* **Filament (fdm)**: a printed part is a shell plus sparse infill, not a
  solid. Shell: every bit of surface carries a skin -- perimeters on side
  faces (3 x 0.4 mm), solid layers on faces within 45 deg of horizontal
  (4 x 0.2 mm) -- but never more than half the wall it sits on (a wall
  thinner than two skins is solid). The rest of the volume is interior,
  filled at the infill share. Support, under the overhangs the overhang
  rule flags in the build direction, is estimated as the space between
  each overhanging bit of surface and whatever is below it (the part or
  the bed), at the support density. It is reported separately and counted
  only if asked for (``with_support``), since the rules today forbid
  support altogether.
* **Powder bed (pbf)**: fully dense, so the solid volume, plus powder
  sealed in any hollow (it cannot get out, so it is shipped). Loose powder
  around the part is reused and not counted.
* **Moulding**: the solid volume per shot; the thickest section is
  reported beside it, as cooling time grows with its square.

Sampled where it says so (shell and support: the same 1.5 mm sample mesh
and exact rays as the rules); volumes and hollows exact.
"""
from __future__ import annotations

import math
import os
import tomllib

import numpy as np

from .rules import HERE


def settings(process: str) -> dict:
    with open(os.path.join(HERE, f"{process}.toml"), "rb") as fh:
        return tomllib.load(fh)["material"]


def material(step_path: str, process: str, direction=None, with_support: bool | None = None,
             support: str | None = None) -> dict:
    """``direction``: the declared build or pull direction (None: the checker
    chooses). ``support`` (filament): "allowed" counts support as material;
    "forbidden" (default) reports it but does not count it. ``with_support``
    overrides the counting either way."""
    from . import check, part_for

    m = settings(process)
    rho = float(m["density"])  # g/cm3
    part = part_for(step_path, 1.5)
    vol = part.solid.volume  # mm3, exact
    out = {"process": process, "material": m["name"], "density_g_cm3": rho, "solid_volume_cm3": vol / 1000,
           "settings": {k: v for k, v in m.items() if k not in ("name", "density", "source")},
           "source": m["source"]}

    if process == "fdm":
        rep = check(step_path, "fdm", direction, support=support)
        if with_support is None:
            with_support = rep.support == "allowed"
        d = np.array(rep.direction, float)
        t = part.thickness()
        up = np.abs(part.N @ d)
        flat = up >= math.cos(math.radians(m["flat_angle"]))
        skin = np.where(flat, m["top_bottom_layers"] * m["layer_height"], m["perimeters"] * m["line_width"])
        half = np.where(np.isfinite(t), t / 2, np.inf)
        shell = float((part.W * np.minimum(skin, half)).sum())
        shell = min(shell, vol)
        interior = vol - shell
        infill = m["infill"] * interior
        # support: under the samples the overhang and bridge rules flag, down
        # to the part or the bed (the same columns the removability rule judges)
        from .fdm import support_columns, support_samples

        need = support_samples(part, rep.rules)
        sup = float(sum(part.W[i] * abs(float(part.N[i] @ d)) * h
                        for i, (h, _) in support_columns(part, d, need).items()))
        support = float(m["support_density"] * sup)
        used = shell + infill + (support if with_support else 0.0)
        out.update({
            "build_direction": rep.direction, "direction_chosen": rep.direction_chosen,
            "shell_cm3": shell / 1000, "interior_cm3": interior / 1000, "infill_cm3": infill / 1000,
            "support_cm3": support / 1000, "support_counted": bool(with_support), "support": rep.support,
            "used_cm3": used / 1000, "grams": used / 1000 * rho,
            "share_of_solid": used / vol if vol else math.nan,
        })
    elif process == "pbf":
        trapped = 0.0
        for _, faces in part.void_faces():
            import build123d as bd

            from OCP.BRep import BRep_Builder
            from OCP.TopoDS import TopoDS_Shell

            sh = TopoDS_Shell()
            bb = BRep_Builder()
            bb.MakeShell(sh)
            for f in faces:
                bb.Add(sh, part.faces[f])
            trapped += abs(bd.Solid.make_solid(bd.Shell(sh)).volume)
        powder_packing = 0.5  # loose PA12 powder is about half as dense as the sintered part
        used = vol + powder_packing * trapped
        out.update({"trapped_powder_cm3": trapped / 1000, "used_cm3": used / 1000, "grams": used / 1000 * rho,
                    "note": "loose powder around the part is reused and not counted; powder sealed in a hollow "
                            "is counted at half the part's density"})
    elif process == "moulding":
        thick, _ = part.ball_max()
        out.update({"used_cm3": vol / 1000, "grams": vol / 1000 * rho, "thickest_section_mm": thick,
                    "note": "cooling (and cycle) time grows with the square of the thickest section"})
    else:
        raise ValueError(f"no material estimate for process {process!r}")
    return out
