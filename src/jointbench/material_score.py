"""The level 3 score: the material the stated process uses, among parts that pass.

A task asks for it with ``tolerances["score"] = "material"``. Lower is better;
the figure is comparable between parts of the same task and process.

* **Filament (fdm):** the plastic the printer lays down: a shell (perimeters
  on side faces, solid layers on top and bottom faces) and sparse infill, in
  the build direction the manufacturing checker chose. Support is forbidden
  in these tasks, so none is counted. Settings in ``dfm/processes/fdm.toml``.
* **Powder (pbf):** the solid volume, plus powder sealed in any hollow
  (it cannot get out, so it is shipped).

Measured by ``dfm.material`` (sampled shell, exact volumes).
"""
from __future__ import annotations

MEASURE = {"fdm": "printed plastic: shell and infill as the printer lays them",
           "pbf": "solid volume, plus any powder sealed inside"}


def asked(task) -> bool:
    return task.tolerances.get("score") == "material"


def process_of(task) -> str:
    return ((task.tolerances.get("manufacturing") or {}).get("process")) or "fdm"


def material_used(task, step_path: str, direction=None) -> dict:
    """The score figure for one part (cm3 and grams), with its breakdown."""
    from .dfm.material import material

    proc = process_of(task)
    if proc == "fdm":
        m = material(step_path, "fdm", direction, support="forbidden", with_support=False)
        parts = {"shell_cm3": round(m["shell_cm3"], 3), "infill_cm3": round(m["infill_cm3"], 3)}
    elif proc == "pbf":
        m = material(step_path, "pbf")
        parts = {"trapped_powder_cm3": round(m["trapped_powder_cm3"], 3)}
    else:
        raise ValueError(f"no material score for process {proc!r}")
    return {"process": proc, "measure": MEASURE[proc], "used_cm3": round(m["used_cm3"], 3),
            "grams": round(m["grams"], 2), "solid_cm3": round(m["solid_volume_cm3"], 3), **parts}


def brief_line(task) -> str:
    """How the score is stated in the brief."""
    from .dfm.material import settings

    proc = process_of(task)
    if proc == "fdm":
        s = settings("fdm")
        how = (f"the plastic the printer lays down: walls of {s['perimeters']} x {s['line_width']:g} mm on the "
               f"sides, {s['top_bottom_layers']} solid layers of {s['layer_height']:g} mm on top and bottom faces, "
               f"and {s['infill']:.0%} infill inside")
    else:
        how = "the part's solid volume, plus any powder sealed inside it"
    return ("- **Score** -- among parts that meet every requirement above, the one that uses the least "
            f"material wins: {how}, each part as printed in the direction the checker finds for it. Stiffness beyond the requirement earns nothing.")
