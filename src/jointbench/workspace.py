"""Writing a task out as a folder an agent (or a person) can work in."""
from __future__ import annotations

import json
import os

import build123d as bd

from . import NAME
from .catalogue import BOLTS, fixed_part_solid
from .geometry import write_step
from .task import Task

SUBMISSION_STUB = '''"""Your answer to this task.

Write ONE solid to output.step (next to this file) that bolts to both fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely, or model
the part in any CAD program and save it as output.step instead.

    python submission.py                 # writes output.step
    CLI_NAME verify . --by <your name> --tool <the CAD tool you used>

(the brief, README.md, says how to run them.)
"""
import json
import pathlib

from build123d import Box, Part, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]  # [{"part", "origin", "normal"}]
BOLTS = POSE["bolts"]  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]


def build() -> Part:
    # Placeholder: a 10 mm cube. It will fail. Replace it with your connector.
    return Box(10, 10, 10)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
'''


def _v(v):
    return "(" + ", ".join(f"{x:.2f}" for x in v) + ")"


def _situation(task: Task) -> str:
    from . import families

    return families.level_module(task).SITUATION


def brief(task: Task) -> str:
    from . import families

    fam = families.of(task)
    if fam.brief is not None:
        return fam.brief(task)
    return _connect_brief(task)


def _connect_brief(task: Task) -> str:
    t = task.tolerances
    faces = {f.part: f for f in task.faces(0)}
    n_parts = len(task.parts)
    all_ = "both" if n_parts == 2 else f"all {n_parts}"
    either = "either fixed part" if n_parts == 2 else "any fixed part"
    obstacles = task.obstacles_at(0)
    lines = [
        f"# Task {task.id} -- design the connecting part",
        "",
        f"Level {task.level}. Units: millimetres. One pose.",
        "",
        _situation(task),
        "",
        f"Design **one solid part** that bolts to {all_}. Any shape that works",
        "passes; nothing is compared to a reference.",
        "",
        "Model it in any CAD program or library and save it as `output.step` in this",
        "folder (`submission.py` is an optional build123d starting point), then run",
        f"`uv run {NAME} verify <this folder> --tool <the CAD tool you used>`.",
        "",
        "## The fixed parts",
        "",
    ]
    for name, itf in task.parts.items():
        f = faces[name]
        lines += [
            f"**{name}** -- {itf.description}.",
            f"Mounting face: a {itf.face_w:g} x {itf.face_h:g} mm flat face through {_v(f.origin)}, "
            f"outward normal {_v(f.normal)}. The part's body lies behind the face; "
            "the space in front of it is yours.",
        ]
        if itf.protrusions:
            lines.append("Sticks out of the face into your space: " + "; ".join(
                f"{p.name.replace('_', ' ')} {2 * p.radius:g} mm dia x {p.height:g} mm, at the face centre"
                for p in itf.protrusions) + ".")
        lines.append(f"Fixed geometry: `fixed_{name}.step` (world coordinates).")
        lines.append("")
    if obstacles:
        lines += [
            "## Obstacles -- keep clear",
            "",
            "Fixed boxes your part must not touch (other equipment, a cable run, a",
            "guard). Each is a box of the given size, centred on its centre, with its",
            "edges along the three given axis directions. Geometry: `obstacles.step`.",
            "",
            "| obstacle | centre | size along axes 1, 2, 3 | axis 1 | axis 2 | axis 3 |",
            "|---|---|---|---|---|---|",
        ]
        for o in obstacles:
            j = o.to_json()
            lines.append(f"| {o.name} | {_v(j['center'])} | "
                         + " x ".join(f"{v:.2f}" for v in o.size)
                         + " | " + " | ".join(_v(a) for a in j["axes"]) + " |")
        lines.append("")
    lines += [
        "## The bolts",
        "",
        "Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped",
        "hole in the fixed part and is put in from your side, so its head bears on",
        "your part. The axis points from the fixed part into your part.",
        "",
        "| bolt | size | position on face | axis | clearance hole dia | grip |",
        "|---|---|---|---|---|---|",
    ]
    for b in task.bolts(0):
        bt = BOLTS[b.size]
        lines.append(f"| {b.id} | {b.size} | {_v(b.position)} | {_v(b.axis)} | "
                     f"{bt.clear_fine:g}-{bt.clear_coarse:g} | {bt.grip_min:g}-{bt.grip_max:g} |")
    lines += [
        "",
        "*Grip* is the thickness of your part under the head: the distance along",
        "the axis from the mounting face to where the hole ends.",
        "",
        "## What is checked",
        "",
        "Your part passes when it meets every requirement marked *required*: "
        + ", ".join(f"{g} ({s})" for g, s in task.requirements().items())
        + ". Fits means the checks below up to the obstacles and tool access; sturdy is "
        "`SECTION_MIN`; clean is the modelling-quality rules; makeable is the manufacturing "
        "rules. Reported requirements are measured on request and never change the verdict."
        + (" Difficulty settings: " + ", ".join(f"{k} {v}" for k, v in task.meta["knobs"].items()) + "."
           if task.meta.get("knobs") else ""),
        "",
        "- `FILE_VALID` -- `output.step` holds exactly one valid, closed solid.",
        f"- `HOLES` -- a through hole on every bolt axis, within {t['hole_axis_offset_max']:g} mm, "
        "diameter in the clearance range, grip in range.",
        f"- `SEATING` -- a flat face of your part lies on each mounting face (stand-off at most "
        f"{t['seat_gap_max']:g} mm) and covers at least {t['seat_contact_min']:.0%} of the ring "
        "around each bolt, from the hole out to the head diameter.",
        f"- `NO_INTERFERENCE` -- your part does not overlap {either} (limit "
        f"{t['overlap_volume_max']:g} mm3), and each bolt has room: its shank, and its head with "
        f"{t['head_clearance_radial']:g} mm clearance around and {t['head_clearance_axial']:g} mm above.",
    ]
    lines += required_lines(task)  # the fixture rule, clean modelling: stated with their limits where required
    if obstacles:
        lines.append("- `CLEAR_PATH` -- your part does not touch any obstacle (same overlap limit).")
    if "tool_length" in t:
        lines.append(
            f"- `TOOL_ACCESS` -- every bolt can be turned with a straight tool: above each head, "
            f"along the bolt axis, a cylinder of the head diameter + {t['tool_diameter_extra']:g} mm and "
            f"{t['tool_length']:g} mm long, starting at the top of the head, must be free of your part, "
            "the fixed parts and the obstacles.")
    from .checks.manufacturing import settings as _mfg
    from .checks.section_min import required_area

    req = set(t.get("required_checks", []))
    names = list(task.parts)
    pairs = [(x, y) for i, x in enumerate(names) for y in names[i + 1:]]
    lines.append(
        f"- `SECTION_MIN` ({'required' if 'SECTION_MIN' in req else 'reported, not required here'}) -- "
        "the material joining any two mounts keeps a minimum cross-section everywhere along the way: "
        "the smallest flat cut that separates one mount's bolts from the other's must have at least "
        + "; ".join(f"{required_area(task, x, y):.1f} mm2 between {x} and {y}" for x, y in pairs)
        + f" ({t.get('section_factor', 2.0):g} x the combined bolt shank area of the weaker mount).")
    m = _mfg(task)
    proc = {"fdm": "filament printing (FDM)", "pbf": "powder bed fusion (selective laser sintering)",
            "moulding": "injection moulding"}.get(m["process"], m["process"])
    lines.append(
        f"- Manufacturing ({'required' if 'MANUFACTURING' in req else 'reported, not required here'}) -- "
        f"the part is to be made by {proc}, quantity {m['quantity']}; each rule of that process "
        "(minimum wall, overhang, ...) is reported as a check of its own.")
    if task.stiffness:
        st = task.stiffness
        if st.get("mode") == "spec":
            lines.append(
                "- `STIFFNESS` (required) -- the part must be stiff enough between its mounts. "
                + " ".join(m.get("statement") or (
                    f"Mount {m['mount']}: at least {m['required']:.1f} N/mm ({m['design_load_N']:g} N, "
                    f"at most {m['allowed_deflection_mm']:.3g} mm).") for m in st["per_mount"])
                + f" ({st['how']}. The checker analyses the part as {_st_material()} and resolves "
                f"+-{st['precision']:g}%; a result inside that band counts as a pass and is flagged.)")
        else:
            lines.append(
                "- `STIFFNESS` (required) -- with one mount held, every other mount is loaded in turn by "
                "forces and moments through its seat and bolt holes; the weakest way of loading each mount "
                f"must reach {st['min']:.1f} N/mm (the checker resolves +-{st['precision']:g}%, "
                "a result inside that band counts as a pass and is flagged).")
    lines += [
        "",
        "**Material score.** A part that passes also gets a material figure: par volume / your volume "
        + (f"(par here: {task.par['volume']:.0f} mm3)" if task.par else "(no par for this task)")
        + ". 1.0 is par, higher is leaner. Par is a lean answer the generator built, not an optimum, "
        "so beating it is possible. A part counts for the score only if it also keeps the minimum "
        "section and the process's minimum wall."
        + ("" if task.stiffness else " Stiffness and strength are not judged in this task."),
    ]
    lines += [
        "",
        "Volume and bounding box are reported, not graded.",
        "",
        "This folder may hold `_solution/` with the generator's reference answer, for",
        "practice. A folder handed out for a recorded run never contains it.",
        "",
    ]
    return "\n".join(lines)


def write_workspace(task: Task, reference: bd.Part, out_dir: str, extra: dict | None = None) -> str:
    d = os.path.join(out_dir, task.id)
    os.makedirs(os.path.join(d, "_solution"), exist_ok=True)
    with open(os.path.join(d, "task.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(task.dumps())
    with open(os.path.join(d, "README.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(brief(task))
    from . import families

    fam = families.of(task)
    stub = fam.submission(task) if fam.submission is not None else SUBMISSION_STUB.replace("CLI_NAME", NAME)
    sub = os.path.join(d, "submission.py")
    with open(sub, "w", encoding="utf-8", newline="\n") as f:
        f.write(stub)
    for name, itf in task.parts.items():
        if itf.kind == "solid":
            # the solid in its own frame, which the grader reads back
            write_step(fixed_part_solid(itf), os.path.join(d, itf.params["step"]))
        # the first pose that places it (a pose may place only some parts)
        pose = next(p for p in task.poses if name in p)
        solid = fixed_part_solid(itf, d).moved(pose[name].location())
        write_step(solid, os.path.join(d, f"fixed_{name}.step"))
    if task.obstacles_at(0):
        boxes = [o.solid() for o in task.obstacles_at(0)]
        write_step(bd.Compound(boxes), os.path.join(d, "obstacles.step"))
    write_step(reference, os.path.join(d, "_solution", "reference.step"))
    if isinstance((extra or {}).get("draw"), dict) and "placement" in extra["draw"]:
        # an adapt task: where the reference goes in each arrangement, beside it
        with open(os.path.join(d, "_solution", "placement.json"), "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(extra["draw"]["placement"], indent=2) + "\n")
    extra = dict(extra or {})
    if isinstance(extra.get("draw"), dict):
        # timings differ from run to run; the folder must not
        extra["draw"] = {k: v for k, v in extra["draw"].items() if k != "timing_s"}
    with open(os.path.join(d, "_solution", "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(extra, indent=2, sort_keys=True) + "\n")
    return d


def _st_material() -> str:
    from .stiffness import settings

    m = settings()["material"]
    return f"{m['name']} (E {m['E']:g} N/mm2)"


def required_lines(task: Task) -> list[str]:
    """The brief's bullet for every requirement the task lists beyond fit,
    in plain words with its limits: what a round's brief must state so an
    agent can meet it. Fit's own checks are written by the round."""
    t = task.tolerances
    req = set(t.get("required_checks", []))
    out = []
    if "KEY_TURN" in req:
        from .catalogue import KEY_SWING, KEY_TILT_MAX, KEYS

        # the fixed parts' bolts in every pose (an adapt task's arrangements), and any bolts the
        # answer adds itself (an assembly's joint)
        sizes = {b.size for k in range(len(task.poses)) for b in task.bolts(k) if b.size in KEYS}
        if t.get("joint") is not None:  # an assembly's joint bolts: the allowed sizes, else the whole nut catalogue
            from .catalogue import NUTS

            sizes |= {s for s in (t["joint"].get("sizes") or NUTS) if s in KEYS}
        sizes = sorted(sizes, key=lambda n: KEYS[n].s)
        keys = "; ".join(f"{n}: {KEYS[n].s:g} mm key, legs {KEYS[n].short_leg:g} and {KEYS[n].long_leg:g} mm"
                         for n in sizes)
        out.append(
            f"- `KEY_TURN` -- every bolt can be turned with its standard L-shaped hex key (ISO 2936), not just "
            f"reached: the short leg in the socket, the long leg sticking out sideways, the key leaning at most "
            f"{KEY_TILT_MAX:g} degrees off the bolt axis (a ball-end key), it must swing {KEY_SWING:g} degrees "
            f"round the bolt without touching your part, the fixed parts or any obstacle. Keys ({keys}; lengths "
            "over the outside of the bend).")
    if "SEAT_INSIDE" in req:
        out.append(
            f"- `SEAT_INSIDE` -- where your part lies on a mounting face it stays at least {t.get('seat_setback', 1.0):g} mm "
            f"inside that face's outline (at most {t.get('seat_overhang_max', 0.5):g} mm2 of seat beyond that line): "
            "no larger than the part it is bolted to, and not flush to its edge. Material that does not touch the "
            "mounting face is free.")
    if "CLEAN" in req:
        from .dfm.rules import load_file
        from .quality import RULES

        q = load_file(RULES)
        out.append("- `CLEAN` -- the part is cleanly modelled, as an engineer would accept the file: "
                   + "; ".join(f"{r.title[0].lower() + r.title[1:]} ({r.limit_text()})" for r in q.rules.values())
                   + ". The report shows these as `Q_...` lines.")
    if "COMPLIANCE" in req and t.get("compliance"):
        out.append(compliance_line(task))
    return out


def compliance_line(task: Task) -> str:
    """The brief's bullet for COMPLIANCE: the stated motion and every limit."""
    from .stiffness import settings as st_settings
    from .stiffness.strength import material as st_material

    c = task.tolerances["compliance"]
    names = list(task.parts)
    held, moved = names[0], names[1]
    m = st_settings()["materials"]["fdm"]
    allow = st_material("fdm")["allowable"]
    faces = {f.part: f for f in task.faces(0)}
    span = float(sum((a - b) ** 2 for a, b in zip(faces[moved].origin, faces[held].origin)) ** 0.5)
    lo, hi = c["band"]
    if c["kind"] == "slide":
        motion = (f"mount {moved} must be able to slide {c['travel']:g} mm, either way, along the direction "
                  f"{_v(c['direction'])} (world axes)")
        along = (f"pushed along that direction through its seat and bolt holes, and otherwise free, mount {moved} "
                 f"moves along it with a stiffness of {lo:g} to {hi:g} N/mm")
        full = f"pushed the full {c['travel']:g} mm"
    else:
        motion = (f"mount {moved} must be able to turn {c['travel']:g} deg, either way, about the axis through "
                  f"{_v(c['point'])} along {_v(c['axis'])} (world axes)")
        along = (f"turned about that axis through its seat and bolt holes, and otherwise free, mount {moved} "
                 f"turns with a stiffness of {lo:g} to {hi:g} N mm per degree")
        full = f"turned the full {c['travel']:g} deg"
    return (
        f"- `COMPLIANCE` -- the part is a guide for one small motion. With mount {held} held at its seat and bolt "
        f"holes, {motion}, easily, and resist every other motion. Judged on the part as printed in PLA, analysed "
        f"as one linear elastic material (E {m['E']:g} N/mm2, Poisson {m['nu']:g}): (1) {along}; (2) every "
        f"other way of loading mount {moved} -- any force, any moment, or any combination at right angles to the "
        f"travel, a moment counting as a force at the {span:.0f} mm span between the mounts -- is at least "
        f"{c['ratio']:g} times stiffer; (3) {full}, the highest stress stays under PLA's allowable "
        f"{allow:g} N/mm2 ({m['strength']:g} N/mm2 printed strength over a safety factor of {m['safety']:g}), "
        f"and every inside corner that carries load at that push is rounded at least "
        f"{st_settings()['strength']['min_radius']:g} mm. The checker resolves "
        f"+-{st_settings()['mesh']['precision']:g}%; a result inside that band counts as a pass and is flagged.")


PROCESS_SENTENCE = {
    # the print-rule decision (PRINT-RULE-jointbench.md): the process as a fact, no rules listed
    "fdm": ("It will be printed in PLA on a common filament printer (0.4 mm nozzle, 0.2 mm layers), "
            "without support material, and used as it comes off the printer."),
    "pbf": ("It will be printed in PA12 nylon by powder bed fusion (selective laser sintering, the part "
            "built up in a bed of powder that holds it while it prints) and used as it comes out of the printer."),
}


def process_sentence(task: Task) -> str | None:
    """The brief's sentence for the stated process, where the task requires its rules."""
    if "MANUFACTURING" not in set(task.tolerances.get("required_checks", [])):
        return None
    proc = ((task.tolerances.get("manufacturing") or {}).get("process")) or "fdm"
    return PROCESS_SENTENCE.get(proc)
