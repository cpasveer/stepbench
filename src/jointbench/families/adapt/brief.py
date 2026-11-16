"""The brief for an adapt task: both arrangements in full, and the placement
file. It states facts and requirements only; nothing in it says how the two
arrangements relate (that is the task)."""
from __future__ import annotations

from ... import NAME
from ...catalogue import BOLTS
from ...task import Task
from ...workspace import process_sentence, required_lines
from .grading import PLACEMENT_FILE, arrangements


def _v(p) -> str:
    return "(" + ", ".join(f"{x:.2f}" for x in p) + ")"


def _parts_text(task: Task, arr: dict) -> str:
    faces = {f.part: f for f in task.faces(arr["pose"])}
    out = []
    for name in arr["parts"]:
        itf, f = task.parts[name], faces[name]
        text = (f"**{name}** -- {itf.description}.\nMounting face: a {itf.face_w:g} x {itf.face_h:g} mm flat face "
                f"through {_v(f.origin)}, outward normal {_v(f.normal)}. The part's body lies behind the face; the "
                "space in front of it is yours.")
        if itf.protrusions:
            text += "\nSticks out of the face into your space: " + "; ".join(
                f"{p.name.replace('_', ' ')} {2 * p.radius:g} mm dia x {p.height:g} mm, at the face centre"
                for p in itf.protrusions) + "."
        text += f"\nFixed geometry: `fixed_{name}.step` (in arrangement {arr['name']}'s coordinates)."
        out.append(text)
    return "\n\n".join(out)


def _load_text(arr: dict) -> str:
    st = arr.get("stiffness")
    if not st:
        return ""
    return "\n**Load in this arrangement.** " + " ".join(m["statement"] for m in st["per_mount"]) + "\n"


def _score_line() -> str:
    from ...dfm.material import settings

    s = settings("fdm")
    return ("- **Score** -- the part's printed weight is measured: the plastic the printer lays down, walls of "
            f"{s['perimeters']} x {s['line_width']:g} mm on the sides, {s['top_bottom_layers']} solid layers of "
            f"{s['layer_height']:g} mm on top and bottom faces and {s['infill']:.0%} infill inside, in the print "
            "direction the checker finds for the part as handed in. Among parts that meet every requirement, "
            "lighter is better. Stiffness beyond the requirement earns nothing.")


def _bolt_rows(task: Task, arr: dict) -> str:
    rows = []
    for b in task.bolts(arr["pose"]):
        bt = BOLTS[b.size]
        rows.append(f"| {b.id} | {b.size} | {_v(b.position)} | {_v(b.axis)} | {bt.clear_fine:g}-{bt.clear_coarse:g} | "
                    f"{bt.grip_min:g}-{bt.grip_max:g} |")
    return "\n".join(rows)


def _situation(task: Task) -> str:
    """The level module's situation, minus its opening (the brief names the arrangements itself)."""
    from ... import families

    text = " ".join(families.level_module(task).SITUATION.split())
    return text.split(". ", 1)[1] if text.startswith("Two arrangements") else text


def brief(task: Task, run: str | None = None) -> str:
    """``run``: how the brief tells the reader to call the tools (default
    ``uv run``; a round may give the full path of its own environment)."""
    t = task.tolerances
    arrs = arrangements(task)
    names = [a["name"] for a in arrs]
    py = f"{run}/python" if run else "uv run python"
    cli = f"{run}/{NAME}" if run else f"uv run {NAME}"
    made = process_sentence(task)
    sections = []
    for a in arrs:
        sections.append(f"""## Arrangement {a['name']}: fixed parts {' and '.join(a['parts'])}

All positions and directions in this section are in arrangement {a['name']}'s coordinates.

{_parts_text(task, a)}
{_load_text(a)}
| bolt | size | position on the surface | axis | clearance hole dia | grip |
|---|---|---|---|---|---|
{_bolt_rows(task, a)}""")
    checks = [
        "- `PLACEMENT` -- `placement.json` gives a rigid move for every arrangement, in the form above.",
        "- `FILE_VALID` -- `output.step` holds exactly one valid, closed solid.",
        f"- `HOLES` -- a through hole on every bolt axis, within {t['hole_axis_offset_max']:g} mm, diameter in the "
        "clearance range, grip in range.",
        f"- `SEATING` -- your part lies on each fixed part around every bolt, covering at least "
        f"{t['seat_contact_min']:.0%} of the ring from the hole out to the head diameter (stand-off at most "
        f"{t['seat_gap_max']:g} mm).",
        f"- `NO_INTERFERENCE` -- your part does not overlap any fixed part (limit {t['overlap_volume_max']:g} mm3), "
        f"and each bolt has room: its shank, and its head with {t['head_clearance_radial']:g} mm clearance around "
        f"and {t['head_clearance_axial']:g} mm above.",
    ] + required_lines(task)
    if made:
        checks.append("- `MANUFACTURING` -- the part can be made as stated above, as it is, with no further work. "
                      "It is printed once: this is judged on `output.step` as you hand it in, not as placed in an "
                      "arrangement; the checker tries standing it on each of its six axis directions and on its "
                      "largest flat faces, and judges it in the direction that works best.")
    loads = [(a, a.get("stiffness")) for a in arrs if a.get("stiffness")]
    if loads:
        st = loads[0][1]
        checks.append("- `STIFFNESS` -- in each arrangement, the load stated in that arrangement's section above. "
                      f"The checker resolves +-{st['precision']:g}%; a result inside that band counts "
                      "as a pass and is flagged.")
    from ... import material_score

    if material_score.asked(task):
        checks.append(_score_line())
    example = ",\n".join(f'  "{n}": {{"rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "translation": [0, 0, 0]}}'
                         for n in names)
    both = " and ".join(names)
    return f"""# Design one part for two arrangements

Units: millimetres.

There are two arrangements of fixed parts, {both}, each given in
coordinates of its own. {_situation(task)}

Design **one solid part** that bolts to both fixed parts of arrangement
{names[0]} and, as the very same part, to both fixed parts of arrangement
{names[1]}. It is used in one arrangement at a time.{(' ' + made) if made else ''}

**Make the best part you can.** Beyond the requirements below, how the part
looks and how it is shaped are your judgement as the designer. Aim for the
part an experienced mechanical designer would be proud of.

Hand in two files in this folder:

- `output.step` -- the part, in coordinates of your choice.
- `{PLACEMENT_FILE}` -- for each arrangement, where the part goes: a rotation
  and a translation that carry the part from the coordinates of
  `output.step` into that arrangement's coordinates.

`submission.py` is a build123d starting point that writes both; you may
replace it entirely.

    {py} submission.py                                   # writes output.step and {PLACEMENT_FILE}
    {cli} verify . --by <your name> --tool build123d  # checks it against every requirement

## How the part is placed and judged

`{PLACEMENT_FILE}` holds one entry per arrangement:

```json
{{
{example}
}}
```

A point p of `output.step` lands at R p + t in that arrangement, where R is
the 3 x 3 `rotation` (given row by row) and t the `translation` (mm). R must
be a rotation: its columns of unit length and square to each other (within
0.0001), and no mirror image. Only rigid moves; the part is never scaled or
mirrored.

The checker places the part into arrangement {names[0]} as you state and checks
every requirement below there, with that arrangement's fixed parts and bolts;
then it does the same in arrangement {names[1]}. The part passes only if it
passes in both. All of the part is present in each arrangement: what serves
one arrangement must also keep clear of the other arrangement's fixed parts
and bolts when the part is placed there.

{(chr(10) * 2).join(sections)}

## The bolts

Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped
hole in the fixed part and is put in from your side, so its head bears on
your part. The axis points from the fixed part into your part.

*Grip* is the thickness of your part under the head: the distance along
the axis from the fixed part's surface to where the hole ends.

## What is checked

Your part passes when it meets every requirement below, in each arrangement.

{chr(10).join(checks)}

The checker may print further lines (section, volume). They are information, not requirements.
"""


SUBMISSION = '''"""Your answer to this task.

Write ONE solid to output.step and, in placement.json, where it goes in each
arrangement (see README.md). Only those two files are graded. You may replace
this script entirely, or model the part in any CAD program instead.
"""
import json
import pathlib

from build123d import Box, Part, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
IDENTITY = {"rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "translation": [0, 0, 0]}


def build() -> Part:
    # Placeholder: a 10 mm cube. It will fail. Replace it with your part.
    return Box(10, 10, 10)


def placement() -> dict:
    # Placeholder: the part as it is, in every arrangement. Replace it.
    return {a["name"]: IDENTITY for a in TASK["meta"]["arrangements"]}


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    (HERE / "placement.json").write_text(json.dumps(placement(), indent=2) + "\\n")
    print("wrote", HERE / "output.step", "and", HERE / "placement.json")
'''


def submission(task: Task) -> str:
    return SUBMISSION
