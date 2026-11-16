"""The brief of a facing task, levels 1-3: the round 5 brief (reviewed),
with each level's requirements stated."""
from __future__ import annotations

from ...catalogue import BOLTS
from ...material_score import brief_line
from ...workspace import NAME, PROCESS_SENTENCE, required_lines

v = lambda p: "(" + ", ".join(f"{x:.2f}" for x in p) + ")"  # noqa: E731


def _parts(task) -> str:
    faces = {f.part: f for f in task.faces(0)}
    out = []
    for name, itf in task.parts.items():
        if itf.kind == "pipe":
            pl = task.poses[0][name]
            out.append(f"**{name}** -- {itf.description}. The pipe's axis runs through {v(pl.point((0, 0, 0)))} along "
                       f"{v(pl.vec((1, 0, 0)))}; its outside radius is {itf.face_h / 2:g} mm and it is {itf.face_w:g} mm "
                       "long, centred on its bolts. Your part bears on the pipe's round surface: there is no flat face "
                       f"here.\nFixed geometry: `fixed_{name}.step` (world coordinates).")
            continue
        f = faces[name]
        shape = (f"a flat round face {itf.face_w:g} mm dia" if itf.kind == "round"
                 else f"a {itf.face_w:g} x {itf.face_h:g} mm flat face")
        text = (f"**{name}** -- {itf.description}.\nMounting face: {shape} "
                f"through {v(f.origin)}, outward normal {v(f.normal)}. The part's body lies behind the face; the "
                "space in front of it is yours.")
        if itf.protrusions:
            text += "\nSticks out of the face into your space: " + "; ".join(
                f"{p.name.replace('_', ' ')} {2 * p.radius:g} mm dia x {p.height:g} mm, at the face centre"
                for p in itf.protrusions) + "."
        out.append(text + f"\nFixed geometry: `fixed_{name}.step` (world coordinates).")
    return "\n\n".join(out)


def brief(task) -> str:
    from ... import families

    SITUATION = families.level_module(task).SITUATION
    has_pipe = any(i.kind == "pipe" for i in task.parts.values())
    req = set(task.tolerances.get("required_checks", []))
    made = (" " + PROCESS_SENTENCE["fdm"]) if "MANUFACTURING" in req else ""
    rows = [f"| {b.id} | {b.size} | {v(b.position)} | {v(b.axis)} | {BOLTS[b.size].clear_fine:g}-"
            f"{BOLTS[b.size].clear_coarse:g} | {BOLTS[b.size].grip_min:g}-{BOLTS[b.size].grip_max:g} |"
            for b in task.bolts(0)]
    checks = [
        "- `FILE_VALID` -- `output.step` holds exactly one valid, closed solid.",
        "- `HOLES` -- a through hole on every bolt axis, within 0.1 mm, diameter in the clearance range, grip in range.",
        "- `SEATING` -- your part lies on each fixed part around every bolt, covering at least 75% of the ring "
        "from the hole out to the head diameter (stand-off at most 0.05 mm)" + (": on a flat face a flat face, on the "
        "pipe a face following its round surface." if has_pipe else "."),
        "- `NO_INTERFERENCE` -- your part does not overlap any fixed part (limit 0.01 mm3), and each bolt has "
        "room: its shank, and its head with 0.5 mm clearance around and 1 mm above.",
    ]
    checks += required_lines(task)
    tol = task.tolerances
    if task.level >= 3 and "tool_length" in tol:
        checks.append(f"- `TOOL_ACCESS` -- above every bolt head, along its axis, a cylinder of the head diameter"
                      f"{' + %g mm' % tol['tool_diameter_extra'] if tol['tool_diameter_extra'] else ''} and "
                      f"{tol['tool_length']:g} mm long is free of your part and the fixed parts: a straight hex key "
                      "can reach the bolt.")
    if "MANUFACTURING" in req:
        checks.append("- `MANUFACTURING` -- the part can be made as stated above, as it is, with no further work. "
                      "The report shows this as `FDM_...` lines.")
    if task.stiffness:
        st = task.stiffness
        held = st.get("held", "A")
        other = next((n for n in task.parts if n != held), "B")
        centre = lambda n: (f"the centre of {n}'s bolts" if task.parts[n].kind == "pipe"  # noqa: E731
                            else f"the centre of {n}'s mounting face")
        checks.append("- `STIFFNESS` -- " + " ".join(m["statement"] for m in st["per_mount"])
                      + f" How it is analysed: {held} is held rigid where your part touches it (its seat and bolt "
                      f"holes); {other}'s load enters your part through {other}'s seat and bolt holes, as through a "
                      f"rigid washer. The span is measured from {centre(held)} to {centre(other)}. The checker resolves +-{st['precision']:g}%; a result well clear of the "
                      "limit may be settled at a first, coarser analysis (+-15%), and the report says which. A "
                      "result inside the band counts as a pass and is flagged.")
        checks.append(brief_line(task))
    return f"""# Design the connecting part

Task {task.id}. Units: millimetres.

{SITUATION.replace(chr(10), ' ')}

Design **one solid part** that bolts to both fixed parts.{made}

**Make the best part you can.** Beyond the requirements below, how the part
looks and how it is shaped are your judgement as the designer. Aim for the
part an experienced mechanical designer would be proud of.

Save it as `output.step` in this folder. `submission.py` is a build123d
starting point; you may replace it entirely. Run it with the Python your
environment provides (it has build123d), then check the part with the
`{NAME}` command:

    python submission.py                                     # writes output.step
    {NAME} verify . --by <your name> --tool <the CAD tool you used>

From a checkout of the kit, put `uv run --project <the kit folder>` before
both commands.

## The fixed parts

{_parts(task)}

## The bolts

Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped
hole in the fixed part and is put in from your side, so its head bears on
your part. The axis points from the fixed part into your part.

| bolt | size | position on the surface | axis | clearance hole dia | grip |
|---|---|---|---|---|---|
{chr(10).join(rows)}

*Grip* is the thickness of your part under the head: the distance along
the axis from the fixed part's surface to where the hole ends.

## What is checked

Your part passes when it meets every requirement below.

{chr(10).join(checks)}

The checker may print further lines (section, volume). They are information, not requirements.

This folder may hold `_solution/` with the generator's reference answer, for
practice. A folder handed out for a recorded run never contains it.
"""
