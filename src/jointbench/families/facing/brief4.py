"""The facing setup's level 4 (assembles) brief: level 4a's, with the facing
setup's situation, fixed parts and box. Requirements and file formats, never
the answer (not how to split, where the joint goes, or which way the parts
come in)."""
from __future__ import annotations

TITLE = "Level 4, assembles: join the two facing mounts, in two parts"

SITUATION = """Two fixed parts, one above the other. Their mounting faces are horizontal and
face each other, with nothing in between, offset a little sideways. A closed
box stands round both of them, and one small opening in one of the box's side
walls (the front wall) is the only way in. The connection has to be built
inside the box, from parts that each fit through that opening."""


def _v(p):
    return "(" + ", ".join(f"{x:.2f}" for x in p) + ")"


def fixed_parts(task) -> str:
    faces = {f.part: f for f in task.faces(0)}
    out = []
    for name, itf in task.parts.items():
        f = faces[name]
        if itf.kind == "round":
            shape = f"a round flat face {itf.face_w:g} mm across"
        else:
            shape = f"a {itf.face_w:g} x {itf.face_h:g} mm flat face"
        boss = "".join(f" A round spigot {2 * p.radius:g} mm across stands {p.height:g} mm proud of the face; "
                       f"your part must clear it." for p in itf.protrusions)
        out.append(f"**{name}** -- {itf.description}.\nMounting face: {shape} through {_v(f.origin)}, outward normal "
                   f"{_v(f.normal)}. The part's body lies behind the face; the space in front of it is yours.{boss}\n"
                   f"Fixed geometry: `fixed_{name}.step` (world coordinates).")
    return "\n".join(p + "\n" for p in out)


def box_section(task) -> str:
    rows = []
    for o in task.obstacles_at(0):
        j = o.to_json()
        ax = j.get("axes") or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        rows.append(f"| {o.name} | {_v(j['center'])} | " + " x ".join(f"{v:.2f}" for v in o.size)
                    + f" | {_v(ax[0])} | {_v(ax[1])} |")
    op = (task.meta or {}).get("opening") or {}
    return f"""## The box -- keep clear

The box is closed round both fixed parts: six walls, each a slab listed below
by its centre, its size along its own x, y and z (thickness) directions, and
those x and y directions in the world (geometry: `obstacles.step`). Where
four walls meet a side, they overlap at the edges.

| wall | centre | size x, y, z | x direction | y direction |
|---|---|---|---|---|
{chr(10).join(rows)}

**The one opening** is in the front wall, whose inside face lies
{op.get('plane', 0.0):.2f} mm from the world origin along its outward normal {_v(op.get('normal', (0, 0, 0)))}. Measured
from the world origin along {_v(op.get('along', (0, 0, 0)))} it spans {op.get('along_range', [0, 0])[0]:.2f} to
{op.get('along_range', [0, 0])[1]:.2f} mm, and along {_v(op.get('up', (0, 0, 0)))} from {op.get('up_range', [0, 0])[0]:.2f} to
{op.get('up_range', [0, 0])[1]:.2f} mm. Everything you put in comes through it.
"""


def brief(task) -> str:
    from ...assemble.brief import brief as shared

    return shared(task, title=TITLE, situation=SITUATION, fixed=fixed_parts(task), box=box_section(task))
