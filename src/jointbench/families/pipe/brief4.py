"""The pipe's level 4 (assembles) brief: level 4a's, with the pipe's own
situation, fixed parts and box. Requirements and file formats, never the
answer (not how to split, where the joint goes, or which way the parts come
in)."""
from __future__ import annotations

import numpy as np

TITLE = "Level 4, assembles: clamp the pipe, in two parts"

SITUATION = """Two fixed parts: a pipe, and a part with a flat mounting face. A closed box
stands round both of them: the pipe runs in through the box's bottom wall and
out through its top wall, and one small opening in the box's front wall is
the only way in. The connection has to be built inside the box, from parts
that each fit through that opening."""


def _v(p):
    return "(" + ", ".join(f"{x:.2f}" for x in p) + ")"


def brief(task) -> str:
    from ...assemble.brief import brief as shared

    fixed, box = sections(task)
    return shared(task, title=TITLE, situation=SITUATION, fixed=fixed, box=box)


def sections(task) -> tuple[str, str]:
    """(the fixed parts' section, the box's section): the pipe's, shared by levels 4 and 5."""
    faces = {f.part: f for f in task.faces(0)}
    fixed = []
    for name, itf in task.parts.items():
        if itf.kind == "pipe":
            pose = task.poses[0][name]
            xs = [p[0] for p, _ in itf.hole_frames]
            mid = (max(xs) + min(xs)) / 2
            ends = [np.asarray(pose.point((mid + k * itf.face_w / 2, 0.0, 0.0))) for k in (-1, 1)]
            fixed.append(f"**{name}** -- {itf.description}.\nIts axis runs from {_v(ends[0])} to {_v(ends[1])} "
                         f"({itf.face_w:g} mm long, {itf.face_h:g} mm outside diameter). Your part bears on "
                         f"its outer surface round each bolt.\nFixed geometry: `fixed_{name}.step` (world "
                         f"coordinates).")
        else:
            f = faces[name]
            fixed.append(f"**{name}** -- {itf.description}.\nMounting face: a {itf.face_w:g} x {itf.face_h:g} mm "
                         f"flat face through {_v(f.origin)}, outward normal {_v(f.normal)}. The part's body lies "
                         f"behind the face; the space in front of it is yours.\nFixed geometry: `fixed_{name}.step` "
                         f"(world coordinates).")
    pipe_r = next(itf.face_h / 2 for itf in task.parts.values() if itf.kind == "pipe")
    rows, holes = [], []
    for o in task.obstacles_at(0):
        j = o.to_json()
        ax = j.get("axes") or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        rows.append(f"| {o.name} | {_v(j['center'])} | " + " x ".join(f"{v:.2f}" for v in o.size)
                    + f" | {_v(ax[0])} | {_v(ax[1])} |")
        for h in j.get("holes") or []:
            holes.append(f"- `{o.name}`: a round hole {2 * h['radius']:.2f} mm across on the line through "
                         f"{_v(h['center'])} along {_v(h['axis'])}: the pipe passes it with "
                         f"{h['radius'] - pipe_r:.2f} mm all round.")
    op = (task.meta or {}).get("opening") or {}
    box = f"""## The box -- keep clear

The box is closed round both fixed parts: six walls, each a slab listed below
by its centre, its size along its own x, y and z (thickness) directions, and
those x and y directions in the world (geometry: `obstacles.step`). Where
four walls meet a side, they overlap at the edges.

| wall | centre | size x, y, z | x direction | y direction |
|---|---|---|---|---|
{chr(10).join(rows)}

The pipe passes through the top and bottom walls:
{chr(10).join(holes)}

**The one opening** is in the front wall, whose inside face lies
{op.get('plane', 0.0):.2f} mm from the world origin along its outward normal {_v(op.get('normal', (0, 0, 0)))}. Measured
from the world origin along {_v(op.get('along', (0, 0, 0)))} it spans {op.get('along_range', [0, 0])[0]:.2f} to
{op.get('along_range', [0, 0])[1]:.2f} mm, and along {_v(op.get('up', (0, 0, 0)))} from {op.get('up_range', [0, 0])[0]:.2f} to
{op.get('up_range', [0, 0])[1]:.2f} mm. Everything you put in comes through it.
"""
    return chr(10).join(p + chr(10) for p in fixed), box
