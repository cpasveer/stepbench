"""The level 4a (assembles) brief: what the model is told. Requirements and file formats,
never the answer (not how to split, where the joint goes, or which way the
parts come in)."""
from __future__ import annotations

from ..catalogue import BOLTS, NUTS, lengths, nut_side, plastic_allowables
from . import joint as J
from .joint import MAX_SEGMENTS, MAX_TURN, PLANE_TOL
from .sweep import TURN_STEP
from ..checks.seat_inside import DEFAULT_SETBACK
from ..material_score import brief_line
from ..workspace import PROCESS_SENTENCE, required_lines



def _v(p):
    return "(" + ", ".join(f"{x:.2f}" for x in p) + ")"


def brief(task, *, title: str | None = None, situation: str | None = None, fixed: str | None = None,
          box: str | None = None) -> str:
    """The brief. A situation other than level 4a's own (a pipe, say) passes its own ``title``,
    ``situation``, ``fixed`` (the fixed parts' section) and ``box`` (the box's section); the rest
    is shared. Where the task requires the joint bolts' length to be stated, the brief asks for it."""
    if situation is None:
        from .generate import SITUATION as situation

    faces = {f.part: f for f in task.faces(0)}
    parts = []
    for name, itf in (task.parts.items() if fixed is None else ()):
        f = faces[name]
        parts.append(f"**{name}** -- {itf.description}.\nMounting face: a {itf.face_w:g} x {itf.face_h:g} mm flat "
                     f"face through {_v(f.origin)}, outward normal {_v(f.normal)}. The part's body lies behind the "
                     f"face; the space in front of it is yours.\nFixed geometry: `fixed_{name}.step` (world coordinates).")
    obs = [f"| {o.name} | {_v(o.to_json()['center'])} | " + " x ".join(f"{v:g}" for v in o.size) + " |"
           for o in (task.obstacles_at(0) if box is None else ())]
    rows = [f"| {b.id} | {b.size} | {_v(b.position)} | {_v(b.axis)} | {BOLTS[b.size].clear_fine:g}-"
            f"{BOLTS[b.size].clear_coarse:g} | {BOLTS[b.size].grip_min:g}-{BOLTS[b.size].grip_max:g} |"
            for b in task.bolts(0)]
    allowed = task.tolerances.get("joint") or {}
    sizes = list(allowed.get("sizes") or NUTS)
    most = allowed.get("max_bolts")
    cat = [f"| {s} | {NUTS[s].pitch:g} | {BOLTS[s].clear_fine:g}-{BOLTS[s].clear_coarse:g} | {BOLTS[s].head_d:g} x {BOLTS[s].head_h:g} | "
           f"{NUTS[s].s:g} / {NUTS[s].e:g} x {NUTS[s].m:g} | {NUTS[s].dw:g} | {BOLTS[s].grip_min:g}-{BOLTS[s].grip_max:g} |"
           for s in sizes]
    al = plastic_allowables()
    st = task.stiffness
    t = task.tolerances
    op = (task.meta or {}).get("opening") or {"wall_x": 0.0, "y": [0.0, 0.0], "z": [0.0, 0.0]}
    if box is None:
        box = f"""## The box -- keep clear

The box round the space between the fixed parts: its walls, floor and
ceiling are fixed boxes, listed below, each centred on its centre with its
edges along the world axes (geometry: `obstacles.step`). Floor and ceiling
leave open exactly the fixed parts' mounting faces. The one opening is in
the end wall at x = {op['wall_x']:.2f}: y from {op['y'][0]:.2f} to {op['y'][1]:.2f}, z from {op['z'][0]:.2f} to {op['z'][1]:.2f}.

| box | centre | size x, y, z |
|---|---|---|
{chr(10).join(obs)}
"""
    if fixed is None:
        fixed = chr(10).join(p + chr(10) for p in parts)
    # the joint bolts' length: asked for where the task requires it
    stated = bool(allowed.get("length_required"))
    length_json = '\n    "length": <mm, one of the catalogue lengths below>,' if stated else ""
    length_note = ("\n  `length`: the bolts' length, one of the catalogue lengths below; each must stand out of"
                   "\n  its nut by one to two thread pitches.") if stated else ""
    length_cat = ("\n\nCatalogue lengths (ISO 4762), mm: "
                  + "; ".join(f"{s_} {', '.join(f'{x:g}' for x in lengths(s_))}" for s_ in sizes)
                  + ". A bolt passes through both parts' grips and its nut, and stands out of the nut by one"
                    " to two thread pitches.") if stated else ""
    length_check = ("\n- `JOINT_LENGTH` -- every joint bolt, at the length you state, is a catalogue length"
                    "\n  and stands out of its nut by one to two thread pitches: the length less both parts'"
                    "\n  thickness along the bolt and the nut's height.") if stated else ""
    req = [l.replace("inside that face's outline", "inside that face's outline (on the fixed parts' faces; "
                     "the joint face is not judged)") for l in required_lines(task)]
    m0 = st["per_mount"][0]
    load = (f"{m0['mount']}'s {m0['mass_kg']:g} kg ({m0['design_load_N']:g} N), acting at the centre of "
            f"{m0['mount']}'s mounting face")
    score_how = brief_line(task).split("material wins: ", 1)[1].split(", each part as printed", 1)[0].split(". Stiffness", 1)[0]
    return f"""# {title or 'Level 4a, assembles: design the connection, in two parts'}

Units: millimetres.

{situation}

Design **two solid parts** that together connect the fixed parts: one bolts
to A, the other to B, and the two bolt to each other with a joint of your
own design. {PROCESS_SENTENCE['fdm'].replace('It will be', 'Each part will be')}

You also decide how they are put together: each part comes in from outside
the whole arrangement along a straight line, then, if you wish, along a path
of straight moves and turns, one part after the other, and must not run into
anything on the way.

**Make the best connection you can.** Beyond the requirements below, how the
parts look and how they are shaped are your judgement as the designer. Aim
for the design an experienced mechanical designer would be proud of.

## What you hand in

Three files in this folder:

- `part1.step`, `part2.step` -- one solid each, in world coordinates, in
  their final, bolted place.
- `assembly.json` -- how they go together:

```json
{{
  "parts": [
    {{"name": "part1", "file": "part1.step", "bolts_to": "<A or B>", "insert_direction": [dx, dy, dz]}},
    {{"name": "part2", "file": "part2.step", "bolts_to": "<the other>", "insert_direction": [dx, dy, dz],
      "path": [{{"move": [dx, dy, dz]}},
               {{"turn": {{"point": [x, y, z], "axis": [ax, ay, az], "degrees": a}}}}]}}
  ],
  "joint": {{
    "bolt": "<a size from the catalogue below>",{length_json}
    "head_on": "<part1 or part2>",
    "axis": [ax, ay, az],
    "positions": [[x, y, z], [x, y, z], ...]
  }}
}}
```

- Parts are put in place **in the order listed**. `insert_direction` is the
  way the part moves as it goes in: it starts beyond everything else and
  travels straight along that direction to its place -- or, if the part has
  a `path`, to where its path begins. `path` (optional): the moves and turns
  that then take it into its place, in order, in world coordinates; a `move`
  is a straight shift, a `turn` a rotation by `degrees` (right-hand rule
  about `axis`) about the line through `point` along `axis`, that line fixed
  in space. The part's place is where its path ends: the STEP file shows it
  there, and the start is found by undoing the path. A path may tilt the
  part on its way in (a turn), as you would tilt a part to get it through an
  opening. A part may also give a `path` and no `insert_direction`: then its
  path starts wholly outside the box. A path has at most {MAX_SEGMENTS} steps; a turn is
  at most {MAX_TURN:g} degrees. The part listed first
  is then bolted to its fixed part, so its bolt heads are in place when the
  second part comes in. The joint bolts go in last.
- The joint: through bolts of one allowed catalogue size (below), at least
  2 of them, each with its head bearing on the part named in `head_on`
  and an ISO 4032 hex nut on the other part. `positions`: where each bolt
  crosses the face where the two parts meet, all on that one face;
  all within {PLANE_TOL:g} mm of one plane, and at least a head diameter apart;
  `axis`: square to that face, pointing from the nut's part into the head's
  part (as at a fixed part, the axis points into the part the head bears on).
  The joint bolts are called J1, J2, ... in the order you list them.{length_note}
- Shear keys are allowed: features of one part that sit in a matching
  recess of the other close to the joint face (a flat flank of each, square
  to the joint face, within {J.KEY_DEPTH:g} mm of it, facing each other at most
  {J.KEY_GAP:g} mm apart). The two parts may touch there too.

`submission.py` is a build123d starting point that writes all three files;
you may replace it entirely.

Run it with the Python your environment provides (it has build123d), then
check the answer with the `jointbench` command:

    python submission.py                     # writes part1.step, part2.step, assembly.json
    jointbench verify . --by <your name> --tool <the CAD tool you used>

From a checkout of the kit, put `uv run --project <the kit folder>` before
both commands.

## The fixed parts

{fixed}
{box}
## The bolts into the fixed parts

Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped
hole in the fixed part and is put in from your side, so its head bears on
your part. The axis points from the fixed part into your part.

| bolt | size | position on the surface | axis | clearance hole dia | grip |
|---|---|---|---|---|---|
{chr(10).join(rows)}

*Grip* is the thickness of your part under the head: the distance along
the axis from the fixed part's surface to where the hole ends.

## The joint bolts: catalogue

Allowed in this task: {', '.join(sizes)}{f", at most {most} bolts" if most else ""}. ISO 4762 socket
head cap screw through both parts, ISO 4032 hex nut.

| size | thread pitch | clearance hole dia | head dia x height | nut across flats / corners x height | nut bearing face dia | grip, each part |
|---|---|---|---|---|---|---|
{chr(10).join(cat)}

Grip, on each side, is the thickness of that part from the joint face to
where its hole ends (under the head on one part, under the nut on the other).{length_cat}

In a printed part the bolt never fails; the plastic round it does. The
plastic is judged as {al['material']} across its layers: {al['strength_MPa']:g} MPa, over a
safety factor of {al['safety']:g}: bearing (plastic pressed) at most {al['bearing_MPa']:.3g} MPa, shear
(plastic torn or punched) at most {al['shear_MPa']:.3g} MPa.

## What is checked

Your answer passes when it meets every requirement below.

**Each part on its own** (named in the checker's report, e.g. `part1 HOLES`).
At the joint, each part sees the other part as one more fixed part: a face
where they meet, with the joint bolts in it.

- `FILE_VALID` -- each file holds exactly one valid, closed solid.
- `HOLES` -- a through hole on every bolt axis of that part, fixed part's and
  joint's, within {t['hole_axis_offset_max']:g} mm, diameter in the clearance range, grip in range.
- `SEATING` -- the part lies on the face it bolts to around every bolt,
  covering at least {t['seat_contact_min']:.0%} of the ring from the hole out to the head (or nut)
  diameter (stand-off at most {t['seat_gap_max']:g} mm): on its fixed part, and on the other part at the joint;
  on a flat face a flat face, on a pipe a face following its round surface.
- `NO_INTERFERENCE` -- the part does not overlap its fixed part or the other
  part (limit {t['overlap_volume_max']:g} mm3), and every bolt has room: its shank, and its head (or
  nut) with {t['head_clearance_radial']:g} mm clearance around and {t['head_clearance_axial']:g} mm above. A nut's room
  is its height plus two thread pitches of bolt end standing out of it, then the {t['head_clearance_axial']:g} mm.
- `CLEAR_PATH` -- the part does not touch the box or the fixed part it
  does not bolt to.
- `TOOL_ACCESS` -- above every bolt head, along its axis, a cylinder of the
  head diameter and {t['tool_length']:g} mm long is free of both parts, the fixed parts
  and the box: a straight hex key can reach. (A nut is held with a spanner
  from the side: it needs only its room, above.)
{chr(10).join(req)}
- `MANUFACTURING` -- the part can be made as stated above, as it is, with no further work. The
  report shows this as `FDM_...` lines.

**The assembly**

- `ASSEMBLY_FILE` -- `assembly.json` is complete: two parts, one bolting to
  A and one to B, a joint of at least 2 catalogue bolts on one face.
- `INSERTION` -- each part, in the order listed, travels in along its
  `insert_direction` from outside everything and then along its `path`
  without running into a fixed part, the box, the part already in place or
  that part's bolt heads (touching while sliding past is fine; limit
  {t['overlap_volume_max']:g} mm3). Straight stretches are checked exactly; a turn at steps at which no
  point of the part travels more than {TURN_STEP:g} mm.
- `JOINT_ROOM` -- each joint bolt's head and nut room is clear of the fixed
  parts and the box, and the two parts touch each other only at the joint
  (its face and its shear keys). Every joint bolt head must also be reachable
  once all is assembled: the part it bears on is judged by `TOOL_ACCESS` above.
- `JOINT_REACH` -- a key reaches every joint bolt from outside: along each
  head's axis, from the top of the head out past everything, a cylinder of
  the head's diameter is free of both parts, the fixed parts and the box.{length_check}
- `JOINT_BOLTS` -- under the load: {load}, pulling in whichever direction is
  worst, the plastic round every joint bolt holds, on both parts. The load and its
  turning effect pass through the joint; they are shared among the bolts as
  a bolt group is designed: a pull along the axis equally; a turning load
  that would open the joint tips it about the edge of the contact face on
  the side it presses, each bolt taking a share in proportion to its
  distance from that edge; a slide across equally, a twist about the axis
  in proportion to each bolt's distance from the bolts' centre. Shear keys
  take the slide in the directions their flanks face (all of it once flanks
  face two different ways), and the twist once a flank reaches {J.KEY_ARM:g} mm or
  more from the bolts' centre along its length; the bolts take the rest.
  For each bolt, on each part, from the part's own shape: **pull-through**
  -- the pull over the ring under the head (or nut) at most the bearing
  stress, and the pull punching a plug of the head's (nut's bearing face's)
  diameter through the thickness under it at most the shear stress;
  **bearing** -- the slide over shank diameter x that part's thickness at most
  the bearing stress; **tear-out** -- the slide over the plastic between the
  hole and the part's free edge, in the direction the bolt pushes that part
  (two planes, each thickness x (edge distance - hole radius)), at most the
  shear stress.
- `STIFFNESS` -- the two parts analysed together, as one body joined over
  the contact at the joint face: {' '.join(m['statement'] for m in st['per_mount'])} How it is
  analysed: {st.get('held', 'A')} is held rigid where your part touches it (its seat and bolt holes);
  the load enters through the loaded fixed part's seat and bolt holes, as through a rigid washer; the
  span runs from the centre of {st.get('held', 'A')}'s mounting (on a pipe, the centre of its bolts) to the
  centre of the loaded part's mounting face. The checker resolves +-{st['precision']:g}%; a result well
  clear of the limit may be settled at a first, coarser analysis (+-15%), and the report says which. A
  result inside the band counts as a pass and is flagged.

**Score** -- among answers that meet every requirement above, the one that
uses the least material wins: {score_how}, for both parts together, each in the print direction the checker finds for it.
Stiffness beyond the requirement earns nothing.

The checker may print further lines (volume). They are information, not requirements.
"""


SUBMISSION = '''"""Your answer to this task: two parts and how they go together.

Writes part1.step, part2.step and assembly.json next to this file. Read
README.md for the brief; every number is in task.json. Only those three
files are graded; you may replace this script entirely, or model the parts
in any CAD program.
"""
import json
import pathlib

from build123d import Box, Pos, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())


def build():
    # Placeholders: two 10 mm cubes. They will fail. Replace them with your parts.
    part1 = Pos(0, 0, 5) * Box(10, 10, 10)
    part2 = Pos(0, 0, 30) * Box(10, 10, 10)
    assembly = {
        "parts": [
            {"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": [0, 0, -1]},
            {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": [0, 0, 1]},
        ],
        "joint": {"bolt": "M4", "head_on": "part1", "axis": [0, 0, -1], "positions": [[0, -10, 20], [0, 10, 20]]},
    }
    return part1, part2, assembly


if __name__ == "__main__":
    p1, p2, asm = build()
    export_step(p1, str(HERE / "part1.step"))
    export_step(p2, str(HERE / "part2.step"))
    (HERE / "assembly.json").write_text(json.dumps(asm, indent=2) + "\\n")
    print("wrote part1.step, part2.step, assembly.json")
'''
