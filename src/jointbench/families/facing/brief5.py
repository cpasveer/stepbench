"""The facing setup's level 5 (locks) brief: level 4's situation, box and ways in, with the
bolted joint between the two parts replaced by a push-and-turn lock (level 4d's
checks). Requirements and file formats, never the answer (not how to split,
where the lock goes, which part turns, or about which axis)."""
from __future__ import annotations

from ...catalogue import BOLTS, plastic_allowables
from ...locks.grade import CONTACT_DEPTH
from ...material_score import brief_line
from ...workspace import PROCESS_SENTENCE, required_lines
from .brief4 import _v, box_section, fixed_parts

TITLE = "Level 5, locks: join the two facing mounts, in two parts that lock together"

SITUATION = """Two fixed parts, one above the other. Their mounting faces are horizontal and
face each other, with nothing in between, offset a little sideways. A closed
box stands round both of them, and one small opening in one of the box's side
walls (the front wall) is the only way in. The connection has to be built
inside the box, from parts that each fit through that opening -- and the two
parts join each other with no bolts and no tool inside the box: one of them is
pushed along an axis and turned about it, and is then locked to the other."""



def brief(task) -> str:
    fixed, box = fixed_parts(task), box_section(task)
    t, st = task.tolerances, task.tolerances["locks"]
    lk = task.meta["lock"]
    al = plastic_allowables()
    rows = [f"| {b.id} | {b.size} | {_v(b.position)} | {_v(b.axis)} | {BOLTS[b.size].clear_fine:g}-"
            f"{BOLTS[b.size].clear_coarse:g} | {BOLTS[b.size].grip_min:g}-{BOLTS[b.size].grip_max:g} |"
            for b in task.bolts(0)]
    score_how = brief_line(task).split("material wins: ", 1)[1].split(", each part as printed", 1)[0].split(". Stiffness", 1)[0]
    return f"""# {TITLE}

Units: millimetres and degrees. World coordinates throughout.

{SITUATION}

Design **two solid parts**: one bolts to A, the other to B. {PROCESS_SENTENCE['fdm'].replace('It will be', 'Each part will be')}
Parts are rigid: nothing may rely on bending (no snap fits, no flexures).

**Make the best connection you can.** Beyond the requirements below, how the
parts look and how they are shaped are your judgement as the designer.

## The sequence

1. The first part listed goes in through the opening along its way in, to
   where it is placed: for the part that moves in the lock, that is where it
   *waits* -- its final place with the lock's turn and then its push undone.
2. The second part goes in along its way in to where it is placed.
3. The moving part is pushed along the lock's axis (`push`), then turned
   about that axis (`turn`): now the two are locked.
4. Both parts are bolted to their fixed parts.

## What you hand in

Three files in this folder:

- `part1.step`, `part2.step` -- one solid each, in world coordinates, in
  their final, locked and bolted place.
- `assembly.json`:

```json
{{
  "parts": [
    {{"name": "part1", "file": "part1.step", "bolts_to": "<A or B>", "insert_direction": [dx, dy, dz],
      "path": [{{"move": [dx, dy, dz]}}, {{"turn": {{"point": [x, y, z], "axis": [ax, ay, az], "degrees": a}}}}]}},
    {{"name": "part2", "file": "part2.step", "bolts_to": "<the other>", "insert_direction": [dx, dy, dz]}}
  ],
  "lock": {{
    "moving": "<part1 or part2>",
    "push": [dx, dy, dz],
    "turn": {{"point": [x, y, z], "axis": [ax, ay, az], "degrees": a}}
  }}
}}
```

- Parts go in **in the order listed**. `insert_direction`: the way the part
  travels as it goes in, straight from beyond everything to where its `path`
  begins (or, with no path, to where it is placed). `path` (optional): moves
  and turns that then take it there (a `move` a straight shift, a `turn` a
  rotation by `degrees`, right-hand rule about `axis`, about the line through
  `point`; all in world coordinates). A path may tilt the part on its way in, as
  you would tilt a part to get it through an opening. A part may give a `path`
  and no `insert_direction`: then its path starts wholly outside the box.
- `lock`: which part moves, its push (a straight shift along the turn's axis,
  within {1:g} degree) and its turn (about that axis). The moving part's file
  shows it after both.

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

*Grip* is the thickness of your part under the head: the distance along the
axis from the fixed part's surface to where the hole ends.

## What is checked

Your answer passes when it meets every requirement below.

**Each part on its own** (named in the checker's report, e.g. `part1 HOLES`),
against the fixed part it bolts to; the box, the other fixed part and the
other part (in place) count as obstacles.

- `FILE_VALID` -- each file holds exactly one valid, closed solid.
- `HOLES` -- a through hole on every bolt axis, within {t['hole_axis_offset_max']:g} mm, diameter in the
  clearance range, grip in range.
- `SEATING` -- the part lies on the face it bolts to around every bolt,
  covering at least {t['seat_contact_min']:.0%} of the ring from the hole out to the head diameter
  (stand-off at most {t['seat_gap_max']:g} mm).
- `NO_INTERFERENCE` -- the part does not overlap its fixed part (limit {t['overlap_volume_max']:g} mm3),
  and every bolt has room: its shank, and its head with {t['head_clearance_radial']:g} mm clearance around
  and {t['head_clearance_axial']:g} mm above.
- `CLEAR_PATH` -- the part does not touch the box, the other fixed part or the other part.
- `TOOL_ACCESS` -- above every bolt head, along its axis, a cylinder of the head diameter and
  {t['tool_length']:g} mm long is free of everything: a straight hex key can reach.
{chr(10).join(required_lines(task))}
- `MANUFACTURING` -- the part can be made as stated above, as it is, with no further work. The
  report shows this as `FDM_...` lines.

**The assembly**

- `ASSEMBLY_FILE` -- `assembly.json` is complete: two parts, one bolting to each fixed part, a lock
  whose push runs along its turn's axis.
- `INSERTION` -- each part, in order, travels in along its `insert_direction` and then its `path`
  without running into a fixed part, the box or the part already there (touching while sliding
  past is fine; limit {t['overlap_volume_max']:g} mm3). Straight stretches exactly; turns in small steps.

**The lock.** The moving part against everything else: the other part, both fixed parts and the
box. The moving part may slide on the fixed part it bolts to as it is pushed and turned; that
contact is left out of `GAP`, `LOCKED` and `UNLOCKS`.

- `SEATED` -- pushed, not yet turned, the moving part overlaps nothing.
- `PUSH` -- the push, from where it waits, runs into nothing.
- `TURN` -- turned in small steps (at most {st['step_max_deg']:g} degrees, and at most {st['step_move_max']:g} mm of travel of
  its farthest point per step), the moving part never overlaps anything.
- `GAP` -- at every step of the turn it stays at least {st['gap_min']:g} mm from the other part
  and the box, so printed surfaces that move past each other do not rub or fuse.
- `LOCKED` -- turned, pulled back against its push, it meets the other part within {st['play_max']:g} mm.
- `UNLOCKS` -- pushed but not yet turned, pulled back {st['loose_shift']:g} mm, it meets nothing: only the
  turn locks it.
- `INTERLOCK` -- turned, it cannot come loose: shifted {st['loose_shift']:g} mm in any of 26 directions,
  or tilted {st['loose_tilt']:g} degrees, it meets something on the way.
- `PLAY` -- turned, it moves freely at most {st['play_max']:g} mm along the axis and either way square to it,
  and tilts at most {st['tilt_max_deg']:g} degrees.
- `LOCK_STRENGTH` -- turned and pulled back with {lk['pull_load_kg']:g} kg ({lk['pull_load_N']:g} N), the plastic where the
  two parts bear on each other carries it. The checker pulls the moving part {CONTACT_DEPTH:g} mm past where it first
  meets the other; where the two then overlap is each contact patch. Bearing: the load over the
  patches' area, projected square to the pull, at most {al['bearing_MPa']:.3g} MPa. Shear: on each part, each
  patch's share punching a plug of the patch's area through the plastic straight behind it, its
  outline taken as round, at most {al['shear_MPa']:.3g} MPa ({al['material']}, {al['strength_MPa']:g} MPa over a safety factor of {al['safety']:g}).

**Score** -- among answers that meet every requirement above, the one that uses the least material
wins: {score_how}, for both parts together, each in the print direction the checker finds for it.

## Making and checking your answer

`submission.py` is a build123d starting point that writes all three files; you may replace it
entirely. Run it with the Python your environment provides (it has build123d), then check the
answer with the `jointbench` command:

    python submission.py                     # writes part1.step, part2.step, assembly.json
    jointbench verify . --by <your name> --tool <the CAD tool you used>

From a checkout of the kit, put `uv run --project <the kit folder>` before both commands.
"""
