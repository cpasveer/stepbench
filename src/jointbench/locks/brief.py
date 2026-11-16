"""The brief (README.md) of a "locks" task: every requirement and file format, never the answer."""
from __future__ import annotations

from .. import NAME
from ..catalogue import BOLTS
from ..task import Task
from ..workspace import process_sentence, required_lines
from ..catalogue import plastic_allowables
from ..material_score import brief_line
from .generate import FILES, SITUATION
from .grade import CONTACT_DEPTH


def _v(v, n=2):
    return "(" + ", ".join(f"{x:.{n}f}" for x in v) + ")"


def brief(task: Task) -> str:
    t, st, lk = task.tolerances, task.tolerances["locks"], task.meta["lock"]
    faces = {f.part: f for f in task.faces(0)}
    th = lk["turn_deg"]
    al = plastic_allowables()
    L = [
        f"# Task {task.id} -- make it lock",
        "",
        f"Level {task.meta.get('level_name', task.level)}. Units: millimetres and degrees. World coordinates throughout.",
        "",
        SITUATION,
        "",
        "Design **two rigid parts**: one bolted to A, one bolted to B. Any design that works passes; nothing is",
        "compared to a reference.",
        "",
        "## The motion",
        "",
        f"- Axis: the line through {_v(lk['axis_point'])} along {_v(lk['axis_direction'])}.",
        f"- B comes in from far away straight along the axis, travelling in the direction {_v(lk['insert_direction'])}, "
        "to the place `task.json` and `fixed_B.step` show: its *start pose*. Model both parts there.",
        f"- Then B turns {th:g} degrees about the axis, right-handed about {_v(lk['axis_direction'])} (thumb along it); "
        f"{'a negative angle turns the other way. ' if th < 0 else ''}`fixed_B_turned.step` shows B turned.",
        "- Turned, B must be locked: it cannot be pulled back out along the axis. Not yet turned, it must still come",
        "  straight out.",
        f"- Once locked, B carries a pull-out load of {lk['pull_load_kg']:g} kg ({lk['pull_load_N']:g} N), straight back "
        "along the axis (opposite the insertion direction). The lock must carry it in the printed plastic.",
        "",
        "## The fixtures",
        "",
    ]
    for name, itf in task.parts.items():
        f = faces[name]
        L += [f"**{name}** -- {itf.description}.",
              f"Mounting face: a {itf.face_w:g} x {itf.face_h:g} mm flat face through {_v(f.origin)}, outward normal "
              f"{_v(f.normal)} (B at its start pose). The body lies behind the face. Geometry: `fixed_{name}.step`.", ""]
    L += [
        "Bolts are ISO 4762 socket head cap screws. Each threads into a tapped hole in its fixture and is put in",
        "from your side, so its head bears on your part. The axis points from the fixture into your part.",
        "",
        "| bolt | size | position on face | axis | clearance hole dia | grip |",
        "|---|---|---|---|---|---|",
    ]
    for b in task.bolts(0):
        bt = BOLTS[b.size]
        L.append(f"| {b.id} | {b.size} | {_v(b.position)} | {_v(b.axis)} | {bt.clear_fine:g}-{bt.clear_coarse:g} | "
                 f"{bt.grip_min:g}-{bt.grip_max:g} |")
    L += [
        "",
        "*Grip* is the thickness of your part under the head: the distance along the axis from the mounting face to",
        "where the hole ends.",
        "",
        "## What you hand in",
        "",
        f"`{FILES['A']}` (bolted to A, with every bolt of A) and `{FILES['B']}` (bolted to B, with every bolt of B), "
        "each a STEP file holding one solid, in world coordinates with B at its start pose. Parts are rigid: nothing "
        "may rely on bending (no snap fits, no flexures). Each part is bolted to its fixture before B comes in.",
        "",
        "## What is checked",
        "",
        "Your answer passes when every check below passes.",
        "",
        "**Each part, against its own fixture:**",
        "",
        "- `FILE_VALID` -- the file holds exactly one valid, closed solid.",
        f"- `HOLES` -- a through hole on every bolt axis, within {t['hole_axis_offset_max']:g} mm, diameter in the "
        "clearance range, grip in range.",
        f"- `SEATING` -- a flat face lies on the mounting face (stand-off at most {t['seat_gap_max']:g} mm) and covers at "
        f"least {t['seat_contact_min']:.0%} of the ring around each bolt, from the hole out to the head diameter.",
        f"- `NO_INTERFERENCE` -- the part does not overlap its fixture (limit {t['overlap_volume_max']:g} mm3), and each "
        f"bolt has room: its shank, and its head with {t['head_clearance_radial']:g} mm clearance around and "
        f"{t['head_clearance_axial']:g} mm above.",
        f"- `TOOL_ACCESS` -- every bolt can be turned with a straight tool: above each head, along the bolt axis, a "
        f"cylinder of the head diameter + {t['tool_diameter_extra']:g} mm and {t['tool_length']:g} mm long, starting at "
        "the top of the head, must be free of the part and its fixture.",
    ]
    L += required_lines(task)
    ps = process_sentence(task)
    if ps:
        L.append("- `MANUFACTURING` -- each part is made on its own. " + ps)
    L += [
        "",
        "**The two together.** A's side is A, its part and its bolt heads; B's side is B, its part and its bolt heads.",
        "",
        f"- `SEATED` -- at the start pose the two sides do not overlap (limit {t['overlap_volume_max']:g} mm3).",
        "- `CONNECT` -- B's side, brought in from outside along the insertion direction to the start pose, sweeps "
        "through nothing of A's side (the exact swept volume).",
        f"- `TURN` -- turned from 0 to {th:g} degrees in equal steps of at most {st['step_max_deg']:g} degrees, small "
        f"enough that no point of B's side moves more than {st['step_move_max']:g} mm per step, B's side never overlaps "
        "A's side.",
        f"- `GAP` -- at every step of the turn the two sides stay at least {st['gap_min']:g} mm apart, so printed "
        "surfaces that move past each other do not rub or fuse.",
        f"- `LOCKED` -- turned, B's side pulled back along the axis (opposite the insertion direction) meets A's side "
        f"within {st['play_max']:g} mm.",
        f"- `UNLOCKS` -- not yet turned, B's side pulled back the same way {st['loose_shift']:g} mm meets nothing: only "
        "the turn locks it.",
        f"- `INTERLOCK` -- turned, B's side cannot come loose: shifted {st['loose_shift']:g} mm in any of 26 directions "
        f"(along the axis, square to it, and every diagonal between), or tilted {st['loose_tilt']:g} degrees about either "
        "direction square to the axis through the axis point, it must meet A's side on the way.",
        f"- `LOCK_STRENGTH` -- turned and pulled back with the stated load, the plastic where the two parts bear on "
        f"each other must carry it. The grader pulls part_B back {CONTACT_DEPTH:g} mm past where it first meets part_A; "
        "where the two then overlap is each contact patch. (1) Bearing: the load over the patches' area, projected "
        f"square to the pull, at most {al['bearing_MPa']:g} MPa. (2) Shear: on each part, each patch's share of the load "
        "punching a plug of the patch's area out through the plastic straight behind it along the pull, the plug's "
        f"outline taken as round, at most {al['shear_MPa']:g} MPa. Allowables: {al['source']}.",
        f"- `PLAY` -- turned, B's side moves freely at most {st['play_max']:g} mm along the axis and either way square to "
        f"it, and tilts freely at most {st['tilt_max_deg']:g} degrees about either direction square to the axis (the one "
        "nearest world +Z made square to the axis -- here world +X -- and the axis crossed with that one).",
        "",
        "Measured on exact geometry: distances and overlap volumes between the solids. The same files always give the "
        "same verdict.",
        "",
        brief_line(task).replace("parts that meet", "answers that meet").replace("uses the least material",
                                                                                  "uses the least material, both parts together"),
        "",
        "## Grading your answer",
        "",
        "```",
        f"python -m {NAME}.locks verify <this folder>",
        "```",
        "",
        "`_solution/` holds a reference answer. If you are the one being tested, do not open it.",
        "",
    ]
    return "\n".join(L)
