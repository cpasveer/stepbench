"""The brief (README.md) of a "moves" task: every requirement and file format, never the answer."""
from __future__ import annotations

from .. import NAME
from ..catalogue import BOLTS
from ..geometry import Placement
from ..task import Task
from ..workspace import process_sentence, required_lines
from . import assembly as AS


def _v(v, n=2):
    return "(" + ", ".join(f"{x:.{n}f}" for x in v) + ")"


def brief(task: Task) -> str:
    from .generate import SITUATION

    t, st, mo = task.tolerances, task.tolerances["moves"], task.meta["motion"]
    fixed, moving = mo["fixed"], mo["moving"]
    faces = {f.part: f for f in task.faces(0)}
    b0 = task.poses[0][moving]
    L = [
        f"# Task {task.id} -- make it move",
        "",
        f"Level {task.meta.get('level_name', task.level)}. Units: millimetres and degrees. World coordinates throughout.",
        "",
        SITUATION,
        "",
        f"Design a mechanism of rigid parts that carries **{moving}** from where it starts through the poses",
        "below, in order. Any mechanism that does it passes; nothing is compared to a reference.",
        "",
        "## The fixtures",
        "",
    ]
    for name, itf in task.parts.items():
        f = faces.get(name)
        role = "stays put" if name == fixed else "is carried; shown here at its start"
        pl = task.poses[0][name]
        L += [f"**{name}** ({role}) -- {itf.description}."]
        if f is not None:
            L.append(f"Mounting face: a {itf.face_w:g} x {itf.face_h:g} mm flat face through {_v(f.origin)}, "
                     f"outward normal {_v(f.normal)}. The body lies behind the face.")
        elif itf.kind == "pipe":
            L.append(f"A round pipe: its axis through {_v(pl.origin)} along {_v(pl.vec((1, 0, 0)))}, "
                     f"{itf.face_h:g} mm outside diameter, {itf.face_w:g} mm long (centred on its bolts); its bolts "
                     "point straight out of its surface (listed below).")
        L += [f"Geometry: `fixed_{name}.step`.", ""]
    L += [
        f"## Where {moving} must go",
        "",
        f"A placement is the origin of {moving}'s own frame (the centre of its mounting face) and the rotation "
        f"of that frame (row by row: world = rotation x local + origin). {moving}'s local +Z is its face's "
        "outward normal.",
        "",
        "| pose | origin | rotation (rows) | geometry |",
        "|---|---|---|---|",
        f"| start | {_v(b0.origin, 4)} | {'; '.join(_v(r, 6) for r in b0.rot)} | `fixed_{moving}.step` |",
    ]
    for k, tg in enumerate(mo["targets"]):
        p = Placement.from_json(tg["placement"])
        L.append(f"| {k + 1} | {_v(p.origin, 4)} | {'; '.join(_v(r, 6) for r in p.rot)} | "
                 f"`fixed_{moving}_pose{k + 1}.step` |")
    L += [
        "",
        f"Starting at the start pose, one continuous motion of your mechanism must bring {moving} to each pose in "
        f"turn, pose 1 first: its face centre within {st['pose_position']:g} mm of the stated origin and its "
        f"orientation within {st['pose_angle_deg']:g} degrees of the stated rotation, at each. Between and after "
        f"the poses {moving} may go anywhere the rules below allow. Nothing loads it.",
        "",
        "Bolts into the fixtures are ISO 4762 socket head cap screws. Each threads into a tapped hole in its",
        "fixture and is put in from your side, so its head bears on your part. The axis points from the fixture",
        "into your part.",
        "",
        "| bolt | size | position on face | axis | clearance hole dia | grip |",
        "|---|---|---|---|---|---|",
    ]
    for b in task.bolts(0):
        bt = BOLTS[b.size]
        L.append(f"| {b.id} | {b.size} | {_v(b.position)} | {_v(b.axis)} | "
                 f"{bt.clear_fine:g}-{bt.clear_coarse:g} | {bt.grip_min:g}-{bt.grip_max:g} |")
    L += [
        "",
        "*Grip* is the thickness of your part under the head: the distance along the axis from the mounting face",
        "to where the hole ends.",
        "",
        "## What you hand in",
        "",
        f"Up to {AS.MAX_PARTS} rigid parts, each a STEP file holding one solid, modelled in world coordinates "
        f"as assembled with {moving} at its start pose, and `{AS.ASSEMBLY_FILE}`, all in this folder.",
        "",
        f"- A part bolts to {fixed} (with every bolt of {fixed}), to {moving} (with every bolt of {moving}), or to "
        "neither: a link held only by its pivots. At least one part bolts to each fixture. Parts bolted to the "
        "same fixture move as one body with it; every link is a body of its own.",
        "- Bodies are joined only by **pivots**: a round pin turning in a round bore. A pin is either part of a "
        "printed part or a catalogue screw (below) clamped in one body and passing through a bore in the other. "
        "All pivot axes must be parallel: the mechanism moves in one plane.",
        "- One pivot is the **drive**: it turns the second part it joins relative to the first, right-handed "
        "about its axis (thumb along the axis), from 0 to the drive's range in degrees (negative turns the "
        "other way). The mechanism must have exactly one way to move, and the drive must control it.",
        "- Parts are rigid: nothing may rely on bending (no flexures, no snap fits). They are assembled, not "
        "printed in place: each part goes into its place along one straight line, from outside everything, in "
        "the order listed, and a part that bolts to a fixture is bolted as soon as it is there. Then the "
        "fasteners go in, in order.",
        "- A fastener is a catalogue item, not a part: an ISO 4762 socket head cap screw (sizes "
        + ", ".join(b for b in BOLTS if b in AS.NUTS) + ", lengths " + ", ".join(str(v) for v in AS.LENGTHS)
        + " mm) with an ISO 4032 hex nut, clamped to the part it is held by. The screw goes in along its axis, "
        "shank first; the nut comes on from the far end.",
        "",
        "```json",
        "{",
        '  "parts": [',
        f'    {{"name": "p1", "file": "p1.step", "bolts_to": "{fixed}", "insert_direction": [dx, dy, dz]}},',
        f'    {{"name": "p2", "file": "p2.step", "bolts_to": "{moving}", "insert_direction": [dx, dy, dz]}},',
        '    {"name": "p3", "file": "p3.step", "bolts_to": null, "insert_direction": [dx, dy, dz]}',
        "  ],",
        '  "pivots": [',
        '    {"name": "J1", "joins": ["p1", "p3"], "point": [x, y, z], "axis": [ax, ay, az]}',
        "  ],",
        '  "drive": {"pivot": "J1", "range_deg": 90},',
        '  "fasteners": [',
        '    {"kind": "bolt", "size": "M5", "length": 30, "head_at": [x, y, z], "axis": [ax, ay, az],',
        '     "grip": 20.0, "held_by": "p1"}',
        "  ]",
        "}",
        "```",
        "",
        "`insert_direction` is the way the part travels as it goes in. A pivot's `point` is any point on its "
        "axis. For a screw, `head_at` is the centre of the head's bearing face, `axis` points from the head into "
        "the shank, `grip` is the distance from the head's bearing face to the nut's bearing face, and `held_by` "
        "names the part it is clamped to. The grader models the screw as its head (ISO 4762 diameter and height) "
        "and a plain shank of the nominal diameter and stated length, and the nut as a ring as wide as the nut "
        "across its corners and as high as the nut ("
        + ", ".join(f"{k} {AS.nut_corner_d(k):.2f} x {m:g}" for k, (s, m) in AS.NUTS.items()) + " mm).",
        "",
        "## What is checked",
        "",
        "Your answer passes when every check below passes.",
        "",
        "**Each part on its own.** A part bolted to a fixture is judged against that fixture, at the start, as a "
        "connecting part is judged in the earlier levels; a link is judged for `FILE_VALID`, `CLEAN` and "
        "`MANUFACTURING` only.",
        "",
        "- `FILE_VALID` -- the file holds exactly one valid, closed solid.",
        f"- `HOLES` -- a through hole on every bolt axis of its fixture, within {t['hole_axis_offset_max']:g} mm, "
        "diameter in the clearance range, grip in range.",
        f"- `SEATING` -- a flat face lies on the fixture's mounting face (stand-off at most {t['seat_gap_max']:g} mm) "
        f"and covers at least {t['seat_contact_min']:.0%} of the ring around each bolt, from the hole out to the "
        "head diameter.",
        f"- `NO_INTERFERENCE` -- the part does not overlap its fixture (limit {t['overlap_volume_max']:g} mm3), "
        f"and each bolt has room: its shank, and its head with {t['head_clearance_radial']:g} mm clearance around "
        f"and {t['head_clearance_axial']:g} mm above.",
        f"- `TOOL_ACCESS` -- every bolt into its fixture can be turned with a straight tool: above each head, along "
        f"the bolt axis, a cylinder of the head diameter + {t['tool_diameter_extra']:g} mm and {t['tool_length']:g} mm "
        "long, starting at the top of the head, must be free of the part itself, both fixtures (the other at its "
        "start pose) and every part listed before it (each part is bolted as soon as it is in place).",
        "- `CLEAR_PATH` -- the part does not overlap the other fixture or a part listed before it (same overlap "
        "limit).",
    ]
    L += required_lines(task)
    ps = process_sentence(task)
    if ps:
        L.append("- `MANUFACTURING` -- each part is made on its own. " + ps)
    L += [
        "",
        "**The assembly, standing still:**",
        "",
        f"- `ASSEMBLY_FILE` -- `{AS.ASSEMBLY_FILE}` is complete and consistent, as above.",
        "- `FASTENERS` -- every screw is long enough for its grip plus its nut, and its head and its nut each bear "
        f"on at least 75% of their ring (from the {', '.join(f'{k} {BOLTS[k].clear_coarse:g}' for k in AS.NUTS)} mm "
        "hole out to the head diameter, or the nut's width across flats) on parts of the body it is clamped to.",
        "- `ASSEMBLED` -- at the start nothing overlaps anything else: parts, screws, nuts, fixtures, the heads of "
        f"the bolts into the fixtures (limit {t['overlap_volume_max']:g} mm3).",
        "- `INSERTION` -- each part, then each screw and nut, travels in along its straight line from outside "
        "everything to its place without running into what is already there. Sliding along a face is fine.",
        f"- `PIVOTS` -- at every declared pivot one body has a round pin (a cylindrical face of a part, or a screw's "
        f"shank) and the other a round bore, both on the declared axis (within {st['pivot_offset_max']:g} mm and "
        f"{st['pivot_angle_max_deg']:g} degrees), overlapping along it by at least 1 mm, the bore's radius at least "
        f"{st['gap_min']:g} mm larger than the pin's.",
        "- `MOBILITY` -- counting the bodies (the two fixtures with what is bolted to them, and each link) and the "
        "pivots, the mechanism has exactly one way to move, and the drive controls it.",
        "",
        "**The motion.** The mechanism is built from your declared pivots and driven from 0 through the drive's "
        "range in small steps; at each step every body's place follows from the pivots.",
        "",
        f"- `MOTION` -- the mechanism closes at every step: it never reaches a dead point (where the drive no "
        f"longer controls it) and never jumps to another way of closing. Steps are at most {st['step_max_deg']:g} "
        f"degrees of drive, and small enough that no point of any body moves more than {st['step_move_max']:g} mm "
        "per step. At no step may two bodies overlap.",
        f"- `GAP` -- at every step any two bodies stay at least {st['gap_min']:g} mm apart, so printed surfaces "
        "that move past each other do not rub or fuse.",
        f"- `POSES` -- during the motion {moving} reaches every pose, in order, within the tolerances above.",
        f"- `INTERLOCK` -- at the start, no pivot comes apart: its two bodies, one shifted against the other "
        f"{st['loose_shift']:g} mm in any of 26 directions (along the pivot axis, square to it, and every diagonal "
        f"between), or tilted {st['loose_tilt']:g} degrees about either direction square to the axis through the "
        "pivot point, must meet on the way.",
        f"- `PLAY` -- at the start, each pivot's two bodies move freely at most {st['play_max']:g} mm along the axis "
        f"and either way square to it, and tilt freely at most {st['tilt_max_deg']:g} degrees about either "
        "direction square to the axis (the one nearest world +Z made square to the axis, and the axis crossed "
        "with that one).",
        "",
        "Measured on exact geometry: distances and overlap volumes between the solids. The same files always give "
        "the same verdict.",
        "",
        "## Grading your answer",
        "",
        "```",
        f"python -m {NAME}.moves verify <this folder>",
        "```",
        "",
        "Volumes and each part's size are reported, not graded.",
        "",
        "`_solution/` holds a reference answer. If you are the one being tested, do not open it.",
        "",
    ]
    return "\n".join(L)
