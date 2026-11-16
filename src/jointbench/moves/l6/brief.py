"""The brief (README.md) of a level 6 task: level 4c's brief (fixtures, poses, the assembly file, the
mechanism's checks) with what level 6 adds: the closed box and its opening, the latch at home and the
stated load, catalogue ball bearings as pivots, and the score. Every rule and number, never the answer."""
from __future__ import annotations

from ... import NAME
from ...catalogue import BEARINGS, BOLTS, plastic_allowables
from ...material_score import brief_line
from ...task import Task
from ...workspace import required_lines
from ..brief import brief as brief_moves
from ..generate import SITUATION as SITUATION_4C


def _v(v, n=4):
    return "(" + ", ".join(f"{x:.{n}f}" for x in v) + ")"

SITUATION = (
    "Two fixtures are given, inside a closed box with one opening in its top wall. Fixture A stays where it\n"
    "is. Fixture B starts where it is placed and must then be carried, by one continuous motion of a\n"
    "linkage you design, through two stated placements into its end placement -- its home, where it sits in\n"
    "the earlier levels. There it must latch, and hold a stated load."
)


def brief(task: Task) -> str:
    t, st, mo = task.tolerances, task.tolerances["moves"], task.meta["motion"]
    fixed, moving = mo["fixed"], mo["moving"]
    la, bx = mo["latch"], mo["box"]
    al = plastic_allowables()
    text = brief_moves(task)
    text = text.replace(SITUATION_4C, SITUATION)
    text = text.replace(f"# Task {task.id} -- make it move", f"# Task {task.id} -- level 6, moves")
    n_poses = len(mo["targets"])
    # where B must go: level 6's own section (assembled at home; the drive takes B outwards)
    from ...geometry import Placement as _Pl

    i = text.index(f"## Where {moving} must go")
    j = text.index("Bolts into the fixtures are ISO 4762")
    b0 = task.poses[0][moving]

    def row(name, p, geo):
        return f"| {name} | {_v(p.origin)} | {'; '.join(_v(r, 6) for r in p.rot)} | `{geo}` |"

    tg = {t_["name"]: _Pl.from_json(t_["placement"]) for t_ in mo["targets"]}
    where = [
        f"## Where {moving} must go",
        "",
        f"A placement is the origin of {moving}'s own frame (the centre of its mounting face) and the rotation of that "
        f"frame (row by row: world = rotation x local + origin). {moving}'s local +Z is its face's outward normal.",
        "",
        "| placement | origin | rotation (rows) | geometry |",
        "|---|---|---|---|",
        row("start", tg["start"], f"fixed_{moving}_start.step"),
        row("pose 1", tg["pose1"], f"fixed_{moving}_pose1.step"),
        row("pose 2", tg["pose2"], f"fixed_{moving}_pose2.step"),
        row("home (end)", b0, f"fixed_{moving}.step"),
        "",
        f"{moving} is carried from the start through pose 1 and pose 2 into home, by one continuous motion of your "
        f"linkage: its face centre within {st['pose_position']:g} mm of each stated origin and its orientation within "
        f"{st['pose_angle_deg']:g} degrees of each stated rotation. **Model and assemble everything with {moving} at "
        f"home** (the latch engaged). The grader drives your linkage from there, over your drive's range, out the way "
        f"{moving} came: it must meet pose 2, then pose 1, then the start, in that order (the same path, travelled "
        f"backwards). Between the poses {moving} may go anywhere the rules below allow.",
        "",
    ]
    text = text[:i] + "\n".join(where) + text[j:]
    text = text.replace(f"as assembled with {moving} at its start pose", f"as assembled with {moving} at home")
    text = text.replace("is carried; shown here at its start", "is carried; shown here at home, its end pose")
    box = [
        "## The box",
        "",
        f"Everything happens inside a closed box (`box.step`, and the obstacles in `task.json`): walls {6:g} mm "
        f"thick, square to the mechanism's directions. Its one opening is in the top wall; where {fixed} runs "
        "through a wall, the wall is open just round it. Every part you hand in goes in through the opening, from "
        "outside the box to its place, without running into a wall, a fixture or anything already in place on the "
        "way: along one straight line (`\"insert_direction\": [dx, dy, dz]`, as above), or along a short path of "
        "straight moves and turns in place of it -- `\"path\": [{\"move\": [dx, dy, dz]}, {\"turn\": {\"point\": "
        "[x, y, z], \"axis\": [ax, ay, az], \"deg\": d}}, ...]`, in world coordinates, applied in order from a start "
        "fully outside the box to the part's place (a turn is right-handed about its axis through its point; at most "
        "20 steps). Every move is swept exactly; every turn in steps that move no point of the part more than 1 mm. Screws and nuts are brought to their line inside the "
        f"box and go in from there: along each screw's axis there must be room, clear of everything and inside the "
        f"walls, for the screw (its length and head, plus {st['screw_room']:g} mm) behind where its head ends up, and "
        f"for its nut (its thickness plus {st['screw_room']:g} mm) beyond the shank's end; each is swept in from there. Every bolt into a "
        "fixture must be reachable and turnable with its key inside the box (`TOOL_ACCESS`, `KEY_TURN`: the walls "
        "count as obstacles).",
        "",
        "## The latch and the load",
        "",
        f"At home, {moving} must be latched: pulled straight down -- along {tuple(la['pull_direction'])} -- on its "
        f"own (with every part bolted to it and the screws they hold, but without your links), it must meet the "
        f"fixed side within {st['latch_play_max']:g} mm: the linkage's path is the only way out. There it carries "
        f"{la['load_kg']:g} kg ({la['load_N']:g} N), pulling it straight down.",
        "",
        "## Pivots on ball bearings",
        "",
        "Each pivot is your choice: a pin (printed, or a catalogue screw) in a bore, as above, or a catalogue ball "
        "bearing (ISO 15: " + ", ".join(f"{k} = {d:g} x {D:g} x {B:g} mm" for k, (d, D, B) in BEARINGS.items())
        + ", bore x outside x width). For a bearing, add to the pivot `\"bearing\": {\"size\": \"608\", "
        "\"seat_on\": part}`: the part it is pressed into, one of the two the pivot joins. The pivot's `point` is "
        "then the centre of the bearing's entry face and its `axis` the way it is pressed in. The part holding it "
        "needs a bore of D to D + 0.1 mm on the axis covering 90 % of its width, 2 mm of wall all round it, a "
        "shoulder past its far face that stops the outer race, nothing inside the inner race's outside radius, and a "
        "straight way in from the entry side; it goes in with that part. The other part needs a pin on the axis of "
        "d - 0.1 to d mm through 90 % of the bearing's width. The pin runs in the bearing with no gap: that pair is "
        "not measured for the printing gap.",
        "",
    ]
    checks = [
        f"- `LATCH` -- at home, {moving}'s side pulled straight down meets the fixed side within "
        f"{st['latch_play_max']:g} mm.",
        f"- `LATCH_STRENGTH` -- the {la['load_N']:g} N pull, there, carried by the plastic where the two sides then "
        f"bear on each other (found exactly: {moving}'s side pulled 0.05 mm past where it first meets): the load "
        f"over the patches' area projected square to the pull at most {al['bearing_MPa']:g} MPa; on each side each "
        f"patch's share punching a plug of the patch's area out through the plastic behind it (outline taken as "
        f"round) at most {al['shear_MPa']:g} MPa.",
        f"- `PIVOT_SEATS` -- the same load through every pivot: on each body, the pin's diameter times its bore's "
        f"length (for a bearing: its outside diameter times its width on its seat, and its bore times its width on the "
        f"pin) carries it at most {al['bearing_MPa']:g} MPa. Allowables: {al['source']}.",
        "- `TOOL_ACCESS`, `CLEAR_PATH` -- for every part bolted to a fixture: a straight tool reaches every bolt, "
        "and the part touches nothing, counting the other fixture, the parts listed before it and the box's walls.",
    ]
    marker = "**The motion.**"
    extra = "\n".join(checks)
    text = text.replace("## What is checked", "\n".join(box) + "\n## What is checked")
    text = text.replace(marker, extra + "\n\n" + marker)
    text = text.replace("## Grading your answer", brief_line(task).replace("parts that meet", "answers that meet")
                        + "\n\n## Grading your answer")
    # graded, counted and archived by the benchmark's own command, as at levels 1-5
    text = text.replace(f"python -m {NAME}.moves verify <this folder>", f"{NAME} verify <this folder>")
    text = text.replace("Volumes and each part's size are reported, not graded.",
                        "Volumes and each part's size are reported, not graded.\n\n"
                        "`submission.py` is a build123d starting point that writes a placeholder part and "
                        "`assembly.json` (it fails); you may replace it entirely, or model in any CAD program.")
    # the model's folder holds no reference answer (it is kept apart): the brief does not mention one
    text = text.replace("`_solution/` holds a reference answer. If you are the one being tested, do not open it.\n", "")
    return text
