"""The browser view of a level 6 answer: the box, the linkage carrying B home, then a pull on the latch.

Built on level 4c's view (``jointbench.moves.view``: STL per body, the motion from the grader's own solver,
B's target poses as ghosts) with the box's walls (see-through) added, and after the drive a few frames in
which B's side is pulled straight down at home until the latch stops it (the play the grader measures).
"""
from __future__ import annotations

import json
import os

import build123d as bd
import numpy as np

from ...task import Task
from ..view import export as export_moves


WORDS = {"pipe": ("the pipe", "the plate"), "flange": ("the flange", "the block"),
         "facing": ("the fixed face", "the face")}  # what the fixtures are called, per setup


def export(task_dir: str, assembly_path: str, out_dir: str, free_mm: float | None = None) -> str:
    export_moves(task_dir, assembly_path, out_dir)
    task = Task.load(os.path.join(task_dir, "task.json"))
    mo = task.meta["motion"]
    with open(os.path.join(out_dir, "motion.json"), encoding="utf-8") as f:
        M = json.load(f)
    walls = task.obstacles_at(0)
    if walls:
        bd.export_stl(bd.Compound([o.solid() for o in walls]), os.path.join(out_dir, "box.stl"))
        M["bodies"].append({"name": "box", "files": [{"file": "box.stl", "color": 0xc8ced8, "opacity": 0.12}]})
    # the solver runs from home (as assembled) outwards; shown the way B travels: from the start home
    M["frames"] = M["frames"][::-1]
    span = M["frames"][0]["deg"]
    for f in M["frames"]:
        f["label"] = f"drive {span - f['deg']:+.0f} deg of {span:+.0f} from the start"
    M["frames"][-1]["label"] = "home: latched"
    last = M["frames"][-1]
    moving = mo["moving"]
    down = np.asarray(mo["latch"]["pull_direction"], float)
    free = free_mm if free_mm is not None else 0.4  # the reference's play; the grader measures any answer's
    for k in range(1, 9):
        t = free * k / 8
        mats = dict(last["m"])
        m = np.array(last["m"][moving], float).reshape(4, 4)
        m[:3, 3] += t * down
        mats[moving] = [round(float(v), 6) for v in m.flatten()]
        lab = f"home; pulled down {t:.2f} mm" + (": the latch holds it" if k == 8 else "")
        M["frames"].append({"deg": last["deg"], "m": mats, "to_targets_mm": last["to_targets_mm"], "label": lab})
    M["title"] = "Moves (level 6): a linkage carries B home in a closed box and latches it"
    fixed_w, carried_w = WORDS.get(task.meta.get("situation"), WORDS["pipe"])
    M["about"] = (f"Grey: {fixed_w} (A, fixed) and {carried_w} B (carried). Colours: the frame on {fixed_w}, the carrier on B, "
                  "the crank and the rocker. Faint: the closed box (its opening is in the top). Orange ghosts: B at the "
                  "poses it must pass, the last one its home. Slide or play: the drive carries B home; at the end B is "
                  "pulled straight down and the latch (a lug on the carrier over a ledge on the frame) stops it.")
    with open(os.path.join(out_dir, "motion.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(M, f)
    return out_dir
