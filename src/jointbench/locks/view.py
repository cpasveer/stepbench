"""A browser view of a "locks" answer: B pushed in along the axis, turned, then pulled back.

Writes STL files per side, ``motion.json`` (B's side placement per frame) and the same three.js
page as level 4c's view (``jointbench.moves.view.PAGE``) into ``out_dir``.
"""
from __future__ import annotations

import json
import os

import build123d as bd
import numpy as np

from ..geometry import read_step
from ..moves import motion as MO
from ..moves.view import PAGE
from ..task import Task
from .generate import FILES


def _mat(pl) -> list:
    m = np.eye(4)
    m[:3, :3] = np.array(pl.rot)
    m[:3, 3] = np.array(pl.origin)
    return [round(float(v), 6) for v in m.flatten()]


def export(task_dir: str, answer_dir: str, out_dir: str, approach: float = 40.0) -> str:
    task = Task.load(os.path.join(task_dir, "task.json"))
    lk = task.meta["lock"]
    os.makedirs(out_dir, exist_ok=True)
    fx = task.fixed_solids(0)
    bodies = []
    for n, col in (("A", 0x66a897), ("B", 0x3f9fb0)):
        bd.export_stl(fx[n], os.path.join(out_dir, f"fixture_{n}.stl"))
        bd.export_stl(read_step(os.path.join(answer_dir, FILES[n])), os.path.join(out_dir, f"part_{n}.stl"))
        bodies.append({"name": n, "files": [{"file": f"fixture_{n}.stl", "color": 0x8d96a5, "opacity": 0.55},
                                            {"file": f"part_{n}.stl", "color": col, "opacity": 0.45 if n == "A" else 1.0}]})
    back = -np.asarray(lk["insert_direction"], float)
    ident = MO.shift((0.0, 0.0, 0.0))
    frames = []
    for i in range(21):  # push in
        t = approach * (1 - i / 20)
        frames.append({"deg": 0.0, "label": f"push in: {t:.0f} mm to go", "m": {"A": _mat(ident), "B": _mat(MO.shift(t * back))}})
    th = float(lk["turn_deg"])
    n = max(1, int(abs(th) // 2))
    for i in range(1, n + 1):  # turn
        a = th * i / n
        frames.append({"deg": a, "label": f"turn: {a:.0f} of {th:g} deg",
                       "m": {"A": _mat(ident), "B": _mat(MO.about(lk["axis_point"], lk["axis_direction"], a))}})
    turned = MO.about(lk["axis_point"], lk["axis_direction"], th)
    for i in range(1, 11):  # pull back: it stops at the lock (the grader measures where)
        t = 0.05 * i
        frames.append({"deg": th, "label": f"pull back {t:.2f} mm: locked (the grader allows {task.tolerances['locks']['play_max']:g} mm)",
                       "m": {"A": _mat(ident), "B": _mat(turned.then(MO.shift(t * back)))}})
    meta = {"task": task.id, "title": "Locks prototype: push in, turn, locked", "bodies": bodies, "targets": [],
            "frames": frames, "stopped": None,
            "about": ("Grey: fixtures A (stays) and B (moves). Green, see-through: the part on A. Blue: the part on B. Slide or press "
                      "play: B comes straight in along the axis (orange line), turns the stated angle, then is pulled "
                      "back and stops after a fraction of a millimetre: locked. Drag to turn the view, scroll to zoom."),
            "pivots": [{"name": "axis", "point": lk["axis_point"], "axis": lk["axis_direction"]}]}
    with open(os.path.join(out_dir, "motion.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(meta, f)
    with open(os.path.join(out_dir, "index.html"), "w", encoding="utf-8", newline="\n") as f:
        f.write(PAGE.replace("<title>Moves prototype</title>", "<title>Locks prototype</title>"))
    return out_dir
