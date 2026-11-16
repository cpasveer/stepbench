"""Writing a mechanism task as a folder an agent can work in.

```
motion-fourbar-s0/
  README.md          the brief: what is missing, its joints, the motion table
  motion.json        the same, machine-readable (the source of truth)
  brief.png          plan view: the mechanism, the gap, the required path
  fixed_<part>.step  every remaining part, in world coordinates, as drawn
  _solution/         the removed part and the full mechanism -- not for the agent
```
"""
from __future__ import annotations

import json
import math
import os

from ..geometry import write_step
from . import kin
from .kin import Mechanism
from .render import picture


def _v(p, n=3):
    return "(" + ", ".join(f"{x:.{n}f}" for x in p) + ")"


def _mode_text(m: Mechanism, removed: str) -> list[str]:
    out = []
    for st in m.steps:
        if st["type"] == "rrr":
            A, B = (m.parts[st["a"]].label or st["a"]), (m.parts[st["b"]].label or st["b"])
            side = "left" if st.get("mode", 1) > 0 else "right"
            sign = "positive" if st.get("mode", 1) > 0 else "negative"
            pa = m.parts[m.joints[st['ja']].other(st['a'])[0]].label
            out.append(f"{A[:1].upper() + A[1:]} and {B} close the loop in one of two ways. The assembly kept: with P the pin on {pa}, "
                       f"Q the pivot of {B} and M their shared pin, all in plane coordinates (u, v), the cross product "
                       f"(Q - P) x (M - P) = (Qu - Pu)(Mv - Pv) - (Qv - Pv)(Mu - Pu) is {sign}. Drawn with u to the right "
                       f"and v up (the normal then points at you), M lies to the {side} of the line from P to Q. "
                       f"This holds at every input value; the two assemblies never swap.")
        if st["type"] == "rrp":
            out.append(f"{m.parts[st['b']].label} sits on the far side of the crank, in the direction of the guide's "
                       f"+x (see the guide direction below), not behind it.")
    return out


def brief(task: dict, m: Mechanism) -> str:
    rem = task["removed"]
    lbl = m.parts[rem].label or rem
    fr = m.frame
    X = tuple(fr.rot[i][0] for i in range(3))
    Y = tuple(fr.rot[i][1] for i in range(3))
    N = tuple(fr.rot[i][2] for i in range(3))
    a0, a1 = m.range
    inp = m.input
    ip = m.parts[inp["part"]].label
    lines = [f"# Design the missing part: {lbl}", ""]
    names = {"fourbar": "four-bar linkage", "slider": "crank-slider", "yoke": "Scotch yoke",
             "oldham": "Oldham coupling"}
    lines += [f"A working {names.get(m.type, m.type)} has had one part removed: **{lbl}**. Model a replacement "
              f"so that the whole mechanism moves as the table below requires, through the full input range, "
              f"without any two parts touching.", ""]
    lines += ["## The mechanism plane", "",
              "Every part moves in a plane parallel to one mechanism plane; heights along its normal never change.",
              f"Plane origin {_v(fr.origin)}; plane x {_v(X, 6)}; plane y {_v(Y, 6)}; normal {_v(N, 6)} (world, mm).",
              "Below, (u, v) are plane coordinates and w the height along the normal; world = origin + u*x + v*y + w*n.",
              ""]
    lines += ["## Input and output", ""]
    if "line" in inp:
        lines.append(f"Input: the angle of {ip}'s groove, from plane x towards plane y, degrees, "
                     f"turned from {a0:.3f} to {a1:.3f}.")
    else:
        lines.append(f"Input: the angle from {ip}'s pivot to its pin, measured from plane x towards plane y, "
                     f"in degrees, turned from {a0:.3f} to {a1:.3f} (one full turn).")
    o = m.output
    op = m.parts[o["part"]].label
    if o["kind"] == "point":
        what = "the output mark (a hole of diameter 3.0, +-0.1)" if o["part"] == rem and o["feature"] == "mark" else \
            f"the centre of {op}'s {m.parts[o['part']].features.get(o['feature']).kind if o['part'] != rem else 'joint'}"
        if o["part"] == rem and o["feature"] == "mark":
            lines.append(f"Output: the position of {what} on **{lbl}**: you place it. "
                         f"It must follow the table within {task['tolerances']['point_mm']} mm.")
        else:
            lines.append(f"Output: the position of {what}. It must follow the table within "
                         f"{task['tolerances']['point_mm']} mm.")
    else:
        lines.append(f"Output: the angle of {op} (pivot to pin, or its groove), from plane x towards plane y, degrees. "
                     f"It must follow the table within {task['tolerances']['angle_deg']} deg.")
    lines.append("")
    lines += ["## The fixed parts", ""]
    for pid, p in m.parts.items():
        if pid == rem:
            continue
        note = " (drawn parked: not where it sits at the first input value)" if pid in task.get("parked", []) else ""
        lines.append(f"- `fixed_{pid}.step`: {p.label}{note}")
    lines.append("")
    lines += [f"## The joints {lbl} must make", ""]
    fits = task["fits"]
    for jid, g in task["joints_of_removed"].items():
        j = m.joints[jid]
        mate = g["feature"]
        with_ = m.parts[g["with"]].label
        lo, hi = fits["clear_min"], fits["clear_max"]
        if j.kind == "pin_slot":
            need = (f"a slot {g['size'] + lo:.2f}-{g['size'] + hi:.2f} wide for its pin (dia {g['size']:.2f})"
                    if mate == "pin" else
                    f"a pin dia {g['size'] - hi:.2f}-{g['size'] - lo:.2f} riding in its slot ({g['size']:.2f} wide)")
        elif mate == "pin":
            need = f"a hole dia {g['size'] + lo:.2f}-{g['size'] + hi:.2f} on its pin (dia {g['size']:.2f})"
        elif mate == "hole":
            need = f"a pin dia {g['size'] - hi:.2f}-{g['size'] - lo:.2f} in its hole (dia {g['size']:.2f})"
        elif mate == "slot":
            need = f"a tongue {g['size'] - hi:.2f}-{g['size'] - lo:.2f} wide in its slot ({g['size']:.2f} wide)"
        else:
            need = f"a slot {g['size'] + lo:.2f}-{g['size'] + hi:.2f} wide round its tongue ({g['size']:.2f} wide)"
        line = (f"- with {with_}: {need}; the {mate} spans w = {g['z_band'][0]:.2f} to {g['z_band'][1]:.2f}, "
                f"and yours must overlap it by at least {fits['engage_min']:g} mm")
        if "world" in g:
            line += f"; at the first input value its axis/mid-plane passes (u, v) = {_v(g['plane_xy'])}, world {_v(g['world'])}"
            if "dir_deg" in g:
                line += f", running at {g['dir_deg']:.3f} deg in the plane"
        else:
            line += "; where it sits is for you to work out from the motion"
        if "mate_blocks" in g:
            bl = g["mate_blocks"]
            t0, t1 = g["travel"]
            where = " and ".join(f"{a:.2f} to {b:.2f}" for a, b in bl)
            many = len(bl) > 1
            line += (f". Its real flat faces run from {where} mm along that direction from the point above"
                     + (f": {len(bl)} separate blocks, {', '.join(f'{b - a:.2f}' for a, b in bl)} mm long, "
                        f"and the gaps between them carry nothing" if many else f" ({bl[0][1] - bl[0][0]:.2f} mm)")
                     + f"; {lbl} moves from {t0:+.2f} to {t1:+.2f} mm along it from where it sits at the first "
                     f"input value. At every input value the tongue must bear on at least "
                     f"{fits.get('bearing_per_width', 1.5):g} x its width of these faces in all "
                     f"({fits.get('bearing_per_width', 1.5) * (g['size'] - (lo + hi) / 2):.1f} mm for a "
                     f"{g['size'] - (lo + hi) / 2:.2f} mm tongue; `GUIDED`)"
                     + (f", and cover at least {100 * fits.get('block_cover', 0.5):g}% of each block's length: "
                        f"in every block, never carried on one alone" if many else ""))
        lines.append(line + ".")
    lines += [""] + _mode_text(m, rem) + [""]
    lines += ["## Rules", "",
              f"- Every round joint is a true cylinder with its axis along the normal; every sliding joint is two "
              f"parallel flat faces. Running clearance {fits['clear_min']}-{fits['clear_max']} mm on size.",
              f"- At every input value (checked every {task['step_deg']:g} deg) no two parts overlap "
              f"(more than {task['tolerances']['overlap_volume_max']} mm3).",
              f"- The transmission angle never falls below {task['tolerances']['transmission_min_deg']:g} deg.",
              f"- A sliding part is guided (`GUIDED`): at every input value it bears on real guide faces (the "
              f"gap between two guide blocks carries nothing) over at least "
              f"{fits.get('bearing_per_width', 1.5):g} x its own width, and where the guide is two or more "
              f"separate blocks it covers at least {100 * fits.get('block_cover', 0.5):g}% of each one, so it is "
              f"held in all of them and cannot tip on one.",
              "- A pin stays between the ends of its slot: the slot's flat sides must run far enough that the pin, "
              "radius included, never reaches an end over the whole range, or the mechanism jams (`POSES`). How "
              "far the pin travels along a slot in your part is for you to work out from the motion.",
              "- Submit `output.step`: one closed solid, in world coordinates, placed where the part sits at the "
              "first input value.", ""]
    lines += ["## The motion table", ""]
    t = task["table"]
    if t["kind"] == "point":
        lines += ["| input deg | output world x | y | z |", "|---|---|---|---|"]
        for v, w in zip(t["inputs"], t["world"]):
            lines.append(f"| {v:.3f} | {w[0]:.4f} | {w[1]:.4f} | {w[2]:.4f} |")
    else:
        lines += ["| input deg | output deg |", "|---|---|"]
        for v, w in zip(t["inputs"], t["world"]):
            lines.append(f"| {v:.3f} | {w[0] % 360:.4f} |")
    lines += ["", "Grade with `python -m jointbench.motion verify <this folder>`."]
    return "\n".join(lines) + "\n"


def write(task: dict, m_full: Mechanism, ref_world, out_dir: str) -> str:
    folder = os.path.join(out_dir, task["id"])
    os.makedirs(os.path.join(folder, "_solution"), exist_ok=True)
    solids = m_full.meta.get("solids", {})
    hidden = Mechanism.from_json(task["mechanism"])
    for pid, p in hidden.parts.items():
        if pid == task["removed"] or pid not in solids:
            continue
        pl = kin.pose_to_placement(m_full.frame, p.shown)
        write_step(solids[pid].moved(pl.location()), os.path.join(folder, f"fixed_{pid}.step"))
    write_step(ref_world, os.path.join(folder, "_solution", "reference.step"))
    full = task.pop("_full", None)
    with open(os.path.join(folder, "_solution", "mechanism_full.json"), "w") as f:
        json.dump(full, f, indent=1)
    with open(os.path.join(folder, "motion.json"), "w") as f:
        json.dump(task, f, indent=1)
    with open(os.path.join(folder, "README.md"), "w") as f:
        f.write(brief(task, hidden))
    for pid, p in hidden.parts.items():
        if pid in solids:
            p.solid = solids[pid]
    pic = {pid: s for pid, s in solids.items() if pid != task["removed"]}
    picture(hidden, pic, os.path.join(folder, "brief.png"), removed=None,
            title=f"{task['id']}: {hidden.parts[task['removed']].label} missing", table=task["table"]["plane"])
    if full is not None:
        task["_full"] = full
    return folder
