"""python -m jointbench.joints <joint> [--selftest | --gen | --verify DIR | --picture PNG]

  --selftest            reference and broken copies through real STEP files, per seed
  --gen --out DIR       write instances: brief, numbers, given parts, the reference
  --verify DIR          grade DIR/output.step (or --step FILE) against that instance
  --picture PNG         the docs picture of one instance
  joint 'all'           every joint in turn (selftest only)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

from . import ORDER, common as C, get, render as R


def _seeds(s: str):
    if "-" in s:
        a, b = s.split("-")
        return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


# --------------------------------------------------------------- pictures


def scene(name, inst):
    """Shapes to draw (joint frame), a title and notes, per joint."""
    m = get(name)
    p = inst.p
    fixed = list(m.fixed_local(inst).values())
    ref = m.reference(inst)
    if name == "revolute":
        items = [(f, R.GREY) for f in fixed]
        items += [(C.rotated_z(ref, p["rot_hi"]), R.PALE), (C.rotated_z(ref, p["rot_lo"]), R.PALE), (ref, R.BLUE)]
        notes = [f"Lever ({'pin in the plate bore' if p['given'] == 'hole' else 'bore on the standing pin'}), "
                 f"fit {p['fit']}, shown at rest (blue) and at {p['rot_lo']:g} / +{p['rot_hi']:g} deg (pale).",
                 "Checked: coaxial, clearance, bearing length, retention, drive hole, and no overlap in "
                 "any step of the swing; the lug marks the forbidden arc."]
        return items, "Revolute joint", notes, (1.0, -1.2, 1.4)
    if name == "interference":
        items = [(R.half(f), R.GREY) for f in fixed] + [(R.half(ref), R.BLUE)]
        notes = [f"{p['D']:g} {p['fit']}, given {p['given']} (grey, cut in half), designed part blue.",
                 "Checked: size in the ISO zone, engagement, lead-in cone at the leading end, and overlap "
                 "only in the declared fit ring."]
        return items, "Interference fit", notes, (1.0, -1.6, 0.6)
    if name == "keyed":
        from .keyed import find_keyway, key_solid
        from .standards import it
        shaft = fixed[0] if p["design"] == "hub" else ref
        hub = ref if p["design"] == "hub" else fixed[0]
        ks = find_keyway(shaft, p["d"], p["b"], internal=False)
        key = key_solid(p, ks, p["b"] - it(p["b"], 9), p["h"] - it(p["h"], 11))
        items = [(shaft, R.GREY if p["design"] == "hub" else R.BLUE),
                 (C.moved_by(hub, (0, 0, p["hub_len"] + 25)), R.BLUE if p["design"] == "hub" else R.GREY),
                 (key, R.ORANGE)]
        notes = [f"Shaft {p['d']:g} mm, DIN 6885 key {p['b']:g} x {p['h']:g} x {p['key_len']:g} (orange), hub lifted "
                 "off to show its keyway.",
                 "Checked: keyway width (N9 / JS9), depth t1 / t2, centre plane on the given one, length, and a "
                 "key at its smallest size fitting both."]
        return items, "Keyed joint", notes, (math.cos(math.radians(p["phi"] + 35)),
                                              math.sin(math.radians(p["phi"] + 35)), 0.5)
    if name == "spline":
        lift = p["hub_len"] + 15 if p["design"] == "shaft" else -(p["hub_len"] + 15)
        items = [(f, R.GREY) for f in fixed] + [(C.moved_by(ref, (0, 0, lift)), R.BLUE)]
        notes = [f"ISO 14 {p['series']} {p['code']}, designed {p['design']} (blue) slid off along z.",
                 "Checked: three sections square to the axis (count, flanks, spacing, width, minor, major), "
                 "engagement, sliding on without collision."]
        return items, "Spline joint", notes, (1.0, -1.3, 1.1)
    if name == "involute":
        items = [(f, R.GREY) for f in fixed] + [(C.moved_by(ref, (0, 0, p["hub_len"] + 15)), R.BLUE)]
        notes = [f"Module {p['m']:g}, {p['z']} teeth, 30 deg flat root; designed shaft (blue) slid out of the hub.",
                 "Checked: every flank an involute of the base circle within 5 um, tooth thickness at the pitch "
                 "circle, tip and root diameters, sliding in."]
        return items, "Involute spline", notes, (1.0, -1.3, 1.1)
    if name == "kinematic":
        from .kinematic import contacts
        cs, _ = contacts(ref, fixed[0])
        import build123d as bd
        below = bd.Pos(0, 0, p["zc"] - 500) * bd.Box(1000, 1000, 1000)
        balls = ref & below
        items = [(f, R.GREY) for f in fixed] + [(balls, R.BLUE)]
        items += [(R.ball(c[0], 0.12 * p["ball_d"] + 0.5), R.RED) for c in cs]
        notes = [f"Balls {p['ball_d']:g} mm seated in {p['groove_angle']:g} deg V-grooves (blue; the top part's plate "
                 "above them not drawn); red: the six contact points found by the check.",
                 "Checked: exactly six point contacts, constraint matrix full rank and well conditioned, all "
                 "contacts compressive under the clamping force."]
        return items, "Kinematic coupling", notes, (0.5, -1.0, 1.1)
    if name == "snapfit":
        items = [(R.half(f, False), R.GREY) for f in fixed] + [(R.half(ref, False), R.BLUE)]
        notes = ["Section through the hook, seated: the beam hangs in a pocket, its lip caught under the plate.",
                 "Checked: no overlap seated; pushed on, overlap within the allowed bending; pulled back, "
                 "caught; square retaining face. Strain reported."]
        return items, "Snap fit", notes, (0.35, 1.0, 0.3)
    raise KeyError(name)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m jointbench.joints", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("joint", help=f"one of {', '.join(ORDER)}, or 'all'")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--seeds", default="0-2", help="e.g. 0-4 or 0,3,7")
    ap.add_argument("--gen", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--out", default="joint_tasks")
    ap.add_argument("--rules-not-told", action="store_true")
    ap.add_argument("--verify", metavar="DIR")
    ap.add_argument("--step", help="file to grade (default DIR/output.step)")
    ap.add_argument("--picture", metavar="PNG")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    names = ORDER if a.joint == "all" else [a.joint]
    if a.selftest:
        bad = 0
        for name in names:
            m = get(name)
            print(f"{name}: {m.TITLE}")
            t0 = time.perf_counter()
            rows = C.selftest(m, _seeds(a.seeds), verbose=not a.json)
            n_ok = sum(r["ok"] for r in rows)
            tc = [r["t_check"] for r in rows]
            print(f"{name}: {n_ok}/{len(rows)} rows as expected; check time median "
                  f"{sorted(tc)[len(tc) // 2]:.2f} s, max {max(tc):.2f} s; total {time.perf_counter() - t0:.1f} s\n")
            bad += len(rows) - n_ok
        return 1 if bad else 0
    if len(names) != 1:
        ap.error("'all' works with --selftest only")
    name = names[0]
    m = get(name)
    if a.gen:
        for seed in range(a.seed, a.seed + a.n):
            kw = {"rules_told": not a.rules_not_told}
            inst = m.generate(seed, **kw)
            d = os.path.join(a.out, f"{name}-s{seed}")
            os.makedirs(os.path.join(d, "_solution"), exist_ok=True)
            for k, s in m.fixed_parts(inst).items():
                C.write_step(s, os.path.join(d, f"fixed_{k}.step"))
            C.write_step(C.to_world(m.reference(inst), inst.frame), os.path.join(d, "_solution", "reference.step"))
            with open(os.path.join(d, "README.md"), "w") as f:
                f.write(f"# {m.TITLE}, seed {seed}\n\n{m.brief(inst)}\n\nWrite your part to output.step.\n")
            with open(os.path.join(d, "instance.json"), "w") as f:
                json.dump({"joint": name, "seed": seed, "generate": kw, **inst.to_json()}, f, indent=1)
            print(d)
        return 0
    if a.verify:
        with open(os.path.join(a.verify, "instance.json")) as f:
            meta = json.load(f)
        inst = m.generate(meta["seed"], **meta.get("generate", {}))
        path = a.step or os.path.join(a.verify, "output.step")
        t0 = time.perf_counter()
        results = C.grade_step(m, inst, path)
        dt = time.perf_counter() - t0
        failed = [r for r in results if r.ok is False and r.required]
        if a.json:
            print(json.dumps({"passed": not failed, "seconds": round(dt, 3),
                              "checks": [r.to_json() for r in results]}, indent=1))
        else:
            print(f"{'PASS' if not failed else 'FAIL'}  {name} seed {meta['seed']}  ({dt:.2f} s)")
            for r in results:
                tag = "info" if not r.required else ("ok  " if r.ok else ("skip" if r.ok is None else "FAIL"))
                print(f"  {tag} {r.name:<16} {r.reason}")
        return 1 if failed else 0
    if a.picture:
        inst = m.generate(a.seed, aligned=True)
        items, title, notes, view = scene(name, inst)
        R.picture(items, a.picture, title, notes, view=view)
        print(a.picture)
        return 0
    ap.error("say what to do: --selftest, --gen, --verify or --picture")


if __name__ == "__main__":
    sys.exit(main())
