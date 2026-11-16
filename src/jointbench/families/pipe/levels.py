"""The pipe setup, levels 1-3: the fixed parts of ``round5``, the reference of
``reference``.

Seed 0 is the showcase, round 5's pipe exactly (``round5.showcase_draw``):
the L4 and L6 setups build on the same fixed parts. Other seeds vary round it:
the pipe's size and bolt pattern, B's size, and where B stands -- along the
pipe, round it (between the two bolt rows), how far out, and how its face is
turned. B's face always stands upright and the pipe leans ``LEAN`` degrees
from the vertical (at most 41: a face along the pipe then leans at most that
much, and prints). One random stream per seed, drawn before the level is
looked at, so every level of a seed has the same fixed parts; a draw is kept
only if the reference passes level 3 with the real grader.
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import numpy as np

from ...geometry import BUILD_ROOM, fits_build_volume, write_step
from ...requirements import draw_stated_load
from . import reference, round5

KNOBS_DEFAULT: dict = {}
SITUATION = round5.SITUATION

# round 5's B, measured from the pipe's frame (see ``showcase_numbers``): along the pipe, round it (deg from the
# rows' middle), out from the axis, the face's lean along the pipe and its turn round the radial direction
SHOWCASE = {"x": -11.47, "phi": -9.17, "r": 102.81, "lean": 39.86, "turn": 13.74}
ODS = (68.0, 76.0, 84.0)
PIPE_BOLTS = ("M5", "M6")
ALONG = (2, 3)
PITCHES = (35.0, 40.0, 45.0)
ROWS = (25.0, 30.0, 35.0)       # deg each row stands off the middle, round the pipe
B_BOLTS = ("M4", "M5")
B_PITCHES = (25, 30, 35)
X_RANGE = 10.0      # mm along the pipe either side of the showcase
PHI_RANGE = 8.0     # deg round the pipe
R_RANGE = 8.0       # mm out from the surface
LEAN = (32.0, 41.0)
TURN_RANGE = 8.0    # deg


def showcase_numbers(d: dict) -> dict:
    """Where B stands, as the numbers ``random_draw`` varies (for the record)."""
    _, _, plb, _ = round5.layout(d)
    F = np.array(plb.origin)
    nb = np.array(plb.vec((0, 0, 1)))
    r = math.hypot(F[1], F[2])
    rho = np.array([0.0, F[1], F[2]]) / r
    tau = np.array([0.0, rho[2], -rho[1]])
    c = nb - nb[0] * np.array([1.0, 0.0, 0.0])
    c /= np.linalg.norm(c)
    return {"x": F[0], "phi": math.degrees(math.atan2(F[1], F[2])), "r": r,
            "lean": math.degrees(math.asin(nb[0])), "turn": math.degrees(math.atan2(c @ tau, c @ -rho))}


def random_draw(rng: random.Random) -> dict:
    od = rng.choice(ODS)
    bolt = rng.choice(PIPE_BOLTS)
    n, pitch, row = rng.choice(ALONG), rng.choice(PITCHES), rng.choice(ROWS)
    xs = [(i - (n - 1) / 2) * pitch for i in range(n)]
    pipe = {"bolt": bolt, "od": od, "holes": [[x, a] for a in (-row, row) for x in xs]}
    b = {"kind": "square", "params": {"bolt": rng.choice(B_BOLTS), "pitch": rng.choice(B_PITCHES)}}
    mirror = rng.choice((1.0, -1.0))  # B on either side of the rows' middle
    x = SHOWCASE["x"] + rng.uniform(-X_RANGE, X_RANGE)
    phi = math.radians(mirror * (SHOWCASE["phi"] + rng.uniform(-PHI_RANGE, PHI_RANGE)))
    r = SHOWCASE["r"] - 38.0 + od / 2 + rng.uniform(-R_RANGE, R_RANGE)
    lean = math.radians(rng.uniform(*LEAN))
    turn = math.radians(mirror * (SHOWCASE["turn"] + rng.uniform(-TURN_RANGE, TURN_RANGE)))
    rho = np.array([0.0, math.sin(phi), math.cos(phi)])
    tau = np.array([0.0, rho[2], -rho[1]])
    c = math.cos(turn) * -rho + math.sin(turn) * tau
    nb = math.sin(lean) * np.array([1.0, 0.0, 0.0]) + math.cos(lean) * c
    centre = np.array([x, 0.0, 0.0]) + r * rho
    return {"pipe": pipe, "B": b, "b_centre": [round(float(v), 6) for v in centre],
            "b_normal": [round(float(v), 9) for v in nb],
            "yaw": round(rng.uniform(0.0, 360.0), 1),
            "origin": [round(rng.uniform(-150.0, 150.0) * 2) / 2 for _ in range(3)],
            "load_kg": draw_stated_load(rng)}


def _frame_ok(task) -> bool:
    return all(max(abs(v) for v in f.normal) <= 0.95 for f in task.faces(0))


def generate(seed: int, level: int, max_tries: int = 40, tally: dict | None = None):
    """Return (task, draw, reference solid in world coordinates) for ``level``
    (1, 2 or 3). Deterministic in ``seed``; the same draw at every level."""
    from ...grade import grade

    tally = tally if tally is not None else {}

    def no(why):
        tally[why] = tally.get(why, 0) + 1

    rng = random.Random(f"pipe-v1-{seed}")
    for _ in range(max_tries):
        d = round5.showcase_draw() if seed == 0 else random_draw(rng)
        strict = round5.task_from_draw(seed, d, 3)
        for _ in range(30):  # turn the frame again until B's face is not near a coordinate plane
            if _frame_ok(strict) or seed == 0:
                break
            no("a face near a coordinate plane (frame redrawn)")
            d["yaw"] = round(rng.uniform(0.0, 360.0), 1)
            strict = round5.task_from_draw(seed, d, 3)
        try:
            ref = reference.build(d, round5.layout(d))
        except Exception:  # noqa: BLE001 - a draw whose reference cannot be built is redrawn
            no("reference could not be built")
            if seed == 0:
                raise
            continue
        if len(ref.solids()) != 1 or not fits_build_volume(ref, BUILD_ROOM):
            no("reference not one solid or too big for the printer")
            continue
        ref = ref.moved(round5.placements(d)[0].location())
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "reference.step")
            write_step(ref, p)
            rep = grade(strict, p, optional=False, score=False, skip=frozenset({"STIFFNESS"}))  # cheap checks first
            if rep.passed:
                rep = grade(strict, p, optional=False, score=False,
                            skip=frozenset({"CLEAN", "MANUFACTURING", "SEAT_INSIDE"}))
        if not rep.passed:
            no("reference fails: " + ", ".join(sorted(set(rep.failed()))))
            if seed == 0:
                raise RuntimeError(f"seed 0 (the showcase): the reference fails: {rep.failed()}")
            continue
        st = next((c for c in rep.checks if c.name == "STIFFNESS"), None)
        if st is not None and (st.details or {}).get("verdict") == "marginal":
            no("stiffness only marginal")
            continue
        task = round5.task_from_draw(seed, d, level)
        a, b = task.parts["A"], task.parts["B"]
        d["summary"] = (f"A {a.description}; B {b.description}"
                        + (f"; {d['load_kg']:g} kg on B" if level >= 3 else ""))
        return task, d, ref
    raise RuntimeError(f"seed {seed}: no valid pipe draw in {max_tries} tries ({tally})")
