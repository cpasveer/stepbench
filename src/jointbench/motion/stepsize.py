"""How small must the grading step be?

Collisions are tested only at the input values stepped through; a clash
that starts and ends between two steps is missed. This study measures how
fine the step has to be for the verdict to stop changing.

For each mechanism type and seed, the removed part is given a bump that
reaches a fixed part by ``depth`` mm at the input value where it comes
closest (0.1, 0.3 and 1 mm: shallow bumps make the shortest clashes, the
hardest to catch). Each is graded at steps of 10, 5, 2, 1, 0.5, 0.25 and
0.1 degrees. The finest is taken as the truth; the study reports, per
step, how many verdicts differ from it, and the angular width of the
shortest clash seen.
"""
from __future__ import annotations

import json
import time

from . import generate
from .selftest import CONFIGS, fouls

STEPS = (10.0, 5.0, 2.0, 1.0, 0.5, 0.25, 0.1)
DEPTHS = (0.1, 0.3, 1.0)


def rows_for(mtype: str, seed: int, knobs: dict) -> list[dict]:
    task, m, ref = generate.make(mtype, seed, knobs)
    rem = task["removed"]
    solids = m.meta["solids"]
    out = []
    for depth in DEPTHS:
        got, closest = fouls(m, rem, solids, depth)
        if got is None:
            continue
        row = {"type": mtype, "seed": seed, "removed": rem, "depth": depth, "closest_at": round(closest % 360, 2),
               "verdicts": {}, "windows": {}, "seconds": {}, "max_move": {}}
        for st in STEPS:
            t0 = time.perf_counter()
            from .grade import grade_solid
            from .kin import Mechanism
            hidden = Mechanism.from_json(task["mechanism"])
            for pid, p in hidden.parts.items():
                if pid != rem and pid in solids:
                    p.solid = solids[pid]
            from .selftest import _world
            rep = grade_solid(task, hidden, _world(m, rem, got), step_deg=st)
            row["seconds"][st] = round(time.perf_counter() - t0, 3)
            hit = next(c for c in rep.checks if c.name == "NO_INTERFERENCE")
            row["verdicts"][st] = "fouls" if hit.ok is False else ("clear" if hit.ok else "skipped")
            row["max_move"][st] = rep.info.get("max_move_mm")
            if hit.ok is False:
                angs = sorted(c[0] for c in hit.details["clashes"])
                row["windows"][st] = [round(angs[0] % 360, 2), round(angs[-1] % 360, 2), len(angs)]
        out.append(row)
    return out


def _job(a):
    try:
        return rows_for(*a)
    except Exception as e:  # noqa: BLE001
        return [{"type": a[0], "seed": a[1], "error": repr(e)}]


def run(n: int, jobs: int, json_path: str | None) -> int:
    work = [(t, s, CONFIGS[t][0]) for t in CONFIGS for s in range(n)]
    rows = []
    if jobs == 1:
        for w in work:
            rows += _job(w)
    else:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(None if jobs == 0 else jobs) as pool:
            for r in pool.imap(_job, work):
                rows += r
    good = [r for r in rows if "verdicts" in r]
    finest = STEPS[-1]
    print(f"{len(good)} shallow-clash cases; truth = verdict at {finest:g} deg")
    print(f"{'step deg':>8} {'missed':>7} {'mean s/grading':>15} {'largest move mm':>16}")
    for st in STEPS:
        missed = sum(1 for r in good if r["verdicts"][st] != r["verdicts"][finest])
        secs = sum(r["seconds"][st] for r in good) / max(1, len(good))
        mv = max((r["max_move"][st] or 0) for r in good)
        print(f"{st:>8g} {missed:>7} {secs:>15.2f} {mv:>16.2f}")
    narrow = [(r["windows"][finest][2] * finest, r) for r in good if finest in r["windows"]]
    if narrow:
        w, r = min(narrow, key=lambda x: x[0])
        print(f"shortest clash: {w:.1f} deg wide ({r['type']} s{r['seed']}, bump {r['depth']} mm deep)")
    for r in rows:
        if "error" in r:
            print("error:", r)
    if json_path:
        with open(json_path, "w") as f:
            json.dump(rows, f, indent=1)
    return 0
