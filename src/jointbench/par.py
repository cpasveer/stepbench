"""Par volume: the yardstick for the material score.

Par is the volume of the leanest answer the generator can build for the
task that meets every requirement the task states -- stiffness included
where it is required -- and also keeps the minimum section (SECTION_MIN)
and the process's minimum wall.

The candidates are the level's own construction made lean: plates only as
thick as the bolts allow, members sized to 1.15 or 1.4 times the required
section, and (where the level offers it, ``PAR_CANDIDATES``) narrower or
wider trusses. They are tried from the smallest volume up; the first that
passes is par. The cheap checks run first, the stiffness analysis (minutes)
only on a candidate that has passed them.

Par must be honest: an answer that beats it by much more than about 1.5
times is a sign that par, or a requirement, is wrong. The grader flags such
an answer in the material score (``beats_par_by``). Par's own stiffness is
recorded (``stiffness_ratio``, figure / requirement). Where the minimum
section, not stiffness, is what limits the lean answer, that ratio is well
above 1 -- the leanest answer is then stiffer than it needs to be.
"""
from __future__ import annotations

import os
import tempfile
from types import SimpleNamespace

FACTORS = (1.15, 1.4)
MAX_STIFFNESS_TRIES = 5
SUSPICIOUS = 1.5  # an answer this much leaner than par is flagged
HOW = ("leanest of the generator's candidate answers that passes every required check{st}, keeps the "
       "minimum section and the minimum wall: {what}")


def candidates(task, level_mod) -> list[dict]:
    fn = getattr(level_mod, "PAR_CANDIDATES", None)
    if fn is not None:
        return fn(task)
    from .checks.section_min import required_area

    names = list(task.parts)
    need = max(required_area(task, a, b) for i, a in enumerate(names) for b in names[i + 1:])
    return [{"member_area": round(f * need, 3), "thin_plates": True} for f in FACTORS]


def compute(task, d: dict, build) -> dict | None:
    import importlib

    from .geometry import write_step
    from .grade import _wall_only, grade

    level_mod = importlib.import_module(build.__module__)
    stiff = "STIFFNESS" in task.tolerances.get("required_checks", [])
    built = []
    log = []  # why each candidate was not par: kept in the draw when none qualifies
    for kw in candidates(task, level_mod):
        try:
            part = build(task, dict(d), **kw)
        except Exception as e:  # noqa: BLE001 - a candidate the construction cannot make
            log.append({"build": kw, "why": f"not built: {str(e)[:80]}"})
            continue
        if len(part.solids()) != 1:
            log.append({"build": kw, "why": "not one solid"})
            continue
        part = part.solids()[0]
        built.append((float(part.volume), kw, part))
    built.sort(key=lambda b: b[0])
    tries = 0
    with tempfile.TemporaryDirectory() as tmp:
        for vol, kw, part in built:
            p = os.path.join(tmp, "lean.step")
            write_step(part, p)
            rep = grade(task, p, manufacturing=False, optional={"SECTION_MIN"}, score=False,
                        skip=frozenset({"STIFFNESS"}))
            sec = next((c for c in rep.checks if c.name == "SECTION_MIN"), None)
            if not rep.passed or sec is None or sec.ok is not True:
                log.append({"build": kw, "why": "fails " + ",".join(rep.failed() + ([] if sec and sec.ok else ["SECTION_MIN"]))})
                continue
            wall = _wall_only(task, SimpleNamespace(solid=part))
            if wall is None or not wall.ok:
                log.append({"build": kw, "why": "minimum wall"})
                continue
            margin = None
            if stiff:
                if tries >= MAX_STIFFNESS_TRIES:
                    log.append({"build": kw, "why": "stiffness tries used up"})
                    break
                tries += 1
                rep = grade(task, p, manufacturing=False, optional=False, score=False)
                st = next((c for c in rep.checks if c.name == "STIFFNESS"), None)
                if not rep.passed or st is None or st.ok is not True:
                    log.append({"build": kw, "why": "fails " + ",".join(rep.failed()),
                                "stiffness_ratio": st.value if st else None})
                    continue
                margin = st.value
            what = ", ".join(f"{k} {v:g}" if isinstance(v, (int, float)) and not isinstance(v, bool) else k
                             for k, v in kw.items() if v is not False)
            out = {"volume": round(vol, 3), "build": kw,
                   "how": HOW.format(st=" (stiffness included)" if stiff else "", what=what)}
            if "member_area" in kw:
                from .checks.section_min import required_area

                names = list(task.parts)
                need = max(required_area(task, a, b) for i, a in enumerate(names) for b in names[i + 1:])
                out["member_area_factor"] = round(kw["member_area"] / need, 3)
            if margin is not None:
                out["stiffness_ratio"] = margin  # weakest mount's figure / its requirement
            return out
    d["par_search"] = log
    return None
