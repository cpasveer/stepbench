"""Each joint as a task family (docs/families.md).

Each ``jointbench.families.<joint>`` package is three lines that call
``family_for``; the wrapping lives here, so the joints stay one package.

Levels, the same for every joint (a joint without a "rules not told" form
stops at level 2):

  level 1  rules told, joint frame = the world (axis along world z, at the origin)
  level 2  rules told, joint frame placed anywhere: positions to be worked out
  level 3  rules not told (only the standard is named), placed anywhere

A task holds the given parts as catalogue kind "solid" (each in the joint
frame, placed by the frame), the joint's numbers and brief in ``meta``, and
for the interference fit the intended overlap. The core checks that apply
are declared per family: no bolts, so no HOLES, SEATING, TOOL_ACCESS or
SECTION_MIN.
"""
from __future__ import annotations

import hashlib
import math
import tempfile

from . import common as C, get

# core checks that apply to a joint family (FILE_VALID always runs).
# MANUFACTURING only where the stated process exists in the checker.
CORE_CHECKS = {"NO_INTERFERENCE", "CLEAN"}
BUILT_PROCESSES = {"fdm", "moulding", "pbf"}

# clean modelling (the core's CLEAN rules) is required from this level on;
# generation rejects a draw whose reference is not clean
CLEAN_FROM = 2

# the process each joint is stated for; see docs/joints.md
PROCESS = {"revolute": "milling", "interference": "milling", "keyed": "milling", "spline": "milling",
           "involute": "milling", "kinematic": "milling", "snapfit": "moulding"}

# joints whose one-sided self-test rows (e.g. "bore given") hold for either side
SIDE_FREE = {"revolute"}

# joints whose brief has no "rules not told" form yet: two levels only
NO_LEVEL3 = {"snapfit", "involute"}

SITUATIONS = {
    "revolute": "A plate with a bore, or a base with a standing pin, is fixed in space, with a lug that limits\n"
                "the swing. A lever must turn on it.",
    "interference": "A hub, or a shaft on a flange, is fixed in space. The mating part is pressed onto it.",
    "keyed": "A shaft or a hub, with its keyway, is fixed in space. The mating part is keyed to it.",
    "spline": "A hub or a shaft with a straight-sided spline is fixed in space. The mating part slides onto it.",
    "involute": "A hub with an internal involute spline is fixed in space. A splined shaft slides into it.",
    "kinematic": "A base with three V-grooves is fixed in space. A part must sit on it on three balls,\n"
                 "held exactly.",
    "snapfit": "A plate with a window is fixed in space. A part must sit on it and hold itself with a\n"
               "cantilever hook.",
}
TITLES = {"revolute": "Design the lever on a pin joint", "interference": "Design the press-fitted part",
          "keyed": "Design the keyed part", "spline": "Design the splined part",
          "involute": "Design the involute-splined shaft", "kinematic": "Design the kinematically seated part",
          "snapfit": "Design the snap-fitted part"}
DESCRIPTIONS = {"revolute": "plate or base with the joint's bore or pin, and a lug",
                "interference": "the given side of the press fit", "keyed": "the given side, with its keyway",
                "spline": "the given side, with its splines", "involute": "hub with an internal involute spline",
                "kinematic": "base with three V-grooves", "snapfit": "plate with a window"}


def n_levels(name: str) -> int:
    return 2 if name in NO_LEVEL3 else 3


def level_settings(level: int) -> dict:
    return {1: {"rules_told": True, "aligned": True},
            2: {"rules_told": True, "aligned": False},
            3: {"rules_told": False, "aligned": False}}[level]


def core_checks(name: str) -> set:
    return CORE_CHECKS | ({"MANUFACTURING"} if PROCESS[name] in BUILT_PROCESSES else set())


def inner_seed(name: str, level: int, seed: int, attempt: int = 0) -> int:
    """The joint generator's seed, from the family's own string (docs/families.md)."""
    h = hashlib.sha256(f"{name}-L{level}-{seed}-{attempt}".encode()).hexdigest()
    return int(h[:8], 16)


def instance(task):
    """The joint instance a task was made from (regenerated: deterministic).

    Level 3 tasks do not carry the inner seed (nor the joint's numbers): it is
    worked out again from the family, level, seed and the draw's attempt."""
    j = task.meta["joint"]
    isd = j["inner_seed"] if "inner_seed" in j else inner_seed(j["joint"], task.level, task.seed, j["attempt"])
    return get(j["joint"]).generate(isd, **j["generate"])


def _intended(name, inst, part):
    """The interference fit's one intended overlap: a cylinder on the joint
    axis out to the far edge of the agent's zone, along the given fitted
    length. (FIT_OVERLAP keeps the exact ring; this is the core's view.)"""
    if name != "interference":
        return []
    from .interference import counterpart_z

    p = inst.p
    z0, z1 = counterpart_z(p)
    if p["given"] == "hub" and inst.rules_told:
        r = p["agent_zone"][1] / 2 + 1e-4
    elif p["given"] == "hub":
        # rules not told: the zone's edge would give the fit away, so a margin
        # well beyond any ISO 286 press fit (FIT_OVERLAP still keeps the ring)
        r = math.ceil((p["given_d"] / 2 + 0.002 * p["D"] + 0.02) * 100) / 100
    else:
        r = p["given_d"] / 2
    f = inst.frame
    return [{"part": part, "kind": "cylinder", "center": list(f.point((0.0, 0.0, (z0 + z1) / 2))),
             "axis": list(f.vec((0.0, 0.0, 1.0))), "radius": r, "length": z1 - z0}]


# what a level 3 task.json must not hold: the joint's worked-out numbers and
# the seed they can be drawn again from (the grader regenerates them)
HIDDEN_AT_L3 = ("params", "inner_seed")


def _task(name, level, seed, inst, gen_kw, knobs, isd, attempt):
    from ..catalogue import make_interface
    from ..task import DEFAULT_TOLERANCES, Task

    mod = get(name)
    parts, pose = {}, {}
    for k, solid in mod.fixed_local(inst).items():
        itf = make_interface("solid", {"step": f"given_{k}.local.step", "description": DESCRIPTIONS[name]})
        itf.solid = solid.solids()[0] if len(solid.solids()) == 1 else solid
        parts[k] = itf
        pose[k] = inst.frame
    tol = dict(DEFAULT_TOLERANCES)
    tol["manufacturing"] = dict(tol["manufacturing"], process=PROCESS[name])
    if level >= CLEAN_FROM:
        tol["required_checks"] = ["CLEAN"]
    meta = {"knobs": knobs, "brief": mod.brief(inst),
            "joint": {"joint": name, "attempt": attempt, "inner_seed": isd, "generate": gen_kw,
                      "params": inst.p, "frame": inst.frame.to_json()}}
    if not inst.rules_told:
        for k in HIDDEN_AT_L3:
            del meta["joint"][k]
    t = Task(f"{name}-L{level}-s{seed}", level, seed, parts, [pose], tol, meta, [], 1, None, name)
    t.intended_overlaps = _intended(name, inst, next(iter(parts)))
    return t


class Level:
    """A level module in the shape families.md asks for."""

    def __init__(self, name: str, level: int):
        self.name, self.LEVEL = name, level
        self.mod = get(name)
        self.SITUATION = SITUATIONS[name]
        self.KNOBS_DEFAULT = {k: None for k in getattr(self.mod, "KNOBS", {}) if k in ("given", "design", "retained")}
        # self-test rows carry over when they only change the part. The core
        # self-test draws the side of the joint from the seed, so a row written
        # for one side carries over only where it holds for either side
        # (SIDE_FREE); the joint's own self-test still runs every row.
        side_ok = set() if name not in SIDE_FREE else {"given", "design"}
        self.SELFTEST_VARIANTS = [(label, (lambda kw: (lambda task: dict(kw)))(v.get("part", {})), expect)
                                  for label, v, expect in self.mod.SELFTEST
                                  if not v.get("check") and set(v.get("knobs", {})) <= side_ok]

    def generate(self, seed, knobs=None):
        knobs = dict(self.KNOBS_DEFAULT, **(knobs or {}))
        for k, v in list(knobs.items()):  # knobs given on the command line arrive as text
            if isinstance(v, str):
                knobs[k] = {"true": True, "false": False}.get(v.lower(), v)
        gen_kw = dict(level_settings(self.LEVEL), **{k: v for k, v in knobs.items() if v is not None})
        for attempt in range(20):  # answer-first: the reference must pass, else the next draw
            isd = inner_seed(self.name, self.LEVEL, seed, attempt)
            inst = self.mod.generate(isd, **gen_kw)
            ref = C.to_world(self.mod.reference(inst), inst.frame)
            with tempfile.TemporaryDirectory() as tmp:
                sol = C.roundtrip(ref, tmp)
            bad = [r for r in self.mod.check(sol, inst) if r.ok is False and r.required]
            if not bad and self.LEVEL >= CLEAN_FROM:
                from ..quality import check_solid

                bad = [r for r in check_solid(sol).rules if not r.ok]
            if not bad:
                task = _task(self.name, self.LEVEL, seed, inst, gen_kw, knobs, isd, attempt)
                side = inst.p.get("given") or inst.p.get("design") or ""
                summary = f"{self.mod.TITLE}{', ' + str(side) + ' side' if side else ''}"
                return task, {"inner_seed": isd, "generate": gen_kw, "summary": summary}, sol
        raise RuntimeError(f"{self.name} level {self.LEVEL} seed {seed}: no draw whose reference passes")

    def build_connector(self, task, draw, **variant):
        inst = instance(task)
        return C.to_world(self.mod.reference(inst, **variant), inst.frame)


def check_module(name: str):
    """One check module running all of the joint's checks (a list, as the interface allows)."""
    mod = get(name)

    class _Check:
        NAME = name.upper()
        GROUP = "fits"

        @staticmethod
        def run(ctx):
            inst = instance(ctx.task)
            out = []
            for k in range(len(ctx.task.poses)):
                out += mod.check(ctx.solid_at(k), inst)
            return out
    return _Check


def brief(task) -> str:
    """README.md of a joint task."""
    from .. import NAME as CLI

    name = task.family
    lines = [f"# Task {task.id} -- {TITLES[name].lower()}", "",
             f"Level {task.level}. Units: millimetres.", "", SITUATIONS[name], "",
             task.meta["brief"], "",
             "The given parts are in `fixed_<name>.step` (world coordinates); `task.json` holds "
             + ("every number." if task.meta["joint"].get("params") is not None else
                "the placement. The standard's numbers are yours to know; the grader does not state them."),
             "Design **one solid part**. Any shape that meets the requirements passes; nothing is compared to a",
             f"reference. Save it as `output.step` in this folder, then run `uv run {CLI} verify <this folder>`."]
    if task.tolerances.get("required_checks"):
        lines += ["", "It must also be cleanly modelled: no sliver faces, tiny edges or steps, stubs or knife "
                      "edges (the CLEAN rules)."]
    return "\n".join(lines) + "\n"


# The submission.py a joint task should get in place of the connecting-part
# one (which speaks of bolts and reads POSE["mounting_faces"], absent here).
# Handed to write_workspace through Family(submission=).
SUBMISSION = '''"""Your answer to this task.

Write ONE solid to output.step (next to this file). README.md is the brief;
the given parts are in fixed_<name>.step, in world coordinates.

Only output.step is graded. You may replace this script entirely, or model
the part in any CAD program and save it as output.step instead.

    uv run python submission.py        # writes output.step
    uv run CLI_NAME verify .       # grades it
"""
import json
import pathlib

from build123d import Box, Part, export_step, import_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
GIVEN = {p["name"]: import_step(str(HERE / p["step"])) for p in TASK["fixed_parts"]}  # world coordinates


def build() -> Part:
    # Placeholder: a 10 mm cube. It will fail. Replace it with your part.
    return Box(10, 10, 10)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
'''


def submission(task) -> str:
    """submission.py of a joint task (the core's Family(submission=) hook)."""
    from .. import NAME as CLI

    return SUBMISSION.replace("CLI_NAME", CLI)


# ---------------------------------------------------------------- baselines


def baselines(name: str) -> dict:
    """Knowledge-free answers that set the floor: naive, and the level-1 answer applied blindly."""
    import build123d as bd

    mod = get(name)

    def naive(task):
        inst = instance(task)
        p = inst.p
        if name == "revolute":  # a plain pin at nominal size (or a plain bore), no arm
            if p["given"] == "hole":
                part = bd.Pos(0, 0, p["L"] / 2) * bd.Cylinder(p["d"] / 2, p["L"] + 4)
            else:
                part = bd.Pos(0, 0, p["L"] / 2) * (bd.Cylinder(p["d"] / 2 + 5, p["L"] - 1)
                                                    - bd.Cylinder(p["d"] / 2, p["L"]))
        elif name == "interference":  # nominal size, no lead-in
            if p["given"] == "hub":
                part = bd.Pos(0, 0, p["hub_len"] / 2) * bd.Cylinder(p["D"] / 2, p["hub_len"] + 6)
            else:
                part = bd.Pos(0, 0, p["hub_len"] / 2) * (bd.Cylinder(p["D"], p["hub_len"])
                                                          - bd.Cylinder(p["D"] / 2, p["hub_len"] + 2))
        elif name == "keyed":  # no keyway
            if p["design"] == "hub":
                part = bd.Pos(0, 0, p["hub_len"] / 2) * (bd.Cylinder(p["hub_od"] / 2, p["hub_len"])
                                                          - bd.Cylinder(p["d"] / 2 + 0.01, p["hub_len"] + 2))
            else:
                part = bd.Pos(0, 0, p["hub_len"] / 2) * bd.Cylinder(p["d"] / 2 - 0.01, p["hub_len"] + 20)
        elif name in ("spline", "involute"):  # a plain round part
            if name == "spline" and p["design"] == "hub":
                part = bd.Pos(0, 0, p["hub_len"] / 2) * (bd.Cylinder(p["hub_od"] / 2, p["hub_len"])
                                                          - bd.Cylinder(p["D"] / 2 + 0.5, p["hub_len"] + 2))
            else:
                r = (p["d"] / 2 if name == "spline" else p["df"][0] / 2) - 0.1
                part = bd.Pos(0, 0, p["hub_len"] / 2 + 5) * bd.Cylinder(r, p["hub_len"] + 10)
        elif name == "kinematic":  # a flat plate lying on the base: face contact
            part = bd.Pos(0, 0, 3) * bd.Cylinder(p["R"] + p["ball_d"], 6)
        elif name == "snapfit":  # a block on the plate, no hook
            part = bd.Pos(0, 0, 5) * bd.Box(2 * p["a"] + 20, 2 * p["wy"] + 16, 10)
        else:
            raise KeyError(name)
        return C.to_world(part, inst.frame)

    def template(task):
        """The reference as built in the joint frame, left at the world origin:
        a level-1 answer applied blindly. Passes level 1 by construction."""
        return mod.reference(instance(task))

    return {"naive": naive, "template": template}


def family_for(name: str):
    from ..families import Family

    mod = get(name)
    return Family(name=name, title=TITLES[name],
                  levels={lv: Level(name, lv) for lv in range(1, n_levels(name) + 1)},
                  checks=[(check_module(name), 1)], baselines=baselines(name), pose_answer=None,
                  core_checks=core_checks(name), brief=brief, doc=mod.TITLE, submission=submission)
