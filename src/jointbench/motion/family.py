"""The planar mechanisms as task families (docs/families.md).

Each ``jointbench.families.<mechanism>`` package is three lines calling
``family_for``; the wrapping lives here so the motion package stays one.

How a mechanism maps onto the interface:

* **Fixed parts** are every part left in the mechanism (frame, housing,
  obstacles, and the moving neighbours), each a catalogue ``solid`` in its
  own frame.
* **Poses** are the rows of the motion table (every 10 deg of input): at
  pose k each fixed part sits where the working mechanism puts it.
* **pose_answer** measures the answer's joints, solves the mechanism with
  it in closed form, and returns where the answer itself sits at pose k. A
  jam is raised as the reason, so ``POSES`` fails with it.
* **pose_scene** returns every fixed part where the answer's own motion
  puts it, so the core sees the scene the answer makes.
* **Core checks**: ``NO_INTERFERENCE`` only: at each table row, the answer
  against the fixed parts and the fixed parts against each other.
* **Family checks** (computed once, inside the first ``pose_answer``):
  ``JOINTS``, ``ASSEMBLES``, ``TRANSMISSION``, ``OUTPUT`` and
  ``MOTION_CLEAR`` -- every part against every other, every 1 deg, each
  where the *answer's* motion puts it (docs/motion.md).

Levels:

  level 1  joints' positions given, mechanism plane = world XY
  level 2  placed anywhere; where an obstacle can bite (a link), one obstacle
  level 3  placed anywhere; the output point is on the missing part (it must
           be placed: path synthesis) where the mechanism allows it, and the
           positions of parts that depend on it withheld
"""
from __future__ import annotations

import math

from ..geometry import Placement
from . import generate, kin, selftest, taskio
from .grade import grade_solid
from .kin import Mechanism

TITLES = {"fourbar": "Design the missing part of a four-bar linkage",
          "slider": "Design the missing part of a crank-slider",
          "yoke": "Design the yoke of a Scotch yoke",
          "oldham": "Design the middle disc of an Oldham coupling"}
SITUATIONS = {
    "fourbar": "A four-bar linkage in a housing has lost one part. A crank turns on the frame; the replacement\n"
               "must make the mechanism move as the table says, through a full turn, touching nothing.",
    "slider": "A crank-slider in a housing has lost one part. The crank turns a full circle; the replacement\n"
              "must make the slider (or a marked point) move as the table says, touching nothing.",
    "yoke": "A Scotch yoke in a housing has lost its yoke. The crank pin must drive it along its guides so\n"
            "that a marked point moves as the table says, touching nothing.",
    "oldham": "An Oldham coupling between two offset shafts has lost its middle disc. The output hub must\n"
              "turn with the input as the table says, touching nothing.",
}
# knobs per level; a mechanism without a part an obstacle can bite on skips it
LEVEL_KNOBS = {
    "fourbar": {1: {"remove": "coupler", "placed": "flat"},
                2: {"remove": "coupler", "obstacles": 1},
                3: {"remove": "coupler", "output_on_removed": True, "obstacles": 1}},
    "slider": {1: {"remove": "rod", "placed": "flat"},
               2: {"remove": "rod", "obstacles": 1},
               3: {"remove": "rod", "output_on_removed": True, "obstacles": 1}},
    "yoke": {1: {"remove": "yoke", "placed": "flat"}, 2: {"remove": "yoke"}},
    "oldham": {1: {"remove": "disc", "placed": "flat"}, 2: {"remove": "disc"}},
}
CORE_CHECKS = {"NO_INTERFERENCE"}
CLEAR_NAME = "MOTION_CLEAR"


def _placement_3d(pose) -> Placement:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return Placement((pose[0], pose[1], 0.0), ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


# ---------------------------------------------------------------- task


def _public(mtask: dict, m: Mechanism) -> dict:
    """What the agent may see: only what the brief states. The joint graph,
    the features of every part, the generator's record and the motion of the
    parts that depend on the missing one stay with the grader."""
    rem = mtask["removed"]
    o, inp = m.output, m.input
    return {
        "type": mtask["type"],
        "removed": rem,
        "removed_label": m.parts[rem].label,
        "frame": m.frame.to_json(),
        "range": list(m.range),
        "input": {"part": inp["part"], "label": m.parts[inp["part"]].label,
                  "measured": "groove direction" if "line" in inp else "pivot to pin"},
        "output": {"kind": o["kind"], "part": o["part"], "label": m.parts[o["part"]].label,
                   "on_removed_part": o["part"] == rem},
        "assembly": taskio._mode_text(m, rem),
        "table": {k: mtask["table"][k] for k in ("inputs", "kind", "world", "plane")},
        "fits": mtask["fits"],
        "tolerances": mtask["tolerances"],
        "step_deg": mtask["step_deg"],
        "joints_of_removed": mtask["joints_of_removed"],
        "parked": mtask.get("parked", []),
        # which draw was kept: with seed and knobs, the grader rebuilds the rest
        "attempt": mtask["generation"]["attempt"],
    }


def _grader_data(mtask: dict) -> dict:
    """Everything the grader needs: the motion task as generated (minus the
    full mechanism, which holds the answer, and minus timings)."""
    g = {k: v for k, v in mtask.items() if not k.startswith("_")}
    g["generation"] = {k: v for k, v in g.get("generation", {}).items() if "seconds" not in k}
    return g


def _task(name, level, seed, mtask, m):
    from ..catalogue import make_interface
    from ..task import DEFAULT_TOLERANCES, Task

    removed = mtask["removed"]
    solids = m.meta["solids"]
    hidden = Mechanism.from_json(mtask["mechanism"])
    parts = {}
    for pid in m.parts:
        if pid == removed:
            continue
        itf = make_interface("solid", {"step": f"given_{pid}.local.step",
                                       "description": m.parts[pid].label or pid})
        itf.solid = solids[pid]
        parts[pid] = itf
    # one stored pose: the drawing (fixed_<part>.step), parked parts parked.
    # The checks run at every table row (check_poses); pose_scene places the
    # parts there, so their motion is never written where the agent reads it
    first, jam = kin.sweep(m, [mtask["table"]["inputs"][0]])
    assert jam is None
    pose0 = {pid: kin.pose_to_placement(m.frame, first[0].poses.get(pid, m.parts[pid].shown)) for pid in parts}
    for pid in mtask.get("parked", []):
        pose0[pid] = kin.pose_to_placement(m.frame, hidden.parts[pid].shown)
    tol = dict(DEFAULT_TOLERANCES)
    tol["manufacturing"] = dict(tol["manufacturing"], process="milling")
    meta = {"knobs": mtask["knobs"], "motion": _public(mtask, m)}
    return Task(f"{name}-L{level}-s{seed}", level, seed, parts, [pose0], tol, meta, [],
                generate.GENERATOR_VERSION, None, name)


_CACHE: dict = {}  # (family, seed, knobs) -> full mechanism
_GRADER: dict = {}  # (family, seed, knobs) -> grader data


def _key(knobs) -> str:
    import json
    return json.dumps(knobs, sort_keys=True)


def _tkey(name, task):
    return (name, task.seed, _key(task.meta["knobs"]))


def _regenerate(name, task):
    mtask, m, _ = generate.make(name, task.seed, task.meta["knobs"], replay=task.meta["motion"].get("attempt"),
                                version=task.generator_version)
    _CACHE[_tkey(name, task)] = m
    _GRADER[_tkey(name, task)] = _grader_data(mtask)


def _mechanism(name, task):
    """The full mechanism a task was made from (regenerated if not in memory: deterministic)."""
    if _tkey(name, task) not in _CACHE:
        _regenerate(name, task)
    return _CACHE[_tkey(name, task)]


def grader_data(task) -> dict:
    """The grader's view of a task: from memory, else rebuilt from seed, knobs
    and the accepted attempt (deterministic; the draws are replayed, not
    graded). Nothing is read from the task folder beyond task.json, so a
    copied, redacted or tampered folder cannot change what the grader knows."""
    k = _tkey(task.family, task)
    if k not in _GRADER:
        mt = task.meta["motion"]
        if "mechanism" in mt:  # a folder written before the grader data left task.json
            _GRADER[k] = mt
        else:
            _regenerate(task.family, task)
    return _GRADER[k]


class Level:
    """A level module in the shape docs/families.md asks for."""

    def __init__(self, name: str, level: int):
        self.name, self.LEVEL = name, level
        self.SITUATION = SITUATIONS[name]
        self.KNOBS_DEFAULT = dict(LEVEL_KNOBS[name][level])
        if level == 3:
            self.KNOBS_DEFAULT["pivots"] = "withheld"
        variants = [("reference", lambda t: {}, None),
                    ("reshaped, same joints", lambda t: {"variant": "reshaped"}, None),
                    ("one joint 0.05 mm out", lambda t: {"variant": "shift", "d": 0.05}, None),
                    ("one joint 2 mm out", lambda t: {"variant": "shift", "d": 2.0}, "OUTPUT"),
                    ("a bump that fouls at one angle", lambda t: {"variant": "fouls"}, CLEAR_NAME),
                    ("a length that jams", lambda t: {"variant": "jams"}, "POSES")]
        if self.KNOBS_DEFAULT.get("obstacles"):
            variants.append(("the straight link, through the obstacle", lambda t: {"variant": "straight"},
                             CLEAR_NAME))
            variants.append(("the bent link mirrored, bend on the wrong side", lambda t: {"variant": "mirrored"},
                             CLEAR_NAME))
        if name == "yoke":
            # GUIDED at its limit, two guide blocks: a centred bar just long
            # enough to stay in both, and one 1 mm short of the far block
            variants.append(("a bar just inside both guide blocks", lambda t: {"variant": "guided", "d": 1.0}, None))
            variants.append(("a bar that leaves one guide block", lambda t: {"variant": "guided", "d": -1.0},
                             "GUIDED"))
        if name == "oldham":
            # GUIDED at its limit, one continuous guide: bearing 1.5 x width +- 1 mm
            variants.append(("a tongue bearing 1.5 x its width + 1 mm", lambda t: {"variant": "guided", "d": 1.0},
                             None))
            variants.append(("a tongue bearing 1.5 x its width - 1 mm", lambda t: {"variant": "guided", "d": -1.0},
                             "GUIDED"))
        self.SELFTEST_VARIANTS = variants

    def generate(self, seed, knobs=None):
        knobs = dict(self.KNOBS_DEFAULT, **(knobs or {}))
        for k, v in list(knobs.items()):  # knobs given on the command line arrive as text
            if isinstance(v, str) and v.lower() in ("true", "false"):
                knobs[k] = v.lower() == "true"
        # the motion generator seeds from "motion-<type>-<seed>-<knobs>"; the
        # level's knobs are part of that string, so each level is its own stream
        mtask, m, ref = generate.make(self.name, seed, knobs)
        task = _task(self.name, self.LEVEL, seed, mtask, m)
        _CACHE[(self.name, seed, _key(knobs))] = m
        _GRADER[(self.name, seed, _key(knobs))] = _grader_data(mtask)
        draw = {"summary": f"{m.parts[mtask['removed']].label} removed", "knobs": knobs,
                "attempt": mtask["generation"]["attempt"]}
        return task, draw, ref

    def build_connector(self, task, draw, variant=None, d=0.0):
        """The reference, or one of the self-test's variants of it."""
        m = _mechanism(self.name, task)
        rem = task.meta["motion"]["removed"]
        solids = m.meta["solids"]
        if variant is None:
            solid = solids[rem]
        elif variant == "reshaped":
            solid = selftest.reshaped(m, rem)
        elif variant == "shift":
            solid, _ = selftest.shifted(m, rem, d)
        elif variant == "fouls":
            solid, _ = selftest.fouls(m, rem, solids)
        elif variant == "jams":
            solid = selftest.jams(m, rem, grader_data(task))
        elif variant == "straight":
            solid = selftest.straight(m, rem)
        elif variant == "mirrored":
            solid = selftest.mirrored(m, rem)
        elif variant == "guided":
            solid = selftest.guided(m, rem, d)
        else:
            raise ValueError(variant)
        return solid.moved(kin.pose_to_placement(m.frame, m.parts[rem].shown).location())


# ---------------------------------------------------------------- grading


def _hidden(task) -> Mechanism:
    return Mechanism.from_json(grader_data(task)["mechanism"])


def _report(task, ctx):
    """Run the motion grader once per grading and keep it on the context."""
    rep = getattr(ctx, "_motion_report", None)
    if rep is None:
        rep = grade_solid(grader_data(task), _hidden(task), ctx.solid, keep_solutions=True)
        ctx._motion_report = rep
    return rep


def pose_answer(task, ctx, k):
    rep = _report(task, ctx)
    sols = rep.keep
    if sols is None:
        bad = next((c for c in rep.checks if c.ok is False), None)
        raise RuntimeError(bad.reason if bad is not None else "the answer could not be posed")
    m = _hidden(task)
    F = m.frame
    Finv = kin.frame_inverse(F)
    rem = task.meta["motion"]["removed"]
    return Finv.then(_placement_3d(sols[k].poses[rem])).then(F)


def pose_scene(task, ctx, k):
    """Every fixed part at pose k as the answer actually drives it: the core
    NO_INTERFERENCE then tests the real scene, fixed parts against each other too."""
    rep = _report(task, ctx)
    if rep.keep is None:
        bad = next((c for c in rep.checks if c.ok is False), None)
        raise RuntimeError(bad.reason if bad is not None else "the answer could not be posed")
    F = _hidden(task).frame
    poses = rep.keep[k].poses
    return {pid: kin.pose_to_placement(F, poses[pid]) for pid in task.parts}


class _MotionChecks:
    NAME = "MOTION"
    GROUP = "fits"

    @staticmethod
    def run(ctx):
        rep = _report(ctx.task, ctx)
        out = []
        for c in rep.checks:
            if c.name == "NO_INTERFERENCE":
                c.name = CLEAR_NAME
            out.append(c)
        return out


def brief(task) -> str:
    from .. import NAME as CLI

    mt = grader_data(task)
    m = _hidden(task)
    text = taskio.brief(mt, m)
    text = text.replace("Grade with `python -m jointbench.motion verify <this folder>`.",
                        f"Save it as `output.step` in this folder, then run `uv run {CLI} verify <this folder>`.")
    head = (f"Task {task.id}, level {task.level}. Units: millimetres.\n\n{SITUATIONS[task.family]}\n\n"
            "Checks: `POSES` (the part can be posed at every table row), `JOINTS`, `ASSEMBLES`, "
            "`TRANSMISSION`, `GUIDED` (sliding joints only), `OUTPUT`, `MOTION_CLEAR` (no two parts touch, every 1 deg, as your part "
            "makes them move) and the core `NO_INTERFERENCE` (at each table row, your part and the fixed "
            "parts as your part makes them move, against each other). `fixed_<part>.step` shows each part "
            "as drawn at the first input value; `task.json` holds the same numbers as this brief.\n\n")
    return text.replace("\n\n", "\n\n" + head, 1)


def baselines(name: str) -> dict:
    """The floor: a straight link through the joint positions given in the
    brief (no output mark, no bend), and the level-1 answer applied blindly."""

    def straight(task):
        """A plain bar through the joint positions the brief gives (none withheld
        are guessed): no output mark, no bend. Built from task.json alone."""
        import build123d as bd

        from .mechs import BOSS, HOLE, W
        mt = task.meta["motion"]
        F = Placement.from_json(mt["frame"])
        pts, z = [], None
        for g in mt["joints_of_removed"].values():
            if g["feature"] in ("pin", "hole") and "plane_xy" in g:
                pts.append(tuple(g["plane_xy"]))
                z = g["z_band"] if z is None else z
        if len(pts) < 2 or z is None:
            pts = (pts or [(0.0, 0.0)]) + [((pts or [(0.0, 0.0)])[0][0] + 50.0, (pts or [(0.0, 0.0)])[0][1])]
            z = z or [7.0, 13.0]
        z0, z1 = (z[0], z[1]) if z[1] - z[0] <= 7 else (z[0] + 7, z[0] + 13)
        (x1, y1), (x2, y2) = pts[0], pts[1]
        L = math.hypot(x2 - x1, y2 - y1)
        face = (bd.Pos(x1, y1) * bd.Circle(BOSS / 2) + bd.Pos(x2, y2) * bd.Circle(BOSS / 2)
                + bd.Pos((x1 + x2) / 2, (y1 + y2) / 2) * bd.Rot(0, 0, math.degrees(math.atan2(y2 - y1, x2 - x1)))
                * bd.Rectangle(L, W))
        bar = bd.Pos(0, 0, z0) * bd.extrude(face, z1 - z0, dir=(0, 0, 1))
        for x, y in pts[:2]:
            bar = bar - bd.Pos(x, y, z0 - 1) * bd.Cylinder(HOLE / 2, z1 - z0 + 2, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
        return bar.solids()[0].moved(F.location())

    def template(task):
        """The level-1 reference of the same seed, placed as level 1 placed it."""
        knobs = dict(LEVEL_KNOBS[name][1])
        mtask, m, ref = generate.make(name, task.seed, knobs, version=task.generator_version)
        return ref

    return {"straight": straight, "template": template}


SUBMISSION = '''"""Your answer to this task: the missing part of a mechanism.

Write ONE solid to output.step (next to this file), modelled where the part
sits at the first input value, in world coordinates. Read README.md for the
brief; task.json holds the same numbers (meta.motion: the mechanism plane,
the motion table, the joints your part must make, fits and tolerances).

Only output.step is graded. You may replace this script entirely, or model
the part in any CAD program and save it as output.step instead.

    uv run python submission.py        # writes output.step
    uv run CLI_NAME verify .           # grades it
"""
import json
import pathlib

from build123d import Box, Location, Part, Plane, export_step

HERE = pathlib.Path(__file__).parent
MOTION = json.loads((HERE / "task.json").read_text())["meta"]["motion"]
F = MOTION["frame"]  # mechanism plane: origin and rotation (columns: plane x, plane y, normal)
ROT = F["rotation"]
PLANE = Plane(origin=tuple(F["origin"]), x_dir=tuple(r[0] for r in ROT), z_dir=tuple(r[2] for r in ROT))
JOINTS = MOTION["joints_of_removed"]  # what your part must mate with, as far as the brief tells
TABLE = MOTION["table"]  # input degrees -> required output (world point or angle)


def build() -> Part:
    # Placeholder: a 10 mm cube on the plane origin. It will fail. Replace it.
    # Work in plane coordinates (u, v, w) and place the result with PLANE.
    return PLANE.location * Box(10, 10, 10)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
'''


def submission(task) -> str:
    from .. import NAME as CLI
    return SUBMISSION.replace("CLI_NAME", CLI)


# ---------------------------------------------------------------- viewer


def motion_poses(task, values=None, answer=None) -> list:
    """Every pose of a mechanism task, for a viewer: one dict per input value,
    part id -> Placement, the same kind task.poses holds.

    Without ``answer``: the reference motion, solved in closed form from the
    mechanism rebuilt from seed, knobs and attempt. Fixed parts are placed as in
    task.poses (their local solids); the removed part's entry places the
    reference part's local solid, ``reference_solid(task)``. ``values`` are the
    input values (degrees or mm, as the table); default the table's inputs.

    With ``answer`` (a STEP path or a solid in world, as submitted): the motion
    that answer drives, at the table's inputs only (``values`` is ignored). The
    removed part's entry then moves the submitted solid from where it was
    modelled. Raises RuntimeError with the grader's reason if it cannot be posed."""
    rem = task.meta["motion"]["removed"]
    if answer is None:
        m = _mechanism(task.family, task)
        vals = list(task.meta["motion"]["table"]["inputs"] if values is None else values)
        sols, jam = kin.sweep(m, vals)
        if jam is not None:
            raise RuntimeError(f"the reference jams at {jam.value}")
        return [{pid: kin.pose_to_placement(m.frame, s.poses.get(pid, m.parts[pid].shown))
                 for pid in m.parts} for s in sols]
    from types import SimpleNamespace

    if isinstance(answer, str):
        from ..geometry import read_step
        answer = read_step(answer)
    ctx = SimpleNamespace(solid=answer)
    out = []
    for k in range(len(task.meta["motion"]["table"]["inputs"])):
        pose = pose_scene(task, ctx, k)
        pose[rem] = pose_answer(task, ctx, k)
        out.append(pose)
    return out


def reference_solid(task):
    """The reference part's solid in its own coordinates (for motion_poses without an answer)."""
    return _mechanism(task.family, task).meta["solids"][task.meta["motion"]["removed"]]


def family_for(name: str):
    from ..families import Family

    return Family(name=name, title=TITLES[name],
                  levels={lv: Level(name, lv) for lv in LEVEL_KNOBS[name]},
                  checks=[(_MotionChecks, 1)], baselines=baselines(name), pose_answer=pose_answer,
                  pose_scene=pose_scene, check_poses=lambda task: len(task.meta["motion"]["table"]["inputs"]),
                  core_checks=CORE_CHECKS, brief=brief, doc="docs/motion.md",
                  # the core's grader_data(task) falls through to this: nothing is
                  # written to _solution for it, so it is always rebuilt from
                  # seed, knobs and attempt (an edited folder cannot steer it)
                  grader=grader_data, submission=submission)
