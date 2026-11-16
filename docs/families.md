# Task families: how to add one

A *task family* is one kind of design problem: the connecting part is the
first; revolute joints, interference fits, keyed and spline joints,
kinematic couplings, snap fits, flexures, and moving mechanisms (four-bar,
Scotch yoke, Oldham coupling, Geneva drive, ratchet and pawl, leaf spring)
are meant to follow, built in parallel by different sessions. This page is
the boundary they build against. Follow it and every tool picks the family
up without an edit to a core file: `gen`, `verify`, `selftest`, `solve`,
`baselines`, `freeze`, the attempt archive, `traces`, and the website (which
reads task folders and attempt logs, not code).

## The rules every family keeps

- **Answer-first.** Build a valid answer first, then place the fixed parts
  so it fits. Grade that answer with the real grader; if it fails any
  requirement the task switches on, throw the draw away. Every published
  task is solvable by construction.
- **Graded on measurement, never resemblance.** Checks measure the
  submitted solid (a STEP file) on exact geometry. Any answer that meets
  the requirements passes. Never compare to the reference shape.
- **Deterministic.** Same seed and knobs, byte-identical task folder. Seed
  your random stream from a string that includes the family and level
  (`f"{family}-L{level}-{seed}"`), never from time; store numbers at full
  precision; write STEP with `jointbench.geometry.write_step`.
- **Everything the agent needs is in the brief.** Every position, size,
  tolerance and limit, in plain words and in `task.json`.
- **Plain reasons.** Each check result is a name, pass or fail, a one-line
  reason with the number it turned on, and where (a point, a bolt, an
  obstacle) when that helps.

## What a family provides

A family is a sub-package of `src/jointbench/families/`, e.g.
`src/jointbench/families/revolute/__init__.py`, defining:

```python
from jointbench.families import Family
from . import level1, level2          # your level modules

FAMILY = Family(
    name="revolute",                  # unique; also the prefix of your task ids
    title="Design the pin joint",
    levels={1: level1, 2: level2},
    checks=[(axis_check, 1), (play_check, 2)],   # extra checks and their first level
    baselines={"naive": naive_answer},           # name -> function(task) -> solid
    pose_answer=None,                 # or a function for moving mechanisms (below)
    core_checks={"NO_INTERFERENCE"},  # which core checks apply (None = all; see below)
    brief=my_brief,                   # function(task) -> README text (None = the connecting-part brief)
)
```

**A level module** provides:

| name | what |
|---|---|
| `LEVEL` | the level number |
| `SITUATION` | the brief's opening paragraph: the situation in plain words |
| `KNOBS_DEFAULT` (optional) | the difficulty knobs and their defaults; recorded in `task.meta.knobs` |
| `generate(seed, knobs=None)` | returns `(task, draw, reference_solid)`, answer-first, deterministic |
| `build_connector(task, draw, **variant)` | the reference and its deliberately broken copies (the name is historical: it builds the answer, whatever the family) |
| `SELFTEST_VARIANTS` | `[(label, lambda task: variant_kwargs, expectation)]`; the expectation is `None` (must pass), `"CHECK"` (must fail that check), or `("ok", "CHECK")` (must pass and that check must hold) |

The task you return is a `jointbench.task.Task` with `family="revolute"`,
an id starting with the family name (`revolute-L1-s0`), its `poses`,
`tolerances` (including `required_checks` for any reported-only check the
level makes required), `meta.knobs`, `generator_version`, and a `par` from
`jointbench.par.compute` if material is scored.

**A check module** (one per check) provides `NAME`, `run(ctx)` returning a
`CheckResult` (or a list of them, one per rule), and optionally
`GROUP` -- the requirement it belongs to, one of `fits`, `clean`,
`makeable`, `sturdy` (default `fits`) -- and `OPTIONAL = True` if it is
reported unless a task lists it in `tolerances["required_checks"]`. Inside
`run`, loop over `range(len(ctx.task.poses))` and use `ctx.solid_at(k)` for
the answer and `ctx.fixed(k)` / `ctx.obstacles(k)` for the scene at pose
`k`. The core checks (`FILE_VALID`, `HOLES`, `SEATING`, `NO_INTERFERENCE`,
`CLEAR_PATH`, `TOOL_ACCESS`, `CLEAN`, `SECTION_MIN`, the manufacturing
rules) always run first; your checks follow.

**Baselines** are scripted, knowledge-free or knowledge-limited answers
built from `task.json` alone. They set the difficulty floor: a level is too
easy if a baseline passes it. Keep at least one naive baseline and one that
applies the previous level's answer blindly.

## Grader-only data and the submission stub

Data the grader needs but the agent must not see (a mechanism's exact
table, a hidden tolerance) does not go in `task.json`, which states only
what the brief states. Return it in your draw as `draw["grader"]`;
`write_workspace` stores the draw in `_solution/reference.json`, which
travels with the grader and never with the agent (a frozen set's
fingerprint covers it; an agent's copy of a task is the folder without
`_solution`). Read it in your checks with `jointbench.families.grader_data(task)`. If
your family supplies `grader=fn(task)` -- recommended -- that hook is used
and must rebuild the data deterministically from the task's seed and
knobs; the file is then never read, so editing a folder cannot make the
grader lenient. Only a family without the hook falls back to the file.

`submission=fn(task) -> str` gives your family's own `submission.py`
starting point (the default is the connecting-part stub).

## How a task is judged

Everything except material is a requirement with a pass or a fail:
*fits*, *clean*, *makeable*, *sturdy*. A task switches the reported-only
ones on through `tolerances["required_checks"]`. A part passes when it
meets every requirement switched on; among passing parts, the one score is
material: `par / volume`. The verdict JSON carries this as `verdict` with
`requirements` per group and `score`. Do not invent another score; if your
family needs one, raise it with the project first.

## Motion

Motion is expressed by the `poses` list in `task.json`: each pose gives a
placement for every fixed part (and its own obstacle list). For a static
family with several poses, the grader runs every still check at every pose
with the answer where it is.

For a moving mechanism the answer itself moves. Supply

```python
def pose_answer(task, ctx, k):
    """Placement of the answer at pose k, or None if it cannot get there."""
```

It is called once per pose, after `FILE_VALID`, as the check `POSES`. It may
measure the answer (for example find its pin holes with
`jointbench.checks.holes.measure` on `ctx.solid`) and solve the mechanism's
closure for pose `k` -- a four-bar's coupler position for crank angle k,
say -- returning a `jointbench.geometry.Placement`. If it returns None for
any pose, `POSES` fails with that pose named and no other check runs. If
every pose is reached, every check then sees `ctx.solid_at(k)`: the answer
as placed at pose `k`. So a family must supply, for the grader to compute
poses: the fixed parts' placements per pose in `task.json`, and the
function that places the answer from its measured geometry.

Two more hooks for mechanisms whose neighbours move with the answer:

```python
def pose_scene(task, ctx, k):
    """{part name: Placement} for every fixed part at check pose k, as the
    answer actually drives them (overrides task.json's stored placements)."""

def check_poses(task):
    """How many poses the checks run at -- may be finer than the poses
    task.json stores (e.g. 360 one-degree steps against 36 stored rows)."""
```

With `pose_scene`, `ctx.fixed(k)` is the scene the answer makes, and
`NO_INTERFERENCE` also tests every pair of fixed parts against each other
at every pose (a wrong answer can drive a crank into a rocker). With
`check_poses`, every check loops over `ctx.poses()`; `pose_answer` and
`pose_scene` are called with check-pose indices; bolts, faces and obstacles
come from the nearest stored pose (`ctx.stored(k)`). Interference
between the answer and the fixed parts at every pose, clearances and tool
access then come for free from the core checks.

## Checklist before you hand over

- `uv run jointbench gen --family <name> --level 1 --n 5 -j 0` writes five
  folders; running it twice gives identical files.
- `uv run jointbench selftest --family <name> --level 1 --n 20 -j 0` shows
  every row "as expected", including at least one broken variant per check
  you added.
- `uv run jointbench baselines --family <name> --level 1 --n 10 -j 0` gives
  the floor; record it in your section of the README.
- A test file `tests/test_family_<name>.py` with determinism, one pass and
  one fail case per check, and the self-test rows.
- Your files only: your family package, its tests, your README section.
  Commit through a private git index (`GIT_INDEX_FILE=...`), never
  `git add -A`.
