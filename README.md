# STEP.bench

*Version 1 is a proof of concept: levels 1 to 3, three setups, one seed,
four models and three CAD programs. Its results are published at
[stepbench.dev](https://stepbench.dev) as they come in. The command is
still called `jointbench`; it becomes `stepbench` after v1.*

STEP.bench measures, objectively, how capable an AI agent is at producing 3D
STEP models that can be used in a mechanical design environment.

The agent is given a set of fixed parts in space and a list of requirements.
It designs the part that connects them and hands it in as a STEP file. The
part is graded by exact measurement, never by a human or an AI judge.

The goal is a test you cannot pass on prior knowledge, nor solve by brute
force. Every task is generated from a random seed, so it exists nowhere
else and its answer is in no dataset. What is left is modelling something
unknown. The parts have no use in the real world, on purpose: the thesis is
that what it takes to produce them is what real engineering work needs. The
bench also shows which CAD tools agents work best in. People may take part
too. Every task is generated so that a good answer fits a small filament
(FDM) printer, so a result can be printed and held.

It is inspired by [ARC-AGI](https://arcprize.org): no brute force, no
memory.

Results, with every graded part in 3D, are published at
[stepbench.dev](https://stepbench.dev).

## Principles

- **Never seen.** Every task is generated from a seed. Nothing has to be
  remembered or looked up: the brief states every position, size and
  tolerance in millimetres.
- **Solvable.** Every task is generated backwards from a reference part,
  which the same grader passes before the task is kept. A fail is therefore
  the agent's, not the task's. In ranking runs the participant never sees
  the reference; on public tasks it is in the task folder for practice (see
  *Run it*).
- **No brute force.** In a recorded run an agent may check its part against
  the grader at most **6 times** per task. The grader refuses a seventh, and
  it never reveals a solution.
- **Exact measurement.** The STEP file is measured on its exact geometry:
  holes, seating, clearances, wall thickness, overhangs, stiffness by finite
  elements, motion without collision. Every check states its limit and the
  number it measured. The same file always gets the same verdict.
- **Any shape that works passes.** A part is never compared with the
  reference.

## One ladder, six levels

Each level keeps every requirement of the levels below and adds one. The
added requirement is rarely hard on its own; the difficulty is meeting it
together with the others. Material added for strength can create an
overhang that will not print; a split that lets a part pass through an
opening can break the path the load takes.

| level | adds | what the grader checks |
|---|---|---|
| L1 **Fits** | a valid part | one clean, closed solid; every bolt goes in; the part sits flat on each mount, inside its outline; nothing collides; every bolt can be turned with its hex key |
| L2 **Printable** | printed in PLA without support | overhangs, wall thickness, gaps, bridges, a stable side to stand on; every printed part judged on its own |
| L3 **Strong** | a stated load | stiffness under the load by finite element analysis, in PLA |
| L4 **Assembles** | two parts joined inside a box | both parts pass through one small opening without collision, each along a short path of straight moves and turns, are bolted together inside with bolts the model picks from the ISO 4762 range, and the joint carries the load |
| L5 **Locks** | no tool reaches inside | the model's parts join by push-and-turn: they go in, turn, cannot be pulled apart, and carry the load |
| L6 **Moves** | a mechanism | inside the box a linkage the model designs, pivoting on catalogue ball bearings or bolt axles, carries a part through the required positions into a latch that locks it and holds the load |

Within one seed, the fixed parts are the same at every level, so the ladder
shows how an agent's result changes as requirements are added. Printed
parts must print from L2 on and be strong from L3 on; bought parts (bolts,
bearings) are taken as they are. Answers with more than one part carry a
parts list, printed and bought, so they can be rebuilt.

## Version 1

Three setups:

| setup | fixed parts |
|---|---|
| **facing** | two flat plates, parallel and facing each other, offset sideways |
| **flange** | a round flange and a block at an angle to it |
| **pipe** | a round pipe and a flat square plate at an angle to it |

Version 1, the proof of concept, is levels 1 to 3 on the three setups:
**9 tests**, each on one task (seed 1). Four models (Claude Fable 5.2,
Opus 5.5, Sonnet 5.5 and Haiku 5.5) take every test in three CAD programs
(build123d, FreeCAD and Rhino 9): 108 runs. Levels 4 to 6 are built in part
and are not part of version 1's claims.

| level | facing | flange | pipe |
|---|---|---|---|
| L1 Fits | v1 | v1 | v1 |
| L2 Printable | v1 | v1 | v1 |
| L3 Strong | v1 | v1 | v1 |
| L4 Assembles | later | later | later |
| L5 Locks | later | later | later |
| L6 Moves | later | later | later |

A test counts as built when its generator makes tasks whose reference passes
the grader. `gen` refuses a test that is not built yet.

## Score

In version 1 a run passes or fails: it passes when its last check meets
every requirement of its level. For every run the site also shows how many
checks it used (at most six), how long it took, and its tokens and price,
computed from the recorded input, output and cache tokens at the provider's
published list prices.

A weight-based score for load tests (the lightest passing part scores 1.00,
a part twice as heavy 0.50), several seeds per test, and a 95 % range per
model are planned for later rounds.

## Public and private seeds

- **Showcase seeds** are public: fixed tasks for each setup and level, to
  practise on and to compare results openly.
- **Ranking seeds** are private: the tasks of a round are drawn freely
  within each setup (other sizes, angles and orientations) and are known to
  nobody in advance. They are published after each round, so every result
  can be reproduced.

Ranking runs are made by the maintainers. See
[docs/participate.md](docs/participate.md).

## Run it

You need [uv](https://docs.astral.sh/uv/) and Python 3.12 or later.

```
git clone https://github.com/cpasveer/stepbench.git && cd stepbench
uv sync --extra stiffness
```

**Generate a task.** `uv run jointbench ladder` lists which tests exist.

```
uv run jointbench gen --setup pipe --level 3 --seed 0 --out tasks
```

`--setup` is `facing`, `flange` or `pipe`; `--level` is 1 to 6, of which
only the built tests are generated. The task folder holds `README.md` (the
brief), `task.json` (every number, the seed included), the fixed parts as
STEP files, an optional build123d starting script `submission.py`, and
**`_solution/`, the hidden reference** that proves the task solvable.

> **Delete `_solution/` before an agent works on the task.** It holds the
> reference answer (`reference.step`, `reference.json`) for practice and for
> checking the grader. With the seed in `task.json` and the public
> generator, the reference can also be regenerated, so a public task is
> practice, not proof. In ranking runs the participant's copy has neither.

Generating a level 3 task and grading a part each take about a minute.

**Model the part** in any CAD program or library that writes STEP, and save
it in the task folder as `output.step`.

**Check it.**

```
uv run jointbench verify tasks/<task> --tool <the CAD tool you used>
```

Exit code 0 is a pass, 1 a fail. Every check prints its reason and its
number, and is archived with the exact file under `tasks/<task>/attempts/`.

**How precise the strength check is.** The finite element analysis runs in
two stages. It first uses coarse elements (4 mm), good to about +-15 %. If
the part is more than 15 % clear of the requirement, either way, that
verdict stands and the report prints `+-15%`. Only a part inside that band
is analysed again with fine elements (2 mm), good to +-2 %, and the report
then prints `+-2%`. A result within 2 % of the limit counts as a pass and
is flagged as marginal. So the decision near the limit is always made at
+-2 %, as the brief says; the number printed is the precision of the stage
that decided.
To hold yourself or your agent to six checks:

```
export JOINTBENCH_MAX_CHECKS=6
```

**Look at it.** The local viewer shows the task, your part and every check
in 3D, and grades a STEP you drop on the page:

```
uv run jointbench play tasks
```

## Results from others

Every ranking result is made by the maintainers. Submitting your own
results is not open yet; it will open later.

## Tools

Agents may use any software, including tools they build themselves, and any
capability of the model, such as image input. No viewer is provided to the
agent: how it looks at its own part is part of the test. Each run records
the model, the CAD program and its version, its add-ons and the connector.
Models are compared with the CAD tool held constant, and tools with the
model held constant.

## More

- [docs/participate.md](docs/participate.md): sets, tracks, how ranking runs
  are made.
- [docs/agent-brief.md](docs/agent-brief.md): what an agent is told, and
  which rules are enforced.
- [docs/environment.md](docs/environment.md): the record of what a run was
  made with.
- [docs/design.md](docs/design.md): how the generator and grader are built.
- [docs/dfm.md](docs/dfm.md), [docs/quality.md](docs/quality.md),
  [docs/stiffness.md](docs/stiffness.md): the printing rules, the
  clean-modelling rules, the load analysis.

## Limits

- The parts have no practical use; that the skills transfer to real design
  work is assumed, not shown.
- Version 1 covers one material and one process: PLA, filament printing.
- An agent is tested together with its tooling; a weak connection between
  model and CAD program lowers the result.
- Version 1 runs each test once per model and program, on one seed: one
  run says little on its own, and small differences between models are
  not significant.

## Credit

The manufacturing criteria were informed by the
[FreeCAD DFM Workbench](https://github.com/ryankembrey/FreeCAD-DFM-Workbench)
(LGPL-2.1). Its list of rule ideas was read; no code was taken from it, and
the limits come from published printing guides, each cited in
`src/jointbench/dfm/processes/`.

## Licence

Code: Apache License 2.0 (`LICENSE`, with attributions in `NOTICE`).
Tasks, results and the text of the site: Creative Commons Attribution 4.0
(CC BY 4.0).
