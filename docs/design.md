# STEP.bench: how it is built

STEP.bench tests how well an agent models a 3D part that exists nowhere,
for a real situation, with nothing to look up.

The situation is one designers meet every day: **design the connecting
part**. The agent is given fixed parts in space, each with a flat mounting
face and a real bolt pattern, and must model one solid that bolts to all of
them.

Every task is generated from a seed, so its answer is in no dataset. The
brief states every position, size and tolerance, so nothing has to be
remembered or looked up. What is left to test is modelling and spatial
reasoning -- the idea behind ARC-AGI, applied to CAD. Difficulty comes from
the situation, not from a written description of the shape: in later
levels, parts at angles, obstacles in the way, and neighbours that move.

The answer is a STEP file. It is graded by measurement on exact geometry
(OpenCASCADE), never by resemblance to a reference shape and never by a judge
model. Any shape that works passes.

Three levels are built. Level 1: two parts, faces parallel and facing each
other, nothing in the way. Level 2: the faces are at an angle and offset
sideways, and the whole scene has an arbitrary position and orientation, so
nothing is axis-aligned. Level 3: two or three parts, obstacles placed
exactly where a straight connection would run, and every bolt must stay
reachable by a straight tool. A planned optional run mode, *rules not told*, withholds the
grading rules (clearance holes, seating, bolt-head room) so that engineering
knowledge is tested separately from modelling.

```
uv sync
uv run jointbench gen --seed 0 --n 20 --level 1 --out tasks
uv run jointbench gen --seed 0 --n 20 --level 2 --out tasks
uv run jointbench gen --seed 0 --n 5 --level 3 --out tasks
uv run jointbench gen --seed 0 --n 5 --level 3 --knob parts=2 --knob obstacles=2 --out tasks
uv run jointbench verify tasks/L2-s0            # grades tasks/L2-s0/output.step
uv run jointbench verify tasks/L2-s0 --json     # same, machine-readable
uv run jointbench selftest --level 2 --n 20 -j 0  # proof the grader can be trusted (all cores)
uv run jointbench solve -q tasks/L2-s*                       # naive baseline
uv run jointbench solve -q --baseline template tasks/L2-s*  # level-1 answer, applied blindly
uv run jointbench baselines --n 10                         # every baseline, levels 1-3
uv run jointbench traces tasks --out traces.jsonl          # attempts as training records
uv run jointbench freeze sets/trial --level 1:4 --seed 1000  # a frozen task set (see below)
uv run pytest
```

## Why it is built this way

**Graded on function, not likeness.** A connector is right if every bolt
goes in, the part sits flat where it is clamped, and nothing collides. Those
are facts about geometry that can be measured exactly, so that is all the
grader measures. A differently shaped body with the same interfaces passes;
that is the definition of correct, not a loophole.

**Answer-first generation.** Each task is made backwards: first a valid
connector is built, then the fixed parts are placed where it fits them. At
level 1 the connector is two plates, one on each face, joined by a wall
beside the bolts. At level 2 it is a plate on each face with a tab on the
side facing the other part, and a bridge between the two tabs (the convex
hull of the two tab rectangles), all built in each face's own frame from
what `task.json` says. The reference is then graded by the same grader an
agent faces, together with a second body whose plates are 1 mm thicker. If either fails, the draw is thrown away and
the next one from the same seeded stream is used. So every task is solvable,
the grader has passed real answers for it before anyone sees it, and no task
hangs on a 1 mm knife edge (an early version had one: a shaft ending just
above where a bolt head would sit).

**Generic mounts, real fasteners.** Fixed parts are generic mounting
interfaces described by shape, not named products: a flat square, rectangular
or round mount, each with a parametric bolt pattern (a square of 4, a grid
of 2-6, or 3-6 on a bolt circle). About 40% of round and square mounts carry
a plain centre spigot standing 2-8 mm proud of the face, so something sticks
into the answer's space; from level 3 on, most of what is in the way comes
from obstacles. Bolts are ISO 4762 socket head cap screws M3 to M8;
clearance holes are the ISO 273 range from the fine to the coarse series.

**Generator versions.** Every `task.json` says which generator made it
(`generator_version`). Version 1 also used a NEMA 17 motor face and 2- and
4-hole flanges; version 2 uses only the generic mounts; version 3 has the
corrected `SECTION_MIN` and leaner par, which changes par and, at level 3,
which draws are admitted. The same seed gives a different task under a
different version; frozen sets keep scores comparable. Version-1 folders still load
and grade unchanged: their interfaces stay in the catalogue, and checks added
later are only reported for them, never required.

**The grader reads only the STEP.** It does not run or read the agent's
code, so the same grader scores a build123d script, a FreeCAD macro, or a
person clicking in any CAD program. This is tested: `examples/freecad_connector.py`
builds connectors with FreeCAD's own code from `task.json` alone, a design the
generator never sees. They pass 15 of 20 level 1 tasks and 17 of 20 level 2
tasks; every failure is a real clash between that simple script's bridge and
a bolt head's clearance space, reported as such.

**Reproducible.** A seed fixes a task completely. Generating the same seeds
twice gives byte-identical folders (the STEP writer's timestamp and an
internal counter are normalised; nothing else varies). Grading the same file
twice gives the same verdict and the same numbers.

**Headless.** Everything is a command-line call with an exit code and a
`--json` option, so it can sit inside a training loop.

## The task an agent sees

```
tasks/L1-s0/
  README.md        the brief: what to connect, bolt positions and axes, tolerances
  task.json        the same, machine-readable (the source of truth)
  fixed_A.step     fixed parts, world coordinates
  fixed_B.step
  submission.py    build123d starting point that writes output.step
  _solution/       the reference answer -- not for the agent
  attempts.jsonl   index: one line per graded attempt
  attempts/0001/   each attempt kept: the graded STEP, its script, the feedback
```

`task.json` describes each fixed part as a catalogue entry in its own frame
plus a placement (origin and rotation matrix), and holds a `poses` list. At level 1 that list has one
pose. Motion later means more poses; the file format and every check already
loop over the list. Each pose also carries the resolved world positions of
faces and bolts, at full precision, so an agent does not have to do the
transforms and lands exactly on the geometry the grader uses.

## The checks

```
PASS  (4/4 checks)  L1-s0
  ok   FILE_VALID       1 closed valid solid, 22 faces
  ok   HOLES            8 holes on axis within 0.000 mm (tol 0.1), dia 3.40-4.50, grip 6.0-6.0 mm
  ok   SEATING          flat on every mounting face; worst bolt ring 100% in contact (need 75%)
  ok   NO_INTERFERENCE  no overlap with fixed parts or bolt space (worst 0.0000 mm3, limit 0.01)
  info volume 137490.6 mm3, bounding box 116.4 x 101.3 x 25.0 mm
```

| check | passes when |
|---|---|
| `FILE_VALID` | the file holds exactly one solid, valid by OpenCASCADE's checker, with a closed skin |
| `HOLES` | for every bolt, a true cylindrical hole whose axis is within 0.1 mm of the bolt axis, diameter in the fine..coarse clearance range, and grip (material under the head) between a minimum and 25 mm |
| `SEATING` | for every bolt, a flat face of the part lies on the mounting face (every point standing off at most 0.05 mm; up to 0.001 mm into it is accepted as rounding) and touches at least 75% of the ring from the hole out to the head diameter |
| `NO_INTERFERENCE` | zero overlap volume (below 0.01 mm³) between the part and each fixed part, and between each bolt's keep-out (shank, plus head with 0.5 mm all round and 1 mm above) and both the part and the fixed parts |
| `CLEAR_PATH` (level 3) | zero overlap with every obstacle; the obstacle hit is named |
| `TOOL_ACCESS` (level 3) | above every head, along the bolt axis, a cylinder of the head diameter and 30 mm long is free of the part, the fixed parts and the obstacles: a straight hex key or driver can reach |
| `SECTION_MIN` | the smallest flat cut that separates one mount's bolts from another's keeps at least 2 x the combined shank area of the weaker mount's bolts (see below) |
| `Q_*` (`CLEAN`) | the modelling-quality checker in `src/jointbench/quality/`: no sliver faces, tiny faces or edges, hairline steps, thin wedges, stubs, ... |
| `FDM_*` / `IM_*` (manufacturing) | the rules of the stated process from the checker in `src/jointbench/dfm/`: minimum wall, overhang, bridges, holes, ... |

## How a part is judged

Everything except material is a requirement with a pass or a fail; material
use is the one score.

| requirement | checks | where it is required |
|---|---|---|
| it fits | `FILE_VALID`, `HOLES`, `SEATING`, `NO_INTERFERENCE`; at level 3 also `CLEAR_PATH`, `TOOL_ACCESS` | always |
| it is sturdy enough | `SECTION_MIN`; `STIFFNESS` as the stronger form | `SECTION_MIN` required from level 3; `STIFFNESS` where the level 3 knob `stiffness` is set |
| it is cleanly modelled | `CLEAN` (`Q_*` rules) | reported at every level (see limits) |
| it can be made | the stated process's rules: filament printing (`FDM_*`, default) or powder bed fusion (`PBF_*`, level 3 knob `process=pbf`) | reported (see limits) |

Which requirements decide the verdict is recorded per task in
`tolerances.required_checks`, so a level -- or one experiment -- can switch
them on. Reported-only checks run once a part fits, appear as `warn`, and
never change the verdict. Every task stays solvable by construction with
whatever it switches on: the reference must pass all of it, or the draw is
rejected.

**The score: material.** Among parts that pass, the score is par volume /
part volume, so 1.0 is par and higher is leaner. Par is the volume of a
deliberately lean answer the generator builds for each task: its own
construction with the plates only as thick as the bolts allow and the
connecting member sized to 1.15 x the required section (enlarged to 1.4, 1.8,
2.5 x if that lean answer does not pass). It is a par, not an optimum;
ratios above 1 are possible and welcome. A part counts for the score only if
it also keeps `SECTION_MIN` and the process's minimum wall (the manufacturing
checker's wall rule), so a paper-thin part cannot win. Stiffness and
strength are not judged: the leanest legal part may still be flimsy.

The verdict JSON carries it in this shape:

```
"verdict": {"passed": true,
            "requirements": {"fits":   {"required": true,  "result": "pass", "failed": []},
                             "sturdy": {"required": true,  "result": "pass", "failed": []},
                             "clean":  {"required": false, "result": "fail", "failed": ["Q_STUB"]}},
            "score": {"material": 1.08, "counts": true, ...}}
```

Headline per model: share of tasks passed (first try, and within the
allowed tries); second figure: median material ratio over its passed tasks.
Traces carry `passed_first_try`, `passed_within_allowed` and `material` per
task; the website's `score` command does the per-model summary.

**How `SECTION_MIN` finds the smallest cut.** Two kinds of flat cut are
measured for every pair of mounts, and the smaller counts.

- *Whole-plane cuts*: planes that put all of one mount's bolts on one side
  and all of the other's on the other. Directions tried: 26 spread over the
  sphere, every cylinder axis, the 12 largest face normals, every straight
  edge of 5 mm or more, and the line between the mounts.
- *Single-member cuts*: planes square to the part's own members (cylinder
  axes, long straight edges, face normals), stepped across the whole part.
  Each separate region of the section (one rod, one arm) is tested on a
  triangle mesh -- cutting the surface along that region's outline, do the
  two mounts end up on different pieces? -- and the smallest that does is
  confirmed exactly by cutting a 0.02 mm wafer out of the solid there. This
  is what finds a rod or arm whose square cut does not separate all bolts
  by one plane, and the arm to one of three mounts.

Planes are stepped 2 mm, then refined at 0.25 mm; candidates are ranked on
the mesh and every reported area is measured exactly on the solid.
Measured precision: members built at 0.9 and 1.1 x the requirement read
0.90 and 1.10 (level 2) and 0.92 / 1.12 (level 3), so the check resolves a
10% band. What it cannot see: a weak spot no flat cut isolates (a twisted
neck), and a connection that is weak only when two members are cut together
where no single plane separates the mounts. About 1-4 s per pair of mounts.

**`STIFFNESS`** (sturdy, where a task switches it on): the stiffness
checker in `src/jointbench/stiffness/` holds one mount and loads the others
by unit forces and moments; the weakest way of loading must reach the
task's stated figure, a fraction (the level 3 knob `stiffness`) of the
reference answer's own figure, written into `task.json` at generation. A
result within the checker's precision is "marginal" and counts as a pass.
Tens of seconds to minutes per part, so it runs only where required or
asked for.

Each check reports pass/fail, a one-line reason and the number it turned on.
A failure names the bolt and the amount, e.g.
`A1: hole axis 0.250 mm off the bolt axis (tol 0.1)` or
`part overlaps fixed part A: 1192.234 mm3 overlap`. Volume and
bounding box are reported, not ranked. Exit code is 0 only if all pass.

**Pass, fail, or error.** Sometimes the geometry kernel cannot measure a
valid part: the manufacturing checker raises its `MeasurementError`
(e.g. "pbf, direction-free rules: Standard_Failure"). That says nothing
about the part, so it is not a failure. The check reads `error`, its
requirement "not measured". If the verdict rests on that check (it is
required, and nothing required failed on its own), the outcome is `error`
and `counts_as_attempt` is false. `verify` exits 3, and the grading is kept
under `errors/NNNN` with its own `errors.jsonl`, never in `attempts.jsonl`.
Scores and traces read only `attempts.jsonl`, so an error never costs the
agent a try. A real failure elsewhere
still decides the verdict: then the outcome is `fail`. An error in a check
that is only reported leaves the outcome alone and is listed under
`verdict.errors`. The stiffness check follows the same rule when its
mesher fails. If the quality checker finds the model clean, the fault is the tool's:
error. If it finds it untidy, the fault is the part's: fail, with the quality faults
as the reason. Any other crash inside a check is still a failure, with
its reason.

## Every attempt is kept

An agent overwrites `output.step` on every try, so `verify` keeps a copy.
Each graded attempt gets a folder, and `attempts.jsonl` stays the index,
one line per attempt pointing to its folder:

```
attempts/0001/
  output.step     the exact file that was graded
  submission.py   the script, if one is linked to the attempt (see below)
  feedback.txt    the grader's output exactly as it was printed (text or JSON)
  result.json     the full record: verdict, every check, and the fields below
  note.txt        free text from the caller, stored as given (optional)
```

The script is linked one of three ways, and the record says which:
`ran` (`verify --build` ran it), `named` (`--script <file>`), or `found`
(an edited `submission.py` sat in the task folder; that is not proof it made
the file). The untouched stub is never archived, and `--no-script` archives
none. That matters when a person models in another CAD program in a folder
where an agent's script is still lying around.

Two kinds of field are recorded, and kept apart:

| measured by the grader | reported by the caller, stored as given |
|---|---|
| verdict; each check with reason and number | `--tool`: CAD tool used (build123d, cadquery, openscad, freecad, anvil, other) |
| the grader's timestamp (ms), so time between attempts and first-to-pass follow | `--wall-seconds` for the whole task |
| SHA-256 of the graded file | `--tokens-in`, `--tokens-out` |
| `step_header`: what the STEP file says about the program that wrote it (originating system, preprocessor, timestamp, author), verbatim, never graded on | `--cost`, `--currency` |
| volume and bounding box | `--model`, `--model-version`, `--tool-calls` |
| | `--note-file`: e.g. the agent's reasoning |
| | `--reported-file`: a JSON file with any of the above |

A run may also carry `run.json` in its folder (`<runs>/<run>/run.json`) with
the same figures for the whole run. Missing values stay missing; nothing is
ever estimated. The STEP header is a useful cross-check on `--tool`
(FreeCAD writes `FreeCAD`, build123d writes `build123d`), but a program can
write anything there, so it is evidence, not proof.

Older logs, from before the archive, stay readable: their lines simply have
no `archive`.

### Traces: attempts as training records

`jointbench traces <folder> --out traces.jsonl` writes one JSON line per
task and submitter. The folder may be one task, a folder of tasks, or a runs
folder. Nothing is re-graded. A record:

```
trace_format_version   1
task_id, level, seed   which task
by                     who submitted
run                    run.json of the run folder, or null
brief                  the task README the submitter was given, verbatim
task                   task.json, verbatim
attempts[]             in order, each with:
  n, time                     grader's attempt number and timestamp
  seconds_since_previous      from the grader's timestamps
  code, code_link             the archived script (or null) and how it is linked
  feedback, feedback_format   the grader's output exactly as the submitter saw it
  feedback_reconstructed      true only for old log lines, rebuilt from stored checks
  passed, failed_checks, checks, info
  sha256, step_header         measured
  tool, reported, note        reported by the caller, as given
summary                n_attempts, passed, first_pass_attempt,
                       seconds_first_attempt_to_pass, tools
```

The sequence code, feedback, revised code, ..., pass is the training
example.

**What a trace does not contain.** The agent's own tool calls, its
intermediate files and its thinking are not seen by the grader and are not
in the record, unless the caller passes them as a note. Geometry is referred
to by hash and archive path, not embedded. The reference answer is never
included.

## Frozen task sets

A seed fixes a task only for one generator version; when the generator
changes (as it did when the named motor mount was replaced by generic
mounts), the same seed gives a different task. To compare scores across
models and dates, tasks are frozen into a **set**: written once as files and
used as files, never regenerated.

```
uv run jointbench freeze sets/trial --name trial --version 1 --level 1:4 --level 2:4 \
    --level 3:2 --knob 3:parts=2 --knob 3:obstacles=2 --seed 1000
uv run jointbench freeze --check sets/trial
```

The set folder holds the task folders and `manifest.json`: the set's name,
version and visibility, and per task its id, level, seed, knobs, generator
version and a fingerprint (the SHA-256 of every file, and one hash over
all). Each task folder also gets `set.json`, so `verify`, the attempt archive
and `traces` record which set and version a task belongs to (`task_set`).
`--check` reports any task whose files changed. A set is never overwritten.

How public, hidden and training tasks are kept apart:

- **Public set** -- committed and published; scores on it are comparable.
- **Hidden set** -- same sizes and knobs, seeds derived from a secret
  (`--hidden`, secret in `JOINTBENCH_HIDDEN_SECRET`, seeds above 10 million),
  kept outside the repository (`sets/hidden*` is ignored by git) and never
  published: a check that a model did not train on the public set.
- **Training tasks** -- generated freshly from any other seeds;
  `freeze --avoid <set>` refuses seeds a set already uses.

No set is frozen yet: "set 1" waits until the levels are final. The
rule that a per-model summary must not mix sets belongs in the website's
`score` command, which reads `task_set` from the archive.

## Proof the grader can be trusted

`jointbench selftest` grades, for every generated task, the reference
answer, a differently shaped body, and broken copies of the reference, each
through a real STEP file. Measured on all 12 cores of a laptop (`-j 0`),
just before the last generator change (members built to a stated section
now keep it along slanted segments; the pass row was then at 1.1 x, now
1.5 x); not yet re-run since:

| level | tasks | time | rows as expected |
|---|---|---|---|
| 1 | 20 | 76 s | 12 of 13 rows on every task; "member at 1.1 x" 19/20 (see below) |
| 2 | 20 | 117 s | 10 of 14 rows on every task; rod rows 18-19/20 (see below) |
| 3 | 6 | 1010 s (generation dominates) | 13 of 15 rows on every task, including both new checks: a spur into an obstacle fails `CLEAR_PATH` 6/6, a bar over a head fails `TOOL_ACCESS` 6/6 |

Every hole, seating, interference, obstacle and tool-access row behaves on
every task. The rows that do not are test variants that are not what their
label says on some tasks, and each case was traced:

- *member at 1.1 / 0.9 x the required section*: where the variant's own
  plate, a spigot clearance cutting the rod, or a slanted arm segment is
  thinner than the member, `SECTION_MIN` correctly finds that (L1-s3,
  L2-s13, L2-s14); where a level 3 arm runs through another mount's plate,
  that mount has a second load path and no single member is a cut (L3-s4).
  The pass row now uses 1.5 x; members built to a stated section now keep
  it along slanted segments, which also makes par leaner.
- *0.4 mm fin*: on one level 3 task the fin lands in a tool path.

What `SECTION_MIN` resolves, measured on members whose weakest section is
the member itself: 0.90 / 1.10 / 1.50 x read 0.90 / 1.10 / 1.50 (level 3)
and 0.90 / 1.10 (level 2); a 10% band is resolved.

Before the speed work a golden set of 69 answers (every self-test variant
on five tasks, levels 1-3) was recorded; afterwards no verdict changed.
Only `SECTION_MIN` values changed, where the new single-member cuts find a
thinner section than before.

## The difficulty floor

Scripted baselines read the task and never think; each writes
`baseline_<name>.step` in the task folder. *naive*: one drilled block
filling the space. *template*: the level-1 answer applied blindly. *hull*
and *rod*: the level-2 answer applied blindly (plates joined by one convex
hull, or by straight rods to a hub). Measured with `jointbench baselines`
(passes; for passing parts the median material ratio, and how many count):

| who | level 1 (10) | level 2 (10) | level 3 (4) |
|---|---|---|---|
| reference | 10/10, 0.46 (par on 9; 9 count) | 10/10, 0.37 (par on 5; 3 count) | 4/4, 0.64 (4 count) |
| naive | 0/10 | 0/10 | 0/4 |
| template | 10/10, 0.56 (9 count) | 0/10 | 0/4 |
| hull | 0/10 | 7/10, 0.42 (3 count) | 0/4 |
| rod | 2/10, 0.90 (0 count) | 5/10, 1.09 (1 count) | 0/4 |

Level 3 defeats every scripted answer.

*Development history.* Early versions of the benchmark were passed by every
model tried, with parts no engineer would accept: a block with holes,
plates joined by a stick, a rod nearly cut through. That is why a part must
now be makeable, cleanly modelled and stiff enough, not only fit. Model
results will be reported again once a frozen set exists under the current
rules.

## Generator version 4: parts that can be made

A part that cannot be made fails, at every level. Each task states its
process (`knobs.process`: `pbf` powder bed fusion by default, `fdm`
filament printing as the strict setting) and its rules are required checks.
The references were rebuilt so they pass them: plates joined by round rods
at most 14 mm across, each leaving its plate square to the face, with a
sphere at every bend (`src/jointbench/levels/rods.py`). That leaves no acute
inside corner, no solid section thicker than the rod, no abrupt change of
thickness, and no thin overlaps -- which is also what the quality checker
(`CLEAN`) asks for. The generator grades every reference against its process
and rejects the draw if it fails, so each task stays solvable by
construction. Self-test on 6 tasks each (`-j 0`, measured 2026-11-12):
level 2 every row as expected (93 s), level 3 every row as expected
(488 s); level 1 every row except one variant, fixed after its run and
confirmed by hand (not re-run as a table). Filament references are not
built yet (round rods lying sideways overhang; a teardrop profile is
needed).

Level 3 now plans with exact distances instead of solids (arms as line
segments with a radius, obstacles as boxes, tool paths as segments; solids
only for the final reference). Generating 20 tasks at 2 parts / 2 obstacles
on 12 cores: 18 of 20 produced within 400 draws, about 21 s per task
overall, most of it the powder check on the reference and on par's lean
answer (about 10 s per part). Before: about 226 s per task (same 20 seeds,
same machine). **Three parts do not generate yet**: three bundles of tool
paths crowd the one hub where the rods meet, so the level 3 default is now
2 parts / 2 obstacles; `--knob parts=3` stays available but rarely finds a
task. A three-part reference needs a different shape (the third arm joining
the first two part-way, not a shared hub).

The stiffness requirement, where switched on (level 3 knob `stiffness=1`),
now comes from the task alone: a design load from the weaker mount's bolt
area over a deflection of span / 250, per loaded mount, stated in N/mm in
the brief and `task.json` (`jointbench.requirements.stiffness_requirement`
is the one place that sets it; `requirements.par` likewise for par).

## Generator version 5: stiffness as a requirement, truss references

Stiffness can be required at levels 1 and 2 (knob `stiffness=1`; at level 3
it already was). For powder tasks it is **off by default** for now: the
benchmark's connecting part is moving to filament printing first (see
below), and the powder version is parked as it stands here.

The requirement comes from the task alone: at each loaded mount, a design
load of 1.2 N per mm2 of the weaker mount's bolt stress area may move that
mount by at most span / 250, in whichever direction the part is weakest.
The brief states it in plain words per mount (the load, where it acts, the
allowed movement, the required N/mm). The grader analyses the part with
coarse elements first and runs the full analysis only near the limit.

**Why the powder references changed.** Powder printing caps a solid
section at 15 mm, and a member's bending stiffness grows with the fourth
power of its size, so no single rod is stiff enough: the version 4
references reached 29-68% of the requirement. Stiffness has to come from
spreading material apart. The references are now trusses
(`src/jointbench/levels/truss.py`): four round chords set apart, each
leaving its plate square to its face, braced by thinner diagonals where a
run is long, with a sphere where a chord turns. Level 1 puts the chords
round the bolts; level 2 runs them from a tab beside the bolts, up a short
column, across, and down into the other tab, the chords mitred at each bend
so the cross-section stays square. Measured with stiffness switched on:
level 1 references 4.7-11 x the requirement, level 2 2.0 x.

**Rules the truss had to respect, each measured.** Rods meeting at under
about 65 degrees leave a wedge of air beside the joint close to powder
printing's 0.5 mm minimum gap (63 deg: 0.37 mm; 70 deg: 0.56-1.06 mm), so
rods meet at 70 degrees or more and diagonals are thinner than chords.
Diagonals that pass close to a rod they do not join are dropped, and
columns are raised until no rod comes near a plate. A draw whose reference
still fails a rule is rejected, as before. (A 1 mm blend at every joint also
opens the wedge, but the kernel takes many minutes to blend a whole truss,
so it is off.)

**Par** is the leanest of the generator's candidate answers that passes
every required check, stiffness included when it is required. The
candidates are tried in order of volume; the reason each one failed is
kept in the draw when none qualifies. Measured with stiffness on:
level 1 par 2.8-4.0 x the requirement (the minimum section, not stiffness,
limits the lean answer there, so it cannot be brought down to 1.2 x);
level 2 par 1.55 x (plates thinned to the bolts' minimum grip bend too much
at level 2, 0.70-0.75 x, so par keeps the reference's plates). An answer
that beats par by more than 1.5 x is flagged in the material score
(`beats_par_by`).

**Self-test rows for stiffness.** Each level has four: the lean answer par
was taken from must pass, and three shapes models built in the November
round must fail STIFFNESS: two plates joined by one slender bar, the same
with a ring plate, and a single rod on thin spokes (measured 8-58% of the
requirement). They are built for every task (`levels/weak.py`) and read
"n/a" on tasks that do not require stiffness.

**Cost, and why powder stiffness is off by default.** One stiffness
analysis takes 30 s to several minutes per part, growing with its volume.
With stiffness on, a level 1 or 2 task took 4-15 minutes to generate on one
core (reference, then par's candidates). That is too slow to develop the
benchmark with; the filament version is to size its reference by formula
and confirm with a single analysis.

## Speed and options

Grading an answer on the fit checks takes about 0.2-0.5 s. By default
`verify` computes only what decides the pass plus the material score (which
needs `SECTION_MIN` and the wall rule); a reported-only group runs only on
request: `--report all` or `--report clean,makeable,sturdy`;
`--skip-reported` computes nothing that is not required (fastest).
`gen`, `selftest`, `baselines` and `freeze` take `--jobs N` (`-j 0` = all
cores); results come back in seed order, so files and tables are the same
as with one process. `task.json` states at the top which requirements
count (`requirements`) and the difficulty settings (`knobs`), and the brief
repeats both.

## Task families

New kinds of task -- other joints, moving mechanisms -- are built as
families against the interface in `docs/families.md`; the connecting part
is the first family (`connect`). A family is a folder under
`src/jointbench/families/` and is picked up by every command without edits
to core files; motion is expressed by the `poses` list plus a function that
places the answer at each pose.

## Where later levels attach

The level, the interface catalogue and the checks are separate modules, so
each next step adds a module rather than changing one.

| next step | where it attaches |
|---|---|
| more interfaces | one builder function in `catalogue.py` |
| level 2: faces at an angle and offset | built: `levels/level2.py`; it needed no new check, because the checks already work on any axis and plane |
| level 3: three parts, obstacles | generator places obstacles; a `CLEAR_PATH` check (part vs obstacle volume) in `checks/` |
| level 3: tool access at each bolt | a `TOOL_ACCESS` check: a straight hex-key or driver envelope along each bolt axis must be free; same machinery as the head keep-out |
| level 4: 3D-printable | a `PRINTABLE` check on the solid: overhang angle, minimum wall, minimum hole, bed contact. The project will write its own rule checker for this rather than depend on an outside tool such as the FreeCAD DFM Workbench, so the rules stay explicit, versioned and headless |
| motion | more entries in `task.json` `poses`; every check already loops over poses. First for mechanisms moving in parallel planes, later for motion in space |
| run modes (rules told / not told / checker available) | a workspace option deciding what the brief contains and whether `verify` is offered |

`checks/__init__.py` lists the checks with the first level each applies to.

## Module map

| file | role |
|---|---|
| `catalogue.py` | bolts (ISO 4762, ISO 273), interfaces, fixed-part solids |
| `geometry.py` | placements and rotations; convex hull; STEP export with normalised header |
| `task.py` | the task format, poses, resolved world bolts |
| `levels/level1.py`, `levels/level2.py` | per level: answer-first generator, reference connector, selftest variants, the brief's description of the situation |
| `checks/` | `FILE_VALID`, `HOLES`, `SEATING`, `NO_INTERFERENCE` |
| `grade.py` | runs the checks and builds the report |
| `archive.py` | keeps every attempt: archive folder, index line, STEP header, reported figures |
| `traces.py` | exports attempt sequences as training records |
| `workspace.py` | writes a task folder and its brief |
| `selftest.py` | the trust table |
| `baseline.py` | the naive block and the blind level-1 template |
| `cli.py` | `gen` / `verify` / `selftest` / `solve` / `traces` (plus the website's `play` / `site` / `score`) |

## Prior art

This builds on, and does not copy, recent CAD benchmarks that already moved
from "does it look right" to "does it work":

- **CAD-bench** ([paper](https://cdn.jsdelivr.net/gh/CAD-bench/cad-bench@main/paper/main.pdf)) --
  an execution-based benchmark of CAD-generating agents in difficulty tiers,
  focused on functional failure modes.
- **CADClamp** ([code](https://github.com/ChristianNHill/cadclamp)) -- AI-generated
  parametric CAD scored on whether the part can actually be printed;
  supports build123d, OpenSCAD, CadQuery and FreeCAD.
- **CADEngBench** ([arXiv 2608.09296](https://arxiv.org/pdf/2608.09296)) --
  parametric design, functional editing and assembly reasoning, with physics
  simulation.
- **CADGenBench** ([code](https://github.com/huggingface/cadgenbench)) --
  drawing-to-CAD and STEP editing; submissions are STEP files, scored on
  geometry, topology, interface compatibility and validity.
- **FunCADBench** -- agents build mechanisms that must work in physical
  simulation, across mechanism families.

What STEP.bench adds:

- **unlimited generated tasks** from a seed, each proven solvable by
  construction, rather than a fixed curated set;
- **the connect-given-parts task**: the answer is not described anywhere, it
  is implied by the parts it must join, so many shapes are correct;
- **human-playable against the same grader**: the graded artefact is a
  plain STEP file, so a person in any CAD program and an agent writing code
  are scored by identical measurement.

## Known limits

- **References are not clean and not printable.** The generator's references
  join pieces with 0.5 mm overlaps (needed for a reliable union in the
  kernel), which the quality checker rightly flags as stubs and slivers, and
  routed arms at arbitrary angles have overhangs no support-free printer can
  make. So `clean` and `makeable` cannot be required yet: tasks would no
  longer be solvable by construction. Making them required needs references
  rebuilt from clean, printable pieces in a chosen build direction -- the
  next piece of work, not done here.
- **Par is the generator's own lean answer**, likely heavier than good
  answers, so ratios above 1.0 will be common. It should be re-derived once the
  requirements that will count at each level are settled.
- **Generation of a three-part level 3 task is slow and uneven**: from
  seconds to many minutes when 100+ draws are rejected (obstacles find no
  legal place); `gen -j 0` spreads tasks over cores.

- **Holes must be true cylinders and seats true planes.** A STEP that
  approximates them with free-form surfaces or a mesh fails `HOLES` or
  `SEATING`. Every mainstream CAD exporter writes analytic cylinders and
  planes, but this has only been tested with build123d and FreeCAD.
- **The 0.1 mm hole-position tolerance is uniform**, stricter than the
  physical slack of large bolts (an M8 in a 9 mm hole would still go in at
  0.4 mm off). It is a stated tolerance, not a simulation of assembly.
- **No strength, stiffness or cost.** A paper-thin wall passes if it meets
  the rules. Volume is reported for later ranking, not used.
- **Bolts are simplified.** Tapped holes in the fixed parts, no nuts or
  washers; grip limits are 3 mm (or 0.75 x d) to 25 mm for every size; tool
  access is not checked (level 3).
- **Level 1 geometry is axis-aligned**: part A's face is always at the origin
  facing +Z. Level 2 removes that with a random overall placement.
- **Level 2 under-samples the larger 4-hole flange.** Draws whose straight
  bridge would clash are rejected, and large plates clash more often, so in
  seeds 0-19 the 4-hole flange is 6 of 40 interfaces at level 2 against 13 of
  40 at level 1. Level 2's range is tilt 20-70 degrees, sideways offset
  25-70 mm, height 35-80 mm.
- **The kernel's union is fragile when solids only touch face to face.** The
  generator works around it by letting pieces overlap by 0.5 mm and by
  building the bridge with flat faces only. An agent can meet the same
  weakness in its own modelling; the grader then reports what it measures.
- **Fixed parts are generic.** Mounts are plain plates with a block behind;
  only the mounting face, its bolt pattern and any spigot matter to a task.
- **Determinism is verified on one machine** with build123d 0.13 / its
  OpenCASCADE build. Other kernel versions may write different (still valid)
  STEP bytes.
- **The difficulty floor is a floor.** 0/20 for a baseline does not measure
  how hard a level is for a capable agent; see *The difficulty floor*.

## Licence and name

Apache License 2.0 -- see `LICENSE`.

The name lives in code in exactly one constant
(`NAME` in `src/jointbench/__init__.py`), plus the package folder and the
package and command names in `pyproject.toml`. Task files carry a format
version number, task ids look like `L1-s0`, and the random stream behind a
seed does not include the name, so a rename leaves every `task.json` and
STEP file byte-identical; only the command named in each brief changes.
(Checked on the rename from the working title to JointBench, the earlier name.)
