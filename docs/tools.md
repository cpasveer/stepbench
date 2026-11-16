# CAD tools: how a part gets to a gradable STEP

The grader reads only `output.step` and measures it exactly. It does not
care which program made the file, but it does need two things from it:
every bolt hole must be a **true cylinder**, and every seat a **true flat
face**. A file that describes a hole as many small flat facets (a mesh, or
a polygonal prism) fails `HOLES`, because there is no cylinder axis to
measure. This is deliberate: an exact model is the deliverable a designer
hands on, and the tolerance of 0.1 mm on a hole axis only has meaning on an
exact surface. The grader is not loosened for any tool.

So each tool needs a route to such a STEP. `jointbench.tools` packages
each proven route as three things: a starting file in the tool's language,
a one-command build to `output.step`, and a short note (`TOOL.md`) for the
brief.

```
uv run python -m jointbench.tools check                          # which tools work on this machine
uv run python -m jointbench.tools prepare tasks/L2-s0 --tool freecad
uv run python -m jointbench.tools build tasks/L2-s0 --tool freecad   # -> tasks/L2-s0/output.step
uv run jointbench verify tasks/L2-s0 --tool freecad --script submission_freecad.py
```

`prepare` writes the starter and `TOOL.md`; for a tool other than build123d
it removes the untouched build123d stub `submission.py`, so the folder
offers one starting point (an edited `submission.py` is left alone).
`--runner "uv run --project /path/to/jointbench"` sets the command prefix
the note shows, for agents that work against a frozen copy. `build` deletes
any old `output.step` first and deletes a half-written one if the build
fails, so a stale file is never graded. It ends with one neutral line, the
same for every tool: number of solids and faces by surface type.

## Summary

| tool | installed here | headless | route to exact STEP | proven on level 1 | in the pilot |
|---|---|---|---|---|---|
| build123d | yes (the package's own dependency) | yes, Python | writes STEP directly (OpenCASCADE) | yes, PASS | yes |
| FreeCAD 1.1.3 | yes, `/Applications/FreeCAD.app` | yes, `freecadcmd` | writes STEP directly (OpenCASCADE) | yes, PASS | yes |
| OpenSCAD | **no** | yes, `openscad -o` | `.scad` -> CSG (OpenSCAD) -> exact solids (FreeCAD's OpenSCAD importer) -> STEP | second half yes, PASS; first half needs OpenSCAD | no |
| SolidCraft 0.30.1 | yes, `/Applications/SolidCraft.app` (installed by the maintainer) | yes, `solidcraft-cli run` (also an MCP server) | writes STEP directly, own kernel | yes, PASS (holes square to their plate only) | yes |
| Anvil | -- | -- | cannot export STEP yet | out of scope | no |

## Evidence (level 1, task `L1-s0`, seed 0)

One hand-made connector, the same in every tool: a plate on each face
(5 mm), a wall between them beside the bolts, a clearance hole per bolt, a
23 mm hole for the motor's boss and an 8 mm hole where its shaft passes the
second plate. It is not the generator's design. Graded with the frozen
grader at commit d8fa88f.

| route | result |
|---|---|
| build123d script -> STEP | PASS 4/4, 25 faces, holes on axis within 0.000 mm |
| FreeCAD script (`Part.makeBox`, `makeCylinder`, `fuse`, `cut`) via `freecadcmd` -> STEP | PASS 4/4, identical numbers |
| OpenSCAD CSG, cylinders with `$fn = 0` (OpenSCAD's default) -> FreeCAD importer -> STEP | PASS 4/4, identical numbers |
| same, `$fn = 17`, `32`, `64` | PASS 4/4: the importer makes true cylinders |
| same, `$fn = 12` or `16` | FAIL `HOLES`: every hole became a 12- or 16-sided prism (135/175 flat faces, no cylinder) |
| SolidCraft script (`solid.box`, `solid.cylinder` with `operation: cut`) via `solidcraft-cli run` -> STEP | PASS 4/4, identical numbers |
| the part as a mesh (STL) converted to a solid in FreeCAD -> STEP (what OpenSCAD's own export gives) | FAIL `HOLES` (1275 flat facets, no cylinder) and `NO_INTERFERENCE` (the faceted holes are narrower than the round ones between their corners, so a bolt shank touches them) |

## build123d

Python on OpenCASCADE, the kernel the grader uses. The task folder's own
`submission.py` is the starter; `build` runs it with the benchmark's
Python. Nothing to convert.

## FreeCAD

A FreeCAD Python script (`submission_freecad.py`) run headless by
`freecadcmd`, using the Part module, which is OpenCASCADE underneath.
FreeCAD writes the STEP itself; nothing is converted. Found at
`/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd`, on the
`PATH`, or at `JOINTBENCH_FREECADCMD`.

Two things the build does that plain `freecadcmd` does not: it passes the
task folder in `JB_TASK` (freecadcmd gives a script no arguments; it treats
them as files to open), and it runs the script through a small wrapper
(`_fcrun.py`) that prints a full Python traceback with line numbers on
failure. On its own, freecadcmd prints only the exception message and still
exits with success, which would give a FreeCAD agent poorer error messages
than a build123d agent gets from plain Python.

## OpenSCAD

OpenSCAD's own 3D exports (STL, OFF, AMF, 3MF) all describe the surface as triangles, so on their own they can
never pass `HOLES`. The route that works uses OpenSCAD only to evaluate the
model (variables, loops, modules) and export the **CSG tree**: the
flattened list of primitives (cube, cylinder, sphere, extrusions) with
their placement matrices and the Boolean operations between them. FreeCAD's
OpenSCAD importer reads that tree and rebuilds each primitive as an exact
OpenCASCADE solid, then does the Booleans exactly.

```
openscad -o build/part.csg submission.scad
JB_CSG=build/part.csg JB_STEP=output.step freecadcmd jointbench/tools/_csg2step.py
```

The build runs both. `prepare` also writes `task.scad`, the task's faces and
bolts as OpenSCAD variables at full precision (OpenSCAD cannot read JSON).

Limits of this route, from FreeCAD's importer (`Mod/OpenSCAD/importCSG.py`,
FreeCAD 1.1.3):

- **Circle sides.** A cylinder or circle becomes a true circle only if its
  `$fn` is 0 (unset; `$fa`/`$fs` then do not matter) or above 16 (16 is
  FreeCAD's default `useMaxFN`). With `$fn` from 3 to 16 it becomes a prism,
  as measured above. The note tells the agent so.
- **hull(), minkowski(), resize(), offset with rounding, surface(),
  import().** The importer cannot build these exactly; it calls OpenSCAD
  to compute them and gets a mesh back. Their faces are triangles. Holes or
  seats cut from such a result by `cylinder`/`cube` are still exact, but
  this is untested. The note steers the agent to `polyhedron` (flat faces,
  exact) for a bridge.
- **Untested end to end.** OpenSCAD is not installed, so the first half
  (`.scad` -> `.csg`) has not been run here. The CSG files used above were
  written in OpenSCAD's export format by hand; tilted placements
  (`multmatrix` with a rotation) were also checked to come through as true
  cylinders. The tests stand in for OpenSCAD with a script that writes such
  a file.

Install, if the maintainer agrees: `brew install --cask openscad@snapshot`
(OpenSCAD 2026.10.11 development snapshot, from files.openscad.org, 73 MB
download). The stable cask (`openscad`, version 2021.01) is disabled in
Homebrew because it fails the macOS Gatekeeper check.

## SolidCraft

A parametric CAD program written in Rust with its own B-rep kernel
(MIT or Apache-2.0; docs.solidcraft.io, github.com/tangency-labs/solidcraft).
The maintainer installed version 0.30.1 as an app; its command-line program is
inside the app bundle, `/Applications/SolidCraft.app/Contents/MacOS/solidcraft-cli`,
and is not on the `PATH` (the app links it into `/opt/homebrew/bin` when
started, or by Help > Install solidcraft-cli command). The tool layer finds
it there, on the `PATH`, or at `JOINTBENCH_SOLIDCRAFT`.

It can be driven two ways:

- **By script.** A script is a JSON list of the same commands its user
  interface runs (`solid.box`, `solid.cylinder`, `solid.move`,
  `solid.combine`, `sketch.create`, `solid.extrude`, ... 570 in all;
  `solidcraft-cli commands` lists them with their parameters).
  `solidcraft-cli run script.json --out part.step` builds and exports;
  `solidcraft-cli eval script.json` builds and reports bodies, face types,
  volume and each step's warnings. A failing step stops the run, is named,
  and nothing is written.
- **By MCP**, command by command (`solidcraft-cli mcp --headless`, with
  tools `execute`, `batch`, `inspect_design`, `measure`, `body_topology`,
  `export`, ...). This is the way its makers intend an AI agent to use it,
  and the natural one for an agent in a client that has the server
  connected.

The tool layer and the pilot use the script. It is the same shape as every
other route (one file, one build command, the file archived with each
attempt), it needs no change to the agent's client, and a Claude Code
subagent can only use MCP servers its parent session already has. The MCP
route would make the comparison less even: the agent would see each
command's result as it works, which a script writer gets only from `eval`.
It is worth a separate run once a session has the server connected
(`solidcraft-cli mcp setup --client claude-code`, which changes that
client's configuration and is the maintainer's call).

**Known limit, found here:** a hole that meets a flat face **at a slant** is
written to STEP as a free-form (B-spline) surface, not a cylinder, although
SolidCraft itself still calls the face a cylinder (`eval` reports
`cylinder`). The grader then finds no hole there. A hole square to the
faces it passes through is written as a true cylinder, also when the plate
and hole are tilted together in space, which is the normal case for a bolt
hole in a plate on a tilted mounting face. Measured with a 30 x 30 x 10 mm
block and a 4 mm hole: vertical hole, cylinder; block and hole turned
together by 30 degrees, cylinder; hole turned 30 degrees in an upright
block, B-spline; a vertical hole through a block turned 30 degrees,
B-spline. This looks like a gap in its STEP writer (a cylinder bounded by
ellipses), not in its modelling, and is worth reporting to its makers. The
note tells the agent, and `tests/test_tools.py` pins the behaviour, so a
fixed version shows up as a failing test. Edges are written as rational
B-spline curves even where they are circles; the grader reads faces, so
this does not matter.

## Pilot: the same model, different tools

**This is a pilot, not a result.** Seven tasks, one run per task and tool:
one bad run moves a tool's average by a whole try. It shows what each tool
makes an agent stumble on; it does not rank the tools.

Set-up, the same for every tool: Claude Sonnet as a Claude Code subagent,
one agent per task, reference answers removed, at most six gradings, the
starter and `TOOL.md` written by `prepare`, built with `build` and graded
by a frozen copy of the grader. Each agent was told to work only in its
task folder and not to read the benchmark's source, examples, tests or
other runs. Afterwards each agent's tool calls were checked for any path
outside its folder (none found, in all 23 runs). SolidCraft agents on level 3
were also told to do the geometry in SolidCraft, not in build123d or
another CAD library (plain Python for numbers allowed); their logs show no
such use. Time, tokens and tool calls are as reported by the subagent tool
and include the grading time (26 to 80 s per try on level 3, with up to
eight agents sharing one machine).

### Level 3 (the four tasks of the model comparison, grader at 598ce0a)

Tries to pass, per task (`-` = not passed):

| tool | s900 | s901 | s950 | s951 | passed | tries in all | mean time | mean tokens | mean tool calls |
|---|---|---|---|---|---|---|---|---|---|
| build123d (the model comparison's Sonnet run) | 1 | 2 | 4 | 1 | 4/4 | 8 | 547 s | 94,100 | 22 |
| FreeCAD | 2 | 1 | 3 | 2 | 4/4 | 8 | 394 s | 88,400 | 17 |
| SolidCraft | 1 | 2 | 6 | 4 | 4/4 | 13 | 1,338 s | 118,600 | 37 |

First failing required check, per task:

| tool | s900 | s901 | s950 | s951 |
|---|---|---|---|---|
| build123d | -- | HOLES, NO_INTERFERENCE, TOOL_ACCESS | HOLES, NO_INTERFERENCE, TOOL_ACCESS | -- |
| FreeCAD | SECTION_MIN | -- | SEATING, NO_INTERFERENCE, CLEAR_PATH, TOOL_ACCESS, SECTION_MIN | HOLES, NO_INTERFERENCE, TOOL_ACCESS |
| SolidCraft | -- | HOLES, SEATING, NO_INTERFERENCE | NO_INTERFERENCE, TOOL_ACCESS, SECTION_MIN | NO_INTERFERENCE |

What the tool seemed to cause, as opposed to ordinary design mistakes
(a beam too thin, a part in a bolt's head space, a hole cut too long),
which all three tools showed alike:

- **FreeCAD: one trap, the same as on level 2.** On s950 the agent placed
  plates with `transformGeometry`, which turns flat faces into free-form
  surfaces, so no seat existed; `transformShape` keeps them flat. Fixed in
  one try. Otherwise FreeCAD behaved like build123d: the same tries in all,
  somewhat less time and fewer tokens.
- **SolidCraft: five tries lost to the tool, out of 13.**
  - s901, try 1: the agent rounded its plane directions to 4 decimals, so
    the holes were a hair off square to the plate, and SolidCraft wrote
    them as free-form surfaces (the slanted-hole limit above, triggered by
    0.0001 of slant). HOLES, SEATING and NO_INTERFERENCE failed together.
  - s950, tries 3 to 5: three files in a row that OpenCASCADE rejects as
    invalid solids (FILE_VALID), after adding wider plates and spheres at
    the bar bends; SolidCraft itself reported no error. The agent got back
    by reverting to an earlier layout. Its final part passes but its
    material score does not count (a 0 mm wall at a joint).
  - s951, tries 1 and 2: a 0.07 mm3 overlap with a fixed part, from plates
    seated exactly on their mounting faces, which in build123d and FreeCAD
    gives no overlap; the agent lifted the plates 0.002 mm. Try 3 graded a
    missing file after a SolidCraft build failed ("shell boundaries
    intersect or touch"). Spheres at beam joints made a build hang.
  - SolidCraft runs also took the most tool calls and time: the agents
    wrote the JSON from Python and iterated with `eval`, and two agents
    ran their own clearance searches in Python.
- **A grader fault seen in passing:** on SolidCraft s951 try 2,
  `SECTION_MIN` reported "could not be measured: AttributeError: 'NoneType'
  object has no attribute 'center'", and on s950 try 1 it reported a
  0.0 mm2 section. Both are worth a look by whoever owns the checks.

### Level 2 (three fresh tasks, seeds 900 to 902, grader at d8fa88f)

| tool | s900 | s901 | s902 | passed | tries in all | mean time | mean tokens | mean tool calls |
|---|---|---|---|---|---|---|---|---|
| build123d | 3 | 1 | 2 | 3/3 | 6 | 100 s | 70,900 | 7 |
| FreeCAD | 3 | 2 | 3 | 3/3 | 8 | 115 s | 73,900 | 10 |
| SolidCraft | 1 | 1 | 1 | 3/3 | 3 | 187 s | 79,800 | 11 |

First failing checks: build123d SEATING and NO_INTERFERENCE (plate too
small, rod in a bolt's space), FILE_VALID (two solids, beam not joined);
FreeCAD NO_INTERFERENCE (bar in a bolt's head space), HOLES and SEATING
(plate turned the wrong way), HOLES and SEATING (seats turned into
free-form surfaces by `transformGeometry`, then a placement overwritten).
In three runs (two build123d, one FreeCAD) a try was wasted grading an
unchanged file after a failed `sed -i` edit, a shell habit rather than a
tool effect. Level 2 separates nothing: every tool passed everything.

On level 2 the SolidCraft agents wrote their JSON from a Python script and
one checked its part in build123d before grading. That was allowed then
and blurs the comparison, which is why level 3 forbade it. One FreeCAD
agent wrote a scratch file to `/tmp`, outside its folder (a write, not a
read of anything forbidden).

## Recording the tool

Every graded attempt should carry `--tool <name>`. The STEP header is a
cross-check: build123d writes `build123d`, FreeCAD writes `FreeCAD` (also
for the OpenSCAD route, since FreeCAD writes the file), SolidCraft writes
`SolidCraft`. The header is
recorded by the grader but never graded on.
