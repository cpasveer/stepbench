# STEP.bench viewer and website

The page that shows a task in 3D, an answer on top of it, the grader's
verdict, and every attempt, and that lets a person try the same task as the
agents by uploading a STEP file.

```
uv run jointbench play tasks              # a folder of tasks, or one task folder
uv run jointbench site tasks --out public # static copy for hosting

uv run jointbench score runs --out scores.json   # score model runs (layout below)
uv run jointbench play runs                      # scores computed live from the runs folder
uv run jointbench play tasks --scores scores.json
uv run jointbench site runs --out public         # static copy with the scores page
```

`play` starts a local server on http://127.0.0.1:8765 (the next free port if
taken) and opens the browser. `site` writes a folder that any static host can
serve; to look at it locally: `python3 -m http.server -d public`.
Add `--no-reference` to `site` to leave the generator's reference answers out.

## The one rule: the browser decides nothing

Every pass or fail on the page was written by the Python grader, the same
code `jointbench verify` runs for an agent. An uploaded STEP is graded by
`grade.grade` and logged by `grade.log_attempt` exactly as `verify` does, and
the page then reads it back from `attempts.jsonl` like any other attempt. A
person and an agent are therefore measured identically, and nothing in
JavaScript re-implements a check.

The browser cannot read STEP either. Python turns each solid into a triangle
mesh for shading plus its true B-rep edges as lines (cylinder seams left
out), so holes, chamfers and silhouettes read as they do in CAD.

## What a visitor can do

| | where |
|---|---|
| see a task: fixed parts, mounting faces (blue outline and normal arrow), bolt axes (dash-dot centre lines, hole and head circles), brief | task page, *Brief* tab |
| orbit, zoom, standard views with the navigation ball, a section plane on X/Y/Z with hatched cut faces, coordinates under the pointer in mm, scale bar | the drawing |
| see an answer on top: orange, each layer solid / ghost / off; the reference answer in green behind a spoiler prompt | *Show* toolbar |
| see the verdict: each check with its reason and number, measurement tables; failing bolts marked red in 3D with the grader's words; overlap regions drawn in red with their volume | *Verdict* tab |
| follow attempts: newest first, click to replay; with `play`, new lines in `attempts.jsonl` appear within half a second and, with *follow* on, are shown at once | *Attempts* tab |
| try it yourself: download the parts, model in any CAD program, upload, see the verdict with marks, try again; kept as the person's own run with measured time, shown on the scores page as a human baseline | *Try it yourself* (`play` only; the static copy says it is a saved copy and how to run it locally) |
| browse: plan-view thumbnail per task, level, seed, gap, attempts, solved; a plain-language introduction | front page |
| see real attempts, one after another, turning slowly: task, who, passed or failed, which try, the failing check and its mark; touch the view to stop and turn it yourself | front page, beside the introduction |

| compare models: one row per model run, tasks attempted, passed first try, passed within the allowed tries, average tries, by level, most-failed check; grouped by model, by CAD tool, or as a model × tool grid; filtered by level and tool; time, tokens and cost beside the pass rate; pass rate against cost | *Scores* page |
| one run's tasks: every attempt in order as a coloured tick, first pass, failed checks; each opens the timeline | click a model |
| watch what a submitter did: step or play through its attempts; each step shows the part, the checks, the grader's feedback as `verify` printed it, the code, and what changed since the previous step (previous part as a blue ghost, checks that flipped, volume, a line diff of the code); the CAD tool and the time and tokens per step when reported | *Timeline* tab |

The number of fixed parts, bolts, checks and poses is read from the data.
With more than one pose a scrubber appears under the drawing (level 1 has one).

View settings can be put in the link: `#/task/L1-s1/1?fixed=ghost&view=front&sec=y,0.4&tab=brief&theme=light`
(`fixed`/`part`/`ref` = solid, ghost, off; `faces`/`bolts` = on, off;
`view` = iso, top, bottom, front, back, right, left; `pose` = index; `tab` = verdict, timeline, made, attempts, brief, try; `dl=1` opens the download panel; `box=1` adds a clipping box; `secpanel=1` opens the section panel; `proj=parallel`; `cmp=1` = previous step as ghost; `proc` and `field` = manufacturing process and rule id). The scores page takes `group` = model, tool, grid.

## Compare

Compares models on the same tasks (`#/compare`), on these rules, which are
also stated on the page:

- **A model's answer to a task is its submission**: the first passing
  attempt within the allowed tries, or, if none passed, its last allowed
  attempt. Never the best attempt picked afterwards. (`submission` per task
  in `score.py`.)
- **Repeated runs**: a model with several runs of a task is shown by its
  **typical run**, the median by tries to pass (a run that never passed
  counts as the most tries; with an even number of runs the less flattering
  middle one; ties go to the earlier run). The page says how many runs there
  are and how many passed, and every run can be chosen. Never only the best
  run. Runs belong to one model when their `run.json` gives the same
  `model` (else the same name).
- **The grid**, per level: tasks down, models across; each cell a small 3D
  picture of the submission (fixed parts see-through, the part in the
  answer colour), pass or fail, the try, the failing check. Tasks that
  separate the models come first (some passed and some did not, most evenly
  split first), then those none passed, then those all passed.
- **The compare page**, per task: two to four submissions side by side,
  each in its own 3D view with its verdict, the try, the failing check,
  time, tokens, material ratio and tool when reported, and the run. The
  cameras and the section cuts are linked, so all views move together.
  *Overlay* puts two submissions in one scene: the first solid in the
  answer colour, the second see-through in green, the fixed parts ghosted.

**On the scores page** the two headline figures per model are named *passed
at the first try* and *passed within the allowed tries*. Where a model has
repeated runs, its row shows the mean over its runs with the lowest and
highest run beside it, never the best run.

## Task cards and grid pictures

The task cards and the compare grid show small 3D pictures (isometric from
the south-west, true edges drawn) instead of 2D plans. One hidden 3D context
draws all of them, one at a time, into ordinary images, so a page with a
hundred tasks has one 3D context and a hundred pictures; a card is drawn
when it comes into view, redrawn when the theme changes, and turns slowly
while the pointer is over it. The meshes are coarse versions made for this
(`thumb.json` per task with the fixed parts, `thumb/<n>.json` per attempt;
about 45 KB each). Where WebGL is not available the 2D plan is shown.
Measured: twelve cards drawn in under 0.2 s in Chrome; headless Chrome with
its software renderer takes seconds per picture, so pictures of
these pages are best taken in a normal browser.

## Scores

A runs folder holds one folder per model run, each holding task folders as
`jointbench gen` writes them, with the attempts log that run produced:

```
runs/<run>/run.json                  optional: name, date, tries_allowed, notes, tool, currency, model_version
runs/<run>/<task-id>/task.json       (and the rest of the task folder)
runs/<run>/<task-id>/attempts.jsonl
```

`jointbench score` only counts; nothing is re-graded. The definitions, also
on the page:

- **tries** for a task: attempts up to and including the first pass, or all
  attempts if it never passed. Attempts after the first pass stay visible
  (faded) but do not count.
- **passed first try**: attempt 1 passed.
- **passed**: an attempt numbered within `tries_allowed` passed (every
  attempt counts when `run.json` gives no limit).
- **average tries**: mean tries over the tasks that passed.

**Measured and reported figures are kept apart.** Pass/fail, checks and
counts come from the grader. Time, tokens, cost, tool calls and CAD tool are
what the submitter wrote into the log or `run.json`; the page shows them in
italics, labelled "reported", and shows a missing one as "—", never as zero.
A total is shown only over tasks where every counted try reported the
figure, with "(n/m)" when only some tasks did.

Reported figures are read as `verify` records them: the `tool` field and
the `reported` object (`wall_seconds`, `tokens_in`, `tokens_out`, `cost`,
`currency`, `model`, `model_version`, `tool_calls`), falling back to
`run.json` for the whole run. A few older names are also accepted (see
`REPORTED_KEYS` in `src/jointbench/web/score.py`).

`run.json` can carry the figures too, where the log has none, in the shape
the model-run harness writes:

```
{"model": "haiku", "wall_seconds": 241.5, "tokens_total": 112740, "tool_calls": 13.0,
 "per_task": {"L2-s700": {"wall_seconds": 327.8, "tokens_total": 136528, "tool_calls": 14}, ...},
 "reported_note": "how the figures were measured"}
```

- `per_task` (also accepted as `tasks`) gives a task's figures for the task
  as a whole. A figure in the attempts log always wins over one here.
- The top-level figures are read as **the mean per task**. When `per_task`
  is there, the mean is recomputed from it, so the two cannot disagree.
- `tokens_total` is input and output together, for when no split was
  reported. It is shown as "tokens, total", labelled "total" in tables,
  and never put in the input or output column.
- `reported_note` is shown on the run's page under its notes.
- Cost is never derived from tokens or time.

### Score charts

Drawn as plain SVG in the site's own style, readable in both themes, with
exact values in a box on hover (or keyboard focus) and a table of exactly
the plotted numbers under each chart. The idea of the first chart comes from
the ARC Prize leaderboard (score against cost per task); nothing of theirs
is used.

- **Score against cost.** One point per model and CAD tool (a model used
  with two tools is two points; repeated runs of one combination are
  pooled). Up: share of tasks passed, at the first try or within the allowed
  tries (switch). Across, on a logarithmic scale: cost, tokens (in + out) or
  time per task (switch). The dashed staircase is the frontier: the best
  score available at or below each cost, through the leading points. A run
  marked as human (`run.json` `"human": true`, or a model name starting with
  "human") is drawn as a diamond with a dotted line across at its score.
- **Where each model falls off.** Pass rate by level, one small chart per
  model and tool; a level a model did not attempt leaves a gap.
- **Which task, which model.** One row per run, one square per task, tasks
  sorted from most to least often passed. Full green: passed at the first
  try; pale: passed later; red outline: failed. A square opens the task.
- **Material among passed tasks.** The benchmark's one score: par volume
  divided by the part's volume (1 = as lean as the task's deliberately lean
  par answer; higher is leaner), one dot per pass and the median per model.
  Only tasks with a par volume (level 3 on) have it; the chart says so.

**Never a figure that was not reported.** The horizontal figure of a point
comes from, in order: every task's own figure (attempts log, else
`run.json` `per_task`); the run's mean per task from `run.json`, when the
point covers whole runs; else the mean over the tasks that did report it,
marked as partial in the table. The table says which source each point
used. On the tokens axis a task counts input + output when both were
reported, else `tokens_total`. A point with none of these is not drawn, and the chart names it.
The level, CAD tool, task set (`run.json` `"set"`) and manufacturing
process (`run.json` `"process"`) filters apply to every chart; the set and
process switches appear only when some run names one.

## Attempt archive and timeline

`jointbench verify` keeps every graded attempt in `<task>/attempts/0001/`,
`0002/`, ... (see "Every attempt is kept" in the main README), and each line
of `attempts.jsonl` points to its folder (`archive`) and names its files
(`files`: step, script, feedback, note). The viewer reads exactly that:

- the part of each step is drawn from the archived STEP, so a step keeps its
  own shape even after the agent overwrote `output.step`;
- the code is the archived script, with how it was linked (`ran`, `named`,
  or `found` -- an edited `submission.py` that lay in the folder, which is
  not proof it made the file);
- the feedback is `feedback.txt`, exactly as the submitter saw it;
- the note is `note.txt`, shown as reported;
- the CAD tool and the time, tokens, cost, model and tool calls come from
  the record's `tool` and `reported` fields and are shown as reported;
- the STEP file header's originating system is shown separately, as read
  from the file (evidence, not proof, and never graded on).

Only text files are served from an archive, at most 512 KB each. Attempts
recorded before the archive existed have no folder; if their file has since
been overwritten, the step is shown as **geometry not kept**, with its
verdict as logged and the feedback rebuilt from the logged checks (labelled
as rebuilt).

## The 3D view

A near-empty canvas with the model alone (white in light, slate in dark),
tools floating at the edges with soft shadows and few borders. The layout
and the way it navigates follow the maintainer's own model viewer (see
"Where the ideas come from" below); everything here is written for
STEP.bench.

- **Top left, a slim rail**: one line icon per control, each with a
  tooltip, a hidden label for screen readers and a key.

  | icon | does | key |
  |---|---|---|
  | block | fixed parts: solid → see-through → hidden | F |
  | bracket | the answer: solid → see-through → hidden | A |
  | bracket with tick | the reference answer (a spoiler, asks first) | R |
  | face with arrow | mounting faces on / off | P |
  | screw | bolts on / off | B |
  | arrow into tray | downloads, in a small panel beside the rail | D |

  A body's state is read from its icon: filled = solid, faint and dashed
  = see-through, crossed out = hidden. Obstacles (level 3) have an icon
  ready.
- **Top right**: the navigation ball; under it a column of round tools --
  show the whole model, zoom (press and drag), pan (press and drag),
  parallel/perspective, section, measure -- and one quiet line naming the
  view and projection, e.g. "Right (+X) · parallel" or "User ·
  perspective". A tool still in effect while its panel is closed shows a
  small dot.
- **Navigation**: left-drag orbits about the pivot (0.005 rad per pixel,
  no inertia, Z stays up); right-drag or Shift-drag pans so the point
  under the cursor follows it; the wheel zooms toward the point under the
  cursor; a double click makes the clicked point the pivot. Moves glide
  (320 ms, eased), turning by heading and elevation so a jump to the
  opposite side swings round the part. Home frames the model from the
  south-west, a little above. Perspective by default (50°); the six axis
  views are parallel and orbiting away returns to perspective, unless
  parallel was chosen with its button.
- **Navigation ball**: positive ends lettered on stubs, negative ends pale
  (lettered on hover); click an end to look from that side, click it again
  for the opposite side; drag the ball to orbit at the same rate as the
  drawing.
- **Keys**: `1` front, `3` right, `7` top, with Ctrl the opposite side; `5`
  parallel/perspective; `9` the other side; numpad `4 6 8 2` turn 15°;
  `0` or Home the whole model; `S` section, `M` measure, `]` side panel;
  Esc puts a tool down.
- **Section** (scissors, `S`): a small panel with the ways to cut.
  - *X, Y, Z*: a plane across that axis through the pivot; the half nearer
    the viewer is taken away.
  - *On face*: click a face of a part; the plane takes the face's position
    and outward direction (snapped to an axis within 4°) and starts 0.5 mm
    outside it, so nothing is cut until it is dragged in.
  - *Box*: six planes around the model, each face dragged on its own and
    never past the opposite one (1 mm apart at least), to isolate a region.
  - Each cut has a dot in the drawing and an arrow to the side being cut
    away; dragging the dot moves the cut along its arrow in 0.1 mm steps
    (1 mm with Ctrl), with a live readout ("18.5 mm into the part", "X
    +6.1 mm"). The panel lists the cuts, each with flip and remove.
  - Every cut face is capped with a solid, hatched face and a clean outline
    as in a technical drawing (fixed parts and the answer hatched in
    opposite directions). The fill uses the stencil buffer, per body and
    plane, so it is exact for curved and angled parts and for several cuts
    at once; the outline is the exact intersection of each plane with each
    mesh, trimmed by the other cuts.
- **Measure** (ruler, `M`): click two points; the distance shows in mm with
  two decimals, and hovering the label shows dx, dy, dz and a cross to
  remove it. A pick snaps to a corner of the part (where three or more true
  edges meet, within 9 px), else to the nearest point on a true edge (within
  7 px), else to the face; the cursor mark shows which (square, diamond,
  dot). Esc drops a half-made measurement, then puts the tool down.
- **Side panel**: folds away with the tab on its edge or `]`, so the model
  fills the window; the choice is kept in the browser.
- **Downloads**: for the answer shown and for every fixed part, as STEP,
  STL and 3MF. STEP is the exact file (for an answer, the archived file the
  grader graded, byte for byte). STL and 3MF are meshed from that STEP by
  OpenCASCADE in millimetres, linear tolerance 0.01 mm, angular tolerance
  0.1 rad. `play` makes them on request and keeps them; the static export
  writes them all (`data/tasks/<key>/download/`). Checked by re-opening
  each: STEP as one solid, STL and 3MF with the solid's size.

### Where the ideas come from

The maintainer's model viewer (private, not part of this project) was read for
its design and behaviour only; nothing was copied, and none of its icons or
assets are used (the icons here are drawn in `site/js/icons.js`). It is a
hand-written WebGL2 renderer; STEP.bench uses three.js. Taken as ideas:

| idea | from the maintainer's viewer | here |
|---|---|---|
| layout: rail top left with a closable floating panel; ball, round tool column and a view/projection line top right; soft shadows, white canvas | viewer layout and house style | `site/js/app.js`, `site/css/style.css` |
| orbit 0.005 rad/px, exact pan, wheel zoom toward the cursor, double-click pivot, no inertia, glides with cubic easing and heading/elevation interpolation | camera controls | `site/js/nav.js` |
| perspective 50° with axis views parallel; orthographic size matched to the perspective view at the pivot; home from the south-west; fit = bounding sphere in the smaller field of view, 5 % margin | camera | `site/js/nav.js` |
| keys 1/3/7 (+Ctrl), 5, 9, numpad turns 15° | keyboard | `site/js/viewer.js` |
| ball: stubs to positive ends, pale negative ends, click again for the opposite view, drag to orbit | orientation gizmo | `site/js/viewer.js` |
| section tools X/Y/Z, on face (normal snapped within 4°, starts just outside the face), box with faces that cannot cross; dot-and-arrow handles dragged along the normal with a readout | section tools | `site/js/clip.js`, `site/js/viewer.js` |
| measuring with corner / edge / face snapping, snap cursor shapes, label with dx dy dz and remove | measure tool | `site/js/measure.js` |

Kept from STEP.bench's own earlier design: the hatched cut faces (per
body kind), the line icons, the verdict panel, colours and both themes.

## Try it yourself: the human route

A person takes a task the way an agent does, in their own CAD program, and
is scored the same way. On any task page (with `play`), *Try it yourself*
(a button under the task name, and the last tab) gives three steps:

1. **Download the parts.** Name and CAD program first (the name keeps their
   tries apart and is what the scores page shows). The fixed parts and, at
   level 3, the obstacles as STEP; the brief and task files below. The first
   download starts their clock.
2. **Model the part** in their own program: one solid, millimetres, the
   parts imported where they are.
3. **Upload the STEP.** It is graded on this computer by the same grader as
   the agents. The page moves to their own copy of the task, with the
   verdict, the failing bolts and overlaps marked in the drawing, and the
   upload box still there for the next try.

**Kept as a run of its own.** A person's tries never go into the task's own
log (in a runs folder that log is a model's). They form a run in the same
layout as a model run, so `jointbench score` reads it unchanged:

```
<runs>/human-<name>/run.json          name, "model": "human", "human": true, tool, clock, per_task
<runs>/human-<name>/<task>/           what a person is given (never the reference answer)
<runs>/human-<name>/<task>/attempts.jsonl and attempts/NNNN/
```

`<runs>` is the runs folder `play` was given; for a folder of tasks it is
`<tasks>/human-runs` (picked up again the next time `play` starts); for a
single task folder, `<task>/.play/human-runs`.

**Time is measured, not reported.** The server records when the person
started the clock and, at their first passing upload, the seconds since
then (`per_task.<task>.wall_seconds` in `run.json`). A task whose clock was
never started has no time; none is estimated. Their CAD program is stored
as reported, on the run and on every attempt.

**On the scores page** a person's run is a human baseline: drawn as a
diamond, labelled with their name and program, with a dotted line across
the chart at their score.

`POST api/start?task=&who=&tool=` starts the clock (making the person's copy
of the task on first use), `POST api/upload?task=&who=&tool=` grades into
it, `GET api/clock?task=&who=` reads a clock without changing anything.

## About page

`#/about` says what the benchmark tests, how a part is judged, what is built
and proven, what is not yet, and how to run it. Its facts are taken from
the repository's `README.md` and `docs/*.md` (`site/js/about.js` says so at
the top: change it when they change); the counts of tasks and runs are read
from the data the copy carries.

## Level 3 in the viewer

- **Parts.** Any number of fixed parts is drawn, from the task file; level 3
  has two or three.
- **Obstacles.** The keep-out boxes the answer must stay clear of
  (`CLEAR_PATH`) are drawn exactly as the task file states them (centre,
  size along the box's own axes), see-through violet by default, with their
  own toolbar button (key O: see-through, solid, hidden) and their own hatch
  in a section. Task cards and the compare grid show them too.
- **Tool paths.** For `TOOL_ACCESS` each bolt gets a dashed cylinder: the
  straight path a hex key needs, built the way the grader builds it -- from
  the top of the head at the measured grip, outward along the axis, the
  task's tool length long, head diameter (+ the allowance) wide. A path the
  grader found blocked is red.
- **Required and advisory checks.** From level 3 the report also holds
  geometry-quality and printing checks that do not decide the verdict. The
  verdict block counts the required ones ("7 of 7 required · 20 of 25 in
  all"), so a pass never reads as a partial pass.
- **Material.** Where the task has a par volume, the grader records the
  material score, par volume divided by the part's volume (higher is
  leaner); a passing part shows it, e.g. "1.21", or the figure with "not
  counted" and the grader's reason (a part below the minimum wall does not
  count). The scores page plots only counted figures and lists the
  rest with their reasons.
- **Compare order.** Tasks that separate the models come first: some
  passed and some did not; then all passed but in different numbers of
  tries, widest spread first; then none passed; then all alike.
- **Scores across levels.** Levels are never pooled in the score chart:
  with more than one level selected, each model and tool has one point per
  level (circle, square, triangle) and each level its own frontier line.
  The pass-rate-by-level charts and the board's level column show the rest.

## Look

Two themes, switched with the button at the top right. **Dark is the
default**: a slate ground, rgb(56, 65, 84) (#384154), with a slightly darker
drawing sheet under the 3D view and light linework. **Light** is paper and
ink. After the first visit the reader's last choice is kept, in that browser
only. Text colours clear 4.5:1 against their ground in both themes; the thin
rules sit around 2:1, enough to read as lines without shouting.

**The top bar** holds the name as plain text, four menu words (Tasks,
Scores, Compare, About) and the theme switch, nothing else: no logo, no
status light, a hairline underneath. The current page is marked by a thin
underline, not a filled box. On a phone the menu takes its own line under
the name.

**Type.** One humanist sans for the name, menu, headings and all interface
text, in sentence case without letter-spacing; the monospaced face only for
numbers in tables and readouts, file names and check names (`HOLES`). The
sans comes from the reader's own computer, so nothing is downloaded and the
offline copy looks the same: Seravek on Apple systems, Gill Sans Nova or
Calibri on Windows, Ubuntu or DejaVu Sans on Linux, then any sans. Small
labels are written in lower case in the scripts and shown with a capital
first letter by the stylesheet.

The verdict reads like the header of an inspection report: plain words in
ruled cells, colour only on the verdict word itself. Pass and fail never
rely on colour alone: the words differ, check marks are ✓ and ✕, and failed
attempts are drawn as hollow squares where passes are filled. Ghosted parts
are half see-through, show their back faces and keep their edges, so they
still read as solids with the parts behind them visible; on the dark ground
they are lightened a little so they do not sink into it.

**Material score** (par volume divided by part volume, higher is leaner)
is shown for passing attempts in the verdict block, the timeline, the
compare cells, a sortable scores column (median over counted passes) and
its own chart, read from what the grader wrote into each attempt
(`info.material`). Geometry-cleanliness faults (sliver faces, tiny steps,
knife edges) are reported as `Q_*` checks; marking them in the drawing like
the manufacturing fields is still to do.

## Front page showcase

Beside the introduction a 3D view shows real attempts one after another.
Each turns slowly for seven seconds under a one-line caption: task, who
submitted it, passed or failed, on which try, and for a failure the first
failing check in a few words; the failing bolt is named in the scene and an
overlap is drawn in red. Failures are included on purpose, alternating with
passes, a failure first: they show what is measured. On the first view two
small labels name the roles, *given* on the grey parts and *to model* on the
answer.

- **Which attempts.** If the folder being shown holds a `showcase.json`, a
  list of `{"task": <key or task id>, "n": <attempt>}`, those are shown in
  that order. Otherwise every attempt whose geometry is kept is a candidate,
  in random order. With no attempts at all, tasks are shown with their
  reference answer (unless the export leaves references out).
- **The visitor is in charge** as soon as they press, drag or use a key on
  the view: cycling stops, the view turns and zooms as on a task page, and
  the caption links to the attempt. Before that the mouse wheel scrolls the
  page, not the model. A small arrow shows the next attempt.
- **Reduced motion** (the system setting): one attempt, still, no cycling.
- **Cost.** It starts only after the text is on screen, uses one 3D context
  (the task-card pictures use one more, shared by all cards), fetches the
  next attempt while the current one turns, and in the static export reads
  meshes converted at export time. Without 3D in the browser the old
  explanatory drawing stays.

## Speed

Measured before and after with headless Chrome on this computer, empty
browser cache each time, first visit to a folder (meshes not yet made),
100 generated tasks (50 at level 1, 50 at level 2) for the front page and
task page, the sample runs for the scores page. "Network" is the same with
the connection held to about 10 Mbit/s and 60 ms delay.

| | before | after |
|---|---|---|
| front page: text shown, local | 0.14 s | 0.09 s |
| front page: first 3D frame, local | 0.28 s | 0.35 s |
| front page: text shown, network | 1.72 s | 0.50 s |
| front page: first 3D frame, network | 2.18 s | 1.12 s |
| front page: transferred | 1.95 MB | 0.48 MB |
| 100 task cards: scroll top to bottom in 4 s | no slow frame | no slow frame |
| 100 task cards: all pictures drawn | within 3 s of the scroll | same |
| task page, first visit (meshes made on demand): parts shown, local | 0.21 s | 0.21 s |
| task page, again: parts shown, local | 0.09 s | 0.10 s |
| task page, first visit, network | 1.72 s | 0.78 s |
| task page: transferred | 1.60 MB | 0.37 MB |
| scores page: drawn, network | 1.64 s | 0.47 s |
| scores page: transferred | 1.52 MB | 0.06 MB |

What made the difference:

- **Compression.** `play` sends scripts, styles, data and meshes gzip-
  compressed when the browser accepts it (three.js 1.27 MB -> 0.26 MB, a
  mesh 80 KB -> about 28 KB), each file compressed once and remembered.
  Static hosts (GitHub Pages, Netlify, Cloudflare and the like) do this
  themselves for the static export.
- **3D code only where 3D is drawn.** three.js and the modules on it load
  only on the front page (fetched alongside the task list, not before the
  text), task pages and the compare pages. The scores page never loads it.
- **Meshes fetched once.** A mesh is named by the hash of its STEP file, so
  the page keeps each one in memory for the visit and the browser may cache
  it; switching attempts or tabs does not fetch it again.
- **Already in place:** meshes made once and kept on disk by file hash; one
  shared 3D context for all card pictures, drawn as they scroll into view;
  the scores page reads one results file.

Not done, on purpose: a binary mesh format. Measured on 19 meshes: JSON
1574 KB, JSON gzip-compressed 546 KB, raw binary 1173 KB, binary compressed
463 KB. Compression alone takes most of the gain; a binary format would
save a further 15 % at the cost of a second file format. Making a mesh
takes 20-40 ms per STEP file; the first one in a fresh `play` process waits
about 3.6 s while the CAD library loads.

## Reviewing the part by one check

The verdict tab lists every check on one line: mark, name, the measured
value with its unit, and the limit on the side the checker states ("≥",
"≤"; "limit" when the checker's words do not say which side). The reason
follows on a second line for failing checks only. Checks are grouped by
requirement (fits, clean, makeable, sturdy, then any family's own group);
each group header gives its count ("clean 8/10") and whether it decides
the verdict or is reported. A group opens when something in it fails and
stays closed when all pass; opening or closing one is remembered in the
browser (`jb.checkGroups`).

Clicking a check (or Enter on it) reviews the part by that check in the 3D
view; clicking again or Esc returns to the plain view and, if the camera
was not moved meanwhile, to where it was. Passing checks can be inspected
too ("least margin" instead of "worst"). Everything drawn is what the
checkers wrote (`site/js/inspect.js`):

| check | drawn from | shown |
|---|---|---|
| `Q_*` | the quality checker's field file (`python -m jointbench.quality --fields`) | per-triangle field, failing faces striped, a dot at every named place, framed on the worst |
| `FDM_*`, `PBF_*`, `IM_*` | the process's field file (`python -m jointbench.dfm --fields`) | the same, plus how it would be made (below) |
| any of these without a field file yet | `details.where` (point and face) | the named faces highlighted on the part; live, the field is measured meanwhile (about 20 s) and replaces them |
| `HOLES` | `details.holes` | every bolt axis labelled with its offset; the worst framed when failing |
| `SEATING` | `details.bolts` | ring contact per bolt, mounting faces shown |
| `NO_INTERFERENCE` | overlap meshes (`marks`) | the overlap regions in red, largest framed |
| `CLEAR_PATH` | `details.overlaps` | obstacles solid, labelled with their overlap |
| `TOOL_ACCESS` | `details.envelopes` | the tool paths, blocked ones named |
| `SECTION_MIN` | `details.sections` | each pair's thinnest cut as a plane, a circle of the measured area, area against need |
| `STIFFNESS` | the stiffness field file (`python -m jointbench.stiffness --fields`) | the weakest load case, deformed and exaggerated, coloured by displacement, over the unloaded shape |

**How it would be made.** For a printing process: the bed is drawn under
the part, the build direction as an arrow up from it, and the face the
part stands on highlighted and named, in the scene and in words ("Stands
on face 92, 3520 mm² on the bed; build direction (0.23, 0.49, 0.84), best
of 12 candidates"). The face is the bed-contact check's own face (else the
"face N down" of the chosen direction); the area is that check's measured
value. For moulding: both pull directions and a parting plane, labelled as
assumed through the middle of the part, because the checker states no
parting line. The same setup is drawn in the "Made how?" tab.

Field files are made like the manufacturing ones: cached by the graded
file's sha256 under `<task>/.play/dfm/`, listed per attempt as `fields`
(processes stay under `dfm`, so "Made how?" lists processes only), and
copied into a static copy when they exist. A static copy never runs a
checker; without the file it shows the named places.

**Data the checks do not write yet** (for their authors):

* `HOLES`: the measured hole axis of each hole (only the offset is given),
  so the offset cannot be drawn as a vector from bolt to hole.
* `SEATING`: where on each ring the contact is; only the share.
* `CLEAR_PATH`, `TOOL_ACCESS`: no overlap region meshes (only
  `NO_INTERFERENCE` has them); the obstacle or tool path is labelled with
  its volume instead.
* `SECTION_MIN`: the outline of the thinnest section; a plane and a circle
  of equal area stand in for it.
* Moulding: no parting line or plane.
* `FILE_VALID`: nothing to place, by nature.
* `STIFFNESS`: the drawing exists but is unchecked on a real attempt; none
  in the runs has the check, and the analysis is not run from the viewer
  (it needs a minute or more).
* Moulding on a large level 3 part took over the server's 10-minute limit
  and was stopped; the setup drawing was checked on a level 1 part.

## Stress, for viewing only

The stiffness analysis's field file (version 2, checker commit 3b5dcb5)
carries von Mises stress under the design loads. The verdict tab lists it as
a line of its own in the sturdy group, **STRESS**, marked ◦ and never
counted: it is not a requirement and in no verdict, and the page says so in
the legend and the panel (the checker's own note). Clicking it shows the
envelope (the highest stress over the six load cases) first, shaded from
the per-vertex values; the colour scale is the file's own, 0 to the 99th
percentile, with anything above in the top colour (a sharp inside corner
has no finite peak). Tags at the checker's probes (peak, 99 %, 90 %,
median). Each load case can be chosen, and drawn on its deformed shape
(displacement × design load × the file's exaggeration) over the unloaded
shape. The stiffness mesh has no face numbers, so shading groups and edge
lines come from a 30° crease angle. Live, the analysis runs on demand
(about half a minute with `--quick`); a static copy shows it where the file
exists. The STRESS line appears for connecting-part tasks only (the
analysis needs their mounts).

## Opening a runs folder, and the first view

`jointbench play <runs>` opens a folder laid out as `<run>/<task>/task.json`
before anything is graded, lists every task, and rescans every 2 s for runs
and tasks that appear later; their attempts are followed as before.
A task page that opens on an answer frames it with its mounting faces from
the three-quarter direction (four corners, above and below) where the fewest
of their points hide behind a fixed part or an obstacle; a `view=` in the link
still wins.

## Joint and mechanism tasks

Their fixed parts are given as solids, read from STEP files beside
`task.json`; the viewer loads every task with its folder, so every family
opens (tested on one generated task of each). Mechanisms with stored poses
play through the pose slider. A request the server cannot answer comes back
with the reason (HTTP 500 and the error), and the page shows it instead of a
bare network error.

## The requirement ladder, and process versions

**Ladder** (`src/jointbench/web/ladder.py`, `site/js/ladder.js`). Every
graded part gets five rungs in a fixed order: valid solid, fits, cleanly
modelled, makeable (by the task's stated process), stiff (the sturdy group:
minimum section, and stiffness where stated). Each rung is met, not met or
not measured at that level, and required or reported. It is read from the
grader's own record (`verdict.requirements`, else the checks of each group;
FILE_VALID for the first rung); nothing is re-graded, and it never calls a
part a pass: that stays the grader's verdict, every required rung met. It
shows on the task page (verdict tab), under each part in Compare and as a
row of its summary table. Per model, on Scores ("How far up the ladder") and
on the Compare grid for the level shown: one stepped bar per model, the share
of its parts reaching each rung. A part reaches a rung when it meets it and
every measured rung below; a rung not measured for a part neither counts nor
blocks, and its share is over the parts where it was measured (a rung nobody
measured is a dashed gap, never a zero). Solid bars: required in at least
one of those tasks; light: only reported. The headline figure is unchanged.
Modelling operations are not shown or counted.

**Process versions** (`rules.task_version`). One challenge exists in three
versions, filament printing, powder bed fusion and injection moulding, its id
carrying the process as a suffix (`L2-s3000-fdm`, `-pbf`, `-moulding`; the
stated process is read from `tolerances.manufacturing.process`). The Tasks
page puts the versions of one base id together in one box, each card tagged
with its process, with a link to see them side by side. Compare
`#/compare/<base>?v=<model>` shows one model's versions side by side, linked
cameras, each part with its process drawn: filament its build direction, bed
and the face it stands on; powder needs no direction (the checker states
none); moulding both pull directions and an assumed parting plane. A part
whose record lacks its process's measurements is measured on demand (live).
From any version, the compare page links to the other versions and to each
model's versions view. The suffix naming is assumed until the build session
confirms it; only `PROCESS_SUFFIXES` and the compare page's `SUF` change if
it differs.

## Smooth fields, free orbit, and judging parts against each other

**Fields drawn smooth** (`site/js/fieldview.js`, after the FreeCAD DFM
Workbench's thickness map). Colour is computed per pixel. Sampled fields
(wall, gap, section, displacement) interpolate the field file's per-corner
values (version 2; for version 1 files the same rule is applied here:
area-weighted mean of the face's measured triangles at each corner), so
values never blend across a real edge. Overhang and draft are computed in
the shader from the smooth surface normal and the stated build or pull
direction, so a cylinder shows a continuous band; which faces are judged at
all still comes from the checker, per face. Normals are smoothed within a
face (angle-weighted) and sharp across edges. The limit is a contour line;
the failing side is hatched in screen space. Turbo rainbow by default,
viridis as the colour-blind-safe switch, smooth or banded, "outside limit"
to colour only what fails. A legend across the top of the view: colour bar,
ticks, the limit, the measured minimum and maximum, and two handles that
stretch the scale over a sub-range (drag, or arrow keys). Value tags: the
checker's probe points (worst, lowest, quartiles, highest; tags closer than
4 % of the part merge), and one wherever the reader clicks the field
(interpolated as drawn). The build arrow is small and a fixed size on
screen, beside the part at the bed; the bed is a faint plane; the standing
face is outlined.

**Free orbit** (`site/js/nav.js`). The camera's orientation is a quaternion:
dragging turns the view about the screen's own axes, with no pole and no
clamp; the part can be turned upside down. Alt + drag rolls about the line
of sight; L returns to upright along the same line of sight; the ball's
axis ends still snap to exact upright views. As with any trackball, small
circular drags add roll: about 7° per 40-pixel circle (measured). The
maintainer's own viewer keeps buildings upright on purpose; that part is not
copied here.

**Compare: judge parts against each other** (`site/js/compare.js`). Above
the side-by-side views: *review by* (every check any shown part has,
grouped fits, clean, makeable, sturdy) and a process switch (as graded,
filament, powder, moulding). A check is drawn on every part at once with
one shared scale and legend; under each part, that check's verdict, value
and limit, the setup sentence, and the part's figures: volume, solid mass
in the process's stated material (PLA 1.24, PA12 1.01, ABS 1.05 g/cm³, from
the process files), material against par, bounding box, faces, stiffness
when measured, and pass/fail per requirement group. A summary table below
(parts across, criteria down; one row per check, grouped) marks the best
value per row (only where the parts differ); a click on a check's cell
shows that check in the views; a click on a figure's name sorts the parts
by it. Live, a process the part was not graded under is measured on
demand per part (cached by file hash and process). A static copy has only
what was measured; `jointbench site --precompute fdm,pbf,moulding`
measures every model run's submission into the copy (never into the run
folders), at seconds to minutes per part and process.

**Data still missing for these views** (whose it is):

* Displacement: the stiffness checker writes it, but no attempt in the
  runs has a STIFFNESS check, so the deformed-shape view is untested on
  real data (stiffness checker).
* Section-thickness map (largest ball): written by the field file v2 for
  some processes; STEP.bench's own SECTION_MIN gives only the thinnest cut
  per pair (grader).
* HOLES (measured hole axis), SEATING (contact region), CLEAR_PATH and
  TOOL_ACCESS (overlap meshes), moulding (parting line): as listed above.
* Silhouettes of round faces are drawn from the 1.5 mm checking mesh;
  shading is smooth, but a finer display mesh would round the outline at
  close zoom (manufacturing checker: `sampling.display_spacing_mm`).

## Development data: runs under earlier rules

The maintainer has dropped the model runs made so far as results: the rules
changed (a part must now be makeable to pass, at every level) and the test is
still in development. Those runs stay in use as sample data to build and
check the pages, but nothing presents them as the benchmark's results.

Which runs are which is decided in one place, `src/jointbench/web/rules.py`.
A part counts as graded under the current rules when the grader applied
exactly the requirements its own task states (`task.json` "requirements":
fits always; clean, makeable, sturdy where the task says required), as the
attempt's `verdict.requirements` records them, on a task from a generator
version that states them (connecting parts: version 4 on; other families:
their own generators, no minimum). Nothing is fixed beyond that: a joint task
that requires clean modelling and a connecting-part task that requires
makeable both count by their own statements. Earlier rules: an attempt with
no requirement record (graded before the grader wrote one), or one graded
with other required groups than the task states now. The banner says what
"current rules" means. The label is recomputed from the task files and logs
whenever a results file is served or exported, so a file scored by older code
cannot keep a stale label (round 3's `scores-round3.json` said "makeable not
required" for every joint and mechanism run). A run is current only when every task it ran is. `score`
writes this per task and per run (`rules: {current, why}`); the index, the
scene and the showcase carry it too. When the rules change again, change
`CURRENT_RULES`.

Where it shows:

* **Scores, a run's page, Compare:** a notice above the figures,
  "Development data, earlier rules", with the reasons (generator version,
  makeable not required). Where current and earlier runs are shown
  together, each earlier row also carries a tag.
* **Front page:** the showcase caption and the task count carry the tag.
* **Task page of a run:** the tag next to the run's name.
* **About:** says there are no results yet, and counts the runs kept as
  development data.
* **Static copy:** `jointbench site` leaves earlier-rules runs off its
  scoreboard by default and records how many in `scores.json` as
  `left_out`; Scores and Compare then say "No results yet". Their attempts
  remain on the task pages, marked. `--dev-runs` puts them back, marked.
* **Live (`play`):** every run is shown, marked; this is the working view.

## A hostable copy

`jointbench site <runs> <tasks> --no-reference --out public` writes a folder
any static host can serve as it is (GitHub Pages, Netlify, Cloudflare Pages,
an S3 bucket, `python3 -m http.server`). `--no-reference` leaves the
reference answers out: in a public copy they would give the tasks away.
The copy names no path from the computer it was made on.

Measured on the combined level 2 and 3 model runs (3 models x 7 tasks) plus
the repository's 40 generated tasks, exported in 90 s:

| | |
|---|---|
| total | 148 MB in 1980 files |
| downloads (STEP, STL, 3MF of every fixed part and attempt) | 93 MB; the largest files are attempt STLs of up to 1 MB |
| manufacturing colour maps (3 attempts) | 11.7 MB, 3.5-4.4 MB each |
| meshes for the 3D view | 14 MB |
| the site itself (scripts, styles, three.js) | 1.6 MB, of which three.js 1.27 MB |
| compressible text, if the host compresses | 116 MB -> about 31 MB |

Load times from a plain file server (no compression), empty browser cache:
front page text 0.06 s and first 3D frame 0.15 s on this computer; over
about 10 Mbit/s with 60 ms delay, text 0.7 s and first 3D frame 2.3 s, a
task page's parts 1.9 s, the scores page 0.65 s. A host that compresses (the
common static hosts do) brings the network figures close to `play`'s:
about 1.1 s for the first 3D frame.

**Only with `play`, not in a static copy:** grading uploads (*Try it
yourself* shows how to run it locally instead); starting a manufacturing
check (colour maps already made before export are included and work);
following new attempts live. A STEP the mesher cannot triangulate gets no
STL or 3MF (four attempts in these runs); the export says which, the
download menu shows those formats as unavailable, and the STEP is offered
as always.

Ways to make it smaller, if a host limits size: leave attempt downloads to
STEP only (the three formats of every attempt are most of the size), or
leave the colour maps out.

## Caching: no stale pages

A browser that kept an old script while fetching a new page showed an old
interface. Now the page names its scripts and styles by a fingerprint of
their content: `v/<12 hex>/js/app.js`. The fingerprint changes whenever any
file under `js/`, `css/` or `vendor/` changes, so a new page can only load
the new files. `play` serves those versioned files as cacheable for a year
and the page itself (`index.html`) and all data as never cached. The static
export writes the files once under `v/<fingerprint>/` and an `index.html`
that points there, so any host can cache them freely. The fingerprint is
computed by `src/jointbench/web/assets.py`.

## How it is put together

```
site/                     the front end, no build step
  index.html
  css/style.css           colours as tokens, dark (default) and light
  js/app.js               pages, tabs, attempts, upload, live updates
  js/scores.js            scores page and one run's tasks
  js/charts.js            the score charts, plain SVG
  js/fetch.js             loading data; meshes kept for the visit
  js/report.js            reading the grader's report (no 3D)
  js/timeline.js          the attempt timeline, report text, code diff
  js/dfm.js               "Made how?": manufacturing fields, legend, direction
  js/quality.js           material figure (prepared), median
  js/icons.js             the line icons
  js/nav.js               navigation: orbit, pan, zoom, views, projection
  js/clip.js              section planes, clipping box, hatched caps
  js/measure.js           point-to-point measuring with snapping
  js/thumbs.js            3D pictures for cards and the compare grid (one shared context)
  js/compare.js           compare grid and compare page
  js/hero.js              the front page's showcase of real attempts
  js/viewer.js            the 3D drawing (three.js, Z up, orthographic camera)
  js/md.js                renders the task brief (README.md)
  vendor/three/           three.js r160, unmodified except one import path; MIT licence kept
                          (OrbitControls.js is vendored but no longer used: navigation is nav.js)
src/jointbench/web/
  mesh.py                 STEP -> mesh JSON (triangles + B-rep edges), cache by sha256
  data.py                 the JSON documents the page reads
  server.py               `play`: standard-library HTTP server, upload grading, live log
  export.py               `site`: the same documents written to a folder
  downloads.py            STEP / STL / 3MF of the fixed parts and each attempt
  score.py                `score`: counts per run from the attempts logs
  assets.py               the fingerprint that versions scripts and styles
tests/test_web.py
```

The live server and the static export produce the same JSON at the same
paths, so the page has one way to load data:

```
data/site.json                  live or static
data/index.json                 every task, for the front page
data/tasks/<id>/scene.json      brief, task.json, fixed-part meshes, faces and bolts, per pose
data/tasks/<id>/attempts.json   attempts.jsonl, with a mesh link per attempt
data/tasks/<id>/marks/<n>.json  overlap regions of a failed attempt
data/tasks/<id>/files/<name>    the files a person needs (never _solution/)
data/mesh/<sha256>.json         one STEP file, meshed
data/scores.json                results of `score` (when there are runs)
data/tasks/<key>/archive/<n>/<file>   archived code and text of attempt n
data/tasks/<key>/dfm/<n>.<process>.json   manufacturing field file of attempt n
data/tasks/<key>/download/<name>          fixed_<X> or attempt-<n>, .step / .stl / .3mf
data/tasks/<key>/thumb.json               the fixed parts as coarse meshes (card picture)
data/tasks/<key>/thumb/<n>.json           attempt n's part as a coarse mesh (grid picture)
data/showcase.json                        attempts for the front page showcase
```

A task from a runs folder has the key `<run>__<task-id>`, because the same
task appears once per model.

`play` additionally answers `POST api/upload?task=<id>&who=<name>` (body: the
STEP file), `POST api/dfm?task=<id>&n=<n>&process=<p>` (runs the
manufacturing checker) and `GET api/events` (Server-Sent Events: one message each time an
attempts log grows).

**Meshes are stored by the content of the file** (its sha256, which the
attempts log already records). An agent usually overwrites `output.step` with
each try; the server meshes every new attempt the moment its log line appears,
so earlier tries stay replayable. An attempt whose file changed before it was
meshed keeps its verdict and shows no shape, and the page says so.

`play` keeps uploads and the mesh cache in a `.play/` folder inside the task
folder (or the folder of tasks). Delete it at any time; it is rebuilt.

**Overlap regions** are the one piece of geometry the viewer computes rather
than reads: the grader reports only the volume of each overlap, not where it
is. For rows the grader already reported over the limit, `data.marks` rebuilds
the intersection with the grader's own keep-out shape and draws it. It cannot
change a verdict; it only shows where one already failed.

## What the grader output does not give (for better marking)

The viewer works from `verify --json` as it is. These would make the 3D
marks more exact without the viewer re-measuring anything:

- **pose index per row.** `HOLES`, `SEATING` and `NO_INTERFERENCE` rows carry
  no pose number. With one pose this does not matter; with several, the
  viewer infers the pose of overlap rows from their order and cannot place
  per-bolt hole rows at all.
- **location of each overlap** (centroid or bounding box). The viewer
  rebuilds the intersection to show it; a reported location would avoid that.
- **the hole that was found**: its measured axis point and direction. Today a
  hole 0.25 mm off its bolt is marked on the bolt's nominal axis; with the
  measured axis both lines could be drawn.
- **seat face location and stand-off per bolt** in structured form (it is only
  in the reason text), to draw the gap under a lifted seat.
- **structured problem records** (`{bolt, check, kind, value, limit}`) instead
  of strings: the viewer finds the bolt by reading the text before the colon.

## Manufacturing check ("Made how?")

The *Made how?* tab shows the project's manufacturing checker
(`src/jointbench/dfm/`, documented in `docs/dfm.md`) for the selected
attempt: one row per rule with pass/fail, reason and limit, and the build
or pull direction it used. Clicking a rule colours the part by that rule's
measurement.

- **Data**: the checker's field file (`--fields`, format in `docs/dfm.md`,
  "Field file"). The viewer draws the checker's own mesh with its values per
  triangle and judges nothing itself. The tab says plainly that this result
  does not change the joint grader's verdict.
- **Colour**: viridis (dark purple to yellow), readable for colour-blind
  viewers and in grey, on the checker's own scale choices (angles on a fixed
  range, thicknesses capped at a few times the limit, "≥" when capped).
  Failing triangles are striped black and white over their colour; plain
  light grey means "not measured here"; region rules mark their region in
  orange. The legend shows the limit as a heavy line with the failing side
  hatched. A blue arrow shows the build or pull direction, and the worst
  place is labelled. The fixed parts turn to ghosts while a field is shown.
- **Where results come from**: a `*.<process>.fields.json` in the attempt's
  archive, or one the viewer made earlier. With `play`, a *Run it* button
  runs `python -m jointbench.dfm <file> --process <p> --fields` on the
  graded file (about 20 s on a level 1 bracket; one at a time) and keeps the
  result in `.play/dfm/<sha256>.<process>.fields.json`. The static export
  copies results already made and never runs the checker.
- A field file is about 3.5 MB for a level 1 bracket; the export carries one
  per attempt and process that was checked.
- Not drawn yet: the `cut` hint for sealed hollows (open the part there).
  The section plane can be used instead.

## Levels beyond 1

**Level 2** (faces at an angle, offset, in an arbitrary placement) needs
nothing special: faces, bolts and checks are drawn from the data on any
axis. The overview describes two faces that do not face each other by the
angle between them and their distance ("faces 70° off, 76 mm apart")
instead of a gap, and can be filtered by level and by model run. Checked on
the first level 2 model runs (three models, three tasks each, archived
attempts): scores, run pages, timelines with the agents' own code diffs,
and the manufacturing panel.

**Level 3** (third fixed part, obstacles, tool access, `CLEAR_PATH` and
`TOOL_ACCESS`, difficulty values per task) is not built yet, because its
`task.json` format is not committed. What already works without change:
any number of fixed parts (drawn, listed and counted from `task.json`),
any check name the grader reports (listed in the verdict and the timeline,
counted on the scores page), and failing bolts marked whenever a check's
problems name the bolt the usual way (`A1: ...`). What waits for the
format: obstacles as their own layer with a distinct look and a toggle,
tool envelopes as an optional layer per bolt, marking the touched obstacle
and the blocked envelope, and the difficulty values in the side panel and
on the task cards.

## Limits

- **No in-browser code editor.** Uploading a STEP is the way in; running
  build123d code typed into a web page would mean executing arbitrary code on
  the host, which is not worth it for a local tool.
- **Labels sit on top of everything**, so a bolt label shows even when its
  bolt is behind a part. Section and ghost help.
- **The section cap is a flat colour**, not hatched.
- **Static copies show relative times** ("3 days ago") from the visitor's clock.
- **`play` serves on 127.0.0.1 only by default.** `--host 0.0.0.0` exposes it
  to the network, including grading of uploaded files; there is no login.
