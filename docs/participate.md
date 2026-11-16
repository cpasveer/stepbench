# Taking part

*This page describes the planned ranking rounds. Version 1 is a proof of
concept: one public seed per test, run by the maintainer, results at
[stepbench.dev](https://stepbench.dev).*

STEP.bench has two kinds of seeds, which differ in who knows them, and
several tracks, which differ in what varies between runs.

## Tracks: one variable at a time

A comparison means something only when one thing changes. So every round
belongs to one track, and its brief says which.

| track | fixed | varies | answers |
|---|---|---|---|
| **model** | the CAD tool (build123d) | the model | which model designs best; **the main ranking** |
| **tool** | the model | the CAD tool: build123d, FreeCAD over MCP, Rhino over MCP, ... | which CAD program agents work best in |
| **open** | nothing | any tool installed in the agent's sandbox | the best result by any means |
| **human** | | a person at any CAD program | how people compare (below) |

Each track publishes the list of tools installed in its sandbox, so a
result can be repeated. The tool actually used is recorded on every
attempt.

No ready-made viewer is given in any track. How an agent looks at its own
part, if it does (a script that renders images, a CAD program's own
screenshot, measuring instead of looking), is part of what is tested. What
it did is recorded as an observation, next to tool, tokens and time (see
*What is recorded*).

## Showcase seeds and ranking seeds

Every task is generated from a seed, and from the seed alone: the same seed
always gives the same task, and the same hidden reference that proves it
solvable. The seeds come in two kinds.

**Showcase seeds** are public. For each setup and level of a version there
is a fixed, published set: the tasks to practise on, to develop an agent or
a harness, and to compare results openly. Anyone can run them as often as
they like. Nothing stops someone from tuning to these exact tasks; that is
why they are not the ranking.

**Ranking seeds** are private. For each round the maintainers draw new
seeds, and the generator draws each task freely within its setup: other
fixture kinds, sizes, angles and orientations than the showcase tasks. No
ranking task exists before its round. The seeds are **published after the
round**, with the results, so anyone can regenerate every ranking task and
regrade every part.

- **Five runs per test, each a new task.** A model runs every test five
  times, each time on a task from a different ranking seed.
- The participant's copy of a ranking task holds no seed and no reference:
  with the public generator, a seed would be enough to rebuild the
  reference. The grader outside the box keeps the full task.
- Ranking results are made by the maintainers in the offline box (below)
  and shown as *verified*.

## Submitting a result

Not open yet. Every published result is made by the maintainers. Submitting
results on the showcase seeds will open later.

## The human track

People take part with any CAD program. They get the same tasks and the same
grader as models: upload a STEP, get the same verdict.

- Same six-check limit per test.
- Time is measured by the grader's server from the moment the person starts
  the task (their first download, or the start button) to their first pass, never reported.
- Shown on its own board, and on the main board marked *human*.
- On ranking seeds, a person gets one test at a time with the clock
  running, through the issue-and-collect service described below.

## Rules for every run

The benchmark tests the model's full creativity, so the rules are few.
Agents may use, and **build**, any tools they like: viewers, measuring
scripts, their own stiffness estimates, any installed CAD program. Within
the track's CAD tool, everything is allowed except:

1. **No access to the answer.** No reference answer, no other runs, no
   other participants' parts.
2. **A limited number of official checks.** At most 6 per test. A test whose
   part never passed counts as not passed. A test that was issued and never
   handed back counts as not passed: no picking the good ones.
3. **Everything is recorded.** The CAD tool used, the tools the agent built
   for itself, tokens in and out, cost at the list price of a stated date,
   and wall-clock time.

The CAD tool is the track's: build123d on the model track, the round's
tool on the tool track, anything installed on the open track.

## Official runs happen in an offline box

*Decided by the maintainers, 13 November 2026.*

The grader is open source. An agent with internet access could download it
and check its part as often as it likes, which would make the six-check
rule meaningless; so would a grader installed next to the agent. So every
official run happens in a closed box:

- **no internet**;
- the track's CAD tools **pre-installed**, and the list of them published;
- **no grader inside**: the only way to check a part is the counted
  `verify` command, which hands the part to a grader outside the box;
- inside the box the agent may still **build any tools of its own**.

**Practice runs on the showcase seeds may be open**: internet, a local grader,
any number of checks. Their results are marked *practice* and never enter
the official ranking.

What the offline box gives up: looking things up during a run (a CAD
library's documentation, an ISO table), and online CAD programs. The brief
states every number needed, and each track's box carries the documentation
of its own tools.

### How the box is built (sketch)

```
 maintainers' machine
 ┌───────────────────────────────────────────────────────────────┐
 │  harness (the agent loop)  ── model API ──>  model provider   │
 │     │ runs each tool call inside the box                      │
 │     v                                                         │
 │  ┌─────────── box: container, network off ───────────┐        │
 │  │  CAD tools of the track, their docs, Python        │        │
 │  │  /task  (the clean task folder, mounted)           │        │
 │  │  verify  -> writes a request into /task/.outbox    │        │
 │  └────────────────────────────────────────────────────┘        │
 │     ^ answer          | request                               │
 │  grader (outside): counts, refuses a 7th, grades, archives    │
 │  clock, token meter, transcript log                           │
 └───────────────────────────────────────────────────────────────┘
```

1. **The image.** One container image per track: the operating system,
   Python, the track's CAD tools (build123d for the model track; FreeCAD,
   OpenSCAD and so on for the others) and their documentation. Not the
   benchmark's package. Built once per version and published with its list
   of contents, so anyone can rebuild the same box for practice.
2. **Network off.** The container starts with networking disabled
   entirely. The agent loop, which needs the model's API, runs outside the
   box and only executes its tool calls (shell commands, file writes)
   inside it. So the box itself never needs the network.
3. **The task folder mounted.** A fresh, clean copy per test: brief,
   `task.json` without seed, fixed parts. No reference, no other tasks.
4. **`verify` is a call out.** Inside the box, `verify` is a small script
   with no grading code. It copies `output.step` and the `--tool` value to
   a request folder shared with the outside, and waits. Outside, the
   grader (with `JOINTBENCH_MAX_CHECKS=6`) reads the request, counts it,
   grades the part against the full task, archives it, and writes the
   feedback back. At six it answers with the refusal. The count, the
   archive and the full task live where the agent cannot write.
5. **Recorded outside.** Before the agent starts, the launcher runs
   the environment probe (`jointbench env`, as a standalone script with no
   grading code) inside the box and keeps the environment record, with
   the image's digest, outside. The harness keeps the clock, reads tokens from the
   provider's usage report, and keeps the transcript, from which tools
   built and visual inspection are filled in.

Who runs the box is the remaining choice, below.


## What is recorded

| figure | how | where |
|---|---|---|
| **environment record**: model and provider, CAD programs and add-ons, Python packages, benchmark version, tasks, box digest | captured by the machine at run start (`jointbench env`, [environment.md](environment.md)) | per run |
| verdict, every check and its number | measured by the grader | every attempt |
| CAD tool | declared (`--tool`); compared with the program the STEP file's header names: consistent, inconsistent or cannot tell | every attempt |
| tokens in and out, cost, price date | reported by the harness from the provider's usage report | per run |
| wall-clock time | the harness's clock, or the grader's for the human track | per run |
| **tools built** | observed from the transcript: scripts and helpers the agent wrote for itself (a renderer, a measuring script, a stiffness estimate), what each was for | per run |
| **visual inspection** | observed from the agent's transcript and tool calls: did it render or look at its model, how (own render script, CAD screenshot, image of a section, none), how often, and did it look at the images it made | per run |

Visual inspection is an observation, not a rule and not a score: it
explains results (does looking help?) without rewarding or punishing it. It
can be filled in by reading the transcript; a harness that logs tool calls
can fill it in automatically by noting image files written and image files
opened. *To do:* a field for it in the run record (`run.json`).

---

## Who runs the box

*For the maintainers to decide.* With the box decided, the question is whose
machine it runs on.

### A. The maintainers run it

The participant hands in an agent: a harness script, or a container that
plugs into the box. The maintainers run it on their machine against freshly
generated tasks.

- **For:** nothing can be faked: the box, the count, the clock and the token
  count are all the maintainers'. Ranking tasks stay unseen because the
  participant never sees a task.
- **Against:** the maintainers pay for compute, and for model calls unless
  the participant supplies an API key with a budget. Each harness takes
  some work to plug in.

### B. The participant runs the box; a service issues and grades

The participant runs the published box image on their own machine. A
small service issues each test (fresh from the secret, without seed),
starts a clock, receives the box's `verify` requests over the network from
the participant's harness, counts and grades them, and counts a test never
handed back as not passed.

- **For:** cheap for the maintainers; any participant can run official tests
  whenever they like.
- **Against:** the box runs on the participant's machine, so whether it was
  really offline, and the token and cost figures, are trusted, not
  enforced. Ranking tasks reach participants one at a time and could be
  collected.

### Recommendation

**A for the official ranking, B for anyone who wants verified checks
without the maintainers' involvement**, marked *checked by the service*
rather than *official*. A is the only arrangement in which the offline
rule is enforced rather than trusted, which is what the decision asks for.
The bill stays small because official runs are few: one per model per
version.

The **human track** uses the service of B: a person cannot be put in a box,
so the count and the clock are enforced on the server, and the offline rule
does not apply to people (a person may look up an ISO table).


## What has to change in the code

Small pieces, none started yet except the first:

1. **Check limit in the grader.** Done: `verify` refuses a seventh check
   when `JOINTBENCH_MAX_CHECKS` (or `--max-checks`) is set.
2. **A clean copy for handing out.** A command that writes the task folder
   a participant gets: no `_solution/`, no seed in `task.json`, a neutral
   task id. The grader keeps the full task.
3. **The box**: the image per track, the `verify` call-out script and a standalone environment probe inside,
   and the grader's side that answers requests and counts.
4. **The grading service** of option B and the human track: issue,
   collect, grade, record.
5. **The single front door** `jointbench gen --setup <facing|flange|pipe>
   --level <1-6> --seed N`, so every showcase task can be generated from the
   public repository (being built).
