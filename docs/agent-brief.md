# The agent brief

What an agent is told before a test, and which of the rules are enforced.

The principle: **the brief states only rules the setup enforces.** A rule
that rests on the agent's good behaviour is not a rule of the benchmark; it
is a hope. Round 5 told agents in words to stay in their folder and to run
the grader at most six times. That worked, but nothing would have noticed
if it had not. From now on, a rule is either enforced or left out.

## The template

Fill in the `{...}` fields. The two lines in brackets `[...]` are facts in
the offline box of official runs and always appear there; in a practice
run, leave out any of them your setup does not make true.

```
You are taking a CAD design test. Your task is in this folder:

{FOLDER}

Read README.md there first; it is the brief. task.json holds every number.
Model the part and save it as output.step in that folder.

{TOOL_LINE}
You may build any tools you need for yourself, such as scripts that render,
measure or analyse your part.

To check your part, run:

    {VERIFY} {FOLDER} --by {WHO} --tool <the CAD tool you used>

The grader allows 6 checks for this test and refuses any after that. A
check can take a few minutes. Only output.step is graded; the last check
is your result.

[This folder is all you can see; there is nothing else to find.]
[You have no internet access.]

The brief lists every requirement. Beyond them, make the best part you can,
as an experienced mechanical designer would.

When you are done, say in at most five sentences whether your last check
passed, how many checks you used, which CAD tool you used, and what shape
you made and why.
```

`{TOOL_LINE}` depends on the round's track (see
[participate.md](participate.md)):

- model track: `Model it with build123d, using {PYTHON}.`
- tool track: `Model it with {TOOL}, available as {HOW} (for example: the FreeCAD MCP server).`
- open track: `You may use any CAD program, library or method installed here that writes a STEP file: {TOOL_LIST}.`

Nothing in any track offers a ready-made way to look at the part. The agent
may build any tool it wants (a renderer, a measuring script, its own
stiffness estimate); whether and how it does is part of what is tested, and
is recorded from its transcript as an observation. The template's second
paragraph says so.

What changed from round 5 (`agent_prompt.txt`):

- **The tool follows the track.** build123d is the tool of the model track,
  not of the benchmark. The starting script in the folder is optional.
- **The tool is recorded.** `--tool` goes on every check, so each attempt
  says what made it.
- **No rule that is not enforced.** "Do not read anything outside the
  folder", "no web access" and "at most six times" were instructions. The
  six-check limit is now enforced by the grader itself and stated as a
  fact. The other two appear only when the setup enforces them, and then as
  a fact ("this folder is all you can see"), not a request.
- "Never kill processes by name" and "keep your final script as
  submission.py" are dropped: the first is a protection for the machine
  running the test and belongs to the harness, the second is not needed
  because every graded file is archived by the grader.

The brief inside the folder (`README.md`, written by `jointbench gen`) now
says the same: model in any CAD program, `submission.py` is an optional
build123d starting point, run `verify --tool`.

## What enforces each rule

| rule | enforced by | status |
|---|---|---|
| at most 6 checks per test | `verify` refuses a 7th when `JOINTBENCH_MAX_CHECKS=6` is set in the agent's shell; it also refuses an unlogged check and two checks at once | **built** |
| only `output.step` is graded, and every graded file is kept | the grader's archive | built |
| the reference answer is not available | the folder handed out has no `_solution/`: `gen` writes it into every task folder, so delete it before the agent starts | built for round folders; in `gen` by hand for now (a `--no-solution` switch is proposed) |
| the generator's answer cannot be rebuilt | no seed in the handed-out `task.json`, or no generator in reach | **to do** |
| only the task folder is visible | the offline box: a container with only that folder mounted | decided; to build |
| no internet | the offline box: networking disabled; the agent loop runs outside | decided; to build |
| no unlimited self-checking | no grader inside the box; `verify` calls out to the grader outside | decided; to build |
| the time | the grader's clock (human track) or the harness's | harness |
| tokens and cost | the model provider's usage report, read by the harness | harness |

## Practice runs and official runs

**Official runs: the offline box.** *Decided by the maintainers, 13 November
2026.* No internet, the track's CAD tools pre-installed, no grader inside:
`verify` in the box is a small call-out that hands `output.step` to the
grader outside, which counts, refuses a seventh, grades and archives. The
count, the clock and the archive live where the agent cannot write, and the
open-source grader cannot be downloaded. Inside the box the agent may build
any tools of its own. How the box is built is sketched in
[participate.md](participate.md#how-the-box-is-built-sketch).

**Practice runs on the public set: open.** Any shell, internet allowed, the
grader installed locally. Setting `JOINTBENCH_MAX_CHECKS=6` makes the local
grader refuse the seventh check, so an agent that follows the brief cannot
exceed it by mistake; an agent that wants to could get round it (unset the
variable, import the grader directly, delete its record). That is accepted:
practice results are marked *practice* and never enter the official
ranking.

## What `verify` does with a limit

With `JOINTBENCH_MAX_CHECKS=N` set (or `--max-checks N`):

- it counts the graded attempts already on record for the task, the larger
  of the log lines and the archived attempt folders, so emptying one does
  not give checks back;
- at N it refuses before measuring anything, so a refused check gives no
  feedback at all (exit code 4);
- it refuses `--no-log`, because an unlogged check could not be counted;
- it holds a lock on the task folder while grading, so two checks started
  at once cannot both slip under the limit (not on Windows);
- a grading the grader itself could not complete (a crash in a check) is not
  an attempt and is not counted, as before. *Maintainer's call:* count these
  too if agents are found provoking them for free feedback.

Without the variable, `verify` behaves exactly as before. In the offline
box this same grader runs outside, with the variable set.
