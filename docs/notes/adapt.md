> Working notes for levels 4–6 (planned for v2). They may be out of date and are not part of v1's claims.

# Adapts (level 4b): one part for two arrangements (prototype)

> **Ladder of 15 November 2026:** Adapts is parked for version 2 and is not part of the version 1 ladder. The text below describes the prototype as it was built ("level 4b").

The benchmark level "adapts", level 4b in the structure of 14 November: 1 fits, 2
printable, 3 strong, then 4a assembles, 4b adapts, 4c moves. Each builds on what is below it, so
adapts includes all of level 3, in both arrangements. Code and task ids use the name, not the
number.

Status: working prototype, angled mounts. Not yet run on any model.

## What the model gets

Two complete arrangements of fixed parts, each in coordinates of its own:

- arrangement 1: fixed parts A and B, their mounting faces at an angle to each other;
- arrangement 2: fixed parts C and D, at a different angle and a different distance.

The angle between an arrangement's two faces is drawn from 55-78 or 102-130 degrees (never near
parallel, never near square); the two arrangements' angles differ by at least 15 degrees and their
mount distances by at least 10 mm. So the part needs four mounting faces at unrelated angles.

In each arrangement one fixed part carries a stated mass (0.5-1.5 kg, as at level 3) with the
other held. The part is printed in PLA without support, once.

The brief states both arrangements in full (every face, every bolt, each arrangement's load, in
that arrangement's own coordinates), the print process as a fact, and that printed weight is
measured and lighter is better. It says nothing about how the arrangements relate, nor the angles.
Both coordinate frames are turned at random: no mounting face within 18 degrees of a coordinate
plane.

The model hands in `output.step` (one solid, in coordinates of its choice) and `placement.json`:
per arrangement a rotation (3 x 3, row by row) and a translation, taking the part into that
arrangement's coordinates. Rigid moves only.

## How it is graded

- **The part itself, once, as handed in:** a valid solid, clean modelling, and the filament
  printing rules. Printing is judged on `output.step` as handed in because the printing checker
  tries the part's own axis directions and largest flat faces as print directions: placing the
  part first would change its verdict.
- **Each arrangement:** the part is placed as declared and checked against that arrangement's
  fixed parts and bolts: holes, seating, seat inside the face, no interference, every bolt
  turnable with its standard hex key (KEY_TURN), and the stated load (stiffness in PLA). All of the
  part is present each time.
- The part passes only if all of that passes. **Score:** printed weight (shell and infill as the
  printer lays them), lower is better.

The report has a section for the part itself and one per arrangement, for example
`arrangement 2 (fixed parts C, D): the part fails: HOLES` followed by
`C1: hole axis 0.212 mm off the bolt axis`.

## How a task is made (working backwards)

The hidden reference is a faceted shell, open underneath, that prints standing on its rim. In the
part's own frame: four mount wall directions are drawn (A and B at arrangement 1's angle, C and D
at arrangement 2's) plus a few closing walls; every wall slopes at least 47 degrees from level,
so neither the outside nor the inside of the shell overhangs. Walls are 6 mm; each mount is a pad
5.5 mm proud of its wall, cut to its fixture's face; the bolt heads sit inside the shell, reached
through the open underside. The construction repairs what would not print: a skirt wall under any
upward-facing wall that reaches the rim, a 5 mm chamfer on any outside edge sharper than square,
straight cavity sides for the last 4 mm above the bed.

A draw is thrown away if a pad does not fit its wall, a bolt head has no room inside, the
fixtures of an arrangement overlap, two downward faces meet in an edge too flat to print, the
arrangements are trivially alike or as far apart, the part does not fit a Prusa MINI with 20 mm
to spare, or the reference fails any check (cheap checks first, the two load analyses last). A
task takes about a minute to generate.

The first task (seed 0): faces at 59 degrees and 53 mm apart, 1.2 kg on B; 116 degrees and 92 mm,
1.1 kg on D. Reference: 133 g printed, stiffness 172 against 56 N/mm needed (arrangement 1) and
40 against 29 N/mm (arrangement 2).

## Where it lives

- `src/jointbench/families/adapt/` -- `angled.py` (generator), `grading.py` (placement file, the
  part's own checks, per-arrangement grading, report), `brief.py`.
- Core changes, small: a stored pose may place only some fixed parts (`task.py`); a family may
  grade its own way (`Family.grade`, one dispatch line in `grade.py`); the workspace writes each
  fixed part from the pose that places it and the reference placement beside the reference; the
  attempt archive keeps `placement.json`.
- Generate: `jointbench gen --family adapt --level 1 --seed 0 --out <dir>`.

## Limits of the prototype

- Mounts are small (bolt-pattern radius plus head radius at most 16 mm: in practice 2-bolt rows),
  so that four pads fit on a shell small enough for the printer.
- The reference is always a shell of this kind; a model need not find that.
- No "alternative body" knife-edge test (level 1 keeps a draw only if a second body also passes).
- The bolts are put in by reaching into the shell from underneath; the hex key's long leg swings
  out through the open underside (KEY_TURN passes in both arrangements on seed 0).
