# "Locks" (level 4d) -- prototype

> **Ladder of 15 November 2026:** this prototype is now **L5 Locks**, on the pipe setup in version 1: the model's two pieces, one on the plate and one on the pipe, join each other by push-and-turn inside the box and carry the load. The text below describes the prototype as it was built ("level 4d").

Next to 4a assembles, 4b adapts and 4c moves. Built on level 2: each part meets everything of levels 1 and 2
(fit, clean modelling, bolts reachable by a straight tool and turnable with their hex key (`KEY_TURN`),
filament printing). Once locked, B carries a stated pull-out load, so 4d is a load test: scored, among answers
that pass, by the printed weight of both parts (level 3's measure, as at 4a and 4b).

Two fixtures face each other. The brief states an axis square to the faces, that B comes in straight along
it, and an angle (30-90 degrees, either way) through which B then turns. The model designs one rigid part on
A and one on B so that B goes in freely, turns freely, is then locked (it cannot be pulled back out), and is
not locked before the turn (it would still come straight out).

```
python -m jointbench.locks gen --seed 0 --out DIR      # a task folder; its reference is graded first
python -m jointbench.locks verify DIR                   # grades DIR/part_A.step and DIR/part_B.step
python -m jointbench.locks check DIR                    # the reference and every broken copy in DIR/_solution
python -m jointbench.locks view DIR OUT                 # STL + motion.json + index.html: push, turn, pull
python -m pytest tests/test_locks.py
```

Code: `src/jointbench/locks/` (generate, brief, grade, view); family `locks`. It reuses level 4c's motion,
distance, small-move and swept-volume code (`jointbench.moves.motion`, `.sweep`, `.grade`).

## What the model hands in

`part_A.step` and `part_B.step`, one solid each, in world coordinates with B at its start pose (pushed in, not
turned). No assembly file: the axis, the insertion direction and the turn are the task's.

## How it is graded

Each part against its own fixture: holes, seating, seat inside, no interference, clean modelling, filament
printing, and `TOOL_ACCESS` (a straight 30 mm tool reaches every bolt; each part is bolted before B comes in,
so only its own fixture is in the way). Then, with A's side = A + part_A + its bolt heads and B's side the same
for B, all exact:

| check | passes when |
|---|---|
| `SEATED` | at the start pose the two sides do not overlap |
| `CONNECT` | B's side, brought in from outside along the insertion direction, sweeps through nothing of A's side (exact swept volume) |
| `TURN` | turned 0 to the stated angle in steps of at most 2 deg and 1.5 mm, it never overlaps |
| `GAP` | at every step of the turn the sides stay 0.3 mm apart |
| `LOCKED` | turned, pulled back along the axis, B's side meets A's within 0.8 mm |
| `UNLOCKS` | not turned, pulled back 5 mm, it meets nothing: only the turn locks it |
| `INTERLOCK` | turned, shifted 5 mm in any of 26 directions or tilted 15 deg, it meets A's side |
| `PLAY` | turned, free shift at most 0.8 mm along and square to the axis, free tilt at most 5 deg |
| `LOCK_STRENGTH` | turned and pulled back with the stated load, the plastic where the parts bear carries it (below) |

**The load.** Each task draws a mass of 0.5-1.5 kg (`requirements.draw_stated_load`, as the other load levels)
that pulls B straight back once locked (x 9.81, as the stiffness analysis turns a mass into force). It is
judged as 4a judges its joint, in the plastic, with the same allowables (`catalogue.plastic_allowables`: 20 MPa
bearing, 11.5 MPa shear, from 40 MPa printed PLA over a safety factor of 2). The contact patches are found
exactly: part_B pulled back 0.05 mm past first touch, its overlap with part_A. Bearing: the load over their area
projected square to the pull. Shear: on each part, each patch's share punching a plug of the patch's area out
through the plastic straight behind it (as a bolt head punches through at 4a), the outline taken as round (the
shortest outline for its area). The largest ratio is the utilisation.

At these loads the check does not bite on a sound lock: the reference on seed 0 is at 0.017 (1 kg against
about 60 kg of capacity), as 4a found for its joint. The score -- the lightest passing answer -- is where the
load matters: a lean lock wins. The `overloaded` broken copy (the reference graded under 100 kg) shows the
check failing.

## The hidden reference: a bayonet

A tube on A's plate, on the axis, with an L-shaped groove inside its bore for each of two or three lugs: a
straight channel down from the top, then a channel round the wall through the turn. A post on B's plate comes
down the tube carrying the lugs. Running gaps 0.4 mm.

**Why the locking faces slope.** Printed without support, a face may overhang at most 45 degrees and a
bridge may span at most 10 mm. A flat slot ceiling would be a bridge as long as the turn (20 mm and more) and
fail. So the lugs' tops and the ceilings above them slope at 45 degrees, falling away from the axis, and the
slots are grooves inside the bore with a 1.5 mm skin outside, not cut through the wall. A slope rising away from
the axis, or a slot through the wall, leaves a 45-degree knife edge about 0.2 mm thick, which fails the 0.8 mm
minimum wall; this way every edge is at least 135 degrees. A pull then closes the 0.4 mm gap
(measured square to the faces) after 0.4 x sqrt 2 = 0.57 mm of travel; the grader measures 0.56 mm on seed 0.
That is why the play limit for 4d is 0.8 mm, not 4c's 0.6 mm: a printable lock with the 0.3 mm minimum gap
already has about 0.42 mm of pull-back play.

Draws are kept only if the tube fits inside A's face between A's bolt heads (outer radius at least 11 mm),
the post clears B's bolt heads, each part fits the Prusa MINI with 20 mm to spare, and the reference passes the
real grader with the printing rules.

## Broken copies (in `_solution/broken/`)

| copy | change | must fail |
|---|---|---|
| collides-while-turning | a block in one turning channel, partway round | `TURN` |
| pulls-out-after-turn | a second straight channel where each turn ends | `LOCKED` |
| locked-before-turn | the straight channels at the turn's end instead of its start | `UNLOCKS` |
| gap-too-small | the post 0.1 mm clear of the bore | `GAP` |
| overloaded | the reference itself, graded under a 100 kg pull | `LOCK_STRENGTH` |

## Limits and open choices

- The axis is square to the faces; a tilted axis would be a small generator change.
- The turn is checked in steps; a feature thinner than about 1.5 mm could slip through between them.
- `LOCKED` pulls straight back; a lock that only holds against a straight pull but lets B out along a curved
  path (turn back a little while pulling) passes. Turning back is how it unlocks, so that is by design.
