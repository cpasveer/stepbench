> Working notes for levels 4–6 (planned for v2). They may be out of date and are not part of v1's claims.

# "Moves" (level 4c) -- prototype: linkage synthesis

> **Ladder of 15 November 2026:** this prototype becomes **L6 Moves**, on all three setups in version 1: inside the box a linkage on catalogue ball bearings or bolt axles carries B through the required positions into a latch that locks it and holds the load. The text below describes the earlier prototype ("level 4c", three poses, no load), which L6 replaces.

The third of the level 4s (4a assembles, 4b adapts, 4c moves). It builds on level 2: every part meets
everything of levels 1 and 2 (fit, clean modelling, bolts reachable and turnable by a straight tool,
filament printing), plus the assembly and the linkage motion. No load.

Two fixtures face each other, as at level 1. Fixture A stays put. Fixture B starts where the task places it
and must be carried, by one continuous motion of a mechanism the model designs, through **three more stated
placements, in order**. No axis is stated, and the poses are chosen so that no single hinge can do it: the
model has to find a linkage -- typically a four-bar, its pivot places and link lengths -- from the poses
alone. The parts are rigid and assembled; nothing is loaded.

```
python -m jointbench.moves gen --seed 0 --out DIR        # a task folder; its reference is graded first
python -m jointbench.moves verify DIR                     # grades DIR/assembly.json and the parts it names
python -m jointbench.moves check DIR                      # the reference and every broken copy in DIR/_solution
python -m jointbench.moves view DIR OUT [--file X.json]  # STL + motion.json + index.html: the answer moving
python -m pytest tests/test_moves.py
```

Code: `src/jointbench/moves/` -- `generate` (the hidden four-bar and the task), `brief`, `grade`,
`linkage` (mobility and posing), `assembly` (the answer's file, catalogue screws and nuts), `motion`
(distances and small moves), `sweep` (level 4's exact swept volume), `view`. Registered as the family `moves`.

## How a task is made (backwards)

1. Fixtures A and B: level 1's catalogue mounts, without centre spigots. B faces down above A.
2. A hidden four-bar in a vertical plane between them (its direction across A drawn in 5 degree steps): two
   ground pivots low over A, two coupler pivots high under B, B riding on the coupler. Each pivot's place is
   drawn; the link lengths follow from them.
3. The crank is driven 60-130 degrees, either way, in closed form (circle meets circle, on the branch it is
   assembled in). B's placements at 25-40 % and 55-75 % of the range and at the end are poses 1, 2 and 3.
   With the start, that is four placements of B: classic four-position synthesis. Three placements leave
   infinitely many four-bars to choose from (any two points on B can be the moving pivots); four leave only
   a handful (Burmester's points), so the pivots have to be solved for, not picked.
4. A draw is rejected when:
   - the linkage nears a dead point anywhere in the range (transmission angle outside 35-145 degrees) or cannot close;
   - a link dips to the frame's bolt heads, or a coupler knuckle comes within reach of its ground knuckle;
   - **one hinge would nearly do**: the best single fixed hinge still misses some pose by less than 10 mm.
     A hinge turns B by each pose's own turn, so each pose alone fixes the only hinge centre that hits it
     (its pole); the best compromise among the poles is found exactly for two poles (tested against a
     brute-force search) and by a shrinking grid for three;
   - consecutive poses lie less than 15 mm or 8 degrees apart;
   - the fixtures come within 3 mm of each other during the motion;
   - a bolt head sits where a reference clevis meets a plate, or a screw's line would cross the other link;
   - a reference part does not fit the Prusa MINI with 20 mm to spare (`fits_build_volume`);
   - the reference fails the real grader, printing rules included.

The model gets fixture A, fixture B at its start, B's two target placements (numbers in `task.json` and the
brief, and `fixed_B_pose1.step` to `fixed_B_pose3.step`), the tolerances and the rules. Nothing of the hidden
linkage is in the folder outside `_solution/` (a test checks the words).

## What the model hands in

Up to 8 STEP parts and `assembly.json`, at the start pose:

```json
{"parts": [{"name": "p1", "file": "p1.step", "bolts_to": "A", "insert_direction": [dx, dy, dz]},
           {"name": "p2", "file": "p2.step", "bolts_to": "B", "insert_direction": [dx, dy, dz]},
           {"name": "p3", "file": "p3.step", "bolts_to": null, "insert_direction": [dx, dy, dz]}],
 "pivots": [{"name": "J1", "joins": ["p1", "p3"], "point": [x, y, z], "axis": [ax, ay, az]}],
 "drive": {"pivot": "J1", "range_deg": 90},
 "fasteners": [{"kind": "bolt", "size": "M5", "length": 30, "head_at": [x, y, z],
                "axis": [ax, ay, az], "grip": 20.0, "held_by": "p1"}]}
```

A part bolts to A, to B, or to neither (a link). Parts bolted to the same fixture move with it as one body.
Pivots join bodies; one pivot is the drive. Pins are printed cylinders or catalogue screws (ISO 4762 with an
ISO 4032 nut).

## How it is graded

Each part on its own: a part bolted to a fixture by the existing grader against that fixture (holes,
seating, seat inside, no interference, clean modelling, filament printing), and `TOOL_ACCESS`: a straight
tool (head diameter, 30 mm long) reaches every bolt head, clear of the other fixture and of the parts put in
before it (each part is bolted as it arrives); a link for a valid file, clean
modelling and printing. Then:

| check | passes when |
|---|---|
| `ASSEMBLY_FILE` | the file is complete and consistent |
| `FASTENERS` | each screw is long enough for grip + nut; head and nut bear on 75% of their ring on its own body |
| `ASSEMBLED` | at the start no two bodies overlap |
| `INSERTION` | each part, then each screw and nut, goes in along its line without sweeping through anything (exact swept volume) |
| `PIVOTS` | each declared pivot has a round pin on one body and a round bore on the other, both on the declared axis (0.05 mm, 0.1 deg), overlapping 1 mm along it, the bore at least 0.3 mm larger in radius |
| `MOBILITY` | the bodies and pivots leave exactly one free motion (3 x (bodies - 1) minus the rank of the pivot equations: Gruebler's count, corrected for redundant pivots), and the drive controls it |
| `MOTION` | driven through the declared range, the mechanism closes at every step (no dead point, no jump to the other branch), steps at most 2 deg of drive and 1.5 mm of any body's movement; no two bodies overlap |
| `GAP` | at every step any two bodies stay 0.3 mm apart |
| `POSES` | B passes poses 1, 2 and 3, in order, each within 0.5 mm (face centre) and 0.5 deg |
| `INTERLOCK` | at the start, no pivot comes apart: its two bodies, shifted 5 mm in 26 directions or tilted 15 deg, meet |
| `PLAY` | at each pivot, free shift at most 0.6 mm and free tilt at most 5 deg |

**The mechanism is built from the declared pivots only** (all axes parallel: planar). Each body's place is
three numbers in the plane; each pivot gives two equations, the drive one. The grader follows the drive in
0.25 degree steps and solves the equations from the previous step (Gauss-Newton), so it stays on the branch
it was assembled in. For a four-bar this is the circle-meets-circle answer to 1e-7 mm (tested); it also
handles six-bars and any other planar linkage with one free motion. `PIVOTS` ties the declared pivots to real
geometry: a declared pivot without a real pin and bore fails there, so a model cannot declare a mechanism its
parts do not have.

## The hidden reference

A frame on A with a clevis (two ears) at each ground pivot; a carrier on B with a clevis at each coupler
pivot; a crank and a rocker, flat links each in its own layer so they can cross; four catalogue M4 or M5 axle
screws with nuts, clamped in the clevises. Running gaps 0.4 mm, so the reference sits inside the 0.3 mm gap
and the 0.6 mm play with room both ways. On seed 0 it passes every check; B reaches all three poses to 0.000 mm.

## Broken copies (in `_solution/broken/`, each must fail its named check)

| copy | change | must fail |
|---|---|---|
| misses-pose | the rocker's ground pivot 3 mm off: still one motion, another path | `POSES` |
| clash-mid-motion | the crank has a second arm that swings down into the frame during the drive (first contact at 27.8 of 118 deg on seed 0) | `MOTION` |
| two-free-motions | no rocker: B hangs on the crank alone | `MOBILITY` |
| pivot-without-pin | the rocker-to-carrier pivot declared, its screw left out | `PIVOTS` |

## Viewer

`python -m jointbench.moves view DIR OUT` writes STL files per body, B at each target as a ghost,
`motion.json` (each body's placement per degree of drive, from the grader's own solver) and `index.html`
(three.js, a slider and a play button). Serve the folder (`python -m http.server`) and open it.

## Limits and open choices

- **Tolerances are a choice to set**: poses 0.5 mm / 0.5 deg; play 0.6 mm / 5 deg against the 0.3 mm gap
  (a play limit equal to the gap would leave one legal size, a knife edge); the hinge-miss margin 10 mm.
- Motion generation only (B's full placement at two poses). Path generation (a point on B through given
  points) would be a small addition to `POSES` and the generator.
- Pivots only (revolute joints): no sliders, cams or gears; all axes parallel (planar).
- The motion is checked in steps; a feature thinner than about 1.5 mm could slip through between steps.
- `INTERLOCK` and `PLAY` are judged at the start pose, pivot by pivot (the two bodies a pivot joins).
- A model may choose a different drive and range from the reference's; the drive is assumed to be turned by
  hand or a motor from outside, and only its geometry is judged.
