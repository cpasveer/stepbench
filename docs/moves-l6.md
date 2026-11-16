# L6 "Moves" -- a linkage carries B home inside a closed box and latches it

Level 6 of the version 1 ladder, on the three setups (pipe first; facing and flange from the same shared
code). Everything of level 3 holds: the parts fit, print in PLA without support, and are strong enough. On
top of that:

- **Motion.** Fixture A stays where it is. Fixture B starts outside its home and must be carried, by one
  continuous motion of a linkage the model designs, through two stated placements into its home (its pose at
  levels 1-3). No axis is stated and no single hinge can pass the poses: the model has to synthesise a
  linkage (typically a four-bar) from the placements alone.
- **The box.** Everything happens in a closed box with one opening in its top wall. Every part goes in
  through it, from outside the box, along a straight line or a short path of moves and turns. Screws and
  nuts are brought to their line inside the box: each needs room on its line, clear of everything, to be
  held there and go in. Every bolt into a fixture must be reachable and turnable with its key inside the box.
- **The latch and the load.** At home, B is latched: pulled straight down on its own (with the parts bolted
  to it, without the links), it meets the fixed side within 0.6 mm. The level 3 load pulls it down there and
  is judged in the plastic: the latch's contact patches (bearing, and a plug punched out behind each) and
  every pivot's seat (pin on bore), against level 4a's allowables.
- **Pivots.** A pin or catalogue screw in a bore, or a catalogue ball bearing (608 and friends) pressed into
  one of the two parts it joins, judged as level 4's bearing seat.

```
python -m jointbench.moves.l6 gen --setup pipe --seed 0 --out DIR   # the task folder, its reference graded first
python -m jointbench.moves.l6 verify DIR                             # grades DIR/assembly.json
python -m jointbench.moves.l6 check DIR                              # the reference and every broken copy in DIR/_solution
python -m jointbench.moves.l6 view DIR OUT                           # STL + motion.json + index.html
python -m pytest tests/test_moves_l6.py
```

Code: `src/jointbench/moves/l6/` -- `generate` (shared: the hidden four-bar, the 2D screens, the reference,
the box, the task), `pipe` (what the pipe setup supplies: its fixtures, the frame's base on the pipe, the
mechanism's plane), `grade` (level 4c's grader plus the home checks), `brief`, `view`, `geom2d`. The shared
grader is `jointbench.moves.grade`, extended for the box, bearings and paths; level 4c's tests still apply.

## How a task is made (backwards)

1. The setup's fixtures, as at levels 1-3 (same draw). The mechanism's plane: through B's home centre, s =
   B's face normal made level, t = world up, every pivot along a = s x t.
2. A hidden four-bar in that plane: two coupler pivots on a carrier bolted to B, two ground pivots on posts
   rising from a frame bolted to A. The drive runs from home out to the start (35-120 degrees); B's
   placements along it are pose 2, pose 1 and the start. A draw is dropped if it nears a dead point, if a
   link or the carrier meets the fixtures or the frame anywhere on the way, if a screw's or nut's line runs
   into anything, if the poses lie too close, or if a single hinge could come within 10 mm of all of them.
3. The latch: a lug on the carrier comes over a ledge on a post of the frame on the drive's last stretch.
4. The reference is built at home: frame (base on A, flared posts cut to A's faces so every crevice
   prints), carrier, crank and rocker in their own layers, four M4 axle screws with nuts. The two link stacks
   stand far enough apart that each nut's room misses the other stack. Each part's straight way in is
   chosen (down, or down and sideways), then the box is drawn round everything with room for every key's
   swing, and the opening sized to let every part through.
5. The reference must pass the whole grader. Broken copies (a ground pivot moved, a tail that clashes
   mid-motion, the rocker left out, a pivot without its pin, no ledge) must fail their named checks.

## What is checked (beyond level 4c and level 3)

| check | what |
|---|---|
| INSERTION | every part in through the opening along its line or path; screws and nuts from their stated room |
| POSES | B meets pose 2, pose 1 and the start in order, within 0.5 mm and 0.5 degrees |
| LATCH | at home, B's side pulled down meets the fixed side within 0.6 mm |
| LATCH_STRENGTH | the load on the latch's contact patches: bearing 20 MPa, plug shear 11.5 MPa |
| PIVOT_SEATS | the load through every pivot's seat: bearing 20 MPa |
| TOOL_ACCESS, KEY_TURN | every bolt into a fixture reached and turned inside the box |

The score is the printed material, as at level 3.
