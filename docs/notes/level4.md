> Working notes for levels 4–6 (planned for v2). They may be out of date and are not part of v1's claims.

# Level 4a: assembles (prototype)

> **Ladder of 15 November 2026:** this prototype is now **L4 Assembles**, on the pipe setup in version 1, built on L3. The text below describes the prototype as it was built (facing situation, "level 4a").

The structure: 1 fits, 2 printable, 3 strong, then three level 4s:
4a assembles (this one), 4b adapts, 4c moves. Assembles builds on level 3.

The answer is **two parts** the model designs, bolted to each other by a
joint of its own design, each put in place along a stated path, holding
together under the stated load. Prototype: the facing situation only.

## The situation: a ship in a bottle (14 November)

Fixed part A faces up from the floor at one end of a closed box; fixed
part B faces down from the ceiling at the other end. The box -- fixed walls,
floor and ceiling round the space between the mounts -- has one small
opening, in the end wall on B's side. Every part goes in through it, one
after the other (each part comes straight in from outside, then may move
and turn along a stated path), and they are joined inside.

What makes it take thought rather than a cut:

- **The opening.** It is barely larger than each half of the connection
  (2-3 mm all round) and much lower than the box, so nothing reaching from
  A up to B fits through it. Tested on every reference: the two reference
  parts fused into one hit the box through the opening at any height and
  from every other side.
- **The order.** The part for A must go in first: it travels the length of
  the box to A, and once the part for B is in, that way is blocked.
- **The joint's place.** Every joint bolt must be reachable by a key from
  outside: along each head's axis, from the head out past everything, a
  cylinder of the head's diameter must be free (`JOINT_REACH`). Inside a
  closed box that means the heads face the opening, in its line of sight.
- **The joint's strength.** The plastic round each bolt is judged
  (pull-through, bearing, tear-out to an edge; below), so a thin flange or
  a hole near an edge fails.

The generator builds the hidden reference first: part 1 is a plate on A
with an upright wall at its end nearest the opening; part 2 is a plate
under B reaching back to a flange that hangs against that upright. Part 1
goes in at the opening's height, along the box and down onto A; part 2 goes
in, meets the upright face to face and slides up onto B. The two bolt
through upright and flange, heads on the flange facing the opening. The box
is placed round that, and a draw is kept only if the reference passes the
full grader (load, plastic, key reach, printing); a reference whose
stiffness is only marginal is made thicker, not accepted.

## The joint is judged by the plastic (14 November)

In a printed part the bolt never fails; the PLA round it does. For each
joint bolt, on each part, from that part's own measured shape: pull-through
(the pull over the ring under the head or nut, and punching a plug through
the thickness under it), bearing (the slide over shank diameter x
thickness), and tear-out (the slide over the plastic between the hole and
the part's free edge, in the direction the bolt pushes it; the edge
distance is measured on the part in 72 directions). Allowable stresses from
the one cited strength of printed PLA, 40 MPa across layers, over a safety
factor of 2: bearing 20 MPa (no credit for compression), shear 11.5 MPa
(40 / sqrt 3 / 2, the von Mises relation; no citable published shear
strength of printed PLA across layers was found). Shear keys are allowed and
take the slide off the bolts. The task allows bolt sizes only up to the
smallest one the reference's joint holds with, and no more bolts.

## What the model hands in

`part1.step`, `part2.step` (one solid each, in place) and `assembly.json`:
which fixed part each bolts to, the straight direction each comes in along
and in which order, and the joint: one catalogue bolt size, the bolt
positions on the face where the parts meet, its axis, and which part the
heads bear on (the other gets ISO 4032 hex nuts).

## How it is graded

All exact measurement, deterministic.

- **Each part, by the existing checks.** To each part, the other part is
  presented as one more fixed part: a mounting face on the joint plane with
  the joint bolts in it (a nut in place of the head on the nut side). So the
  joint is judged by exactly the logic used at a fixed part: holes on the
  stated bolt axes and of clearance size, each part lying on the other
  round every bolt, grip in range, head and nut room, a hex key's reach to
  every head. The fixed part a part does not bolt to counts as an obstacle
  to it. Clean modelling and filament printing as at level 2/3.
- **INSERTION.** Each part, in order, is swept from outside the scene to
  its place along its direction. The swept space is exact: the prisms
  traced by the part's leading faces; turns along a path are checked in
  steps of at most 0.25 mm of travel. Any overlap with a fixed part, the
  box, the part already in place or its bolt heads fails, naming what was
  hit. Sliding along a face is zero volume and passes.
- **JOINT_REACH.** A key reaches every joint bolt from outside (above).
- **JOINT_ROOM.** Heads and nuts clear of the fixed parts and the box; the
  two parts touch only at the joint face (so the analysis below cannot be
  flattered by unbolted contact).
- **JOINT_BOLTS.** The loaded fixed part hangs on one part, which reaches
  the other only through the joint, so the joint carries the whole load and
  its turning effect -- statics, not stiffness. That is split into bolt
  forces as a bolt group is designed (a pull shared equally; an opening
  moment tipping the joint about the far edge of the contact face, each bolt
  in proportion to its distance from that edge; a slide shared equally; a
  twist in proportion to distance from the bolts' centre; shear keys take
  slide and twist off the bolts), for 4000 load directions. The plastic
  round each bolt is then judged as above; the worst ratio decides.
- **STIFFNESS.** The two parts fused at the joint face and analysed as one
  body with level 3's stated-load check, unchanged (capped run time; a part
  that cannot be measured fails). Treating the bolted face as bonded is the
  usual assumption for a tightened joint below its opening load, which
  JOINT_BOLTS checks.

- **Score.** Among answers that pass, the lighter wins: the plastic the
  printer lays down for both parts (shell and 20% infill, level 3's
  measure), each in the print direction its printing check chose.

Printing and the load are judged in filament (PLA) only.

## Simplifications, to be judged

- Mounts are square or two-column rectangular, square to the world axes, no
  spigots (they leave room for an upright between the bolt columns).
- Joint bolts all one size, on one flat face, through bolt and nut only (no
  threaded inserts). The nut needs room but no tool reach along its axis
  (a spanner holds it from the side).
- Keys for the bolts into A and B are judged as at level 3: a 30 mm hex key
  along each bolt's axis, inside the box. Through the one opening they
  could not be reached at all (A's heads face up, B's face down); the
  fixed parts' bolts are taken as put in from the box's inside. Your call.
- `jointbench verify` grades but does not archive level 4a attempts yet.

## Commands

    python -m jointbench.assemble gen --seed 0 --out DIR   # task folder + hidden reference + broken copies
    python -m jointbench.assemble check DIR/L4a-s0          # reference must pass, each broken copy must fail as labelled
    python -m jointbench.assemble view DIR/L4a-s0 OUT       # STL files and a page: box, parts, each part's way in
    jointbench verify DIR/L4a-s0                            # grade part1/part2/assembly.json in the folder
