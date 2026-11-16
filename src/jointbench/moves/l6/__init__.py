"""Level 6, "moves", on the version 1 setups: a linkage carries B home inside a closed box and latches it.

Modules: ``generate`` (the shared generator), ``pipe`` (what the pipe setup supplies), ``grade`` (the level 6
grader), ``brief``, ``view``, ``geom2d`` (small 2D helpers for the generator's screens); run with
``python -m jointbench.moves.l6 gen|verify|check|view``.

WORK IN PROGRESS (15 November 2026). State and hand-over notes:

Setup-independent (shared, meant to be reused as is):
* the grader: ``moves.grade`` (parts, assembly file, pivots, mobility, motion, gaps, poses, pivots pulled and
  tilted), extended for level 6 with the closed box (``task.obstacles``: walls stand with the fixed fixture in
  every collision, gap, insertion and key-access check; screws and nuts need room inside the box), catalogue
  ball-bearing pivots (``"bearing": {"size", "seat_on"}`` on a pivot; seat judged by ``checks.bearing_seat``),
  and ``l6.grade``'s home checks: LATCH (B's side pulled straight down meets the fixed side within the play),
  LATCH_STRENGTH (4d's rule, 4a's allowables), PIVOT_SEATS (the load through every pivot's seat);
* the box (``generate.box``/``box_walls``: walls in the mechanism's axes, windows where the fixed fixture runs
  through, one opening in the top wall sized to let every part come straight down);
* the hidden four-bar machinery (``generate.draw``: two-position construction -- B's start drawn above and
  behind home, each ground pivot on its coupler pivot's bisector, drive range from the linkage; ``path``,
  ``coupler``, ``b_placement``, ``hinge_miss``), the 2D screens (``screen``), the reference builder (``build``,
  ``at_start``), broken copies (``BROKEN``), the folder writer, the brief and the viewer.

What a setup must supply (see ``pipe.Setup``):
* ``name``, ``fixed`` / ``carried`` (fixture names), ``parts()`` (interfaces), ``home`` ({name: Placement}:
  levels 1-3's poses), ``load_kg`` (level 3's);
* ``frame()``: the mechanism plane -- origin (B's home centre), s (B's face normal, level: towards the fixed
  fixture), t (world up), axis (s x t, every pivot's direction); the generator assumes world up is the print
  direction of the frame and the latch's pull;
* ``base(z_bed, z_top)``: the frame's base on the fixed fixture, in the world (posts rise straight up from it),
  ``finish(body, z_bed)``: carve the fixed fixture out and cut to the print bed, ``keep_out()``, and
  ``fixed_solids()``. ``generate.heights`` reads the fixed fixture's ``hole_frames`` (a flat mount needs
  another rule there).

Optional, for a setup the shared draw does not suit (``flange.py`` uses all five; the pipe none):
* ``heights(P)`` -> (bed z, top z): the frame's base heights, instead of ``generate.heights``'s rule;
* ``draw(rng, P, scene)`` -> the hidden four-bar in the shared draw's format (or None), instead of
  ``generate.draw``; the screens, reference, box and grading after it are the shared ones;
* ``layers_half``: how far the layers may reach along the axis from B's centre (default: B's face, less 1.5);
* ``posts_clear_keep_out = True``: the shared draw then also keeps its pivot and latch posts clear of
  ``keep_out()`` (``generate._hits_keep``, which a setup's own draw can call too);
* ``insert_path(name, solid)`` -> a part's way in for the reference's assembly file (moves and turns), or None for
  straight down (the flange's frame swings onto its boss like a door).

Still open:
* the pipe reference has not yet passed the full grader: the draw search's 2D screens are tuned to the pipe and
  the last full grades failed on B's bolt heads (fixed in the screens since), on the way in past B at the start
  (screened since) and on one pipe bolt's head room (35 mm3, frame post near a key channel: not yet looked at);
* the draw search is slow (minutes per accepted draw); generation is once per setup;
* facing and flange: setup modules to write (``facing.py``, ``flange.py``) with their own base; a flat fixed
  fixture needs ``heights`` to use its face instead of bolt frames;
* tests (tests/test_moves_l6.py) and docs (docs/moves-l6.md) not written yet; the 4c tests still apply to the
  shared grader and should be run after any change to ``moves.grade``.
"""
