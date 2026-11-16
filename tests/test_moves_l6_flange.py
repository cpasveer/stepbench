"""Level 6 "moves" on the flange setup: the same fixed parts as levels 1-3, the frame's base on the flange, the
setup's own draw, and a reference that the level 6 grader passes (printing rules left out here for time: the
generator itself grades with them)."""
import os
import tempfile

import numpy as np
import pytest

from jointbench.families.flange import shell as SH
from jointbench.moves.l6 import flange as F
from jointbench.moves.l6 import generate as G
from jointbench.moves.l6.grade import grade

SEED = 0


@pytest.fixture(scope="module")
def setup():
    return G.setup_for("flange", SEED)


@pytest.fixture(scope="module")
def made():
    """(task, draw, reference, assembly): seed 0's reference rebuilt from its stored draw (moves/l6/draws/: the one
    the generator kept after grading it with the printing rules), not graded yet (the test grades it)."""
    return G.generate("flange", SEED, check=False)


def test_same_fixed_parts_as_levels_1_to_3(setup):
    task, _ = F._levels(SEED)
    for n in "AB":
        assert setup.home[n].origin == task.poses[0][n].origin and setup.home[n].rot == task.poses[0][n].rot
        assert setup.parts()[n].description == task.parts[n].description
    assert setup.fixed == "A" and setup.carried == "B"  # the flange stays, the block is carried
    assert setup.load_kg == SH._draw(__import__("random").Random(f"flange-{SEED}"), SEED)["load_kg"]


def test_frame_plane_is_level_along_the_blocks_face(setup):
    fr = setup.frame()
    assert abs(fr["s"][2]) < 1e-9 and abs(fr["axis"][2]) < 1e-9 and fr["t"][2] == 1.0
    n_b = np.asarray(setup.home["B"].vec((0, 0, 1)))
    assert float(np.dot(fr["s"], n_b)) > 0.999  # B's face is upright: s is its normal


def test_base_is_one_solid_on_a_flat_bed_clear_of_the_flange(setup):
    P = G.Plane(setup.frame())
    z_bed, z_top = G.heights(setup, P)
    body = setup.finish(setup.base(z_bed, z_top), z_bed)
    assert len(body.solids()) == 1
    bb = body.bounding_box()
    assert abs(bb.min.Z - z_bed) < 1e-6  # it stands on its bed
    fx = setup.fixed_solids()
    assert (body & fx["A"]).volume < 1e-3 and (body & fx["B"]).volume < 1e-3
    for k in setup.keep_out():  # every flange bolt's key way is free
        assert (body & k).volume < 1e-3


def test_generated_task(made):
    task, d, start, asm = made
    mo = task.meta["motion"]
    levels = F._levels(SEED)[0]
    assert task.id == "moves-L6-flange-s0" and mo["fixed"] == "A" and mo["moving"] == "B"
    for n in "AB":  # assembled at home: both fixtures where levels 1-3 have them
        assert np.allclose(task.poses[0][n].origin, levels.poses[0][n].origin, atol=1e-9)
    assert [t["name"] for t in mo["targets"]] == ["pose2", "pose1", "start"]
    assert d["hinge_miss_mm"] >= G.HINGE_MISS_MIN
    assert task.obstacles_at(0)  # the closed box
    assert set(start) == {"frame", "carrier", "crank", "rocker"}


def test_seed_0_frame_swings_onto_the_boss(setup, made):
    """Seed 0 keeps its boss: the frame comes down held open, then swings shut about a vertical line."""
    assert setup.itf_a.protrusions
    _, _, _, asm = made
    frame = next(p for p in asm["parts"] if p["name"] == "frame")
    assert [list(st)[0] for st in frame["path"]] == ["move", "turn"]
    assert frame["path"][1]["turn"]["axis"] == [0.0, 0.0, 1.0]
    assert all("insert_direction" in p for p in asm["parts"] if p["name"] != "frame")


def test_reference_passes_the_level_6_grader(made):
    task, d, start, asm = made
    with tempfile.TemporaryDirectory() as tmp:
        rep = grade(task, G.write_answer(tmp, start, asm), manufacturing=False)
        assert rep.passed, rep.text()


@pytest.mark.parametrize("seed", [2])
def test_other_seeds_draw_and_screen(seed):
    """Seed 2 has its own fixed parts (levels 1-3's); the setup's draw finds a linkage that passes
    the 2D screens and builds (the full generator then grades it)."""
    import random

    setup = G.setup_for("flange", seed)
    P = G.Plane(setup.frame())
    z_bed, z_top = G.heights(setup, P)
    scene = G.Scene2D(setup, P, z_bed, z_top)
    rng = random.Random(f"{G.NAME}-L6-flange-{seed}")
    for _ in range(int(os.environ.get("JB_L6_DRAWS", "60000"))):
        d = setup.draw(rng, P, scene)
        if d is not None and G.screen(d, setup, P, scene) is None:
            break
    else:
        pytest.fail("no draw passed the screens")
    parts, _ = G.build(setup, P, d, z_bed, z_top)
    assert all(len(s.solids()) == 1 for s in parts.values())
