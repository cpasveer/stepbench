"""COMPLIANCE: the travel as a vector, the band verdict, and that the check is
judged only where a task lists it. The finite-element self-test (block, rod,
parallelogram, short blades) is slow and is not part of this test suite."""
import math

import numpy as np

from jointbench.checks import OPTIONAL
from jointbench.stiffness.compliance import Travel, _band


def test_slide_is_a_pure_translation():
    t = Travel("slide", 1.0, (2, 10), 50, direction=[0, 3, 4])
    e, n = t.vector(np.zeros(3), 100.0)
    assert np.allclose(e, [0, 0.6, 0.8, 0, 0, 0]) and n == 1.0


def test_turn_moves_the_mount_round_the_axis_point():
    # axis z through (10, 0, 0); the mount at the origin moves along -y per radian, by 10 mm
    t = Travel("turn", 1.0, (1, 5), 50, axis=[0, 0, 1], point=[10, 0, 0])
    e, n = t.vector(np.zeros(3), 100.0)
    m = e * n
    assert np.allclose(m[:3], [0, -10, 0]) and np.allclose(m[3:], [0, 0, 100])
    assert math.isclose(n, math.hypot(10, 100))


def test_band_verdicts():
    assert _band(5, 2, 10, 0.02) == "pass"
    assert _band(1, 2, 10, 0.02) == "fail"
    assert _band(11, 2, 10, 0.02) == "fail"
    assert _band(2.01, 2, 10, 0.02) == "marginal"


def test_judged_only_where_required():
    assert "COMPLIANCE" in OPTIONAL
