"""The quick test on an arrangement: can a part for it be printed without support at all?"""
import math

from jointbench.dfm.printable import print_directions


def face(n, at=10.0):
    """A mounting face at ``at`` along its own normal (the part on the outside of a block)."""
    o = tuple(at * v for v in n)
    return (o, n, [o])


def test_two_faces_looking_at_each_other_print_standing_upright():
    out = print_directions([face((1, 0, 0)), face((-1, 0, 0))])
    assert out and out[0]["bed"] is None
    assert abs(out[0]["direction"][0]) < 1e-3  # both faces upright: the bed is across them
    assert out[0]["margin_deg"] > 44


def test_three_square_faces_print_on_the_diagonal():
    out = print_directions([face((1, 0, 0)), face((0, 1, 0)), face((0, 0, 1))])
    assert out and out[0]["bed"] is None
    d = out[0]["direction"]
    assert all(abs(abs(v) - 1 / math.sqrt(3)) < 0.05 for v in d)


def test_no_direction_when_every_face_is_too_flat_and_the_bed_is_blocked():
    # within 30 deg of vertical, three square faces can never all stand; lying one on the bed
    # puts the other two below it
    assert print_directions([face((1, 0, 0)), face((0, 1, 0)), face((0, 0, 1))], overhang_deg=30) == []


def test_a_seat_can_lie_on_the_bed_when_the_rest_stands_above_it():
    # the part inside a block: faces look inward, the bed face is the lowest
    out = print_directions([face((1, 0, 0), -10), face((0, 1, 0), -10), face((0, 0, 1), -10)],
                           overhang_deg=30)
    assert out and all(x["bed"] is not None for x in out)
