"""The standard tables the joints use, each with its source.

* ISO 286-1 / -2 (limits and fits): standard tolerance grades IT5-IT11 and
  the fundamental deviations of the shaft letters a, d, e, f, g, h, k, n, p,
  r, s and the hole letters H, JS, N, P, for sizes up to 120 mm.
* DIN 6885-1 (form A parallel keys; the key sections b x h are the ones of
  ISO/R 773): key section and keyway depths per shaft diameter.
* ISO 14 (straight-sided splines with cylindrical centring on the minor
  diameter): light and medium series, and its tolerance classes for a
  sliding hub.

Every number is stored in micrometres (deviations) or millimetres (sizes),
exactly as printed in the tables, so a zone is computed, never interpolated.
"""
from __future__ import annotations

from dataclasses import dataclass

# ISO 286 size steps, "over a up to and including b". The fine steps are
# needed for the letters a, d, e, f, g, r, s whose deviations change inside
# a main step.
STEPS = [3, 6, 10, 14, 18, 24, 30, 40, 50, 65, 80, 100, 120]
# main step of each fine step (IT values change only per main step)
_MAIN = [0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7]

# IT grades (um) per main step: 0-3, 3-6, 6-10, 10-18, 18-30, 30-50, 50-80, 80-120
IT = {
    5: [4, 5, 6, 8, 9, 11, 13, 15],
    6: [6, 8, 9, 11, 13, 16, 19, 22],
    7: [10, 12, 15, 18, 21, 25, 30, 35],
    8: [14, 18, 22, 27, 33, 39, 46, 54],
    9: [25, 30, 36, 43, 52, 62, 74, 87],
    10: [40, 48, 58, 70, 84, 100, 120, 140],
    11: [60, 75, 90, 110, 130, 160, 190, 220],
}

# shaft fundamental deviations (um) per fine step. For a..h it is the upper
# deviation es; for k..s the lower deviation ei.
SHAFT_ES = {
    "a": [-270, -270, -280, -290, -290, -300, -300, -310, -320, -340, -360, -380, -410],
    "d": [-20, -30, -40, -50, -50, -65, -65, -80, -80, -100, -100, -120, -120],
    "e": [-14, -20, -25, -32, -32, -40, -40, -50, -50, -60, -60, -72, -72],
    "f": [-6, -10, -13, -16, -16, -20, -20, -25, -25, -30, -30, -36, -36],
    "g": [-2, -4, -5, -6, -6, -7, -7, -9, -9, -10, -10, -12, -12],
    "h": [0] * 13,
}
SHAFT_EI = {
    "k": [0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3],  # valid for grades 4..7
    "n": [4, 8, 10, 12, 12, 15, 15, 17, 17, 20, 20, 23, 23],
    "p": [6, 12, 15, 18, 18, 22, 22, 26, 26, 32, 32, 37, 37],
    "r": [10, 15, 19, 23, 23, 28, 28, 34, 34, 41, 43, 51, 54],
    "s": [14, 19, 23, 28, 28, 35, 35, 43, 43, 53, 59, 71, 79],
}


def step(size: float) -> int:
    for i, b in enumerate(STEPS):
        if size <= b + 1e-9:
            return i
    raise ValueError(f"size {size} mm is outside the tables (up to 120 mm)")


def it(size: float, grade: int) -> float:
    """Standard tolerance in mm."""
    return IT[grade][_MAIN[step(size)]] / 1000.0


def shaft_zone(size: float, letter: str, grade: int) -> tuple[float, float]:
    """(smallest, largest) allowed shaft diameter in mm, e.g. 25 p6."""
    i, t = step(size), it(size, grade)
    if letter == "js":
        return size - t / 2, size + t / 2
    if letter in SHAFT_ES:
        es = SHAFT_ES[letter][i] / 1000.0
        return size + es - t, size + es
    if letter in SHAFT_EI:
        if letter == "k" and grade > 7:
            ei = 0.0
        else:
            ei = SHAFT_EI[letter][i] / 1000.0
        return size + ei, size + ei + t
    raise KeyError(f"shaft letter {letter!r} not in the table")


def hole_zone(size: float, letter: str, grade: int) -> tuple[float, float]:
    """(smallest, largest) allowed hole diameter or slot width in mm, e.g. 25 H7."""
    i, t = step(size), it(size, grade)
    if letter == "H":
        return size, size + t
    if letter == "JS":
        return size - t / 2, size + t / 2
    if letter == "N" and grade >= 9:
        es = -0.004 if i == 0 else 0.0
        return size + es - t, size + es
    if letter == "P" and grade >= 8:
        es = -SHAFT_EI["p"][i] / 1000.0
        return size + es - t, size + es
    raise KeyError(f"hole zone {letter}{grade} not in the table")


def zone(size: float, code: str) -> tuple[float, float]:
    """A zone from its ISO code: 'H7', 'p6', 'JS9', 'a11' ..."""
    letters = code.rstrip("0123456789")
    grade = int(code[len(letters):])
    if letters[0].isupper():
        return hole_zone(size, letters, grade)
    return shaft_zone(size, letters, grade)


def mid(z: tuple[float, float]) -> float:
    return 0.5 * (z[0] + z[1])


# ------------------------------------------------------------- DIN 6885-1


@dataclass(frozen=True)
class Key:
    d_over: float  # shaft diameter over ...
    d_upto: float  # ... up to and including
    b: float  # key width
    h: float  # key height
    t1: float  # keyway depth in the shaft
    t2: float  # keyway depth in the hub

    @property
    def depth_tol(self) -> float:
        """Upper tolerance of t1 and t2 (lower is 0): +0.1 up to h = 6, +0.2 above."""
        return 0.1 if self.h <= 6 else 0.2


KEYS = [
    Key(6, 8, 2, 2, 1.2, 1.0),
    Key(8, 10, 3, 3, 1.8, 1.4),
    Key(10, 12, 4, 4, 2.5, 1.8),
    Key(12, 17, 5, 5, 3.0, 2.3),
    Key(17, 22, 6, 6, 3.5, 2.8),
    Key(22, 30, 8, 7, 4.0, 3.3),
    Key(30, 38, 10, 8, 5.0, 3.3),
    Key(38, 44, 12, 8, 5.0, 3.3),
    Key(44, 50, 14, 9, 5.5, 3.8),
    Key(50, 58, 16, 10, 6.0, 4.3),
    Key(58, 65, 18, 11, 7.0, 4.4),
    Key(65, 75, 20, 12, 7.5, 4.9),
    Key(75, 85, 22, 14, 9.0, 5.4),
    Key(85, 95, 25, 14, 9.0, 5.4),
    Key(95, 110, 28, 16, 10.0, 6.4),
]

# DIN 6885 standard key lengths (mm)
KEY_LENGTHS = [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 70, 80,
               90, 100, 110, 125, 140, 160, 180, 200]


def key_for(d: float) -> Key:
    for k in KEYS:
        if k.d_over < d <= k.d_upto + 1e-9:
            return k
    raise ValueError(f"no DIN 6885 key for shaft diameter {d} mm")


# ------------------------------------------------------------- ISO 14


@dataclass(frozen=True)
class Spline:
    series: str
    n: int  # number of splines
    d: float  # minor diameter
    D: float  # major diameter
    B: float  # spline (tooth) width

    @property
    def code(self) -> str:
        return f"{self.n} x {self.d:g} x {self.D:g}"


SPLINES = [Spline("light", *v) for v in [
    (6, 23, 26, 6), (6, 26, 30, 6), (6, 28, 32, 7), (8, 32, 36, 6), (8, 36, 40, 7),
    (8, 42, 46, 8), (8, 46, 50, 9), (8, 52, 58, 10), (8, 56, 62, 10), (8, 62, 68, 12),
]] + [Spline("medium", *v) for v in [
    (6, 11, 14, 3), (6, 13, 16, 3.5), (6, 16, 20, 4), (6, 18, 22, 5), (6, 21, 25, 5),
    (6, 23, 28, 6), (6, 26, 32, 6), (6, 28, 34, 7), (8, 32, 38, 6), (8, 36, 42, 7),
    (8, 42, 48, 8), (8, 46, 54, 9), (8, 52, 60, 10), (8, 56, 65, 10), (8, 62, 72, 12),
]]

# ISO 14 tolerance classes, sliding fit, hub not treated after broaching:
# hub d H7, D H10, B H9; shaft d f7, D a11, B d10
SPLINE_SLIDING = {"hub": {"d": "H7", "D": "H10", "B": "H9"},
                  "shaft": {"d": "f7", "D": "a11", "B": "d10"}}
