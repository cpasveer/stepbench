"""Stiff references: a space truss between two plates.

Why a truss. Powder printing caps a solid section at 15 mm, and a member's
bending stiffness grows with the fourth power of its size, so one rod cannot
be made stiff enough by thickening it. A truss spreads the material: several
parallel rods ("chords") set well apart, joined by diagonal rods, so a load
on one mount is carried as pull and push along the rods instead of bending.
The chords give stiffness in bending, the diagonals in shear and twisting.

The shape, built from ``levels.rods`` pieces only (rods square out of a
plate, spheres at every node), so it stays printable and cleanly modelled:

* each plate carries four chord nodes at the corners of a rectangle, placed by
  the level: around the bolts (level 1) or on a tab beside them (level 2);
* chord ``i`` leaves plate A square to its face (a short stub), runs
  straight across and enters plate B square to its face;
* along the straight run, every chord gets nodes at the same fractions of
  its length, and each side panel between neighbouring chords gets one
  diagonal per bay, all turning the same way round, with a sphere at each
  node;
* every pair of rods meeting at a node is at least ``MIN_ANGLE`` apart (a
  sharper joint leaves a crevice, and a wedge of air too narrow for powder),
  checked by ``angles_ok``; a draw that cannot meet it is rejected.
"""
from __future__ import annotations

import math

import build123d as bd

from . import rods as R

MIN_ANGLE = 70.0   # deg between any two rods at a node: under 45 the quality checker sees a crevice;
#                    nearer 60 the wedge of air beside the joint comes close to powder
#                    printing's 0.5 mm minimum gap (measured on level 2 trusses: 63 deg
#                    0.37 mm, 70 deg 0.56-1.06 mm)
STUB = 4.0         # mm a chord runs square out of its plate before it turns
R_DIAG_MIN = 2.0   # mm: diagonals carry pull and push only; thinner than any chord (R_MIN 3)
BAY = 0.36         # bay length as a share of the chord spacing (diagonal at about 70 deg to the chords)


def corners(frame: bd.Plane, x0: float, y0: float, x1: float, y1: float, z: float) -> list[bd.Vector]:
    """Four chord nodes at the corners of a rectangle in the plate frame, at
    height z (inside the plate), in order round it."""
    return [frame.from_local_coords(bd.Vector(x, y, z)) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def pair(na: list, nb: list, ua: bd.Vector, ub: bd.Vector, stub_a: float, stub_b: float) -> list[int]:
    """Which node on B each node on A goes to: keeps the cyclic order (no
    two chords cross) and makes the chords' straight runs shortest."""
    n = len(na)
    best = None
    for rev in (False, True):
        for s in range(n):
            perm = [((-i if rev else i) + s) % n for i in range(n)]
            L = sum(((nb[j] + ub * stub_b) - (na[i] + ua * stub_a)).length for i, j in enumerate(perm))
            if best is None or L < best[0]:
                best = (L, perm)
    return best[1]


def layout(na: list, nb: list, ua: bd.Vector, ub: bd.Vector, stub_a: float, stub_b: float,
           a: float, bay: float | None = None) -> dict:
    """The truss as points and rods (no solids yet): chords, stations along
    their straight runs, diagonals."""
    bay = BAY if bay is None else bay
    perm = pair(na, nb, ua, ub, stub_a, stub_b)
    n = len(na)
    runs = [(na[i] + ua * stub_a, nb[perm[i]] + ub * stub_b) for i in range(n)]
    length = max((q - p).length for p, q in runs)
    # a run shorter than about the chord spacing is a stocky frame already
    # (the plates brace it); a diagonal there would lie almost flat
    m = 0 if length < 0.6 * a else math.ceil(length / (bay * a))
    st = [[p + (q - p) * (j / max(m, 1)) for j in range(max(m, 1) + 1)] for p, q in runs]
    chords = [[na[i]] + st[i] + [nb[perm[i]]] for i in range(n)]
    diags = [(st[i][j], st[(i + 1) % n][j + 1]) for i in range(n) for j in range(m)]
    return {"chords": chords, "diagonals": diags, "stations": st, "bays": m, "perm": perm}


def angles_ok(lay: dict, min_deg: float = MIN_ANGLE) -> tuple[bool, float]:
    """Every pair of rods meeting at a node at least ``min_deg`` apart."""
    rods_at: dict = {}

    def key(p):
        return (round(p.X, 4), round(p.Y, 4), round(p.Z, 4))

    def add(p, q):
        if (q - p).length > 1e-6:
            rods_at.setdefault(key(p), (p, []))[1].append((q - p).normalized())

    for ch in lay["chords"]:
        for p, q in zip(ch[:-1], ch[1:]):
            add(p, q)
            add(q, p)
    for p, q in lay["diagonals"]:
        add(p, q)
        add(q, p)
    worst = 180.0
    ends = {key(ch[0]) for ch in lay["chords"]} | {key(ch[-1]) for ch in lay["chords"]}
    for k, (p, dirs) in rods_at.items():
        if k in ends:
            continue  # inside the plate
        for i in range(len(dirs)):
            for j in range(i + 1, len(dirs)):
                c = max(-1.0, min(1.0, dirs[i].dot(dirs[j])))
                worst = min(worst, math.degrees(math.acos(c)))
    return worst >= min_deg, worst


def _bends(points: list) -> list:
    """The polyline without its straight-through points: a sphere is needed
    only where the chord turns (on a straight chord a diagonal simply ends
    on its axis, inside it)."""
    out = [points[0]]
    for p, q in zip(points[1:-1], points[2:]):
        u, v = (p - out[-1]), (q - p)
        if u.length < 1e-6 or v.length < 1e-6:
            continue
        if u.normalized().dot(v.normalized()) < 1 - 1e-9:
            out.append(p)
    out.append(points[-1])
    return out


GAP_CLEAR = 1.0  # mm of clear space kept between rods that do not meet (powder: at least 0.5)


def prune(lay: dict, r_chord: float, r_diag: float) -> dict:
    """Drop every diagonal that passes closer than ``GAP_CLEAR`` to a rod it
    does not meet (near a bend, a column's diagonal can graze the run's
    chords); a chord grazing another chord rejects the layout."""
    def key(p):
        return (round(p.X, 4), round(p.Y, 4), round(p.Z, 4))

    chord_pieces = [(a, b, i) for i, ch in enumerate(lay["chords"]) for a, b in zip(ch[:-1], ch[1:])
                    if (b - a).length > 1e-6]
    for x in range(len(chord_pieces)):
        for y in range(x + 1, len(chord_pieces)):
            a, b, i = chord_pieces[x]
            c, e, j = chord_pieces[y]
            if i == j:
                continue
            if R.seg_seg(a, b, c, e) < 2 * r_chord + GAP_CLEAR:
                raise ValueError("two chords pass too close to each other")
    on_chord = [{key(p) for p in ch} for ch in lay["chords"]]
    kept, dropped = [], 0
    for p, q in lay["diagonals"]:
        ends = {key(p), key(q)}
        own = {i for i, pts in enumerate(on_chord) if ends & pts}  # the chords it joins: one rod each
        bad = False
        for a, b, i in chord_pieces:
            if i in own:
                continue
            if R.seg_seg(p, q, a, b) < r_diag + r_chord + GAP_CLEAR:
                bad = True
                break
        if not bad:
            for c, e in kept:
                if {key(c), key(e)} & ends:
                    continue
                if R.seg_seg(p, q, c, e) < 2 * r_diag + GAP_CLEAR:
                    bad = True
                    break
        if bad:
            dropped += 1
        else:
            kept.append((p, q))
    return dict(lay, diagonals=kept, dropped=dropped)


BLEND = (1.0, 0.6)  # mm: blend radii tried at rod-to-rod joints, largest first


def solid(lay: dict, r_chord: float, r_diag: float, blend: bool = False) -> bd.Part:
    """The rods as one solid. Where a diagonal enters a chord the union
    leaves a sharp crease with a wedge of air beside it narrower than
    powder printing allows; a small blend (fillet) along every such joint
    opens it (measured: 0.26 mm gap without, 3.3 mm with a 1.2 mm blend).
    Off by default: on a whole truss the kernel takes many minutes to blend,
    so the references keep their joints at 70 degrees or more instead,
    which leaves the wedge open enough without a blend."""
    body = None
    for ch in lay["chords"]:
        piece = R.path(_bends(ch), r_chord)  # rods, a sphere where the chord turns
        body = piece if body is None else body + piece
    for p, q in lay["diagonals"]:
        body = body + R.rod(p, q, r_diag)
    if blend and lay["diagonals"]:
        joints = [e for e in body.edges() if e.geom_type.name not in ("LINE", "CIRCLE")]
        for rad in BLEND:
            try:
                out = body.fillet(rad, joints)
                if out.is_valid and len(out.solids()) == 1:
                    lay["blend"] = rad
                    return out
            except Exception:  # noqa: BLE001 - the kernel could not blend at this radius
                continue
        lay["blend"] = None
    return body


# --------------------------------------------------------------------------
# a truss along a bent centre line (levels 2 and 3)

CLEAR = 5.0  # mm between the plate face and the first diagonal's node, beyond the chord's radius


def _mitre(points: list, offsets: list) -> list[list]:
    """Each chord's bend points: chord k runs parallel to the centre line at
    offset ``offsets[k]`` (perpendicular to the first segment) and is
    mitred at every bend of the centre line, so the cross-section keeps its
    shape all the way along."""
    chords = []
    for o in offsets:
        pts = [points[0] + o]
        for c, (u1, u2) in zip(points[1:-1], zip(
                [(q - p).normalized() for p, q in zip(points[:-2], points[1:-1])],
                [(q - p).normalized() for p, q in zip(points[1:-1], points[2:])])):
            m = (u1 + u2)
            if m.length < 1e-6:
                raise ValueError("the centre line turns back on itself")
            m = m.normalized()
            prev = pts[-1]
            s = (c - prev).dot(m) / u1.dot(m)
            pts.append(prev + u1 * s)
        last_dir = (points[-1] - points[-2]).normalized()
        prev = pts[-1]
        pts.append(prev + last_dir * (points[-1] - points[-2]).dot(last_dir)
                   - last_dir * (prev - points[-2]).dot(last_dir))
        chords.append(pts)
    return chords


def square_offsets(u: bd.Vector, frame_x: bd.Vector, a: float) -> list[bd.Vector]:
    """The four corners of an a x a square perpendicular to ``u``, sides
    along ``frame_x`` (made perpendicular to u) and u x frame_x."""
    x = (frame_x - u * frame_x.dot(u)).normalized()
    y = u.cross(x)
    h = a / 2
    return [x * sx * h + y * sy * h for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def path_layout(points: list, frame_x: bd.Vector, a: float, r: float, *, t_start: float, t_end: float,
                bay: float | None = None) -> dict:
    """A four-chord truss along the centre line ``points`` (first and last
    point at mid-thickness inside the end plates; the first and last
    segments square to them). The chords sit at the corners of an a x a
    square, its sides along ``frame_x`` and the first segment's normal.
    Each straight segment gets its own bays; near a plate the first node is
    ``CLEAR`` mm beyond the chord's radius off the face, so no diagonal meets
    the plate at a slant."""
    bay = BAY if bay is None else bay
    u0 = (points[1] - points[0]).normalized()
    chords = _mitre(points, square_offsets(u0, frame_x, a))
    n = len(chords)
    segs = len(points) - 1
    diags, stations = [], []
    for k in range(segs):
        ends = [(ch[k], ch[k + 1]) for ch in chords]
        u = (points[k + 1] - points[k]).normalized()
        # diagonals keep clear of the plates and of the bends: a bend node
        # carries only its two chord pieces and its sphere
        s0 = (t_start / 2 if k == 0 else 0.0) + r + CLEAR
        s1 = (t_end / 2 if k == segs - 1 else 0.0) + r + CLEAR
        # nodes on common cross-section planes, inside the stretch every
        # chord of this segment covers, so every bay is the same length
        c0 = points[k]
        lo = max((p - c0).dot(u) for p, _ in ends) + s0
        hi = min((q - c0).dot(u) for _, q in ends) - s1
        L = hi - lo
        m = 0 if L < 0.6 * a else math.ceil(L / (bay * a))
        if m == 0:
            stations.append(None)
            continue
        st = [[p + u * (lo + L * j / m - (p - c0).dot(u)) for j in range(m + 1)] for p, _ in ends]
        stations.append(st)
        diags += [(st[i][j], st[(i + 1) % n][j + 1]) for i in range(n) for j in range(m)]
    # every chord as its full polyline, stations included, for the angle check
    full = []
    for i, ch in enumerate(chords):
        pts = [ch[0]]
        for k in range(segs):
            if stations[k] is not None:
                pts += [p for p in stations[k][i] if (p - pts[-1]).length > 1e-6]
            pts.append(ch[k + 1])
        full.append(pts)
    return {"chords": full, "diagonals": diags, "bays": sum(len(s[0]) - 1 for s in stations if s), "perm": None,
            "bends": chords}


def mitre_room(u1: bd.Vector, u2: bd.Vector, a: float) -> float:
    """How far a chord at the square's corner reaches back along u1 at a
    bend from u1 to u2: the column before the bend must be longer."""
    c = max(-1.0, min(1.0, u1.dot(u2)))
    phi = math.acos(c)
    return (a / math.sqrt(2)) * math.tan(phi / 2)


PLATE_CLEAR = 1.5  # mm between any rod and a plate's surface, beyond the rod's radius


def plate_clearance(lay: dict, plate: dict, r_chord: float, r_diag: float) -> float:
    """The smallest gap between a rod and ``plate`` (a plate dict
    from ``construct.plate``: frame, z0, t, x0..x1+tab, y0..y1), over the
    plate's footprint. Rods that start inside the plate (a chord leaving it
    square) are left out: they are meant to be joined to it."""
    fr = plate["frame"]
    top = plate["z0"] + plate["t"]
    x0, x1 = plate["x0"], plate["x1"] + plate.get("tab", 0.0)
    y0, y1 = plate["y0"], plate["y1"]
    rods = [(a, b, r_chord) for ch in lay["chords"] for a, b in zip(ch[:-1], ch[1:])]
    rods += [(a, b, r_diag) for a, b in lay["diagonals"]]
    worst = math.inf
    for a, b, r in rods:
        la, lb = fr.to_local_coords(a), fr.to_local_coords(b)

        def inside(q):
            return x0 <= q.X <= x1 and y0 <= q.Y <= y1 and plate["z0"] - 1e-6 <= q.Z <= top + 1e-6

        if inside(la) or inside(lb):  # starts inside this plate: joined to it
            continue
        n = max(2, int(2 * (b - a).length) + 1)
        for k in range(n + 1):
            q = la + (lb - la) * (k / n)
            dx = max(x0 - q.X, 0.0, q.X - x1)
            dy = max(y0 - q.Y, 0.0, q.Y - y1)
            dz = max(plate["z0"] - q.Z, 0.0, q.Z - top)
            worst = min(worst, math.sqrt(dx * dx + dy * dy + dz * dz) - r)  # to the plate's box
    return worst
