# A connector built in FreeCAD (not build123d), reading only task.json.
# Works for level 1 and level 2: everything is built in each face's own frame.
# Run:  freecadcmd examples/freecad_connector.py tasks/L2-s0
# then grade it with the benchmark's `verify` command
import json
import math
import sys

import FreeCAD as App
import Part

task_dir = sys.argv[-1]
T = json.load(open(task_dir + "/task.json"))
pose = T["poses"][0]
R = pose["resolved"]
faces = {f["part"]: f for f in R["mounting_faces"]}
bolts = R["bolts"]
TAB = 20.0  # longer than the generator's reference tab


def V(p):
    return App.Vector(*p)


def frame(part, side):
    """Face frame whose x axis points along ``side`` (projected into the face)."""
    o, n = V(faces[part]["origin"]), V(faces[part]["normal"])
    u = side - n * side.dot(n)
    if u.Length < 1e-6:
        u = n.cross(App.Vector(1, 0, 0)) if abs(n.x) < 0.9 else n.cross(App.Vector(0, 1, 0))
    u.normalize()
    return o, u, n.cross(u), n


def hull(points, eps=1e-7):
    """Convex hull of a few points as a solid with flat faces."""
    import itertools
    faces_, seen = [], set()
    for i, j, k in itertools.combinations(range(len(points)), 3):
        nn = (points[j] - points[i]).cross(points[k] - points[i])
        if nn.Length < 1e-9:
            continue
        nn.normalize()
        d = [(p - points[i]).dot(nn) for p in points]
        if max(d) > eps and min(d) < -eps:
            continue
        on = tuple(sorted(m for m, x in enumerate(d) if abs(x) <= eps))
        if on in seen:
            continue
        seen.add(on)
        c = sum((points[m] for m in on), App.Vector()) * (1.0 / len(on))
        uu = points[on[0]] - c
        uu.normalize()
        vv = nn.cross(uu)
        ring = sorted(on, key=lambda m: math.atan2((points[m] - c).dot(vv), (points[m] - c).dot(uu)))
        poly = [points[m] for m in ring]
        faces_.append(Part.Face(Part.makePolygon(poly + [poly[0]])))
    return Part.Solid(Part.Shell(faces_))


# Which side the tabs and bridge go: towards the other part when the parts
# are well apart sideways; otherwise both tabs on the same side, beside the
# bolts, like a wall.
oa, ob = V(faces["A"]["origin"]), V(faces["B"]["origin"])
na = V(faces["A"]["normal"])
w = ob - oa
lateral = w - na * w.dot(na)
if lateral.Length > 30:
    sides = {"A": w, "B": -w}
else:
    side = na.cross(App.Vector(0, 0, 1)) if abs(na.z) < 0.9 else App.Vector(1, 0, 0)
    sides = {"A": side, "B": side}


plates, quads, thick = [], {}, {}
for part in ("A", "B"):
    o, u, v, n = frame(part, sides[part])
    mine = [b for b in bolts if b["id"][0] == part]
    t = math.ceil(mine[0]["grip_range"][0])
    thick[part] = t
    e = mine[0]["head_diameter"] / 2 + 3
    xs = [(V(b["position"]) - o).dot(u) for b in mine]
    ys = [(V(b["position"]) - o).dot(v) for b in mine]
    x0, x1, y0, y1 = min(xs) - e, max(xs) + e + TAB, min(ys) - e, max(ys) + e

    def P(x, y, z):
        return o + u * x + v * y + n * z

    base = Part.Face(Part.makePolygon([P(x0, y0, 0), P(x1, y0, 0), P(x1, y1, 0), P(x0, y1, 0), P(x0, y0, 0)]))
    plates.append(base.extrude(n * t))
    xa = x1 - TAB
    quads[part] = [P(xa, y0, t - 0.5), P(x1, y0, t - 0.5), P(x1, y1, t - 0.5), P(xa, y1, t - 0.5)]

bridge = hull(quads["A"] + quads["B"])

body = plates[0].fuse(bridge).fuse(plates[1])
for b in bolts:
    d = b["clearance_diameter_range"][0] + 0.1
    p, a = V(b["position"]), V(b["axis"])
    body = body.cut(Part.makeCylinder(d / 2, thick[b["id"][0]] + 1, p - a * 0.5, a))
for fp in T["fixed_parts"]:
    pl = pose["placements"][fp["name"]]
    o, M = pl["origin"], pl["rotation"]
    n = App.Vector(M[0][2], M[1][2], M[2][2])
    for pr in fp["interface"]["protrusions"]:
        c = [sum(M[i][k] * [pr["x"], pr["y"], 0][k] for k in range(3)) + o[i] for i in range(3)]
        body = body.cut(Part.makeCylinder(pr["radius"] + 0.6, pr["height"] + 1, V(c) - n * 0.01, n))
body = body.removeSplitter()
body.exportStep(task_dir + "/output.step")
print("wrote", task_dir + "/output.step", body.isValid(), len(body.Solids))
