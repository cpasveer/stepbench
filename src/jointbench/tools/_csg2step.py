# Run by FreeCAD (freecadcmd), not by the jointbench Python.
# Reads an OpenSCAD CSG file with FreeCAD's own OpenSCAD importer and writes
# a STEP file. Circles and cylinders come through as true circles and
# cylinders when OpenSCAD's $fn is 0 (unset) or above 16; with $fn 3..16 the
# importer builds a polygonal prism instead, which is what the user asked for.
#
#   JB_CSG=in.csg JB_STEP=out.step freecadcmd _csg2step.py
import os
import sys

import FreeCAD as App
import Part
import importCSG

src, out = os.environ["JB_CSG"], os.environ["JB_STEP"]
doc = App.newDocument("jointbench_csg")
importCSG.insert(src, doc.Name)
doc.recompute()
roots = [o for o in doc.Objects if not o.InList and getattr(o, "Shape", None) is not None
         and not o.Shape.isNull()]
if not roots:
    print("JB_ERROR no geometry in the CSG file")
    sys.exit(2)
shape = roots[0].Shape
for o in roots[1:]:  # several top-level objects: OpenSCAD's implicit union
    shape = shape.fuse(o.Shape)
solids = shape.Solids
if len(solids) == 1:
    shape = solids[0].removeSplitter()
shape.exportStep(out)
print("JB_OK", len(solids))
