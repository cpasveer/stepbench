"""Areas and volumes by adaptive integration, everywhere in the checker.

OpenCASCADE's default area and volume integration samples each boundary edge at
a fixed number of points. On a face bounded by one long closed B-spline edge
(Rhino writes a rounded rectangle that way) that is far too few, and the result
is badly wrong: a 2679 mm2 flange face came out as -2307 mm2, and the part's
volume as 218 000 mm3 instead of 88 000. The adaptive form of the same exact
integration refines until the relative error is below EPS; on simple faces it
agrees with the default to within 1e-6.

Importing this module (jointbench/__init__.py does) makes every call that does
not ask for a precision itself use the adaptive form: the checker's own calls,
and build123d's .area, .volume and centre-of-mass properties.
"""
from OCP.BRepGProp import BRepGProp

EPS = 1e-7  # relative error of every area and volume

_volume, _surface = BRepGProp.VolumeProperties_s, BRepGProp.SurfaceProperties_s


def _adaptive(plain):
    def call(shape, props, *args, **kw):
        if args or kw:  # the caller chose its own precision or options
            return plain(shape, props, *args, **kw)
        return plain(shape, props, EPS)
    call.__name__ = getattr(plain, "__name__", "properties")
    return call


def install() -> None:
    if getattr(BRepGProp, "_jointbench_adaptive", False):
        return
    BRepGProp.VolumeProperties_s = _adaptive(_volume)
    BRepGProp.SurfaceProperties_s = _adaptive(_surface)
    BRepGProp._jointbench_adaptive = True
    try:  # build123d copied the functions into a table when it was imported
        from build123d.topology.shape_core import Shape
        lut = Shape.shape_properties_LUT
        for k, f in list(lut.items()):
            if f is _volume:
                lut[k] = BRepGProp.VolumeProperties_s
            elif f is _surface:
                lut[k] = BRepGProp.SurfaceProperties_s
    except ImportError:
        pass


install()
