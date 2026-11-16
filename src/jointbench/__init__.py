"""JointBench: a generated 3D CAD benchmark -- design the connecting part."""
__version__ = "0.1.0"

# The one place the project name lives in code. A rename is: this constant,
# the package folder, and the two names in pyproject.toml.
NAME = "jointbench"

# exact areas and volumes on every kind of STEP file (see exact_measure.py)
from . import exact_measure  # noqa: E402,F401
