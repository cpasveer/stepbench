# Run by FreeCAD (freecadcmd): runs the agent's FreeCAD script and prints a
# full Python traceback if it fails. freecadcmd on its own prints only the
# exception message, without the line, which is a poorer error than a
# build123d user gets from plain Python.
#
#   JB_SCRIPT=submission_freecad.py JB_TASK=<task> freecadcmd _fcrun.py
import os
import runpy
import sys
import traceback

script = os.environ["JB_SCRIPT"]
sys.argv = [script]
try:
    runpy.run_path(script, run_name="__main__")
except SystemExit as e:
    if e.code not in (None, 0):
        print(f"JB_FAILED exit {e.code}")
except BaseException:
    traceback.print_exc(file=sys.stdout)
    print("JB_FAILED")
else:
    print("JB_DONE")
