"""Build the compiled helper for the largest-ball measurement.

    uv run python -m jointbench.dfm.build_native

Needs Rust (https://rustup.rs). Compiles ``_native/`` and places the module
next to this file as ``_fastball.abi3.so`` (``.pyd`` on Windows); one build
serves every Python from 3.12 up. Without it the checker runs the exact
routine everywhere: identical verdicts, only slower.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def build() -> str:
    target = os.path.join(os.path.expanduser("~"), ".cache", "jointbench-native")
    env = dict(os.environ, CARGO_TARGET_DIR=target)
    # run inside _native/ so its .cargo/config.toml (macOS link flags) applies
    subprocess.run(["cargo", "build", "--release"], cwd=os.path.join(HERE, "_native"), check=True, env=env)
    rel = os.path.join(target, "release")
    src = next(os.path.join(rel, f) for f in ("lib_fastball.dylib", "lib_fastball.so", "_fastball.dll")
               if os.path.exists(os.path.join(rel, f)))
    dst = os.path.join(HERE, "_fastball.pyd" if sys.platform == "win32" else "_fastball.abi3.so")
    # copy beside, then rename over: a process that has the old module loaded
    # keeps its copy intact (overwriting a loaded library in place can crash it)
    tmp = dst + ".new"
    shutil.copyfile(src, tmp)
    os.replace(tmp, dst)
    return dst


if __name__ == "__main__":
    print(f"built {build()}")
