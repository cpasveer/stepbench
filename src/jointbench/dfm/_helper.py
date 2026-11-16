"""Helper process for splitting exact per-sample measurements over cores.
Reads (op, step path, job) as a pickle from the file named first, writes
the result as a pickle to the file named second. ops: "ball" (samples, normals, thicknesses -> ball
diameters), "rays" (samples, normals -> wall and gap ray lengths). See
geom._run_helpers."""
import pickle
import sys

from .geom import _ball_points, _faces_of, _rays_points, load


def main():
    with open(sys.argv[1], "rb") as fh:
        op, path, job = pickle.load(fh)
    shape = load(path).wrapped
    if op == "ball":
        P, N, T = job
        out = _ball_points(_faces_of(shape), P, N, T)
    elif op == "rays":
        P, N = job
        out = _rays_points(shape, P, N)
    else:
        raise SystemExit(f"unknown op {op}")
    with open(sys.argv[2], "wb") as fh:
        pickle.dump(out, fh)


if __name__ == "__main__":
    main()
