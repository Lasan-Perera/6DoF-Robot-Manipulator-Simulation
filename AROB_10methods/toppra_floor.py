"""General TOPP-RA time-optimal floor, for the ACTUAL 6-joint geometric path
of any method, in either experiment. Extends crosscheck_toppra.py's
already-verified single-joint result (TOPP-RA reproduces LSPB to 1.36e-6 s)
to the real multi-joint case this study needs.

The path must be parameterised by ARC LENGTH (cumulative Euclidean distance
along the joint-space samples), not by sample index. A first attempt used
uniform index spacing and was off by 5.25e-3 s from the known-correct LSPB
value, worsening rather than converging as TOPP-RA's own grid was refined --
the fault was in the SplineInterpolator's fit to unevenly-spaced-in-distance
points labelled as if evenly spaced, not in TOPP-RA's own resolution. With
arc-length parameterisation the same case matches to 1.37e-06 s, and the
result is now essentially stable under refinement.

Constraints: velocity and acceleration only (matching crosscheck_toppra.py's
own scope) -- no jerk constraint, so jerk-limited methods (double-S,
modsine) are expected to come in under 100% by design, not by error.
"""
import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint

ta.setup_logging('ERROR')


def topp_floor(pos_samples, v_max, a_max, n_grid=2001):
    """pos_samples: (N, ndof) position samples of a method's OWN geometric
    path (timing is discarded -- only the shape matters). Returns the
    TOPP-RA time-optimal duration for that exact path under v_max/a_max."""
    pos_samples = np.asarray(pos_samples, dtype=float)
    n, ndof = pos_samples.shape
    seglen = np.linalg.norm(np.diff(pos_samples, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    if s[-1] == 0.0:
        return 0.0
    s = s / s[-1]
    path = ta.SplineInterpolator(s, pos_samples)
    pc_vel = constraint.JointVelocityConstraint([[-v_max, v_max]] * ndof)
    pc_acc = constraint.JointAccelerationConstraint([[-a_max, a_max]] * ndof)
    gridpoints = np.linspace(0, path.duration, n_grid)
    inst = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, solver_wrapper='seidel')
    traj = inst.compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError('TOPP-RA returned no trajectory')
    return traj.duration
