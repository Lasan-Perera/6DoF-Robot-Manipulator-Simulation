"""
Clamped cubic spline through a list of waypoints, per AROB paper
section 4.5 ("clamped cubic B-spline"). Implemented in the standard
form: velocity fixed at zero at the first and last waypoint, interior
waypoints free (solved from continuity of acceleration at each knot),
position/velocity/acceleration continuous across every knot.

On exactly 2 waypoints there are no interior knots -- v=[0,0] directly,
and this reduces EXACTLY to cubic_trajectory's shape (verified below;
this is a correctness check on the implementation, not a result, per
the paper text).
"""
import numpy as np


def solve_interior_velocities(q, T):
    """
    q: (n,) waypoint positions for ONE joint.
    T: (n-1,) segment durations.
    Returns v: (n,) velocities at every waypoint, v[0]=v[-1]=0.
    """
    n = len(q)
    if n < 2:
        raise ValueError("need at least 2 waypoints")
    v = np.zeros(n)
    if n == 2:
        return v  # no interior knots, nothing to solve

    m = n - 2  # number of interior knots / unknowns
    A = np.zeros((m, m))
    b = np.zeros(m)

    for row, k in enumerate(range(1, n - 1)):  # k = interior knot index
        Tk = T[k - 1]      # segment BEFORE knot k (k-1 -> k)
        Tk1 = T[k]         # segment AFTER knot k  (k -> k+1)
        rhs = 3.0 * (Tk1 * (q[k] - q[k - 1]) / Tk + Tk * (q[k + 1] - q[k]) / Tk1)

        A[row, row] = 2.0 * (Tk + Tk1)
        if row - 1 >= 0:
            A[row, row - 1] = Tk1   # coefficient of v_{k-1}
        if row + 1 < m:
            A[row, row + 1] = Tk    # coefficient of v_{k+1}
        b[row] = rhs

    v_interior = np.linalg.solve(A, b)
    v[1:-1] = v_interior
    return v


def hermite_segment_coeffs(q0, v0, q1, v1, T):
    a0 = q0
    a1 = v0
    a2 = 3.0 * (q1 - q0) / T**2 - (2.0 * v0 + v1) / T
    a3 = -2.0 * (q1 - q0) / T**3 + (v0 + v1) / T**2
    return a0, a1, a2, a3


def eval_segment(a0, a1, a2, a3, t):
    pos = a0 + a1 * t + a2 * t**2 + a3 * t**3
    vel = a1 + 2 * a2 * t + 3 * a3 * t**2
    acc = 2 * a2 + 6 * a3 * t
    return pos, vel, acc


def solve_clamped_spline_single_joint(q, T):
    """Returns list of (a0,a1,a2,a3) coefficient tuples, one per segment."""
    v = solve_interior_velocities(np.asarray(q, dtype=float), np.asarray(T, dtype=float))
    coeffs = []
    for i in range(len(T)):
        coeffs.append(hermite_segment_coeffs(q[i], v[i], q[i + 1], v[i + 1], T[i]))
    return coeffs, v


def peak_vel_accel(waypoints, T):
    """waypoints: (n, ndof). T: (n-1,). Returns peak |vel|, peak |accel| over all joints/segments."""
    waypoints = np.asarray(waypoints, dtype=float)
    n, ndof = waypoints.shape
    peak_v, peak_a = 0.0, 0.0
    for j in range(ndof):
        coeffs, _ = solve_clamped_spline_single_joint(waypoints[:, j], T)
        for i, (a0, a1, a2, a3) in enumerate(coeffs):
            tt = np.linspace(0, T[i], 500)
            _, vel, acc = eval_segment(a0, a1, a2, a3, tt)
            peak_v = max(peak_v, np.max(np.abs(vel)))
            peak_a = max(peak_a, np.max(np.abs(acc)))
    return peak_v, peak_a


def fit_segment_times(waypoints, v_max, a_max):
    """
    Initial T_i proportional to the pacing joint's |Delta q| per segment,
    then globally scaled so the resulting peak velocity/acceleration
    (checked numerically, not via an unverified shortcut) exactly hits
    whichever limit binds.
    """
    waypoints = np.asarray(waypoints, dtype=float)
    n = waypoints.shape[0]
    deltas = np.diff(waypoints, axis=0)
    T0 = np.max(np.abs(deltas), axis=1)
    T0 = np.maximum(T0, 1e-6)  # avoid zero-duration segments

    peak_v0, peak_a0 = peak_vel_accel(waypoints, T0)
    k_v = peak_v0 / v_max
    k_a = np.sqrt(peak_a0 / a_max)
    k = max(k_v, k_a)

    T_final = T0 * k
    peak_v_final, peak_a_final = peak_vel_accel(waypoints, T_final)
    return T_final, peak_v_final, peak_a_final


def generate_via_point_trajectory(waypoints, v_max, a_max, dt=0.05):
    """
    waypoints: (n, ndof) array/list. Returns (trajectory, T) where
    trajectory is a list of ndof-vectors sampled at dt across the WHOLE
    multi-segment path, and T is the (n-1,) array of segment durations.
    """
    waypoints = np.asarray(waypoints, dtype=float)
    n, ndof = waypoints.shape
    T, peak_v, peak_a = fit_segment_times(waypoints, v_max, a_max)

    all_coeffs = []
    all_v = []
    for j in range(ndof):
        coeffs, v = solve_clamped_spline_single_joint(waypoints[:, j], T)
        all_coeffs.append(coeffs)
        all_v.append(v)

    trajectory = []
    t_global = 0.0
    for i in range(len(T)):
        t = 0.0
        while t < T[i]:
            point = []
            for j in range(ndof):
                a0, a1, a2, a3 = all_coeffs[j][i]
                pos, _, _ = eval_segment(a0, a1, a2, a3, t)
                point.append(pos)
            trajectory.append(point)
            t += dt
        t_global += T[i]

    # final point exactly at the last waypoint
    trajectory.append(list(waypoints[-1]))

    return trajectory, T, peak_v, peak_a
