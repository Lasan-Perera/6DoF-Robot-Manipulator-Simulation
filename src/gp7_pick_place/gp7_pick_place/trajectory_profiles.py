import numpy as np
from scipy.interpolate import CubicSpline


def _pacing_joint(q0, qf):
    delta_qs = [qf[i] - q0[i] for i in range(6)]
    idx = max(range(6), key=lambda i: abs(delta_qs[i]))
    return delta_qs[idx]


def max_kinematic_values(trajectory, dt):
    """Finite-difference peak |velocity| / |acceleration| per joint --
    needed for B-spline, whose peaks have no closed form, and useful as
    a post-hoc constraint check on all four methods."""
    traj = np.array(trajectory)
    vel = np.diff(traj, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    return np.max(np.abs(vel), axis=0), np.max(np.abs(acc), axis=0)


# ---------------------------------------------------------------------------
# 1. Trapezoidal velocity profile (LSPB) -- unchanged math from trajectory.py
# ---------------------------------------------------------------------------

def segment_timing_trapezoidal(delta_q, v_max, accel):
    if abs(delta_q) >= v_max**2 / accel:
        tb = v_max / accel
        tf = tb + abs(delta_q) / v_max
    else:
        tb = (abs(delta_q) / accel) ** 0.5
        tf = 2 * tb
    return tb, tf


def trapezoidal_position(t, q0, qf, tb, tf):
    V_i = (qf - q0) / (tf - tb)
    alpha_i = V_i / tb
    if t < tb:
        return q0 + (alpha_i / 2) * t**2
    elif t <= tf - tb:
        a0 = (q0 + qf - V_i * tf) / 2
        return a0 + V_i * t
    else:
        t_prime = tf - t
        return qf - (alpha_i / 2) * t_prime**2


def generate_trapezoidal_trajectory(q0, qf, v_max, accel, dt=0.05):
    tb, tf = segment_timing_trapezoidal(_pacing_joint(q0, qf), v_max, accel)
    trajectory, t = [], 0.0
    while t <= tf:
        trajectory.append([q0[i] if q0[i] == qf[i] else
                            trapezoidal_position(t, q0[i], qf[i], tb, tf)
                            for i in range(6)])
        t += dt
    if trajectory[-1] != list(qf):
        trajectory.append(list(qf))
    return trajectory, tf


# ---------------------------------------------------------------------------
# 2. Cubic polynomial -- position + velocity boundary conditions ONLY.
#    Acceleration is unconstrained -> nonzero at t=0 and t=T. That's not a
#    bug, it's the point: this is your evidence for "why quintic exists".
#    s(tau)=3tau^2-2tau^3, s'max=1.5 @ tau=0.5, s''max=6 @ tau=0 or 1.
# ---------------------------------------------------------------------------

def segment_timing_cubic(delta_q, v_max, accel):
    T_vel = 1.5 * abs(delta_q) / v_max
    T_acc = (6 * abs(delta_q) / accel) ** 0.5
    return max(T_vel, T_acc)


def cubic_position(t, q0, qf, T):
    delta_q = qf - q0
    a2 = 3 * delta_q / T**2
    a3 = -2 * delta_q / T**3
    return q0 + a2 * t**2 + a3 * t**3


def generate_cubic_trajectory(q0, qf, v_max, accel, dt=0.05):
    T = segment_timing_cubic(_pacing_joint(q0, qf), v_max, accel)
    trajectory, t = [], 0.0
    while t <= T:
        trajectory.append([q0[i] if q0[i] == qf[i] else
                            cubic_position(t, q0[i], qf[i], T)
                            for i in range(6)])
        t += dt
    if trajectory[-1] != list(qf):
        trajectory.append(list(qf))
    return trajectory, T


# ---------------------------------------------------------------------------
# 3. Quintic polynomial -- position + velocity + acceleration boundary
#    conditions, all zero at both ends. No discontinuity anywhere.
#    s(tau)=10tau^3-15tau^4+6tau^5, s'max=1.875 @ tau=0.5,
#    s''max=5.7735 @ tau~0.211/0.789, s'''max=60 @ tau=0 or 1.
# ---------------------------------------------------------------------------

def segment_timing_quintic(delta_q, v_max, accel):
    T_vel = 1.875 * abs(delta_q) / v_max
    T_acc = (5.7735 * abs(delta_q) / accel) ** 0.5
    return max(T_vel, T_acc)


def quintic_position(t, q0, qf, T):
    delta_q = qf - q0
    a3 = 10 * delta_q / T**3
    a4 = -15 * delta_q / T**4
    a5 = 6 * delta_q / T**5
    return q0 + a3 * t**3 + a4 * t**4 + a5 * t**5


def generate_quintic_trajectory(q0, qf, v_max, accel, dt=0.05):
    T = segment_timing_quintic(_pacing_joint(q0, qf), v_max, accel)
    trajectory, t = [], 0.0
    while t <= T:
        trajectory.append([q0[i] if q0[i] == qf[i] else
                            quintic_position(t, q0[i], qf[i], T)
                            for i in range(6)])
        t += dt
    if trajectory[-1] != list(qf):
        trajectory.append(list(qf))
    return trajectory, T


# ---------------------------------------------------------------------------
# 4. B-spline -- architecturally different from the three above: takes a
#    LIST of waypoints, not just q0/qf, because its entire point is passing
#    through interior points at nonzero velocity (Experiment 2 only).
#
#    A clamped cubic spline IS an interpolating cubic B-spline -- same
#    curve, solved via the standard tridiagonal system instead of explicit
#    basis functions. bc_type='clamped' fixes velocity=0 at the FIRST and
#    LAST waypoint only; interior waypoints are free (no stop).
#
#    `accel` is accepted but not used to derive T in closed form -- unlike
#    the other three, peak accel here depends on the whole waypoint set
#    jointly, not one segment. Use max_kinematic_values() after generating
#    to check the limit; rescale segment_times and regenerate if violated.
# ---------------------------------------------------------------------------

def _segment_times(waypoints, v_max):
    # Velocity-limited chord parameterisation: each segment gets the time
    # its pacing joint needs at v_max. A documented design choice, not the
    # only valid one -- state it plainly in the methodology section.
    times = [0.0]
    for k in range(1, len(waypoints)):
        delta = max(abs(waypoints[k][i] - waypoints[k - 1][i]) for i in range(6))
        times.append(times[-1] + max(delta / v_max, 1e-3))
    return times


def generate_bspline_trajectory(waypoints, v_max, accel, dt=0.05):
    times = _segment_times(waypoints, v_max)
    T = times[-1]
    splines = [CubicSpline(times, [wp[i] for wp in waypoints], bc_type='clamped')
               for i in range(6)]

    trajectory, t = [], 0.0
    while t <= T:
        trajectory.append([float(splines[i](t)) for i in range(6)])
        t += dt
    if trajectory[-1] != list(waypoints[-1]):
        trajectory.append(list(waypoints[-1]))
    return trajectory, T