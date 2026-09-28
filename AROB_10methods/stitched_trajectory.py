""""Stitched" multi-segment trajectory for the eight point-to-point methods
(1-8), run back to back across a waypoint list with a full stop (v=0) at every
interior waypoint. This is the P1-8 baseline the two non-stopping methods (9,
10) are compared against in Experiment 2.

One dispatch table, not eight special cases: the six Group A methods (cubic,
quintic, septic, cycloidal, modsine, modtrap) go through
shape_trajectories.METHODS, already verified in phase 1 -- this is the SAME
cubic/quintic math as phase 1, not a second implementation of it. LSPB goes
through lspb_full_profile.py (its own construction, no jerk term). Double-S
goes through double_s_trajectory.py (phase 2).

dt = 0.01 s everywhere (P1: the earlier four-method study used 0.02 in
Experiment 2, inconsistent with Experiment 1's 0.01; fixed here).
"""
import numpy as np
import lspb_full_profile as lfp
import shape_trajectories as st
from double_s_trajectory import METHOD as DS

METHOD_NAMES = ('lspb', 'cubic', 'quintic', 'septic', 'cycloidal', 'modsine', 'modtrap', 'doubles')


def _segment_profile(method, q0, qf, v_max, a_max, j_max, dt):
    """Returns (ts, pos, vel, acc, T) for one must-stop segment."""
    q0 = np.asarray(q0, dtype=float); qf = np.asarray(qf, dtype=float)
    if method == 'lspb':
        ts, pos, vel, acc, tb, tf = lfp.lspb_full_profile(q0, qf, v_max, a_max, dt=dt)
        return ts, pos, vel, acc, tf
    if method == 'doubles':
        delta = qf - q0
        pacing_idx = int(np.argmax(np.abs(delta)))
        T = DS.segment_duration(delta[pacing_idx], v_max, a_max, j_max)
        ts, pos, vel, acc = DS.full_profile(q0, qf, T, dt, v_max, a_max, j_max)
        return ts, pos, vel, acc, T
    if method in st.METHODS:
        m = st.METHODS[method]
        delta = qf - q0
        pacing_idx = int(np.argmax(np.abs(delta)))
        T = m.segment_duration(delta[pacing_idx], v_max, a_max, j_max)
        ts, pos, vel, acc = m.full_profile(q0, qf, T, dt)
        return ts, pos, vel, acc, T
    raise ValueError(f'unknown method: {method}')


def generate_stitched_trajectory(method, waypoints, v_max, a_max, j_max, dt=0.01):
    """
    waypoints: (n, ndof). Runs n-1 independent must-stop segments back to
    back. Returns (ts, pos, vel, acc, T_segments, peak_v, peak_a, n_stops).
    ts is the GLOBAL time axis (cumulative across segments). n_stops = n,
    every waypoint is a forced v=0 stop, by construction.
    """
    waypoints = np.asarray(waypoints, dtype=float)
    n = waypoints.shape[0]

    ts_all, pos_all, vel_all, acc_all = [], [], [], []
    T_segments = []
    t_global = 0.0

    for i in range(n - 1):
        q0, qf = waypoints[i], waypoints[i + 1]
        ts, pos, vel, acc, T = _segment_profile(method, q0, qf, v_max, a_max, j_max, dt)
        T_segments.append(T)
        start_idx = 1 if i > 0 else 0            # drop the duplicated shared sample
        for k in range(start_idx, len(ts)):
            ts_all.append(t_global + ts[k])
            pos_all.append(pos[k]); vel_all.append(vel[k]); acc_all.append(acc[k])
        t_global += T

    ts_all = np.array(ts_all); pos_all = np.array(pos_all)
    vel_all = np.array(vel_all); acc_all = np.array(acc_all)
    T_segments = np.array(T_segments)

    peak_v = float(np.max(np.abs(vel_all)))
    peak_a = float(np.max(np.abs(acc_all)))
    return ts_all, pos_all, vel_all, acc_all, T_segments, peak_v, peak_a, n
