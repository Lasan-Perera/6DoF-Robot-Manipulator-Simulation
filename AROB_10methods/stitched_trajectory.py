"""
"Stitched" multi-segment trajectory: run LSPB/cubic/quintic three times
back-to-back across a waypoint list, forcing v=0 (a full stop) at every
interior waypoint -- this is how these three methods are NORMALLY
deployed (each is only defined for a single must-stop segment), used
as the comparison baseline against the B-spline's single continuous
pass-through (bspline_trajectory.generate_via_point_trajectory).
"""
import numpy as np
import lspb_full_profile as lfp
import cubic_trajectory as ct
import quintic_trajectory as qt


def _segment_profile(method, q0, qf, v_max, a_max, dt):
    if method == "lspb":
        ts, pos, vel, acc, tb, tf = lfp.lspb_full_profile(q0, qf, v_max, a_max, dt=dt)
        return ts, pos, vel, acc, tf
    elif method == "cubic":
        delta = qf - q0
        pacing_idx = int(np.argmax(np.abs(delta)))
        T = ct.segment_duration(delta[pacing_idx], v_max, a_max)
        ts, pos, vel, acc = ct.cubic_full_profile(q0, qf, T, dt=dt)
        return ts, pos, vel, acc, T
    elif method == "quintic":
        delta = qf - q0
        pacing_idx = int(np.argmax(np.abs(delta)))
        T = qt.segment_duration(delta[pacing_idx], v_max, a_max)
        ts, pos, vel, acc, _ = qt.quintic_full_profile(q0, qf, T, dt=dt)
        return ts, pos, vel, acc, T
    else:
        raise ValueError(method)


def generate_stitched_trajectory(method, waypoints, v_max, a_max, dt=0.02):
    """
    waypoints: (n, ndof). Runs n-1 independent must-stop segments back to
    back. Returns (ts, pos, vel, acc, T_segments, n_stops) -- ts is the
    GLOBAL time axis (cumulative across segments), n_stops is the number
    of waypoints where velocity is forced to 0 (= n, every waypoint,
    since each segment independently starts/ends at rest).
    """
    waypoints = np.asarray(waypoints, dtype=float)
    n = waypoints.shape[0]

    ts_all, pos_all, vel_all, acc_all = [], [], [], []
    T_segments = []
    t_global = 0.0

    for i in range(n - 1):
        q0, qf = waypoints[i], waypoints[i + 1]
        ts, pos, vel, acc, T = _segment_profile(method, q0, qf, v_max, a_max, dt)
        T_segments.append(T)

        # drop the very first sample of every segment AFTER the first one,
        # since it's identical to the previous segment's last sample
        # (both are exactly the shared waypoint, v=0)
        start_idx = 1 if i > 0 else 0
        for k in range(start_idx, len(ts)):
            ts_all.append(t_global + ts[k])
            pos_all.append(pos[k])
            vel_all.append(vel[k])
            acc_all.append(acc[k])

        t_global += T

    ts_all = np.array(ts_all)
    pos_all = np.array(pos_all)
    vel_all = np.array(vel_all)
    acc_all = np.array(acc_all)
    T_segments = np.array(T_segments)

    peak_v = np.max(np.abs(vel_all))
    peak_a = np.max(np.abs(acc_all))
    n_stops = n  # every waypoint is a forced v=0 stop, by construction

    return ts_all, pos_all, vel_all, acc_all, T_segments, peak_v, peak_a, n_stops
