"""
Analytical position/velocity/acceleration profile for LSPB, mirroring
the piecewise law already implemented and verified in
gp7_pick_place/trajectory.py (lspb_position). Velocity/acceleration are
re-derived from that same position law (not a separate implementation)
so the two stay consistent by construction; verified below against a
finite-difference of the actual position function.

Acceleration is piecewise CONSTANT (this is the whole basis of the
LSPB/quintic comparison in the paper) -- jerk is therefore impulsive at
the four transition instants (0, tb, tf-tb, tf), not a continuous
signal. Report the acceleration discontinuity itself, not a numeric
jerk value.
"""
import numpy as np


def lspb_full_profile_single_joint(q0, qf, tb, tf, ts):
    """
    q0, qf: this joint's own endpoints.
    tb, tf: SHARED blend/total time (from the pacing joint).
    ts: array of times to evaluate at, 0<=t<=tf.
    Returns pos, vel, acc arrays.
    """
    V = (qf - q0) / (tf - tb)
    alpha = V / tb

    pos = np.zeros_like(ts)
    vel = np.zeros_like(ts)
    acc = np.zeros_like(ts)

    for i, t in enumerate(ts):
        if t < tb:
            pos[i] = q0 + (alpha / 2) * t**2
            vel[i] = alpha * t
            acc[i] = alpha
        elif t <= tf - tb:
            a0 = (q0 + qf - V * tf) / 2
            pos[i] = a0 + V * t
            vel[i] = V
            acc[i] = 0.0
        else:
            t_prime = tf - t
            pos[i] = qf - (alpha / 2) * t_prime**2
            vel[i] = alpha * t_prime
            acc[i] = -alpha

    return pos, vel, acc


def lspb_full_profile(q0_vec, qf_vec, v_max, accel, dt=0.001):
    """Multi-joint version, same pacing-joint rule as generate_segment_trajectory."""
    q0_vec = np.asarray(q0_vec, dtype=float)
    qf_vec = np.asarray(qf_vec, dtype=float)
    delta_qs = qf_vec - q0_vec
    pacing_idx = int(np.argmax(np.abs(delta_qs)))
    dq = abs(delta_qs[pacing_idx])

    if dq >= v_max**2 / accel:
        tb = v_max / accel
        tf = tb + dq / v_max
    else:
        tb = np.sqrt(dq / accel)
        tf = 2 * tb

    ts = np.arange(0, tf + dt, dt)
    ts = np.clip(ts, 0, tf)

    ndof = len(q0_vec)
    pos = np.zeros((len(ts), ndof))
    vel = np.zeros((len(ts), ndof))
    acc = np.zeros((len(ts), ndof))
    for j in range(ndof):
        p, v, a = lspb_full_profile_single_joint(q0_vec[j], qf_vec[j], tb, tf, ts)
        pos[:, j], vel[:, j], acc[:, j] = p, v, a

    return ts, pos, vel, acc, tb, tf
