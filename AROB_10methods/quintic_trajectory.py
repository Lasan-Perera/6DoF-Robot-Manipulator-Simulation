"""
Quintic polynomial joint-space trajectory (rest-to-rest), per AROB paper
section 4.4/4.6.

s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5,  tau = t/T in [0,1]
q(t)  = q0 + dq * s(tau)
qd(t) =      dq * s'(tau) / T
qdd(t)=      dq * s''(tau) / T^2
qddd(t)=     dq * s'''(tau) / T^3

Closed-form peaks (paper Eq. 5): |s'|max=15/8, |s''|max=10/sqrt(3),
|s'''|max=60, all attained inside (0,1), not at the endpoints.

Multi-joint synchronisation: same "pacing joint" rule as LSPB (section
4.2) -- the joint with the largest |Delta q| sets T via Eq. 6; every
other joint is sampled on that same T, so all six start/stop together
and the joint-space path is a straight line (same requirement stated
for LSPB, carried over here for a fair comparison).
"""
import numpy as np

S1_MAX = 15.0 / 8.0          # |s'|max
S2_MAX = 10.0 / np.sqrt(3.0)  # |s''|max
S3_MAX = 60.0                 # |s'''|max


def s_and_derivs(tau):
    tau = np.asarray(tau, dtype=float)
    s   = 10*tau**3 - 15*tau**4 + 6*tau**5
    sp  = 30*tau**2 - 60*tau**3 + 30*tau**4
    spp = 60*tau    - 180*tau**2 + 120*tau**3
    sppp= 60        - 360*tau    + 360*tau**2
    return s, sp, spp, sppp


def segment_duration(delta_q: float, v_max: float, a_max: float) -> float:
    """T_min respecting both v_max and a_max (paper Eq. 6)."""
    dq = abs(delta_q)
    if dq == 0:
        return 0.0
    T_v = S1_MAX * dq / v_max
    T_a = np.sqrt(S2_MAX * dq / a_max)
    return max(T_v, T_a)


def quintic_position(t, q0, qf, T):
    if T == 0:
        return q0
    tau = np.clip(t / T, 0.0, 1.0)
    s, _, _, _ = s_and_derivs(tau)
    return q0 + (qf - q0) * s


def generate_segment_trajectory(q0, qf, v_max, a_max, dt=0.05):
    """
    Same interface shape as trajectory.generate_segment_trajectory():
    find the pacing joint (max |Delta q|), get shared T from it, sample
    every joint at that same T using ITS OWN delta_q (so all 6 arrive
    together). Returns positions only, matching the existing LSPB
    generator's return type; velocities/accelerations available via
    quintic_full_profile() below for the paper's kinematic-profile plots.
    """
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    delta_qs = qf - q0
    pacing_idx = int(np.argmax(np.abs(delta_qs)))
    T = segment_duration(delta_qs[pacing_idx], v_max, a_max)

    trajectory = []
    t = 0.0
    while t <= T:
        point = [quintic_position(t, q0[i], qf[i], T) for i in range(len(q0))]
        trajectory.append(point)
        t += dt

    if T > 0 and not np.allclose(trajectory[-1], qf):
        trajectory.append(list(qf))
    if T == 0:
        trajectory = [list(q0)]

    return trajectory, T


def quintic_full_profile(q0, qf, T, dt=0.001):
    """For the paper's kinematic plots: full pos/vel/accel/jerk per joint."""
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    delta_qs = qf - q0
    ts = np.arange(0, T + dt, dt)
    ts = np.clip(ts, 0, T)
    tau = ts / T
    s, sp, spp, sppp = s_and_derivs(tau)

    pos = q0[None, :] + delta_qs[None, :] * s[:, None]
    vel = delta_qs[None, :] * sp[:, None] / T
    acc = delta_qs[None, :] * spp[:, None] / T**2
    jerk = delta_qs[None, :] * sppp[:, None] / T**3
    return ts, pos, vel, acc, jerk
