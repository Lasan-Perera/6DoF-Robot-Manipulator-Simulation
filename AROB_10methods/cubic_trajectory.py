"""
Cubic polynomial joint-space trajectory (rest-to-rest velocity), per
AROB paper section 4.3.

s(tau) = 3*tau^2 - 2*tau^3,  tau = t/T in [0,1]

Only 4 boundary conditions (position + velocity at each end) -- with
zero endpoint velocity, acceleration is UNCONSTRAINED and peaks exactly
at the boundaries (tau=0, tau=1). This is the defining property that
motivates quintic: the controller is handed a step in commanded
acceleration as the segment begins and another as it ends.

Closed-form peaks: |s'|max = 3/2 at tau=1/2, |s''|max = 6 at tau in {0,1}.
"""
import numpy as np

S1_MAX = 3.0 / 2.0
S2_MAX = 6.0


def s_and_derivs(tau):
    tau = np.asarray(tau, dtype=float)
    s   = 3*tau**2 - 2*tau**3
    sp  = 6*tau    - 6*tau**2
    spp = 6        - 12*tau
    return s, sp, spp


def segment_duration(delta_q: float, v_max: float, a_max: float) -> float:
    dq = abs(delta_q)
    if dq == 0:
        return 0.0
    T_v = S1_MAX * dq / v_max
    T_a = np.sqrt(S2_MAX * dq / a_max)
    return max(T_v, T_a)


def cubic_position(t, q0, qf, T):
    if T == 0:
        return q0
    tau = np.clip(t / T, 0.0, 1.0)
    s, _, _ = s_and_derivs(tau)
    return q0 + (qf - q0) * s


def generate_segment_trajectory(q0, qf, v_max, a_max, dt=0.05):
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    delta_qs = qf - q0
    pacing_idx = int(np.argmax(np.abs(delta_qs)))
    T = segment_duration(delta_qs[pacing_idx], v_max, a_max)

    trajectory = []
    t = 0.0
    while t <= T:
        point = [cubic_position(t, q0[i], qf[i], T) for i in range(len(q0))]
        trajectory.append(point)
        t += dt

    if T > 0 and not np.allclose(trajectory[-1], qf):
        trajectory.append(list(qf))
    if T == 0:
        trajectory = [list(q0)]

    return trajectory, T


def cubic_full_profile(q0, qf, T, dt=0.001):
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    delta_qs = qf - q0
    ts = np.arange(0, T + dt, dt)
    ts = np.clip(ts, 0, T)
    tau = ts / T
    s, sp, spp = s_and_derivs(tau)

    pos = q0[None, :] + delta_qs[None, :] * s[:, None]
    vel = delta_qs[None, :] * sp[:, None] / T
    acc = delta_qs[None, :] * spp[:, None] / T**2
    return ts, pos, vel, acc
