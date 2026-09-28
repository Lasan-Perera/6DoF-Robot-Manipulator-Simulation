"""Cross-check: does TOPP-RA reproduce LSPB on a rest-to-rest segment?

PROTOCOL.md section 7 states that TOPP-RA under velocity and acceleration
limits alone should reproduce the trapezoidal profile, and that TOPP-RA
supplies the time-optimal floor every method's duration is reported against.
The first is an assertion about two constructions of the same time-optimal
problem; it is tested here rather than cited.

TOPP-RA parameterises a geometric PATH. For a single joint moving q0 -> qf the
path is the straight line itself, so the comparison is against the LSPB timing
of eq. (2), which is the time-optimal solution under a velocity and an
acceleration bound.
"""
from __future__ import annotations
import math
import numpy as np


def lspb_duration(h, v_max, a_max):
    if h >= v_max ** 2 / a_max:
        tb = v_max / a_max
        return tb + h / v_max, tb
    tb = math.sqrt(h / a_max)
    return 2 * tb, tb


def lspb_position(t, h, v_max, a_max):
    T, tb = lspb_duration(h, v_max, a_max)
    V = h / (T - tb)
    alpha = V / tb
    t = np.clip(np.asarray(t, float), 0.0, T)
    q = np.empty_like(t)
    m1, m3 = t < tb, t > T - tb
    m2 = ~(m1 | m3)
    q[m1] = 0.5 * alpha * t[m1] ** 2
    q[m2] = (h - V * T) / 2 + V * t[m2]
    tp = T - t[m3]
    q[m3] = h - 0.5 * alpha * tp ** 2
    return q, T


def toppra_profile(h, v_max, a_max, n_grid=2001):
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
    ta.setup_logging('ERROR')
    # straight line in one joint, parameterised on s in [0, 1]
    path = ta.SplineInterpolator([0.0, 1.0], np.array([[0.0], [h]]))
    pc_vel = constraint.JointVelocityConstraint([[-v_max, v_max]])
    pc_acc = constraint.JointAccelerationConstraint([[-a_max, a_max]])
    gridpoints = np.linspace(0, path.duration, n_grid)
    inst = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints,
                       solver_wrapper='seidel')
    traj = inst.compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError('TOPP-RA returned no trajectory')
    return traj


def main():
    cases = [('Exp 1 segment', 0.47319, 0.5, 1.0),
             ('short (no cruise phase)', 0.02, 0.5, 1.0)]
    print(f"{'case':26} {'LSPB T':>9} {'TOPPRA T':>9} {'dT':>10} {'max |dq|':>11}")
    worst_T = worst_q = 0.0
    for label, h, v_max, a_max in cases:
        T_lspb, _ = lspb_duration(h, v_max, a_max)
        traj = toppra_profile(h, v_max, a_max)
        T_top = traj.duration
        ts = np.linspace(0, min(T_lspb, T_top), 4001)
        q_top = np.asarray(traj(ts)).reshape(-1)
        q_lspb, _ = lspb_position(ts, h, v_max, a_max)
        dq = float(np.max(np.abs(q_top - q_lspb)))
        worst_T = max(worst_T, abs(T_lspb - T_top))
        worst_q = max(worst_q, dq)
        print(f'{label:26} {T_lspb:9.6f} {T_top:9.6f} '
              f'{abs(T_lspb - T_top):10.2e} {dq:11.2e}')
    print(f'\nworst duration difference : {worst_T:.3e} s')
    print(f'worst position difference : {worst_q:.3e} rad')
    # TOPP-RA discretises the path, so exact agreement is not expected; the
    # claim is that it recovers the same profile, not the same floating point.
    ok = worst_T < 1e-3 and worst_q < 1e-3
    print('VERDICT:', 'reproduces LSPB' if ok else 'DOES NOT reproduce LSPB -- do not claim it')


if __name__ == '__main__':
    main()
