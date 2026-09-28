"""LSPB with via-point blends (method 10), six joints. Classical construction
(decision 4, reverted 2026-09-28: the pseudo-via-point version is over-
constrained for more than one joint -- see the note at the bottom of this
docstring).

Structure: start ramp (rest -> v1) -- linear 1 -- blend@Q1 (v1->v2)
           -- linear 2 -- blend@Q2 (v2->v3) -- linear 3 -- end ramp (v3->rest)

Per-joint nominal cruise velocities come from the TRUE waypoints and each
segment's own pacing-joint duration T_k = |dq_pace,k| / v_max (same rule as
every other stitched method). Blend durations are the pacing-joint rule of
sec. 4.4, generalised to six joints sharing one clock: at each interior
waypoint, t_b = max_j |dv_j| / a_max, long enough that EVERY joint's blend
acceleration stays within a_max, not only the segment's own pacing joint.
Same rule at the two outer ramps against rest.

Linear-segment durations follow the standard construction: each blend is
centred on the nominal cumulative time of its waypoint, and the linear portion
either side is what remains,
    T_lin,k = T_k - tb_before/2 - tb_after/2 .
This is well-posed for all six joints simultaneously (3 shared unknowns,
solved once, not per joint), unlike the pseudo-via-point construction tried
first: aiming the lines at a shifted point makes ONE joint pass through its
via point exactly, but with only 3 shared linear-segment durations and 6
joints x 2 interior waypoints = 12 independent exact-passage constraints, the
system is over-determined and the path missed q3 by 0.25 rad on the real
via-point waypoints. Confirmed structural, not a sign error, before reverting.

The path therefore does NOT pass exactly through qv1/qv2 -- it cuts the
corner, like the classical LSPB-via-point construction always has. The
deviation at each interior waypoint (evaluated at the blend's own midpoint,
which sits exactly at the nominal cumulative time by construction) is
computed and returned, per joint, as data: this is what C5-C7 compares
against the B-spline's near-zero deviation.

On exactly two waypoints there are no interior blends, only the start and end
ramps -- this reduces to plain LSPB, checked below.
"""
from __future__ import annotations
import numpy as np
from shape_trajectories import _time_grid


def _pacing_time(dq_seg, v_max):
    return float(np.max(np.abs(dq_seg))) / v_max


class LspbBlend:
    name = 'lspbblend'
    jerk_bounded = False
    endpoint_jerk_impulse = True

    def plan(self, W, v_max, a_max):
        """Returns (phases, T, deviations). phases: list of
        ('acc'|'lin', duration, p0, v0, a) with a=0 for 'lin'. deviations:
        {} on 2 waypoints, else {'qv1': (ndof,) array, 'qv2': (ndof,) array}
        of achieved-minus-intended position at each interior waypoint's
        nominal crossing time."""
        W = np.asarray(W, dtype=float)
        n, ndof = W.shape

        if n == 2:
            Q0, Q1 = W
            dq = Q1 - Q0
            T = _pacing_time(dq, v_max)
            tb = float(np.max(np.abs(dq))) / T / a_max   # = v_max/a_max exactly when the pacing joint reaches v_max
            # match plain LSPB exactly: use its own trapezoidal rule, not the
            # generic ramp/ramp split, since a single-segment "blend to rest
            # then immediately blend from rest" IS trapezoidal LSPB.
            pace = float(np.max(np.abs(dq)))
            if pace / v_max >= v_max / a_max:
                tb = v_max / a_max
                T = tb + pace / v_max
            else:
                tb = np.sqrt(pace / a_max)
                T = 2 * tb
            v_cruise = dq / (T - tb)
            a0 = v_cruise / tb
            p_after_ramp = Q0 + 0.5 * a0 * tb ** 2
            p_before_end = p_after_ramp + v_cruise * (T - 2 * tb)
            phases = [('acc', tb, Q0, np.zeros(ndof), a0),
                      ('lin', T - 2 * tb, p_after_ramp, v_cruise, np.zeros(ndof)),
                      ('acc', tb, p_before_end, v_cruise, -a0)]
            return phases, T, {}

        assert n == 4, 'written for exactly 4 waypoints'
        Q0, Q1, Q2, Q3 = W
        T1 = _pacing_time(Q1 - Q0, v_max)
        T2 = _pacing_time(Q2 - Q1, v_max)
        T3 = _pacing_time(Q3 - Q2, v_max)
        v1, v2, v3 = (Q1 - Q0) / T1, (Q2 - Q1) / T2, (Q3 - Q2) / T3

        tb0 = float(np.max(np.abs(v1))) / a_max
        tb1 = float(np.max(np.abs(v2 - v1))) / a_max
        tb2 = float(np.max(np.abs(v3 - v2))) / a_max
        tb3 = float(np.max(np.abs(v3))) / a_max
        a0v = v1 / tb0
        a1 = (v2 - v1) / tb1
        a2 = (v3 - v2) / tb2
        a3v = -v3 / tb3

        T_lin1 = T1 - tb0 / 2 - tb1 / 2
        T_lin2 = T2 - tb1 / 2 - tb2 / 2
        T_lin3 = T3 - tb2 / 2 - tb3 / 2
        if min(T_lin1, T_lin2, T_lin3) < 0:
            raise ValueError('a blend duration exceeds its adjacent nominal segment time; '
                              'v_max/a_max too aggressive for these waypoints')

        # forward integration, phase by phase
        p = Q0.copy()
        phases = []
        phases.append(('acc', tb0, p.copy(), np.zeros(ndof), a0v))
        p = p + v1 * tb0 / 2

        t_qv1_mid = tb0 + T_lin1 + tb1 / 2      # nominal crossing time of Q1
        phases.append(('lin', T_lin1, p.copy(), v1, np.zeros(ndof)))
        p_before_blend1 = p + v1 * T_lin1
        phases.append(('acc', tb1, p_before_blend1.copy(), v1, a1))
        p_after_blend1 = p_before_blend1 + v1 * tb1 + 0.5 * a1 * tb1 ** 2
        blend1_mid = p_before_blend1 + v1 * (tb1 / 2) + 0.5 * a1 * (tb1 / 2) ** 2

        phases.append(('lin', T_lin2, p_after_blend1.copy(), v2, np.zeros(ndof)))
        p_before_blend2 = p_after_blend1 + v2 * T_lin2
        phases.append(('acc', tb2, p_before_blend2.copy(), v2, a2))
        p_after_blend2 = p_before_blend2 + v2 * tb2 + 0.5 * a2 * tb2 ** 2
        blend2_mid = p_before_blend2 + v2 * (tb2 / 2) + 0.5 * a2 * (tb2 / 2) ** 2
        t_qv2_mid = tb0 + T_lin1 + tb1 + T_lin2 + tb2 / 2

        phases.append(('lin', T_lin3, p_after_blend2.copy(), v3, np.zeros(ndof)))
        p_before_end = p_after_blend2 + v3 * T_lin3
        phases.append(('acc', tb3, p_before_end.copy(), v3, a3v))

        T = sum(ph[1] for ph in phases)
        deviations = {'qv1': blend1_mid - Q1, 'qv2': blend2_mid - Q2,
                      't_qv1': t_qv1_mid, 't_qv2': t_qv2_mid}
        return phases, T, deviations

    def segment_duration(self, W, v_max, a_max, j_max=None):
        return self.plan(W, v_max, a_max)[1]

    def full_profile(self, W, T, dt, v_max, a_max, j_max=None):
        phases, T_check, deviations = self.plan(W, v_max, a_max)
        if abs(T - T_check) > 1e-9:
            raise ValueError(f'T={T} does not match the construction ({T_check})')
        ts = _time_grid(T, dt)
        ndof = np.asarray(W).shape[1]
        pos = np.zeros((len(ts), ndof)); vel = np.zeros((len(ts), ndof))
        t_cursor = 0.0
        for kind, dur, p0, v0, a in phases:
            m = (ts >= t_cursor - 1e-12) & (ts <= t_cursor + dur + 1e-9)
            tt = np.clip(ts[m], t_cursor, t_cursor + dur) - t_cursor
            pos[m] = p0[None, :] + v0[None, :] * tt[:, None] + 0.5 * a[None, :] * (tt[:, None] ** 2)
            vel[m] = v0[None, :] + a[None, :] * tt[:, None]
            t_cursor += dur
        return ts, pos, vel, deviations


METHOD = LspbBlend()
