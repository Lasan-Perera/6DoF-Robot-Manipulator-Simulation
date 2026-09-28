"""Double-S (method 8) for six joints.

Builds on double_s.py, which is verified for ONE joint and h > 0. Here the
pacing joint (largest |dq|) is profiled with double_s.evaluate, normalised to
    s(tau) = q_pace(tau*T) / h ,
and every joint uses that same s(tau), exactly as in sec. 4.1 of the briefing:
    pos_j = q0_j + dq_j * q_pace/h ,  vel_j = dq_j * v_pace/h ,
    acc_j = dq_j * a_pace/h ,         jerk_j = dq_j * j_pace/h .
All six joints start and stop together and the joint-space path is straight.

Unlike the fixed shapes, the shape depends on the limits (which of v_max, a_max,
j_max binds changes the phase structure), so the limits are explicit arguments
everywhere. There are no hidden defaults.

Interface, matching shape_trajectories where it can:
    segment_duration(dq, v_max, a_max, j_max)            -> T
    describe(dq, v_max, a_max, j_max)                    -> dict (T, regime, binding, phases)
    full_profile(q0, qf, T, dt, v_max, a_max, j_max)     -> (ts, pos, vel, acc)
    commanded_peaks(dq, v_max, a_max, j_max)             -> (peak_vel, peak_acc, peak_jerk)
    isj(dq, v_max, a_max, j_max)                         -> integral of jerk^2 dt
T is not a free parameter: full_profile requires the T that segment_duration
returns and raises if it differs.
"""
from __future__ import annotations
import numpy as np
import double_s as ds
from shape_trajectories import _time_grid


class DoubleS:
    name = 'doubles'
    jerk_bounded = True
    endpoint_jerk_impulse = False

    @staticmethod
    def describe(dq, v_max, a_max, j_max):
        h = abs(float(dq))
        if h == 0.0:
            return dict(T=0.0, regime=0, binding='none', Tj1=0.0, Ta=0.0, Tv=0.0,
                        Tj2=0.0, Td=0.0, a_lim=0.0)
        Tj1, Ta, Tv, Tj2, Td, a_lim = ds.phase_durations(h, v_max, a_max, j_max)
        regime = 1 if Tv > 0 else (2 if a_lim >= a_max * (1 - 1e-9) else 3)
        binding = {1: 'vel', 2: 'acc', 3: 'jerk'}[regime]
        return dict(T=float(Ta + Tv + Td), regime=regime, binding=binding,
                    Tj1=float(Tj1), Ta=float(Ta), Tv=float(Tv), Tj2=float(Tj2),
                    Td=float(Td), a_lim=float(a_lim))

    def segment_duration(self, dq, v_max, a_max, j_max):
        return self.describe(dq, v_max, a_max, j_max)['T']

    def full_profile(self, q0, qf, T, dt, v_max, a_max, j_max, with_jerk=False):
        q0 = np.asarray(q0, dtype=float); qf = np.asarray(qf, dtype=float)
        dqv = qf - q0
        pace = int(np.argmax(np.abs(dqv))); h = abs(float(dqv[pace]))
        if h == 0.0:
            z = np.zeros((1, len(q0)))
            out = (np.zeros(1), q0[None, :].copy(), z, z.copy())
            return out + (z.copy(),) if with_jerk else out
        T_ds = self.segment_duration(h, v_max, a_max, j_max)
        if abs(T - T_ds) > 1e-9:
            raise ValueError(f'T={T} is not the double-S duration {T_ds}; '
                             'pass the value segment_duration returns')
        ts = _time_grid(T_ds, dt)
        q, v, a, j, _T = ds.evaluate(ts, h, v_max, a_max, j_max)
        d = dqv[None, :] / h
        out = (ts, q0[None, :] + d * q[:, None], d * v[:, None], d * a[:, None])
        return out + (d * j[:, None],) if with_jerk else out

    def commanded_peaks(self, dq, v_max, a_max, j_max):
        """Analytic pacing-joint peaks. Velocity after the acceleration phase is
        a_lim*(Ta - Tj1): trapezoid of the acceleration ramp, plateau, ramp."""
        d = self.describe(dq, v_max, a_max, j_max)
        return d['a_lim'] * (d['Ta'] - d['Tj1']), d['a_lim'], float(j_max)

    def isj(self, dq, v_max, a_max, j_max):
        """Jerk is +-j_max for 2*Tj1 + 2*Tj2 seconds and zero otherwise."""
        d = self.describe(dq, v_max, a_max, j_max)
        return float(j_max) ** 2 * (2 * d['Tj1'] + 2 * d['Tj2'])


METHOD = DoubleS()
