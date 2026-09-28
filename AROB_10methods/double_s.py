"""Double-S (jerk-limited trapezoidal) profile, rest to rest.

The seven-phase construction of Biagiotti and Melchiorri: the acceleration
steps of an LSPB profile are themselves ramped at a bounded rate, so jerk is
piecewise constant instead of impulsive. Restricted here to v0 = v1 = 0 and
a0 = a1 = 0, which is all both experiments need.

Unlike the fixed shape functions, the shape depends on dq, v_max, a_max and
j_max, because which of the three limits binds changes the phase structure.
Three regimes are handled, in the order the durations are tested:

  1. v_max reached (Tv > 0), with or without a_max reached;
  2. v_max not reached but a_max reached -- Ta from the quadratic below;
  3. neither reached -- four jerk phases, Tj = cbrt(h / 2j).

The profile is evaluated by integrating the piecewise-constant jerk in closed
form, never numerically, so the derivatives handed to the paper's figures are
exact.
"""
from __future__ import annotations
import numpy as np


def phase_durations(h: float, v_max: float, a_max: float, j_max: float):
    """Return (Tj1, Ta, Tv, Tj2, Td, a_lim) for a rest-to-rest move of h > 0."""
    if h <= 0:
        raise ValueError('h must be positive; handle the sign outside')

    # --- regime 1: assume v_max is reached ---------------------------------
    if v_max * j_max >= a_max ** 2:
        Tj = a_max / j_max
        Ta = Tj + v_max / a_max
    else:                                   # a_max never reached in the ramp
        Tj = np.sqrt(v_max / j_max)
        Ta = 2 * Tj
    Tv = h / v_max - Ta                     # v0 = v1 = 0 makes Ta/2 + Td/2 = Ta
    if Tv > 0:
        return Tj, Ta, Tv, Tj, Ta, j_max * Tj

    # --- regime 2: v_max not reached, assume a_max is ----------------------
    Tj = a_max / j_max
    disc = a_max ** 4 / j_max ** 2 + 4 * a_max * h
    Ta = (a_max ** 2 / j_max + np.sqrt(disc)) / (2 * a_max)
    if Ta >= 2 * Tj:
        return Tj, Ta, 0.0, Tj, Ta, a_max

    # --- regime 3: neither limit reached -----------------------------------
    Tj = (h / (2 * j_max)) ** (1 / 3)
    return Tj, 2 * Tj, 0.0, Tj, 2 * Tj, j_max * Tj


def _jerk_phases(h, v_max, a_max, j_max):
    """(duration, jerk) for each of the seven phases, zero-length ones kept."""
    Tj1, Ta, Tv, Tj2, Td, _ = phase_durations(h, v_max, a_max, j_max)
    return [(Tj1, +j_max), (Ta - 2 * Tj1, 0.0), (Tj1, -j_max),
            (Tv, 0.0),
            (Tj2, -j_max), (Td - 2 * Tj2, 0.0), (Tj2, +j_max)]


def evaluate(t, h, v_max, a_max, j_max):
    """Position, velocity, acceleration and jerk at times t, exactly.

    Integrates the piecewise-constant jerk phase by phase. t outside [0, T] is
    clamped to the endpoints.
    """
    phases = _jerk_phases(h, v_max, a_max, j_max)
    T = sum(d for d, _ in phases)
    t = np.clip(np.asarray(t, dtype=float), 0.0, T)
    q = np.zeros_like(t); v = np.zeros_like(t); a = np.zeros_like(t); j = np.zeros_like(t)

    t0 = 0.0
    q0 = v0 = a0 = 0.0
    for dur, jk in phases:
        if dur <= 0:
            continue
        m = (t >= t0) & (t <= t0 + dur + 1e-15)
        if m.any():
            dt = t[m] - t0
            j[m] = jk
            a[m] = a0 + jk * dt
            v[m] = v0 + a0 * dt + 0.5 * jk * dt ** 2
            q[m] = q0 + v0 * dt + 0.5 * a0 * dt ** 2 + jk * dt ** 3 / 6
        # advance the phase's own end state
        q0 = q0 + v0 * dur + 0.5 * a0 * dur ** 2 + jk * dur ** 3 / 6
        v0 = v0 + a0 * dur + 0.5 * jk * dur ** 2
        a0 = a0 + jk * dur
        t0 += dur
    return q, v, a, j, T


def verify(h=0.47319, v_max=0.5, a_max=1.0, j_max=7.7667, tol=1e-9):
    """Assert the profile is admissible. Returns a list of failures."""
    t = np.linspace(0, 1, 200001)
    Tj1, Ta, Tv, Tj2, Td, a_lim = phase_durations(h, v_max, a_max, j_max)
    T = Ta + Tv + Td
    q, v, a, j, T2 = evaluate(t * T, h, v_max, a_max, j_max)
    bad = []
    if abs(T2 - T) > tol:                bad.append(f'phase sum {T2} != {T}')
    if abs(q[0]) > tol:                  bad.append(f'q(0)={q[0]:.2e}, want 0')
    if abs(q[-1] - h) > 1e-6:            bad.append(f'q(T)={q[-1]:.9f}, want {h}')
    if abs(v[0]) > tol or abs(v[-1]) > 1e-6:
        bad.append(f'v endpoints {v[0]:.2e}/{v[-1]:.2e}, want 0')
    if abs(a[0]) > tol or abs(a[-1]) > 1e-6:
        bad.append(f'a endpoints {a[0]:.2e}/{a[-1]:.2e}, want 0')
    for name, arr, lim in (('v', v, v_max), ('a', a, a_max), ('j', j, j_max)):
        if np.max(np.abs(arr)) > lim * (1 + 1e-6):
            bad.append(f'{name} peak {np.max(np.abs(arr)):.6f} exceeds limit {lim}')
    return bad, (T, np.max(np.abs(v)), np.max(np.abs(a)), np.max(np.abs(j)))


if __name__ == '__main__':
    for label, h, v, a, jm in (('Exp 1 segment', 0.47319, 0.5, 1.0, 7.7667),
                               ('short move (v_max not reached)', 0.02, 0.5, 1.0, 7.7667),
                               ('very short (no limit reached)', 0.0005, 0.5, 1.0, 7.7667)):
        bad, (T, pv, pa, pj) = verify(h, v, a, jm)
        Tj1, Ta, Tv, Tj2, Td, a_lim = phase_durations(h, v, a, jm)
        # a_lim is the acceleration the profile actually reaches, so it, not
        # the phase durations, says which regime was taken.
        regime = ('v_max reached' if Tv > 0
                  else 'a_max reached' if a_lim >= a * (1 - 1e-9)
                  else 'neither limit reached')
        print(f'{label:32} T={T:7.4f}s  regime: {regime}')
        print(f'{"":32} peaks v={pv:.4f} a={pa:.4f} j={pj:.4f}   '
              f'{"all checks pass" if not bad else "FAIL"}')
        for b in bad:
            print('    !', b)
