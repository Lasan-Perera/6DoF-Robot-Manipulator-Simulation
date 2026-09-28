"""Normalised shape functions s(tau) for the single-segment methods.

Every method is expressed as q(t) = q0 + dq*s(tau), tau = t/T in [0,1], with
s(0)=0, s(1)=1. The peaks |s'|max, |s''|max, |s'''|max are what the timing rule
of PROTOCOL.md section 3 needs, and they are properties of the shape alone.

Derivatives are ANALYTIC everywhere: polynomials are differentiated
symbolically, trigonometric and cam profiles piecewise in closed form. Nothing
here uses finite differences -- triple-differencing a sampled s(tau) returns
garbage (it gave |s'''| = 3.0e6 for the cubic, whose true value is 12).

The cam profiles (modified sine, modified trapezoid) are defined by their
ACCELERATION shape and integrated twice, then normalised so s(1)=1. Their
published coefficients are checked against, not copied from, the literature:
verify() asserts the boundary conditions that make a profile admissible
(s(0)=0, s(1)=1, s'(0)=s'(1)=0) rather than trusting a table.
"""
from __future__ import annotations
import numpy as np

TAU = np.linspace(0.0, 1.0, 2_000_001)   # 5e-7 spacing; peaks quoted to 4 dp


# --------------------------------------------------------------------------
# Polynomial family -- exact symbolic differentiation
# --------------------------------------------------------------------------
def _poly(coeffs):
    from numpy.polynomial import Polynomial as P
    s = P(coeffs)
    return s, s.deriv(1), s.deriv(2), s.deriv(3)


POLYNOMIALS = {
    'cubic':   [0, 0, 3, -2],
    'quintic': [0, 0, 0, 10, -15, 6],
    'septic':  [0, 0, 0, 0, 35, -84, 70, -20],
}


# --------------------------------------------------------------------------
# Trigonometric
# --------------------------------------------------------------------------
def cycloidal(t):
    w = 2 * np.pi
    return (t - np.sin(w * t) / w,
            1 - np.cos(w * t),
            w * np.sin(w * t),
            w * w * np.cos(w * t))


# --------------------------------------------------------------------------
# Cam family -- piecewise acceleration, integrated twice
#
# Both are the standard constructions: an acceleration shape that starts and
# ends at zero, is antisymmetric about tau=1/2, and has zero net area (so the
# segment ends at rest). A profile that fails either condition is not
# admissible and verify() rejects it.
# --------------------------------------------------------------------------
def _mod_sine_accel(t):
    """Neklutin modified sine: sine ramps over the outer eighths."""
    a = np.empty_like(t)
    m1, m3 = t <= 1 / 8, t >= 7 / 8
    m2 = ~(m1 | m3)
    a[m1] = np.sin(4 * np.pi * t[m1])
    a[m2] = np.cos((4 * np.pi / 3) * (t[m2] - 1 / 8))
    a[m3] = -np.cos(4 * np.pi * (t[m3] - 7 / 8))
    return a


def _mod_sine_jerk(t):
    j = np.empty_like(t)
    m1, m3 = t <= 1 / 8, t >= 7 / 8
    m2 = ~(m1 | m3)
    j[m1] = 4 * np.pi * np.cos(4 * np.pi * t[m1])
    j[m2] = -(4 * np.pi / 3) * np.sin((4 * np.pi / 3) * (t[m2] - 1 / 8))
    j[m3] = 4 * np.pi * np.sin(4 * np.pi * (t[m3] - 7 / 8))
    return j


def _mod_trap_accel(t):
    """Modified trapezoid: sine ramps into and out of constant acceleration."""
    a = np.empty_like(t)
    s1 = t <= 1 / 8
    s2 = (t > 1 / 8) & (t <= 3 / 8)
    s3 = (t > 3 / 8) & (t <= 5 / 8)
    s4 = (t > 5 / 8) & (t <= 7 / 8)
    s5 = t > 7 / 8
    a[s1] = np.sin(4 * np.pi * t[s1])
    a[s2] = 1.0
    a[s3] = np.cos(4 * np.pi * (t[s3] - 3 / 8))
    a[s4] = -1.0
    a[s5] = -np.cos(4 * np.pi * (t[s5] - 7 / 8))
    return a


def _mod_trap_jerk(t):
    j = np.zeros_like(t)
    s1 = t <= 1 / 8
    s3 = (t > 3 / 8) & (t <= 5 / 8)
    s5 = t > 7 / 8
    j[s1] = 4 * np.pi * np.cos(4 * np.pi * t[s1])
    j[s3] = -4 * np.pi * np.sin(4 * np.pi * (t[s3] - 3 / 8))
    j[s5] = 4 * np.pi * np.sin(4 * np.pi * (t[s5] - 7 / 8))
    return j


def _integrate_cam(accel_fn, jerk_fn, t):
    """Integrate a unit-amplitude acceleration shape into a normalised s(tau).

    Scaling the acceleration by k scales s by k, so normalising s(1) to 1 fixes
    the amplitude. Returns (s, s', s'', s''') with s(1)=1 exactly.
    """
    a_raw = accel_fn(t)
    v_raw = np.concatenate([[0.0], np.cumsum((a_raw[1:] + a_raw[:-1]) / 2 * np.diff(t))])
    s_raw = np.concatenate([[0.0], np.cumsum((v_raw[1:] + v_raw[:-1]) / 2 * np.diff(t))])
    k = 1.0 / s_raw[-1]
    return s_raw * k, v_raw * k, a_raw * k, jerk_fn(t) * k


CAMS = {
    'modified sine':      (_mod_sine_accel, _mod_sine_jerk),
    'modified trapezoid': (_mod_trap_accel, _mod_trap_jerk),
}

# Coefficients the cam-design literature quotes, used ONLY as a cross-check on
# our own integration. If these disagree, the implementation is wrong -- the
# table is not the authority, the boundary conditions are.
PUBLISHED = {
    'modified sine':      (1.7596, 5.5280),
    'modified trapezoid': (2.0000, 4.8881),
}


def all_shapes():
    """{name: (s, s', s'', s''')} sampled on TAU, analytic derivatives."""
    out = {}
    for name, c in POLYNOMIALS.items():
        s, d1, d2, d3 = _poly(c)
        out[name] = (s(TAU), d1(TAU), d2(TAU), d3(TAU))
    out['cycloidal'] = cycloidal(TAU)
    for name, (af, jf) in CAMS.items():
        out[name] = _integrate_cam(af, jf, TAU)
    return out


def verify(shapes, tol=1e-4):
    """Assert every shape is admissible. Returns a list of failures."""
    bad = []
    for name, (s, d1, d2, d3) in shapes.items():
        if abs(s[0]) > tol:            bad.append(f'{name}: s(0)={s[0]:.2e}, want 0')
        if abs(s[-1] - 1) > tol:       bad.append(f'{name}: s(1)={s[-1]:.6f}, want 1')
        if abs(d1[0]) > tol:           bad.append(f"{name}: s'(0)={d1[0]:.2e}, want 0")
        if abs(d1[-1]) > tol:          bad.append(f"{name}: s'(1)={d1[-1]:.2e}, want 0")
        if abs(np.trapezoid(d2, TAU)) > tol:
            bad.append(f'{name}: acceleration has nonzero net area, segment does not end at rest')
    return bad


def peaks(shapes):
    return {n: (np.max(np.abs(d1)), np.max(np.abs(d2)), np.max(np.abs(d3)))
            for n, (s, d1, d2, d3) in shapes.items()}


if __name__ == '__main__':
    sh = all_shapes()
    bad = verify(sh)
    print('admissibility:', 'all pass' if not bad else 'FAILURES')
    for b in bad:
        print('  !', b)
    print(f"\n{'method':20} {chr(124)+chr(115)+chr(39)+chr(124)+'max':>10} "
          f"{chr(124)+chr(115)+chr(39)*2+chr(124)+'max':>10} {chr(124)+chr(115)+chr(39)*3+chr(124)+'max':>11}   cross-check")
    closed = {'cubic': (1.5, 6.0, 12.0), 'quintic': (15 / 8, 10 / 3 ** .5, 60.0),
              'septic': (35 / 16, None, 52.5),
              'cycloidal': (2.0, 2 * np.pi, 4 * np.pi ** 2)}
    for n, (p1, p2, p3) in peaks(sh).items():
        note = ''
        if n in closed:
            c = closed[n]
            errs = [abs(p - c[i]) for i, p in enumerate((p1, p2, p3)) if c[i] is not None]
            note = f'closed form, max err {max(errs):.2e}'
        elif n in PUBLISHED:
            q1, q2 = PUBLISHED[n]
            note = f'published {q1:.4f}/{q2:.4f}, err {abs(p1-q1):.4f}/{abs(p2-q2):.4f}'
        print(f'{n:20} {p1:10.4f} {p2:10.4f} {p3:11.4f}   {note}')
