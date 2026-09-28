"""Analytic single-segment generators for the fixed-shape methods (Group A).

Methods: cubic, quintic, septic, cycloidal, modsine, modtrap.
(cubic and quintic are included so the machinery can be validated against the
durations already established; methods 4-7 are the new ones.)

Every method is q(t) = q0 + dq*s(tau), tau = t/T. Here s, s', s'', s''' are
CALLABLE closed forms, so any t can be evaluated exactly. Nothing is
interpolated from a grid and nothing is finite-differenced. The cam profiles
are integrated piecewise in closed form from their acceleration shape.

Timing rule (PROTOCOL.md sec. 3): the shortest T satisfying
    T >= max( |s'|max*|dq|/v_max , sqrt(|s''|max*|dq|/a_max) ,
              (|s'''|max*|dq|/j_max)^(1/3) )
where the jerk term is applied ONLY if the jerk is bounded (not for the cubic).
Joints are synchronised: the pacing joint (largest |dq|) fixes T, every joint
uses the same s(tau).

Interface per method (METHODS[name]):
    .duration_terms(dq, v_max, a_max, j_max) -> (T, T_vel, T_acc, T_jerk, binding)
    .segment_duration(dq, v_max, a_max, j_max) -> T
    .full_profile(q0, qf, T, dt=0.01) -> (ts, pos, vel, acc)
    .commanded_peaks(dq, T) -> (peak_vel, peak_acc, peak_jerk)   [pacing joint]
"""
from __future__ import annotations
import numpy as np
from numpy.polynomial import Polynomial as P
from numpy.polynomial.legendre import leggauss

F = 4.0 * np.pi


# ---------------------------------------------------------------- polynomials
def _poly_shape(coeffs):
    s = P(coeffs)
    d = [s, s.deriv(1), s.deriv(2), s.deriv(3)]

    def shape(tau):
        tau = np.asarray(tau, dtype=float)
        return tuple(p(tau) for p in d)

    def peak(k):                       # exact: endpoints + roots of next derivative
        pk, pn = d[k], (d[k].deriv(1) if k == 3 else d[k + 1])
        cand = [0.0, 1.0]
        pn = pn.trim()
        if len(pn.coef) > 1:
            r = pn.roots()
            cand += [x.real for x in r if abs(x.imag) < 1e-12 and 0 < x.real < 1]
        return float(max(abs(pk(c)) for c in cand))

    def isj():                         # exact integral of s'''^2 over [0,1]
        return float((d[3] * d[3]).integ(1)(1.0) - (d[3] * d[3]).integ(1)(0.0))

    return shape, (peak(1), peak(2), peak(3)), isj()


# ---------------------------------------------------------------- cycloidal
def _cycloidal(tau):
    tau = np.asarray(tau, dtype=float)
    w = 2 * np.pi
    return (tau - np.sin(w * tau) / w, 1 - np.cos(w * tau),
            w * np.sin(w * tau), w * w * np.cos(w * tau))


# ---------------------------------------------------------------- cam family
# A piece is (tau0, tau1, a(u), j(u), V(u), S(u)) with u = tau - tau0 and V, S the
# exact antiderivatives of a (zero initial conditions). Carry constants are
# propagated analytically from piece to piece.
def _sine_up():
    return (lambda u: np.sin(F * u), lambda u: F * np.cos(F * u),
            lambda u: (1 - np.cos(F * u)) / F, lambda u: u / F - np.sin(F * u) / F ** 2)


def _sine_down():
    return (lambda u: -np.cos(F * u), lambda u: F * np.sin(F * u),
            lambda u: -np.sin(F * u) / F, lambda u: (np.cos(F * u) - 1) / F ** 2)


def _const(sign):
    return (lambda u: sign + 0 * u, lambda u: 0 * u,
            lambda u: sign * u, lambda u: sign * u ** 2 / 2)


def _cosine_ramp(w):                       # a = cos(w u)
    return (lambda u: np.cos(w * u), lambda u: -w * np.sin(w * u),
            lambda u: np.sin(w * u) / w, lambda u: (1 - np.cos(w * u)) / w ** 2)


def _cam(bps, kinds):
    bps = np.asarray(bps, dtype=float)
    pieces = [(bps[i], bps[i + 1]) + kinds[i] for i in range(len(kinds))]
    carry, v0, s0 = [], 0.0, 0.0
    for (t0, t1, a, j, V, S) in pieces:
        carry.append((v0, s0))
        L = t1 - t0
        s0 = s0 + v0 * L + S(L)
        v0 = v0 + V(L)
    k = 1.0 / s0                            # s_raw(1) = s0 -> normalise s(1)=1

    def shape(tau):
        tau = np.clip(np.asarray(tau, dtype=float), 0.0, 1.0)
        idx = np.searchsorted(bps[1:-1], tau, side='right')
        s = np.empty_like(tau); v = np.empty_like(tau)
        a = np.empty_like(tau); j = np.empty_like(tau)
        for i, (t0, t1, af, jf, Vf, Sf) in enumerate(pieces):
            m = idx == i
            if not m.any():
                continue
            u = tau[m] - t0
            cv, cs = carry[i]
            s[m] = cs + cv * u + Sf(u)
            v[m] = cv + Vf(u)
            a[m] = af(u)
            j[m] = jf(u)
        return k * s, k * v, k * a, k * j

    def peak_of(fn_index):                   # dense grid, then zoom on the argmax
        lo, hi = 0.0, 1.0
        for _ in range(6):
            g = np.linspace(lo, hi, 200001)
            y = np.abs(shape(g)[fn_index])
            i = int(np.argmax(y)); h = g[1] - g[0]
            lo, hi = max(0.0, g[i] - 2 * h), min(1.0, g[i] + 2 * h)
        return float(y[i])

    def isj():                               # exact: Gauss-Legendre on each smooth piece
        x, w = leggauss(48); tot = 0.0
        for (t0, t1, *_r) in pieces:
            tau = 0.5 * (t1 - t0) * x + 0.5 * (t1 + t0)
            tot += 0.5 * (t1 - t0) * np.sum(w * shape(tau)[3] ** 2)
        return float(tot)

    return shape, (peak_of(1), peak_of(2), peak_of(3)), isj(), k


# ---------------------------------------------------------------- the methods
class Method:
    def __init__(self, name, shape, peaks, isj, jerk_bounded, endpoint_jerk_impulse):
        self.name, self.shape, self.peaks = name, shape, peaks
        self.isj_factor = isj
        self.jerk_bounded = jerk_bounded
        self.endpoint_jerk_impulse = endpoint_jerk_impulse

    def duration_terms(self, dq, v_max, a_max, j_max):
        dq = abs(float(dq))
        if dq == 0.0:
            return 0.0, 0.0, 0.0, 0.0, 'none'
        p1, p2, p3 = self.peaks
        Tv, Ta = p1 * dq / v_max, np.sqrt(p2 * dq / a_max)
        Tj = (p3 * dq / j_max) ** (1 / 3) if self.jerk_bounded else 0.0
        T = max(Tv, Ta, Tj)
        binding = ('vel', 'acc', 'jerk')[int(np.argmax([Tv, Ta, Tj]))]
        return float(T), float(Tv), float(Ta), float(Tj), binding

    def segment_duration(self, dq, v_max, a_max, j_max):
        return self.duration_terms(dq, v_max, a_max, j_max)[0]

    def full_profile(self, q0, qf, T, dt=0.01):
        q0 = np.asarray(q0, dtype=float); qf = np.asarray(qf, dtype=float)
        ts = _time_grid(T, dt)
        s, s1, s2, _ = self.shape(ts / T)
        d = (qf - q0)[None, :]
        return (ts, q0[None, :] + d * s[:, None],
                d * s1[:, None] / T, d * s2[:, None] / T ** 2)

    def commanded_peaks(self, dq, T):
        p1, p2, p3 = self.peaks; dq = abs(float(dq))
        return p1 * dq / T, p2 * dq / T ** 2, p3 * dq / T ** 3

    def isj(self, dq, T):                    # integral of jerk^2 dt, pacing joint
        return float(dq) ** 2 * self.isj_factor / T ** 5


def _time_grid(T, dt):
    n = int(np.floor(T / dt + 1e-9))
    ts = np.arange(n + 1) * dt
    if T - ts[-1] > 1e-9:
        ts = np.append(ts, T)
    else:
        ts[-1] = T
    return ts


def _build():
    out = {}
    for name, c, endimp in (('cubic', [0, 0, 3, -2], True),
                            ('quintic', [0, 0, 0, 10, -15, 6], False),
                            ('septic', [0, 0, 0, 0, 35, -84, 70, -20], False)):
        sh, pk, isj = _poly_shape(c)
        out[name] = Method(name, sh, pk, isj, name != 'cubic', endimp)
    out['cycloidal'] = Method('cycloidal', _cycloidal, (2.0, 2 * np.pi, 4 * np.pi ** 2),
                              8 * np.pi ** 4, True, False)
    w = F / 3
    sh, pk, isj, k = _cam([0, 1 / 8, 7 / 8, 1], [_sine_up(), _cosine_ramp(w), _sine_down()])
    out['modsine'] = Method('modsine', sh, pk, isj, True, False); out['modsine'].k = k
    sh, pk, isj, k = _cam([0, 1 / 8, 3 / 8, 5 / 8, 7 / 8, 1],
                          [_sine_up(), _const(+1.0), _cosine_ramp(F), _const(-1.0), _sine_down()])
    out['modtrap'] = Method('modtrap', sh, pk, isj, True, False); out['modtrap'].k = k
    return out


METHODS = _build()
J_MAX_DEG = 445.0                                   # PROTOCOL.md sec. 2
J_MAX = float(np.deg2rad(J_MAX_DEG))                # rad/s^3
