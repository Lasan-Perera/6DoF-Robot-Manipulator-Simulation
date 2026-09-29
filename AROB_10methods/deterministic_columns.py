"""Deterministic columns for Tables 4 and 5: T, % opt., jmax, integral(j^2)dt,
and (Table 5 only) Stops -- for all 18 method x experiment combinations.
Nothing here needs Gazebo; every value is computed from the already-verified
generators (Phases 1-4) and the TOPP-RA floor (Phase 6a).

Experiment 1 -- 8 point-to-point methods, single must-stop segment
(lspb.json). Experiment 2 -- all 10 methods on the via-point stretch
(data/verification/via_point.json): the 8 point-to-point methods run
STITCHED (Phase 4, forced stop at every waypoint, Stops=4), plus bspline and
lspbblend running natively through the via-points (Stops=2 for bspline;
lspbblend does not fully stop at the interior waypoints either, but its
construction is corner-cutting rather than a single continuous pass, see
PROTOCOL.md's Phase 3 changelog -- Stops recorded as 2 for both non-stitched
methods, matching decision 4's "endpoints only" once corner-cutting was
adopted).
"""
import json, os
import numpy as np

import lspb_full_profile as lfp
import shape_trajectories as st
import double_s_trajectory as ds_mod
import stitched_trajectory as stt
import bspline_trajectory as bt
import lspb_blend_trajectory as lb
import toppra_floor as tf

V_MAX, A_MAX, J_MAX = 0.5, 1.0, 7.766715171374766
POINT_TO_POINT = ('lspb', 'cubic', 'quintic', 'septic', 'cycloidal', 'modsine', 'modtrap', 'doubles')
DISPLAY_NAME = {'lspb': 'Trapezoidal', 'cubic': 'Cubic', 'quintic': 'Quintic',
                'septic': 'Septic', 'cycloidal': 'Cycloidal', 'modsine': 'Mod. sine',
                'modtrap': 'Mod. trap.', 'doubles': 'Double-S',
                'bspline': 'B-spline', 'lspbblend': 'LSPB blends'}


def _isj_point_to_point(method, dq, T):
    """integral j^2 dt for one point-to-point method's pacing joint. LSPB's
    jerk is impulsive (undefined integral) -- return None, not a number."""
    if method == 'lspb':
        return None
    if method == 'doubles':
        return ds_mod.METHOD.isj(dq, V_MAX, A_MAX, J_MAX)
    return st.METHODS[method].isj(dq, T)


def _max_jerk_point_to_point(method, dq, T):
    if method == 'lspb':
        return None
    if method == 'doubles':
        _, _, pj = ds_mod.METHOD.commanded_peaks(dq, V_MAX, A_MAX, J_MAX)
        return pj
    _, _, pj = st.METHODS[method].commanded_peaks(dq, T)
    return pj


def experiment1_table(lspb_json_path='lspb.json'):
    with open(lspb_json_path) as f:
        d = json.load(f)
    q0, qf = np.array(d['q0']), np.array(d['qf'])
    delta = qf - q0
    pace = int(np.argmax(np.abs(delta)))
    dq = abs(delta[pace])

    rows = []
    for m in POINT_TO_POINT:
        if m == 'lspb':
            T = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=0.01)[5]
            ts, pos, vel, acc, tb, tf_ = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=0.01)
        elif m == 'doubles':
            T = ds_mod.METHOD.segment_duration(dq, V_MAX, A_MAX, J_MAX)
            ts, pos, vel, acc = ds_mod.METHOD.full_profile(q0, qf, T, 0.01, V_MAX, A_MAX, J_MAX)
        else:
            T = st.METHODS[m].segment_duration(dq, V_MAX, A_MAX, J_MAX)
            ts, pos, vel, acc = st.METHODS[m].full_profile(q0, qf, T, 0.01)

        floor = tf.topp_floor(pos, V_MAX, A_MAX)
        pct_opt = 100.0 * floor / T
        jmax = _max_jerk_point_to_point(m, dq, T)
        isj = _isj_point_to_point(m, dq, T)
        rows.append(dict(method=m, display=DISPLAY_NAME[m], T=T, pct_opt=pct_opt,
                          jmax=jmax, isj=isj))
    return rows


def experiment2_table(via_point_json_path=None):
    path = via_point_json_path
    if path is None:
        for p in ('data/verification/via_point.json', 'via_point.json'):
            if os.path.exists(p):
                path = p; break
    with open(path) as f:
        vp = json.load(f)
    W = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])

    rows = []
    for m in POINT_TO_POINT:
        ts, pos, vel, acc, T_seg, peak_v, peak_a, n_stops = stt.generate_stitched_trajectory(
            m, W, V_MAX, A_MAX, J_MAX, dt=0.01)
        T = float(T_seg.sum())
        floor = tf.topp_floor(pos, V_MAX, A_MAX)
        pct_opt = 100.0 * floor / T
        # jmax / isj: max over the 3 segments (each an independent must-stop move)
        deltas = np.diff(W, axis=0)
        jmax_vals, isj_vals = [], []
        for i in range(3):
            pacing_idx = int(np.argmax(np.abs(deltas[i])))
            dq_i = abs(deltas[i][pacing_idx])
            j = _max_jerk_point_to_point(m, dq_i, T_seg[i])
            isj_i = _isj_point_to_point(m, dq_i, T_seg[i])
            if j is not None: jmax_vals.append(j)
            if isj_i is not None: isj_vals.append(isj_i)
        jmax = max(jmax_vals) if jmax_vals else None
        isj = max(isj_vals) if isj_vals else None
        rows.append(dict(method=m, display=DISPLAY_NAME[m], T=T, pct_opt=pct_opt,
                          stops=n_stops, jmax=jmax, isj=isj))

    # B-spline
    ts_b, pos_b, vel_b, acc_b, T_b, pv_b, pa_b = bt.generate_via_point_trajectory(W, V_MAX, A_MAX, dt=0.01)
    T = float(T_b.sum())
    floor = tf.topp_floor(pos_b, V_MAX, A_MAX)
    jmax_b = 0.0
    for j in range(6):
        coeffs, _ = bt.solve_clamped_spline_single_joint(W[:, j], T_b)
        for (a0, a1, a2, a3) in coeffs:
            jmax_b = max(jmax_b, abs(6 * a3))
    rows.append(dict(method='bspline', display=DISPLAY_NAME['bspline'], T=T,
                      pct_opt=100.0 * floor / T, stops=2, jmax=jmax_b, isj=None))

    # LSPB blends
    phases, T_lb, dev = lb.METHOD.plan(W, V_MAX, A_MAX)
    ts_lb, pos_lb, vel_lb, dev2 = lb.METHOD.full_profile(W, T_lb, 0.01, V_MAX, A_MAX)
    floor_lb = tf.topp_floor(pos_lb, V_MAX, A_MAX)
    rows.append(dict(method='lspbblend', display=DISPLAY_NAME['lspbblend'], T=T_lb,
                      pct_opt=100.0 * floor_lb / T_lb, stops=2, jmax=None, isj=None,
                      deviation=dev))
    return rows
