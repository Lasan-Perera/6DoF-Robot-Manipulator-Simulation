"""Phase 6a acceptance checks: general multi-joint TOPP-RA floor
(toppra_floor.py), needed for the "% opt." column in Tables 4 and 5.

Run from AROB_10methods/ with toppra_floor.py, lspb_full_profile.py,
shape_trajectories.py, bspline_trajectory.py, lspb.json and
data/verification/via_point.json present. Requires: pip install toppra
--break-system-packages
"""
import json, os, sys
import numpy as np
import toppra_floor as tf
import lspb_full_profile as lfp
import shape_trajectories as st
import bspline_trajectory as bt

V_MAX, A_MAX, J_MAX = 0.5, 1.0, 7.766715171374766
fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

with open('lspb.json') as f:
    d = json.load(f)
q0, qf = np.array(d['q0']), np.array(d['qf'])

print('[1] matches the already-verified LSPB value (crosscheck_toppra.py got 1.36e-6 s single-joint)')
ts, pos, vel, acc, tb, tf_ = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=0.01)
floor = tf.topp_floor(pos, V_MAX, A_MAX)
check(abs(floor - tf_) < 1e-5, f"floor={floor:.6f}  LSPB T={tf_:.6f}  diff={abs(floor-tf_):.2e}")

print('\n[2] stable under sample-density refinement (must NOT drift)')
floors = []
for dt in (0.02, 0.01, 0.005, 0.002):
    _, p, *_ = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=dt)
    floors.append(tf.topp_floor(p, V_MAX, A_MAX))
spread = max(floors) - min(floors)
check(spread < 1e-5, f"floors at 4 sample densities: {[round(x,6) for x in floors]}, spread {spread:.2e}")

print('\n[3] floor depends only on path GEOMETRY, not on which method generated the samples')
delta = qf - q0; pace = int(np.argmax(np.abs(delta)))
T_c = st.METHODS['cubic'].segment_duration(delta[pace], V_MAX, A_MAX, J_MAX)
_, pos_c, *_ = st.METHODS['cubic'].full_profile(q0, qf, T_c, 0.01)
floor_c = tf.topp_floor(pos_c, V_MAX, A_MAX)
check(abs(floor_c - floor) < 1e-9, f"floor from cubic's own samples of the SAME straight line: {floor_c:.9f} vs {floor:.9f}")

print("\n[4] curved path (B-spline): floor must be <= the methods own actual time")
for path in ('data/verification/via_point.json', 'via_point.json'):
    if os.path.exists(path):
        vp = json.load(open(path)); break
W = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])
ts_b, pos_b, vel_b, acc_b, T_b, pv_b, pa_b = bt.generate_via_point_trajectory(W, V_MAX, A_MAX, dt=0.01)
floor_b = tf.topp_floor(pos_b, V_MAX, A_MAX)
check(floor_b <= T_b.sum(), f"B-spline T={T_b.sum():.4f}  floor={floor_b:.4f}  (% opt would be {100*floor_b/T_b.sum():.1f}%)")

print('\n' + ('ALL PHASE 6a CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
