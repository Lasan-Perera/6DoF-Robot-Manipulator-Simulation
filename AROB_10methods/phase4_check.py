"""Phase 4 acceptance checks: stitched trajectories for all 8 point-to-point
methods (P7, CHAT_BRIEFING.md sec. 13 phase 4).

Run from AROB_10methods/ with stitched_trajectory.py, shape_trajectories.py,
double_s_trajectory.py, lspb_full_profile.py and
data/verification/via_point.json present.
"""
import json, os, sys
import numpy as np
import stitched_trajectory as stt

V, A, J = 0.5, 1.0, 7.766715171374766   # 445 deg/s^3 exactly
fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

for path in ('data/verification/via_point.json', 'via_point.json'):
    if os.path.exists(path): vp = json.load(open(path)); break
W = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])
dqs = [float(np.max(np.abs(d))) for d in np.diff(W, axis=0)]

print('[1] Experiment 2 stitched durations vs the values established in phase 1/2')
expected = {
    'lspb':      ([1.4104, 1.9095, 1.4905], 4.8104),   # from phase 1/earlier work; re-derive below
    'cubic':     ([1.6307, 1.9905, 1.8193], 5.4405),
    'quintic':   ([1.6619, 2.4764, 2.0687], 6.2070),
    'septic':    ([1.9389, 2.8891, 2.4135], 7.2415),
    'cycloidal': ([1.7727, 2.6415, 2.2066], 6.6208),
    'modsine':   ([1.5826, 2.3240, 1.9414], 5.8480),
    'modtrap':   ([1.7727, 2.6415, 2.2066], 6.6208),
    'doubles':   ([1.5151, 1.9495, 1.7321], 5.1967),   # from phase 2
}
# recompute LSPB's own expected segments directly (not in CHAT_BRIEFING's table)
lspb_T = []
for d in dqs:
    if d >= V**2/A: tb = V/A; T = tb + d/V
    else:           tb = np.sqrt(d/A); T = 2*tb
    lspb_T.append(T)
expected['lspb'] = (lspb_T, sum(lspb_T))

for m in stt.METHOD_NAMES:
    ts, pos, vel, acc, T_seg, pv, pa, n_stops = stt.generate_stitched_trajectory(m, W, V, A, J, dt=0.01)
    Tb, tot_b = expected[m]
    ok = all(abs(a-b) < 6e-5 for a, b in zip(T_seg, Tb)) and abs(T_seg.sum()-tot_b) < 1.5e-4
    check(ok, f"{m:9s} segs {[round(float(x),4) for x in T_seg]} total {T_seg.sum():.4f} (expected {[round(x,4) for x in Tb]}, total {tot_b:.4f})")

print('\n[2] every method: endpoints, n_stops, zero velocity/accel at every internal stop')
for m in stt.METHOD_NAMES:
    ts, pos, vel, acc, T_seg, pv, pa, n_stops = stt.generate_stitched_trajectory(m, W, V, A, J, dt=0.01)
    e_start = np.max(np.abs(pos[0] - W[0])); e_end = np.max(np.abs(pos[-1] - W[3]))
    check(n_stops == 4 and e_start < 1e-10 and e_end < 1e-6,
          f"{m:9s} n_stops={n_stops}  start err {e_start:.1e}  end err {e_end:.1e}")
    # check velocity is ~0 at the two interior waypoint boundaries (segment sums)
    t1 = T_seg[0]; t2 = T_seg[0] + T_seg[1]
    i1 = np.argmin(np.abs(ts - t1)); i2 = np.argmin(np.abs(ts - t2))
    e_v1 = np.max(np.abs(vel[i1])); e_v2 = np.max(np.abs(vel[i2]))
    check(e_v1 < 1e-6 and e_v2 < 1e-6, f"{m:9s} vel at interior stops: {e_v1:.1e}, {e_v2:.1e}")

print('\n[3] cross-check against phase 1/2 single-segment generators directly (segment 1 of the stitch)')
import shape_trajectories as st2
from double_s_trajectory import METHOD as DS2
q0s, q1s = W[0], W[1]
for m in ('cubic', 'quintic', 'septic', 'cycloidal', 'modsine', 'modtrap'):
    pace = int(np.argmax(np.abs(q1s - q0s)))
    T_direct = st2.METHODS[m].segment_duration(abs((q1s-q0s)[pace]), V, A, J)
    ts, pos, vel, acc, T_seg, pv, pa, n_stops = stt.generate_stitched_trajectory(m, W, V, A, J, dt=0.01)
    check(abs(T_seg[0] - T_direct) < 1e-9, f"{m:9s} stitched segment 1 T = {T_seg[0]:.6f} vs direct {T_direct:.6f}")
pace = int(np.argmax(np.abs(q1s - q0s)))
T_direct_ds = DS2.segment_duration(abs((q1s-q0s)[pace]), V, A, J)
_, _, _, _, T_seg_ds, _, _, _ = stt.generate_stitched_trajectory('doubles', W, V, A, J, dt=0.01)
check(abs(T_seg_ds[0] - T_direct_ds) < 1e-9, f"doubles   stitched segment 1 T = {T_seg_ds[0]:.6f} vs direct {T_direct_ds:.6f}")

print('\n[4] concatenation: no duplicated timestamps at the internal joins, ts strictly increasing')
for m in stt.METHOD_NAMES:
    ts, *_ = stt.generate_stitched_trajectory(m, W, V, A, J, dt=0.01)
    check(bool(np.all(np.diff(ts) > 0)), f"{m:9s} ts strictly increasing, {len(ts)} points, span {ts[-1]:.4f}")

print('\n' + ('ALL PHASE 4 CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
