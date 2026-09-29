"""Phase 3 acceptance checks: blended LSPB, method 10 (CHAT_BRIEFING.md sec. 13).

Decision 4 reverted 2026-09-28 to the classical corner-cutting construction:
the pseudo-via-point version passed exactly through the waypoints for one
joint but was over-constrained for six (final position missed qf by 0.25 rad).
This method now reports a genuine, nonzero waypoint deviation instead, which
is data for the C5-C7 contrasts, not a defect to eliminate.

Run from AROB_10methods/ with lspb_blend_trajectory.py, shape_trajectories.py,
lspb.json and data/verification/via_point.json present.
"""
import json, os, sys
import numpy as np
import lspb_full_profile as lfp
import shape_trajectories as st
from lspb_blend_trajectory import METHOD as LB

V, A = 0.5, 1.0
fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

print('[1] two waypoints reduces exactly to plain LSPB')
lspb = json.load(open('lspb.json')); q0, qf = np.array(lspb['q0']), np.array(lspb['qf'])
phases, T, dev = LB.plan(np.array([q0, qf]), V, A)
check(dev == {}, 'no deviations reported on 2 waypoints')
ts_ref, pos_ref, vel_ref, acc_ref, tb_ref, tf_ref = lfp.lspb_full_profile(q0, qf, V, A, dt=0.001)
check(abs(T - tf_ref) < 1e-9, f"T = {T:.6f} vs lspb_full_profile {tf_ref:.6f}")
ts, pos, vel, acc, _ = LB.full_profile(np.array([q0, qf]), T, 0.001, V, A)
n = min(len(ts), len(ts_ref))
check(np.max(np.abs(pos[:n] - pos_ref[:n])) < 1e-9,
      f"position matches lspb_full_profile.lspb_full_profile, max diff {np.max(np.abs(pos[:n]-pos_ref[:n])):.2e}")

print('\n[2] Experiment 2 waypoints: endpoints, velocity continuity, total time')
for path in ('data/verification/via_point.json', 'via_point.json'):
    if os.path.exists(path): vp = json.load(open(path)); break
W = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])
phases, T, dev = LB.plan(W, V, A)
ts, pos, vel, acc, dev2 = LB.full_profile(W, T, 0.001, V, A)
check(np.max(np.abs(pos[0]-W[0])) < 1e-13 and np.max(np.abs(vel[0])) < 1e-13,
      f"start: pos err {np.max(np.abs(pos[0]-W[0])):.1e}, vel {np.max(np.abs(vel[0])):.1e}")
check(np.max(np.abs(pos[-1]-W[3])) < 1e-10 and np.max(np.abs(vel[-1])) < 1e-13,
      f"end:   pos err {np.max(np.abs(pos[-1]-W[3])):.1e}, vel {np.max(np.abs(vel[-1])):.1e}")

t_cursor, worst = 0.0, 0.0
for i, (kind, dur, p0, v0, a) in enumerate(phases[:-1]):
    t_cursor += dur
    v_end = v0 + a * dur
    worst = max(worst, float(np.max(np.abs(v_end - phases[i+1][3]))))
check(worst < 1e-12, f"velocity continuity at all 6 internal phase boundaries, worst diff {worst:.2e}")

T1 = np.max(np.abs(W[1]-W[0])) / V; T2 = np.max(np.abs(W[2]-W[1])) / V; T3 = np.max(np.abs(W[3]-W[2])) / V
tb0 = np.max(np.abs((W[1]-W[0])/T1)) / A; tb3 = np.max(np.abs((W[3]-W[2])/T3)) / A
T_hand = T1 + T2 + T3 + tb0/2 + tb3/2
check(abs(T - T_hand) < 1e-12, f"T = {T:.4f} vs hand formula T1+T2+T3+tb0/2+tb3/2 = {T_hand:.4f}")

print('\n[3] waypoint deviation: analytic formula vs independently sampled trajectory')
for label, t_nom, Qi, key in (('qv1', dev['t_qv1'], W[1], 'qv1'), ('qv2', dev['t_qv2'], W[2], 'qv2')):
    idx = np.argmin(np.abs(ts - t_nom))
    sampled = pos[idx] - Qi
    e = float(np.max(np.abs(sampled - dev[key])))
    check(e < 1e-3, f"{label}: analytic max|dev|={np.max(np.abs(dev[key])):.4f} rad, sampled max|dev|={np.max(np.abs(sampled)):.4f} rad, diff {e:.1e} (grid dt=0.001)")

print('\n[4] mixed-sign synthetic waypoints (robustness beyond the one real case)')
rng = np.random.default_rng(0)
W2 = np.cumsum(rng.uniform(-0.9, 0.9, size=(4, 6)), axis=0)  # comparable scale to the real waypoints
try:
    phases2, T2_, dev2_ = LB.plan(W2, V, A)
    ts2, pos2, vel2, acc2, _ = LB.full_profile(W2, T2_, 0.001, V, A)
    ok = (np.max(np.abs(pos2[0]-W2[0])) < 1e-12 and np.max(np.abs(vel2[0])) < 1e-12
          and np.max(np.abs(pos2[-1]-W2[3])) < 1e-9 and np.max(np.abs(vel2[-1])) < 1e-12)
    check(ok, f"random 4-waypoint case: endpoints and zero end-velocity hold, T={T2_:.4f}")
except ValueError as e:
    check(False, f"random case raised: {e}")

print('\n[5] NEWLY RETURNED acceleration integrates back to velocity (added when full_profile was fixed to actually return acc, not silently drop it)')
dt_fine = 0.0005
ts5, pos5, vel5, acc5, _ = LB.full_profile(W, T, dt_fine, V, A)
v_int = np.cumsum(acc5[:-1] * dt_fine, axis=0)
v_int = np.vstack([np.zeros((1, 6)), v_int]) + vel5[0]
e5 = float(np.max(np.abs(v_int - vel5)))
check(e5 < 1e-3, f"integral of returned acc matches returned vel, max diff {e5:.2e} (Euler integration truncation, dt={dt_fine})")

print('\n' + ('ALL PHASE 3 CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
