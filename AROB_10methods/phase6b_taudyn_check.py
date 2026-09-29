"""Phase 6b acceptance checks: tau_dyn calculator, verified against synthetic
logs with known injected residuals (no live Gazebo needed for this check).

Run from AROB_10methods/ with tau_dyn.py, gravity_model_full.py, gp7_links.py,
raw_fk.py, gripper_links.py, gripper_fk.py present.
"""
import sys
import numpy as np
import tau_dyn as td
import gravity_model_full as gmf

fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

rng = np.random.default_rng(0)
n = 500
log_times = np.linspace(0, 2, n)
log_positions = rng.uniform(-1, 1, size=(n, 6))
predicted = np.array([gmf.gravity_torque_full(log_positions[i], 0.75, False) for i in range(n)])

print('[1] perfect tracking -> tau_dyn exactly 0')
val, _ = td.compute_tau_dyn(log_times, log_positions, predicted.copy(), 0.75, False)
check(val == 0.0, f"tau_dyn = {val}")

print('\n[2] known constant residual on one joint recovered exactly')
eff = predicted.copy(); eff[:, 2] += 5.0
val, per_joint = td.compute_tau_dyn(log_times, log_positions, eff, 0.75, False)
check(val == 5.0 and np.allclose(per_joint, [0,0,5,0,0,0]),
      f"tau_dyn = {val}, per-joint = {np.round(per_joint,4).tolist()}")

print('\n[3] known sinusoidal residual matches its theoretical RMS')
amp = 3.0
eff2 = predicted.copy(); eff2[:, 4] += amp * np.sin(2*np.pi*log_times)
val3, _ = td.compute_tau_dyn(log_times, log_positions, eff2, 0.75, False)
expected = amp / np.sqrt(2)
check(abs(val3 - expected) < 0.01, f"tau_dyn = {val3:.4f}  theoretical RMS = {expected:.4f}")

print('\n[4] NaN samples excluded, not propagated')
eff3 = eff.copy(); eff3[10:20, 0] = np.nan
val4, _ = td.compute_tau_dyn(log_times, log_positions, eff3, 0.75, False)
check(val4 == 5.0, f"tau_dyn with 10 NaN rows injected = {val4} (expect unchanged, 5.0)")

print('\n[5] max-across-joints picks the correct joint even when several have residual')
eff4 = predicted.copy()
eff4[:, 0] += 2.0; eff4[:, 3] += 7.5; eff4[:, 5] += 1.0
val5, per_joint5 = td.compute_tau_dyn(log_times, log_positions, eff4, 0.75, False)
check(val5 == 7.5, f"tau_dyn = {val5}  (expect 7.5, the largest of the three injected residuals)")

print('\n' + ('ALL PHASE 6b CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
