"""Phase 5a acceptance checks: arm-only gravity torque model (no gripper, no
payload yet -- that data was not available when this was written; see
PROTOCOL.md).

Run from AROB_10methods/ with gp7_links.py, raw_fk.py, gravity_model.py
present. Cross-checks against the project's own already-Gazebo-verified D-H
forward kinematics, so no separate kinematics.py import is needed here.
"""
import sys
import numpy as np
import gp7_links as gl
import raw_fk as rf
import gravity_model as gm

fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

DH_PARAMS = [(0.040,90,0.330),(0.445,0,0),(0.040,90,0),(0,270,0.440),(0,90,0),(0,0,0.080)]
def dh_transform(a, alpha_deg, d, theta_deg):
    al=np.deg2rad(alpha_deg); th=np.deg2rad(theta_deg)
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
def _conv(j):
    j=[np.rad2deg(a) for a in j]; return [j[0],90-j[1],j[2],-j[3],j[4],-j[5]]
def dh_forward_kinematics(q):
    tdh=_conv(q); T=np.eye(4)
    for (a,al,d),th in zip(DH_PARAMS,tdh): T=T@dh_transform(a,al,d,th)
    return T

def potential_energy(q):
    coms = rf.link_com_positions(q)
    masses = [l['mass'] for l in gl.LINKS[1:]]
    return sum(m * 9.81 * p[2] for m, p in zip(masses, coms))

def numeric_gravity_torque(q, h=1e-6):
    tau = np.zeros(6); q = np.asarray(q, dtype=float)
    for i in range(6):
        qp = q.copy(); qp[i] += h
        qm = q.copy(); qm[i] -= h
        tau[i] = (potential_energy(qp) - potential_energy(qm)) / (2 * h)
    return tau

print('[1] mass total (incl. base_link, kept for this check only)')
check(abs(gl.TOTAL_MASS - 34.0) < 0.01, f"total = {gl.TOTAL_MASS} kg (expect ~34.0, xacro values round to 3dp)")

print('\n[2] raw_fk.tool0_frame vs the already-Gazebo-verified D-H forward_kinematics')
rng = np.random.default_rng(2)
worst_fk = 0.0
for _ in range(50):
    q = rng.uniform(-3, 3, size=6)
    worst_fk = max(worst_fk, float(np.max(np.abs(dh_forward_kinematics(q) - rf.tool0_frame(q)))))
check(worst_fk < 1e-10, f"worst |D-H - raw| over 50 random configs: {worst_fk:.2e}")

print('\n[3] joint 1 gravity torque is EXACTLY zero for any q (vertical axis)')
worst1 = max(abs(gm.gravity_torque(rng.uniform(-3, 3, size=6))[0]) for _ in range(500))
check(worst1 == 0.0, f"worst |tau_1| over 500 random configs: {worst1}")

print('\n[4] analytic gravity torque vs finite-difference potential energy (the decisive check)')
worst = 0.0
for _ in range(300):
    q = rng.uniform(-3, 3, size=6)
    e = float(np.max(np.abs(gm.gravity_torque(q) - numeric_gravity_torque(q))))
    worst = max(worst, e)
check(worst < 1e-6, f"worst |analytic - numeric| over 300 random configs: {worst:.2e} N*m")

print('\n[5] at q=0, effort limits are not exceeded by gravity alone (sanity, not a hard requirement)')
t0 = gm.gravity_torque(np.zeros(6))
limits = [212.66, 205.80, 106.82, 55.86, 32.68, 22.54]   # from the xacro's own <limit effort=...>
ok = all(abs(t) <= lim for t, lim in zip(t0, limits))
print(f"      torques at q=0: {np.round(t0,4).tolist()}")
print(f"      joint effort limits: {limits}")
check(ok, "gravity-only holding torque stays within each joint's rated effort at q=0")

print('\n' + ('ALL PHASE 5a CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
