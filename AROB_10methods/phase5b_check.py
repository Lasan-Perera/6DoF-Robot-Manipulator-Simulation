"""Phase 5b acceptance checks: full gravity torque model (arm + gripper +
box), extending phase 5a. Confirmed geometry (gp7.urdf.xacro, 2026-09-29),
not assumed: gripper base_link = tool0 exactly; tcp = tool0 + 0.112 m along
tool0's own local Z; grasp targets tcp at world z=0.04; box_b's resting COM
(pick_place.sdf) is world (0.55,-0.25,0.025), same X,Y as the grasp -> box
offset from tcp = (0,0,-0.015) m, held constant while rigidly carried.

Run from AROB_10methods/ with gp7_links.py, raw_fk.py, gravity_model.py,
gripper_links.py, gripper_fk.py, gravity_model_full.py present.
"""
import sys
import numpy as np
import gp7_links as gl
import raw_fk as rf
import gripper_links as gpl
import gripper_fk as gf
import gravity_model as gm
import gravity_model_full as gmf

fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

print('[1] gripper mass is state-independent and left/right mirror-symmetric')
masses_at_states = [gf.lumped_mass_com(t)[0] for t in (0.0, 0.3, 0.75, 0.8)]
check(max(masses_at_states) - min(masses_at_states) < 1e-12,
      f"total mass identical at 4 knuckle angles: {masses_at_states[0]:.6f} kg")
check(abs(gpl.TOTAL_MASS - 0.920864) < 1e-5, f"total gripper mass = {gpl.TOTAL_MASS:.6f} kg")
worst_x = max(abs(gf.lumped_mass_com(t)[1][0]) for t in np.linspace(0, 0.8, 30))
check(worst_x < 2e-5, f"worst |lumped com_x| over 30 states: {worst_x:.2e} (mirror symmetry)")

print('\n[2] full-model gravity torque vs finite-difference potential energy')
def potential_energy_full(q, gripper_theta, carrying_box):
    coms = rf.link_com_positions(q)
    masses = [l['mass'] for l in gl.LINKS[1:]]
    U = sum(m * 9.81 * p[2] for m, p in zip(masses, coms))
    p_tool0, R_tool0, p_tcp = gmf._tool0_and_tcp_world(q)
    _, com_local = gf.lumped_mass_com(gripper_theta)
    U += gmf.GRIPPER_MASS * 9.81 * (p_tool0 + R_tool0 @ com_local)[2]
    if carrying_box:
        U += gmf.BOX_MASS * 9.81 * (p_tcp + R_tool0 @ gmf.BOX_OFFSET_FROM_TCP)[2]
    return U

def numeric_full(q, gripper_theta, carrying_box, h=1e-6):
    tau = np.zeros(6); q = np.asarray(q, dtype=float)
    for i in range(6):
        qp = q.copy(); qp[i] += h; qm = q.copy(); qm[i] -= h
        tau[i] = (potential_energy_full(qp, gripper_theta, carrying_box)
                  - potential_energy_full(qm, gripper_theta, carrying_box)) / (2*h)
    return tau

rng = np.random.default_rng(3)
for label, theta, box in (('gripper open, no box', 0.0, False), ('gripper closed, carrying box', 0.75, True)):
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(-3, 3, size=6)
        worst = max(worst, float(np.max(np.abs(gmf.gravity_torque_full(q, theta, box) - numeric_full(q, theta, box)))))
    check(worst < 1e-6, f"{label:28s} worst |analytic-numeric| over 200 configs: {worst:.2e} N*m")

print('\n[3] joint 1 torque is EXACTLY zero, full model, any gripper/box state')
worst1 = max(abs(gmf.gravity_torque_full(rng.uniform(-3,3,size=6), 0.75, True)[0]) for _ in range(300))
check(worst1 == 0.0, f"worst |tau_1| over 300 random configs: {worst1}")

print('\n[4] adding mass monotonically increases joint 2 holding torque at home pose (physical sanity)')
t_arm = gm.gravity_torque(np.zeros(6))
t_grip = gmf.gravity_torque_full(np.zeros(6), 0.0, False)
t_box = gmf.gravity_torque_full(np.zeros(6), 0.75, True)
check(abs(t_arm[1]) < abs(t_grip[1]) < abs(t_box[1]),
      f"|tau_2|: arm-only {abs(t_arm[1]):.3f} < +gripper {abs(t_grip[1]):.3f} < +box {abs(t_box[1]):.3f} N*m")

print('\n[5] effort limits, full model at home pose (sanity, not a hard requirement)')
limits = [212.66, 205.80, 106.82, 55.86, 32.68, 22.54]
ok = all(abs(t) <= lim for t, lim in zip(t_box, limits))
check(ok, f"gravity-only holding torque (gripper closed + box) within rated effort: {np.round(t_box,4).tolist()}")

print('\n' + ('ALL PHASE 5b CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
