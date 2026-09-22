import numpy as np
import kinematics as kin

q_test = [0.3, 0.2, -0.1, 0.4, 0.15, 0.2]
T = kin.forward_kinematics(q_test)
wc = kin.wrist_center(T[:3, 3], T[:3, :3])

original = kin.solve_position_joints(wc)
variants = kin.solve_position_joints_elbow_variants(wc)

print("original solve_position_joints:", np.round(np.rad2deg(original), 3))
print("variant A:                     ", np.round(np.rad2deg(variants[0]), 3))
print("variant B:                     ", np.round(np.rad2deg(variants[1]), 3))

for name, (t1, t2, t3) in zip(["A", "B"], variants):
    T_check = kin.forward_kinematics([t1, t2, t3, 0, 0, 0])
    wc_check = kin.wrist_center(T_check[:3, 3], T_check[:3, :3])
    print(f"variant {name} wrist center: {np.round(wc_check, 4)}  (target: {np.round(wc, 4)})")

for name, (t1, t2, t3) in zip(["A", "B"], variants):
    full = [t1, t2, t3, 0, 0, 0]
    print(f"variant {name} within limits: {kin.within_limits(full)}")
