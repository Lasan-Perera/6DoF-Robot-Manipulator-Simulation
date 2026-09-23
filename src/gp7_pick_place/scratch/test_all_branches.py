import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))

# this code is to test the inverse kinematics function for all branches
import numpy as np
import kinematics as kin

q_true = [0.3, 0.2, -0.1, 0.4, 0.15, 0.2]
T6 = kin.forward_kinematics(q_true)
T_tcp = T6.copy()
T_tcp[:3, 3] = T6[:3, 3] + kin.L_TCP * T6[:3, 2]

branches = kin.inverse_kinematics_tcp_all_branches(T_tcp)
print(f"{len(branches)} branches found\n")
for i, q in enumerate(branches):
    T_check = kin.forward_kinematics(q)
    p = T_check[:3, 3] + kin.L_TCP * T_check[:3, 2]
    err = np.linalg.norm(p - T_tcp[:3, 3])
    print(f"branch {i}: {np.round(np.rad2deg(q), 2)}  pos_err={err:.2e}  within_limits={kin.within_limits(q)}")
