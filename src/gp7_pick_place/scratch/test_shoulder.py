import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))

import numpy as np
import kinematics as kin

q_true = [0.3, 0.2, -0.1, 0.4, 0.15, 0.2]
T = kin.forward_kinematics(q_true)
wc = kin.wrist_center(T[:3, 3], T[:3, :3])

variants = kin.solve_position_joints_shoulder_variants(wc)
for name, (t1, t2, t3) in zip(["forward", "flipped"], variants):
    T_check = kin.forward_kinematics([t1, t2, t3, 0, 0, 0])
    wc_check = kin.wrist_center(T_check[:3, 3], T_check[:3, :3])
    err = np.linalg.norm(wc_check - wc)
    full = [t1, t2, t3, 0, 0, 0]
    print(f"{name}: theta=({np.round(np.rad2deg([t1,t2,t3]),2)})  wc_err={err:.2e}  within_limits(pos only)={kin.within_limits(full)}")
