import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))

import numpy as np
import kinematics as kin

# downward grasp: tcp z-axis points down (-Z in base frame)
R_down = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
])

def check(name, xyz):
    T = np.eye(4)
    T[:3, :3] = R_down
    T[:3, 3] = xyz
    try:
        q = kin.inverse_kinematics_tcp(T)[0]
        deg = np.rad2deg(q)
        lims = [(-170,170), (-65,145), (-70,190), (-190,190), (-135,135), (-360,360)]
        ok = all(lo <= d <= hi for d, (lo, hi) in zip(deg, lims))
        print(f"{name}: {np.round(deg,2)}  within limits: {ok}")
    except Exception as e:
        print(f"{name}: FAILED - {e}")

check("Box B pick ", [0.55, -0.25, 0.025])
check("Box A place", [0.50,  0.50, 0.225])
