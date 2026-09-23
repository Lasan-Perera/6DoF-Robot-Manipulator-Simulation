import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))

import numpy as np
import kinematics as kin

R_down = np.array([[1,0,0],[0,-1,0],[0,0,-1.0]])
def T(xyz):
    M = np.eye(4); M[:3,:3] = R_down; M[:3,3] = xyz
    return M

p1 = kin.inverse_kinematics_tcp(T([0.55, -0.25, 0.15]))[0]
p2 = kin.inverse_kinematics_tcp(T([0.55, -0.24, 0.15]))[0]
print("plain, waypoint jump (deg):", np.round(np.rad2deg(np.array(p2) - np.array(p1)), 2))

p2c = kin.inverse_kinematics_tcp_continuous(T([0.55, -0.24, 0.15]), previous_solution=p1)
print("continuous, waypoint jump (deg):", np.round(np.rad2deg(np.array(p2c) - np.array(p1)), 2))
