import numpy as np
import kinematics as kin

q = [0.3, 0.2, -0.1, 0.4, 0.15, 0.2]
T = kin.forward_kinematics(q)          # this is tool0's pose, verified
wc_true = kin.wrist_center(T[:3,3], T[:3,:3])
print("wrist centre from FK path:", np.round(wc_true, 4))

T_tcp = np.array([
    [-0.069,  0.376,  0.924, 0.713],
    [ 0.565, -0.749,  0.347, 0.232],
    [ 0.822,  0.546, -0.161, 0.643],
    [ 0.0,    0.0,    0.0,   1.0  ],
])
wc_tcp = T_tcp[:3,3] - kin.L_TCP * T_tcp[:3,2]
print("wrist centre from tcp path:", np.round(wc_tcp, 4))
print("difference:", np.round(wc_tcp - wc_true, 4))
