import numpy as np
import kinematics as kin

# tcp pose measured live from TF at q = [0.3, 0.2, -0.1, 0.4, 0.15, 0.2]
T_tcp = np.array([
    [-0.069,  0.376,  0.924, 0.713],
    [ 0.565, -0.749,  0.347, 0.232],
    [ 0.822,  0.546, -0.161, 0.643],
    [ 0.0,    0.0,    0.0,   1.0  ],
])

solved = kin.inverse_kinematics_tcp(T_tcp)[0]
print("true:  ", np.round(np.rad2deg([0.3, 0.2, -0.1, 0.4, 0.15, 0.2]), 3))
print("solved:", np.round(np.rad2deg(solved), 3))
