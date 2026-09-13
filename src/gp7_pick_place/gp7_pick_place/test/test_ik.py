import numpy as np
import kinematics as kin

true_angles = [0.2, -0.3, 0.4, 0.5, 0.3, -0.4]
T_target = kin.forward_kinematics(true_angles)

solved = kin.inverse_kinematics(T_target)[0]
print("true:  ", np.round(np.rad2deg(true_angles), 3))
print("solved:", np.round(np.rad2deg(solved), 3))

T_check = kin.forward_kinematics(solved)
print("pose match:", np.allclose(T_target, T_check, atol=1e-4))
