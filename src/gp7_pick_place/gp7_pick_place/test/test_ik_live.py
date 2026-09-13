import numpy as np
import kinematics as kin

# pick an arbitrary target pose directly (not derived from your own FK)
target_pos = np.array([0.5, 0.3, 0.6])
target_rot = np.eye(3)   # tool pointing along the same orientation as the zero pose

T_target = np.eye(4)
T_target[:3, :3] = target_rot
T_target[:3, 3] = target_pos

solved = kin.inverse_kinematics(T_target)[0]
print("solved joint angles (rad):", np.round(solved, 4))
print("as trajectory positions:  ", list(np.round(solved, 4)))
