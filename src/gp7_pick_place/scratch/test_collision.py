import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))
import numpy as np
import collision as coll
import kinematics as kin

obstacle_center = np.array([0.55, -0.25, 0.20])
obstacle_radius = 0.04

home = [0.0]*6
print("home pose clearance:", coll.arm_min_distance(home, obstacle_center, obstacle_radius))

# a pose from your grasp-approach sequence, roughly mid-descent
T = np.eye(4); T[:3,:3] = np.diag([1,-1,-1.0]); T[:3,3] = [0.55, -0.25, 0.20]
q_near = kin.inverse_kinematics_tcp(T)[0]
print("near-obstacle pose clearance:", coll.arm_min_distance(q_near, obstacle_center, obstacle_radius))

# how far is the TCP itself from the obstacle center?
T_check = kin.forward_kinematics(q_near)
p_frame6 = T_check[:3, 3]
tcp_pos = p_frame6 + kin.L_TCP * T_check[:3, 2]
print("frame 6 origin:", np.round(p_frame6, 4))
print("actual TCP position:", np.round(tcp_pos, 4))
print("target was:", obstacle_center)
