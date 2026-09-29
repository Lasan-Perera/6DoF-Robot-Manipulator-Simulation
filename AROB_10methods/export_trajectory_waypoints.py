"""
AROB paper -- exports the real joint-space q0/qf for Experiment 1 and the
real 4-point via-point list for Experiment 2, computed from the actual
verified waypoints/IK in gp7_pick_place -- not retyped from memory.
"""
import sys, os, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                 "src", "gp7_pick_place", "gp7_pick_place"))
import numpy as np
import kinematics as kin

R_DOWN = np.diag([1.0, -1.0, -1.0])

# Experiment 1: single must-stop segment (approach -> grasp)
exp1_xyz = [
    [0.55, -0.25, 0.30],   # approach
    [0.55, -0.25, 0.06],   # grasp (changed 2026-09-29 to match Exp 2's grasp height)
]

# Experiment 2: grasp (start) -> lift (via 1) -> mid-transit, raised (via 2)
# -> place-approach (end)
exp2_xyz = [
    [0.55, -0.25, 0.06],   # grasp (start, must-stop)
    [0.55, -0.25, 0.30],   # lift (via-point 1)
    [0.525, 0.125, 0.50],  # mid-transit, raised (via-point 2, new)
    [0.50,  0.50,  0.30],  # place-approach (end, must-stop)
]

def solve_chain(xyz_list):
    solutions = []
    prev = None
    for xyz in xyz_list:
        T = np.eye(4); T[:3, :3] = R_DOWN; T[:3, 3] = xyz
        q = kin.inverse_kinematics_tcp_continuous(T, previous_solution=prev)
        solutions.append(list(q))
        prev = q
    return solutions

exp1_q = solve_chain(exp1_xyz)
exp2_q = solve_chain(exp2_xyz)

print("Experiment 1 (approach -> grasp):")
for xyz, q in zip(exp1_xyz, exp1_q):
    print(f"  xyz={xyz}  q(deg)={np.round(np.rad2deg(q), 3).tolist()}")

print("\nExperiment 2 (grasp -> lift -> mid-transit -> place-approach):")
for xyz, q in zip(exp2_xyz, exp2_q):
    print(f"  xyz={xyz}  q(deg)={np.round(np.rad2deg(q), 3).tolist()}")

V_MAX = 0.5   # rad/s -- TODO: confirm final Table 1 value
ACCEL = 1.0   # rad/s^2 -- TODO: confirm final Table 1 value

with open("lspb.json", "w") as f:
    json.dump({"q0": exp1_q[0], "qf": exp1_q[1]}, f, indent=2)

import os
os.makedirs("data/verification", exist_ok=True)
with open("data/verification/via_point.json", "w") as f:
    json.dump({
        "q0": exp2_q[0], "qv1": exp2_q[1], "qv2": exp2_q[2], "qf": exp2_q[3],
        "v_max": V_MAX, "accel": ACCEL,
    }, f, indent=2)

print("\nwrote lspb.json, data/verification/via_point.json")
