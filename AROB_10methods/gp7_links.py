"""Per-link mass/COM and raw joint geometry, parsed directly from
gp7_macro.xacro (uploaded 2026-09-29). Mass/COM/inertia values are unchanged
from that file -- not re-derived here.

base_link is fixed to world (world-base_link is a FIXED joint), so its mass
contributes zero gravity torque to any joint and is excluded from the torque
sum below, but is kept here for the mass total cross-check.

Joint axes are the RAW URDF axis vectors from <axis xyz="...">, not the
D-H-frame axes used elsewhere in this project (three of those carry a sign
flip relative to the physical joint, exactly why theta2/4/6 needed
conversion in kinematics.py). Each link's <inertial><origin> is expressed in
that link's own URDF frame, so placing it correctly requires chaining these
raw axes, not the D-H ones.
"""
import numpy as np

# name, mass (kg), com in own link frame (m), joint origin rel. parent (m), raw joint axis (unit vector)
LINKS = [
    # base_link: fixed to world, mass kept for the total check, no joint entry
    dict(name='base_link', mass=8.884, com=(-0.0058, -0.0000, 0.0938)),
    dict(name='link_1_s', mass=5.286, com=(0.0151, -0.0000, -0.0438),
         joint_origin=(0, 0, 0.330), axis=(0, 0, 1)),
    dict(name='link_2_l', mass=10.063, com=(-0.0274, 0.0002, 0.1955),
         joint_origin=(0.040, 0, 0), axis=(0, 1, 0)),
    dict(name='link_3_u', mass=4.502, com=(-0.0176, -0.0007, 0.0194),
         joint_origin=(0, 0, 0.445), axis=(0, -1, 0)),
    dict(name='link_4_r', mass=4.601, com=(-0.1937, -0.0001, -0.0000),
         joint_origin=(0.440, 0, 0.040), axis=(-1, 0, 0)),
    dict(name='link_5_b', mass=0.626, com=(0.0176, 0.0000, 0.0002),
         joint_origin=(0, 0, 0), axis=(0, -1, 0)),
    dict(name='link_6_t', mass=0.039, com=(0.0741, 0.0000, 0.0001),
         joint_origin=(0, 0, 0), axis=(-1, 0, 0)),
]
FLANGE_OFFSET = (0.080, 0, 0)          # joint_6_t -> flange, fixed
# flange -> tool0 fixed rotation: rpy = (radians(180), radians(-90), 0)
TOOL0_RPY = (np.pi, -np.pi / 2, 0.0)

TOTAL_MASS = sum(l['mass'] for l in LINKS)
