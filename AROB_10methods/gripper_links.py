"""Robotiq 2F-85 gripper link data, parsed directly from
robotiq_2f_85_macro_urdf.xacro (uploaded 2026-09-29). 9 links: base_link plus
4 mirror pairs (left/right knuckle, finger, inner_knuckle, finger_tip).

Only robotiq_85_left_knuckle_joint is independently actuated; every other
moving joint is a <mimic> of it (see MIMICS below) -- this is what
run_and_log_via_point.py already commands as GRIPPER_CLOSE/GRIPPER_OPEN.
Left/right pairs are mirror images (x and, where present, the ixz/iyz cross
term flip sign; mass/iyy/izz are identical) -- used below as a build-time
sanity check, not assumed a priori.
"""
import numpy as np

# name, mass (kg), com in own link frame (m), joint origin rel. parent (m), axis (unit vector), parent
GRIPPER_LINKS = {
    'base_link': dict(mass=6.6320197e-01, com=(0.0, 2.274e-05, 0.03232288)),
    'left_knuckle': dict(mass=1.384773208e-02, com=(0.01213197, 0.0002, -0.00058647),
                          joint_origin=(0.03060114, 0.0, 0.05490452), axis=(0, -1, 0), parent='base_link'),
    'right_knuckle': dict(mass=1.384773208e-02, com=(-0.01213197, 0.0002, -0.00058647),
                           joint_origin=(-0.03060114, 0.0, 0.05490452), axis=(0, -1, 0), parent='base_link'),
    'left_finger': dict(mass=4.260376752e-02, com=(0.00346899, -0.00079447, 0.01867121),
                         joint_origin=(0.03152616, 0.0, -0.00376347), fixed=True, parent='left_knuckle'),
    'right_finger': dict(mass=4.260376752e-02, com=(-0.00346899, -5.53e-06, 0.01867121),
                          joint_origin=(-0.03152616, 0.0, -0.00376347), fixed=True, parent='right_knuckle'),
    'left_inner_knuckle': dict(mass=2.969376448e-02, com=(0.01897699, 0.00015001, 0.02247101),
                                joint_origin=(0.0127, 0.0, 0.06142), axis=(0, -1, 0), parent='base_link'),
    'right_inner_knuckle': dict(mass=2.969376448e-02, com=(-0.01926824, 5.001e-05, 0.02222178),
                                 joint_origin=(-0.0127, 0.0, 0.06142), axis=(0, -1, 0), parent='base_link'),
    'left_finger_tip': dict(mass=4.268588744e-02, com=(-0.01456706, -0.0008, 0.01649701),
                             joint_origin=(0.00563134, 0.0, 0.04718515), axis=(0, -1, 0), parent='left_finger'),
    'right_finger_tip': dict(mass=4.268588744e-02, com=(0.01456706, 5e-05, 0.01649701),
                              joint_origin=(-0.00563134, 0.0, 0.04718515), axis=(0, -1, 0), parent='right_finger'),
}
# mimic relationship: (link, multiplier) -- joint angle = multiplier * theta_left_knuckle
MIMICS = {
    'left_knuckle': 1.0,
    'right_knuckle': -1.0,
    'left_inner_knuckle': 1.0,
    'right_inner_knuckle': -1.0,
    'left_finger_tip': -1.0,
    'right_finger_tip': 1.0,
}
TOTAL_MASS = sum(l['mass'] for l in GRIPPER_LINKS.values())
