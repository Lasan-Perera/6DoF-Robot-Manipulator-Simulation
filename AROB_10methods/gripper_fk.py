"""Forward kinematics for the gripper's own internal chain (relative to its
own base_link), and the lumped total mass/COM as a function of the master
knuckle angle -- needed because the finger sub-links move as the gripper
opens/closes, so the gripper's own COM (relative to its base_link) is not
fixed.
"""
import numpy as np
import gripper_links as gp


def _rotation_about_axis(axis, theta):
    axis = np.asarray(axis, dtype=float); axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def link_world_frames(theta_master):
    """theta_master: robotiq_85_left_knuckle_joint's own angle (rad). Returns
    {link_name: (position, rotation)} for all 8 moving links, relative to
    the gripper's own base_link frame (treated as world/identity here)."""
    frames = {'base_link': (np.zeros(3), np.eye(3))}
    # walk links in an order where each one's parent is already resolved
    order = ['left_knuckle', 'right_knuckle', 'left_inner_knuckle', 'right_inner_knuckle',
             'left_finger', 'right_finger', 'left_finger_tip', 'right_finger_tip']
    for name in order:
        link = gp.GRIPPER_LINKS[name]
        p_parent, R_parent = frames[link['parent']]
        origin = np.array(link['joint_origin'], dtype=float)
        p = p_parent + R_parent @ origin
        if link.get('fixed'):
            R = R_parent
        else:
            theta = gp.MIMICS[name] * theta_master
            R = R_parent @ _rotation_about_axis(link['axis'], theta)
        frames[name] = (p, R)
    return frames


def lumped_mass_com(theta_master):
    """Returns (total_mass, com) -- com relative to the gripper's own
    base_link frame, at the given master joint angle."""
    frames = link_world_frames(theta_master)
    total_m = 0.0
    weighted_p = np.zeros(3)
    for name, link in gp.GRIPPER_LINKS.items():
        p, R = frames[name]
        com_world = p + R @ np.array(link['com'])
        total_m += link['mass']
        weighted_p += link['mass'] * com_world
    return total_m, weighted_p / total_m
