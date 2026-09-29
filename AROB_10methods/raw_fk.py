"""Forward kinematics built directly from the URDF's own raw joint origins
and axes (gp7_links.py), independent of the D-H frame convention used
elsewhere in this project. Needed because each link's <inertial><origin> is
expressed in this raw frame, not the D-H one, and three joints (2, 4, 6)
differ between the two conventions by more than just an origin shift.

Returns, per joint, BOTH the pivot point (where that joint's own rotation
axis actually sits -- parent frame's origin plus parent frame's rotation
applied to this joint's own translation offset) and the frame after that
joint's rotation has been applied. These are the SAME point only when a
joint's own translation offset is zero (true for joints 5 and 6 here, false
for 2, 3, 4) -- conflating them was a real bug caught by cross-checking
against finite-difference potential energy in gravity_model.py, see
PROTOCOL.md changelog.
"""
import numpy as np
import gp7_links as gl


def _rotation_about_axis(axis, theta):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def joint_pivots_and_link_frames(q):
    """q: 6 physical joint angles (rad). Returns (pivots, link_frames):
    pivots[i] = (position, axis_in_world) for joint i+1's own rotation axis,
                i=0..5 (pivots[0] is joint 1's pivot, world frame).
    link_frames[i] = (position, rotation) for link (i+1)'s own frame, AFTER
                joint (i+1)'s rotation, i=0..5 (link_frames[0] = link_1_s)."""
    pivots, link_frames = [], []
    p, R = np.zeros(3), np.eye(3)
    for i, link in enumerate(gl.LINKS[1:]):
        origin = np.array(link['joint_origin'], dtype=float)
        axis_local = np.array(link['axis'], dtype=float)
        pivot_p = p + R @ origin              # where THIS joint's axis sits
        pivot_axis = R @ axis_local            # THIS joint's axis, in world frame
        pivots.append((pivot_p.copy(), pivot_axis.copy()))
        R = R @ _rotation_about_axis(axis_local, q[i])
        p = pivot_p
        link_frames.append((p.copy(), R.copy()))
    return pivots, link_frames


def link_frames(q):
    """Back-compatible: just the (position, rotation) frames, dropping pivots."""
    return joint_pivots_and_link_frames(q)[1]


def link_com_positions(q):
    _, frames = joint_pivots_and_link_frames(q)
    coms = []
    for link, (p, R) in zip(gl.LINKS[1:], frames):
        com_local = np.array(link['com'], dtype=float)
        coms.append(p + R @ com_local)
    return coms


def tool0_frame(q):
    frames = link_frames(q)
    p6, R6 = frames[-1]
    p_flange = p6 + R6 @ np.array(gl.FLANGE_OFFSET)
    R_tool0 = R6 @ _rpy_matrix(*gl.TOOL0_RPY)
    T = np.eye(4)
    T[:3, :3] = R_tool0
    T[:3, 3] = p_flange
    return T
