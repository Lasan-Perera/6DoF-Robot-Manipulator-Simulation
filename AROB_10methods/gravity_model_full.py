"""Full gravity torque model: arm (7 links, phase 5a) + gripper (always
present, lumped as one point mass whose position depends on the knuckle
angle) + box (only while carrying, Experiment 2's stretch, lumped as one
point mass rigidly offset from tcp).

Confirmed from gp7.urdf.xacro (2026-09-29), not assumed:
  - gripper base_link = tool0 exactly, zero offset, zero rotation
  - tcp = tool0 + 0.112 m along tool0's own local Z axis (the file's
    <origin xyz="0 0 0.112"/>; its comment says "along its x-axis", which
    is stale -- the code, not the comment, is what Gazebo executes, and it
    matches what tcp_position() has computed throughout this project)
  - grasp targets tcp at world z=0.04 (export_trajectory_waypoints.py);
    box_b's own resting COM is world (0.55,-0.25,0.025) (pick_place.sdf),
    same X,Y as the grasp (centered, top-down grasp) -- so box COM relative
    to tcp at grasp, held constant while rigidly carried, is (0,0,-0.015) m

Both the gripper and the box are downstream of ALL SIX arm joints (rigidly
attached past link 6), so each contributes to every joint's torque sum, not
a subset of them -- unlike the arm's own links, which only contribute to
joints at or before their own position in the chain.
"""
import numpy as np
import gp7_links as gl
import raw_fk as rf
import gripper_fk as gf

G = np.array([0.0, 0.0, -9.81])
GRIPPER_MASS = 0.920864                 # from gripper_links.TOTAL_MASS
BOX_MASS = 0.1                          # pick_place.sdf
BOX_OFFSET_FROM_TCP = np.array([0.0, 0.0, -0.015])
TCP_OFFSET_FROM_TOOL0 = np.array([0.0, 0.0, 0.112])   # local Z of tool0


def _tool0_and_tcp_world(q):
    T = rf.tool0_frame(q)
    p_tool0, R_tool0 = T[:3, 3], T[:3, :3]
    p_tcp = p_tool0 + R_tool0 @ TCP_OFFSET_FROM_TOOL0
    return p_tool0, R_tool0, p_tcp


def gravity_torque_full(q, gripper_theta=0.75, carrying_box=False):
    """gripper_theta: the knuckle joint's own angle (rad); GRIPPER_CLOSE=0.75
    is this project's established closed value, used as the default since
    Experiment 2's measured stretch always runs with the gripper closed."""
    pivots, frames = rf.joint_pivots_and_link_frames(q)
    coms = rf.link_com_positions(q)
    masses = [l['mass'] for l in gl.LINKS[1:]]

    p_tool0, R_tool0, p_tcp = _tool0_and_tcp_world(q)
    _, gripper_com_local = gf.lumped_mass_com(gripper_theta)
    gripper_com_world = p_tool0 + R_tool0 @ gripper_com_local

    extra_masses = [GRIPPER_MASS]
    extra_coms = [gripper_com_world]
    if carrying_box:
        box_com_world = p_tcp + R_tool0 @ BOX_OFFSET_FROM_TCP   # box orientation = tool0's (rigid grasp)
        extra_masses.append(BOX_MASS)
        extra_coms.append(box_com_world)

    tau = np.zeros(6)
    for i in range(6):
        o_i, z_i = pivots[i]
        V = np.zeros(3)
        for k in range(i, 6):
            V += masses[k] * (np.array(coms[k]) - o_i)
        for m, p in zip(extra_masses, extra_coms):     # gripper/box: downstream of ALL 6 joints
            V += m * (p - o_i)
        tau[i] = -np.dot(G, np.cross(z_i, V))
    return tau
