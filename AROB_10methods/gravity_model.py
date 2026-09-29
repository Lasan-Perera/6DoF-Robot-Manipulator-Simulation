"""Gravity torque model for the arm's own 7 links (base_link excluded --
fixed to world, contributes nothing). Gripper and payload not yet included.

    tau_i = -sum_{k=i}^{N} m_k * g . (z_pivot_i x (p_com_k - o_pivot_i))

where o_pivot_i, z_pivot_i are joint i's OWN pivot point and axis, in world
frame (raw_fk.joint_pivots_and_link_frames -- NOT the same as the previous
link's own frame origin whenever that joint has a nonzero translation
offset, which joints 2/3/4 do; conflating the two was a real bug, caught by
cross-checking against finite-difference potential energy, not by the
"joint 1 must be exactly zero" check alone -- that check is blind to height
errors specifically, since joint 1's axis is vertical. See PROTOCOL.md.

g = (0, 0, -9.81) m/s^2.
"""
import numpy as np
import gp7_links as gl
import raw_fk as rf

G = np.array([0.0, 0.0, -9.81])


def gravity_torque(q):
    pivots, frames = rf.joint_pivots_and_link_frames(q)
    coms = rf.link_com_positions(q)
    masses = [l['mass'] for l in gl.LINKS[1:]]

    tau = np.zeros(6)
    for i in range(6):
        o_i, z_i = pivots[i]
        V = np.zeros(3)
        for k in range(i, 6):
            V += masses[k] * (np.array(coms[k]) - o_i)
        tau[i] = -np.dot(G, np.cross(z_i, V))
    return tau
