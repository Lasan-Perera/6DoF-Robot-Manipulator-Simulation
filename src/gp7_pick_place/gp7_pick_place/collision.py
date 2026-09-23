import numpy as np
import jacobian as jac
import kinematics as kin


def point_to_segment_distance(point, seg_start, seg_end):
    # Calculate the distance from a point to a line segment in 3D space.
    seg_vec = seg_end - seg_start
    point_vec = point - seg_start

    seg_len_sq = np.dot(seg_vec, seg_vec)
    if seg_len_sq < 1e-12:
        # seg_start and seg_end are the same point -- degenerate segment
        return np.linalg.norm(point - seg_start)

    t = np.dot(point_vec, seg_vec) / seg_len_sq
    t_clamped = np.clip(t, 0.0, 1.0)

    closest_point = seg_start + t_clamped * seg_vec
    distance = np.linalg.norm(point - closest_point)
    return distance

def arm_min_distance(joint_angles_rad, obstacle_center, obstacle_radius):
    origins, _ = jac.frame_origins_and_axes(joint_angles_rad)

    T = kin.forward_kinematics(joint_angles_rad)
    tcp_point = T[:3, 3] + kin.L_TCP * T[:3, 2]
    origins = origins + [tcp_point]  # add TCP as the final segment endpoint

    min_dist = float('inf')
    for i in range(len(origins) - 1):
        d = point_to_segment_distance(obstacle_center, origins[i], origins[i + 1])
        min_dist = min(min_dist, d)

    return min_dist - obstacle_radius