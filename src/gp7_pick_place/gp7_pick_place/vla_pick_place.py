"""
Vision-driven pick-and-place: pick whatever object was located by
locate_object.py (reads vla/target.json), place it on the fixed
platform ("box B"). Reuses the exact waypoint/gripper offsets tuned
in run_pick_place.py.

To change what gets picked:
  1. Edit the query in the locate_object.py call (Terminal 3, venv)
  2. Rerun capture_frame.py + locate_object.py to regenerate target.json
  3. Rerun this script
"""
import json
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import kinematics as kin
import trajectory as traj

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
GRIPPER_JOINT = ['robotiq_85_left_knuckle_joint']
V_MAX = 0.5
ACCEL = 1.0
R_DOWN = np.diag([1.0, -1.0, -1.0])

APPROACH_HEIGHT = 0.30       # from run_pick_place.py
GRIPPER_CLOSE = 0.75         # from run_pick_place.py
GRIPPER_OPEN = 0.0

# fixed place target: the white platform ("box B"), static, known position
PLATFORM_X, PLATFORM_Y, PLATFORM_TOP_Z = 0.55, 0.5, 0.20
# object half-height derived from its own top_z, not hardcoded --
# every object rests on the ground (base z=0), so half-height = top_z/2


class VlaPickPlace(Node):
    def __init__(self):
        super().__init__('vla_pick_place')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

    def _run_goal(self, client, joint_names, points_with_time, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        for positions, t in points_with_time:
            p = JointTrajectoryPoint()
            p.positions = list(positions)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)
        self.get_logger().info(f'{label}: sending {len(points_with_time)} points')
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'{label}: done')

    def send_arm_segment(self, points, dt, label='arm'):
        self._run_goal(self.arm_client, ARM_JOINTS, [(q, i * dt) for i, q in enumerate(points)], label)

    def send_gripper(self, position, duration=2.0):
        self._run_goal(self.gripper_client, GRIPPER_JOINT, [([position], duration)], 'gripper')


def solve_waypoints(pick_x, pick_y, pick_top_z, place_z):
    waypoints_xyz = [
        [pick_x, pick_y, APPROACH_HEIGHT],
        [pick_x, pick_y, pick_top_z],
        [pick_x, pick_y, APPROACH_HEIGHT],
        [PLATFORM_X, PLATFORM_Y, APPROACH_HEIGHT],
        [PLATFORM_X, PLATFORM_Y, place_z],
        [PLATFORM_X, PLATFORM_Y, APPROACH_HEIGHT],
    ]
    solutions = []
    prev = None
    for xyz in waypoints_xyz:
        T = np.eye(4); T[:3, :3] = R_DOWN; T[:3, 3] = xyz
        q = kin.inverse_kinematics_tcp_continuous(T, previous_solution=prev)
        solutions.append(q)
        prev = q
    return solutions


def main():
    with open('vla/target.json') as f:
        target = json.load(f)
    pick_x, pick_y, pick_top_z = target['world_x'], target['world_y'], target['top_z']
    object_half_height = pick_top_z / 2
    place_z = PLATFORM_TOP_Z + object_half_height
    print(f"picking '{target['query']}' at ({pick_x:.3f}, {pick_y:.3f}, top_z={pick_top_z})")
    print(f"object half-height: {object_half_height:.4f}, place_z: {place_z:.4f}")
    print(f"placing on platform at ({PLATFORM_X}, {PLATFORM_Y}, {place_z:.4f})")

    rclpy.init()
    node = VlaPickPlace()

    solutions = solve_waypoints(pick_x, pick_y, pick_top_z, place_z)
    dt = 0.05

    # home -> waypoint 1 first, avoids the startup jerk (see run_pick_place.py)
    home = [0.0] * 6
    points = traj.generate_segment_trajectory(home, list(solutions[0]), V_MAX, ACCEL, dt)
    node.send_arm_segment(points, dt, 'home -> approach')

    gripper_actions = {1: GRIPPER_CLOSE, 4: GRIPPER_OPEN}
    for i in range(len(solutions) - 1):
        points = traj.generate_segment_trajectory(list(solutions[i]), list(solutions[i + 1]), V_MAX, ACCEL, dt)
        node.send_arm_segment(points, dt)
        if (i + 1) in gripper_actions:
            node.send_gripper(gripper_actions[i + 1])

    print("Pick-and-place complete.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
