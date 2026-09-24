"""
Read vla/sequence_targets.json (from gather_targets.py) and execute all
rounds in one continuous run, chaining from wherever the arm ended up
after each round -- only goes through home once, at the very start.
"""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import kinematics as kin
import trajectory as traj

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
GRIPPER_JOINT = ['robotiq_85_left_knuckle_joint']
V_MAX = 0.5
ACCEL = 1.0
R_DOWN = np.diag([1.0, -1.0, -1.0])
APPROACH_HEIGHT = 0.30
RELEASE_OFFSET = 0.05
GRIPPER_CLOSE = 0.75
GRIPPER_OPEN = 0.0


class ExecuteSequence(Node):
    def __init__(self):
        super().__init__('execute_sequence')
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


def solve_round(pick_x, pick_y, pick_top_z, place_z, platform_x, platform_y, prev_solution):
    waypoints_xyz = [
        [pick_x, pick_y, APPROACH_HEIGHT],
        [pick_x, pick_y, pick_top_z],
        [pick_x, pick_y, APPROACH_HEIGHT],
        [platform_x, platform_y, APPROACH_HEIGHT],
        [platform_x, platform_y, place_z + RELEASE_OFFSET],
        [platform_x, platform_y, APPROACH_HEIGHT],
    ]
    solutions = []
    prev = prev_solution
    for xyz in waypoints_xyz:
        T = np.eye(4); T[:3, :3] = R_DOWN; T[:3, 3] = xyz
        q = kin.inverse_kinematics_tcp_continuous(T, previous_solution=prev)
        solutions.append(q)
        prev = q
    return solutions


def main():
    with open('vla/sequence_targets.json') as f:
        rounds = json.load(f)

    rclpy.init()
    node = ExecuteSequence()

    dt = 0.05
    home = [0.0] * 6
    prev_solution = home

    for i, r in enumerate(rounds):
        pick, place = r['pick'], r['place']
        print(f"\n=== round {i+1}: {pick['query']} -> {place['query']} ===")

        object_half_height = pick['top_z'] / 2
        place_z = place['top_z'] + object_half_height

        solutions = solve_round(
            pick['world_x'], pick['world_y'], pick['top_z'],
            place_z, place['world_x'], place['world_y'], prev_solution,
        )

        points = traj.generate_segment_trajectory(list(prev_solution), list(solutions[0]), V_MAX, ACCEL, dt)
        node.send_arm_segment(points, dt, 'move -> approach')

        gripper_actions = {1: GRIPPER_CLOSE, 4: GRIPPER_OPEN}
        for j in range(len(solutions) - 1):
            points = traj.generate_segment_trajectory(list(solutions[j]), list(solutions[j + 1]), V_MAX, ACCEL, dt)
            node.send_arm_segment(points, dt)
            if (j + 1) in gripper_actions:
                node.send_gripper(gripper_actions[j + 1])

        prev_solution = solutions[-1]

    points = traj.generate_segment_trajectory(list(prev_solution), home, V_MAX, ACCEL, dt)
    node.send_arm_segment(points, dt, 'return home')

    print("\nSequence complete.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
