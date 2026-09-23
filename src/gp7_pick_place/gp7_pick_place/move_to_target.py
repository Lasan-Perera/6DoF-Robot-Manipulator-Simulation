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
V_MAX = 0.5
ACCEL = 1.0
HOVER_HEIGHT = 0.15  # metres above the object's top surface
R_DOWN = np.diag([1.0, -1.0, -1.0])


class MoveToTarget(Node):
    def __init__(self):
        super().__init__('move_to_target')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()

    def send_path(self, points, dt):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for i, q in enumerate(points):
            p = JointTrajectoryPoint()
            p.positions = list(q)
            t = i * dt
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


def main():
    with open('vla/target.json') as f:
        target = json.load(f)

    x, y, top_z = target['world_x'], target['world_y'], target['top_z']
    hover_z = top_z + HOVER_HEIGHT
    print(f"moving to hover above '{target['query']}': ({x:.3f}, {y:.3f}, {hover_z:.3f})")

    rclpy.init()
    node = MoveToTarget()

    home = [0.0] * 6
    T = np.eye(4); T[:3, :3] = R_DOWN; T[:3, 3] = [x, y, hover_z]
    q_target = kin.inverse_kinematics_tcp(T)[0]

    dt = 0.05
    points = traj.generate_segment_trajectory(home, list(q_target), V_MAX, ACCEL, dt)
    node.send_path(points, dt)
    print("done")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
