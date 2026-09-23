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

# same TCP target used throughout -- only the branch changes
TARGET_XYZ = [0.55, -0.25, 0.15]
R_DOWN = np.diag([1.0, -1.0, -1.0])


def solve_branches():
    T = np.eye(4)
    T[:3, :3] = R_DOWN
    T[:3, 3] = TARGET_XYZ

    all_branches = kin.inverse_kinematics_tcp_all_branches(T)
    labeled = []
    names = ['elbow-up, no flip', 'elbow-up, wrist-flip',
             'elbow-down, no flip', 'elbow-down, wrist-flip']
    for name, q in zip(names, all_branches):
        ok = kin.within_limits(q)
        labeled.append((name, q, ok))
    return labeled


class BranchDemo(Node):
    def __init__(self):
        super().__init__('branch_demo')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()

    def move_to(self, q_target, label):
        q_current = self.current_q if hasattr(self, 'current_q') else [0.0]*6
        points = traj.generate_segment_trajectory(list(q_current), list(q_target), V_MAX, ACCEL, dt=0.05)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for i, q in enumerate(points):
            p = JointTrajectoryPoint()
            p.positions = list(q)
            t = i * 0.05
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)

        self.get_logger().info(f'moving to: {label}')
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.current_q = q_target
        self.get_logger().info(f'  reached {label}')


def main():
    rclpy.init()
    node = BranchDemo()

    branches = solve_branches()
    node.get_logger().info('Branches found for this target:')
    for name, q, ok in branches:
        node.get_logger().info(f'  {name}: {np.round(np.rad2deg(q), 1)}  within_limits={ok}')

    home = [0.0]*6
    node.move_to(home, 'home')

    for name, q, ok in branches:
        if not ok:
            node.get_logger().warn(f'skipping "{name}" -- exceeds joint limits, cannot command safely')
            continue
        node.move_to(q, name)
        import time
        time.sleep(2.0)  # hold pose so you can see it

    node.move_to(home, 'home')
    node.get_logger().info('Demo complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
