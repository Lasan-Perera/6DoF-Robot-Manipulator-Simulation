"""
Full pick-and-place run: 6 waypoints, LSPB trajectories between them,
gripper actuation at grasp and release. Make sure Gazebo is already
running (gazebo.launch.py) before running this.
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import kinematics as kin
import trajectory as traj

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
GRIPPER_JOINT = ['robotiq_85_left_knuckle_joint']

V_MAX = 0.5
ACCEL = 1.0

WAYPOINTS_XYZ = [
    [0.55, -0.25, 0.30],
    [0.55, -0.25, 0.025],
    [0.55, -0.25, 0.30],
    [0.50,  0.50, 0.30],
    [0.50,  0.50, 0.225],
    [0.50,  0.50, 0.30],
]
GRIPPER_ACTIONS = {1: 0.6, 4: 0.0}  # after arriving at waypoint index: close(1) / open(0)


def solve_waypoints():
    solutions = []
    prev = None
    for xyz in WAYPOINTS_XYZ:
        T0 = np.eye(4); T0[:3, :3] = np.diag([1, -1, -1.0]); T0[:3, 3] = xyz
        theta1 = kin.inverse_kinematics_tcp(T0)[0][0]
        R = kin.upright_orientation_tracking_yaw(theta1)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = xyz
        q = kin.inverse_kinematics_tcp_continuous(T, previous_solution=prev)
        solutions.append(q)
        prev = q
    return solutions


class PickPlaceRunner(Node):
    def __init__(self):
        super().__init__('pick_place_runner')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        time.sleep(1.0)  # let publishers connect

    def send_segment(self, points, dt):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        for i, q in enumerate(points):
            p = JointTrajectoryPoint()
            p.positions = list(q)
            t = i * dt
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            msg.points.append(p)
        self.arm_pub.publish(msg)
        tf = (len(points) - 1) * dt
        self.get_logger().info(f'segment: {len(points)} points, {tf:.2f}s')
        time.sleep(tf + 0.3)

    def send_gripper(self, position, duration=1.5):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINT
        p = JointTrajectoryPoint()
        p.positions = [position]
        p.time_from_start.sec = int(duration)
        msg.points = [p]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'gripper -> {position}')
        time.sleep(duration + 0.3)


def main():
    rclpy.init()
    node = PickPlaceRunner()

    solutions = solve_waypoints()
    node.get_logger().info('IK solved for all 6 waypoints:')
    for i, q in enumerate(solutions):
        node.get_logger().info(f'  {i+1}: {np.round(np.rad2deg(q), 2)}')

    dt = 0.05
    for i in range(len(solutions) - 1):
        q0, qf = solutions[i], solutions[i + 1]
        points = traj.generate_segment_trajectory(list(q0), list(qf), V_MAX, ACCEL, dt)
        node.send_segment(points, dt)

        if (i + 1) in GRIPPER_ACTIONS:
            node.send_gripper(GRIPPER_ACTIONS[i + 1])

    node.get_logger().info('Pick-and-place sequence complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
