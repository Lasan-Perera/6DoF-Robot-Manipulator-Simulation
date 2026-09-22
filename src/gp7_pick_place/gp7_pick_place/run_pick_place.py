import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

import kinematics as kin
import trajectory as traj

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
GRIPPER_JOINT = ['robotiq_85_left_knuckle_joint']

V_MAX = 0.5
ACCEL = 1.0

WAYPOINTS_XYZ = [
    [0.55, -0.25, 0.30],
    [0.55, -0.25, 0.05],
    [0.55, -0.25, 0.30],
    [0.50,  0.50, 0.30],
    [0.50,  0.50, 0.325],
    [0.50,  0.50, 0.330],
]
GRIPPER_ACTIONS = {1: 0.75, 4: 0.0}  # after arriving at waypoint index: close(1) / open(0)


def solve_waypoints():
    solutions = []
    prev = None
    R_fixed = np.diag([1.0, -1.0, -1.0])  # straight-down, yaw=0, matches box's axis-aligned spawn
    for xyz in WAYPOINTS_XYZ:
        T = np.eye(4); T[:3, :3] = R_fixed; T[:3, 3] = xyz
        q = kin.inverse_kinematics_tcp_continuous(T, previous_solution=prev)
        solutions.append(q)
        prev = q
    return solutions


class PickPlaceRunner(Node):
    def __init__(self):
        super().__init__('pick_place_runner')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('waiting for arm_controller action server...')
        self.arm_client.wait_for_server()
        self.get_logger().info('waiting for gripper_controller action server...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('both action servers ready')

    def _run_goal(self, client, joint_names, points_with_time, label):
        """
        points_with_time: list of (positions_list, time_from_start_seconds)
        Blocks (via spin_until_future_complete) until the controller
        reports the goal actually finished -- no polling, no guessing.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        for positions, t in points_with_time:
            p = JointTrajectoryPoint()
            p.positions = list(positions)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)

        self.get_logger().info(f'{label}: sending goal, {len(points_with_time)} points, '
                                f'tf={points_with_time[-1][1]:.2f}s')

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'{label}: goal REJECTED')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        ok = (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        self.get_logger().info(f'{label}: finished, error_code={result.error_code} '
                                f'({"OK" if ok else "FAILED"})')
        return ok

    def send_arm_segment(self, points, dt):
        points_with_time = [(q, i * dt) for i, q in enumerate(points)]
        return self._run_goal(self.arm_client, ARM_JOINTS, points_with_time, 'arm')

    def send_gripper(self, position, duration=2.0):
        return self._run_goal(self.gripper_client, GRIPPER_JOINT, [([position], duration)], 'gripper')

    def check_tcp_pose(self, label=''):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform('base_link', 'tcp', rclpy.time.Time())
                p = t.transform.translation
                self.get_logger().info(f'{label} tcp actual: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
                return
            except Exception:
                continue
        self.get_logger().warn(f'{label} tcp lookup failed')


def main():
    rclpy.init()
    node = PickPlaceRunner()

    solutions = solve_waypoints()
    node.get_logger().info('IK solved for all 6 waypoints:')
    for i, q in enumerate(solutions):
        node.get_logger().info(f'  {i+1}: {np.round(np.rad2deg(q), 2)}')

    dt = 0.05

    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.get_logger().info('moving home -> waypoint 1 first (avoids a jump/jerk)')
    points = traj.generate_segment_trajectory(home, list(solutions[0]), V_MAX, ACCEL, dt)
    node.send_arm_segment(points, dt)

    for i in range(len(solutions) - 1):
        q0, qf = solutions[i], solutions[i + 1]
        points = traj.generate_segment_trajectory(list(q0), list(qf), V_MAX, ACCEL, dt)
        node.send_arm_segment(points, dt)

        if (i + 1) in GRIPPER_ACTIONS:
            node.check_tcp_pose(label=f'waypoint {i+1} before close/open')
            node.send_gripper(GRIPPER_ACTIONS[i + 1])

    node.get_logger().info('Pick-and-place sequence complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()