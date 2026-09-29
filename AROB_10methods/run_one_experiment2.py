"""Experiment 2 run driver: ONE method, ONE repeat per invocation (decision
confirmed: Exp 2 needs a fresh Gazebo restart before every run, since the box
ends up dropped at the platform). An outer shell loop restarts Gazebo and
calls this script once per (method, repeat) pair -- see run_experiment2.sh.

Usage: python3 run_one_experiment2.py [lspb|cubic|quintic|septic|cycloidal|
                                        modsine|modtrap|doubles|bspline|lspbblend]
"""
import sys, os, csv, time, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                 "src", "gp7_pick_place", "gp7_pick_place"))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import kinematics as kin
import lspb_full_profile as lfp
import stitched_trajectory as stt
import bspline_trajectory as bt
import lspb_blend_trajectory as lb
import gravity_model_full as gmf
import tau_dyn as td

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
GRIPPER_JOINT = ['robotiq_85_left_knuckle_joint']
GRIPPER_CLOSE, GRIPPER_OPEN = 0.75, 0.0
V_MAX, A_MAX, J_MAX = 0.5, 1.0, 7.766715171374766
METHODS = ('lspb', 'cubic', 'quintic', 'septic', 'cycloidal', 'modsine', 'modtrap', 'doubles',
           'bspline', 'lspbblend')
STITCHED = ('lspb', 'cubic', 'quintic', 'septic', 'cycloidal', 'modsine', 'modtrap', 'doubles')


def tcp_position(q):
    T = kin.forward_kinematics(q)
    return T[:3, 3] + kin.L_TCP * T[:3, 2]


def build_profile(method, waypoints):
    if method == 'bspline':
        ts, pos, vel, acc, T, pv, pa = bt.generate_via_point_trajectory(waypoints, V_MAX, A_MAX, dt=0.01)
        return ts, pos, vel, acc, T.sum()
    if method == 'lspbblend':
        phases, T, dev = lb.METHOD.plan(waypoints, V_MAX, A_MAX)
        ts, pos, vel, acc, dev2 = lb.METHOD.full_profile(waypoints, T, 0.01, V_MAX, A_MAX)
        return ts, pos, vel, acc, T
    if method in STITCHED:
        ts, pos, vel, acc, T_seg, pv, pa, n_stops = stt.generate_stitched_trajectory(
            method, waypoints, V_MAX, A_MAX, J_MAX, dt=0.01)
        return ts, pos, vel, acc, T_seg.sum()
    raise ValueError(method)


class Exp2Runner(Node):
    def __init__(self):
        super().__init__('exp2_runner')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.log_times, self.log_positions, self.log_efforts = [], [], []
        self.logging = False
        self.latest_position = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 50)

    def _js_cb(self, msg):
        idx = [msg.name.index(j) for j in ARM_JOINTS if j in msg.name]
        if len(idx) != 6:
            return
        pos = [msg.position[i] for i in idx]
        self.latest_position = pos
        if not self.logging:
            return
        eff = [msg.effort[i] if len(msg.effort) > i else float('nan') for i in idx]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.log_times.append(t); self.log_positions.append(pos); self.log_efforts.append(eff)

    def get_current_position(self, timeout_sec=3.0):
        start = time.time()
        while self.latest_position is None and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_position is None:
            raise RuntimeError('never received /joint_states')
        return list(self.latest_position)

    def get_gripper_position(self, timeout_sec=2.0):
        result = {'pos': None}
        def cb(msg):
            if 'robotiq_85_left_knuckle_joint' in msg.name:
                result['pos'] = msg.position[msg.name.index('robotiq_85_left_knuckle_joint')]
        sub = self.create_subscription(JointState, '/joint_states', cb, 10)
        start = time.time()
        while result['pos'] is None and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return result['pos']

    def _run_arm_goal(self, ts, pos, vel, acc):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for i in range(len(ts)):
            p = JointTrajectoryPoint()
            p.positions = list(pos[i]); p.velocities = list(vel[i]); p.accelerations = list(acc[i])
            t = ts[i]
            p.time_from_start.sec = int(t); p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('arm goal rejected')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def move_to_unlogged(self, q_target, v_max=0.5, a_max=1.0, dt=0.02):
        q_current = np.array(self.get_current_position())
        q_target = np.array(q_target)
        if np.allclose(q_current, q_target, atol=1e-3):
            return
        ts, pos, vel, acc, tb, tf = lfp.lspb_full_profile(q_current, q_target, v_max, a_max, dt=dt)
        self.logging = False
        self._run_arm_goal(ts, pos, vel, acc)

    def send_gripper(self, position, duration=1.0, converge_tol=0.05, converge_timeout=1.5):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINT
        p = JointTrajectoryPoint()
        p.positions = [position]
        p.time_from_start.sec = int(duration)
        goal.trajectory.points = [p]
        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        start = time.time(); last_pos = None
        while time.time() - start < converge_timeout:
            last_pos = self.get_gripper_position(timeout_sec=1.0)
            if last_pos is not None and abs(last_pos - position) < converge_tol:
                return last_pos
        print(f"WARNING: gripper did not converge (last read: {last_pos}, target: {position})")
        return last_pos

    def run_measured(self, ts, pos, vel, acc):
        self.log_times, self.log_positions, self.log_efforts = [], [], []
        self.logging = True
        wall_start = time.time()
        self._run_arm_goal(ts, pos, vel, acc)
        wall_end = time.time()
        self.logging = False
        return wall_end - wall_start


def compute_tracking_rmse_mm(ts_cmd, pos_cmd, log_times, log_positions):
    log_times = np.array(log_times); log_times = log_times - log_times[0]
    errors_mm = []
    for i, t in enumerate(log_times):
        if t > ts_cmd[-1]:
            continue
        q_cmd = np.array([np.interp(t, ts_cmd, pos_cmd[:, j]) for j in range(6)])
        q_actual = np.array(log_positions[i])
        err = np.linalg.norm(tcp_position(q_cmd) - tcp_position(q_actual)) * 1000.0
        errors_mm.append(err)
    errors_mm = np.array(errors_mm)
    return float(np.sqrt(np.mean(errors_mm ** 2)))


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in METHODS:
        print(f"usage: python3 run_one_experiment2.py [{'|'.join(METHODS)}]")
        sys.exit(1)
    method = sys.argv[1]

    for path in ('data/verification/via_point.json', 'via_point.json'):
        if os.path.exists(path):
            with open(path) as f:
                vp = json.load(f)
            break
    waypoints = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])

    ts, pos, vel, acc, T = build_profile(method, waypoints)
    print(f"{method}: T={T:.4f}s, {len(ts)} points")

    rclpy.init()
    node = Exp2Runner()

    print("moving to approach height above Box B (unlogged)...")
    node.move_to_unlogged(waypoints[1])   # qv1 (lift) shares grasp's X,Y at a safe height
    print("descending straight down to grasp pose (unlogged)...")
    node.move_to_unlogged(waypoints[0])
    print("closing gripper on Box B...")
    gripper_pos = node.send_gripper(GRIPPER_CLOSE)
    print(f"gripper knuckle position after close (converged): {gripper_pos}")

    print("running the measured stretch...")
    real_T = node.run_measured(ts, pos, vel, acc)
    print(f"real execution time: {real_T:.4f}s (commanded T={T:.4f}s)")
    print(f"logged {len(node.log_times)} /joint_states samples")

    print("opening gripper (unlogged)...")
    node.send_gripper(GRIPPER_OPEN)

    rmse = compute_tracking_rmse_mm(ts, pos, node.log_times, node.log_positions)
    print(f"tracking RMSE: {rmse:.4f} mm")

    tau_dyn_val, _ = td.compute_tau_dyn(node.log_times, node.log_positions, node.log_efforts,
                                         gripper_theta=GRIPPER_CLOSE, carrying_box=True)
    print(f"tau_dyn: {tau_dyn_val:.4f} N*m")

    row = dict(method=method, T_commanded_s=T, T_real_s=real_T,
               tracking_rmse_mm=rmse, tau_dyn_Nm=tau_dyn_val, n_samples=len(node.log_times))
    write_header = not os.path.exists('results_table5.csv')
    with open('results_table5.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)
    print("appended to results_table5.csv")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
