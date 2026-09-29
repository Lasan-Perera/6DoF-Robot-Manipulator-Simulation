"""Experiment 1 run driver: all 8 point-to-point methods, one Gazebo session
(decision confirmed: Exp 1 needs no restart between runs, no box involved).

Usage: python3 run_all_experiment1.py [n_repeats]   (default 30)

For each of the 8 methods, repeats the method n times: move to the segment
start (unlogged), run the measured segment, log /joint_states (position +
effort), compute tracking RMSE and tau_dyn, append one row to
results_table4.csv. No gripper action needed -- Experiment 1 never grasps.
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
import shape_trajectories as st
import double_s_trajectory as ds_mod
import gravity_model_full as gmf
import tau_dyn as td

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
V_MAX, A_MAX, J_MAX = 0.5, 1.0, 7.766715171374766
METHODS = ('lspb', 'cubic', 'quintic', 'septic', 'cycloidal', 'modsine', 'modtrap', 'doubles')


def tcp_position(q):
    T = kin.forward_kinematics(q)
    return T[:3, 3] + kin.L_TCP * T[:3, 2]


def build_profile(method, q0, qf):
    delta = qf - q0
    pace = int(np.argmax(np.abs(delta)))
    dq = delta[pace]
    if method == 'lspb':
        ts, pos, vel, acc, tb, T = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=0.01)
        return ts, pos, vel, acc, T
    if method == 'doubles':
        T = ds_mod.METHOD.segment_duration(dq, V_MAX, A_MAX, J_MAX)
        ts, pos, vel, acc = ds_mod.METHOD.full_profile(q0, qf, T, 0.01, V_MAX, A_MAX, J_MAX)
        return ts, pos, vel, acc, T
    T = st.METHODS[method].segment_duration(dq, V_MAX, A_MAX, J_MAX)
    ts, pos, vel, acc = st.METHODS[method].full_profile(q0, qf, T, 0.01)
    return ts, pos, vel, acc, T


class Exp1Runner(Node):
    def __init__(self):
        super().__init__('exp1_runner')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
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
            raise RuntimeError('goal rejected')
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
    n_repeats = int(sys.argv[1]) if len(sys.argv) > 1 else 30

    with open('lspb.json') as f:
        d = json.load(f)
    q0, qf = np.array(d['q0']), np.array(d['qf'])

    rclpy.init()
    node = Exp1Runner()

    write_header = not os.path.exists('results_table4.csv')
    fieldnames = ['method', 'repeat', 'T_commanded_s', 'T_real_s', 'tracking_rmse_mm', 'tau_dyn_Nm', 'n_samples']
    fcsv = open('results_table4.csv', 'a', newline='')
    writer = csv.DictWriter(fcsv, fieldnames=fieldnames)
    if write_header:
        writer.writeheader()

    for method in METHODS:
        ts, pos, vel, acc, T = build_profile(method, q0, qf)
        print(f"\n=== {method} === T={T:.4f}s, {len(ts)} points")

        for rep in range(n_repeats):
            print(f"  repeat {rep+1}/{n_repeats}...")
            node.move_to_unlogged(q0)
            real_T = node.run_measured(ts, pos, vel, acc)
            rmse = compute_tracking_rmse_mm(ts, pos, node.log_times, node.log_positions)
            tau_dyn_val, _ = td.compute_tau_dyn(node.log_times, node.log_positions, node.log_efforts,
                                                 gripper_theta=0.0, carrying_box=False)
            row = dict(method=method, repeat=rep, T_commanded_s=T, T_real_s=real_T,
                       tracking_rmse_mm=rmse, tau_dyn_Nm=tau_dyn_val, n_samples=len(node.log_times))
            writer.writerow(row); fcsv.flush()
            print(f"    T_real={real_T:.4f}s  RMSE={rmse:.4f}mm  tau_dyn={tau_dyn_val:.4f}Nm")

    fcsv.close()
    node.destroy_node()
    rclpy.shutdown()
    print("\ndone -- results_table4.csv")


if __name__ == '__main__':
    main()
