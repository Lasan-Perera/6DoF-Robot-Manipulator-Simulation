"""
Send ONE commanded profile (LSPB / cubic / quintic) to the real arm
controller with position+velocity+acceleration at every point, log
/joint_states throughout, compute tracking RMSE (mm, at tcp) and real
execution time. Appends one row to results_experiment1.csv per run.

Usage: python3 run_and_log_profile.py lspb
       python3 run_and_log_profile.py cubic
       python3 run_and_log_profile.py quintic
"""
import sys, os, csv, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                 "src", "gp7_pick_place", "gp7_pick_place"))
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import kinematics as kin
import lspb_full_profile as lfp
import cubic_trajectory as ct
import quintic_trajectory as qt

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
V_MAX = 0.5   # TODO: confirm final value, keep identical to experiment1_compare.py
A_MAX = 1.0


def tcp_position(q):
    T = kin.forward_kinematics(q)
    return T[:3, 3] + kin.L_TCP * T[:3, 2]


def build_profile(method, q0, qf):
    if method == "lspb":
        ts, pos, vel, acc, tb, tf = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=0.01)
        return ts, pos, vel, acc, tf
    elif method == "cubic":
        delta = qf - q0
        pacing_idx = int(np.argmax(np.abs(delta)))
        T = ct.segment_duration(delta[pacing_idx], V_MAX, A_MAX)
        ts, pos, vel, acc = ct.cubic_full_profile(q0, qf, T, dt=0.01)
        return ts, pos, vel, acc, T
    elif method == "quintic":
        delta = qf - q0
        pacing_idx = int(np.argmax(np.abs(delta)))
        T = qt.segment_duration(delta[pacing_idx], V_MAX, A_MAX)
        ts, pos, vel, acc, _ = qt.quintic_full_profile(q0, qf, T, dt=0.01)
        return ts, pos, vel, acc, T
    else:
        raise ValueError(method)


class ProfileRunner(Node):
    def __init__(self):
        super().__init__('profile_runner')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.log_times = []
        self.log_positions = []
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
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.log_times.append(t)
        self.log_positions.append(pos)

    def get_current_position(self, timeout_sec=3.0):
        start = time.time()
        while self.latest_position is None and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_position is None:
            raise RuntimeError("never received /joint_states")
        return list(self.latest_position)

    def move_to_unlogged(self, q_target, v_max=0.5, a_max=1.0, dt=0.02):
        """Pre-positioning move, NOT logged/measured -- gets the arm to a
        known start state before the actual experiment run begins."""
        import lspb_full_profile as _lfp
        q_current = np.array(self.get_current_position())
        q_target = np.array(q_target)
        if np.allclose(q_current, q_target, atol=1e-3):
            return  # already there
        ts, pos, vel, acc, tb, tf = _lfp.lspb_full_profile(q_current, q_target, v_max, a_max, dt=dt)
        self.logging = False
        self.run_profile(ts, pos, vel, acc)

    def run_profile(self, ts, pos, vel, acc):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for i in range(len(ts)):
            p = JointTrajectoryPoint()
            p.positions = list(pos[i])
            p.velocities = list(vel[i])
            p.accelerations = list(acc[i])
            t = ts[i]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)

        self.log_times = []
        self.log_positions = []
        self.logging = True

        wall_start = time.time()
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("goal rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wall_end = time.time()

        self.logging = False
        real_execution_time = wall_end - wall_start
        return real_execution_time


def compute_tracking_rmse_mm(ts_cmd, pos_cmd, log_times, log_positions):
    """
    Interpolate the commanded profile onto the /joint_states log
    timestamps, convert both to tcp Cartesian position via FK, RMSE
    over the whole run, in mm.
    """
    log_times = np.array(log_times)
    log_times = log_times - log_times[0]  # relative time, matches ts_cmd's 0-start

    errors_mm = []
    for i, t in enumerate(log_times):
        if t > ts_cmd[-1]:
            continue
        q_cmd = np.array([np.interp(t, ts_cmd, pos_cmd[:, j]) for j in range(6)])
        q_actual = np.array(log_positions[i])
        p_cmd = tcp_position(q_cmd)
        p_actual = tcp_position(q_actual)
        err = np.linalg.norm(p_cmd - p_actual) * 1000.0  # m -> mm
        errors_mm.append(err)

    errors_mm = np.array(errors_mm)
    rmse = np.sqrt(np.mean(errors_mm**2))
    return rmse, errors_mm


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("lspb", "cubic", "quintic"):
        print("usage: python3 run_and_log_profile.py [lspb|cubic|quintic]")
        sys.exit(1)
    method = sys.argv[1]

    with open("lspb.json") as f:
        d = json.load(f)
    q0 = np.array(d["q0"])
    qf = np.array(d["qf"])

    ts, pos, vel, acc, T = build_profile(method, q0, qf)
    print(f"{method}: T={T:.4f}s, {len(ts)} points")

    rclpy.init()
    node = ProfileRunner()

    print("moving to the segment's start pose first (unlogged)...")
    node.move_to_unlogged(q0)
    print("arrived, starting the measured run")

    real_exec_time = node.run_profile(ts, pos, vel, acc)
    print(f"real execution time: {real_exec_time:.4f}s (commanded T={T:.4f}s)")
    print(f"logged {len(node.log_times)} /joint_states samples during the move")

    rmse_mm, errors_mm = compute_tracking_rmse_mm(ts, pos, node.log_times, node.log_positions)
    print(f"tracking RMSE: {rmse_mm:.4f} mm")

    row = {
        "method": method,
        "T_commanded_s": T,
        "T_real_s": real_exec_time,
        "tracking_rmse_mm": rmse_mm,
        "n_samples": len(node.log_times),
    }

    write_header = not os.path.exists("results_experiment1.csv")
    with open("results_experiment1.csv", "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)
    print("appended to results_experiment1.csv")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
