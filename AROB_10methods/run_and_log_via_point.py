"""
Experiment 2 runner: grasp Box B, run the measured multi-waypoint
stretch (grasp -> lift -> mid-transit -> place-approach) using EITHER
the continuous B-spline or a stitched LSPB/cubic/quintic path, log
/joint_states (position + effort) throughout, compute tracking RMSE
(mm, tcp) and real execution time. Appends one row to
results_experiment2.csv per run.

Usage: python3 run_and_log_via_point.py bspline
       python3 run_and_log_via_point.py lspb
       python3 run_and_log_via_point.py cubic
       python3 run_and_log_via_point.py quintic
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
import bspline_trajectory as bt
import stitched_trajectory as st

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
GRIPPER_JOINT = ['robotiq_85_left_knuckle_joint']
GRIPPER_CLOSE = 0.75
GRIPPER_OPEN = 0.0


def tcp_position(q):
    T = kin.forward_kinematics(q)
    return T[:3, 3] + kin.L_TCP * T[:3, 2]


def build_profile(method, waypoints, v_max, a_max):
    if method == "bspline":
        ts, pos, vel, acc, T, peak_v, peak_a = bt.generate_via_point_trajectory(
            waypoints, v_max, a_max, dt=0.02)
        return ts, pos, vel, acc, T.sum()
    elif method in ("lspb", "cubic", "quintic"):
        ts, pos, vel, acc, T_seg, peak_v, peak_a, n_stops = st.generate_stitched_trajectory(
            method, waypoints, v_max, a_max, dt=0.02)
        return ts, pos, vel, acc, T_seg.sum()
    else:
        raise ValueError(method)


class ViaPointRunner(Node):
    def __init__(self):
        super().__init__('via_point_runner')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.log_times = []
        self.log_positions = []
        self.log_efforts = []
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
        self.log_times.append(t)
        self.log_positions.append(pos)
        self.log_efforts.append(eff)

    def get_current_position(self, timeout_sec=3.0):
        start = time.time()
        while self.latest_position is None and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_position is None:
            raise RuntimeError("never received /joint_states")
        return list(self.latest_position)

    def _run_arm_goal(self, ts, pos, vel, acc):
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

        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("arm goal rejected")
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

    def get_gripper_position(self, timeout_sec=2.0):
        result = {"pos": None}
        def cb(msg):
            if 'robotiq_85_left_knuckle_joint' in msg.name:
                idx = msg.name.index('robotiq_85_left_knuckle_joint')
                result["pos"] = msg.position[idx]
        sub = self.create_subscription(JointState, '/joint_states', cb, 10)
        start = time.time()
        while result["pos"] is None and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return result["pos"]

    def send_gripper(self, position, duration=1.0, converge_tol=0.05, converge_timeout=1.5):
        """
        Send the gripper goal, then DON'T trust the action's own success
        signal alone -- poll the actual joint position until it's within
        converge_tol of the target or converge_timeout elapses. Found
        empirically: closing against contact resistance from a box can
        take longer than the commanded duration, and the controller can
        report success purely from elapsed time, not real convergence.
        """
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

        start = time.time()
        last_pos = None
        while time.time() - start < converge_timeout:
            last_pos = self.get_gripper_position(timeout_sec=1.0)
            if last_pos is not None and abs(last_pos - position) < converge_tol:
                return last_pos
        print(f"WARNING: gripper did not converge within {converge_timeout}s "
              f"(last read: {last_pos}, target: {position})")
        return last_pos

    def run_measured(self, ts, pos, vel, acc):
        self.log_times = []
        self.log_positions = []
        self.log_efforts = []
        self.logging = True

        wall_start = time.time()
        self._run_arm_goal(ts, pos, vel, acc)
        wall_end = time.time()

        self.logging = False
        return wall_end - wall_start


def compute_tracking_rmse_mm(ts_cmd, pos_cmd, log_times, log_positions):
    log_times = np.array(log_times)
    log_times = log_times - log_times[0]

    errors_mm = []
    for i, t in enumerate(log_times):
        if t > ts_cmd[-1]:
            continue
        q_cmd = np.array([np.interp(t, ts_cmd, pos_cmd[:, j]) for j in range(6)])
        q_actual = np.array(log_positions[i])
        p_cmd = tcp_position(q_cmd)
        p_actual = tcp_position(q_actual)
        err = np.linalg.norm(p_cmd - p_actual) * 1000.0
        errors_mm.append(err)

    errors_mm = np.array(errors_mm)
    rmse = np.sqrt(np.mean(errors_mm**2))
    return rmse, errors_mm


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("bspline", "lspb", "cubic", "quintic"):
        print("usage: python3 run_and_log_via_point.py [bspline|lspb|cubic|quintic]")
        sys.exit(1)
    method = sys.argv[1]

    with open("data/verification/via_point.json") as f:
        d = json.load(f)
    waypoints = np.array([d["q0"], d["qv1"], d["qv2"], d["qf"]])
    v_max, a_max = d["v_max"], d["accel"]

    ts, pos, vel, acc, T_total = build_profile(method, waypoints, v_max, a_max)
    print(f"{method}: T={T_total:.4f}s, {len(ts)} points")

    rclpy.init()
    node = ViaPointRunner()

    print("moving to approach height above Box B (unlogged)...")
    node.move_to_unlogged(waypoints[1])  # qv1 (lift) shares grasp's X,Y at a safe height
    print("descending straight down to grasp pose (unlogged)...")
    node.move_to_unlogged(waypoints[0])
    print("closing gripper on Box B...")
    gripper_pos = node.send_gripper(GRIPPER_CLOSE)
    print(f"gripper knuckle position after close (converged): {gripper_pos}  (target: {GRIPPER_CLOSE})")

    print("running the measured stretch...")
    real_exec_time = node.run_measured(ts, pos, vel, acc)
    print(f"real execution time: {real_exec_time:.4f}s (commanded T={T_total:.4f}s)")
    print(f"logged {len(node.log_times)} /joint_states samples")

    print("opening gripper (unlogged)...")
    node.send_gripper(GRIPPER_OPEN)

    rmse_mm, errors_mm = compute_tracking_rmse_mm(ts, pos, node.log_times, node.log_positions)
    print(f"tracking RMSE: {rmse_mm:.4f} mm")

    efforts = np.array(node.log_efforts)
    peak_torque = np.nanmax(np.abs(efforts)) if efforts.size else float('nan')
    print(f"peak torque (any joint): {peak_torque:.4f} N*m")

    row = {
        "method": method,
        "T_commanded_s": T_total,
        "T_real_s": real_exec_time,
        "tracking_rmse_mm": rmse_mm,
        "peak_torque_Nm": peak_torque,
        "n_samples": len(node.log_times),
    }

    write_header = not os.path.exists("results_experiment2.csv")
    with open("results_experiment2.csv", "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)
    print("appended to results_experiment2.csv")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
