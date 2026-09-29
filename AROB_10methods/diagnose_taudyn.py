"""Diagnostic: print measured effort vs predicted gravity torque at the FIRST
few logged samples of a run, where the arm should still be essentially at
rest (right after the pre-positioning move settles, before the measured
trajectory has built up any real speed). If tau_dyn's large, uniform values
across methods are a real bug rather than genuine dynamics, it should show
up clearly here as a mismatch even at near-zero velocity.

Usage: python3 diagnose_taudyn.py lspb   (or any Experiment 1 method)
"""
import sys, os, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                 "src", "gp7_pick_place", "gp7_pick_place"))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import lspb_full_profile as lfp
import gravity_model_full as gmf

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']


class DiagNode(Node):
    def __init__(self):
        super().__init__('diag_node')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.arm_client.wait_for_server()
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self._cb, 50)

    def _cb(self, msg):
        idx = [msg.name.index(j) for j in ARM_JOINTS if j in msg.name]
        if len(idx) != 6:
            return
        pos = [msg.position[i] for i in idx]
        eff = [msg.effort[i] if len(msg.effort) > i else float('nan') for i in idx]
        self.latest = (pos, eff)

    def wait_for_state(self, timeout=3.0):
        import time
        start = time.time()
        while self.latest is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest


def main():
    with open('lspb.json') as f:
        d = json.load(f)
    q0 = np.array(d['q0'])

    rclpy.init()
    node = DiagNode()

    print("reading current state (arm should be at rest, wherever it currently is)...")
    import time
    for _ in range(20):
        pos, eff = node.wait_for_state()
        pred = gmf.gravity_torque_full(np.array(pos), gripper_theta=0.0, carrying_box=False)
        residual = np.array(eff) - pred
        print(f"  measured: {np.round(eff,3).tolist()}")
        print(f"  predicted:{np.round(pred,3).tolist()}")
        print(f"  residual: {np.round(residual,3).tolist()}   max|residual|={np.max(np.abs(residual)):.3f}")
        print()
        time.sleep(0.3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
