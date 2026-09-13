"""
Keys:
  w / s   -> x+ / x-   (reach out / pull in)
  a / d   -> y+ / y-   (left / right)
  q / e   -> z+ / z-   (up / down)
  [ / ]   -> smaller / larger step size
  r       -> reset to a safe default pose
  Ctrl-C  -> quit
"""
import sys
import termios
import tty

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import kinematics as kin

JOINT_NAMES = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

# start at a known-good, known-reachable pose
DEFAULT_TARGET = np.array([0.5, 0.3, 0.6])
DEFAULT_ROT = np.eye(3)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class TeleopNode(Node):
    def __init__(self):
        super().__init__('gp7_teleop')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.target = DEFAULT_TARGET.copy()
        self.step = 0.02

    def send(self):
        T = np.eye(4)
        T[:3, :3] = DEFAULT_ROT
        T[:3, 3] = self.target
        try:
            joints = kin.inverse_kinematics(T)[0]
        except Exception as e:
            print(f"  IK failed for {np.round(self.target, 3)}: {e}")
            return

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.4 * 1e9)
        msg.points = [point]
        self.pub.publish(msg)
        print(f"  target: {np.round(self.target, 3)}  joints(deg): {np.round(np.rad2deg(joints), 1)}")


def main():
    rclpy.init()
    node = TeleopNode()
    print(__doc__)
    node.send()

    try:
        while rclpy.ok():
            key = get_key()
            if key == '\x03':  # Ctrl-C
                break
            moved = True
            if key == 'w': node.target[0] += node.step
            elif key == 's': node.target[0] -= node.step
            elif key == 'a': node.target[1] += node.step
            elif key == 'd': node.target[1] -= node.step
            elif key == 'q': node.target[2] += node.step
            elif key == 'e': node.target[2] -= node.step
            elif key == '[': node.step = max(0.005, node.step - 0.005); moved = False
            elif key == ']': node.step = node.step + 0.005; moved = False
            elif key == 'r': node.target = DEFAULT_TARGET.copy()
            else: moved = False

            if moved or key == 'r':
                node.send()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
