"""Live readout of the 6 arm joint velocities. Run while a trajectory
plays in Gazebo to see settling behavior in real time."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

class VelWatcher(Node):
    def __init__(self):
        super().__init__('vel_watcher')
        self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg):
        idx = [msg.name.index(j) for j in ARM_JOINTS if j in msg.name]
        vels = [msg.velocity[i] for i in idx]
        line = '  '.join(f'{v:+.4f}' for v in vels)
        print(f'\r{line}', end='', flush=True)

rclpy.init()
node = VelWatcher()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
