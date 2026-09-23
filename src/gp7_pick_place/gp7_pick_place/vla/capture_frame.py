"""
Subscribe to the camera topic, save the first frame received as a .ppm
image (trivial raw format, no extra deps -- PIL/Moondream can read it
directly). Run this in a normal ROS-sourced terminal (needs rclpy),
not inside vla_venv.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameCapture(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.saved = False
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

    def cb(self, msg):
        if self.saved:
            return
        assert msg.encoding == 'rgb8', f'unexpected encoding: {msg.encoding}'
        header = f'P6\n{msg.width} {msg.height}\n255\n'.encode('ascii')
        with open('frame.ppm', 'wb') as f:
            f.write(header)
            f.write(bytes(msg.data))
        self.get_logger().info(f'saved frame.ppm ({msg.width}x{msg.height})')
        self.saved = True


def main():
    rclpy.init()
    node = FrameCapture()
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
