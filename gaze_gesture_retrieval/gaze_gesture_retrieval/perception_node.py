"""Optional camera publisher used during simulation / desk-top testing.

Reads the system webcam (``/dev/video0`` by default) and republishes it as a
``sensor_msgs/Image`` on ``/user_camera/image_raw`` so the gaze and gesture
nodes have something to subscribe to when the robot's own camera is the only
real sensor available.

You can either run this node, or remap the camera topic parameters of the
gaze and gesture nodes to use Gazebo's published camera topic directly.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('topic', '/user_camera/image_raw')
        self.declare_parameter('frame_id', 'user_camera')

        device = self.get_parameter('device').value
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        rate = float(self.get_parameter('rate_hz').value)
        topic = self.get_parameter('topic').value
        self._frame_id = self.get_parameter('frame_id').value

        self._cap = cv2.VideoCapture(device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device}')
        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, topic, 10)
        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(f'perception_node publishing webcam to {topic}')

    def _tick(self) -> None:
        if not self._cap.isOpened():
            return
        ok, frame = self._cap.read()
        if not ok:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
