"""Gesture recognition ROS 2 node.

Subscribes to a user-facing camera image stream and publishes:
    /gesture/label        std_msgs/String   raw gesture label every tick
    /gesture/stable       std_msgs/String   gesture only after it has been
                                            held for ``confirm_hold_s`` seconds
    /gesture/confidence   std_msgs/Float32  confidence in [0, 1]
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
import numpy as np

from cv_bridge import CvBridge

from .utils.gesture_classifier import GestureClassifier


class GestureNode(Node):
    def __init__(self) -> None:
        super().__init__('gesture_node')

        self.declare_parameter('camera_topic', '/user_camera/image_raw')
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('confirm_hold_s', 0.6)
        self.declare_parameter('max_num_hands', 1)

        topic = self.get_parameter('camera_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._hold = float(self.get_parameter('confirm_hold_s').value)
        max_hands = int(self.get_parameter('max_num_hands').value)

        self._bridge = CvBridge()
        self._classifier = GestureClassifier(max_num_hands=max_hands)
        self._latest_frame: np.ndarray | None = None

        self._candidate = 'none'
        self._candidate_since = time.monotonic()
        self._published_stable = 'none'

        self.create_subscription(Image, topic, self._on_image, 10)
        self._pub_label = self.create_publisher(String, '/gesture/label', 10)
        self._pub_stable = self.create_publisher(String, '/gesture/stable', 10)
        self._pub_conf = self.create_publisher(Float32, '/gesture/confidence', 10)

        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'gesture_node started: subscribing to {topic}, hold={self._hold}s'
        )

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge conversion failed: {exc}')

    def _tick(self) -> None:
        if self._latest_frame is None:
            self._pub_label.publish(String(data='none'))
            return
        result = self._classifier.estimate(self._latest_frame)

        self._pub_label.publish(String(data=result.label))
        self._pub_conf.publish(Float32(data=float(result.confidence)))

        if result.label != self._candidate:
            self._candidate = result.label
            self._candidate_since = time.monotonic()

        held = time.monotonic() - self._candidate_since
        if held >= self._hold and self._candidate != self._published_stable:
            self._published_stable = self._candidate
            self._pub_stable.publish(String(data=self._candidate))


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
