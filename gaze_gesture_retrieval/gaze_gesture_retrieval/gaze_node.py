"""Gaze estimation ROS 2 node.

Subscribes to a user-facing camera image stream and publishes:
    /gaze/yaw_deg          std_msgs/Float32      head yaw in degrees
    /gaze/pitch_deg        std_msgs/Float32      head pitch in degrees
    /gaze/direction        geometry_msgs/Vector3 unit gaze direction vector
    /gaze/state            std_msgs/String       'left' | 'right' | 'center' | 'none'
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image

import numpy as np
import cv2

from cv_bridge import CvBridge

from .utils.gaze_estimator import GazeEstimator


class GazeNode(Node):
    def __init__(self) -> None:
        super().__init__('gaze_node')

        self.declare_parameter('camera_topic', '/user_camera/image_raw')
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('smoothing_alpha', 0.4)
        self.declare_parameter('yaw_left_deg', -12.0)
        self.declare_parameter('yaw_right_deg', 12.0)
        self.declare_parameter('use_face_landmarks', True)

        topic = self.get_parameter('camera_topic').value
        self._alpha = float(self.get_parameter('smoothing_alpha').value)
        self._yaw_left = float(self.get_parameter('yaw_left_deg').value)
        self._yaw_right = float(self.get_parameter('yaw_right_deg').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        use_lm = bool(self.get_parameter('use_face_landmarks').value)

        self._bridge = CvBridge()
        self._estimator = GazeEstimator(use_face_landmarks=use_lm)
        self._latest_frame: np.ndarray | None = None
        self._yaw = 0.0
        self._pitch = 0.0

        self.create_subscription(Image, topic, self._on_image, 10)
        self._pub_yaw = self.create_publisher(Float32, '/gaze/yaw_deg', 10)
        self._pub_pitch = self.create_publisher(Float32, '/gaze/pitch_deg', 10)
        self._pub_dir = self.create_publisher(Vector3, '/gaze/direction', 10)
        self._pub_state = self.create_publisher(String, '/gaze/state', 10)

        self.create_timer(1.0 / max(1.0, rate), self._tick)

        self.get_logger().info(
            f'gaze_node started: subscribing to {topic}, '
            f'face_landmarks={use_lm}, thresholds=({self._yaw_left}, {self._yaw_right}) deg'
        )

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge conversion failed: {exc}')

    def _tick(self) -> None:
        if self._latest_frame is None:
            self._pub_state.publish(String(data='none'))
            return
        result = self._estimator.estimate(self._latest_frame)
        if not result.valid:
            self._pub_state.publish(String(data='none'))
            return
        self._yaw = self._alpha * result.yaw_deg + (1.0 - self._alpha) * self._yaw
        self._pitch = self._alpha * result.pitch_deg + (1.0 - self._alpha) * self._pitch

        self._pub_yaw.publish(Float32(data=float(self._yaw)))
        self._pub_pitch.publish(Float32(data=float(self._pitch)))
        v = Vector3()
        v.x, v.y, v.z = result.direction
        self._pub_dir.publish(v)

        if self._yaw < self._yaw_left:
            state = 'left'
        elif self._yaw > self._yaw_right:
            state = 'right'
        else:
            state = 'center'
        self._pub_state.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = GazeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
