#!/usr/bin/env python3
"""TurtleBot3 Jetson camera publisher.

Self-contained ROS 2 Python script intended to run **directly on the
Jetson NX** (no Docker), in the same spirit as the publisher scripts
from Assignment 3 and Assignment 4.

It opens a USB / CSI camera with OpenCV's V4L backend and republishes
frames as ``sensor_msgs/Image`` so the Remote PC's ``yolo_node`` can
consume them.

Usage (on the Jetson, with ROS 2 Humble already sourced):

    python3 jetson_camera_publisher.py
    # or, with overrides:
    python3 jetson_camera_publisher.py \\
        --ros-args \\
        -p video_device:=0 \\
        -p width:=640 \\
        -p height:=480 \\
        -p fps:=15.0 \\
        -p topic:=/robot_camera/image_raw \\
        -p frame_id:=robot_camera

Requirements on the Jetson (one-time):
    sudo apt install -y python3-opencv ros-humble-cv-bridge

The published topic name (``/robot_camera/image_raw`` by default)
matches the ``yolo_node.camera_topic`` parameter on the Remote PC, so
no remap or launch override is needed by default.
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2

try:
    from cv_bridge import CvBridge
    _HAS_BRIDGE = True
except Exception:
    _HAS_BRIDGE = False


class JetsonCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('jetson_camera_publisher')

        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('topic', '/robot_camera/image_raw')
        self.declare_parameter('frame_id', 'robot_camera')

        device = self.get_parameter('video_device').value
        # Allow either an integer index (0) or a path ("/dev/video0").
        if isinstance(device, str) and device.startswith('/dev/'):
            cap_arg = device
        else:
            try:
                cap_arg = int(device)
            except (TypeError, ValueError):
                cap_arg = 0

        self._w = int(self.get_parameter('width').value)
        self._h = int(self.get_parameter('height').value)
        self._fps = float(self.get_parameter('fps').value)
        topic = str(self.get_parameter('topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        self._cap = cv2.VideoCapture(cap_arg, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().fatal(
                f'Could not open camera "{cap_arg}". '
                f'Try a different /dev/videoN or check permissions '
                f'(sudo usermod -aG video $USER).'
            )
            raise SystemExit(1)

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS))

        self._bridge = CvBridge() if _HAS_BRIDGE else None

        self._pub = self.create_publisher(Image, topic, 10)
        self.create_timer(1.0 / max(1.0, self._fps), self._tick)

        self.get_logger().info(
            f'JetsonCameraPublisher: device={cap_arg} '
            f'requested={self._w}x{self._h}@{self._fps:.1f} -> '
            f'actual={actual_w}x{actual_h}@{actual_fps:.1f} '
            f'topic={topic} frame_id={self._frame_id} '
            f'cv_bridge={"yes" if _HAS_BRIDGE else "no (using manual encode)"}'
        )

    def _tick(self) -> None:
        ok, frame_bgr = self._cap.read()
        if not ok or frame_bgr is None:
            self.get_logger().warning('camera read failed (transient)')
            return

        msg = self._encode(frame_bgr)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

    def _encode(self, frame_bgr) -> Image:
        if self._bridge is not None:
            return self._bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        # Fallback (no cv_bridge installed): hand-build the Image message.
        msg = Image()
        msg.height = frame_bgr.shape[0]
        msg.width = frame_bgr.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame_bgr.tobytes()
        return msg

    def destroy_node(self) -> bool:
        try:
            self._cap.release()
        finally:
            return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = JetsonCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
