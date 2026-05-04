"""YOLOv8 object detection ROS 2 node.

Subscribes to the *robot's* RGB camera and publishes:
    /perception/detections   vision_msgs/Detection2DArray

Each detection's bounding box is in image pixels; the ``id`` field of the
Detection2D corresponds to the COCO class index, and the class label is
stored in ``ObjectHypothesisWithPose.hypothesis.class_id`` as a string.

The Ultralytics ``ultralytics`` package is required at runtime. To keep the
node optional during development, the class falls back to publishing an
empty array if YOLO cannot be loaded. Install with:
    pip3 install ultralytics
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

import numpy as np
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except Exception:
    _HAS_YOLO = False


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_node')

        self.declare_parameter('camera_topic', '/robot_camera/image_raw')
        self.declare_parameter('weights', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter(
            'classes',
            ['bottle', 'cup', 'teddy bear', 'cell phone', 'book', 'apple', 'banana', 'orange'],
        )
        self.declare_parameter('publish_rate_hz', 6.0)

        self._topic = self.get_parameter('camera_topic').value
        self._conf = float(self.get_parameter('confidence').value)
        self._allowed = set(self.get_parameter('classes').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        weights = self.get_parameter('weights').value

        self._bridge = CvBridge()
        self._latest: np.ndarray | None = None
        self._model = None

        if _HAS_YOLO:
            try:
                self._model = YOLO(weights)
                self.get_logger().info(f'YOLOv8 loaded from {weights}')
            except Exception as exc:
                self.get_logger().error(f'Failed to load YOLOv8 ({weights}): {exc}')
        else:
            self.get_logger().warning(
                'ultralytics not installed; yolo_node will publish empty detections'
            )

        self.create_subscription(Image, self._topic, self._on_image, 10)
        self._pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.create_timer(1.0 / max(1.0, rate), self._tick)

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._frame_id = msg.header.frame_id or 'camera_rgb_frame'
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge conversion failed: {exc}')

    def _tick(self) -> None:
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._latest is None or self._model is None:
            self._pub.publish(msg)
            return
        msg.header.frame_id = getattr(self, '_frame_id', 'camera_rgb_frame')

        try:
            results = self._model.predict(self._latest, conf=self._conf, verbose=False)
        except Exception as exc:
            self.get_logger().warning(f'YOLO inference failed: {exc}')
            self._pub.publish(msg)
            return

        for r in results:
            names = r.names
            for box in r.boxes:
                cls_idx = int(box.cls[0].item())
                label = names.get(cls_idx, str(cls_idx))
                if self._allowed and label not in self._allowed:
                    continue
                conf = float(box.conf[0].item())
                xyxy = box.xyxy[0].tolist()
                x1, y1, x2, y2 = xyxy
                d = Detection2D()
                d.bbox.center.position.x = float((x1 + x2) / 2.0)
                d.bbox.center.position.y = float((y1 + y2) / 2.0)
                d.bbox.size_x = float(x2 - x1)
                d.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = conf
                d.results.append(hyp)
                d.id = str(cls_idx)
                msg.detections.append(d)

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
