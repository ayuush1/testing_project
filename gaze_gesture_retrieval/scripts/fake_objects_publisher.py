#!/usr/bin/env python3
"""Publish a fake ``Detection2DArray`` and webcam frame so the rest of the
stack can be tested without a working YOLO install or robot camera.

Run inside the Docker container after sourcing your workspace overlay:

    python3 scripts/fake_objects_publisher.py

Press 1/2/3 in the OpenCV window to switch the active fake object.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
import numpy as np
import cv2
from cv_bridge import CvBridge


FAKE = [
    ('bottle',     (200, 240)),
    ('cup',        (320, 240)),
    ('teddy bear', (440, 240)),
]


class FakePub(Node):
    def __init__(self) -> None:
        super().__init__('fake_objects_publisher')
        self._pub_det = self.create_publisher(
            Detection2DArray, '/perception/detections', 10)
        self._pub_img = self.create_publisher(Image, '/robot_camera/image_raw', 10)
        self._bridge = CvBridge()
        self.create_timer(0.2, self._tick)

    def _tick(self) -> None:
        # Publish a synthetic image with three coloured circles as objects.
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        for idx, (label, (cx, cy)) in enumerate(FAKE):
            colour = [(0, 0, 255), (0, 255, 0), (255, 0, 0)][idx]
            cv2.circle(img, (cx, cy), 40, colour, -1)
            cv2.putText(img, label, (cx - 40, cy - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        msg_img = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.header.frame_id = 'fake_camera'
        self._pub_img.publish(msg_img)

        det = Detection2DArray()
        det.header.stamp = msg_img.header.stamp
        det.header.frame_id = 'fake_camera'
        for label, (cx, cy) in FAKE:
            d = Detection2D()
            d.bbox.center.position.x = float(cx)
            d.bbox.center.position.y = float(cy)
            d.bbox.size_x = 80.0
            d.bbox.size_y = 80.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = 0.9
            d.results.append(hyp)
            det.detections.append(d)
        self._pub_det.publish(det)


def main():
    rclpy.init()
    node = FakePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
