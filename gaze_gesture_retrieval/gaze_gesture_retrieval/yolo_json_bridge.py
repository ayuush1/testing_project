"""Bridge from Assignment 4 style JSON YOLO detections to the project's
``vision_msgs/Detection2DArray`` topic that ``fusion_node`` consumes.

Subscribes to:
    /yolo/detections_json     std_msgs/String              JSON payload from the
                                                           Jetson YOLO publisher

Publishes:
    /perception/detections    vision_msgs/Detection2DArray same shape that
                                                           ``yolo_node`` would
                                                           publish

This lets us run YOLO inference on the Jetson (low network bandwidth, ~few
KB/s of JSON instead of ~13 MB/s of raw images) without changing any of the
fusion / control / orchestration logic.

Run this *instead of* ``yolo_node`` when working with the physical robot.
"""
from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloJsonBridge(Node):
    def __init__(self) -> None:
        super().__init__('yolo_json_bridge')

        self.declare_parameter('input_topic', '/yolo/detections_json')
        self.declare_parameter('output_topic', '/perception/detections')
        self.declare_parameter('confidence', 0.0)
        self.declare_parameter(
            'classes',
            ['bottle', 'cup', 'teddy bear', 'cell phone', 'book',
             'apple', 'banana', 'orange'],
        )

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        self._min_conf = float(self.get_parameter('confidence').value)
        allowed = list(self.get_parameter('classes').value)
        self._allowed = set(allowed) if allowed else set()

        self._pub = self.create_publisher(Detection2DArray, out_topic, 10)
        self.create_subscription(String, in_topic, self._on_json, 10)

        self.get_logger().info(
            f'yolo_json_bridge: {in_topic} -> {out_topic} '
            f'(min_conf={self._min_conf}, allowed_classes='
            f'{sorted(self._allowed) if self._allowed else "ALL"})'
        )

    def _on_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Bad JSON from publisher: {exc}')
            return

        out = Detection2DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(payload.get('frame_id', 'camera_link'))

        for det in payload.get('detections', []):
            label = str(det.get('class_name', ''))
            conf = float(det.get('confidence', 0.0))
            if conf < self._min_conf:
                continue
            if self._allowed and label not in self._allowed:
                continue
            bbox = det.get('bbox', {})
            cx = float(bbox.get('cx', 0.0))
            cy = float(bbox.get('cy', 0.0))
            w = float(bbox.get('w', 0.0))
            h = float(bbox.get('h', 0.0))

            d = Detection2D()
            d.bbox.center.position.x = cx
            d.bbox.center.position.y = cy
            d.bbox.size_x = w
            d.bbox.size_y = h
            d.id = str(det.get('class_id', ''))
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = conf
            d.results.append(hyp)
            out.detections.append(d)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloJsonBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
