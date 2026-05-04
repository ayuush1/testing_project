"""Multimodal fusion node.

Combines the gaze direction with YOLOv8 object detections to pick a target
object that the user is looking at, and waits for a confirmation gesture
(``grab`` by default) before promoting it to a *locked* target.

Inputs
------
    /gaze/yaw_deg            std_msgs/Float32
    /gaze/state              std_msgs/String          left|right|center|none
    /perception/detections   vision_msgs/Detection2DArray
    /gesture/stable          std_msgs/String

Outputs
-------
    /fusion/candidate_label  std_msgs/String          best gaze-aligned class
    /fusion/locked_target    std_msgs/String          target after confirmation
    /fusion/locked_pixel     geometry_msgs/Point      pixel x,y of the locked
                                                     bbox centre (z = 0)
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray


class FusionNode(Node):
    def __init__(self) -> None:
        super().__init__('fusion_node')

        self.declare_parameter('gaze_cone_deg', 25.0)
        self.declare_parameter('confirm_gesture', 'grab')
        self.declare_parameter('target_lock_timeout_s', 2.0)

        self._cone = float(self.get_parameter('gaze_cone_deg').value)
        self._confirm = str(self.get_parameter('confirm_gesture').value)
        self._timeout = float(self.get_parameter('target_lock_timeout_s').value)

        self._yaw_deg = 0.0
        self._gaze_state = 'none'
        self._last_detections: Detection2DArray | None = None
        self._last_det_time = 0.0
        self._candidate_label: str | None = None
        self._candidate_center: tuple[float, float] | None = None
        self._image_width = 640.0  # updated from detections frame implicitly

        self.create_subscription(Float32, '/gaze/yaw_deg', self._on_yaw, 10)
        self.create_subscription(String, '/gaze/state', self._on_gaze_state, 10)
        self.create_subscription(
            Detection2DArray, '/perception/detections', self._on_detections, 10
        )
        self.create_subscription(String, '/gesture/stable', self._on_gesture, 10)

        self._pub_candidate = self.create_publisher(String, '/fusion/candidate_label', 10)
        self._pub_locked = self.create_publisher(String, '/fusion/locked_target', 10)
        self._pub_pixel = self.create_publisher(Point, '/fusion/locked_pixel', 10)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'fusion_node: cone={self._cone} deg, confirm gesture="{self._confirm}"'
        )

    # ------------------------------------------------------------------ subs
    def _on_yaw(self, msg: Float32) -> None:
        self._yaw_deg = float(msg.data)

    def _on_gaze_state(self, msg: String) -> None:
        self._gaze_state = msg.data

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._last_detections = msg
        self._last_det_time = time.monotonic()

    def _on_gesture(self, msg: String) -> None:
        if msg.data != self._confirm:
            return
        if self._candidate_label is None or self._candidate_center is None:
            self.get_logger().warning(
                f'Got "{msg.data}" but no gaze-aligned candidate to lock')
            return
        self._pub_locked.publish(String(data=self._candidate_label))
        p = Point()
        p.x, p.y = self._candidate_center
        p.z = 0.0
        self._pub_pixel.publish(p)
        self.get_logger().info(
            f'LOCKED target "{self._candidate_label}" '
            f'@ pixel=({p.x:.1f}, {p.y:.1f})'
        )

    # ----------------------------------------------------------------- logic
    def _tick(self) -> None:
        if (
            self._last_detections is None
            or time.monotonic() - self._last_det_time > self._timeout
            or len(self._last_detections.detections) == 0
        ):
            self._candidate_label = None
            self._candidate_center = None
            return

        # Estimate the desired pixel column from the gaze yaw. A perfectly
        # forward gaze (yaw=0) maps to image centre. The conversion uses the
        # configured cone half-angle as the field of view bound.
        # First update image_width from last detection bbox extent if we can.
        max_x = 1.0
        for d in self._last_detections.detections:
            cx = d.bbox.center.position.x + d.bbox.size_x / 2.0
            if cx > max_x:
                max_x = cx
        self._image_width = max(self._image_width, max_x)

        gaze_norm = max(-1.0, min(1.0, self._yaw_deg / self._cone))
        target_px = (1.0 - gaze_norm) * self._image_width / 2.0  # left yaw -> right of image? handled below
        # When the user looks to the LEFT (yaw negative in our convention) the
        # robot-camera target column is on the LEFT of its frame, so we flip:
        target_px = (self._image_width / 2.0) + gaze_norm * (self._image_width / 2.0)

        best = None
        best_dist = float('inf')
        for d in self._last_detections.detections:
            if not d.results:
                continue
            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            dist = abs(cx - target_px)
            if dist < best_dist:
                best_dist = dist
                best = (d.results[0].hypothesis.class_id, (cx, cy))

        if best is not None:
            self._candidate_label, self._candidate_center = best
            self._pub_candidate.publish(String(data=self._candidate_label))


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
