"""HUD overlay so the user can *see* where the gaze pointer is.

Opens a single OpenCV window that shows:

* The robot's camera image (if available on ``robot_camera_topic``)
  or a synthetic black canvas the size of the most recent detection
  frame (useful when only JSON detections come in from the Jetson).
* All YOLO bounding boxes from ``/perception/detections``.
* A vertical red line where the gaze cursor is currently pointing.
* The current candidate box highlighted green, locked target cyan.
* A status bar at the top with mode / candidate / locked / gaze yaw.

This node is purely visual; turning it off doesn't affect anything else.
Enable it with the ``enable_overlay:=true`` launch argument or run it
standalone:

    ros2 run gaze_gesture_retrieval gaze_overlay
"""
from __future__ import annotations

import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

try:
    from cv_bridge import CvBridge
    _HAS_BRIDGE = True
except Exception:
    _HAS_BRIDGE = False


class GazeOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('gaze_overlay_node')

        self.declare_parameter('robot_camera_topic', '/robot_camera/image_raw')
        self.declare_parameter('window_name', 'gaze overlay')
        self.declare_parameter('gaze_cone_deg', 25.0)
        # Visualization-only: the cursor saturates at this yaw. Bigger than
        # gaze_cone_deg so the cursor still moves when the (head + iris)
        # yaw exceeds the fusion cone.
        self.declare_parameter('cursor_yaw_range_deg', 40.0)
        self.declare_parameter('default_width', 1280)
        self.declare_parameter('default_height', 720)
        self.declare_parameter('display_rate_hz', 15.0)

        cam_topic = str(self.get_parameter('robot_camera_topic').value)
        self._win = str(self.get_parameter('window_name').value)
        self._cone = float(self.get_parameter('gaze_cone_deg').value)
        self._cursor_range = float(self.get_parameter('cursor_yaw_range_deg').value)
        self._w = int(self.get_parameter('default_width').value)
        self._h = int(self.get_parameter('default_height').value)
        rate = float(self.get_parameter('display_rate_hz').value)

        self._bridge = CvBridge() if _HAS_BRIDGE else None
        self._frame: np.ndarray | None = None
        self._detections: Detection2DArray | None = None
        self._yaw = 0.0
        self._head_yaw = 0.0
        self._iris_yaw = 0.0
        self._candidate = ''
        self._locked = ''
        self._locked_pixel: tuple[float, float] | None = None
        self._mode = 'NAVIGATION'
        self._retrieval_state = 'IDLE'
        self._t_locked = 0.0

        self.create_subscription(Image, cam_topic, self._on_image, 5)
        self.create_subscription(
            Detection2DArray, '/perception/detections', self._on_detections, 10
        )
        self.create_subscription(Float32, '/gaze/yaw_deg', self._on_yaw, 10)
        self.create_subscription(
            Float32, '/gaze/head_yaw_deg',
            lambda m: setattr(self, '_head_yaw', float(m.data)), 10,
        )
        self.create_subscription(
            Float32, '/gaze/iris_yaw_deg',
            lambda m: setattr(self, '_iris_yaw', float(m.data)), 10,
        )
        self.create_subscription(
            String, '/fusion/candidate_label', self._on_candidate, 10
        )
        self.create_subscription(
            String, '/fusion/locked_target', self._on_locked, 10
        )
        self.create_subscription(
            Point, '/fusion/locked_pixel', self._on_locked_pixel, 10
        )
        self.create_subscription(String, '/system/mode', self._on_mode, 10)
        self.create_subscription(
            String, '/retrieval/state',
            lambda m: setattr(self, '_retrieval_state', m.data), 10,
        )

        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'gaze_overlay_node: camera_topic={cam_topic}, '
            f'cone={self._cone} deg, window="{self._win}"'
        )

    # ------------------------------------------------------------------ subs
    def _on_image(self, msg: Image) -> None:
        if self._bridge is None:
            return
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._w = self._frame.shape[1]
            self._h = self._frame.shape[0]
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge convert failed: {exc}')

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._detections = msg

    def _on_yaw(self, msg: Float32) -> None:
        self._yaw = float(msg.data)

    def _on_candidate(self, msg: String) -> None:
        self._candidate = msg.data

    def _on_locked(self, msg: String) -> None:
        self._locked = msg.data
        self._t_locked = time.monotonic()

    def _on_locked_pixel(self, msg: Point) -> None:
        self._locked_pixel = (float(msg.x), float(msg.y))

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    # ------------------------------------------------------------------ tick
    def _tick(self) -> None:
        if self._frame is not None:
            canvas = self._frame.copy()
        else:
            canvas = np.zeros((self._h, self._w, 3), dtype=np.uint8)
            cv2.putText(
                canvas, 'no robot camera image (Arch B / edge inference)',
                (20, self._h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2,
            )

        h, w = canvas.shape[:2]
        cursor_norm = max(-1.0, min(1.0,
            self._yaw / max(1e-6, self._cursor_range)))
        target_x = int(w / 2.0 + cursor_norm * (w / 2.0))
        # Also draw a faint guideline showing the fusion cone limit, so the
        # user can see where the cursor stops affecting target selection.
        cone_norm = max(-1.0, min(1.0, self._cone / self._cursor_range))
        cone_left = int(w / 2.0 - cone_norm * (w / 2.0))
        cone_right = int(w / 2.0 + cone_norm * (w / 2.0))
        cv2.line(canvas, (cone_left, 0), (cone_left, h),  (90, 90, 90), 1)
        cv2.line(canvas, (cone_right, 0), (cone_right, h), (90, 90, 90), 1)

        # Draw all detections.
        if self._detections is not None:
            for d in self._detections.detections:
                cx = d.bbox.center.position.x
                cy = d.bbox.center.position.y
                bw = d.bbox.size_x
                bh = d.bbox.size_y
                x1 = int(cx - bw / 2.0)
                y1 = int(cy - bh / 2.0)
                x2 = int(cx + bw / 2.0)
                y2 = int(cy + bh / 2.0)
                label = d.results[0].hypothesis.class_id if d.results else ''
                colour = (200, 200, 200)
                if label == self._candidate:
                    colour = (0, 255, 0)         # green
                if label == self._locked and time.monotonic() - self._t_locked < 4.0:
                    colour = (255, 255, 0)       # cyan-ish
                cv2.rectangle(canvas, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(canvas, label, (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

        # Draw gaze cursor.
        cv2.line(canvas, (target_x, 0), (target_x, h), (0, 0, 255), 2)
        cv2.circle(canvas, (target_x, h // 2), 6, (0, 0, 255), -1)

        # Status bar.
        bar = 36
        cv2.rectangle(canvas, (0, 0), (w, bar), (0, 0, 0), -1)
        text = (
            f'mode={self._mode}  state={self._retrieval_state}  '
            f'yaw={self._yaw:+5.1f}'
            f' (h{self._head_yaw:+4.1f},i{self._iris_yaw:+4.1f})  '
            f'cand={self._candidate or "-"}  '
            f'lock={self._locked or "-"}'
        )
        cv2.putText(canvas, text, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        cv2.imshow(self._win, canvas)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GazeOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
