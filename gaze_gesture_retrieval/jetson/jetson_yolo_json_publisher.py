#!/usr/bin/env python3
"""TurtleBot3 Jetson NX YOLO JSON publisher.

Runs YOLO inference *on the Jetson* (using CUDA / TensorRT) and publishes
detections as a small JSON string on ``/yolo/detections_json``. This is
the pattern from Assignment 4: only a few kB/s of metadata go over
``small_blue_wifi`` instead of ~13 MB/s of raw images.

A companion node, ``yolo_json_bridge`` on the Remote PC, parses the
JSON and republishes ``vision_msgs/Detection2DArray`` on
``/perception/detections`` -- which is what ``fusion_node`` already
consumes. Nothing else in the project needs to change.

Run on the Jetson NX (no Docker), with ROS 2 Humble sourced:

    python3 jetson_yolo_json_publisher.py
    # CUDA model:        -p model:=yolo11n.pt    -p device:=cuda:0
    # TensorRT model:    -p model:=yolo11n.engine -p device:=cuda:0
    # USB camera (instead of CSI):  -p camera_source:=v4l2  -p video_device:=/dev/video0

Requirements (one-time):
    sudo apt install -y python3-opencv ros-humble-rclpy
    pip3 install ultralytics
"""
from __future__ import annotations

import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except Exception:
    _HAS_YOLO = False


# CSI pipeline for the Raspberry Pi V2 camera plugged into the Jetson's
# CSI port. Same pipeline as in the course's Assignment 4 sample.
CSI_GST_PIPELINE = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, "
    "format=(string)NV12, framerate=(fraction)30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! appsink"
)


class YoloJsonPublisher(Node):
    def __init__(self) -> None:
        super().__init__('yolo_json_publisher')

        self.declare_parameter('topic', '/yolo/detections_json')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_source', 'csi')   # 'csi' or 'v4l2'
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)

        topic = str(self.get_parameter('topic').value)
        model_name = str(self.get_parameter('model').value)
        device = str(self.get_parameter('device').value)
        self._conf = float(self.get_parameter('confidence').value)
        rate = float(self.get_parameter('rate_hz').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        cam_source = str(self.get_parameter('camera_source').value).lower()
        video_device = str(self.get_parameter('video_device').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)

        if not _HAS_YOLO:
            self.get_logger().fatal(
                'ultralytics is not installed. Run: pip3 install ultralytics'
            )
            raise SystemExit(1)

        self.get_logger().info(
            f'Loading YOLO model {model_name!r} on {device!r}...'
        )
        self._model = YOLO(model_name)
        try:
            self._model.to(device)
        except Exception as exc:
            self.get_logger().warning(
                f'Could not move YOLO to {device!r} ({exc}); using default.'
            )

        if cam_source == 'csi':
            self._cap = cv2.VideoCapture(CSI_GST_PIPELINE, cv2.CAP_GSTREAMER)
            cam_desc = 'CSI (gstreamer/nvarguscamerasrc)'
        else:
            self._cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cam_desc = f'V4L2 ({video_device}) {width}x{height}'

        if not self._cap.isOpened():
            self.get_logger().fatal(
                f'Failed to open camera ({cam_desc}). '
                f'Check that the camera is connected and the user is in the '
                f'video group (sudo usermod -aG video $USER).'
            )
            raise SystemExit(1)

        self._pub = self.create_publisher(String, topic, 10)
        self.create_timer(1.0 / max(1.0, rate), self._tick)

        self.get_logger().info(
            f'YoloJsonPublisher ready: source={cam_desc}, '
            f'topic={topic}, conf={self._conf}, rate={rate:.1f} Hz'
        )

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return

        results = self._model(frame, conf=self._conf, verbose=False)
        if not results:
            return
        r = results[0]
        h, w = frame.shape[:2]

        payload = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'frame_id': self._frame_id,
            'image_width': int(w),
            'image_height': int(h),
            'detections': [],
        }

        names = r.names
        for box in r.boxes:
            cls_idx = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            cx, cy, bw, bh = box.xywh[0].tolist()
            payload['detections'].append({
                'class_id': cls_idx,
                'class_name': names.get(cls_idx, str(cls_idx)),
                'confidence': conf,
                'bbox': {
                    'cx': float(cx),
                    'cy': float(cy),
                    'w': float(bw),
                    'h': float(bh),
                },
            })

        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._cap.release()
        finally:
            return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = YoloJsonPublisher()
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
