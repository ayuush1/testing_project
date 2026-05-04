"""Lightweight gaze / head-pose estimation.

Uses MediaPipe FaceMesh when available (preferred) and falls back to a Haar
cascade based head-yaw estimate. Returns yaw and pitch in degrees plus a
unit vector representing the gaze direction in the camera frame.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import mediapipe as mp
    _HAS_MP = True
except Exception:
    _HAS_MP = False

import cv2


@dataclass
class GazeResult:
    yaw_deg: float
    pitch_deg: float
    direction: Tuple[float, float, float]
    valid: bool


# 3D model of a generic face used for solvePnP head-pose.
_FACE_MODEL = np.array([
    [0.0,    0.0,    0.0],     # nose tip
    [0.0,   -63.6,  -12.5],    # chin
    [-43.3, 32.7,  -26.0],     # left eye left corner
    [43.3,  32.7,  -26.0],     # right eye right corner
    [-28.9, -28.9, -24.1],     # left mouth corner
    [28.9,  -28.9, -24.1],     # right mouth corner
], dtype=np.float64)

# Corresponding indices in MediaPipe FaceMesh
_MP_IDX = [1, 152, 263, 33, 287, 57]


class GazeEstimator:
    def __init__(self, use_face_landmarks: bool = True):
        self.use_face_landmarks = use_face_landmarks and _HAS_MP
        if self.use_face_landmarks:
            self._mesh = mp.solutions.face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=1,
                refine_landmarks=False,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
        else:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self._cascade = cv2.CascadeClassifier(cascade_path)

    def estimate(self, frame_bgr: np.ndarray) -> GazeResult:
        if frame_bgr is None or frame_bgr.size == 0:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)

        if self.use_face_landmarks:
            return self._estimate_facemesh(frame_bgr)
        return self._estimate_haar(frame_bgr)

    def _estimate_facemesh(self, frame_bgr: np.ndarray) -> GazeResult:
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        out = self._mesh.process(rgb)
        if not out.multi_face_landmarks:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)

        lms = out.multi_face_landmarks[0].landmark
        image_pts = np.array(
            [[lms[i].x * w, lms[i].y * h] for i in _MP_IDX],
            dtype=np.float64,
        )
        focal = float(w)
        camera_matrix = np.array([[focal, 0, w / 2.0],
                                  [0, focal, h / 2.0],
                                  [0, 0, 1.0]], dtype=np.float64)
        dist = np.zeros((4, 1))
        ok, rvec, _tvec = cv2.solvePnP(
            _FACE_MODEL, image_pts, camera_matrix, dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)

        rot, _ = cv2.Rodrigues(rvec)
        # Forward axis of the head in camera frame = -Z column
        forward = -rot[:, 2]
        yaw = math.degrees(math.atan2(forward[0], forward[2]))
        pitch = math.degrees(math.atan2(-forward[1],
                                        math.sqrt(forward[0] ** 2 + forward[2] ** 2)))
        n = np.linalg.norm(forward)
        if n < 1e-6:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)
        forward = forward / n
        return GazeResult(
            yaw_deg=yaw,
            pitch_deg=pitch,
            direction=(float(forward[0]), float(forward[1]), float(forward[2])),
            valid=True,
        )

    def _estimate_haar(self, frame_bgr: np.ndarray) -> GazeResult:
        h, w = frame_bgr.shape[:2]
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        faces = self._cascade.detectMultiScale(gray, 1.2, 5)
        if len(faces) == 0:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)
        x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
        cx = x + fw / 2.0
        # Approximate yaw from horizontal offset of face centre.
        yaw = (cx - w / 2.0) / (w / 2.0) * 35.0
        pitch = (y + fh / 2.0 - h / 2.0) / (h / 2.0) * 25.0
        rad_yaw = math.radians(yaw)
        rad_pitch = math.radians(pitch)
        forward = (
            math.sin(rad_yaw) * math.cos(rad_pitch),
            -math.sin(rad_pitch),
            math.cos(rad_yaw) * math.cos(rad_pitch),
        )
        return GazeResult(yaw, pitch, forward, True)
