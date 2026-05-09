"""Lightweight gaze / head-pose estimation.

Uses MediaPipe FaceMesh when available (preferred) and falls back to a Haar
cascade based head-yaw estimate. Returns yaw and pitch in degrees plus a
unit vector representing the gaze direction in the camera frame.

When ``use_iris=True`` (default), MediaPipe's ``refine_landmarks`` is enabled
to also detect 5 iris landmarks per eye. The iris centre is measured
relative to the eye socket (inner / outer corners) and added to the head
yaw, giving a "head turn + eye look" combined gaze estimate. Set
``use_iris=False`` for pure head-pose mode.
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
    head_yaw_deg: float = 0.0
    iris_yaw_deg: float = 0.0


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

# MediaPipe FaceMesh "refine_landmarks=True" iris indices.
# Each iris is a 5-point ring; the first (468 / 473) is the centre point.
_LEFT_IRIS_CENTER = 468
_RIGHT_IRIS_CENTER = 473
# Eye socket corners. We use these to normalise the iris position so it is
# independent of how big the face is in the frame.
_LEFT_EYE_OUTER = 33    # outer corner of the user's left eye
_LEFT_EYE_INNER = 133   # inner corner of the user's left eye
_RIGHT_EYE_OUTER = 263  # outer corner of the user's right eye
_RIGHT_EYE_INNER = 362  # inner corner of the user's right eye


class GazeEstimator:
    def __init__(
        self,
        use_face_landmarks: bool = True,
        use_iris: bool = True,
        iris_yaw_range_deg: float = 18.0,
        iris_pitch_range_deg: float = 12.0,
        iris_deadzone: float = 0.07,
    ) -> None:
        """Construct the estimator.

        Parameters
        ----------
        use_face_landmarks
            Use MediaPipe FaceMesh + solvePnP. Falls back to a Haar cascade
            head-yaw estimate when MediaPipe is not available.
        use_iris
            Add iris-based fine cursor on top of head pose. Requires
            FaceMesh ``refine_landmarks=True`` (one extra landmark
            inference; ~5 ms cost).
        iris_yaw_range_deg
            How many degrees of *additional* yaw a fully eccentric eye
            position contributes. ~15 to 25 is a sensible range for most
            users / webcams.
        iris_pitch_range_deg
            Same idea for vertical eye motion.
        iris_deadzone
            Normalised iris offset (0..1) below which the iris contribution
            is ignored. Stops normal saccades from twitching the cursor.
        """
        self.use_face_landmarks = use_face_landmarks and _HAS_MP
        self.use_iris = use_iris and self.use_face_landmarks
        self._iris_yaw_range = float(iris_yaw_range_deg)
        self._iris_pitch_range = float(iris_pitch_range_deg)
        self._iris_dz = float(iris_deadzone)

        if self.use_face_landmarks:
            self._mesh = mp.solutions.face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=1,
                refine_landmarks=self.use_iris,
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

    # ---------------------------------------------------------------- internals
    def _estimate_facemesh(self, frame_bgr: np.ndarray) -> GazeResult:
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        out = self._mesh.process(rgb)
        if not out.multi_face_landmarks:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)

        lms = out.multi_face_landmarks[0].landmark

        # ---- Head pose via solvePnP ---------------------------------------
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
        forward = -rot[:, 2]
        head_yaw = math.degrees(math.atan2(forward[0], forward[2]))
        head_pitch = math.degrees(math.atan2(
            -forward[1],
            math.sqrt(forward[0] ** 2 + forward[2] ** 2),
        ))

        # ---- Iris-based fine cursor ---------------------------------------
        iris_yaw = 0.0
        iris_pitch = 0.0
        if self.use_iris and len(lms) > _RIGHT_IRIS_CENTER:
            iris_yaw, iris_pitch = self._iris_offsets_deg(lms, w, h)

        total_yaw = head_yaw + iris_yaw
        total_pitch = head_pitch + iris_pitch

        rad_y, rad_p = math.radians(total_yaw), math.radians(total_pitch)
        direction = (
            math.sin(rad_y) * math.cos(rad_p),
            -math.sin(rad_p),
            math.cos(rad_y) * math.cos(rad_p),
        )
        n = math.sqrt(sum(c * c for c in direction))
        if n < 1e-6:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)
        direction = tuple(c / n for c in direction)

        return GazeResult(
            yaw_deg=total_yaw,
            pitch_deg=total_pitch,
            direction=direction,
            valid=True,
            head_yaw_deg=head_yaw,
            iris_yaw_deg=iris_yaw,
        )

    def _iris_offsets_deg(self, lms, w: int, h: int) -> Tuple[float, float]:
        """Return additional (yaw, pitch) in degrees from iris position.

        Computes the iris centre relative to the eye socket midpoint,
        normalises by half the eye width / height, applies a dead-zone,
        and scales by the configured ranges. Left + right eye contributions
        are averaged.
        """
        def offset(center_idx, outer_idx, inner_idx):
            cx = lms[center_idx].x * w
            cy = lms[center_idx].y * h
            ox = lms[outer_idx].x * w
            oy = lms[outer_idx].y * h
            ix = lms[inner_idx].x * w
            iy = lms[inner_idx].y * h
            mid_x = (ox + ix) / 2.0
            mid_y = (oy + iy) / 2.0
            half_w = max(1.0, abs(ox - ix) / 2.0)
            half_h = max(1.0, half_w * 0.55)   # eyes are wider than tall
            dx = (cx - mid_x) / half_w
            dy = (cy - mid_y) / half_h
            return dx, dy

        # Note: in MediaPipe coordinates, x grows to the *image's* right.
        # When the user looks to their own left (= image right since the
        # webcam mirrors), the iris drifts to the +x side of the socket.
        # We average both eyes and apply a dead-zone.
        l_dx, l_dy = offset(_LEFT_IRIS_CENTER, _LEFT_EYE_OUTER, _LEFT_EYE_INNER)
        r_dx, r_dy = offset(_RIGHT_IRIS_CENTER, _RIGHT_EYE_OUTER, _RIGHT_EYE_INNER)
        dx = (l_dx + r_dx) / 2.0
        dy = (l_dy + r_dy) / 2.0

        if abs(dx) < self._iris_dz:
            dx = 0.0
        else:
            dx = math.copysign(abs(dx) - self._iris_dz, dx) / max(1e-6, 1.0 - self._iris_dz)
        if abs(dy) < self._iris_dz:
            dy = 0.0
        else:
            dy = math.copysign(abs(dy) - self._iris_dz, dy) / max(1e-6, 1.0 - self._iris_dz)

        dx = max(-1.0, min(1.0, dx))
        dy = max(-1.0, min(1.0, dy))

        # The webcam is mirrored relative to the world: a leftward iris in
        # the image corresponds to looking to the user's right. Our head
        # yaw convention follows the image, so add the iris offset directly.
        yaw_deg = dx * self._iris_yaw_range
        pitch_deg = dy * self._iris_pitch_range
        return yaw_deg, pitch_deg

    def _estimate_haar(self, frame_bgr: np.ndarray) -> GazeResult:
        h, w = frame_bgr.shape[:2]
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        faces = self._cascade.detectMultiScale(gray, 1.2, 5)
        if len(faces) == 0:
            return GazeResult(0.0, 0.0, (1.0, 0.0, 0.0), False)
        x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
        cx = x + fw / 2.0
        yaw = (cx - w / 2.0) / (w / 2.0) * 35.0
        pitch = (y + fh / 2.0 - h / 2.0) / (h / 2.0) * 25.0
        rad_yaw = math.radians(yaw)
        rad_pitch = math.radians(pitch)
        forward = (
            math.sin(rad_yaw) * math.cos(rad_pitch),
            -math.sin(rad_pitch),
            math.cos(rad_yaw) * math.cos(rad_pitch),
        )
        return GazeResult(yaw, pitch, forward, True,
                          head_yaw_deg=yaw, iris_yaw_deg=0.0)
