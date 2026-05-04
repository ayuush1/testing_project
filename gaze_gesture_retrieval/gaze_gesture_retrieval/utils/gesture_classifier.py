"""Rule-based gesture classifier built on top of MediaPipe Hands.

Recognised gestures (returned as strings):
    open_palm, fist, point, thumbs_up, peace, grab, none

The implementation is intentionally rule-based so that it works without any
additional model file. A learned classifier can be substituted by replacing
``classify_landmarks``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

try:
    import mediapipe as mp
    _HAS_MP = True
except Exception:
    _HAS_MP = False

import cv2


# MediaPipe Hands landmark indices we need
WRIST = 0
THUMB_TIP = 4
INDEX_MCP, INDEX_PIP, INDEX_TIP = 5, 6, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP = 9, 10, 12
RING_MCP, RING_PIP, RING_TIP = 13, 14, 16
PINKY_MCP, PINKY_PIP, PINKY_TIP = 17, 18, 20


@dataclass
class GestureResult:
    label: str
    confidence: float
    valid: bool


class GestureClassifier:
    def __init__(self, max_num_hands: int = 1):
        if not _HAS_MP:
            self._hands = None
            return
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    def estimate(self, frame_bgr: np.ndarray) -> GestureResult:
        if self._hands is None or frame_bgr is None or frame_bgr.size == 0:
            return GestureResult('none', 0.0, False)
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        out = self._hands.process(rgb)
        if not out.multi_hand_landmarks:
            return GestureResult('none', 0.0, False)
        lms = out.multi_hand_landmarks[0].landmark
        pts = np.array([[p.x, p.y, p.z] for p in lms], dtype=np.float32)
        label, conf = self.classify_landmarks(pts)
        return GestureResult(label=label, confidence=conf, valid=True)

    @staticmethod
    def _finger_extended(points: np.ndarray, mcp: int, pip: int, tip: int) -> bool:
        # Tip should be above PIP (smaller y) for an extended finger when the
        # hand is roughly upright. We also allow the case where the tip is far
        # from the wrist relative to the PIP.
        wrist = points[WRIST]
        d_tip = np.linalg.norm(points[tip] - wrist)
        d_pip = np.linalg.norm(points[pip] - wrist)
        return (points[tip, 1] < points[pip, 1] - 0.02) or (d_tip > d_pip * 1.05)

    @staticmethod
    def _thumb_extended(points: np.ndarray) -> bool:
        # Thumb extended ~horizontally; check distance from index MCP.
        return np.linalg.norm(points[THUMB_TIP] - points[INDEX_MCP]) > 0.08

    @classmethod
    def classify_landmarks(cls, pts: np.ndarray) -> tuple[str, float]:
        index = cls._finger_extended(pts, INDEX_MCP, INDEX_PIP, INDEX_TIP)
        middle = cls._finger_extended(pts, MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP)
        ring = cls._finger_extended(pts, RING_MCP, RING_PIP, RING_TIP)
        pinky = cls._finger_extended(pts, PINKY_MCP, PINKY_PIP, PINKY_TIP)
        thumb = cls._thumb_extended(pts)

        ext = [thumb, index, middle, ring, pinky]
        n_ext = sum(ext)

        # Open palm: all five extended.
        if n_ext == 5:
            return 'open_palm', 0.9
        # Fist / grab: nothing extended.
        if n_ext == 0:
            return 'grab', 0.9
        # Pointing: only index extended.
        if index and not middle and not ring and not pinky:
            return 'point', 0.85
        # Peace / V: index and middle only.
        if index and middle and not ring and not pinky:
            return 'peace', 0.8
        # Thumbs up: only thumb extended and pointing up.
        if thumb and not index and not middle and not ring and not pinky:
            if pts[THUMB_TIP, 1] < pts[WRIST, 1]:
                return 'thumbs_up', 0.85
        # Stop: open palm with thumb folded across counts as fallback open.
        if index and middle and ring and pinky:
            return 'open_palm', 0.7
        return 'none', 0.3
