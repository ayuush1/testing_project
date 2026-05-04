#!/usr/bin/env python3
"""Manual sanity check for the gaze and gesture utility modules.

Opens the system webcam, draws the inferred gaze yaw and gesture label on
each frame, and exits on ``q``. Useful before launching the full ROS 2
stack to make sure the camera and MediaPipe are happy.

Run from inside the Docker container:
    cd ~/my_code/gaze_gesture_retrieval
    python3 scripts/test_perception_offline.py
"""
import sys
import os

import cv2

THIS = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(THIS, '..')))

from gaze_gesture_retrieval.utils.gaze_estimator import GazeEstimator
from gaze_gesture_retrieval.utils.gesture_classifier import GestureClassifier


def main() -> int:
    gaze = GazeEstimator(use_face_landmarks=True)
    gest = GestureClassifier(max_num_hands=1)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print('Cannot open webcam', file=sys.stderr)
        return 1

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        g = gaze.estimate(frame)
        h = gest.estimate(frame)
        text = (
            f'yaw={g.yaw_deg:+6.1f} pitch={g.pitch_deg:+6.1f} '
            f'gesture={h.label} ({h.confidence:.2f})'
        )
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow('gaze_gesture_retrieval offline test', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
