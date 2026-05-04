#!/usr/bin/env bash
# Install Python dependencies for the gaze + gesture retrieval project.
# Run this *inside* the remote_pc_humble Docker container.
set -e

pip3 install --upgrade pip
pip3 install \
    mediapipe==0.10.14 \
    opencv-python \
    ultralytics \
    numpy

apt-get update
apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-rqt-image-view \
    v4l-utils

echo "[gaze_gesture_retrieval] Dependencies installed."
