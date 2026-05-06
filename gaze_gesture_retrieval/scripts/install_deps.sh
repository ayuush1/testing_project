#!/usr/bin/env bash
# Install Python dependencies for the gaze + gesture retrieval project.
# Run this *inside* the remote_pc_humble Docker container.
set -e

pip3 install --upgrade pip

# IMPORTANT: pin numpy < 2 because the system-installed matplotlib and
# python3-opencv inside the course Docker image are compiled against
# NumPy 1.x. If pip pulls in NumPy 2.x (which ultralytics will happily
# do), you get cascading "_ARRAY_API not found" import errors as soon
# as anything imports matplotlib (and mediapipe imports matplotlib).
pip3 install "numpy<2"

pip3 install \
    mediapipe==0.10.14 \
    opencv-python \
    ultralytics

# Re-pin numpy<2 in case ultralytics tried to upgrade it again above.
pip3 install --force-reinstall --no-deps "numpy<2"

apt-get update
apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-rqt-image-view \
    ros-humble-topic-tools \
    v4l-utils

echo "[gaze_gesture_retrieval] Dependencies installed."
echo
echo "Reminder: on the HOST (outside Docker) run 'xhost +local:root' so"
echo "that GUI windows from the container can attach to your X display."
