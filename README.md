# Gaze-Guided & Gesture-Controlled Object Retrieval — TurtleBot3

Final project for the Texas State University CS 4379K / CS 5342
*Introduction to Autonomous Robotics* course (Spring 2026), implementing
the proposal in `docs/proposal.pdf`.

The complete ROS 2 Humble package, configuration, launch files, and the
detailed step-by-step run guide live under
[`gaze_gesture_retrieval/`](gaze_gesture_retrieval/).

* [Project README](gaze_gesture_retrieval/README.md)
* [Step-by-step run guide](gaze_gesture_retrieval/docs/RUNNING.md)
* [Architecture notes](gaze_gesture_retrieval/docs/ARCHITECTURE.md)

The package is designed to drop into the course's
`~/turtlebot_docker/my_code` shared folder so that it survives container
rebuilds, and reuses the `turtlebot3_manipulation_*` simulation /
navigation / MoveIt packages provided by Assignment 1.
