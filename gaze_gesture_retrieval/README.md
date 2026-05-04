# Gaze-Guided & Gesture-Controlled Object Retrieval — TurtleBot3 + OpenMANIPULATOR-X

CS 4379K / CS 5342 Final Project implementation, built on top of the course's
ROS 2 Humble Docker environment from
[`Robotics_Assignment_1`](https://github.com/dmz44/Robotics_Assignment_1).

This package implements the multimodal HRI system from the proposal:

* **Gaze** (head pose) → target selection in retrieval mode and *left/right
  steering* in navigation mode.
* **Hand gestures** (MediaPipe Hands) → confirmation in retrieval mode and
  *forward / backward / stop* in navigation mode.
* **YOLOv8** detects candidate objects from the robot's camera; the
  `fusion_node` selects the gaze-aligned candidate.
* **Nav2** drives the TurtleBot3 base to the target.
* **MoveIt2 / FollowJointTrajectory** drives the OpenMANIPULATOR-X arm through
  pick / lift / deliver / release / home poses.

> The package is designed to live inside the course's shared folder
> (`~/turtlebot_docker/my_code`) so that nothing that depends on it is
> ever lost when the container is recreated.

---

## 1. Repository layout

```
gaze_gesture_retrieval/
├── package.xml
├── setup.py / setup.cfg
├── README.md                       <- this file
├── docs/RUNNING.md                 <- detailed step-by-step run guide
├── config/params.yaml              <- ROS 2 parameters (gaze cone, speeds, poses)
├── launch/
│   ├── perception.launch.py        <- camera + gaze + gesture + YOLO only
│   ├── teleop_only.launch.py       <- hybrid gaze/gesture driving
│   └── bringup.launch.py           <- full stack (perception + fusion + control)
├── scripts/
│   ├── install_deps.sh             <- pip + apt deps inside the container
│   ├── test_perception_offline.py  <- standalone webcam sanity check
│   └── fake_objects_publisher.py   <- synthetic detections for sim debugging
└── gaze_gesture_retrieval/
    ├── perception_node.py          <- webcam -> /user_camera/image_raw
    ├── gaze_node.py                <- gaze estimation (FaceMesh / Haar)
    ├── gesture_node.py             <- gesture classification
    ├── yolo_node.py                <- YOLOv8 detector
    ├── fusion_node.py              <- gaze + detections fusion + lock-on
    ├── mode_manager.py             <- NAVIGATION <-> RETRIEVAL toggle
    ├── teleop_bridge.py            <- gaze/gesture -> /cmd_vel
    ├── arm_controller.py           <- string commands -> arm + gripper actions
    ├── nav_client.py               <- Nav2 NavigateToPose client
    ├── retrieval_orchestrator.py   <- pick & deliver state machine
    └── utils/{gaze_estimator, gesture_classifier, geometry}.py
```

## 2. Dependencies

* The course Docker image (`remote_pc_humble`) from Assignment 1 already
  ships ROS 2 Humble, the `turtlebot3_manipulation_*` packages and Nav2.
* This project additionally needs MediaPipe, OpenCV, Ultralytics (YOLOv8)
  and `cv_bridge`. Install them with

  ```bash
  cd ~/my_code/gaze_gesture_retrieval
  bash scripts/install_deps.sh
  ```

  > Tip: because the Docker container is ephemeral, either keep the package
  > inside `~/my_code/...` (mapped to the host) and re-run `install_deps.sh`
  > after each `docker compose up`, **or** add the matching `RUN`
  > statements to your `Dockerfile` so they bake into the image.

## 3. Build

The package is an `ament_python` ROS 2 package. From inside the container:

```bash
docker exec -it remote_pc_humble bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/my_code/gaze_gesture_retrieval ~/ros2_ws/src/gaze_gesture_retrieval
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gaze_gesture_retrieval
source install/setup.bash
```

## 4. Quick start

For the full step-by-step run guide (every command, every terminal,
expected output) see [`docs/RUNNING.md`](docs/RUNNING.md).

The condensed version:

```bash
# Terminal 1: Gazebo simulation
ros2 launch turtlebot3_manipulation_gazebo gazebo.launch.py

# Terminal 2: MoveIt servo / arm controllers
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py

# Terminal 3: Nav2 with the map saved during Assignment 1
ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py \
    map_yaml_file:=$HOME/map.yaml

# Terminal 4: our multimodal HRI stack
source ~/ros2_ws/install/setup.bash
ros2 launch gaze_gesture_retrieval bringup.launch.py
```

In RViz set the initial pose with **2D Pose Estimate**, then:

* Show your hand to the camera (`open_palm` to drive forward, `fist` to
  stop, `point` to drive backward) while turning your head left/right to
  steer.
* Say `thumbs_up` to switch to **RETRIEVAL** mode.
* Look at one of the detected objects, then make a `grab` gesture; the
  orchestrator will navigate to it, close the gripper, lift, deliver to
  the user pose, release and return home.

## 5. Topic graph

```
+---------------------+        +---------------+        +-----------------+
| perception_node     |  Image | gaze_node     |  yaw   | fusion_node     |
| (webcam)            +------->+ gesture_node  +------->+                 |
+---------------------+        +---------------+        |                 |
                                                        |  gesture/stable |
                                                        |                 |
+---------------------+        +---------------+        |  Detection2DA. |
| robot camera (gz)   +------->+ yolo_node     +------->+                 |
+---------------------+        +---------------+        +--------+--------+
                                                                 |
                                                                 v
                                                         +------------------+
                                                         | retrieval_       |
                                                         | orchestrator     |
                                                         +--------+---------+
                                                                  |
                                       +--------------------------+--------------------------+
                                       v                                                     v
                                +--------------+                                    +-----------------+
                                | nav_client   | -- /navigate_to_pose --> Nav2      | arm_controller  |
                                +--------------+                                    +-----------------+
                                                                                       |     |
                                                              FollowJointTrajectory ---+     +--- GripperCommand

In NAVIGATION mode the gaze + gesture state instead reach
+--------------+
| teleop_bridge| -- /cmd_vel --> base controller
+--------------+
```

## 6. Parameter tuning

Every interesting threshold lives in [`config/params.yaml`](config/params.yaml).
Key knobs:

| Parameter | Effect |
| --- | --- |
| `gaze_node.yaw_left_deg` / `yaw_right_deg` | dead-zone for centred head |
| `gaze_node.smoothing_alpha` | larger -> more responsive, more jittery |
| `gesture_node.confirm_hold_s` | stability window before `/gesture/stable` fires |
| `fusion_node.gaze_cone_deg` | half-angle of the gaze "spotlight" used to pick the best detection |
| `fusion_node.confirm_gesture` | gesture that locks the highlighted target |
| `teleop_bridge.linear_speed` / `angular_speed` | base speeds in NAVIGATION mode |
| `arm_controller.pose_*` | predefined OpenMANIPULATOR-X joint poses |
| `retrieval_orchestrator.object_table` | known world coords of objects (`label:x:y`) |

## 7. License

MIT, matching the parent course repositories.
