# Detailed Step-by-Step Run Guide

This guide walks through running the **Gaze-Guided & Gesture-Controlled
Object Retrieval** system end-to-end on the course's Docker-based ROS 2
Humble setup. It assumes you have already finished Milestone 1 of the
course (Docker image built, Assignment 1's saved `~/map.yaml` available).

> Convention used below:
> * `[Host]` — commands you run on the laptop / desktop *outside* Docker.
> * `[Docker]` — commands inside `docker exec -it remote_pc_humble bash`.
> * `[Jetson]` — commands run on the TurtleBot3's NVIDIA Jetson SBC.

You can run the **whole project in simulation** (Sections 0–4 + 6) or on
the **physical TurtleBot3** (Sections 0, 1, 2, 5, 6).

---

## 0. Prerequisites

1. Course Docker repository cloned and the `remote_pc_humble` container
   already built (Assignment 1 §"Container Setup").
2. A map saved as `~/map.yaml` (from Assignment 1's SLAM section). If you
   do not have one yet, see Section 7 below.
3. A working webcam connected to your laptop *and* — for the simulation
   case — the Gazebo camera plugin already publishing on
   `/camera/image_raw` (the OMX URDF in `turtlebot3_manipulation_gazebo`
   ships one by default).

## 1. Drop the package in the shared folder

`[Host]`
```bash
cd ~/turtlebot_docker/my_code
git clone https://github.com/<your-fork>/gaze_gesture_retrieval.git
# or copy this directory directly into ~/turtlebot_docker/my_code/
```

The `my_code` folder is volume-mounted into the container at
`/root/my_code`, so the package shows up at `~/my_code/gaze_gesture_retrieval`
inside Docker without any further work.

## 2. Allow GUI + start the container

`[Host]`
```bash
xhost +local:root
cd ~/turtlebot_docker
HOST_UID=$(id -u) USER_HOME=$HOME docker compose up -d
```

`[Host]`
```bash
docker exec -it remote_pc_humble bash
```

You should now see the prompt `root@remote-pc-humble:~#`.

## 3. Install runtime dependencies (one-time per container life)

`[Docker]`
```bash
cd ~/my_code/gaze_gesture_retrieval
bash scripts/install_deps.sh
```

This installs MediaPipe, Ultralytics (YOLOv8), OpenCV-Python, `cv_bridge`,
`vision_msgs`, and a handful of helper packages. Because the container is
ephemeral, repeat this command after every `docker compose up -d --build`,
*or* fold it into your Dockerfile (see "Permanent install" at the bottom).

## 4. Build & source the workspace

`[Docker]`
```bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/my_code/gaze_gesture_retrieval ~/ros2_ws/src/gaze_gesture_retrieval

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gaze_gesture_retrieval
source install/setup.bash
```

`colcon build` should finish with `Summary: 1 package finished`.

> **Tip — every new shell**: in addition to `source /opt/ros/humble/setup.bash`,
> always run `source ~/ros2_ws/install/setup.bash` before launching any
> node from this package.

### 4.1 Smoke-test perception (no ROS)

`[Docker]`
```bash
cd ~/my_code/gaze_gesture_retrieval
python3 scripts/test_perception_offline.py
```

A window pops up showing your webcam with the inferred yaw and gesture
label drawn on top. Press **q** to close. If nothing appears, fix the
camera / X11 forwarding before continuing.

---

## 5. Run in **simulation** (Gazebo + Nav2)

You need four (or five) Docker shells. Open each one with

```bash
docker exec -it remote_pc_humble bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 5.1 Terminal 1 — Gazebo world

`[Docker #1]`
```bash
ros2 launch turtlebot3_manipulation_gazebo gazebo.launch.py
```

Wait until the Gazebo window finishes loading. Drop a few mappable
objects (cubes, the OMX-friendly bottle) into the world if you like.

### 5.2 Terminal 2 — MoveIt servo + arm controllers

`[Docker #2]`
```bash
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
```

This brings up `/arm_controller/follow_joint_trajectory` and
`/gripper_controller/gripper_cmd` — the two action servers that
`arm_controller.py` talks to.

### 5.3 Terminal 3 — Nav2 with the saved map

`[Docker #3]`
```bash
ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py \
    map_yaml_file:=$HOME/map.yaml
```

In RViz click **2D Pose Estimate** and set the initial pose so the
particle cloud aligns with the robot in Gazebo.

### 5.4 Terminal 4 — our multimodal HRI stack

`[Docker #4]`
```bash
ros2 launch gaze_gesture_retrieval bringup.launch.py
```

You should immediately see logs from each node:

```
[gaze_node]: gaze_node started: subscribing to /user_camera/image_raw, ...
[gesture_node]: gesture_node started: subscribing to /user_camera/image_raw ...
[yolo_node]: YOLOv8 loaded from yolov8n.pt
[fusion_node]: fusion_node: cone=25.0 deg, confirm gesture="grab"
[mode_manager]: mode_manager: start=NAVIGATION, switch_gesture="thumbs_up"
[teleop_bridge]: teleop_bridge: publishing on /cmd_vel ...
[arm_controller]: arm_controller ready: traj=/arm_controller/...
[nav_client]: nav_client connecting to /navigate_to_pose
[retrieval_orchestrator]: retrieval_orchestrator ready, known objects=[...]
```

> If `yolo_node` logs *"ultralytics not installed; yolo_node will publish
> empty detections"*, re-run `scripts/install_deps.sh`.

### 5.5 Wire the simulator camera into the perception nodes

The default `bringup.launch.py` runs `perception_node`, which publishes
your laptop's webcam onto `/user_camera/image_raw` (so the gaze and
gesture nodes always have a feed of *your* face).

For the **robot's** camera (used by YOLO), the easiest setup is to point
`yolo_node` at the Gazebo camera topic via the launch argument
`robot_camera_topic` (defined in `bringup.launch.py`):

`[Docker #4 — recommended]`
```bash
ros2 launch gaze_gesture_retrieval bringup.launch.py \
    robot_camera_topic:=/camera/image_raw
```

> NOTE: `ros2 launch` does **not** accept `--ros-args --remap ...` on the
> command line — that syntax is only valid for `ros2 run`. Use the
> launch argument above instead.

Alternatively, open a 5th shell and run a thin relay:

`[Docker #5]`
```bash
ros2 run topic_tools relay /camera/image_raw /robot_camera/image_raw
```

Or set `yolo_node.camera_topic` directly in `config/params.yaml` to
`/camera/image_raw` and rebuild.

### 5.6 Drive the robot — Navigation Control Mode

By default the system starts in **NAVIGATION** mode. With the camera
pointed at your face:

| Input                   | Effect                       |
| ----------------------- | ---------------------------- |
| Open palm               | base moves forward           |
| Closed fist             | base stops                   |
| Index pointing          | base moves backward          |
| Head turned left        | base yaws left (CCW)         |
| Head turned right       | base yaws right (CW)         |
| Thumbs up (held ~0.6 s) | toggle to **RETRIEVAL** mode |

Watch the published `/cmd_vel` to confirm:

`[Docker #5]`
```bash
ros2 topic echo /cmd_vel --once
```

### 5.7 Retrieve an object — Retrieval Mode

1. Hold the **thumbs-up** gesture for ~0.6 s. `mode_manager` logs
   `Mode switched -> RETRIEVAL` and starts publishing `RETRIEVAL` on
   `/system/mode`.
2. Look at one of the YOLO-detected objects in Gazebo. The
   `fusion_node` continuously publishes the best gaze-aligned
   candidate on `/fusion/candidate_label`:

   `[Docker #5]`
   ```bash
   ros2 topic echo /fusion/candidate_label
   ```
3. Make a closed-fist (`grab`) gesture. The fusion node locks the
   target:

   ```
   [fusion_node]: LOCKED target "bottle" @ pixel=(412.0, 248.0)
   ```
4. The orchestrator transitions through
   `APPROACH -> PRE_GRASP -> GRASP -> LIFT -> DELIVER -> RELEASE -> HOME`.
   You can watch every transition in the logs:

   ```
   [retrieval_orchestrator]: state IDLE -> APPROACH
   [nav_client]: Sending nav goal x=1.05, y=0.00
   [retrieval_orchestrator]: state APPROACH -> PRE_GRASP
   [arm_controller]: Sending gripper goal: pos=0.019
   [arm_controller]: Sending arm goal: [0.0, -0.3, 0.2, 0.6]
   ...
   [retrieval_orchestrator]: state HOME -> DONE
   ```
5. The world coordinates of each object live in
   `config/params.yaml -> retrieval_orchestrator.object_table`. Update
   them to match the layout in your Gazebo world (the same x/y you would
   use as a Nav2 goal in RViz).

### 5.8 Useful diagnostic commands

```bash
ros2 topic list
ros2 topic echo /gaze/state
ros2 topic echo /gesture/label
ros2 topic echo /perception/detections
ros2 topic echo /fusion/locked_target
ros2 topic echo /system/mode
ros2 topic echo /nav/status
ros2 topic echo /arm/command
ros2 node list
ros2 node info /retrieval_orchestrator
```

---

## 6. Run on the **physical TurtleBot3**

The procedure mirrors the simulation case, but `hardware.launch.py` does
**not** start a camera node — you have to publish frames yourself, the
same way Assignment 4 used a custom `chess_vision_publisher.py`. We
ship a tiny self-contained script for this in
[`jetson/jetson_camera_publisher.py`](../jetson/jetson_camera_publisher.py).

### 6.1 One-time setup on the Jetson

`[Jetson]`
```bash
sudo apt update
sudo apt install -y python3-opencv ros-humble-cv-bridge
sudo usermod -aG video $USER     # log out + back in once
```

Copy the publisher script over from the Remote PC (or git clone the
repo on the Jetson if it has internet):

`[Host or Docker]`
```bash
scp ~/my_code/testing_project/gaze_gesture_retrieval/jetson/jetson_camera_publisher.py \
    nvidia@<jetson-ip>:~/jetson_camera_publisher.py
```

### 6.2 Bring up the robot

`[Jetson — terminal A]`
```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

`[Jetson — terminal B]`
```bash
source /opt/ros/humble/setup.bash
python3 ~/jetson_camera_publisher.py
# adjust on the fly with --ros-args -p video_device:=/dev/video0 -p fps:=15.0
```

You should see something like:

```
[INFO] JetsonCameraPublisher: device=0 requested=640x480@15.0 ->
       actual=640x480@15.0 topic=/robot_camera/image_raw frame_id=robot_camera
```

### 6.3 Verify the camera link from the Remote PC

`[Docker]`
```bash
ros2 topic list | grep image
# /robot_camera/image_raw          <- from the Jetson
# /user_camera/image_raw           <- from perception_node (your webcam)

ros2 topic hz /robot_camera/image_raw
# average rate: ~15.0 Hz

ros2 run rqt_image_view rqt_image_view /robot_camera/image_raw   # optional eyeball check
```

If `Publisher count: 0` from the laptop while the Jetson clearly logs
that it's publishing, your `ROS_DOMAIN_ID` doesn't match across
machines — fix it before continuing (`printenv ROS_DOMAIN_ID` on both
sides must be identical).

### 6.4 Bring up Nav2, MoveIt, and our stack on the Remote PC

`[Docker #1]`
```bash
ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py \
    map_yaml_file:=$HOME/map.yaml
```

`[Docker #2]`
```bash
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
```

`[Docker #3]`
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch gaze_gesture_retrieval bringup.launch.py
```

Because the Jetson publisher already uses `/robot_camera/image_raw`
(the default `yolo_node.camera_topic`), **no launch override is needed**.
The laptop's webcam continues to feed `gaze_node` and `gesture_node`
through `perception_node` → `/user_camera/image_raw`.

> **Safety**: the `teleop_bridge` automatically zeroes `/cmd_vel` when
> the gesture stream stops for more than 1 s. If you walk away from the
> camera the robot stops on its own.

---

## 7. Saving a fresh map (if you skipped Assignment 1)

```bash
# Term 1
ros2 launch turtlebot3_manipulation_gazebo gazebo.launch.py
# Term 2
ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py
# Term 3
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
# Term 4
ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop
# After driving around, Term 5
ros2 run nav2_map_server map_saver_cli -f ~/map
```

`~/map.yaml` and `~/map.pgm` will appear in your container home directory
(persisted via the same volume mapping as `my_code`).

---

## 8. Permanent install (optional)

To avoid re-running `install_deps.sh` after every `docker compose down`,
edit `~/turtlebot_docker/Dockerfile` and append:

```dockerfile
RUN pip3 install --no-cache-dir \
        mediapipe==0.10.14 \
        opencv-python \
        ultralytics

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-transport \
        ros-humble-rqt-image-view \
        v4l-utils && \
    rm -rf /var/lib/apt/lists/*
```

Then:
```bash
cd ~/turtlebot_docker
HOST_UID=$(id -u) USER_HOME=$HOME docker compose up -d --build
```

---

## 9. Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| `gaze_node` says *"none"* forever | Camera permissions: re-run `xhost +local:root`; check `/dev/video0` is exposed in `docker-compose.yml`. |
| `AttributeError: _ARRAY_API not found` while importing matplotlib / mediapipe / cv2 | `pip` pulled NumPy 2.x but the system matplotlib/cv2 are compiled against NumPy 1.x. Fix: `pip3 install "numpy<2"` and re-run. (`scripts/install_deps.sh` already pins this.) |
| `Authorization required, but no authorization protocol specified` / `qt.qpa.xcb: could not connect to display :1` | X11 forwarding handshake failed. On the **host** terminal (outside Docker) run `xhost +local:root`. Inside the container check `echo $DISPLAY`; if empty, `export DISPLAY=:0`. |
| `colcon build` prints `ignoring unknown package 'gaze_gesture_retrieval'` | Your symlink in `~/ros2_ws/src/` points at the wrong directory. The package's `package.xml` must live directly inside the symlink target. Re-create with `ln -sfn ~/my_code/<correct-path>/gaze_gesture_retrieval ~/ros2_ws/src/gaze_gesture_retrieval`. |
| `ros2: error: unrecognized arguments: --ros-args --remap ...` when running `ros2 launch` | `--ros-args --remap` is only valid for `ros2 run`, not `ros2 launch`. Use the `robot_camera_topic:=/camera/image_raw` launch argument, or run a `topic_tools relay`. |
| `ros2 run topic_tools relay ...` says *"package not found"* | `topic_tools` isn't installed in the course image by default. `apt-get install -y ros-humble-topic-tools`. The current `scripts/install_deps.sh` already does this; if your container was created before that change, re-run it. The `cwd` you run `ros2 run` from is irrelevant — only sourcing matters. |
| MediaPipe import error | `pip3 install mediapipe==0.10.14`. Newer 0.10.x versions also work; pin the version that matches your CUDA/CPU. |
| YOLO logs "Failed to load yolov8n.pt" | First launch downloads the weights; ensure the container has internet. |
| Robot does not move in Gazebo | Verify `/cmd_vel` is being published (`ros2 topic hz /cmd_vel`) and that `/cmd_vel` has a subscriber (Gazebo's `diff_drive` plugin). |
| Nav2 "Goal rejected" | Set the initial pose with **2D Pose Estimate** in RViz before sending a target. |
| Arm doesn't move | Confirm `/arm_controller/follow_joint_trajectory` and `/gripper_controller/gripper_cmd` are advertised (they come from `servo.launch.py` + the simulation/hardware bringup). |
| `retrieval_orchestrator` warns *"Locked target X has no entry in object_table"* | Add an entry under `retrieval_orchestrator.object_table` in `config/params.yaml` with the object's world `x:y`. |

---

## 10. Demo script (~3-minute video)

A suggested narration order, mirroring the proposal's two interaction
modes:

1. *Show the system starting up* with `bringup.launch.py` and explain
   each window (Gazebo, RViz, terminal logs).
2. *Navigation Control Mode*: drive forward/backward/stop with hand
   gestures and steer with head pose.
3. *Mode switch*: thumbs-up gesture; show `/system/mode` flipping.
4. *Retrieval Mode*: gaze at a bottle, confirm with `grab`, watch the
   robot navigate, grasp, lift, deliver, release and return home.
5. *Parameter tweak*: change `fusion_node.gaze_cone_deg` from 25 to 5
   and re-run; show that only objects almost exactly under your gaze
   are now selected.
