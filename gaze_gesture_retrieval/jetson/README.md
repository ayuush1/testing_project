# Jetson-side scripts

These scripts run **directly on the TurtleBot3's NVIDIA Jetson NX**, not
inside Docker. They are the equivalent of the publisher script you wrote
for Assignment 4 (`chess_vision_publisher.py`).

## What's here

| File | Purpose | Architecture |
| --- | --- | --- |
| `jetson_camera_publisher.py` | Open the camera with OpenCV (V4L2) and publish raw frames as `sensor_msgs/Image` on `/robot_camera/image_raw`. YOLO inference happens on the Remote PC. | "image streaming" |
| `jetson_yolo_json_publisher.py` | Run YOLO inference *on the Jetson* (CUDA / TensorRT) and publish detections as a tiny JSON `std_msgs/String` on `/yolo/detections_json`. The Remote PC's `yolo_json_bridge` translates that to `vision_msgs/Detection2DArray` for `fusion_node`. | "edge inference" — same pattern as Assignment 4 |

Pick one — never run both at the same time. **For the physical robot
on `small_blue_wifi` the edge-inference script is strongly recommended**
because it sends only a few KB/s instead of ~13 MB/s of raw images.

## One-time setup on the Jetson

```bash
sudo apt update
sudo apt install -y python3-opencv ros-humble-cv-bridge
# allow your user to access /dev/videoN:
sudo usermod -aG video $USER
# log out + back in (or reboot once) for the group change to take effect.
```

If `ros-humble-cv-bridge` isn't available, the publisher will fall back
to a manual `bgr8` encoder so it still works.

## Copy the script onto the Jetson

You can either `scp` it over from the Remote PC:

```bash
# from inside the Docker container or on the host:
scp ~/my_code/testing_project/gaze_gesture_retrieval/jetson/jetson_camera_publisher.py \
    nvidia@<jetson-ip>:~/jetson_camera_publisher.py
```

…or pull the repo directly on the Jetson if it has internet:

```bash
cd ~
git clone https://github.com/<your-fork>/testing_project.git
cp testing_project/gaze_gesture_retrieval/jetson/jetson_camera_publisher.py ~/
```

## Running it — Architecture A (image streaming)

```bash
source /opt/ros/humble/setup.bash
python3 ~/jetson_camera_publisher.py
```

You should see a log line like:

```
[INFO] JetsonCameraPublisher: device=0 requested=640x480@15.0 -> actual=640x480@15.0 topic=/robot_camera/image_raw frame_id=robot_camera cv_bridge=yes
```

On the Remote PC:

```bash
ros2 launch gaze_gesture_retrieval bringup.launch.py    # default: yolo_source:=local
```

## Running it — Architecture B (edge inference, recommended)

This is the Assignment 4 pattern. Install ultralytics on the Jetson once:

```bash
pip3 install ultralytics
```

Then run the JSON publisher:

```bash
source /opt/ros/humble/setup.bash
python3 ~/jetson_yolo_json_publisher.py
# CUDA model:        --ros-args -p model:=yolo11n.pt
# TensorRT engine:   --ros-args -p model:=yolo11n.engine
# USB camera (instead of CSI / Pi camera):
#                    --ros-args -p camera_source:=v4l2 -p video_device:=/dev/video0
```

Expected log line:

```
[INFO] YoloJsonPublisher ready: source=CSI (gstreamer/nvarguscamerasrc),
       topic=/yolo/detections_json, conf=0.4, rate=10.0 Hz
```

On the Remote PC, switch the launch argument to `remote`:

```bash
ros2 launch gaze_gesture_retrieval bringup.launch.py yolo_source:=remote
```

The `yolo_json_bridge` node will subscribe to `/yolo/detections_json`,
parse each JSON message, and republish a `vision_msgs/Detection2DArray`
on `/perception/detections` — exactly the topic `fusion_node` already
consumes, so nothing else changes.

Verify:

```bash
ros2 topic hz /yolo/detections_json
ros2 topic echo /perception/detections --once
```

## Tunable parameters

Pass overrides on the command line just like you would for any ROS 2
node:

```bash
python3 ~/jetson_camera_publisher.py --ros-args \
    -p video_device:=/dev/video0 \
    -p width:=640 \
    -p height:=480 \
    -p fps:=15.0 \
    -p topic:=/robot_camera/image_raw \
    -p frame_id:=robot_camera
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| `video_device` | `0` | Either an integer index (`0`, `1`, ...) or a path (`/dev/video0`). |
| `width` / `height` | `640` / `480` | Frame size — keep small to save bandwidth on `small_blue_wifi`. |
| `fps` | `15.0` | Frame rate. YOLOv8 doesn't need 30. |
| `topic` | `/robot_camera/image_raw` | Topic name; must match `yolo_node.camera_topic` on the Remote PC (default already matches). |
| `frame_id` | `robot_camera` | Stamped on every `Image` message. |

## Verify from the Remote PC

In a Docker shell on the laptop:

```bash
ros2 topic list | grep image
# /robot_camera/image_raw

ros2 topic info -v /robot_camera/image_raw
# Publisher count: 1   <- the Jetson

ros2 topic hz /robot_camera/image_raw
# average rate: ~15.0
```

Visual check:

```bash
ros2 run rqt_image_view rqt_image_view /robot_camera/image_raw
```

Then launch the rest of the stack normally:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch gaze_gesture_retrieval bringup.launch.py
```

`yolo_node` will pick up the Jetson camera frames automatically (no
launch override needed because the topic names match).

## Keeping it running

The Jetson is **not** Docker, so files you create persist across
reboots. The running process itself does not — close the terminal and
the publisher dies. Two convenient options:

1. Run the publisher inside `tmux` so you can detach and reattach:
   ```bash
   tmux new -s cam
   source /opt/ros/humble/setup.bash
   python3 ~/jetson_camera_publisher.py
   # detach with Ctrl-B then D; reattach later with `tmux attach -t cam`
   ```

2. (Advanced) Make a systemd unit so it auto-starts at boot. See the
   "Step 6 — Persist the Jetson camera publisher" section of
   `docs/RUNNING.md`.
