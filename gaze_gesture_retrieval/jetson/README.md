# Jetson-side scripts

These scripts run **directly on the TurtleBot3's NVIDIA Jetson NX**, not
inside Docker. They are the equivalent of the publisher script you wrote
for Assignment 4 (`chess_vision_publisher.py`).

## What's here

| File | Purpose |
| --- | --- |
| `jetson_camera_publisher.py` | Open the USB camera with OpenCV and publish frames as `sensor_msgs/Image` on `/robot_camera/image_raw`. |

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

## Running it

```bash
source /opt/ros/humble/setup.bash
python3 ~/jetson_camera_publisher.py
```

You should see a log line like:

```
[INFO] JetsonCameraPublisher: device=0 requested=640x480@15.0 -> actual=640x480@15.0 topic=/robot_camera/image_raw frame_id=robot_camera cv_bridge=yes
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
