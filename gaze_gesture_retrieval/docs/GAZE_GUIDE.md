# Gaze Guide

Everything you need to know to use, tune, and demo the gaze pipeline.

## 1. What "gaze" actually means in this project

The gaze input is computed from your laptop's webcam in two parts that
are **summed together** before being used:

| Component | Source | Range | Role |
| --- | --- | --- | --- |
| Head yaw (and pitch) | MediaPipe FaceMesh + OpenCV `solvePnP` on six face landmarks | ±~45° | Big, robust motion |
| Iris yaw (and pitch) | MediaPipe FaceMesh `refine_landmarks` iris keypoints | ±`iris_yaw_range_deg` (default 18°) | Fine cursor |

The combined yaw is published on `/gaze/yaw_deg`; the two contributions
are also published separately on `/gaze/head_yaw_deg` and
`/gaze/iris_yaw_deg` for diagnostics. There is no per-user calibration
step — the iris contribution is normalised by the eye-socket width so it
adapts automatically.

## 2. How the gaze drives the system

### 2.1 In NAVIGATION mode (driving the robot)

The yaw is bucketed into three states by `gaze_node`:

```
yaw < yaw_left_deg     ->  /gaze/state = "left"
yaw > yaw_right_deg    ->  /gaze/state = "right"
otherwise              ->  /gaze/state = "center"
```

`teleop_bridge` reads the state and applies an angular velocity:

```
"left"  -> +angular_speed
"right" -> -angular_speed
"center"->  0
```

So **head turn = robot turn**. The dead-zone in the centre (`yaw_left_deg`
to `yaw_right_deg`, default ±12°) prevents accidental yaw from jittering
the base.

### 2.2 In RETRIEVAL mode (picking a target)

`fusion_node` interprets the same yaw as a *horizontal cursor* on the
robot's camera image:

```
target_pixel_x = image_centre + clip(yaw / gaze_cone_deg, ±1) * (image_width / 2)
```

Then it finds the YOLO bounding box whose centre is closest to that
column and publishes its label on `/fusion/candidate_label`. When you
make the confirm gesture (default `grab`), the candidate is locked.

* Want the cursor to be more **sensitive** → decrease `gaze_cone_deg`.
* Want the cursor to be more **forgiving** → increase `gaze_cone_deg`.

`gaze_cone_deg` lives under `fusion_node` in `config/params.yaml` and
defaults to 25°.

## 3. Watching the gaze in real time

There are three convenient surfaces.

### 3.1 The HUD overlay (recommended for telepresence)

Launch with the overlay enabled (default in `bringup.launch.py`):

```bash
ros2 launch gaze_gesture_retrieval bringup.launch.py
```

You get a window titled **"gaze overlay"** with:

* The robot camera (or a black canvas if only JSON detections are coming
  in).
* All YOLO bounding boxes drawn in white.
* A **red vertical line + dot** = the gaze cursor.
* The **green** box = current candidate (the one your gaze is on).
* A **cyan** box = the locked target (for ~4 s after locking).
* Status bar:
  ```
  mode=RETRIEVAL  yaw=+12.4 (head=+8.1, iris=+4.3)  cand=bottle  lock=bottle
  ```

If the iris part of the yaw stays at ~0 even when you move your eyes,
your iris isn't being detected — see the troubleshooting list below.

### 3.2 Topic-level monitoring

```bash
ros2 topic echo /gaze/yaw_deg          # combined
ros2 topic echo /gaze/head_yaw_deg     # head only
ros2 topic echo /gaze/iris_yaw_deg     # iris only
ros2 topic echo /gaze/state            # left | right | center | none
ros2 topic echo /fusion/candidate_label
ros2 topic echo /fusion/locked_target
```

### 3.3 Offline test (no ROS, no robot)

```bash
cd ~/my_code/testing_project/gaze_gesture_retrieval
python3 scripts/test_perception_offline.py
```

A window pops up with the inferred yaw, pitch, and current gesture label
overlaid on your webcam.

## 4. Practical workflow for retrieval

The order matters. Always do this:

1. **Switch to RETRIEVAL** with thumbs-up. Wait for the status bar to
   show `mode=RETRIEVAL`.
2. **Aim the gaze cursor** by combining a head turn and a small eye
   movement. Watch the cursor (red line) slide across the boxes; the
   one closest to it goes **green** and `cand=` updates.
3. **Confirm** with a closed-fist (`grab`) gesture. Hold ~0.6 s. The
   box turns **cyan**, the log says `LOCKED target "<name>"`, and the
   orchestrator starts driving the robot.

If you grab while the cursor is *not* over any box, you'll get
`Got "grab" but no gaze-aligned candidate to lock` — that's the system
correctly refusing to act on an empty cursor.

If you grab while in NAVIGATION mode, you'll get
`Ignoring target "<name>" because mode=NAVIGATION`. Switch first.

## 5. Tuning

All knobs are in `config/params.yaml`. Edit, then:

```bash
cd ~/ros2_ws
colcon build --packages-select gaze_gesture_retrieval
source install/setup.bash
# relaunch
```

| Parameter | Default | Effect |
| --- | --- | --- |
| `gaze_node.use_iris` | `true` | Master switch for iris-based fine cursor. |
| `gaze_node.iris_yaw_range_deg` | `18.0` | More = eyes contribute a bigger swing. |
| `gaze_node.iris_pitch_range_deg` | `12.0` | Vertical contribution (currently unused by fusion, available on `/gaze/pitch_deg`). |
| `gaze_node.iris_deadzone` | `0.07` | Ignore iris offsets below this fraction of an eye-width — kills saccade twitches. |
| `gaze_node.smoothing_alpha` | `0.4` | Larger = more responsive, more jittery. |
| `gaze_node.yaw_left_deg` | `-12.0` | NAVIGATION dead-zone (left edge). |
| `gaze_node.yaw_right_deg` | `12.0` | NAVIGATION dead-zone (right edge). |
| `fusion_node.gaze_cone_deg` | `25.0` | Half-angle of the gaze cone in RETRIEVAL. Smaller = you must look almost exactly at the box. |

### Suggested presets

* **Mostly head, very little eye:** `iris_yaw_range_deg: 6.0`, `iris_deadzone: 0.20`. Eyes serve as a tiny fine-tune; works best if you wear glasses.
* **Eyes-driven cursor:** `iris_yaw_range_deg: 25.0`, `iris_deadzone: 0.04`. You can keep your head still and just look at the right object. Requires a clean, well-lit face.
* **Fully head pose (no iris)**: set `use_iris: false`. This is what the original commit did. Most robust under bad lighting.

## 6. Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| `iris_yaw_deg` stays at 0 even when you look around | MediaPipe FaceMesh refine_landmarks is off. Check `gaze_node.use_iris` is `true`. Also make sure your `mediapipe` is ≥ 0.10 (`pip3 install -U "mediapipe>=0.10.14"`). |
| Iris yaw value flickers wildly when the head is still | Lighting is too low or contrast is too high; FaceMesh's iris keypoints are noisy. Increase `iris_deadzone` to 0.1+ or lower `iris_yaw_range_deg`. |
| Cursor drifts off-centre when looking straight | Webcam is mounted off-centre relative to your face. Either physically recenter, or accept the offset (your `head_yaw_deg` in the status bar will show it). |
| Robot doesn't yaw in NAVIGATION mode | Your gaze yaw is inside the dead-zone. Decrease `yaw_left_deg` magnitude (e.g. `-7.0`) and/or `yaw_right_deg` (e.g. `7.0`). |
| Wrong YOLO box keeps becoming the candidate | Decrease `fusion_node.gaze_cone_deg` to make the cursor more sensitive. |
| `Got "grab" but no gaze-aligned candidate to lock` | The cursor is over empty space at the moment of grabbing, or YOLO didn't have any detections in the last 2 s. Check the green highlight in the overlay before grabbing. |
| `Ignoring target "X" because mode=NAVIGATION` | Switch to RETRIEVAL first (thumbs-up) and then grab. |

## 7. Why head + iris instead of pure eye gaze?

Pure screen-space eye trackers require per-user, per-session
calibration to map iris position to laptop-screen pixels. They also
break easily when you move your head, wear glasses, or turn off lights.

Head + iris is a good engineering compromise:

* **Head** does the big motions; it's robust and needs no calibration.
* **Iris** does the small adjustments; it's much faster than turning
  the head, and noisy iris frames just contribute small jitter to an
  already-stable head yaw rather than ruining the estimate.

The proposal explicitly allows either head pose or eye-tracking;
combining them gets the best of both.
