# Architecture Notes

Compact reference for graders / reviewers.

## Modes of operation

The `mode_manager` node maintains exactly one of two states, broadcast on
`/system/mode`:

* **NAVIGATION** (default): hybrid hand-gesture + gaze driving. The
  `teleop_bridge` translates `(/gesture/label, /gaze/state)` into a
  `geometry_msgs/Twist` on `/cmd_vel`. The retrieval pipeline ignores
  locked targets in this mode, so accidentally fixating on an object does
  not cause the arm to swing.
* **RETRIEVAL**: gaze + gesture select an object detected by YOLOv8. The
  `retrieval_orchestrator` then drives the robot to it, performs the
  pick + lift + deliver + release sequence, and returns home.

Switching is triggered by holding a `thumbs_up` gesture for
`gesture_node.confirm_hold_s` seconds (default 0.6 s). The toggle gesture
is configurable via `mode_manager.mode_switch_gesture`.

## Multimodal fusion logic

`fusion_node` receives three streams:

1. `/gaze/yaw_deg` (smoothed head yaw, degrees).
2. `/perception/detections` (YOLOv8 2D bounding boxes from the robot
   camera).
3. `/gesture/stable` (gesture string after a stability window).

For every recent detection set the node:

1. Estimates the pixel column corresponding to the user's gaze:
   `target_px = W/2 + clip(yaw / cone, ±1) * (W/2)`
2. Picks the detection whose bbox centre is *horizontally* closest to
   `target_px` and publishes the candidate label on
   `/fusion/candidate_label`.
3. When the stable gesture matches `fusion_node.confirm_gesture` (default
   `grab`), it locks the candidate and publishes
   `/fusion/locked_target` + `/fusion/locked_pixel`.

This keeps the fusion logic decoupled from the choice of object detector
or gaze estimator — anything that can publish `Detection2DArray` and
`Float32` works.

## Retrieval state machine

```
IDLE --(locked target)--> APPROACH --(nav succeeded)--> PRE_GRASP
       --> GRASP --> LIFT --> DELIVER --(nav succeeded)--> RELEASE
       --> HOME --> DONE
```

* `APPROACH` and `DELIVER` publish `geometry_msgs/PoseStamped` to
  `/nav/goal`; the `nav_client` adapter forwards them to Nav2's
  `NavigateToPose` action server and republishes the action result on
  `/nav/status`.
* The arm transitions are sent as plain strings (`open`, `extend`,
  `close`, `carry`, `home`) on `/arm/command`; `arm_controller` turns
  them into `FollowJointTrajectory` and `GripperCommand` action goals
  using the joint poses defined in `config/params.yaml`.
* On `nav_client` reporting `failed`, the orchestrator skips ahead to
  `HOME` so the robot doesn't get stuck in a half-completed pick.

## Failure modes & safety

* The teleop bridge zeroes `/cmd_vel` if no gesture has arrived for 1 s.
* `arm_controller` waits up to 2 s for the action server before sending
  any goal — useful when the simulation takes a while to come up.
* All perception is rate-limited (`publish_rate_hz` parameters) so the
  Jetson / laptop can keep up.
* YOLO detections are filtered to a small whitelist of "graspable"
  COCO classes (bottle, cup, teddy bear, ...). Tune via
  `yolo_node.classes`.

## Relationship to the course's existing packages

* Uses `turtlebot3_manipulation_gazebo` for simulation.
* Uses `turtlebot3_manipulation_moveit_config` (`servo.launch.py`) to
  bring up the OMX joint and gripper controllers.
* Uses `turtlebot3_manipulation_navigation2` for Nav2 + RViz.
* Replaces the `turtlebot3_manipulation_teleop` keyboard demo with the
  multimodal `teleop_bridge` (NAVIGATION mode).

This means the project drops into the existing course Docker container
without modifying any course-provided source files; the only required
side-effect is the `pip3` / `apt` installs documented in
`scripts/install_deps.sh`.
