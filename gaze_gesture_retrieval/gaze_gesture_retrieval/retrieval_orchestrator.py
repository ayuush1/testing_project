"""Visual-servo retrieval state machine.

This is the engine that runs after the user locks a target with their
gaze + ``peace`` gesture in RETRIEVAL mode. Inspired directly by the
``BottleAutonomyController`` flow used on the physical robot.

State machine
-------------

    IDLE -- /fusion/locked_target --> ALIGN
    SEARCH       spin in place until the target reappears
    ALIGN        spin in place to centre the target horizontally
    APPROACH     drive forward with proportional yaw correction until
                 the target's bbox is large enough (i.e. close enough)
    PICK         scripted arm sequence:
                     home -> open -> final_grab -> close -> home
                 each step has its own duration; non-blocking ticks
    RETURN_TURN  spin for ~return_turn_time_s seconds
    RETURN_DRIVE drive backward for ~return_drive_time_s seconds
    PLACE        scripted arm sequence:
                     place -> open -> home
    DONE

Subscribes
----------
    /fusion/locked_target            std_msgs/String
    /perception/detections           vision_msgs/Detection2DArray
    /system/mode                     std_msgs/String

Publishes
---------
    /cmd_vel                         geometry_msgs/Twist  (during the
                                     visual-servo phases)
    /arm/command                     std_msgs/String      (sent to the
                                     arm_controller)
    /retrieval/state                 std_msgs/String      (current state name)

Optional Nav2 path (``approach_mode:=nav2``) keeps the original
``/nav/goal`` behaviour using a static ``object_table``. The default is
``approach_mode:=visual_servo``, which works without a calibrated map.
"""
from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped, Twist
from vision_msgs.msg import Detection2DArray

from .utils.geometry import quaternion_from_yaw, project_pose_in_front


class State(Enum):
    IDLE = auto()
    SEARCH = auto()
    ALIGN = auto()
    APPROACH = auto()
    PICK = auto()
    RETURN_TURN = auto()
    RETURN_DRIVE = auto()
    PLACE = auto()
    DONE = auto()


# Macro arm sequences. Each entry is (action_kind, value, duration_s).
# action_kind: 'arm' -> publish a pose name on /arm/command
#              'gripper' -> publish 'open'/'close' on /arm/command
PICK_SEQUENCE: List[Tuple[str, str, float]] = [
    ('arm',      'home',       1.6),
    ('gripper',  'open',       0.6),
    ('arm',      'final_grab', 1.8),
    ('gripper',  'close',      0.8),
    ('arm',      'home',       1.9),
]

PLACE_SEQUENCE: List[Tuple[str, str, float]] = [
    ('arm',      'place',      1.8),
    ('gripper',  'open',       0.8),
    ('arm',      'home',       1.9),
]


class RetrievalOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('retrieval_orchestrator')

        # ----- visual servo gains / thresholds ---------------------------
        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('center_tolerance_px', 70.0)
        self.declare_parameter('target_bbox_width_px', 220.0)
        self.declare_parameter('bbox_width_tolerance_px', 20.0)
        self.declare_parameter('search_turn_speed', 0.32)
        self.declare_parameter('max_turn_speed', 0.70)
        self.declare_parameter('max_forward_speed', 0.18)
        self.declare_parameter('kp_turn', 0.0025)
        self.declare_parameter('kp_forward', 0.0025)
        self.declare_parameter('detection_timeout_s', 0.7)

        # ----- return-to-base timings ------------------------------------
        self.declare_parameter('return_turn_time_s', 4.5)
        self.declare_parameter('return_drive_time_s', 2.5)
        self.declare_parameter('return_drive_speed', -0.15)

        # ----- mode of operation -----------------------------------------
        self.declare_parameter('approach_mode', 'visual_servo')   # or 'nav2'
        self.declare_parameter('do_return_after_pick', True)

        # ----- legacy Nav2 path (still available) ------------------------
        self.declare_parameter('approach_offset', 0.45)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter(
            'object_table',
            ['bottle:1.5:0.0', 'cup:0.8:1.0', 'teddy bear:1.2:-0.8'],
        )
        self.declare_parameter('user_pose', [0.0, 0.0, 0.0])

        self._image_w = float(self.get_parameter('image_width').value)
        self._center_tol = float(self.get_parameter('center_tolerance_px').value)
        self._target_w = float(self.get_parameter('target_bbox_width_px').value)
        self._w_tol = float(self.get_parameter('bbox_width_tolerance_px').value)
        self._search_turn = float(self.get_parameter('search_turn_speed').value)
        self._max_turn = float(self.get_parameter('max_turn_speed').value)
        self._max_fwd = float(self.get_parameter('max_forward_speed').value)
        self._kp_turn = float(self.get_parameter('kp_turn').value)
        self._kp_fwd = float(self.get_parameter('kp_forward').value)
        self._det_timeout = float(self.get_parameter('detection_timeout_s').value)
        self._ret_turn_t = float(self.get_parameter('return_turn_time_s').value)
        self._ret_drive_t = float(self.get_parameter('return_drive_time_s').value)
        self._ret_drive_v = float(self.get_parameter('return_drive_speed').value)
        self._approach_mode = str(self.get_parameter('approach_mode').value).lower()
        self._do_return = bool(self.get_parameter('do_return_after_pick').value)
        self._offset = float(self.get_parameter('approach_offset').value)
        self._map = str(self.get_parameter('map_frame').value)
        self._user = list(self.get_parameter('user_pose').value)
        self._objects = self._parse_objects(
            list(self.get_parameter('object_table').value)
        )

        # Runtime state
        self._state = State.IDLE
        self._mode = 'NAVIGATION'
        self._target_label: Optional[str] = None
        self._t_state_entered = time.monotonic()
        self._last_bbox: Optional[Tuple[float, float, float, float]] = None
        self._last_bbox_t = 0.0
        self._last_good_w: Optional[float] = None
        # Sub-state for arm sequences
        self._seq_idx = 0
        self._seq_step_started = 0.0
        self._seq_step_sent = False
        self._nav_sent = False

        self.create_subscription(String, '/system/mode', self._on_mode, 10)
        self.create_subscription(String, '/fusion/locked_target', self._on_target, 10)
        self.create_subscription(
            Detection2DArray, '/perception/detections', self._on_detections, 10
        )

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_arm = self.create_publisher(String, '/arm/command', 10)
        self._pub_state = self.create_publisher(String, '/retrieval/state', 10)
        # Legacy Nav2 path
        self._pub_goal = self.create_publisher(PoseStamped, '/nav/goal', 10)
        self.create_subscription(String, '/nav/status', self._on_nav_status, 10)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'retrieval_orchestrator ready: approach_mode={self._approach_mode}, '
            f'do_return={self._do_return}, '
            f'image_width={self._image_w}, target_bbox_w={self._target_w}'
        )

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def _parse_objects(tokens) -> Dict[str, Tuple[float, float]]:
        out: Dict[str, Tuple[float, float]] = {}
        for tok in tokens:
            try:
                parts = tok.split(':')
                if len(parts) < 3:
                    continue
                label = ':'.join(parts[:-2]).strip()
                x = float(parts[-2])
                y = float(parts[-1])
                out[label] = (x, y)
            except Exception:
                continue
        return out

    def _enter(self, new: State) -> None:
        if new == self._state:
            return
        self.get_logger().info(f'state {self._state.name} -> {new.name}')
        self._state = new
        self._t_state_entered = time.monotonic()
        self._seq_idx = 0
        self._seq_step_started = time.monotonic()
        self._seq_step_sent = False
        self._nav_sent = False
        self._publish_state()

    def _publish_state(self) -> None:
        self._pub_state.publish(String(data=self._state.name))

    def _publish_twist(self, lin: float = 0.0, ang: float = 0.0) -> None:
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self._pub_cmd.publish(t)

    def _bbox_visible(self) -> bool:
        return (
            self._last_bbox is not None
            and (time.monotonic() - self._last_bbox_t) < self._det_timeout
        )

    # ----------------------------------------------------------------- subs
    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_detections(self, msg: Detection2DArray) -> None:
        if self._target_label is None:
            return
        # Pick the highest-confidence detection that matches the locked label.
        best = None
        best_score = -1.0
        for d in msg.detections:
            if not d.results:
                continue
            label = d.results[0].hypothesis.class_id
            score = d.results[0].hypothesis.score
            if label != self._target_label:
                continue
            if score > best_score:
                best_score = score
                best = d
        if best is None:
            return
        cx = best.bbox.center.position.x
        cy = best.bbox.center.position.y
        bw = best.bbox.size_x
        bh = best.bbox.size_y
        self._last_bbox = (cx, cy, bw, bh)
        self._last_bbox_t = time.monotonic()
        self._last_good_w = bw

    def _on_target(self, msg: String) -> None:
        if self._mode != 'RETRIEVAL':
            self.get_logger().info(
                f'Ignoring target "{msg.data}" because mode={self._mode}')
            return
        if self._state not in (State.IDLE, State.DONE):
            self.get_logger().warning(
                f'Already executing retrieval (state={self._state.name})')
            return
        self._target_label = msg.data
        # Start the visual servo. If Nav2 mode is configured and the
        # object is in the table, jump to the legacy approach instead.
        if self._approach_mode == 'nav2' and msg.data in self._objects:
            self._enter(State.APPROACH)
            return
        # Skip SEARCH if we already have a fresh detection of the target
        # (the user just looked at it, so it's almost certainly visible).
        if self._bbox_visible():
            self._enter(State.ALIGN)
        else:
            self._enter(State.SEARCH)

    def _on_nav_status(self, msg: String) -> None:
        if self._approach_mode != 'nav2':
            return
        s = msg.data
        if s == 'succeeded' and self._state == State.APPROACH:
            self._enter(State.PICK)
        elif s == 'failed' and self._state == State.APPROACH:
            self.get_logger().error(
                'Nav2 failed in APPROACH; aborting back to home')
            self._enter(State.PICK)   # arm-only fallback so the robot still
                                       # demonstrates the pick sequence

    # ----------------------------------------------------------------- arm seq
    def _run_sequence(self, sequence: List[Tuple[str, str, float]],
                      done_state: State) -> None:
        if self._seq_idx >= len(sequence):
            self._enter(done_state)
            return
        kind, name, duration = sequence[self._seq_idx]
        elapsed = time.monotonic() - self._seq_step_started
        if not self._seq_step_sent:
            self._pub_arm.publish(String(data=name))
            self.get_logger().info(
                f'  {self._state.name} step {self._seq_idx + 1}/{len(sequence)}: '
                f'{kind}={name} (t<{duration:.1f}s)'
            )
            self._seq_step_sent = True
        if elapsed >= duration:
            self._seq_idx += 1
            self._seq_step_started = time.monotonic()
            self._seq_step_sent = False

    # ----------------------------------------------------------------- tick
    def _tick(self) -> None:
        # Heartbeat the state for the overlay / diagnostics.
        self._publish_state()
        s = self._state
        if s in (State.IDLE, State.DONE):
            return

        # ------- legacy Nav2 mode --------------------------------------
        if self._approach_mode == 'nav2' and s == State.APPROACH:
            if not self._nav_sent and self._target_label in self._objects:
                obj_xy = self._objects[self._target_label]
                x, y, yaw = project_pose_in_front(obj_xy, (0.0, 0.0), self._offset)
                ps = PoseStamped()
                ps.header.frame_id = self._map
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = x
                ps.pose.position.y = y
                qx, qy, qz, qw = quaternion_from_yaw(yaw)
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                self._pub_goal.publish(ps)
                self._nav_sent = True
            return

        # ------- visual-servo states -----------------------------------
        if s == State.SEARCH:
            if self._bbox_visible():
                self._publish_twist(0.0, 0.0)
                self._enter(State.ALIGN)
            else:
                self._publish_twist(0.0, self._search_turn)
            return

        if s == State.ALIGN:
            if not self._bbox_visible():
                self._enter(State.SEARCH)
                return
            cx, _cy, bw, _bh = self._last_bbox
            error_x = cx - self._image_w / 2.0
            if abs(error_x) <= self._center_tol:
                self._publish_twist(0.0, 0.0)
                self._enter(State.APPROACH)
                return
            turn = max(-self._max_turn,
                       min(self._max_turn, -self._kp_turn * error_x))
            self._publish_twist(0.0, turn)
            return

        if s == State.APPROACH:
            if not self._bbox_visible():
                # If we lost sight while close enough, jump straight to the
                # pick. Otherwise go back to searching.
                if (
                    self._last_good_w is not None
                    and self._last_good_w >= (self._target_w - self._w_tol)
                ):
                    self.get_logger().info(
                        f'Target lost while close (w={self._last_good_w:.1f}); '
                        f'committing to PICK')
                    self._publish_twist(0.0, 0.0)
                    self._enter(State.PICK)
                else:
                    self._enter(State.SEARCH)
                return
            cx, _cy, bw, _bh = self._last_bbox
            error_x = cx - self._image_w / 2.0
            width_error = self._target_w - bw

            centred = abs(error_x) <= self._center_tol
            close = abs(width_error) <= self._w_tol or bw >= self._target_w

            if centred and close:
                self._publish_twist(0.0, 0.0)
                self._enter(State.PICK)
                return

            turn = max(-self._max_turn,
                       min(self._max_turn, -self._kp_turn * error_x))
            fwd = max(0.0, min(self._max_fwd, self._kp_fwd * width_error))
            if abs(turn) > 0.35:
                fwd *= 0.65
            self._publish_twist(fwd, turn)
            return

        if s == State.PICK:
            self._publish_twist(0.0, 0.0)
            done = State.RETURN_TURN if self._do_return else State.DONE
            self._run_sequence(PICK_SEQUENCE, done)
            return

        if s == State.RETURN_TURN:
            elapsed = time.monotonic() - self._t_state_entered
            if elapsed < self._ret_turn_t:
                self._publish_twist(0.0, self._search_turn)
            else:
                self._publish_twist(0.0, 0.0)
                self._enter(State.RETURN_DRIVE)
            return

        if s == State.RETURN_DRIVE:
            elapsed = time.monotonic() - self._t_state_entered
            if elapsed < self._ret_drive_t:
                self._publish_twist(self._ret_drive_v, 0.0)
            else:
                self._publish_twist(0.0, 0.0)
                self._enter(State.PLACE)
            return

        if s == State.PLACE:
            self._publish_twist(0.0, 0.0)
            self._run_sequence(PLACE_SEQUENCE, State.DONE)
            return


def main(args=None):
    rclpy.init(args=args)
    node = RetrievalOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort safety stop on shutdown.
        try:
            node._publish_twist(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
