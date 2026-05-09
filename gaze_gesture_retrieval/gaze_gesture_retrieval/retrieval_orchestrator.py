"""High level retrieval state machine.

Runs only when ``/system/mode == 'RETRIEVAL'``. After a target is locked by
``fusion_node`` the orchestrator drives the following sequence:

    APPROACH      -> publish a Nav2 goal in front of the object
    PRE_GRASP     -> arm to ``extend`` pose, gripper open
    GRASP         -> close gripper
    LIFT          -> arm to ``carry`` pose
    DELIVER       -> publish a Nav2 goal back to the user pose (configurable)
    RELEASE       -> arm to ``extend``, gripper open
    HOME          -> arm to ``home`` pose
    DONE

Object world coordinates are taken from a simple lookup table loaded as a
parameter (string of "label:x:y" entries). When running in simulation, fill
this table with locations from your saved Gazebo world; on the physical
robot, replace this lookup with a depth-aware projection of the locked
pixel into the map frame.
"""
from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped

from .utils.geometry import quaternion_from_yaw, project_pose_in_front


class State(Enum):
    IDLE = auto()
    APPROACH = auto()
    PRE_GRASP = auto()
    GRASP = auto()
    LIFT = auto()
    DELIVER = auto()
    RELEASE = auto()
    HOME = auto()
    DONE = auto()


class RetrievalOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('retrieval_orchestrator')

        self.declare_parameter('approach_offset', 0.45)
        self.declare_parameter('map_frame', 'map')
        # "label:x:y" tokens describing known object world poses.
        self.declare_parameter(
            'object_table',
            ['bottle:1.5:0.0', 'cup:0.8:1.0', 'teddy bear:1.2:-0.8'],
        )
        self.declare_parameter('user_pose', [0.0, 0.0, 0.0])  # x, y, yaw

        self._offset = float(self.get_parameter('approach_offset').value)
        self._map = str(self.get_parameter('map_frame').value)
        self._user = list(self.get_parameter('user_pose').value)
        self._objects = self._parse_objects(
            list(self.get_parameter('object_table').value)
        )

        self._state = State.IDLE
        self._mode = 'NAVIGATION'
        self._target_label: Optional[str] = None
        self._robot_xy: Tuple[float, float] = (0.0, 0.0)
        self._t_state_entered = time.monotonic()
        # Set of action keywords ("home", "open", "extend", ...) already
        # published for the *current* state so we don't spam the arm
        # controller with duplicates 5 times a second.
        self._actions_sent: set[str] = set()
        # Whether the navigation goal has already been published for the
        # current APPROACH or DELIVER state.
        self._nav_sent = False

        self.create_subscription(String, '/system/mode', self._on_mode, 10)
        self.create_subscription(String, '/fusion/locked_target', self._on_target, 10)
        self.create_subscription(Point, '/fusion/locked_pixel', lambda _msg: None, 10)
        self.create_subscription(String, '/nav/status', self._on_nav_status, 10)

        self._pub_arm = self.create_publisher(String, '/arm/command', 10)
        self._pub_goal = self.create_publisher(PoseStamped, '/nav/goal', 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'retrieval_orchestrator ready, known objects={list(self._objects)}'
        )

    # --------------------------------------------------------------- helpers
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
        self.get_logger().info(f'state {self._state.name} -> {new.name}')
        self._state = new
        self._t_state_entered = time.monotonic()
        self._actions_sent.clear()
        self._nav_sent = False

    def _send_arm_once(self, command: str) -> None:
        """Publish an /arm/command keyword at most once per state entry."""
        if command in self._actions_sent:
            return
        self._actions_sent.add(command)
        self._pub_arm.publish(String(data=command))

    def _send_pose(self, x: float, y: float, yaw: float) -> None:
        ps = PoseStamped()
        ps.header.frame_id = self._map
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self._pub_goal.publish(ps)

    # ----------------------------------------------------------------- subs
    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_target(self, msg: String) -> None:
        if self._mode != 'RETRIEVAL':
            self.get_logger().info(
                f'Ignoring target "{msg.data}" because mode={self._mode}')
            return
        if msg.data not in self._objects:
            self.get_logger().warning(
                f'Locked target "{msg.data}" has no entry in object_table')
            return
        if self._state not in (State.IDLE, State.DONE):
            self.get_logger().warning(
                f'Already executing retrieval (state={self._state.name})')
            return
        self._target_label = msg.data
        self._enter(State.APPROACH)

    def _on_nav_status(self, msg: String) -> None:
        s = msg.data
        if s == 'succeeded':
            if self._state == State.APPROACH:
                self._enter(State.PRE_GRASP)
            elif self._state == State.DELIVER:
                self._enter(State.RELEASE)
        elif s == 'failed' and self._state in (State.APPROACH, State.DELIVER):
            self.get_logger().error(
                f'Nav2 failed in {self._state.name}; aborting')
            self._enter(State.HOME)

    # ------------------------------------------------------------------ tick
    def _tick(self) -> None:
        s = self._state
        elapsed = time.monotonic() - self._t_state_entered
        if s == State.IDLE or s == State.DONE:
            return
        if s == State.APPROACH:
            if self._target_label is None:
                self._enter(State.IDLE)
                return
            if not self._nav_sent:
                obj_xy = self._objects[self._target_label]
                x, y, yaw = project_pose_in_front(obj_xy, self._robot_xy, self._offset)
                self._send_pose(x, y, yaw)
                self._nav_sent = True
            return
        if s == State.PRE_GRASP:
            self._send_arm_once('open')
            self._send_arm_once('extend')
            if elapsed > 3.0:
                self._enter(State.GRASP)
            return
        if s == State.GRASP:
            self._send_arm_once('close')
            if elapsed > 2.0:
                self._enter(State.LIFT)
            return
        if s == State.LIFT:
            self._send_arm_once('carry')
            if elapsed > 3.0:
                self._enter(State.DELIVER)
            return
        if s == State.DELIVER:
            if not self._nav_sent:
                self._send_pose(self._user[0], self._user[1], self._user[2])
                self._nav_sent = True
            return
        if s == State.RELEASE:
            self._send_arm_once('extend')
            self._send_arm_once('open')
            if elapsed > 3.0:
                self._enter(State.HOME)
            return
        if s == State.HOME:
            self._send_arm_once('home')
            if elapsed > 3.0:
                self._enter(State.DONE)
            return


def main(args=None):
    rclpy.init(args=args)
    node = RetrievalOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
