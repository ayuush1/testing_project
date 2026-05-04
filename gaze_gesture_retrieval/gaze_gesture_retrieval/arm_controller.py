"""Arm + gripper controller for the OpenMANIPULATOR-X.

Exposes a string-based command topic ``/arm/command`` that accepts the
following commands:

    home        -> move to the home pose
    extend      -> extend forward (pre-grasp)
    carry       -> tucked carrying pose
    wave        -> friendly wave pose
    open        -> open the gripper
    close       -> close the gripper

The poses are loaded from the parameter server (see ``config/params.yaml``).

Internally the node uses two ROS 2 actions:
    /arm_controller/follow_joint_trajectory   (control_msgs/FollowJointTrajectory)
    /gripper_controller/gripper_cmd           (control_msgs/GripperCommand)
"""
from __future__ import annotations

from typing import Iterable, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand

ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']


class ArmController(Node):
    def __init__(self) -> None:
        super().__init__('arm_controller')

        self.declare_parameter(
            'joint_trajectory_action', '/arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('move_time_s', 2.5)
        self.declare_parameter('pose_home',   [0.0, -1.0, 0.7, 0.7])
        self.declare_parameter('pose_extend', [0.0, -0.3, 0.2, 0.6])
        self.declare_parameter('pose_carry',  [0.0, -1.2, 0.2, 1.0])
        self.declare_parameter('pose_wave',   [0.0, -0.2, -1.0, 1.2])
        self.declare_parameter('gripper_open', 0.019)
        self.declare_parameter('gripper_close', -0.01)

        self._move_t = float(self.get_parameter('move_time_s').value)
        self._poses = {
            'home':   list(self.get_parameter('pose_home').value),
            'extend': list(self.get_parameter('pose_extend').value),
            'carry':  list(self.get_parameter('pose_carry').value),
            'wave':   list(self.get_parameter('pose_wave').value),
        }
        self._g_open = float(self.get_parameter('gripper_open').value)
        self._g_close = float(self.get_parameter('gripper_close').value)

        traj_action = self.get_parameter('joint_trajectory_action').value
        grip_action = self.get_parameter('gripper_action').value

        self._traj_client = ActionClient(self, FollowJointTrajectory, traj_action)
        self._grip_client = ActionClient(self, GripperCommand, grip_action)

        self.create_subscription(String, '/arm/command', self._on_command, 10)
        self.get_logger().info(
            f'arm_controller ready: traj={traj_action}, gripper={grip_action}'
        )

    # -------------------------------------------------------------- helpers
    def _send_joint_goal(self, positions: Iterable[float]) -> None:
        if not self._traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowJointTrajectory action server unavailable')
            return
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        sec = int(self._move_t)
        ns = int((self._move_t - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = ns
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(f'Sending arm goal: {pt.positions}')
        self._traj_client.send_goal_async(goal)

    def _send_gripper_goal(self, position: float) -> None:
        if not self._grip_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('GripperCommand action server unavailable')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 1.0
        self.get_logger().info(f'Sending gripper goal: pos={position:.3f}')
        self._grip_client.send_goal_async(goal)

    # -------------------------------------------------------------- subs
    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd in self._poses:
            self._send_joint_goal(self._poses[cmd])
        elif cmd == 'open':
            self._send_gripper_goal(self._g_open)
        elif cmd == 'close':
            self._send_gripper_goal(self._g_close)
        else:
            self.get_logger().warning(f'Unknown arm command "{cmd}"')


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
