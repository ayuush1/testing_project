"""Thin wrapper around the Nav2 ``NavigateToPose`` action server.

Subscribes to ``/nav/goal`` (geometry_msgs/PoseStamped) and forwards each
incoming pose as a NavigateToPose goal. Publishes ``/nav/status`` (String)
with the current navigation state ('idle' | 'navigating' | 'succeeded' |
'failed').
"""
from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateToPose
    _HAS_NAV2 = True
except Exception:
    _HAS_NAV2 = False


class NavClient(Node):
    def __init__(self) -> None:
        super().__init__('nav_client')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self._action = self.get_parameter('nav_action').value

        self._pub_status = self.create_publisher(String, '/nav/status', 10)
        self.create_subscription(PoseStamped, '/nav/goal', self._on_goal, 10)

        if _HAS_NAV2:
            self._client = ActionClient(self, NavigateToPose, self._action)
            self.get_logger().info(f'nav_client connecting to {self._action}')
        else:
            self._client = None
            self.get_logger().warning(
                'nav2_msgs not available; nav_client will only republish goals'
            )
        self._publish_status('idle')

    def _publish_status(self, s: str) -> None:
        self._pub_status.publish(String(data=s))

    def _on_goal(self, msg: PoseStamped) -> None:
        if self._client is None:
            self.get_logger().warning(
                'Received /nav/goal but Nav2 action client unavailable')
            return
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose server unavailable')
            self._publish_status('failed')
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self._publish_status('navigating')
        self.get_logger().info(
            f'Sending nav goal x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose send_goal failed: {exc}')
            self._publish_status('failed')
            return
        if not handle.accepted:
            self.get_logger().warning('NavigateToPose goal rejected')
            self._publish_status('failed')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            future.result()
            self._publish_status('succeeded')
            self.get_logger().info('NavigateToPose succeeded')
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose failed: {exc}')
            self._publish_status('failed')


def main(args=None):
    rclpy.init(args=args)
    node = NavClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
