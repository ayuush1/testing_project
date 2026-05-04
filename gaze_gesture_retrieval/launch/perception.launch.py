"""Launch only the perception (camera + gaze + gesture + YOLO) stack.

Useful for tuning the perception pipeline without bringing up the robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('gaze_gesture_retrieval')
    params = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='gaze_gesture_retrieval',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
        Node(
            package='gaze_gesture_retrieval',
            executable='gaze_node',
            name='gaze_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='gaze_gesture_retrieval',
            executable='gesture_node',
            name='gesture_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='gaze_gesture_retrieval',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[params],
        ),
    ])
