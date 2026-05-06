"""Top-level bringup: perception + fusion + control + retrieval orchestrator.

This launch file does *not* start Gazebo, Nav2, or the TurtleBot3 hardware /
manipulation stack. Run those separately following the assignment manual,
e.g.:

    ros2 launch turtlebot3_manipulation_gazebo gazebo.launch.py
    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py \\
        map_yaml_file:=$HOME/map.yaml
    ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py

Then in another shell:
    ros2 launch gaze_gesture_retrieval bringup.launch.py

Launch arguments
----------------
    robot_camera_topic:=/camera/image_raw    # what yolo_node subscribes to
    user_camera_topic:=/user_camera/image_raw  # gaze + gesture input

Example using the Gazebo turtlebot camera directly:
    ros2 launch gaze_gesture_retrieval bringup.launch.py \\
        robot_camera_topic:=/camera/image_raw
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('gaze_gesture_retrieval')
    params = os.path.join(pkg, 'config', 'params.yaml')

    user_camera = LaunchConfiguration('user_camera_topic')
    robot_camera = LaunchConfiguration('robot_camera_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'user_camera_topic',
            default_value='/user_camera/image_raw',
            description='Image topic for gaze + gesture (user-facing camera)',
        ),
        DeclareLaunchArgument(
            'robot_camera_topic',
            default_value='/robot_camera/image_raw',
            description='Image topic for YOLO (robot camera). Set to '
                        '/camera/image_raw to use the Gazebo turtlebot camera '
                        'directly.',
        ),

        Node(package='gaze_gesture_retrieval', executable='perception_node',
             name='perception_node', output='screen',
             parameters=[{'topic': user_camera}]),
        Node(package='gaze_gesture_retrieval', executable='gaze_node',
             name='gaze_node', output='screen',
             parameters=[params, {'camera_topic': user_camera}]),
        Node(package='gaze_gesture_retrieval', executable='gesture_node',
             name='gesture_node', output='screen',
             parameters=[params, {'camera_topic': user_camera}]),
        Node(package='gaze_gesture_retrieval', executable='yolo_node',
             name='yolo_node', output='screen',
             parameters=[params, {'camera_topic': robot_camera}]),
        Node(package='gaze_gesture_retrieval', executable='fusion_node',
             name='fusion_node', output='screen', parameters=[params]),
        Node(package='gaze_gesture_retrieval', executable='mode_manager',
             name='mode_manager', output='screen', parameters=[params]),
        Node(package='gaze_gesture_retrieval', executable='teleop_bridge',
             name='teleop_bridge', output='screen', parameters=[params]),
        Node(package='gaze_gesture_retrieval', executable='arm_controller',
             name='arm_controller', output='screen', parameters=[params]),
        Node(package='gaze_gesture_retrieval', executable='nav_client',
             name='nav_client', output='screen', parameters=[params]),
        Node(package='gaze_gesture_retrieval', executable='retrieval_orchestrator',
             name='retrieval_orchestrator', output='screen', parameters=[params]),
    ])
