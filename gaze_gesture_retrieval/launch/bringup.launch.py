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
    user_camera_topic:=/user_camera/image_raw   # gaze + gesture input
    robot_camera_topic:=/robot_camera/image_raw # only used when yolo_source=local
    yolo_source:=local                          # 'local'  -> run yolo_node here
                                                # 'remote' -> run yolo_json_bridge,
                                                #             expects the Jetson
                                                #             to publish JSON

Examples
--------
Simulation (Gazebo robot camera):
    ros2 launch gaze_gesture_retrieval bringup.launch.py \\
        robot_camera_topic:=/camera/image_raw

Physical robot, YOLO running on the Jetson (Assignment 4 style):
    ros2 launch gaze_gesture_retrieval bringup.launch.py yolo_source:=remote
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('gaze_gesture_retrieval')
    params = os.path.join(pkg, 'config', 'params.yaml')

    user_camera = LaunchConfiguration('user_camera_topic')
    robot_camera = LaunchConfiguration('robot_camera_topic')
    yolo_source = LaunchConfiguration('yolo_source')
    enable_overlay = LaunchConfiguration('enable_overlay')

    is_local = IfCondition(
        PythonExpression(["'", yolo_source, "' == 'local'"]))
    is_remote = IfCondition(
        PythonExpression(["'", yolo_source, "' == 'remote'"]))
    overlay_on = IfCondition(
        PythonExpression(["'", enable_overlay, "'.lower() == 'true'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'user_camera_topic',
            default_value='/user_camera/image_raw',
            description='Image topic for gaze + gesture (user-facing camera).',
        ),
        DeclareLaunchArgument(
            'robot_camera_topic',
            default_value='/robot_camera/image_raw',
            description='Robot-camera topic that yolo_node subscribes to '
                        '(only used when yolo_source:=local). Set to '
                        '/camera/image_raw to use Gazebo turtlebot camera.',
        ),
        DeclareLaunchArgument(
            'yolo_source',
            default_value='local',
            description="'local' runs yolo_node on this machine and "
                        'subscribes to robot_camera_topic. '
                        "'remote' runs yolo_json_bridge instead and expects "
                        'the Jetson to publish on /yolo/detections_json '
                        '(Assignment 4 style edge inference).',
        ),
        DeclareLaunchArgument(
            'enable_overlay',
            default_value='true',
            description='Open an OpenCV HUD window showing the robot camera, '
                        'YOLO boxes, gaze cursor, candidate and locked '
                        'target. Strongly recommended for telepresence use.',
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

        # Arch A: local YOLO on this Remote PC.
        Node(package='gaze_gesture_retrieval', executable='yolo_node',
             name='yolo_node', output='screen',
             parameters=[params, {'camera_topic': robot_camera}],
             condition=is_local),

        # Arch B: subscribe to JSON detections coming from the Jetson.
        Node(package='gaze_gesture_retrieval', executable='yolo_json_bridge',
             name='yolo_json_bridge', output='screen',
             parameters=[params],
             condition=is_remote),

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
        Node(package='gaze_gesture_retrieval', executable='gaze_overlay',
             name='gaze_overlay_node', output='screen',
             parameters=[params, {'robot_camera_topic': robot_camera}],
             condition=overlay_on),
    ])
