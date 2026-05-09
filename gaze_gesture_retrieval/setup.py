from setuptools import setup
from glob import glob
import os

package_name = 'gaze_gesture_retrieval'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CS 4379K / CS 5342 Final Project Team',
    maintainer_email='student@txstate.edu',
    description='Gaze-Guided and Gesture-Controlled Object Retrieval using TurtleBot3.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Perception
            'perception_node = gaze_gesture_retrieval.perception_node:main',
            'gaze_node       = gaze_gesture_retrieval.gaze_node:main',
            'gesture_node    = gaze_gesture_retrieval.gesture_node:main',
            'yolo_node       = gaze_gesture_retrieval.yolo_node:main',
            'yolo_json_bridge = gaze_gesture_retrieval.yolo_json_bridge:main',
            'gaze_overlay    = gaze_gesture_retrieval.gaze_overlay_node:main',
            # Fusion / decision
            'fusion_node     = gaze_gesture_retrieval.fusion_node:main',
            'mode_manager    = gaze_gesture_retrieval.mode_manager:main',
            # Control
            'teleop_bridge   = gaze_gesture_retrieval.teleop_bridge:main',
            'arm_controller  = gaze_gesture_retrieval.arm_controller:main',
            'nav_client      = gaze_gesture_retrieval.nav_client:main',
            # Top-level orchestrator
            'retrieval_orchestrator = gaze_gesture_retrieval.retrieval_orchestrator:main',
        ],
    },
)
