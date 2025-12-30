"""
Launch file for Hexapod perception nodes

Starts camera and face recognition nodes:
- camera_node: Publishes camera frames
- face_recognition_node: Detects and recognizes faces
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('hexapod_perception')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'perception.yaml')

    return LaunchDescription([
        # Camera Node
        Node(
            package='hexapod_perception',
            executable='camera_node',
            name='camera_node',
            parameters=[config_file],
            output='screen',
        ),

        # Face Recognition Node
        Node(
            package='hexapod_perception',
            executable='face_recognition_node',
            name='face_recognition_node',
            parameters=[config_file],
            output='screen',
        ),
    ])
