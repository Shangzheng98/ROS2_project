from distutils.command.config import config
from click import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

import os
def generate_launch_description():

    config = os.path.join (
        get_package_share_directory('auto_aim'),
        'config',
        'autoAim.yaml'

    )
    return LaunchDescription(
        [
            Node(
                package="auto_aim",
                namespace='',
                executable="auto_aim_node",
                name='auto_aim_node',
                parameters=[config]
            )
        ]
    )