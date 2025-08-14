import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

    workspace = os.getenv("SONIA_WS", None)

    if workspace is None:
        raise Exception("env var SONIA_WS not set")
    
    return LaunchDescription(
        [
            Node(
                package="sonia_bt_runner",
                executable="run_server",
            )
        ]
    )