from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # return launch
    return LaunchDescription(
        [
            Node(
                package="move_tortoisebot",
                executable="move_tortoisebot",
                name="move_tortoisebot_node",
                output="screen",
            )
        ]
    )