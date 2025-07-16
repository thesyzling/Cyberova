#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        teleop_node
    ]) 