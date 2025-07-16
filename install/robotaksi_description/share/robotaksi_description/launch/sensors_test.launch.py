#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robotaksi_description_dir = get_package_share_directory('robotaksi_description')
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robotaksi_description_dir, '/launch/gazebo_with_controllers.launch.py']),
        launch_arguments={
            'world': 'empty.world',
            'gui': 'true',
            'server': 'true',
            'debug': 'false'
        }.items()
    )
    
    # RViz2 with sensors configuration
    rviz_config_file = os.path.join(robotaksi_description_dir, 'config', 'robotaksi_sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_node,
    ]) 