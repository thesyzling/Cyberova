#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paket dizinlerini al
    smooth_controller_share = get_package_share_directory('last_smooth_controller')
    last_description_share = get_package_share_directory('last_description')
    
    # Config dosyaları
    slam_config_file = os.path.join(smooth_controller_share, 'config', 'slam_config.yaml')
    rviz_config_file = os.path.join(smooth_controller_share, 'config', 'slam_rviz.rviz')
    
    # URDF dosyasını xacro ile işle
    xacro_file = os.path.join(last_description_share, 'urdf', 'last.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    # Launch argümanları
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo sunucusunu başlat
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false',
            'physics': 'ode',
            'verbose': 'false'
        }.items()
    )
    
    # Gazebo istemcisini başlat
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )
    
    # Robot'u Gazebo'ya spawn et (3 saniye sonra)
    urdf_spawn_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'last_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )
    
    # Joint State Broadcaster spawner (5 saniye sonra)
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )
    
    # Diff Drive Controller spawn (5 saniye sonra)
    diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )
    
    # SLAM Toolbox Node (8 saniye sonra) - Transform timing için geciktirildi
    slam_toolbox_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config_file,
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        # Transform timing optimization
                        'tf_buffer_duration': 10.0,  # Longer buffer duration
                        'tf_timeout': 2.0,           # Increased timeout
                        'minimum_time_interval': 0.5, # Process every 0.5 seconds
                        'transform_timeout': 0.5,     # Transform timeout
                        'minimum_travel_distance': 0.2, # Move 20cm before processing
                        'minimum_travel_heading': 0.17, # 10 degrees rotation
                    }
                ],
            )
        ]
    )
    
    # RViz Node (12 saniye sonra)
    rviz_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_slam',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )
    
    # Teleop Node (14 saniye sonra)
    teleop_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='gnome-terminal --',
                remappings=[
                    ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
                ]
            )
        ]
    )
    
    return LaunchDescription([
        # Launch argumentleri
        use_sim_time_arg,
        
        # Gazebo
        gazebo_server,
        gazebo_client,
        
        # Robot
        robot_state_publisher_node,
        urdf_spawn_node,
        
        # Controllers
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        
        # SLAM
        slam_toolbox_node,
        
        # Visualization
        rviz_node,
        
        # Control
        teleop_node,
    ]) 