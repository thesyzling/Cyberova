#!/usr/bin/env python3
"""
SLAM Mapping Launch File - STABİL VERSİYON
Gazebo'da harita oluşturmak için kullanılır
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Paket dizinlerini al
    smooth_controller_share = get_package_share_directory('last_smooth_controller')
    last_description_share = get_package_share_directory('last_description')
    
    # Config dosyaları
    slam_params_file = os.path.join(smooth_controller_share, 'config', 'slam_mapping_params.yaml')
    rviz_config_file = os.path.join(smooth_controller_share, 'config', 'slam_rviz.rviz')
    ekf_config_file = os.path.join(smooth_controller_share, 'config', 'ekf.yaml')
    
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
    
    use_saved_map_arg = DeclareLaunchArgument(
        'use_saved_map',
        default_value='false',
        description='Whether to continue from a saved map'
    )
    
    saved_map_path_arg = DeclareLaunchArgument(
        'saved_map_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('last_smooth_controller'), 'maps', 'saved_map'
        ]),
        description='Path to the saved map'
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
    
    # Robot'u Gazebo'ya spawn et
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
                    '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )
    
    # Joint State Broadcaster spawner
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
    
    # Diff Drive Controller spawn
    diff_drive_spawner = TimerAction(
        period=6.0,
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
    
    # EKF (Robot Localization) Node
    ekf_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )
    
    # SLAM Toolbox Node - Yeni harita oluşturma (8 saniye sonra)
    slam_node_new_map = TimerAction(
        period=8.0,
        actions=[
            Node(
                condition=UnlessCondition(LaunchConfiguration('use_saved_map')),
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'mode': 'mapping',
                        'map_file_name': '',
                    }
                ],
            )
        ]
    )
    
    # SLAM Toolbox Node - Kaydedilmiş haritadan devam etme
    slam_node_saved_map = TimerAction(
        period=8.0,
        actions=[
            Node(
                condition=IfCondition(LaunchConfiguration('use_saved_map')),
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'mode': 'mapping',
                        'map_file_name': LaunchConfiguration('saved_map_path'),
                    }
                ],
            )
        ]
    )
    
    # RViz Node - SLAM monitoring
    rviz_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_slam_mapping',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )
    
    return LaunchDescription([
        # Launch argumentleri
        use_sim_time_arg,
        use_saved_map_arg,
        saved_map_path_arg,
        
        # Gazebo
        gazebo_server,
        gazebo_client,
        
        # Robot
        robot_state_publisher_node,
        urdf_spawn_node,
        
        # Controllers
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        
        # SENSOR FUSION
        ekf_node,
        
        # SLAM - STABİL VERSİYON
        slam_node_new_map,
        slam_node_saved_map,
        
        # Visualization
        rviz_node,
    ]) 