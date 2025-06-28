import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import UnlessCondition # UnlessCondition artık kullanılmıyor ama kalsın
from launch.event_handlers import OnProcessExit

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Paketinizin paylaşım dizinini alın
    share_dir = get_package_share_directory('last_description')

    # URDF dosyasının yolu
    xacro_file = os.path.join(share_dir, 'urdf', 'last.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Kontrolör YAML dosyasının yolu (artık manuel controller_manager başlatmadığımız için doğrudan kullanılmıyor ama spawner'lar ve gazebo_ros2_control kullanıyor)
    controller_config_file = os.path.join(share_dir, 'config', 'robot_control.yaml')

    # Launch argümanlarını tanımla
    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Set to "true" to start Gazebo in a paused state.'
    )

    # Robot State Publisher düğümü
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': True} # Gazebo ile zaman senkronizasyonu için
        ]
    )

    # Gazebo sunucusunu başlat (gzserver)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': LaunchConfiguration('paused'),
            'physics': 'ode',
            'verbose': 'true'
        }.items()
    )

    # Gazebo istemcisini başlat (gzclient)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # URDF modelini Gazebo'ya spawn et
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'last', # Modelinizin adı
            '-topic', 'robot_description', # robot_state_publisher'dan gelen URDF konusu
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1' # Yerde durması için yeterli yükseklik
        ],
        output='screen'
    )

    # Joint State Broadcaster spawner'ı (eklem durumlarını yayınlar)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--activate"],
        parameters=[{'use_sim_time': True}], # Spawner'ın da simülasyon zamanını kullanması önemli
        output='screen'
    )

    # Diff Drive Controller spawner'ı (robotun hareketini kontrol eder)
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager", "--activate"],
        parameters=[{'use_sim_time': True}], # Spawner'ın da simülasyon zamanını kullanması önemli
        output='screen'
    )

    # Launch açıklamasını döndür
    return LaunchDescription([
        paused_arg,
        gazebo_server, # Gazebo sunucusunu başlat
        gazebo_client, # Gazebo istemcisini başlat
        robot_state_publisher_node, # Robotun durumunu yayınla
        urdf_spawn_node, # Robotu Gazebo'ya spawn et

        # urdf_spawn_node'un bitmesini bekleyip kontrolörleri başlat
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawn_node, # spawn_entity.py'nin bitmesini bekle
                on_exit=[
                    joint_state_broadcaster_spawner, # Joint State Broadcaster'ı etkinleştir
                    diff_drive_controller_spawner, # Diff Drive Controller'ı etkinleştir
                ]
            )
        )
    ])
