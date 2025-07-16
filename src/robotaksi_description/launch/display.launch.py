from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('robotaksi_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'robotaksi.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'robotaksi_sim.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='False'
    )

    show_gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Only use joint_state_publisher when NOT using ros2_control AND gui is false
    joint_state_publisher_node = Node(
        condition=IfCondition(
            PythonExpression([
                "'", use_ros2_control, "' != 'true' and '", show_gui, "' != 'true'"
            ])
        ),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Only use joint_state_publisher_gui when NOT using ros2_control AND gui is true
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(
            PythonExpression([
                "'", use_ros2_control, "' != 'true' and '", show_gui, "' == 'true'"
            ])
        ),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
