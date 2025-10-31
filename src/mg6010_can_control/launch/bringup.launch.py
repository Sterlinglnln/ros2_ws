import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mg6010_can_control')

    ros2_control_path = LaunchConfiguration(
        'ros2_control_config',
        default=os.path.join(pkg_share, 'config', 'ros2_control.yaml')
    )
    controllers_path = LaunchConfiguration(
        'controllers_config',
        default=os.path.join(pkg_share, 'config', 'controllers.yaml')
    )

    declare_ros2_control = DeclareLaunchArgument(
        'ros2_control_config', default_value=ros2_control_path,
        description='ros2_control hardware config')
    declare_controllers = DeclareLaunchArgument(
        'controllers_config', default_value=controllers_path,
        description='controller manager configuration')

    # robot_state_publisher 仅为了连通性（如果你已有完整 URDF 可加载）
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # ros2_control_node
    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[LaunchConfiguration('ros2_control_config')],
        output='screen'
    )

    spawner_js = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    spawner_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--param-file', LaunchConfiguration('controllers_config')],
        output='screen'
    )

    return LaunchDescription([
        declare_ros2_control,
        declare_controllers,
        rsp,
        ros2_control,
        spawner_js,
        spawner_diff
    ])
