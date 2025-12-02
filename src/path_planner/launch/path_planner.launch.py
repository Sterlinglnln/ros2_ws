import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('fishbot_description')
    navigation_share = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_model = PathJoinSubstitution(
        [description_share, 'urdf', 'fishbot', 'fishbot.urdf.xacro'])
    default_map = PathJoinSubstitution(
        [navigation_share, 'maps', 'room.yaml'])
    default_params = PathJoinSubstitution(
        [navigation_share, 'config', 'nav2_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to fishbot XACRO'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Map yaml to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 params file'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),
        Node(
            package='path_planner',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
