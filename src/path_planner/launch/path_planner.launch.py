import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # 声明 use_sim_time 参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),

        # 启动你自己的 PathPlanner 节点
        Node(
            package='path_planner',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
        )
    ])
