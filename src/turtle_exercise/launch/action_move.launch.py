from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('order', default_value='6'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='turtle_exercise',
            executable='action_move',
        ),

        ExecuteProcess(cmd=[[
            'ros2 action send_goal --feedback ',
            '/turtle_move ',
            'interface_exercise/action/Move ',
            '"{order: ',
            LaunchConfiguration('order'),'}"',
        ]],
        shell=True)
    ])