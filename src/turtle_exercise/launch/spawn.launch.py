from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    num_args = DeclareLaunchArgument('num', default_value='10')

    return LaunchDescription([
        num_args,
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='turtle_exercise',
            executable='spawn',
            parameters=[{
                'num': LaunchConfiguration('num')
            }]
        ),
    ])
