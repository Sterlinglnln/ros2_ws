from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_exercise',
            executable='draw_spiral',
            name='drawspiral',
            parameters=[
                {'turtle_name': 'turtle1'},
            ]
        ),
    ])
