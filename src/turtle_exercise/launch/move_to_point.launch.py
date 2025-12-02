from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('turtle_exercise')
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='turtle_exercise',
            executable='move_to_point',
            parameters=[{
                'x': 2.0,
                'y': 2.0
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', pkg_path + '/rviz/tf_broadcast.rviz',]
        )
    ])
