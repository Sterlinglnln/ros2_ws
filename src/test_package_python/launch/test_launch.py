from sys import executable

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='turtle1',
        name='node1',
    )
    ld.add_action(node1)

    node2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='turtle2',
    )
    ld.add_action(node2)

    return ld