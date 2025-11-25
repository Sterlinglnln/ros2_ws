from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        remappings=[('/turtle1/pose', '/sim1/pose')]
    )
    ld.add_action(node1)

    return ld