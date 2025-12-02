from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('turtle_exercise')
    ld = LaunchDescription()

    sim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    ld.add_action(sim)

    tf1 = Node(
        package='turtle_exercise',
        executable='tf_broadcast',
        parameters=[
            {'name': 'turtle1'}
        ]
    )
    ld.add_action(tf1)

    tf2 = Node(
        package='turtle_exercise',
        executable='tf_broadcast',
        parameters=[
            {'name': 'turtle2'}
        ]
    )
    ld.add_action(tf2)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', pkg_path + '/rviz/tf_following.rviz',]
    )
    ld.add_action(rviz)

    follow = Node(
        package='turtle_exercise',
        executable='tf_following',
        parameters=[
            {'source_frame': 'turtle1'}
        ]
    )
    ld.add_action(follow)

    random_walk = Node(
        package='turtle_exercise',
        executable='random_walk',
        namespace='turtle1'
    )
    ld.add_action(random_walk)

    return ld
