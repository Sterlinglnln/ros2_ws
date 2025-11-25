from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('test_package_python'),
        'config',
        'turtlesim.yaml'
    )
    # r = DeclareLaunchArgument(
    #     'background_r', default_value=TextSubstitution(text='0')
    # )
    # g = DeclareLaunchArgument(
    #     'background_g', default_value=TextSubstitution(text='15')
    # )
    # b = DeclareLaunchArgument(
    #     'background_b', default_value=TextSubstitution(text='125')
    # )
    # ld.add_action(r)
    # ld.add_action(g)
    # ld.add_action(b)

    node1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        # parameters=[{
        #     # 'background_r': LaunchConfiguration('background_r'),
        #     # 'background_g': LaunchConfiguration('background_g'),
        #     # 'background_b': LaunchConfiguration('background_b'),
        # }]
        parameters=[config]
    )
    ld.add_action(node1)

    return ld
