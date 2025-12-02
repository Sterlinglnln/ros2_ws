from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import random

def generate_launch_description():
    pkg_path = get_package_share_directory('turtle_exercise')
    turtle_num = 10
    nodes = []

    # turtlesim 主节点要最先启动
    sim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    nodes.append(sim_node)

    # Spawn + random_walk + TF for 10 turtles
    for i in range(turtle_num):
        name = f"turtle{i+1}"

        # 随机坐标
        x = random.random() * 11
        y = random.random() * 11
        theta = 0.0

        # Spawn command (数组形式，绝对不会拆字符)
        spawn_cmd = [
            'ros2', 'service', 'call',
            '/spawn',
            'turtlesim/srv/Spawn',
            f'{{x: {x}, y: {y}, theta: {theta}, name: "{name}"}}'
        ]

        nodes.append(
            ExecuteProcess(
                cmd=spawn_cmd,
                shell=False
            )
        )

        # random_walk
        nodes.append(
            Node(
                package='turtle_exercise',
                executable='random_walk',
                namespace=name
            )
        )

        # TF 广播
        nodes.append(
            Node(
                package='turtle_exercise',
                executable='tf_broadcast',
                parameters=[{'name': name}]
            )
        )

    # RViz
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', f'{pkg_path}/rviz/multi_tf_broadcast.rviz']
        )
    )

    return LaunchDescription(nodes)
