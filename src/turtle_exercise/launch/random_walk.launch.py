from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import random

def generate_launch_description():
    turtle_num = 10
    ld = LaunchDescription()

    sim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    ld.add_action(sim)

    for i in range(turtle_num):
        name = 'turtle' + str(i + 1)
        turtle = Node(
            package='turtle_exercise',
            executable='random_walk',
            namespace=name
        )
        ld.add_action(turtle)

        x = random.random() * 11
        y = random.random() * 11
        cmd = f'x: {x}, y: {y}, name: {name}'
        ep = ExecuteProcess(
            cmd=[['ros2 service call ',
                  '/spawn ',
                  'turtlesim/srv/Spawn ',
                  '"{' + cmd + '}"'
                ]], shell=True
        )
        ld.add_action(ep)


    return ld
