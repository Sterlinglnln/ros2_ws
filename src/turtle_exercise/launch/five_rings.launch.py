from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtle_pose = [
        (3.5, 7.0),   # 蓝（左上）
        (6.0, 7.0),   # 黑（中上）
        (8.5, 7.0),   # 红（右上）
        (4.75, 5.0),  # 黄（左下）
        (7.25, 5.0)   # 绿（右下）
    ]

    ring_color = [
        (0, 0, 255),     # 蓝
        (0, 0, 0),       # 黑
        (255, 0, 0),     # 红
        (255, 215, 0),   # 黄
        (0, 255, 0)      # 绿
    ]

    # turtlesim
    ld.add_action(Node(
        package="turtlesim",
        executable="turtlesim_node"
    ))

    # 清除默认turtle1
    ld.add_action(Node(
        package="turtle_exercise",
        executable="clear_turtle"
    ))

    # 生成五只乌龟
    for i in range(5):
        name=f"my_turtle_{i+1}"
        ld.add_action(Node(
            package='turtle_exercise',
            executable='spawn_cmd',
            parameters=[{
                'x': float(turtle_pose[i][0]),
                'y': float(turtle_pose[i][1]),
                'name': name
            }]
        ))

    # 设置画笔
    for i in range(5):
        name=f"my_turtle_{i+1}"
        r,g,b = ring_color[i]
        ld.add_action(Node(
            package='turtle_exercise',
            executable='set_pen',
            parameters=[{
                'r': int(r), 'g': int(g), 'b': int(b),
                'width': 5,
                'name': name
            }]
        ))

    return ld
