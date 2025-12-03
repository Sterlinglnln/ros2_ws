import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取功能包路径
    pkg_path = get_package_share_directory("single_robot_arm")

    # URDF / RViz / Gazebo 配置等路径
    urdf_path = os.path.join(pkg_path, "urdf", "single_robot_arm.urdf")
    rviz_config_path = os.path.join(pkg_path, "config", "display_robot.rviz")

    # 如果你有 world，可以放到 worlds 目录
    world_path = os.path.join(pkg_path, "worlds", "test_world.sdf")
    world_exists = os.path.exists(world_path)

    # ------------------------------
    # Robot State Publisher（URDF）
    # ------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["cat ", urdf_path]),
                value_type=str,
            )
        }],
    )

    # ------------------------------
    # Gazebo 启动（ros_gz_sim）
    # ------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            # 如果 world 存在，则加载，否则使用空白世界
            "gz_args": f"{world_path} -r" if world_exists else "-r",
        }.items(),
    )

    # ------------------------------
    # 将机器人实体导入 Gazebo
    # ------------------------------
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "single_robot_arm",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    # ------------------------------
    # 如果你没有桥接配置文件，可以写死默认桥接
    # ------------------------------
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # ------------------------------
    # RViz2 显示
    # ------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        bridge_node,
        rviz_node
    ])
