from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 获取当前包的安装位置
package_path = get_package_share_directory('urdf_test')
print(package_path)

# 读取 urdf 文件
urdf_path = package_path + '/urdf/robot.urdf'
robot_desc = open(urdf_path).read()

def generate_launch_description():
    robot_desc_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True},
                    {'robot_description': robot_desc},]
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', package_path + 'urdf/rviz.rviz',]
    )

    return LaunchDescription([
        robot_desc_pub_node,
        joint_state_pub_node,
        rviz_node,
    ])
