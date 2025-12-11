from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/camera/depth/color/points')
    frames_to_accumulate = LaunchConfiguration('frames_to_accumulate', default='30')
    min_point_threshold = LaunchConfiguration('min_point_threshold', default='12000')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size', default='0.005')
    save_mesh_path = LaunchConfiguration('save_mesh_path', default='')
    publish_mesh = LaunchConfiguration('publish_mesh', default='true')
    auto_reconstruct = LaunchConfiguration('auto_reconstruct', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value=pointcloud_topic,
                              description='PointCloud2 topic from the D435i depth node'),
        DeclareLaunchArgument('frames_to_accumulate', default_value=frames_to_accumulate,
                              description='How many frames to integrate before reconstructing'),
        DeclareLaunchArgument('min_point_threshold', default_value=min_point_threshold,
                              description='Minimum accumulated points to trigger reconstruction'),
        DeclareLaunchArgument('voxel_leaf_size', default_value=voxel_leaf_size,
                              description='Leaf size for VoxelGrid downsampling'),
        DeclareLaunchArgument('save_mesh_path', default_value=save_mesh_path,
                              description='Optional path to persist the reconstructed mesh as a PLY file'),
        DeclareLaunchArgument('publish_mesh', default_value=publish_mesh,
                              description='Publish pcl_msgs/PolygonMesh when true'),
        DeclareLaunchArgument('auto_reconstruct', default_value=auto_reconstruct,
                              description='Automatically reconstruct once enough frames are buffered'),
        Node(
            package='realsense_head_reconstruction',
            executable='head_reconstruction_node',
            name='head_reconstruction_node',
            output='screen',
            parameters=[{
                'pointcloud_topic': pointcloud_topic,
                'frames_to_accumulate': ParameterValue(frames_to_accumulate, value_type=int),
                'min_point_threshold': ParameterValue(min_point_threshold, value_type=int),
                'voxel_leaf_size': ParameterValue(voxel_leaf_size, value_type=float),
                'save_mesh_path': save_mesh_path,
                'publish_mesh': ParameterValue(publish_mesh, value_type=bool),
                'auto_reconstruct': ParameterValue(auto_reconstruct, value_type=bool)
            }]
        )
    ])
