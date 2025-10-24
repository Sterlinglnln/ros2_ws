import rosbag2_py
import sensor_msgs.msg
import numpy as np
import open3d as o3d
from rclpy.serialization import deserialize_message
from sensor_msgs_py import point_cloud2

# 配置包路径和话题名称
BAG_PATH = "test_sample_0.mcap"
TOPIC = "/camera/camera/depth/color/points"

# 初始化ROS2包读取器
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
reader.open(storage_options, converter_options)

print("正在读取bag，提取第一帧点云数据...")

# 遍历bag文件中的消息
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == TOPIC:
        msg = deserialize_message(data, sensor_msgs.msg.PointCloud2)
        points_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = [p for p in points_gen]

        if len(points_list) == 0:
            print("跳过空点云帧...")
            continue  # 跳至下一帧
        
        print(f"找到有效点云帧，共 {len(points_list)} 个点，正在转换为Open3D格式...")
        points_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [(p[0], p[1], p[2]) for p in points_gen]
        points = np.array(points, dtype=float)

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        o3d.io.write_point_cloud("scan_extracted.pcd", cloud)
        o3d.io.write_point_cloud("scan_extracted.ply", cloud)
        print("成功保存 scan_extracted.pcd 和 scan_extracted.ply！")
        break

print("完成！")
