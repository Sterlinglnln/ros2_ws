# realsense_head_reconstruction

该 ROS 2 功能包提供一个节点，可以订阅 Intel RealSense D435i 发布的 `PointCloud2` 数据，筛选出头部区域的点并在积累多帧后使用 PCL 的 Greedy Projection Triangulation 算法进行简单的三维重建。节点内置体素滤波 + ICP 点云配准，可在多帧累积前将点云对齐，解决旋转采集时的重影问题。重建结果既可以作为点云发布，也可以通过 `pcl_msgs/PolygonMesh` 发布，同时支持将网格自动保存到 PLY 文件。

## 依赖

- ROS 2（tested with Humble/Foxy 同类发行版）
- `librealsense2` 驱动与 `realsense2_camera` ROS 2 驱动（负责发布 `/camera/depth/color/points`）
- PCL 1.10+（ROS 2 自带）

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select realsense_head_reconstruction
source install/setup.bash
```

## 运行

1. 启动 RealSense D435i 驱动：

   ```bash
   ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true align_depth:=true
   ```

2. 启动重建节点：

   ```bash
   ros2 launch realsense_head_reconstruction head_reconstruction.launch.py \
     pointcloud_topic:=/camera/depth/color/points \
     frames_to_accumulate:=30 \
     min_point_threshold:=12000 \
     voxel_leaf_size:=0.005 \
     save_mesh_path:=${HOME}/head_mesh.ply
   ```

参数说明：

- `pointcloud_topic`：RealSense 发布的彩色对齐点云话题。
- `frames_to_accumulate`：每次重建前需要累积的帧数。
- `min_point_threshold`：即使未达到帧数阈值，只要点数超过此值也会触发重建。
- `voxel_leaf_size`：体素滤波叶子尺寸，单位米。
- `z_limits` / `y_limits`：可在 `params` 中覆盖，控制头部 ROI，默认 `[0.2, 1.2]` 米和 `[-0.2, 0.4]` 米。
- `save_mesh_path`：非空时会将最新网格保存为 PLY 文件。
- `publish_mesh`：布尔值，指定是否发布 `pcl_msgs/PolygonMesh`。
- `auto_reconstruct`：`true` 时积累到阈值自动重建；`false` 时需通过服务手动开始/结束一次采集。
- `use_registration`：是否在累积之前运行 ICP 点云配准（默认启用）。
- `icp_max_correspondence_distance`、`icp_max_iterations`、`icp_transformation_epsilon`、`icp_fitness_epsilon`：ICP 相关参数，可根据噪声与运动速度微调。

Topic 输出：

- `filtered_pointcloud`：筛选 + 下采样后的单帧头部点云。
- `registered_pointcloud`：完成 ICP 配准后的点云（仅在启用配准时发布）。
- `accumulated_pointcloud`：本次重建前的累计点云（触发重建时才发布）。
- `reconstructed_mesh`：`pcl_msgs/PolygonMesh`，方便 RViz2 或下游节点直接显示。

## 可视化

在 RViz2 中添加 `PointCloud2` 显示 `filtered_pointcloud` / `accumulated_pointcloud`，再添加 `Polygon` 显示 `reconstructed_mesh`，即可看到实时点云与重建后的头部网格。如果只需要离线后处理，可利用保存的 PLY 文件在 MeshLab/Open3D 中打开。

## 头部转动一圈采集方案

若希望受试者缓慢旋转头部一圈后再执行一次三维重建，建议将参数 `auto_reconstruct:=false`，并配合下面的服务完成一次采集流程：

1. 开始采集并清空上一轮数据：

   ```bash
   ros2 service call /start_head_capture std_srvs/srv/Trigger {}
   ```

2. 让受试者缓慢转头，等待 RealSense 将整圈点云发布出来（可在 RViz2 中观察 `filtered_pointcloud`）。

3. 完成旋转后调用停止/重建服务，该服务会冻结当前积累的点并立即触发重建，完成后会将网格发布并按需保存：

   ```bash
   ros2 service call /stop_and_reconstruct std_srvs/srv/Trigger {}
   ```

如需再次采集，只需重复执行步骤 1–3；当 `auto_reconstruct:=true` 时，节点也会在达到帧数或点数阈值后自动重置进入下一轮。
