import open3d as o3d
import numpy as np

# 读取点云数据
print("加载点云文件：test_scan_cut.ply")
pcd = o3d.io.read_point_cloud("test_scan_cut.ply")
print("原始点云共有点数：", np.array(pcd.points).shape[0])

# 可视化原始点云
print("显示原始点云")
o3d.visualization.draw_geometries([pcd])

# Voxel 体素下采样
print("进行体素下采样...")
down_pcd = pcd.voxel_down_sample(voxel_size=0.008)
print("下采样后点云共有点数：", np.array(down_pcd.points).shape[0])

# 统计滤波
print("进行统计滤波...")
cl, ind = down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
clean_pcd = down_pcd.select_by_index(ind)
print("滤波后点云共有点数：", np.array(clean_pcd.points).shape[0])

# 法线估计
print("进行法线估计...")
clean_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
clean_pcd.orient_normals_consistent_tangent_plane(k=10)
print("法线估计完成")

# 可视化处理后的点云
print("显示处理后的点云")
o3d.visualization.draw_geometries([clean_pcd], point_show_normal=False)

# 保存处理后的点云
output_filename = "backpack_processed.ply"
o3d.io.write_point_cloud(output_filename, clean_pcd)
print(f"处理后的点云已保存为：{output_filename}")
