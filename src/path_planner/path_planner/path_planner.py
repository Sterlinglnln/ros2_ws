import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

from path_planner.Astar import Point, AStar


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ----------------- 速度发布 ----------------- #
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ----------------- 订阅地图 ----------------- #
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=map_qos
        )

        # 地图相关
        self.mapheight = None
        self.mapwidth = None
        self.resolution = None
        self.origin = None
        self.mp = None          # 原始 map（OccupancyGrid 的 data reshape 后）
        self.astar: AStar | None = None

        # ----------------- TF 监听 ----------------- #
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----------------- 路径可视化 ----------------- #
        self.plan_path_pub = self.create_publisher(Path, '/plan_path', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)

        self.robot_points: list[Point] = []

        # ----------------- 目标点管理 ----------------- #
        self.target_points: list[tuple[float, float]] = []
        self.current_target_index: int = 0

        self.target_subscriber = self.create_subscription(
            Float64MultiArray,
            'target_points_topic',
            self.target_callback,
            10
        )

        # ----------------- 导航计时器 ----------------- #
        self.timer = self.create_timer(1.0, self.navigation)

        self.get_logger().info("PathPlanner 节点启动完成")

    # ==================================================
    #                 目标点相关
    # ==================================================

    def target_callback(self, msg: Float64MultiArray):
        """接收目标点列表，每两个值为 (x, y)。"""
        if len(msg.data) % 2 != 0:
            self.get_logger().warn("接收到的目标点数量不是偶数，忽略最后一个残缺点")
        new_target_points = []
        for i in range(0, len(msg.data) - 1, 2):
            new_target_points.append((float(msg.data[i]), float(msg.data[i + 1])))

        self.target_points.extend(new_target_points)
        self.get_logger().info(f"收到新的目标点 {new_target_points}，当前总目标点数：{len(self.target_points)}")

    # ==================================================
    #                  地图回调 & 预处理
    # ==================================================

    def map_callback(self, map_data: OccupancyGrid):
        """接收 /map，并预处理为给 A* 使用的二值占据栅格。"""
        self.mapheight = map_data.info.height
        self.mapwidth = map_data.info.width
        self.resolution = map_data.info.resolution
        self.origin = map_data.info.origin.position

        raw = np.array(map_data.data, dtype=np.int16).reshape(self.mapheight, self.mapwidth)
        self.mp = raw

        uniques = np.unique(raw)
        self.get_logger().info(
            f"map received: h={self.mapheight}, w={self.mapwidth}, "
            f"min={raw.min()}, max={raw.max()}, unique={uniques[:10]}"
        )

        # --------- 处理 unknown (-1) ---------
        # 如果你想把 unknown 当障碍，改为：raw[raw < 0] = 100
        raw[raw < 0] = 0  # 这里先当作空闲

        # --------- 概率值二值化：>=50 认为是障碍 ----------
        bin_map = np.zeros_like(raw, dtype=np.uint8)
        bin_map[raw >= 50] = 255  # 高于一定阈值的都当障碍

        # --------- 膨胀操作，扩大障碍，避免贴边 ---------
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(bin_map, kernel, iterations=1)

        # --------- 转为 0 / 100 地图给 A* 用 -------
        astar_map = np.zeros_like(dilated, dtype=np.uint8)
        astar_map[dilated > 0] = 100

        self.astar = AStar(astar_map, obstacle_threshold=100)
        self.get_logger().info("A* 地图初始化完成，可以进行路径规划")

    # ==================================================
    #                      导航逻辑
    # ==================================================

    def navigation(self):
        """定时执行的导航主循环。"""
        # 1. 地图与 A* 是否就绪
        if self.astar is None:
            self.get_logger().info('等待地图数据和 A* 初始化...')
            return

        # 2. 获取机器人在 map 下的 pose
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except Exception:
            self.get_logger().error('机器人定位失败，请在 RViz2 中初始化机器人位姿（2D Pose Estimate）！')
            return

        mapx = tf.transform.translation.x
        mapy = tf.transform.translation.y
        quat = [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        x_idx, y_idx = self.map2index(mapx, mapy)

        robot_point = Point(int(x_idx), int(y_idx))
        self.robot_path(robot_point)

        # 3. 是否有目标点
        if not self.target_points:
            self.get_logger().warn('没有目标点，请设置目标点！')
            return

        # 当前目标是否超出范围
        if self.current_target_index >= len(self.target_points):
            self.get_logger().warn('已经完成所有点，等待新目标！')
            self.vel_pub.publish(Twist())
            return

        # 当前目标点（map 坐标 -> index）
        tx, ty = self.map2index(*self.target_points[self.current_target_index])

        # 4. 判断是否到达当前目标点（在栅格索引上的距离）
        if abs(x_idx - tx) + abs(y_idx - ty) < 3:
            human_idx = self.current_target_index + 1
            self.get_logger().info(f'到达第 {human_idx} 个目标点.')
            self.vel_pub.publish(Twist())
            self.astar.path = []
            self.current_target_index += 1
            return

        # 5. 仍未到达，继续规划 / 更新路径并控制
        self.get_logger().info(f"The target position list is: {self.target_points}")
        self.get_logger().info(f"The index is: {self.current_target_index}")

        start = Point(int(x_idx), int(y_idx))
        goal = Point(int(tx), int(ty))

        self.get_logger().info(f"The robot current position in array is: {[x_idx, y_idx]}")
        self.get_logger().info(f"The robot target position in array is: {[tx, ty]}")

        # 首次规划
        if not self.astar.path:
            self.astar.Run(start, goal)
            if not self.astar.path:
                self.get_logger().error("A* 无法找到路径，请检查起点/终点是否在障碍上或地图是否被堵死")
                self.vel_pub.publish(Twist())
                return
            # 如果你要看 matplotlib 路径，在这里手动调 self.astar.show()
            # self.astar.show()
        else:
            # 已有路径，做局部更新
            self.astar.update(start)
            self.get_logger().info("路径更新完毕！")

        # 6. 发布规划路径
        if self.astar.path:
            self.plan_path(self.astar.path)

        # 7. 基于路径做简单速度控制
        if len(self.astar.path) > 5:
            self.speed_controller(self.astar.path[0], self.astar.path[5], yaw)
        elif len(self.astar.path) > 1:
            self.speed_controller(self.astar.path[0], self.astar.path[-1], yaw)

    # ==================================================
    #                路径发布 / 轨迹记录
    # ==================================================

    def plan_path(self, path_points: list[Point]):
        """发布规划路径到 /plan_path。"""
        path_record = Path()
        current_time = self.get_clock().now().to_msg()
        path_record.header.stamp = current_time
        path_record.header.frame_id = 'map'

        for p in path_points:
            x, y = self.index2map(p.x, p.y)
            pose = PoseStamped()
            pose.header.stamp = current_time
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_record.poses.append(pose)

        self.plan_path_pub.publish(path_record)

    def robot_path(self, robot_point: Point):
        """发布机器人实际轨迹到 /robot_path。"""
        if not self.robot_points or robot_point != self.robot_points[-1]:
            self.robot_points.append(robot_point)

        path_record = Path()
        current_time = self.get_clock().now().to_msg()
        path_record.header.stamp = current_time
        path_record.header.frame_id = 'map'

        for p in self.robot_points:
            x, y = self.index2map(p.x, p.y)
            pose = PoseStamped()
            pose.header.stamp = current_time
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_record.poses.append(pose)

        self.robot_path_pub.publish(path_record)

    # ==================================================
    #                        工具函数
    # ==================================================

    def speed_controller(self, p: Point, tp: Point, current_yaw: float):
        """根据当前点与局部目标点控制机器人速度。"""
        dx = tp.x - p.x
        dy = tp.y - p.y
        td = np.arctan2(dy, dx)

        vel_msg = Twist()
        direction, angledist = self.calcangle(current_yaw, td)

        if angledist < 0.3:
            vel_msg.angular.z = direction * angledist * 0.5
            vel_msg.linear.x = float(np.clip((abs(dx) + abs(dy)) * 0.05, 0, 0.5))
        else:
            vel_msg.angular.z = direction * (angledist if angledist < 0.2 else 0.2)
            vel_msg.linear.x = 0.01

        self.vel_pub.publish(vel_msg)

    def map2index(self, mapx: float, mapy: float):
        """map 坐标 -> 栅格索引。"""
        x = (mapx - self.origin.x) / self.resolution
        y = (mapy - self.origin.y) / self.resolution
        return x, y

    def index2map(self, x: int, y: int):
        """栅格索引 -> map 坐标。"""
        return self.resolution * x + self.origin.x, self.resolution * y + self.origin.y

    @staticmethod
    def calcangle(start_theta: float, end_theta: float):
        """计算从 start_theta 转到 end_theta 的最小夹角和方向。"""
        # change -pi～pi to 0～2pi
        if start_theta < 0:
            start_theta = 2 * np.pi + start_theta
        if end_theta < 0:
            end_theta = 2 * np.pi + end_theta

        theta = end_theta - start_theta
        if theta > 0:
            if theta > np.pi:
                return -1, 2 * np.pi - theta
            else:
                return 1, theta
        if theta < 0:
            if theta > -np.pi:
                return -1, abs(theta)
            else:
                return 1, 2 * np.pi + theta
        return 0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"PathPlanner 异常退出: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
