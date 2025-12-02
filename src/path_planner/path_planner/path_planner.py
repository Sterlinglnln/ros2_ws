import rclpy
import numpy as np
import cv2
import tf_transformations
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray

from path_planner.Astar import AStar, Point

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.declare_parameter('use_sim_time', False)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # 地图相关
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_height = None  # 地图高度
        self.map_width = None   # 地图宽度
        self.resolution = None   # 地图分辨率
        self.origin = None      # 地图原点
        self.map = None         # 存储地图数组
        self.astar = None      # A*算法实例

        # 监听 tf 树
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布规划与实际路径
        self.plan_path_pub = self.create_publisher(Path, 'plan_path', 10)
        self.robot_path_pub = self.create_publisher(Path, 'robot_path', 10)

        # 初始化目标点列表
        self.target_points = []
        self.current_target_index = 0   # 当前目标点
        
        # 订阅接受目标点的话题
        self.target_subscriber = self.create_subscription(
            Float64MultiArray,
            'target_points_topic',
            self.target_callback,
            10
        )
        self.robot_points = []  # 记录机器人路径点

        # 创建导航的计时器
        self.timer = self.create_timer(1.0, self.navigation)

    def target_callback(self, msg):
        # 接收目标点并更新目标点列表
        new_target_points = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
        self.target_points.extend(new_target_points)

    def speed_controller(self, p, tp, c):
        # 计算和发布小车的速度
        dx = tp.x - p.x
        dy = tp.y - p.y
        td = np.arctan2(dy, dx)
        vel_msg = Twist()
        direction, angledist = self.calcangle(c, td)
        if angledist < 0.3:
            vel_msg.angular.z = direction * angledist * 0.5
            vel_msg.linear.x = np.clip((abs(dx) + abs(dy)) * 0.5, 0, 0.5)
        else:
            vel_msg.angular.z = direction * angledist if angledist < 0.2 else direction * 0.2
            vel_msg.linear.x = 0.01
        self.vel_pub.publish(vel_msg)

    def map_callback(self, map_data):
        # 接受地图
        self.map_height = map_data.info.height
        self.map_width = map_data.info.width
        self.resolution = map_data.info.resolution
        self.origin = map_data.info.origin.position

        # 将地图数据转换为数组
        mp = np.array(map_data.data).reshape(self.map_height, self.map_width)

        # 进行膨胀处理
        kernel = np.ones((5, 5), np.uint8)
        mp = cv2.dilate(mp.astype(np.uint8), kernel, iterations=1)
        self.map = mp
        self.astar = AStar(mp)

    def navigation(self):
        # 等待地图和A *算法初始化
        if self.astar is None or self.map is None:
            self.get_logger().info('Waiting for map data...')
            return
        
        try:
            # 通过 TF 获取机器人的位置
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            self.get_logger().info('TF lookup failed')
            return
        
        mapx = t.transform.translation.x
        mapy = t.transform.translation.y
        q = [t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w]
        _,_,c = tf_transformations.euler_from_quaternion(q)
        x, y = self.map2index(mapx, mapy)
        robot_point = Point(int(x), int(y))
        self.robot_path(robot_point)
        if self.target_points==[]:
            self.get_logger().info('No target points available')
            return
        if self.current_target_index >= len(self.target_points):
            self.get_logger().info('All target points have been reached, waiting for new targets...')
            return
        if self.current_target_index < len(self.target_points):
            tx, ty = self.map2index(* self.target_points[self.current_target_index])
        start_point = Point(int(x), int(y))
        goal_point = Point(int(tx), int(ty))
        if abs(x - tx) + abs(y - ty) < 3:   # 到达目标点的条件
            self.get_logger().info(f'到达第 {self.current_target_index} 个目标点...')
            self.vel_pub.publish(Twist())
            self.astar.path = []
            self.current_target_index += 1     # 更新下一个目标点
        else:
            self.get_logger().info(f'The target position list is {self.target_points}')
            self.get_logger().info(f'The index is {self.current_target_index}')
            self.get_logger().info(f'The current position of robot in array is {[x, y]}')
            self.get_logger().info(f'The target position in array is {[tx, ty]}')
            if self.astar.path == []:
                self.astar.Run(start_point, goal_point)
            else:
                self.astar.update(start_point)
                self.get_logger().info(f'Path has already been updated.')
            if len(self.astar.path) > 0:
                self.plan_path(self.astar.path)
            if len(self.astar.path) > 5:
                self.speed_controller(self.astar.path[0], self.astar.path[5], c)
            elif len(self.astar.path) > 1:
                self.speed_controller(self.astar.path[0], self.astar.path[1], c)

    def plan_path(self, path_points):
        # 发布路径规划
        path_record = Path()
        current_time = self.get_clock().now()
        for point in path_points:
            x,y = self.index2map(point.x, point.y)
            pose = PoseStamped()
            pose.header.stamp = current_time.to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_record.header.stamp = current_time.to_msg()
            path_record.header.frame_id = 'map'
            path_record.poses.append(pose)
        self.plan_path_pub.publish(path_record)

    def robot_path(self, robot_point):
        # 发布机器人实际路径
        path_record = Path()
        current_time = self.get_clock().now()
        if len(self.robot_points)==0 or robot_point!=self.robot_points[-1]:
            self.robot_points.append(robot_point)
        for point in self.robot_points:
            x,y = self.index2map(point.x, point.y)
            pose = PoseStamped()
            pose.header.stamp = current_time.to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_record.header.stamp = current_time.to_msg()
            path_record.header.frame_id = 'map'
            path_record.poses.append(pose)
        self.robot_path_pub.publish(path_record)

    def map2index(self, mapx, mapy):
        # 地图坐标转换为数组索引
        x = (mapx - self.origin.x) / self.resolution
        y = (mapy - self.origin.y) / self.resolution
        return x, y
    
    def index2map(self, x, y):
        # 数组索引转换为地图坐标
        return x * self.resolution + self.origin.x, y * self.resolution + self.origin.y
    
    def calcangle(self, start_theta, end_theta):
        # 计算角度差
        # 归一化到 [-pi, pi]
        if start_theta < 0: start_theta = np.pi * 2 + start_theta
        if end_theta < 0: end_theta = np.pi * 2 + end_theta
        theta = end_theta - start_theta
        if theta > 0:
            if theta > np.pi:
                return -1, np.pi * 2 - theta
            else:
                return 1, theta
        if theta < 0:
            if theta > -np.pi:
                return -1, abs(theta)
            else:
                return 1, np.pi * 2 + theta
        return 0, 0
    
def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
