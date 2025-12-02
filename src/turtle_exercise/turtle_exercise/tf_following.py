import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn


class TFFollowingNode(Node):
    def __init__(self):
        super().__init__('turtle_following')

        # 参数：跟随哪个坐标系
        self.source_frame = self.declare_parameter(
            'source_frame', 'turtle1'
        ).get_parameter_value().string_value

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.turtle_spawned = False
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # 等待 spawn 服务可用
        self.get_logger().info("Waiting for /spawn service...")
        self.spawn_client.wait_for_service()

        # 发送 turtle2 spawn 请求
        self.call_spawn_service()

        # 正确的 topic 名！必须是 cmd_vel
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # 定时器
        self.timer = self.create_timer(0.5, self.on_timer)

    def call_spawn_service(self):
        req = Spawn.Request()
        req.name = 'turtle2'
        req.x = 3.0
        req.y = 2.0
        req.theta = 0.0

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response and response.name == 'turtle2':
                self.get_logger().info("turtle2 spawned successfully!")
                self.turtle_spawned = True
            else:
                self.get_logger().warn("Spawn failed, retrying...")
                self.call_spawn_service()
        except Exception as e:
            self.get_logger().error(f"Spawn service call failed: {e}")
            self.call_spawn_service()

    def on_timer(self):
        if not self.turtle_spawned:
            self.get_logger().info('Waiting for turtle2 to be spawned...')
            return

        try:
            now = rclpy.time.Time()
            # frame: source_frame → turtle2
            trans = self.tf_buffer.lookup_transform(
                'turtle2',
                self.source_frame,
                now
            )
        except Exception:
            self.get_logger().warn("TF not ready...")
            return

        msg = Twist()

        # 角速度：指向目标
        msg.angular.z = math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x
        )

        # 线速度：前进
        msg.linear.x = 0.8 * math.sqrt(
            trans.transform.translation.x ** 2 +
            trans.transform.translation.y ** 2
        )

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFFollowingNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
