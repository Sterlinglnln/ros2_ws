import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import string
import random

class SpawnNode(Node):
    def __init__(self):
        super().__init__('spawn_node_client')
        self.num = self.declare_parameter(
            'num', int('10')).get_parameter_value().integer_value
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=5.0):
            print('Service not available, waiting again...')
        self.t = self.create_timer(0.1, self.request)

    def generate_safe_name(self):
        first = random.choice(string.ascii_letters)
        rest = ''.join(random.choices(string.ascii_letters + string.digits, k = 9))
        return first + rest

    def request(self):
        if self.num == 0:
            self.get_logger().info(f'乌龟创建完成！')
            self.destroy_timer(self.t)
            self.destroy_node()
            raise Exception('done!')
        req = Spawn.Request()
        req.x = random.random() * 11
        req.y = random.random() * 11
        req.theta = random.random() * 6.28
        req.name = self.generate_safe_name()
        self.req = self.client.call_async(req)
        self.req.add_done_callback(self.resp_callback)

    def resp_callback(self, future):
        resp = future.result()
        if resp and resp.name:
            self.get_logger().info(f'乌龟 {resp.name} 创建成功！')
            self.num -= 1

def main(args=None):
    rclpy.init(args=args)
    node = SpawnNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
