import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

class SpawnNode(Node):
    def __init__(self):
        super().__init__('spawn_node')
        
        x = self.declare_parameter('x', float('5.5')).get_parameter_value().double_value
        y = self.declare_parameter('y', float('5.5')).get_parameter_value().double_value
        name = self.declare_parameter('name', str('turtle')).get_parameter_value().string_value

        self.spawn = self.create_client(Spawn, '/spawn')
        while not self.spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.call_spawn_service(x, y, name)
        self.cmd_publisher = self.create_publisher(Twist, name + '/cmd_vel', 10)
        self.create_timer(0.1, self.pub_vel)

    def pub_vel(self):
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.75
        self.cmd_publisher.publish(cmd)

    def call_spawn_service(self, x, y, name):
        req = Spawn.Request()
        req.name = name
        req.x = x
        req.y = y
        req.theta = 0.0
        future = self.spawn.call_async(req)

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
