import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen

class PenNode(Node):
    def __init__(self):
        super().__init__('set_pen_node')

        name  = self.declare_parameter('name', str('turtle')).get_parameter_value().string_value
        r     = self.declare_parameter('r', int(0)).get_parameter_value().integer_value
        g     = self.declare_parameter('g', int(0)).get_parameter_value().integer_value
        b     = self.declare_parameter('b', int(0)).get_parameter_value().integer_value
        width = self.declare_parameter('width', int(5)).get_parameter_value().integer_value

        self.client = self.create_client(SetPen, name + '/set_pen')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"⏳ {name}/set_pen 服务未就绪，等待中…")

        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = r, g, b, width, 0
        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = PenNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
