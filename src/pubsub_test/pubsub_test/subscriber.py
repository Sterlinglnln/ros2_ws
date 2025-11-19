import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubNode(Node):
    def __init__(self, node_name='subnode'):
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()
        self.sub = self.create_subscription(String, '/hello', self.process_msg, 5)

    def process_msg(self, msg):
        self.logger.info(f'receive message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()