import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubNode(Node):
    def __init__(self, node_name='pubnode'):
        super().__init__(node_name=node_name)
        self.pub = self.create_publisher(String, '/hello', 5)
        self.msg = String()
        self.num = 0
        self.logger = self.get_logger()
        self.create_timer(1.0, self.pubmsg)

    def pubmsg(self):
        self.msg.data = f'the {self.num}th hello message.'
        self.num += 1
        self.pub.publish(self.msg)
        self.logger.info(f'send message: {self.msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()