import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill

class ClearTurtle(Node):
    def __init__(self):
        super().__init__('clear_turtle')
        self.client = self.create_client(Kill, 'kill')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.call_service()

    def call_service(self):
        self.req = Kill.Request()
        self.req.name = 'turtle1'
        req = self.client.call_async(self.req)
        req.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        if response is not None:
            self.get_logger().info('Turtle cleared successfully.')
        else:
            self.get_logger().error('Failed to clear turtle.')

def main(args=None):
    rclpy.init(args=args)
    node = ClearTurtle()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
