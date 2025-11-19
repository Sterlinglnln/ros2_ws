import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ClientNode(Node):
    def __init__(self, node_name='clientnode'):
        super().__init__(node_name=node_name)
        self.req = SetBool.Request()
        self.client = self.create_client(SetBool, '/testservice')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.create_timer(1.0, self.send_request)

    def send_request(self):
        self.req.data = not self.req.data
        req = self.client.call_async(self.req)
        req.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        if response is not None:
            self.get_logger().info(f'Service response: success={response.success}, message="{response.message}"')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
