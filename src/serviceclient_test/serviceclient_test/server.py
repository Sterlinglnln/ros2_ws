import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ServerNode(Node):
    def __init__(self, node_name='servernode'):
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()
        self.server = self.create_service(
            SetBool,
            '/testservice',
            self.response_callback
        )

    def response_callback(self, request, response):
        reqdata = request.data
        if reqdata:
            response.success = True
            response.message = 'Request data was True'
        else:
            response.success = False
            response.message = 'Request data was False'
        self.logger.info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
