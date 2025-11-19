import rclpy
from rclpy.node import Node
from test_msgs.action import Fibonacci
from rclpy.action import ActionClient

class ActionClientNode(Node):
    def __init__(self, node_name='actionclientnode'):
        super().__init__(node_name=node_name)
        self._logger = self.get_logger()
        self.actionclient = ActionClient(self, Fibonacci, '/fibonacci')
        while not self.actionclient.wait_for_server(timeout_sec=10.0):
            print('Action server not available, waiting...')
        self.send_goal(10)

    def send_goal(self, order = 10):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.send_goal_future = self.actionclient.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._logger.info(f'Received feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.info('Goal rejected :(')
            return

        self._logger.info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self._logger.info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ActionClientNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()