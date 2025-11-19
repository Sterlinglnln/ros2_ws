import rclpy
from rclpy.node import Node
from test_msgs.action import Fibonacci
from rclpy.action import ActionServer
import time

class ActionserverNode(Node):
    def __init__(self, node_name='actionservernode'):
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()
        self.actionserver = ActionServer(
            self,
            Fibonacci,
            '/fibonacci',
            self.execute_callback
        )
        self.logger.info('action server started...')

    def execute_callback(self, goal_handle):
        self.logger.info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            partial_sequence.append(partial_sequence[i] + partial_sequence[i - 1])
            progress = [i+1, goal_handle.request.order]
            self.logger.info(f'Feedback: {progress}')
            feedback_msg.sequence = progress
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = partial_sequence
        self.logger.info('Goal succeeded!')
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = ActionserverNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
