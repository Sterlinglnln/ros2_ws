import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
import math
import time
from rclpy.action import ActionServer
from interface_exercise.action import Move

class TeleportTurtleClient(Node):
    def __init__(self):
        super().__init__('teleop_turtle_client')
        self._action_server = ActionServer(
            self, Move, 'turtle_move', self.execute_callback)
        self.server_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.server_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TeleportAbsolute.Request()

    def calculate_polygon_vertices(self, base_length, num_sides, fix_x, fix_y):
        angle_increment = 360 / num_sides
        angle_radians = math.radians(angle_increment / 2)
        sin_half_thelta = math.sin(angle_radians)
        radius = base_length / (2 * sin_half_thelta)
        self.vertics = []

        for i in range(num_sides - 1):
            angle_deg = (i+1) * angle_increment
            angle_rad = math.radians(angle_deg)
            x = fix_x - radius + radius * math.cos(angle_rad)
            y = fix_y + radius * math.sin(angle_rad)
            self.vertics.append((x, y))
        self.vertics.append((fix_x, fix_y))

    def send_request(self, x, y, thelta):
        self.req.x = x
        self.req.y = y
        self.req.theta = thelta

        future = self.server_client.call_async(self.req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
            
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Move.Feedback()
        feedback_msg.partial_sequence = []
        self.calculate_polygon_vertices(
            1.5, goal_handle.request.order, 5.5, 5.5)
        
        for i in range(goal_handle.request.order):
            feedback_msg.partial_sequence.append(self.vertics[i][0])
            feedback_msg.partial_sequence.append(self.vertics[i][1])
            self.send_request(self.vertics[i][0], self.vertics[i][1], 1.57)
            self.get_logger().info('Feedback: The turtle has moved to: {}'.format(self.vertics[i]))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Move.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
    
def main(args=None):
    rclpy.init(args=args)

    node = TeleportTurtleClient()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
