import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk')
        self.turn = False if random.random() < 0.5 else True
        self.current_msg = Twist()
        self.walk = self.create_timer(3, self.random_speed)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def random_speed(self):
        msg = Twist()
        if self.turn:
            sign = 1 if random.random() < 0.5 else -1
            msg.angular.z = random.uniform(1.0, 2 * 1.0) *sign
            msg.linear.x = 0.0
            self.walk.cancel()
            self.walk = self.create_timer(random.uniform(0, 2.0), self.random_speed)
        else:
            msg.angular.z = 0.0
            msg.linear.x = 1.0
            self.walk.cancel()
            bu = random.uniform(2.5, 4.5)
            self.walk = self.create_timer(bu, self.random_speed)
        self.turn = not self.turn
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalk()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
