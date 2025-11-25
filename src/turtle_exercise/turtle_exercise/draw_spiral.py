import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedPublisher(Node):
    def __init__(self):
        super().__init__('draw_spiral')
        self.turtle_name = self.declare_parameter(
            'turtle_name', 'turtle1').get_parameter_value().string_value
        self.publisher = self.create_publisher(Twist, f'{self.turtle_name}/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.cmd = Twist()
        self.cmd.linear.x = 0
        self.cmd.angular.z = 3.0

    def on_timer(self):
        self.cmd.linear.x += 0.03
        self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()
    try:
        rclpy.spin(speed_publisher)
    except Exception:
        speed_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
