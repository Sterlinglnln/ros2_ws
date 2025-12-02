import math
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from turtlesim.msg import Pose

class MoveToPoint(Node):
    def __init__(self):
        super().__init__('move_to_point')
        x = self.declare_parameter('x', 2.0).get_parameter_value().double_value
        y = self.declare_parameter('y', 2.0).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        self.pose_hub = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            1
        )

        self.turtle_frame = False
        self.point_frame = False

        self.timer1 = self.create_timer(1.0, self.on_timer1)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_target(x, y)

    def on_timer1(self):
        from_frame_rel = 'point'
        to_frame_rel = 'turtle1'

        if self.turtle_frame and self.point_frame:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                now
            )

            msg = Twist()
            scale_rotation_rate = 1.0
            msg.angular.z = scale_rotation_rate * math.atan2(
                trans.transform.translation.y,
                trans.transform.translation.x)
            
            scale_forward_speed = 0.5
            msg.linear.x = scale_forward_speed * math.sqrt(
                trans.transform.translation.x ** 2 +
                trans.transform.translation.y ** 2
            )

            self.publisher.publish(msg)

    def pose_callback(self, msg):
        trnasform = TransformStamped()
        trnasform.header.stamp = self.get_clock().now().to_msg()
        trnasform.header.frame_id = 'World'
        trnasform.child_frame_id = 'turtle1'
        trnasform.transform.translation.x = msg.x
        trnasform.transform.translation.y = msg.y
        trnasform.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        trnasform.transform.rotation.x = q[0]
        trnasform.transform.rotation.y = q[1]
        trnasform.transform.rotation.z = q[2]
        trnasform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(trnasform)
        self.turtle_frame = True

    def pub_target(self, x=2.0, y=2.0):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'World'
        transform.child_frame_id = 'point'
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.tf_static_broadcaster.sendTransform(transform)
        self.point_frame = True

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoint()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
