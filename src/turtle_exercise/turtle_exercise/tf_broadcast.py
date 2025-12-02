import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.name = self.declare_parameter('name', str('turtle1')).get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_hub = self.create_subscription(
            Pose,
            self.name + '/pose',
            self.pose_callback,
            1
        )
    
    def pose_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'World'
        transform.child_frame_id = self.name
        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
