import rclpy
import math
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw):
    # 将欧拉角转换为四元数
    roll = roll / 2.0
    pitch = pitch / 2.0
    yaw = yaw / 2.0
    ci = math.cos(roll)
    si = math.sin(roll)
    cj = math.cos(pitch)
    sj = math.sin(pitch)
    ck = math.cos(yaw)
    sk = math.sin(yaw)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q: list[float] = [0.0] * 4
    q[0] = cj * sc - sj * cs # x
    q[1] = cj * ss + sj * cc # y
    q[2] = cj * cs - sj * sc # z
    q[3] = cj * cc + sj * ss # w
    return q

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.x = self.declare_parameter('x', 3.0).get_parameter_value().double_value
        self.y = self.declare_parameter('y', 4.0).get_parameter_value().double_value
        self.z = self.declare_parameter('z', 5.0).get_parameter_value().double_value
        self.roll = self.declare_parameter('roll', 0.0).get_parameter_value().double_value
        self.pitch = self.declare_parameter('pitch', 0.0).get_parameter_value().double_value
        self.yaw = self.declare_parameter('yaw', 0.0).get_parameter_value().double_value
        self.child_frame = self.declare_parameter('child_frame', 'frame_A').get_parameter_value().string_value
        self.parent_frame = self.declare_parameter('parent_frame', 'World').get_parameter_value().string_value
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.logger = self.get_logger()
        self.pub_transform()

    def pub_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        quat = quaternion_from_euler(self.roll/180*math.pi, self.pitch/180*math.pi, self.yaw/180*math.pi)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.logger.info(f'publish static tf: {t}')
        self.tf_static_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
