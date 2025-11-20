import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

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

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_tf2_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.num = 0
        self.create_timer(0.03, self.pub_transforms)

    def pub_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'frame_A'
        t.child_frame_id = 'frame_B'

        theta = self.num / 180.0 * math.pi
        t.transform.translation.x = math.cos(theta) * 2.0
        t.transform.translation.y = math.sin(theta) * 2.0
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, theta + math.pi / 2.0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.num += 1
        if self.num >= 360:
            self.num = 0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
