import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time

class FrameSubscriber(Node):
    def __init__(self):
        super().__init__('tf2_frame_listener')
        self.logger = self.get_logger()
        self.target_frame = self.declare_parameter(
            'target_frame', 'frame_B').get_parameter_value().string_value
        self.source_frame = self.declare_parameter(
            'source_frame', 'frame_A').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.logger = self.get_logger()
        self.timer = self.create_timer(1.0, self.show_tf)

    def show_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time()
            )
            self.logger.info(f"parent frame: {t.header.frame_id}, child frame: {t.child_frame_id}, transform is {t.transform}")
        except Exception as e:
            print(e)
            return
        
def main(args=None):
    rclpy.init(args=args)
    node = FrameSubscriber()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()
