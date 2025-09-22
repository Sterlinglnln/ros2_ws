import rclpy
from rclpy.node import Node
import serial
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # initialize serial connection
        self.ser = serial.Serial(
            port=self.get_parameter('serial_port').value,
            baudrate=self.get_parameter('baud_rate').value,
            timeout=1
        )
        if not self.ser.is_open:
            self.ser.open()
        self.get_logger().info(f"Serial port {self.ser.name} opened.")
        
        # create subcriber of topic
        self.cmd_sub = self.create_subscription(
            str,
            'motor_cmd',
            self.cmd_callback,
            10
        )
        
    def cmd_callback(self, msg):
        """deal with the command received from motor_cmd topic"""
        if msg == "forward":
            self.ser.write(b'forward\n')
            self.get_logger().info("Command sent: forward")
        elif msg == "backward":
            self.ser.write(b'backward\n')
            self.get_logger().info("Command sent: backward")
        elif msg == "stop":
            self.ser.write(b'stop\n')
            self.get_logger().info("Command sent: stop")
    
    def __del__(self):
        if self.ser.is_open:
            self.ser.close()
            
def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
       