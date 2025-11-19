import rclpy
from rclpy.node import Node
import rclpy.parameter
from typing import List
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):
    def __init__(self):
        super().__init__('param_node')
        self.logger = self.get_logger()
        # 1. 创建参数
        self.declare_parameter('number', 10)
        self.declare_parameter('string', 'Hello')
        # 2. 测试是否存在参数
        if self.has_parameter('string'):
            self.logger.info(f'has parameter string')
        # 3. 获取参数
        self.length = self.declare_parameter('length', 3).get_parameter_value().integer_value
        self.logger.info(f'length: {self.length}')
        # 4. 设置参数回调函数
        self.add_on_set_parameters_callback(self.show)

        self.updateparam()

    def updateparam(self):
        # 取得参数
        number_param = self.get_parameter('number').get_parameter_value().integer_value
        self.logger.info(f'number: {number_param}')
        # 创建并设置参数
        new_param = rclpy.parameter.Parameter('number', rclpy.Parameter.Type.INTEGER, 1)
        all_new_params = [new_param]
        self.logger.info(f'Change param number to 1')
        self.set_parameters(all_new_params)

    def show(self, params: List[rclpy.parameter.Parameter]) -> SetParametersResult:
        for i in params:
            if i.type_ == rclpy.Parameter.Type.INTEGER:
                value = i.value
            else:
                value = i.value
            self.logger.info(f'Param {i.name} changed to {value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    try:
        rclpy.spin(node)
    except Exception:
        rclpy.shutdown()
        exit(0)


if __name__ == '__main__':
    main()
