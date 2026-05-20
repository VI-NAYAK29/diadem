#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

class ParamSetter(Node):
    def __init__(self):
        super().__init__('param_setter')
        self.cli = self.create_client(SetParameters, '/camera/camera/set_parameters')
        self.timer = self.create_timer(1.0, self.check_and_set_param)
        self.get_logger().info('ParamSetter node has been started.')

    def check_and_set_param(self):
        if not self.cli.service_is_ready():
            self.get_logger().warn('/camera/camera/set_parameters service not available yet')
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='depth_module.emitter_enabled', type_=Parameter.Type.INTEGER, value=0).to_parameter_msg()
        ]

        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.parameter_set_callback)

    def parameter_set_callback(self, future):
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info('Successfully set parameter: depth_module.emitter_enabled to 0')
                self.timer.cancel()
            else:
                self.get_logger().error('Failed to set parameter: depth_module.emitter_enabled')
        except Exception as e:
            self.get_logger().error(f'Exception while setting parameter: {e}')

def main(args=None):
    rclpy.init(args=args)
    param_setter = ParamSetter()
    rclpy.spin(param_setter)
    param_setter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()