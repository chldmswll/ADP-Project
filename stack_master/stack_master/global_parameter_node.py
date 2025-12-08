import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class GlobalParams(Node):
    def __init__(self):
        super().__init__(
            'global_parameters',
            allow_undeclared_parameters=False,  # 반드시 False
            automatically_declare_parameters_from_overrides=True
        )  

        self.add_on_set_parameters_callback(self.on_param_change)

    def on_param_change(self, params):
        for p in params:
            self.get_logger().info(
                f"PARAM SET: {p.name} = {p.value}"
            )
        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = GlobalParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
