import math
import rclpy
from rclpy.node import Node
from gps_nav_interfaces.msg import OpticalFlow


class PreprocessNode(Node):
    def __init__(self, **kwargs):
        super().__init__('flow_preprocess', **kwargs)
        for name, value in [('flow_scale_x',0.0),('flow_scale_y',0.0),
                            ('calibration_valid',False),('invert_x',False),('invert_y',False),
                            ('movement_deadband',0.0),('smoothing_alpha',1.0)]:
            self.declare_parameter(name,value)
        if self.get_parameter('smoothing_alpha').value != 1.0:
            self.get_logger().warning('nonunity smoothing_alpha disables metric output: '
                                      'smoothing displacement increments biases dead reckoning')
        self.pub = self.create_publisher(OpticalFlow, '/optical_flow', 100)
        self.create_subscription(OpticalFlow, '/optical_flow/raw', self.on_raw, 100)

    def on_raw(self, raw):
        out = OpticalFlow()
        out.header = raw.header; out.raw_x = raw.raw_x; out.raw_y = raw.raw_y
        out.quality = raw.quality; out.source = raw.source
        sx = self.get_parameter('flow_scale_x').value
        sy = self.get_parameter('flow_scale_y').value
        valid = (self.get_parameter('calibration_valid').value and
                 self.get_parameter('smoothing_alpha').value == 1.0 and
                 math.isfinite(sx) and math.isfinite(sy) and sx > 0 and sy > 0)
        if valid:
            dx = raw.raw_x * sx * (-1 if self.get_parameter('invert_x').value else 1)
            dy = raw.raw_y * sy * (-1 if self.get_parameter('invert_y').value else 1)
            deadband = max(0.0, self.get_parameter('movement_deadband').value)
            if abs(dx) < deadband: dx = 0.0
            if abs(dy) < deadband: dy = 0.0
            out.displacement_x_m = dx; out.displacement_y_m = dy
            out.metric_valid = True
        self.pub.publish(out)


def main():
    rclpy.init(); node = PreprocessNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
