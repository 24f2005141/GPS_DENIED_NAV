import rclpy
from rclpy.node import Node
from gps_nav_interfaces.msg import OpticalFlow
from std_msgs.msg import String
from .hereflow_reader import HereFlowReader


class HereFlowNode(Node):
    def __init__(self, **kwargs):
        super().__init__('hereflow', **kwargs)
        for name, value in [('serial_port','/dev/ttyUSB0'),('baud_rate',19200),
                            ('serial_timeout',0.01),('reconnect_interval',2.0),
                            ('byte_order','little'),('sensor_stale_s',0.6)]:
            self.declare_parameter(name, value)
        self.reader = HereFlowReader(
            self.get_parameter('serial_port').value,
            self.get_parameter('baud_rate').value,
            self.get_parameter('serial_timeout').value,
            self.get_parameter('reconnect_interval').value,
            self.get_parameter('byte_order').value,
            self.get_parameter('sensor_stale_s').value)
        self.last_reported_error = ''
        self.pub = self.create_publisher(OpticalFlow, '/optical_flow/raw', 100)
        self.health = self.create_publisher(String, '/sensor_status/hereflow', 10)
        self.create_timer(0.02, self.poll)
        self.create_timer(1.0, self.publish_health)

    def poll(self):
        old_connections = self.reader.connections
        packets, _ = self.reader.poll()
        if self.reader.connections > old_connections:
            self.get_logger().info('serial port opened')
        error = self.reader.last_error
        if error and error != self.last_reported_error:
            self.get_logger().warning(f'serial unavailable: {error}')
        self.last_reported_error = error
        for packet in packets:
            msg = OpticalFlow()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'flow_sensor'
            msg.raw_x = packet.x; msg.raw_y = packet.y; msg.quality = packet.quality
            msg.metric_valid = False; msg.source = 'hereflow_uart'
            self.pub.publish(msg)

    def publish_health(self):
        status = self.reader.status()
        msg = String()
        msg.data = (' '.join(f'{key}={value}' for key, value in status.items()) +
                    ' units=raw_counts_unverified metric_valid=False')
        self.health.publish(msg)

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()


def main():
    rclpy.init(); node = HereFlowNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
