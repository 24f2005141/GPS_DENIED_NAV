"""ROS-to-USB-serial bridge; motor commands have a separate default-off gate."""
import time
import rclpy
from rclpy.node import Node
from gps_nav_interfaces.msg import RobotAck, RobotCommand, RobotLinkStatus
from .robot_protocol import Command, ProtocolError
from .robot_serial import RobotSerialClient


class RobotTransportNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_transport',**kwargs)
        defaults=[('serial_port','/dev/ttyACM0'),('baud_rate',115200),
                  ('reconnect_s',1.0),('ack_timeout_s',0.4),
                  ('command_timeout_s',0.35),('motor_commands_enabled',False)]
        for name,value in defaults: self.declare_parameter(name,value)
        get=lambda n:self.get_parameter(n).value
        self.client=RobotSerialClient(get('serial_port'),get('baud_rate'),
                                      get('reconnect_s'),get('ack_timeout_s'),
                                      get('command_timeout_s'))
        self.motor_commands_enabled=get('motor_commands_enabled')
        self.ack_pub=self.create_publisher(RobotAck,'/robot/ack',20)
        self.link_pub=self.create_publisher(RobotLinkStatus,'/robot/link_status',20)
        self.create_subscription(RobotCommand,'/robot/command',self.on_command,20)
        self.create_timer(0.02,self.tick)

    def on_command(self,msg):
        direction=msg.command if self.motor_commands_enabled else 'STOP'
        speed=msg.speed_permille if self.motor_commands_enabled else 0
        try: self.client.queue(Command(msg.sequence,direction,speed),time.monotonic())
        except ProtocolError as exc:
            self.get_logger().error('invalid robot command: '+exc.reason)
            self.client.queue(Command(max(1,msg.sequence),'STOP',0),time.monotonic())

    def tick(self):
        now=time.monotonic()
        for ack in self.client.tick(now):
            msg=RobotAck(); msg.header.stamp=self.get_clock().now().to_msg()
            msg.sequence=ack.sequence; msg.accepted=ack.accepted
            msg.command=ack.command; msg.reason=ack.reason
            self.ack_pub.publish(msg)
        status=RobotLinkStatus(); status.header.stamp=self.get_clock().now().to_msg()
        status.connected=self.client.connected; status.ack_fresh=self.client.ack_fresh(now)
        if self.client.last_ack:
            status.last_ack_sequence=self.client.last_ack.sequence
            status.last_ack_command=self.client.last_ack.command
        status.reason=self.client.reason
        self.link_pub.publish(status)

    def destroy_node(self):
        self.client.close(time.monotonic())
        super().destroy_node()


def main():
    rclpy.init(); node=RobotTransportNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
