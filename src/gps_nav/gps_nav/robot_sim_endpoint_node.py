"""ROS-only simulated microcontroller using the exact serial protocol codec."""
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from gps_nav_interfaces.msg import RobotAck, RobotCommand, RobotLinkStatus
from .firmware_model import MicrocontrollerModel
from .robot_protocol import Ack, Command, ProtocolError, decode_ack, encode_ack, encode_command


class RobotSimEndpointNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_sim_endpoint',**kwargs)
        self.declare_parameter('watchdog_s',0.5)
        self.declare_parameter('max_speed_permille',200)
        self.declare_parameter('max_linear_m_s',1.125)
        self.declare_parameter('max_turn_rad_s',3.92699081699)
        self.declare_parameter('cmd_vel_topic','/simulation/robot_cmd_vel')
        self.model=MicrocontrollerModel(self.get_parameter('watchdog_s').value,
                                        self.get_parameter('max_speed_permille').value)
        self.connected=True; self.last_ack=None; self.last_ack_time=None
        self.ack_pub=self.create_publisher(RobotAck,'/robot/ack',20)
        self.link_pub=self.create_publisher(RobotLinkStatus,'/robot/link_status',20)
        self.motion_pub=self.create_publisher(
            Twist,self.get_parameter('cmd_vel_topic').value,20)
        self.create_subscription(RobotCommand,'/robot/command',self.on_command,20)
        self.create_service(SetBool,'/robot/sim_link/set_connected',self.set_connected)
        self.create_timer(0.05,self.tick)

    def set_connected(self,request,response):
        self.connected=request.data
        if not self.connected:
            self.model.stop(); self.model.fault=True
            self.last_ack_time=None
        response.success=True
        response.message='simulated link connected' if self.connected else 'simulated link disconnected'
        return response

    def on_command(self,msg):
        if not self.connected: return
        now=time.monotonic()
        try:
            encoded=encode_command(Command(msg.sequence,msg.command,msg.speed_permille))
            ack=decode_ack(self.model.process(encoded,now))
        except ProtocolError as exc:
            self.model.stop(); self.model.fault=True
            ack=Ack(msg.sequence,False,'UNKNOWN',exc.reason)
            decode_ack(encode_ack(ack))
        result=RobotAck(); result.header.stamp=self.get_clock().now().to_msg()
        result.sequence=ack.sequence; result.accepted=ack.accepted
        result.command=ack.command; result.reason=ack.reason
        self.ack_pub.publish(result)
        self.last_ack=ack; self.last_ack_time=now

    def tick(self):
        now=time.monotonic()
        expired=self.model.watchdog(now)
        motion=Twist()
        fraction=self.model.speed/self.model.max_speed
        if self.model.armed and not self.model.fault and self.connected:
            if self.model.direction=='FORWARD':
                motion.linear.x=self.get_parameter('max_linear_m_s').value*fraction
            elif self.model.direction=='BACKWARD':
                motion.linear.x=-self.get_parameter('max_linear_m_s').value*fraction
            elif self.model.direction=='LEFT':
                motion.angular.z=self.get_parameter('max_turn_rad_s').value*fraction
            elif self.model.direction=='RIGHT':
                motion.angular.z=-self.get_parameter('max_turn_rad_s').value*fraction
        self.motion_pub.publish(motion)
        status=RobotLinkStatus(); status.header.stamp=self.get_clock().now().to_msg()
        status.connected=self.connected
        status.ack_fresh=self.connected and self.last_ack is not None and self.last_ack.accepted and self.last_ack_time is not None and (
            now-self.last_ack_time<=self.model.watchdog_s)
        if self.last_ack:
            status.last_ack_sequence=self.last_ack.sequence
            status.last_ack_command=self.last_ack.command
        status.reason=('disconnected' if not self.connected else
                       'watchdog expired; motors stopped' if expired or self.model.fault else
                       'acknowledgement fresh' if status.ack_fresh else 'awaiting acknowledgement')
        self.link_pub.publish(status)


def main():
    rclpy.init(); node=RobotSimEndpointNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
