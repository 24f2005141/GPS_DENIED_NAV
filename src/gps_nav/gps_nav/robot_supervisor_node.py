"""ROS guidance-to-command safety gate; no direct serial or motor access."""
import json
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Empty
from geometry_msgs.msg import PointStamped
from gps_nav_interfaces.msg import (LocalizationStatus, NavigationStatus,
                                    RobotAck, RobotCommand, RobotControlStatus,
                                    RobotLinkStatus)
from .robot_supervisor_core import ControlSupervisor, Guidance, Health, Link
from .robot_protocol import Ack


class RobotSupervisorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_supervisor',**kwargs)
        defaults=[('allow_physical_control',False),('max_speed_permille',100),
                  ('forward_speed_permille',80),('turn_speed_permille',60),
                  ('allow_reverse',False),('input_stale_s',0.5),
                  ('ack_timeout_s',0.5),('command_period_s',0.1),('log_file','')]
        for name,value in defaults: self.declare_parameter(name,value)
        get=lambda n:self.get_parameter(n).value
        self.core=ControlSupervisor(get('allow_physical_control'),get('max_speed_permille'),
                                    get('forward_speed_permille'),get('turn_speed_permille'),
                                    get('allow_reverse'),get('input_stale_s'),get('ack_timeout_s'))
        self.log_stream=open(get('log_file'),'a',encoding='utf-8') if get('log_file') else None
        self.command_pub=self.create_publisher(RobotCommand,'/robot/command',20)
        self.status_pub=self.create_publisher(RobotControlStatus,'/robot/control_status',20)
        self.create_subscription(NavigationStatus,'/navigation_status',self.on_navigation,20)
        self.create_subscription(LocalizationStatus,'/localization_status',self.on_localization,20)
        self.create_subscription(RobotLinkStatus,'/robot/link_status',self.on_link,20)
        self.create_subscription(RobotAck,'/robot/ack',self.on_ack,20)
        self.create_subscription(PointStamped,'/navigation_goal',self.on_goal,10)
        self.create_subscription(Empty,'/session_reset',lambda _:self.core.reset_session(),10)
        self.create_service(SetBool,'/robot/set_enabled',self.set_enabled)
        self.create_service(Trigger,'/robot/request_stop',self.request_stop)
        self.create_service(Trigger,'/robot/reset_fault',self.reset_fault)
        self.create_timer(get('command_period_s'),self.tick)

    def record(self,kind,fields):
        if self.log_stream:
            self.log_stream.write(json.dumps({'monotonic_s':time.monotonic(),
                                              'event':kind,**fields})+'\n')
            self.log_stream.flush()

    def on_navigation(self,msg):
        self.core.update_guidance(Guidance(msg.recommendation,msg.distance_valid,
                                           msg.destination_reached,time.monotonic()))

    def on_localization(self,msg):
        self.core.update_health(Health(msg.position_valid,msg.heading_valid,msg.flow_fresh,
                                       msg.imu_fresh,msg.metric_calibrated,time.monotonic()))

    def on_link(self,msg):
        self.core.update_link(Link(msg.connected,msg.ack_fresh,time.monotonic()))

    def on_ack(self,msg):
        self.core.acknowledge(Ack(msg.sequence,msg.accepted,msg.command,msg.reason))
        self.record('ack',{'sequence':msg.sequence,'accepted':msg.accepted,
                           'command':msg.command,'reason':msg.reason})

    def on_goal(self,msg): self.core.set_goal((msg.point.x,msg.point.y))

    def set_enabled(self,request,response):
        if request.data:
            response.success,response.message=self.core.request_enable(time.monotonic())
        else:
            self.core.stop('control disabled by user')
            response.success=True; response.message='STOP requested; control disabled'
            self.tick()
        return response

    def request_stop(self,_,response):
        self.core.stop('STOP requested by user')
        response.success=True; response.message='STOP requested; control disabled'
        self.tick()
        return response

    def reset_fault(self,_,response):
        response.success,response.message=self.core.request_reset(time.monotonic())
        return response

    def tick(self):
        command=self.core.tick(time.monotonic())
        if command is not None:
            msg=RobotCommand(); msg.header.stamp=self.get_clock().now().to_msg()
            msg.sequence=command.sequence; msg.command=command.direction
            msg.speed_permille=command.speed_permille
            self.command_pub.publish(msg)
            self.record('command',{'sequence':command.sequence,'command':command.direction,
                                   'speed_permille':command.speed_permille})
        status=RobotControlStatus(); status.header.stamp=self.get_clock().now().to_msg()
        status.enabled=self.core.enabled; status.arming=self.core.arming
        status.fault_latched=self.core.fault_latched
        status.arrival_latched=self.core.arrival_latched
        status.link_connected=self.core.link.connected if self.core.link else False
        status.acknowledgement_fresh=self.core.link.ack_fresh if self.core.link else False
        status.reason=self.core.reason; status.last_issued_command=self.core.last_issued
        if self.core.last_ack:
            status.last_ack_sequence=self.core.last_ack.sequence
            status.last_ack_command=self.core.last_ack.command
        self.status_pub.publish(status)

    def destroy_node(self):
        self.core.stop('supervisor shutdown')
        if rclpy.ok(context=self.context):
            try:
                self.tick()
            except Exception as exc:
                self.get_logger().warn(f'shutdown STOP publish failed: {exc}')
        if self.log_stream: self.log_stream.close()
        super().destroy_node()


def main():
    rclpy.init(); node=RobotSupervisorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
