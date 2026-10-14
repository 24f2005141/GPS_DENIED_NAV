import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool
from gps_nav_interfaces.msg import OpticalFlow
from gps_nav_interfaces.msg import NavigationStatus
from .sim_core import GoalSimulator, PathSimulator


class SimulatorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('simulator', **kwargs)
        defaults = [('mode','goal'),('path','straight'),('seed',42),('dt',0.05),('flow_hz',20.0),
                    ('imu_hz',20.0),('flow_scale_m_per_count',0.01),('noise_m',0.0),
                    ('goal_x_m',3.0),('goal_y_m',0.0),('speed_m_s',0.6),
                    ('turn_rate_rad_s',math.pi/2),('initial_heading_deg',0.0),
                    ('align_initial_heading_to_goal',False),('max_duration_s',30.0),
                    ('yaw_bias_rad_s',0.0),('flow_dropout_start',-1.0),
                    ('flow_dropout_end',-1.0),('imu_dropout_start',-1.0),
                    ('imu_dropout_end',-1.0),('low_quality_start',-1.0),
                    ('low_quality_end',-1.0),('startup_delay_s',2.0),
                    ('cmd_vel_topic','/simulation/cmd_vel')]
        for name,value in defaults: self.declare_parameter(name,value)
        get = lambda n: self.get_parameter(n).value
        self.mode = get('mode')
        if self.mode not in ('scripted','goal','interactive'):
            raise ValueError('simulation mode must be scripted, goal, or interactive')
        if self.mode != 'scripted' and get('path') != 'straight':
            raise ValueError('a scripted path requires mode=scripted')
        if not math.isfinite(get('initial_heading_deg')):
            raise ValueError('initial_heading_deg must be finite')
        common = dict(dt=get('dt'),seed=get('seed'),noise_m=get('noise_m'),
                      yaw_bias_rad_s=get('yaw_bias_rad_s'),
                      dropout_start=get('flow_dropout_start'),
                      dropout_end=get('flow_dropout_end'),
                      low_quality_start=get('low_quality_start'),
                      low_quality_end=get('low_quality_end'))
        self.sim = (GoalSimulator(get('goal_x_m'),get('goal_y_m'),
                                  get('speed_m_s'),get('turn_rate_rad_s'),
                                  None if get('align_initial_heading_to_goal') else
                                  math.radians(get('initial_heading_deg')),**common)
                    if self.mode == 'goal' else PathSimulator(get('path'),**common))
        if self.mode == 'interactive':
            self.sim.yaw=math.radians(get('initial_heading_deg'))
        if get('max_duration_s') < 0 or (self.mode != 'interactive' and get('max_duration_s') == 0):
            raise ValueError('max_duration_s must be positive, or zero for unlimited interactive mode')
        self.scale = get('flow_scale_m_per_count')
        if self.scale <= 0: raise ValueError('simulation flow scale must be positive')
        self.flow_interval = 1/get('flow_hz'); self.imu_interval = 1/get('imu_hz')
        self.last_flow = self.last_imu = -1e9
        self.flow_dx = self.flow_dy = 0.0
        self.flow_pub = self.create_publisher(OpticalFlow,'/optical_flow/raw',100)
        self.imu_pub = self.create_publisher(Imu,'/imu/data',100)
        self.truth_pub = self.create_publisher(PointStamped,'/simulation/ground_truth',100)
        self.done_pub = self.create_publisher(Bool,'/simulation/done',10)
        self.done = False
        self.arrival_seen = False; self.completion_reason = None
        self.manual_speed = 0.0; self.manual_rate = 0.0
        self.create_subscription(Twist, get('cmd_vel_topic'), self.on_command, 10)
        self.create_subscription(NavigationStatus, '/navigation_status', self.on_navigation, 20)
        self.ready_at = time.monotonic() + get('startup_delay_s')
        self.create_timer(get('dt'),self.tick)
        motion=(f'goal speed={get("speed_m_s"):.3f}m/s '
                f'turn_rate={get("turn_rate_rad_s"):.3f}rad/s' if self.mode == 'goal' else
                f'scripted path={get("path")}' if self.mode == 'scripted' else
                'interactive initial command=(0.000m/s,0.000rad/s)')
        self.get_logger().info(
            f'effective mode={self.mode} start=(0.000,0.000) '
            f'goal=({get("goal_x_m"):.3f},{get("goal_y_m"):.3f}) {motion} '
            f'dt={get("dt"):.3f}s')

    def tick(self):
        if self.done: return
        if time.monotonic() < self.ready_at: return
        if self.mode == 'goal' and self.arrival_seen:
            self.finish('destination reached')
            return
        max_duration = self.get_parameter('max_duration_s').value
        if max_duration > 0 and self.sim.t >= max_duration:
            self.finish('timeout')
            return
        sample = (self.sim.step() if self.mode != 'interactive' else
                  self.sim.manual_step(self.manual_speed, self.manual_rate))
        if sample is None:
            if self.flow_dx or self.flow_dy:
                self.publish_flow(self.last_stamp)
            self.finish('scripted path complete')
            return
        stamp = self.get_clock().now().to_msg()
        self.last_stamp = stamp
        truth=PointStamped(); truth.header.stamp=stamp; truth.header.frame_id='map_enu'
        truth.point.x=sample.x; truth.point.y=sample.y; self.truth_pub.publish(truth)
        dx,dy=self.sim.noisy_flow(sample)
        self.flow_dx+=dx; self.flow_dy+=dy
        if sample.t-self.last_imu >= self.imu_interval-1e-9:
            self.last_imu=sample.t
            if not self.get_parameter('imu_dropout_start').value <= sample.t < self.get_parameter('imu_dropout_end').value:
                imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='imu_link'
                reported_yaw=sample.yaw+self.get_parameter('yaw_bias_rad_s').value*sample.t
                imu.orientation.z=math.sin(reported_yaw/2)
                imu.orientation.w=math.cos(reported_yaw/2)
                imu.orientation_covariance[0]=1e-6; imu.orientation_covariance[4]=1e-6
                imu.orientation_covariance[8]=1e-6
                imu.angular_velocity.z=sample.yaw_rate
                imu.linear_acceleration.x=sample.acceleration_x
                imu.linear_acceleration.z=9.80665
                self.imu_pub.publish(imu)
        if sample.t-self.last_flow >= self.flow_interval-1e-9:
            self.last_flow=sample.t
            self.publish_flow(stamp)
            self.flow_dx=self.flow_dy=0.0

    def finish(self, reason):
        self.done = True; self.completion_reason = reason
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info(f'simulation complete ({reason}) truth=({self.sim.x:.3f},{self.sim.y:.3f})')

    def on_navigation(self, msg):
        if self.mode == 'goal' and msg.destination_reached:
            self.arrival_seen = True

    def publish_flow(self, stamp):
        if self.sim.flow_available():
            flow=OpticalFlow(); flow.header.stamp=stamp; flow.header.frame_id='flow_sensor'
            flow.raw_x=round(self.flow_dx/self.scale)
            flow.raw_y=round(self.flow_dy/self.scale)
            flow.quality=self.sim.quality(); flow.source='simulation'
            self.flow_pub.publish(flow)

    def on_command(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.manual_speed = msg.linear.x
            self.manual_rate = msg.angular.z


def main():
    rclpy.init(); node=SimulatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
