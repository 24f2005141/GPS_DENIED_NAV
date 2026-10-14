import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty
from gps_nav_interfaces.msg import OpticalFlow, LocalizationStatus
from .core import Tracker


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def timestamp_in_window(stamp, now, stale_s, future_tolerance_s):
    """Compare ROS timestamps in integer nanoseconds, without epoch float rounding."""
    age_ns = ((now.sec - stamp.sec) * 1_000_000_000 +
              now.nanosec - stamp.nanosec)
    return (-round(future_tolerance_s * 1_000_000_000) <= age_ns <=
            round(stale_s * 1_000_000_000))


def yaw_from_quaternion(q):
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if norm < 0.5 or not math.isfinite(norm):
        return None
    x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class EstimatorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('estimator', **kwargs)
        for name,value in [('stale_s',0.6),('minimum_quality',30),
                           ('future_stamp_tolerance_s',0.05),
                           ('rotation_offset_deg',0.0),('max_yaw_variance_rad2',0.5),
                           ('heading_reference_valid',False),('heading_offset_deg',0.0),
                           ('expected_imu_frame','imu_link'),
                           ('publish_velocity',False),('velocity_max_interval_s',0.15)]:
            self.declare_parameter(name,value)
        self.tracker = Tracker(self.get_parameter('stale_s').value,
                               self.get_parameter('minimum_quality').value)
        self.path = Path(); self.path.header.frame_id = 'map_enu'
        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_position', 20)
        self.odom_pub = self.create_publisher(Odometry, '/estimated_odometry', 20)
        self.velocity_pub = self.create_publisher(TwistStamped, '/estimated_velocity', 20)
        self.path_pub = self.create_publisher(Path, '/estimated_path', 10)
        self.status_pub = self.create_publisher(LocalizationStatus, '/localization_status', 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 100)
        self.create_subscription(OpticalFlow, '/optical_flow', self.on_flow, 100)
        self.create_subscription(Empty, '/session_reset', self.on_reset, 10)
        self.create_timer(0.1, self.publish_status)

    def timestamp_fresh(self, stamp):
        return timestamp_in_window(
            stamp, self.get_clock().now().to_msg(),
            self.get_parameter('stale_s').value,
            self.get_parameter('future_stamp_tolerance_s').value)

    def on_reset(self, _):
        self.tracker.reset()
        self.path.poses.clear()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        self.publish_status()
        self.get_logger().info('estimator reset')

    def on_imu(self, msg):
        if (not self.get_parameter('heading_reference_valid').value or
                msg.header.frame_id != self.get_parameter('expected_imu_frame').value or
                not self.timestamp_fresh(msg.header.stamp)):
            self.tracker.heading_valid = False
            self.publish_status()
            return
        variance = msg.orientation_covariance[8]
        if (any(not math.isfinite(v) for v in msg.orientation_covariance) or
                any(msg.orientation_covariance[i] < 0 for i in (0,4,8)) or
                variance <= 0 or variance > self.get_parameter('max_yaw_variance_rad2').value):
            self.tracker.heading_valid = False
            self.publish_status()
            return
        yaw = yaw_from_quaternion(msg.orientation)
        if yaw is not None:
            offset = math.radians(self.get_parameter('heading_offset_deg').value)
            self.tracker.imu(yaw + offset,seconds(msg.header.stamp))
        else:
            self.tracker.heading_valid = False
            self.publish_status()

    def on_flow(self, msg):
        if not self.timestamp_fresh(msg.header.stamp):
            self.tracker.flow_healthy = False
            self.tracker.reason = 'stale/future optical-flow timestamp'
            self.publish_status()
            return
        previous_stamp = self.tracker.last_accepted_flow
        previous_x, previous_y = self.tracker.x, self.tracker.y
        accepted = self.tracker.flow(msg.displacement_x_m,msg.displacement_y_m,
                                     seconds(msg.header.stamp),msg.quality,msg.metric_valid,
                                     math.radians(self.get_parameter('rotation_offset_deg').value))
        if not accepted: return
        pose = PoseStamped(); pose.header = msg.header; pose.header.frame_id = 'map_enu'
        pose.pose.position.x = self.tracker.x; pose.pose.position.y = self.tracker.y
        pose.pose.orientation.z = math.sin(self.tracker.yaw/2)
        pose.pose.orientation.w = math.cos(self.tracker.yaw/2)
        self.pose_pub.publish(pose)
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)
        odom = Odometry(); odom.header = pose.header; odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose
        odom.pose.covariance[0] = 1e6; odom.pose.covariance[7] = 1e6
        odom.pose.covariance[35] = 1e6  # unknown real-world uncertainty
        odom.twist.covariance[0] = 1e6
        odom.twist.covariance[7] = 1e6
        self.odom_pub.publish(odom)
        if self.get_parameter('publish_velocity').value and previous_stamp is not None:
            dt = seconds(msg.header.stamp) - previous_stamp
            if 0 < dt <= self.get_parameter('velocity_max_interval_s').value:
                v = TwistStamped(); v.header = pose.header
                v.twist.linear.x = (self.tracker.x - previous_x) / dt
                v.twist.linear.y = (self.tracker.y - previous_y) / dt
                self.velocity_pub.publish(v)
        self.publish_status()

    def publish_status(self):
        now = seconds(self.get_clock().now().to_msg())
        valid, imu_fresh, flow_fresh, reason = self.tracker.status(now)
        m = LocalizationStatus(); m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map_enu'; m.position_valid = valid
        m.heading_valid = self.tracker.heading_valid and imu_fresh
        m.imu_fresh = imu_fresh; m.flow_fresh = flow_fresh
        m.metric_calibrated = self.tracker.metric_calibrated
        m.flow_quality = self.tracker.flow_quality; m.reason = reason
        self.status_pub.publish(m)


def main():
    rclpy.init(); node = EstimatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
