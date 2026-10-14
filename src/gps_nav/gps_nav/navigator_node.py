import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from gps_nav_interfaces.msg import LocalizationStatus, NavigationStatus
from std_msgs.msg import Empty
from .core import Navigator
from .estimator_node import seconds, yaw_from_quaternion


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')
        for name,value in [('destination_tolerance_m',0.25),('arrival_stable_updates',3),
                           ('straight_threshold_deg',15.0),('back_threshold_deg',135.0)]:
            self.declare_parameter(name,value)
        self.nav = Navigator(self.get_parameter('destination_tolerance_m').value,
                             self.get_parameter('arrival_stable_updates').value,
                             self.get_parameter('straight_threshold_deg').value,
                             self.get_parameter('back_threshold_deg').value)
        self.goal = None; self.status = None; self.last_pose = None
        self.last_result = None
        self.pub = self.create_publisher(NavigationStatus, '/navigation_status', 10)
        self.create_subscription(PointStamped, '/navigation_goal', self.on_goal, 10)
        self.create_subscription(LocalizationStatus, '/localization_status', self.on_status, 10)
        self.create_subscription(PoseStamped, '/estimated_position', self.on_pose, 100)
        self.create_subscription(Empty, '/session_reset', self.on_reset, 10)
        self.create_timer(0.2, self.on_timer)

    def on_reset(self, _):
        self.nav.reset(); self.last_pose = None; self.last_result = None

    def on_goal(self, msg):
        goal = (msg.point.x,msg.point.y)
        if self.goal is None or abs(goal[0]-self.goal[0])+abs(goal[1]-self.goal[1]) > 1e-8:
            self.nav.reset()
            self.last_result = None
        self.goal = goal

    def on_status(self, msg):
        self.status = msg
        if not (msg.position_valid and msg.metric_calibrated and msg.heading_valid):
            self.evaluate(fresh_update=False)

    def on_pose(self, msg):
        self.last_pose = msg
        self.evaluate(fresh_update=True)

    def on_timer(self): self.evaluate(fresh_update=False)

    def evaluate(self, fresh_update):
        if self.goal is None: return
        from .core import Guidance
        status = self.status
        valid = (status is not None and status.position_valid and
                 status.metric_calibrated and status.heading_valid)
        heading = status is not None and status.heading_valid
        degraded = status is not None and (not status.flow_fresh or not status.imu_fresh)
        if self.last_pose is None or not valid:
            if self.last_pose is not None and degraded:
                label = 'SENSOR DEGRADED'
            elif self.last_pose is not None and status is not None and not heading:
                label = 'HEADING UNAVAILABLE'
            else:
                label = 'POSITION UNRELIABLE'
            result = Guidance(label=label)
            self.nav.reset()
        elif fresh_update:
            p = self.last_pose.pose.position
            yaw = yaw_from_quaternion(self.last_pose.pose.orientation)
            result = self.nav.evaluate(p.x,p.y,yaw or 0.0,*self.goal,valid,heading,degraded)
            self.last_result = result
        else:
            result = self.last_result or Guidance(label='POSITION UNRELIABLE')
        m = NavigationStatus(); m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map_enu'; m.distance_valid = valid
        m.distance_m = result.distance; m.target_bearing_rad = result.bearing
        m.heading_error_rad = result.error; m.recommendation = result.label
        m.destination_reached = result.reached
        self.pub.publish(m)


def main():
    rclpy.init(); node = NavigatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
