import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
from gps_nav_interfaces.srv import SetSession, SetDestination
from .geo import ENU, validate


class WaypointNode(Node):
    def __init__(self, **kwargs):
        super().__init__('waypoint', **kwargs)
        for name, default in [('origin_lat', 0.0), ('origin_lon', 0.0),
                              ('goal_lat', 0.0), ('goal_lon', 0.0000269494585)]:
            self.declare_parameter(name, default)
        self.origin = (self.get_parameter('origin_lat').value, self.get_parameter('origin_lon').value)
        self.destination = (self.get_parameter('goal_lat').value, self.get_parameter('goal_lon').value)
        validate(*self.origin); validate(*self.destination)
        self.frame = ENU(*self.origin)
        self.pub = self.create_publisher(PointStamped, '/navigation_goal', 10)
        self.reset_pub = self.create_publisher(Empty, '/session_reset', 10)
        self.create_service(SetSession, '/set_session', self.set_session)
        self.create_service(SetDestination, '/set_destination', self.set_destination)
        self.create_timer(1.0, self.publish_goal)
        self.publish_goal()

    def publish_goal(self):
        x, y = self.frame.to_local(*self.destination)
        m = PointStamped(); m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map_enu'; m.point.x = x; m.point.y = y
        self.pub.publish(m)

    def set_session(self, req, res):
        try:
            validate(req.origin_lat, req.origin_lon)
            validate(req.goal_lat, req.goal_lon)
            self.origin = (req.origin_lat, req.origin_lon)
            self.destination = (req.goal_lat, req.goal_lon)
            self.frame = ENU(*self.origin)
            self.reset_pub.publish(Empty())
            self.publish_goal()
            res.success = True; res.message = 'session reset with new origin and goal'
        except ValueError as exc:
            res.success = False; res.message = str(exc)
        return res

    def set_destination(self, req, res):
        try:
            validate(req.latitude, req.longitude)
            self.destination = (req.latitude, req.longitude)
            self.publish_goal()
            res.success = True; res.message = 'destination updated; origin unchanged'
        except ValueError as exc:
            res.success = False; res.message = str(exc)
        return res


def main():
    rclpy.init(); node = WaypointNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
