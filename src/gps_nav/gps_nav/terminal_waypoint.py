"""Interactive waypoint entry using the same validation and ROS services."""
import argparse
import rclpy
from rclpy.node import Node
from gps_nav_interfaces.srv import SetSession, SetDestination
from .geo import validate


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--destination-only',action='store_true')
    args=parser.parse_args()
    if args.destination_only:
        goal=(float(input('Destination latitude: ')),float(input('Destination longitude: ')))
        validate(*goal)
        service='/set_destination'; typ=SetDestination
    else:
        origin=(float(input('Initial latitude: ')),float(input('Initial longitude: ')))
        goal=(float(input('Destination latitude: ')),float(input('Destination longitude: ')))
        validate(*origin); validate(*goal)
        service='/set_session'; typ=SetSession
    rclpy.init(); node=Node('terminal_waypoint')
    try:
        client=node.create_client(typ,service)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'{service} unavailable')
        request=typ.Request()
        if args.destination_only:
            request.latitude,request.longitude=goal
        else:
            request.origin_lat,request.origin_lon=origin
            request.goal_lat,request.goal_lon=goal
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        response=future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else 'service timeout')
        print(response.message)
    finally:
        node.destroy_node(); rclpy.shutdown()
