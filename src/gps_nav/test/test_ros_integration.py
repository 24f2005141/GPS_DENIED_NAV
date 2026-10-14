import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Empty, String
from gps_nav_interfaces.srv import SetSession, SetDestination
from gps_nav.waypoint_node import WaypointNode
from gps_nav.hereflow_node import HereFlowNode
from gps_nav.estimator_node import EstimatorNode
from gps_nav.preprocess_node import PreprocessNode
from gps_nav.navigator_node import NavigatorNode
from gps_nav_interfaces.msg import OpticalFlow, LocalizationStatus, NavigationStatus
from sensor_msgs.msg import Imu


def wait(executor, future, seconds=4):
    deadline=time.monotonic()+seconds
    while not future.done() and time.monotonic()<deadline:
        executor.spin_once(timeout_sec=0.05)
    assert future.done()
    return future.result()


def test_waypoint_services_over_ros():
    rclpy.init()
    nodes=[]; executor=SingleThreadedExecutor()
    try:
        way=WaypointNode(); client_node=Node('waypoint_test_client')
        nodes=[way,client_node]
        goals=[]; resets=[]
        client_node.create_subscription(PointStamped,'/navigation_goal',goals.append,10)
        client_node.create_subscription(Empty,'/session_reset',resets.append,10)
        session=client_node.create_client(SetSession,'/set_session')
        destination=client_node.create_client(SetDestination,'/set_destination')
        for node in nodes: executor.add_node(node)
        assert session.wait_for_service(timeout_sec=4)
        bad=SetSession.Request(origin_lat=91.0,origin_lon=0.0,goal_lat=0.0,goal_lon=0.0)
        assert not wait(executor,session.call_async(bad)).success
        valid=SetSession.Request(origin_lat=0.0,origin_lon=0.0,goal_lat=0.0,goal_lon=0.001)
        assert wait(executor,session.call_async(valid)).success
        assert wait(executor,destination.call_async(SetDestination.Request(latitude=0.001,longitude=0.0))).success
        deadline=time.monotonic()+2
        while (not goals or goals[-1].point.y < 100 or not resets) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
        assert goals[-1].point.y > 100
        assert abs(goals[-1].point.x) < 1
        assert len(resets)==1
    finally:
        for node in nodes:
            executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.shutdown()


def test_missing_serial_port_is_safe():
    rclpy.init()
    from rclpy.parameter import Parameter
    node=HereFlowNode(parameter_overrides=[Parameter('serial_port',value='/dev/does-not-exist')])
    monitor=Node('serial_health_test_monitor')
    received=[]
    monitor.create_subscription(String,'/sensor_status/hereflow',received.append,10)
    executor=SingleThreadedExecutor(); executor.add_node(monitor)
    try:
        node.poll()
        assert node.reader.port is None
        assert node.reader.parser.valid == 0
        node.publish_health()
        deadline=time.monotonic()+2
        while not received and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received
        assert 'connected=False' in received[-1].data
        assert 'data_stale=True' in received[-1].data
        assert 'metric_valid=False' in received[-1].data
    finally:
        executor.remove_node(monitor)
        node.destroy_node(); monitor.destroy_node()
        executor.shutdown(); rclpy.shutdown()


def test_imu_requires_orientation_covariance():
    rclpy.init()
    from rclpy.parameter import Parameter
    node=EstimatorNode(parameter_overrides=[Parameter('heading_reference_valid',value=True)])
    try:
        m=Imu(); m.header.stamp=node.get_clock().now().to_msg()
        m.header.frame_id='imu_link'; m.orientation.w=1.0
        node.on_imu(m)
        assert not node.tracker.heading_valid
        m.orientation_covariance[0]=1e-6
        m.orientation_covariance[8]=1e-6
        node.on_imu(m)
        assert node.tracker.heading_valid
        m.orientation_covariance[8]=float('nan')
        node.on_imu(m)
        assert not node.tracker.heading_valid
        m.orientation_covariance[8]=1e-6
        m.header.frame_id='wrong_frame'
        node.on_imu(m)
        assert not node.tracker.heading_valid
        m.header.frame_id='imu_link'; m.header.stamp.sec=1; m.header.stamp.nanosec=0
        node.on_imu(m)
        assert not node.tracker.heading_valid
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_preprocess_requires_calibration_and_unfiltered_displacement():
    rclpy.init()
    from rclpy.parameter import Parameter
    monitor=Node('flow_guard_test_monitor')
    received=[]
    monitor.create_subscription(OpticalFlow,'/optical_flow',received.append,10)
    nodes=[]; executor=SingleThreadedExecutor()
    try:
        raw=OpticalFlow(); raw.raw_x=10; raw.raw_y=-5; raw.quality=255
        calibrated=[Parameter('calibration_valid',value=True),
                    Parameter('flow_scale_x',value=0.01),
                    Parameter('flow_scale_y',value=0.01)]
        for overrides, expected in [([],False), ([*calibrated,
                              Parameter('smoothing_alpha',value=0.5)],False),
                              (calibrated,True)]:
            node=PreprocessNode(parameter_overrides=overrides)
            nodes.append(node)
            executor.add_node(monitor)
            node.on_raw(raw)
            deadline=time.monotonic()+2
            while not received and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=0.05)
            assert received and received[-1].metric_valid is expected
            if expected:
                assert received[-1].displacement_x_m == 0.1
                assert received[-1].displacement_y_m == -0.05
            received.clear()
            executor.remove_node(monitor)
            node.destroy_node(); nodes.remove(node)
    finally:
        for node in nodes: node.destroy_node()
        executor.remove_node(monitor); monitor.destroy_node()
        executor.shutdown(); rclpy.shutdown()


def test_navigation_node_refuses_uncalibrated_or_headingless_guidance():
    rclpy.init()
    node=NavigatorNode(); monitor=Node('navigation_guard_test_monitor')
    received=[]
    monitor.create_subscription(NavigationStatus,'/navigation_status',received.append,10)
    executor=SingleThreadedExecutor(); executor.add_node(monitor)
    try:
        goal=PointStamped(); goal.point.x=1.0; node.on_goal(goal)
        pose=PoseStamped(); pose.pose.orientation.w=1.0
        status=LocalizationStatus()
        status.position_valid=True; status.heading_valid=True
        status.imu_fresh=True; status.flow_fresh=True
        node.on_status(status); node.on_pose(pose)
        deadline=time.monotonic()+2
        while not received and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received and not received[-1].distance_valid
        assert received[-1].recommendation == 'POSITION UNRELIABLE'
        received.clear()
        status.metric_calibrated=True; status.heading_valid=False
        node.on_status(status); node.on_pose(pose)
        deadline=time.monotonic()+2
        while len(received)<2 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received and not received[-1].distance_valid
        assert received[-1].recommendation == 'HEADING UNAVAILABLE'
        assert not received[-1].destination_reached
    finally:
        executor.remove_node(monitor)
        node.destroy_node(); monitor.destroy_node()
        executor.shutdown(); rclpy.shutdown()


def test_estimator_rejects_delayed_flow_before_position_update():
    rclpy.init()
    node=EstimatorNode()
    try:
        node.tracker.imu(0, 1.0)
        flow=OpticalFlow(); flow.header.stamp.sec=1
        flow.metric_valid=True; flow.displacement_x_m=5.0; flow.quality=255
        node.on_flow(flow)
        assert node.tracker.x == 0
        assert node.tracker.last_flow is None
        assert node.tracker.reason == 'stale/future optical-flow timestamp'
    finally:
        node.destroy_node(); rclpy.shutdown()
