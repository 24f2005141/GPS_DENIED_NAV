"""Regression guards for the finite demo's shared ROS timestamp domain."""
from builtin_interfaces.msg import Time
import rclpy
from rclpy.parameter import Parameter
from gps_nav.estimator_node import EstimatorNode, timestamp_in_window
from gps_nav.simulator_node import SimulatorNode
from gps_nav_interfaces.msg import OpticalFlow
from sensor_msgs.msg import Imu


def stamp_at(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def test_timestamp_window_at_exact_valid_stale_and_future_boundaries():
    now = stamp_at(1_800_000_000_123_456_789)
    assert timestamp_in_window(stamp_at(1_800_000_000_123_456_789), now, .6, .05)
    assert timestamp_in_window(stamp_at(1_800_000_000_173_456_789), now, .6, .05)
    assert not timestamp_in_window(stamp_at(1_800_000_000_173_456_790), now, .6, .05)
    assert timestamp_in_window(stamp_at(1_799_999_999_523_456_789), now, .6, .05)
    assert not timestamp_in_window(stamp_at(1_799_999_999_523_456_788), now, .6, .05)


def test_estimator_accepts_matching_clock_and_rejects_old_and_future_flow():
    rclpy.init()
    node = EstimatorNode(parameter_overrides=[Parameter('heading_reference_valid',value=True),
                                               Parameter('publish_velocity',value=True)])
    try:
        now = node.get_clock().now().nanoseconds
        imu = Imu(); imu.header.stamp = stamp_at(now)
        imu.header.frame_id = 'imu_link'; imu.orientation.w = 1.0
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = 1e-6
        imu.orientation_covariance[8] = 1e-6
        node.on_imu(imu)
        assert node.tracker.heading_valid
        flow = OpticalFlow(); flow.metric_valid = True; flow.quality = 255
        flow.displacement_x_m = .05; flow.header.stamp = stamp_at(now)
        node.on_flow(flow)
        assert node.tracker.x == .05
        assert node.tracker.last_accepted_flow is not None
        flow.header.stamp = stamp_at(now + 1_000_000_000)
        node.on_flow(flow)
        assert node.tracker.x == .05
        assert node.tracker.reason == 'stale/future optical-flow timestamp'
        flow.header.stamp = stamp_at(now - 1_000_000_000)
        node.on_flow(flow)
        assert node.tracker.x == .05
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_simulator_imu_and_flow_share_one_sample_stamp():
    rclpy.init()
    node = SimulatorNode(parameter_overrides=[Parameter('startup_delay_s',value=0.0)])
    imu, flow, truth = [], [], []
    try:
        node.imu_pub.publish = imu.append
        node.flow_pub.publish = flow.append
        node.truth_pub.publish = truth.append
        node.tick()
        assert len(imu) == len(flow) == len(truth) == 1
        assert imu[0].header.stamp == flow[0].header.stamp == truth[0].header.stamp
    finally:
        node.destroy_node(); rclpy.shutdown()
