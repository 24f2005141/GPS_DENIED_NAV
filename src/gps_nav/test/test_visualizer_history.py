"""The recorder and renderer retain the same full estimated multi-turn route."""
import math
import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from gps_nav.core import Tracker
from gps_nav.demo import Monitor
from gps_nav.sim_core import GOALS, PathSimulator
from gps_nav.visualizer_node import VisualizerNode
from gps_nav.trajectory_history import TrajectoryHistory
from gps_nav_interfaces.msg import LocalizationStatus, NavigationStatus


def test_history_reset_discards_older_queued_poses():
    history = TrajectoryHistory()
    old = PoseStamped(); old.header.stamp.sec = 10; old.pose.position.x = 1.0
    newer = PoseStamped(); newer.header.stamp.sec = 12; newer.pose.position.x = 2.0
    assert history.add_pose(old)
    history.clear((11, 0))
    history.merge_path(Path(poses=[old, newer]))
    assert history.ordered_samples() == [((12, 0), (2.0, 0.0))]


def test_observed_path_length_omits_unmeasured_gap():
    history=TrajectoryHistory()
    for sec, x in [(10,1.0),(11,2.0),(13,5.0)]:
        pose=PoseStamped(); pose.header.stamp.sec=sec
        pose.pose.position.x=x
        history.add_pose(pose)
    length,gaps=history.length_from_start(max_gap_s=1.0)
    assert length == 2.0 and gaps == 1
    xs,_=history.plot_coordinates(max_gap_s=1.0)
    assert math.isnan(xs[2])


def test_destination_change_preserves_history_and_updates_graph(tmp_path):
    rclpy.init(domain_id=221)
    visual=VisualizerNode(parameter_overrides=[
        Parameter('headless',value=True),
        Parameter('draw_period_s',value=0.0),
        Parameter('mode',value='interactive'),
        Parameter('output_file',value=str(tmp_path/'interactive.png'))])
    try:
        east=PointStamped(); east.point.x=3.0
        visual.on_goal(east)
        pose=PoseStamped(); pose.header.stamp.sec=10
        pose.pose.orientation.w=1.0; pose.pose.position.x=1.0
        visual.on_pose(pose)
        arrived=NavigationStatus(); arrived.destination_reached=True
        visual.on_nav(arrived)
        visual.draw(save=False)
        assert 'Arrival event: yes' in visual.info.get_text()
        north=PointStamped(); north.point.y=3.0
        visual.on_goal(north)
        visual.draw(save=False)
        assert visual.goal == (0.0,3.0)
        assert len(visual.history.snapshot()) == 1
        assert 'Arrival event: no' in visual.info.get_text()
        assert 'interactive mode' in visual.ax.get_title()
        xlim=visual.ax.get_xlim(); ylim=visual.ax.get_ylim()
        assert xlim[1]-xlim[0] == pytest.approx(ylim[1]-ylim[0])
    finally:
        visual.destroy_node(); rclpy.try_shutdown()


def test_complete_multi_turn_plot_ignores_late_short_path_snapshot(tmp_path):
    rclpy.init(domain_id=220)
    visual = VisualizerNode(parameter_overrides=[
        Parameter('headless', value=True),
        Parameter('draw_period_s', value=0.0),
        Parameter('output_file', value=str(tmp_path/'multi.png'))])
    monitor = Monitor()
    try:
        goal = PointStamped(); goal.point.x, goal.point.y = GOALS['multi']
        visual.on_goal(goal)
        visual.draw(save=False)
        assert visual.draw_history == [0]
        simulator = PathSimulator('multi', dt=0.05)
        tracker = Tracker()
        poses = []
        while (sample := simulator.step()) is not None:
            assert tracker.imu(sample.yaw, sample.t)
            assert tracker.flow(sample.dx_body, sample.dy_body, sample.t, 255, True)
            pose = PoseStamped()
            seconds, nanoseconds = divmod(round(sample.t * 1_000_000_000), 1_000_000_000)
            pose.header.stamp.sec = 1_800_000_000 + seconds
            pose.header.stamp.nanosec = nanoseconds
            pose.pose.position.x = tracker.x
            pose.pose.position.y = tracker.y
            pose.pose.orientation.z = math.sin(tracker.yaw/2)
            pose.pose.orientation.w = math.cos(tracker.yaw/2)
            visual.on_pose(pose)
            monitor.on_pose(pose)
            poses.append(pose)
            if len(poses) in (40,100):
                visual.draw(save=False)
        assert len(poses) == 280
        assert visual.draw_history == [0,40,100]
        old_snapshot = Path(); old_snapshot.poses = poses[:5]
        visual.on_path(old_snapshot)  # A delayed path message must not truncate history.
        assert visual.history.samples == monitor.estimate_samples
        assert len(visual.history.samples) == len(poses)
        reached = NavigationStatus(); reached.destination_reached = True
        visual.on_nav(reached)
        visual.on_nav(NavigationStatus())  # Post-run health must not erase arrival.
        visual.on_status(LocalizationStatus())  # IMU is currently stale.
        visual.draw()
        assert visual.draw_history[-1] == 280
        lines = [line for line in visual.ax.lines if line.get_label() == 'Estimated trajectory']
        assert len(lines) == 1
        xs, ys = list(lines[0].get_xdata()), list(lines[0].get_ydata())
        assert len(xs) == len(poses)
        assert max(ys) > 1.9 and max(xs) > 2.9
        assert math.dist((xs[-1], ys[-1]), GOALS['multi']) < 1e-9
        assert visual.ax.get_xlim()[1] > GOALS['multi'][0]
        assert visual.ax.get_ylim()[1] > max(ys)
        assert 'Arrival event: yes' in visual.info.get_text()
        assert 'IMU: stale' in visual.info.get_text()
        assert (tmp_path/'multi.png').stat().st_size > 0
    finally:
        visual.destroy_node(); monitor.destroy_node(); rclpy.try_shutdown()
