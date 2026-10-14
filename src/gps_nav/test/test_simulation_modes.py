"""Mode selection and continuous manual motion guards."""
import math

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

from gps_nav.demo import arguments, destination
from gps_nav.sim_core import GoalSimulator, PathSimulator
from gps_nav.simulator_node import SimulatorNode


def test_goal_mode_is_default_and_distinct_from_multi_route():
    parser=arguments()
    args=parser.parse_args([])
    assert args.mode == 'goal'
    _,goal,_=destination(parser,args)
    assert goal == pytest.approx((3.0,0.0),abs=1e-5)
    goal_sim=GoalSimulator(3.0,0.0)
    goal_samples=[goal_sim.step() for _ in range(100)]
    assert max(abs(s.y) for s in goal_samples) < 1e-10
    assert goal_samples[-1].x == pytest.approx(3.0)
    multi=PathSimulator('multi')
    multi_samples=[]
    while (sample := multi.step()) is not None:
        multi_samples.append(sample)
    assert max(s.y for s in multi_samples) > 1.9
    assert len(multi_samples) > len(goal_samples)


def test_goal_motion_respects_initial_heading_and_never_teleports():
    sim=GoalSimulator(3.0,0.0,speed=0.6,turn_rate=math.pi/2,
                      initial_heading=math.pi/2,dt=0.05)
    samples=[sim.step() for _ in range(130)]
    assert samples[0].x==0.0 and samples[0].y==0.0
    assert samples[0].yaw < math.pi/2
    assert samples[19].x==pytest.approx(0.0,abs=1e-10)
    assert samples[-1].x==pytest.approx(3.0)
    assert max(abs(s.y) for s in samples)<1e-9
    assert max(math.dist((a.x,a.y),(b.x,b.y)) for a,b in zip(samples,samples[1:])) <= 0.03+1e-10


def test_interactive_node_keeps_stepping_until_explicit_stop():
    rclpy.init(domain_id=222)
    node=SimulatorNode(parameter_overrides=[
        Parameter('mode',value='interactive'),
        Parameter('max_duration_s',value=0.0),
        Parameter('startup_delay_s',value=0.0)])
    try:
        cmd=Twist(); cmd.linear.x=0.6
        node.on_command(cmd)
        for _ in range(100):
            node.tick()
        assert node.sim.x == pytest.approx(3.0,abs=1e-9)
        assert node.sim.y == pytest.approx(0.0,abs=1e-9)
        assert node.sim.t == pytest.approx(5.0)
        assert not node.done
        cmd.linear.x=0.0; cmd.angular.z=math.pi/2
        node.on_command(cmd)
        for _ in range(20):
            node.tick()
        assert node.sim.y == pytest.approx(0.0,abs=1e-9)
        assert node.sim.yaw == pytest.approx(math.pi/2)
        assert not node.done
    finally:
        node.destroy_node(); rclpy.try_shutdown()
