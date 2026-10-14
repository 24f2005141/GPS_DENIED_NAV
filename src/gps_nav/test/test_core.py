import math
import pytest
from gps_nav.core import Navigator, Tracker, rotate, wrap
from gps_nav.sim_core import GoalSimulator, PathSimulator, GOALS


@pytest.mark.parametrize('deg,expected', [(0,(1,0)),(90,(0,1)),(180,(-1,0)),(270,(0,-1))])
def test_rotation(deg, expected):
    assert rotate(1, 0, math.radians(deg)) == pytest.approx(expected, abs=1e-12)


def test_tracker_guard_and_reset():
    t=Tracker(stale_s=0.5)
    assert not t.flow(1,0,1,255,False)
    assert t.imu(0,1)
    assert t.flow(1,0,1.1,255,True)
    assert t.x == 1 and not t.flow(1,0,1.1,255,True)
    assert not t.flow(1,0,1.2,0,True)
    assert not t.flow(1,0,2,255,True)
    assert not t.status(2)[0]
    t.reset(); assert t.x == 0 and not t.position_valid


def test_zero_and_reverse_motion():
    t=Tracker(); t.imu(0,1.0)
    assert t.flow(0,0,1.0,255,True)
    assert (t.x,t.y)==(0,0)
    assert t.flow(-0.5,0,1.1,255,True)
    assert t.x == pytest.approx(-0.5)


def test_heading_and_mounting():
    t=Tracker(); t.imu(math.pi/2, 0)
    assert t.flow(1,0,0.1,255,True)
    assert (t.x,t.y)==pytest.approx((0,1),abs=1e-12)
    t.reset(); t.imu(0,0)
    assert t.flow(1,0,0.1,255,True,math.pi/2)
    assert (t.x,t.y)==pytest.approx((0,1),abs=1e-12)


def test_guidance():
    n=Navigator(tolerance=0.2,stable_updates=2)
    assert n.evaluate(0,0,0,1,0,True,True).label=='GO STRAIGHT'
    assert n.evaluate(0,0,0,0,1,True,True).label=='TURN LEFT'
    assert n.evaluate(0,0,0,0,-1,True,True).label=='TURN RIGHT'
    assert n.evaluate(0,0,0,-1,0,True,True).label=='GO BACK'
    assert n.evaluate(0,0,0,1,0,True,False).label=='HEADING UNAVAILABLE'
    assert n.evaluate(0,0,0,1,0,False,True).label=='POSITION UNRELIABLE'
    assert n.evaluate(0,0,0,1,0,True,True,True).label=='SENSOR DEGRADED'
    assert not n.evaluate(0,0,0,0.1,0,True,True).reached
    assert n.evaluate(0,0,0,0.1,0,True,True).reached
    assert wrap(math.pi + 0.1) == pytest.approx(-math.pi+0.1)


@pytest.mark.parametrize('path', ['straight','turn','multi'])
def test_sim_truth(path):
    s=PathSimulator(path, dt=0.05)
    while s.step() is not None: pass
    assert (s.x,s.y)==pytest.approx(GOALS[path], abs=1e-10)


def test_manual_simulation():
    s=PathSimulator(dt=0.05)
    for _ in range(20): s.manual_step(1.0,0.0)
    assert (s.x,s.y)==pytest.approx((1.0,0.0),abs=1e-12)
    for _ in range(20): s.manual_step(0.0,math.pi/2)
    for _ in range(20): s.manual_step(1.0,0.0)
    assert (s.x,s.y)==pytest.approx((1.0,1.0),abs=1e-12)


def test_goal_simulator_straight_baseline_and_initial_turn():
    straight=GoalSimulator(3.0,0.0,speed=0.6,dt=0.05)
    samples=[straight.step() for _ in range(100)]
    assert (straight.x,straight.y)==pytest.approx((3.0,0.0),abs=1e-10)
    assert all(abs(sample.y) < 1e-10 for sample in samples)
    assert all(sample.dx_body >= 0 for sample in samples)
    assert sum(math.hypot(sample.dx_body,sample.dy_body) for sample in samples)==pytest.approx(3.0)
    turned=GoalSimulator(1.0,0.0,speed=0.5,turn_rate=math.pi/2,
                         initial_heading=math.pi/2,dt=0.05)
    first=turned.step()
    assert (first.x,first.y)==(0.0,0.0)
    assert first.yaw_rate == pytest.approx(-math.pi/2)
    for _ in range(80): turned.step()
    assert (turned.x,turned.y)==pytest.approx((1.0,0.0),abs=1e-10)


def test_multi_path_remains_non_straight():
    sim=PathSimulator('multi')
    visited=[]
    while (sample:=sim.step()) is not None:
        visited.append((sample.x,sample.y))
    assert max(y for _,y in visited) > 1.9
    assert (sim.x,sim.y)==pytest.approx((3.0,0.0),abs=1e-10)
