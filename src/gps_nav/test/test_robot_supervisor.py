from gps_nav.robot_protocol import Ack
from gps_nav.robot_supervisor_core import ControlSupervisor, Guidance, Health, Link


def ready(supervisor,now=1.0,label='GO STRAIGHT'):
    supervisor.update_health(Health(True,True,True,True,True,now))
    supervisor.update_guidance(Guidance(label,True,False,now))
    supervisor.update_link(Link(True,True,now))


def arm(supervisor,now=1.0):
    ready(supervisor,now)
    assert supervisor.request_enable(now)[0]
    command=supervisor.tick(now)
    assert command.direction=='ARM'
    supervisor.acknowledge(Ack(command.sequence,True,'ARM','ARMED'))
    assert supervisor.enabled


def test_disabled_startup_and_explicit_enable_with_ack():
    supervisor=ControlSupervisor(allow_control=True)
    assert supervisor.tick(0).direction=='STOP'
    assert not supervisor.request_enable(0)[0]
    ready(supervisor)
    assert supervisor.request_enable(1)[0]
    arm_command=supervisor.tick(1)
    assert arm_command.direction=='ARM'
    assert supervisor.tick(1.1) is None
    supervisor.acknowledge(Ack(arm_command.sequence,True,'ARM','ARMED'))
    move=supervisor.tick(1.11)
    assert move.direction=='FORWARD' and move.speed_permille==80


def test_physical_gate_sensor_failure_latches_stop_and_requires_reset():
    supervisor=ControlSupervisor(allow_control=False)
    ready(supervisor)
    assert not supervisor.request_enable(1)[0]
    supervisor.allow_control=True
    arm(supervisor)
    assert supervisor.tick(1.01).direction=='FORWARD'
    supervisor.update_health(Health(True,False,True,True,True,1.02))
    assert supervisor.tick(1.02).direction=='STOP'
    assert supervisor.fault_latched and not supervisor.enabled
    assert not supervisor.request_enable(1.03)[0]
    ready(supervisor,1.04)
    assert supervisor.request_reset(1.04)[0]
    reset=supervisor.tick(1.04)
    assert reset.direction=='RESET'
    supervisor.acknowledge(Ack(reset.sequence,True,'RESET','RESET'))
    assert not supervisor.fault_latched and not supervisor.enabled
    assert supervisor.request_enable(1.05)[0]


def test_stale_guidance_link_loss_ack_timeout_and_arrival():
    supervisor=ControlSupervisor(allow_control=True,input_stale_s=0.2,ack_timeout_s=0.2)
    arm(supervisor)
    assert supervisor.tick(1.21).direction=='STOP'  # All inputs are stale.
    assert supervisor.fault_latched
    ready(supervisor,2.0)
    assert supervisor.request_reset(2)[0]
    assert supervisor.tick(2).direction=='RESET'
    assert supervisor.tick(2.21).direction=='STOP'
    assert supervisor.fault_latched
    supervisor=ControlSupervisor(allow_control=True)
    arm(supervisor)
    supervisor.update_link(Link(False,False,1.1))
    assert supervisor.tick(1.1).direction=='STOP' and supervisor.fault_latched
    supervisor=ControlSupervisor(allow_control=True)
    arm(supervisor)
    supervisor.update_guidance(Guidance('DESTINATION REACHED',True,True,1.1))
    assert supervisor.tick(1.1).direction=='STOP'
    assert supervisor.arrival_latched and not supervisor.enabled
    supervisor.update_health(Health(False,False,False,False,False,1.2))
    assert supervisor.tick(1.2).direction=='STOP'
    assert supervisor.reason=='destination reached'


def test_back_guidance_never_reverses_without_permission():
    supervisor=ControlSupervisor(allow_control=True)
    arm(supervisor)
    supervisor.update_guidance(Guidance('GO BACK',True,False,1.1))
    assert supervisor.tick(1.1).direction=='STOP'
    assert not supervisor.enabled
    reverse=ControlSupervisor(allow_control=True,allow_reverse=True)
    arm(reverse)
    reverse.update_guidance(Guidance('GO BACK',True,False,1.1))
    assert reverse.tick(1.1).direction=='BACKWARD'


def test_destination_change_requires_reenable_without_erasing_arrival_history():
    supervisor=ControlSupervisor(allow_control=True)
    supervisor.set_goal((3,0))
    arm(supervisor)
    supervisor.set_goal((0,3))
    assert supervisor.tick(1.1).direction=='STOP'
    assert not supervisor.enabled
    assert supervisor.reason=='destination changed; enable control again'
