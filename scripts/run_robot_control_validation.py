"""End-to-end closed-loop simulation, including STOP, link fault and recovery."""
import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PointStamped, PoseStamped
from gps_nav_interfaces.msg import (LocalizationStatus, NavigationStatus,
                                    RobotAck, RobotCommand, RobotControlStatus,
                                    RobotLinkStatus)


parser=argparse.ArgumentParser()
parser.add_argument('--output-dir',default='reports/robot_control_validation')
args=parser.parse_args()
out=Path(args.output_dir).resolve(); out.mkdir(parents=True,exist_ok=True)
domain=100+secrets.randbelow(100)
env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(domain)
launch=['ros2','launch','gps_nav','robot_simulation.launch.py',
        'headless:=true',f'output_file:={out/"trajectory.png"}']
events={'commands':[],'acks':[],'truth':[],'poses':[],'control':None,'link':None,
        'localization':None,'navigation':None}

def record_command(msg):
    events['commands'].append({'sequence':msg.sequence,'command':msg.command,
                               'speed_permille':msg.speed_permille})

def record_ack(msg):
    events['acks'].append({'sequence':msg.sequence,'accepted':msg.accepted,
                           'command':msg.command,'reason':msg.reason})

def record_truth(msg): events['truth'].append((msg.point.x,msg.point.y))

def record_pose(msg):
    events['poses'].append((msg.header.stamp.sec,msg.header.stamp.nanosec,
                           msg.pose.position.x,msg.pose.position.y))

def latest(key): return lambda msg:events.__setitem__(key,msg)

def wait_for(node,predicate,seconds,label):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.1)
        if predicate(): return
    raise RuntimeError(f'timed out waiting for {label}; '
                       f'control={events["control"]} link={events["link"]} '
                       f'localization={events["localization"]} '
                       f'navigation={events["navigation"]}')

def call(node,client,request):
    assert client.wait_for_service(timeout_sec=5.0)
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=5.0)
    if not future.done() or not future.result().success:
        raise RuntimeError(f'service failed: {future.result() if future.done() else "timeout"}')
    return future.result().message

with (out/'launch_stdout.txt').open('w') as stdout, (out/'launch_stderr.txt').open('w') as stderr:
    proc=subprocess.Popen(launch,stdout=stdout,stderr=stderr,env=env)
    rclpy.init(domain_id=domain)
    node=Node('robot_control_validation')
    try:
        node.create_subscription(RobotCommand,'/robot/command',record_command,100)
        node.create_subscription(RobotAck,'/robot/ack',record_ack,100)
        node.create_subscription(PointStamped,'/simulation/ground_truth',record_truth,100)
        node.create_subscription(PoseStamped,'/estimated_position',record_pose,100)
        node.create_subscription(RobotControlStatus,'/robot/control_status',latest('control'),20)
        node.create_subscription(RobotLinkStatus,'/robot/link_status',latest('link'),20)
        node.create_subscription(LocalizationStatus,'/localization_status',latest('localization'),20)
        node.create_subscription(NavigationStatus,'/navigation_status',latest('navigation'),20)
        enable=node.create_client(SetBool,'/robot/set_enabled')
        disconnect=node.create_client(SetBool,'/robot/sim_link/set_connected')
        reset=node.create_client(Trigger,'/robot/reset_fault')

        wait_for(node,lambda: events['control'] is not None and events['link'] is not None and
                 events['localization'] is not None and events['navigation'] is not None and
                 events['localization'].position_valid and events['localization'].heading_valid and
                 events['navigation'].distance_valid and events['link'].ack_fresh,
                 15,'ready stationary graph')
        stationary_start=events['truth'][-1] if events['truth'] else None
        stationary_deadline=time.monotonic()+0.4
        while time.monotonic()<stationary_deadline:
            rclpy.spin_once(node,timeout_sec=0.05)
        stationary_end=events['truth'][-1]
        assert stationary_start is not None and abs(stationary_end[0]-stationary_start[0])<1e-9, (
            f'pre-enable motion: start={stationary_start} end={stationary_end} '
            f'control={events["control"]} commands={events["commands"][-5:]}')
        assert not events['control'].enabled
        arm_message=call(node,enable,SetBool.Request(data=True))
        wait_for(node,lambda: events['control'] and events['control'].enabled and
                 any(c['command']=='FORWARD' for c in events['commands']),6,'FORWARD command')
        wait_for(node,lambda: events['truth'] and events['truth'][-1][0]>0.6,8,'robot motion')
        before_disconnect=events['truth'][-1]
        disconnect_message=call(node,disconnect,SetBool.Request(data=False))
        wait_for(node,lambda: events['control'] and events['control'].fault_latched and
                 events['control'].last_issued_command=='STOP',4,'fault STOP')
        stopped_at=events['truth'][-1]
        pause_until=time.monotonic()+0.7
        while time.monotonic()<pause_until: rclpy.spin_once(node,timeout_sec=0.05)
        stopped_after=events['truth'][-1]
        assert abs(stopped_after[0]-stopped_at[0])<0.05
        reconnect_message=call(node,disconnect,SetBool.Request(data=True))
        wait_for(node,lambda: events['link'] and events['link'].ack_fresh and
                 events['localization'] and events['localization'].position_valid,5,'recovered link')
        reset_message=call(node,reset,Trigger.Request())
        wait_for(node,lambda: events['control'] and not events['control'].fault_latched,5,'fault reset ACK')
        assert not events['control'].enabled
        rearm_message=call(node,enable,SetBool.Request(data=True))
        wait_for(node,lambda: events['control'] and events['control'].arrival_latched and
                 events['control'].last_issued_command=='STOP',15,'arrival STOP')
        # Let the independent visualizer consume the latched arrival before shutdown.
        settle_until=time.monotonic()+0.3
        while time.monotonic()<settle_until: rclpy.spin_once(node,timeout_sec=0.05)
        final_truth=events['truth'][-1]
        final_estimate=events['poses'][-1][2:]
        assert events['control'].arrival_latched and abs(final_estimate[0]-3.0)<0.25
        report={'ros_domain_id':domain,'startup_stopped':True,
                'stationary_start':stationary_start,'stationary_end':stationary_end,
                'arm_response':arm_message,'before_disconnect':before_disconnect,
                'disconnect_response':disconnect_message,
                'stopped_at':stopped_at,'stopped_after':stopped_after,
                'reconnect_response':reconnect_message,
                'reset_response':reset_message,'rearm_response':rearm_message,
                'final_truth_m':final_truth,'final_estimate_m':final_estimate,
                'estimated_pose_samples':len(events['poses']),
                'arrival_latched':events['control'].arrival_latched,
                'final_command':events['control'].last_issued_command,
                'command_counts':{name:sum(c['command']==name for c in events['commands'])
                                  for name in ('STOP','ARM','RESET','FORWARD','LEFT','RIGHT')},
                'accepted_acks':sum(a['accepted'] for a in events['acks']),
                'rejected_acks':sum(not a['accepted'] for a in events['acks']),
                'example_commands':events['commands'][:5],
                'example_acks':events['acks'][:5]}
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        node.destroy_node(); rclpy.try_shutdown()
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.terminate(); proc.wait(timeout=5)
