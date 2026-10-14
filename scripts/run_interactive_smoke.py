"""Exercise an interactive demo with external ROS motion and destination updates."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from gps_nav_interfaces.srv import SetDestination


parser=argparse.ArgumentParser()
parser.add_argument('--output-dir',default='reports/interactive_smoke')
args=parser.parse_args()
out=Path(args.output_dir).resolve(); out.mkdir(parents=True,exist_ok=True)
domain=211
with (out/'stdout.txt').open('w') as stdout, (out/'stderr.txt').open('w') as stderr:
    proc=subprocess.Popen(['ros2','run','gps_nav','demo','--mode','interactive',
                           '--goal-x-m','3','--goal-y-m','0','--max-duration-s','8',
                           '--domain-id',str(domain),'--output-dir',str(out)],
                          stdout=stdout,stderr=stderr)
    rclpy.init(domain_id=domain)
    node=Node('interactive_smoke_controller')
    try:
        command=node.create_publisher(Twist,'/simulation/cmd_vel',10)
        client=node.create_client(SetDestination,'/set_destination')
        if not client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('destination service did not appear')
        start=time.monotonic()
        moving=Twist(); moving.linear.x=0.6
        while time.monotonic()-start < 5.0:
            command.publish(moving)
            rclpy.spin_once(node,timeout_sec=0.1)
        request=SetDestination.Request(latitude=0.000027,longitude=0.0)
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=3.0)
        if not future.done() or not future.result().success:
            raise RuntimeError('destination update failed')
        stopped=Twist()
        for _ in range(5):
            command.publish(stopped)
            rclpy.spin_once(node,timeout_sec=0.1)
        code=proc.wait(timeout=20.0)
    finally:
        node.destroy_node(); rclpy.try_shutdown()
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5.0)
result=json.loads((out/'interactive.json').read_text())
print(json.dumps({'exit_code':code,'mode':result['mode'],
                  'trajectory_samples':result['trajectory_samples'],
                  'estimated_m':result['estimated_m'],
                  'initial_goal_m':result['initial_goal_m'],
                  'final_goal_m':result['goal_m'],
                  'plot_history_matches_metrics':result['plot_history_matches_metrics'],
                  'destination_reached':result['destination_reached']},indent=2))
if (code or result['mode'] != 'interactive' or result['estimated_m'][0] < 0.3 or
    result['goal_m'][1] < 2.8 or not result['plot_history_matches_metrics'] or
    result['destination_reached']):
    raise SystemExit(1)
