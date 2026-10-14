"""ROS simulation with shared metrics and live or headless visualization."""
import argparse
import json
import math
from pathlib import Path
import secrets
from threading import Event, Thread
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from std_msgs.msg import Bool
from gps_nav_interfaces.msg import NavigationStatus, LocalizationStatus

from .geo import ENU, nearby_geodetic, validate
from .sim_core import GOALS
from .waypoint_node import WaypointNode
from .preprocess_node import PreprocessNode
from .estimator_node import EstimatorNode
from .navigator_node import NavigatorNode
from .simulator_node import SimulatorNode
from .visualizer_node import VisualizerNode
from .trajectory_history import TrajectoryHistory


class Monitor(Node):
    def __init__(self):
        super().__init__('demo_monitor')
        self.truth=None; self.pose=None; self.goal=None; self.nav=None; self.status=None; self.done=False
        self.arrival_seen=False
        self.truth_samples={}; self.trajectory=TrajectoryHistory()
        self.estimate_samples=self.trajectory.samples
        self.velocity_samples=0; self.last_velocity=None
        self.create_subscription(PointStamped,'/simulation/ground_truth',self.on_truth,100)
        self.create_subscription(PointStamped,'/navigation_goal',self.on_goal,10)
        self.create_subscription(PoseStamped,'/estimated_position',self.on_pose,100)
        self.create_subscription(NavigationStatus,'/navigation_status',self.on_nav,100)
        self.create_subscription(LocalizationStatus,'/localization_status',self.on_status,100)
        self.create_subscription(Bool,'/simulation/done',self.on_done,10)
        self.create_subscription(TwistStamped,'/estimated_velocity',self.on_velocity,100)

    def on_truth(self,m):
        self.truth=(m.point.x,m.point.y)
        self.truth_samples[(m.header.stamp.sec,m.header.stamp.nanosec)]=self.truth
    def on_goal(self,m):
        new_goal=(m.point.x,m.point.y)
        if self.goal is not None and math.dist(self.goal,new_goal) > 1e-8:
            self.arrival_seen=False
        self.goal=new_goal
    def on_pose(self,m):
        self.pose=(m.pose.position.x,m.pose.position.y)
        self.trajectory.add_pose(m)
    def on_nav(self,m):
        self.nav=m
        self.arrival_seen |= m.destination_reached
    def on_status(self,m): self.status=m
    def on_done(self,m): self.done=m.data
    def on_velocity(self,m):
        self.velocity_samples += 1
        self.last_velocity=(m.twist.linear.x,m.twist.linear.y)


def p(name,value): return Parameter(name,value=value)


def arguments():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=('goal','scripted','interactive'),default='goal')
    parser.add_argument('--path',choices=GOALS,default='straight',
                        help='scripted validation path; ignored in goal mode')
    parser.add_argument('--start-lat','--origin-lat',dest='origin_lat',type=float)
    parser.add_argument('--start-lon','--origin-lon',dest='origin_lon',type=float)
    parser.add_argument('--destination-lat','--goal-lat',dest='goal_lat',type=float)
    parser.add_argument('--destination-lon','--goal-lon',dest='goal_lon',type=float)
    parser.add_argument('--goal-x-m',type=float)
    parser.add_argument('--goal-y-m',type=float)
    parser.add_argument('--speed-m-s',type=float,default=0.6)
    parser.add_argument('--turn-rate-deg-s',type=float,default=90.0)
    parser.add_argument('--initial-heading-deg',type=float,
                        help='initial ENU heading: 0 East, 90 North; defaults to 0')
    parser.add_argument('--max-duration-s',type=float,
                        help='defaults to 30 s; unlimited in interactive mode')
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--flow-hz',type=float,default=20.0)
    parser.add_argument('--imu-hz',type=float,default=20.0)
    parser.add_argument('--noise-m',type=float,default=0.0)
    parser.add_argument('--yaw-bias-rad-s',type=float,default=0.0)
    parser.add_argument('--flow-dropout-start',type=float,default=-1.0)
    parser.add_argument('--flow-dropout-end',type=float,default=-1.0)
    parser.add_argument('--imu-dropout-start',type=float,default=-1.0)
    parser.add_argument('--imu-dropout-end',type=float,default=-1.0)
    parser.add_argument('--low-quality-start',type=float,default=-1.0)
    parser.add_argument('--low-quality-end',type=float,default=-1.0)
    parser.add_argument('--output-dir',default='reports')
    parser.add_argument('--show',action='store_true',help='show the graph live and keep it open after completion')
    parser.add_argument('--capture-live-frames',action='store_true',
                        help='save initial and in-progress GUI frames for diagnostics')
    parser.add_argument('--gui-hold-seconds',type=float,default=None,
                        help='optional finite post-run GUI hold, useful for automated GUI smoke tests')
    parser.add_argument('--domain-id',type=int,default=None,
                        help='ROS domain; defaults to a fresh isolated domain')
    return parser


def destination(parser,args):
    try:
        origin_lat=0.0 if args.origin_lat is None else args.origin_lat
        origin_lon=0.0 if args.origin_lon is None else args.origin_lon
        validate(origin_lat,origin_lon)
        gps_pair=(args.goal_lat is not None or args.goal_lon is not None)
        local_pair=(args.goal_x_m is not None or args.goal_y_m is not None)
        if gps_pair and local_pair:
            parser.error('choose geographic or local goal coordinates, not both')
        if gps_pair:
            if args.goal_lat is None or args.goal_lon is None:
                parser.error('provide both --goal-lat and --goal-lon')
            validate(args.goal_lat,args.goal_lon)
            goal_geo=(args.goal_lat,args.goal_lon)
            default_scripted=False
        else:
            if local_pair and (args.goal_x_m is None or args.goal_y_m is None):
                parser.error('provide both --goal-x-m and --goal-y-m')
            offset=(args.goal_x_m,args.goal_y_m) if local_pair else (
                GOALS[args.path] if args.mode == 'scripted' else (3.0,0.0))
            goal_geo=nearby_geodetic(origin_lat,origin_lon,*offset)
            default_scripted=not local_pair and args.mode == 'scripted'
        local=ENU(origin_lat,origin_lon).to_local(*goal_geo)
        return goal_geo,local,default_scripted
    except ValueError as exc:
        parser.error(str(exc))


def read_number(label, lower, upper, input_fn, output_fn, default=None):
    while True:
        try:
            raw=input_fn(label)
        except EOFError as exc:
            raise ValueError(f'input ended while reading {label.strip()}') from exc
        if not raw.strip() and default is not None:
            return default
        try:
            value=float(raw)
        except ValueError:
            output_fn(f'Invalid number for {label.strip()}; try again.')
            continue
        if not math.isfinite(value) or not lower <= value <= upper:
            output_fn(f'{label.strip()} must be a finite number from {lower:g} to {upper:g}; try again.')
            continue
        return value


def resolve_coordinates(parser,args,input_fn=None,output_fn=None):
    """Prompt only for absent geographic fields in goal mode; keep local-goal runs noninteractive."""
    input_fn=input if input_fn is None else input_fn
    output_fn=print if output_fn is None else output_fn
    local_any=args.goal_x_m is not None or args.goal_y_m is not None
    geo_any=args.goal_lat is not None or args.goal_lon is not None
    if local_any and geo_any:
        parser.error('choose geographic or local goal coordinates, not both')
    if args.mode != 'goal':
        args.origin_lat=0.0 if args.origin_lat is None else args.origin_lat
        args.origin_lon=0.0 if args.origin_lon is None else args.origin_lon
        args.initial_heading_deg=0.0 if args.initial_heading_deg is None else args.initial_heading_deg
        return
    if local_any:
        if args.goal_x_m is None or args.goal_y_m is None:
            parser.error('provide both --goal-x-m and --goal-y-m')
        if (args.origin_lat is None) != (args.origin_lon is None):
            parser.error('provide both start coordinates with a local goal')
        args.origin_lat=0.0 if args.origin_lat is None else args.origin_lat
        args.origin_lon=0.0 if args.origin_lon is None else args.origin_lon
        args.initial_heading_deg=0.0 if args.initial_heading_deg is None else args.initial_heading_deg
        return
    needs_prompt=any(v is None for v in (args.origin_lat,args.origin_lon,args.goal_lat,args.goal_lon))
    if needs_prompt:
        output_fn('GPS-Denied Navigation Assistance')
        output_fn('Mode: Goal-directed simulation')
        if args.origin_lat is None or args.origin_lon is None:
            output_fn('\nEnter current position:')
        if args.origin_lat is None:
            args.origin_lat=read_number('Current latitude: ',-90,90,input_fn,output_fn)
        if args.origin_lon is None:
            args.origin_lon=read_number('Current longitude: ',-180,180,input_fn,output_fn)
        if args.goal_lat is None or args.goal_lon is None:
            output_fn('\nEnter destination:')
        if args.goal_lat is None:
            args.goal_lat=read_number('Destination latitude: ',-90,90,input_fn,output_fn)
        if args.goal_lon is None:
            args.goal_lon=read_number('Destination longitude: ',-180,180,input_fn,output_fn)
    try:
        validate(args.origin_lat,args.origin_lon)
        validate(args.goal_lat,args.goal_lon)
    except ValueError as exc:
        parser.error(str(exc))
    while math.hypot(*ENU(args.origin_lat,args.origin_lon).to_local(args.goal_lat,args.goal_lon)) < 0.01:
        if not needs_prompt:
            parser.error('destination is within 0.01 m of the start; choose a distinct waypoint')
        output_fn('Destination is within 0.01 m of the start; enter a different destination.')
        args.goal_lat=read_number('Destination latitude: ',-90,90,input_fn,output_fn)
        args.goal_lon=read_number('Destination longitude: ',-180,180,input_fn,output_fn)
    if args.initial_heading_deg is None:
        args.initial_heading_deg=(read_number(
            '\nInitial heading in degrees [0 = East, 90 = North] (default 0): ',
            -float('inf'),float('inf'),input_fn,output_fn,default=0.0)
            if needs_prompt else 0.0)


def select_gui_backend(parser):
    import matplotlib.pyplot as plt
    for backend in ('QtAgg','TkAgg'):
        try:
            plt.switch_backend(backend)
            return
        except ImportError:
            continue
    parser.error('no usable Matplotlib GUI backend/display is available')


def main():
    parser=arguments(); args=parser.parse_args()
    if args.mode != 'scripted' and args.path != 'straight':
        parser.error('--path selects a validation route; use --mode scripted')
    try:
        resolve_coordinates(parser,args)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.initial_heading_deg):
        parser.error('--initial-heading-deg must be finite')
    if args.mode == 'goal' and (not math.isfinite(args.speed_m_s) or args.speed_m_s <= 0 or
                                not math.isfinite(args.turn_rate_deg_s) or args.turn_rate_deg_s <= 0):
        parser.error('goal speed and turning rate must be finite and positive')
    if args.gui_hold_seconds is not None and args.gui_hold_seconds < 0:
        parser.error('--gui-hold-seconds must be nonnegative')
    domain_id=args.domain_id if args.domain_id is not None else 100+secrets.randbelow(101)
    if not 0 <= domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    goal_geo,goal,default_scripted=destination(parser,args)
    if args.mode == 'goal' and math.hypot(*goal) < 0.01:
        parser.error('destination is within 0.01 m of the start; choose a distinct waypoint')
    if args.max_duration_s is None:
        args.max_duration_s=(0.0 if args.mode == 'interactive' else
                             max(30.0,math.hypot(*goal)/args.speed_m_s+
                                 math.pi/math.radians(args.turn_rate_deg_s)+10.0)
                             if args.mode == 'goal' else 30.0)
    if not math.isfinite(args.max_duration_s) or args.max_duration_s < 0 or (
            args.mode != 'interactive' and args.max_duration_s == 0):
        parser.error('--max-duration-s must be finite and positive, or zero for unlimited interactive mode')
    if args.mode == 'interactive' and not args.show and args.max_duration_s == 0:
        parser.error('headless interactive mode needs --max-duration-s or use --show')
    if args.show: select_gui_backend(parser)
    output=Path(args.output_dir).resolve(); output.mkdir(parents=True,exist_ok=True)
    run_name=args.path if args.mode == 'scripted' else args.mode
    plot_file=output/(run_name+'.png')
    motion=(f'goal speed={args.speed_m_s:.3f}m/s '
            f'turn_rate={args.turn_rate_deg_s:.3f}deg/s' if args.mode == 'goal' else
            f'scripted path={args.path}' if args.mode == 'scripted' else
            'interactive initial command=(0.000m/s,0.000rad/s)')
    print(f'effective mode={args.mode} geographic start=({args.origin_lat:.7f},'
          f'{args.origin_lon:.7f}) destination=({goal_geo[0]:.7f},{goal_geo[1]:.7f})\n'
          f'Local start: (0.000, 0.000) m\n'
          f'Local destination: ({goal[0]:.3f}, {goal[1]:.3f}) m\n'
          f'Straight-line distance: {math.hypot(*goal):.3f} m\n'
          f'Initial heading: {args.initial_heading_deg:.3f} degrees\n'
          f'{motion} dt=0.050s timeout={args.max_duration_s:.3f}s '
          f'ROS_DOMAIN_ID={domain_id}',flush=True)

    rclpy.init(domain_id=domain_id)
    nodes=[]; executor=SingleThreadedExecutor(); spin_thread=None; stop_spin=Event()
    spin_errors=[]
    try:
        waypoint=WaypointNode(parameter_overrides=[
            p('origin_lat',args.origin_lat),p('origin_lon',args.origin_lon),
            p('goal_lat',goal_geo[0]),p('goal_lon',goal_geo[1])])
        preprocess=PreprocessNode(parameter_overrides=[
            p('flow_scale_x',0.01),p('flow_scale_y',0.01),p('calibration_valid',True)])
        estimator=EstimatorNode(parameter_overrides=[
            p('heading_reference_valid',True),p('publish_velocity',True)])
        navigator=NavigatorNode()
        visual=VisualizerNode(parameter_overrides=[
            p('headless',not args.show),p('output_file',str(plot_file)),
            p('draw_period_s',0.0),p('save_on_draw',False),
            p('mode',args.mode)])
        goal_msg=PointStamped(); goal_msg.point.x,goal_msg.point.y=goal
        visual.on_goal(goal_msg)  # Initial graph is ready before sensor movement.
        sim=SimulatorNode(parameter_overrides=[
            p('mode',args.mode),p('path',args.path),p('goal_x_m',goal[0]),p('goal_y_m',goal[1]),
            p('speed_m_s',args.speed_m_s),
            p('turn_rate_rad_s',math.radians(args.turn_rate_deg_s)),
            p('align_initial_heading_to_goal',False),
            p('initial_heading_deg',args.initial_heading_deg),
            p('max_duration_s',args.max_duration_s),
            p('seed',args.seed),p('flow_hz',args.flow_hz),p('imu_hz',args.imu_hz),
            p('noise_m',args.noise_m),p('yaw_bias_rad_s',args.yaw_bias_rad_s),
            p('flow_dropout_start',args.flow_dropout_start),
            p('flow_dropout_end',args.flow_dropout_end),
            p('imu_dropout_start',args.imu_dropout_start),
            p('imu_dropout_end',args.imu_dropout_end),
            p('low_quality_start',args.low_quality_start),
            p('low_quality_end',args.low_quality_end)])
        monitor=Monitor()
        nodes=[waypoint,preprocess,estimator,navigator,visual,sim,monitor]
        for node in nodes: executor.add_node(node)

        first_draw_before_truth=None; live_draws=0; live_max_poses=0
        live_capture_pose_count=None; initial_frame=None; progress_frame=None
        max_wall_time=(float('inf') if args.mode == 'interactive' and args.max_duration_s == 0
                       else max(60.0,args.max_duration_s+12.0))
        if args.show:
            import matplotlib.pyplot as plt
            visual.draw(save=False)
            plt.show(block=False)
            first_draw_before_truth=not monitor.truth_samples
            if args.capture_live_frames:
                initial_frame=output/(run_name+'_initial.png')
                visual.fig.savefig(initial_frame,dpi=120)

            def spin_ros():
                try:
                    while not stop_spin.is_set() and rclpy.ok():
                        executor.spin_once(timeout_sec=0.05)
                except ExternalShutdownException:
                    pass
                except Exception as exc:
                    spin_errors.append(exc)

            spin_thread=Thread(target=spin_ros,name='demo_ros_executor',daemon=True)
            spin_thread.start()

        start=time.monotonic(); finished_at=None; next_draw=start; user_stopped=False
        try:
            while time.monotonic()-start < max_wall_time:
                if args.show:
                    if visual.closed:
                        user_stopped=True
                        break
                    now=time.monotonic()
                    if now >= next_draw:
                        visual.draw(save=False)
                        if not monitor.done:
                            live_draws += 1
                            pose_count=len(visual.history.snapshot())
                            live_max_poses=max(live_max_poses,pose_count)
                            if args.capture_live_frames and progress_frame is None and pose_count >= 10:
                                progress_frame=output/(run_name+'_in_progress.png')
                                visual.fig.savefig(progress_frame,dpi=120)
                                live_capture_pose_count=pose_count
                        next_draw=now+0.1
                    plt.pause(0.01)
                else:
                    executor.spin_once(timeout_sec=0.1)
                if spin_errors: raise RuntimeError('ROS executor failed') from spin_errors[0]
                if monitor.done and finished_at is None: finished_at=time.monotonic()
                if finished_at is not None and time.monotonic()-finished_at > 3.0: break
        except KeyboardInterrupt:
            user_stopped=True

        if args.show:
            stop_spin.set(); executor.shutdown(timeout_sec=2.0)
            spin_thread.join(timeout=3.0)
            if spin_thread.is_alive(): raise RuntimeError('ROS executor did not stop')
            if visual.closed and args.mode != 'interactive':
                print('graph closed; simulation stopped')
                return
        if user_stopped and args.mode == 'interactive':
            print('interactive simulation stopped by user',flush=True)

        final_truth_stamp=max(monitor.truth_samples) if monitor.truth_samples else None
        recorded=monitor.trajectory.snapshot()
        plotted=visual.history.snapshot()
        final_sample_processed=final_truth_stamp in recorded
        plot_history_matches_metrics=plotted == recorded
        if not visual.closed:
            visual.draw(save=True)
        else:
            visual.fig.savefig(plot_file,dpi=120)

        final_goal=monitor.goal or goal
        estimated_length,gaps=monitor.trajectory.length_from_start()
        direct_distance=math.hypot(*final_goal)
        achieved_displacement=math.hypot(*monitor.pose) if monitor.pose else None
        result={'mode':args.mode,'path':args.path if args.mode == 'scripted' else None,
                'ros_domain_id':domain_id,'origin_geo':[args.origin_lat,args.origin_lon],
                'initial_heading_deg':args.initial_heading_deg,
                'speed_m_s':args.speed_m_s if args.mode == 'goal' else None,
                'turn_rate_deg_s':args.turn_rate_deg_s if args.mode == 'goal' else None,
                'simulation_dt_s':0.05,'max_duration_s':args.max_duration_s,
                'initial_goal_geo':goal_geo,'initial_goal_m':goal,
                'goal_geo':nearby_geodetic(args.origin_lat,args.origin_lon,*final_goal),
                'goal_m':final_goal,
                'completed':monitor.done,'completion_reason':sim.completion_reason or
                ('user stopped' if user_stopped else None),
                'final_sample_processed':final_sample_processed,
                'plot_history_matches_metrics':plot_history_matches_metrics,
                'plotted_trajectory_samples':len(plotted),
                'truth_m':monitor.truth,'estimated_m':monitor.pose,
                'destination_reached':monitor.arrival_seen,
                'last_recommendation':monitor.nav.recommendation if monitor.nav else None,
                'last_status':monitor.status.reason if monitor.status else None,
                'plot':str(plot_file),'velocity_samples':monitor.velocity_samples,
                'last_velocity_enu_m_s':monitor.last_velocity,
                'straight_line_distance_m':direct_distance,
                'estimated_path_length_m':estimated_length,
                'achieved_displacement_m':achieved_displacement,
                'unobserved_path_gaps':gaps,
                'path_over_achieved_displacement':(
                    estimated_length/achieved_displacement if args.mode == 'goal' and
                    achieved_displacement and gaps == 0 and estimator.tracker.metric_calibrated
                    else None),
                'path_length_ratio':(estimated_length/direct_distance if args.mode == 'goal' and
                                     monitor.arrival_seen and direct_distance > 0 and gaps == 0 and
                                     estimator.tracker.metric_calibrated else None),
                'first_draw_before_truth':first_draw_before_truth,
                'live_draws_before_completion':live_draws,
                'live_max_poses_before_completion':live_max_poses,
                'initial_frame':str(initial_frame) if initial_frame else None,
                'in_progress_frame':str(progress_frame) if progress_frame else None,
                'in_progress_frame_poses':live_capture_pose_count}
        if monitor.truth and monitor.pose:
            result['endpoint_error_m']=math.dist(monitor.truth,monitor.pose)
            result['goal_error_m']=math.dist(final_goal,monitor.pose)
        paired=monitor.truth_samples.keys() & recorded.keys()
        if paired:
            squared=[math.dist(monitor.truth_samples[k],recorded[k])**2 for k in paired]
            result['trajectory_rmse_m']=math.sqrt(sum(squared)/len(squared))
            result['trajectory_samples']=len(squared)
        (output/(run_name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        if (args.mode != 'interactive' and not monitor.done) or monitor.pose is None:
            raise RuntimeError('demo did not complete pipeline')
        if not final_sample_processed and not (args.mode == 'interactive' and user_stopped):
            raise RuntimeError('final simulated flow did not reach the estimator')
        if not plot_history_matches_metrics:
            raise RuntimeError('visualizer history differs from recorded estimated poses')
        if default_scripted and monitor.truth is not None and math.dist(monitor.truth,GOALS[args.path]) > 0.01:
            raise RuntimeError('final truth does not match scripted path goal')

        if args.show:
            hold_start=time.monotonic()
            while not visual.closed and (args.gui_hold_seconds is None or
                                         time.monotonic()-hold_start < args.gui_hold_seconds):
                plt.pause(0.05)
    finally:
        stop_spin.set()
        if spin_thread is not None and spin_thread.is_alive():
            executor.shutdown(timeout_sec=2.0)
            spin_thread.join(timeout=3.0)
        for node in reversed(nodes):
            executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.try_shutdown()
