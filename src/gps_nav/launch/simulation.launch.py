from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from gps_nav.geo import ENU, nearby_geodetic
from gps_nav.sim_core import GOALS
import math


def simulation_nodes(context, config):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('scripted','goal','interactive'):
        raise ValueError('mode must be scripted, goal, or interactive')
    path = LaunchConfiguration('path').perform(context)
    if path not in GOALS:
        raise ValueError(f'unknown simulation path: {path}')
    if mode != 'scripted' and path != 'straight':
        raise ValueError('a scripted path requires mode:=scripted')
    origin_lat = float(LaunchConfiguration('origin_lat').perform(context))
    origin_lon = float(LaunchConfiguration('origin_lon').perform(context))
    goal_lat_text = LaunchConfiguration('goal_lat').perform(context)
    goal_lon_text = LaunchConfiguration('goal_lon').perform(context)
    if (goal_lat_text == 'auto') != (goal_lon_text == 'auto'):
        raise ValueError('set both goal_lat and goal_lon, or leave both as auto')
    if goal_lat_text == 'auto':
        default_goal = GOALS[path] if mode == 'scripted' else (3.0,0.0)
        goal_lat, goal_lon = nearby_geodetic(origin_lat, origin_lon, *default_goal)
    else:
        goal_lat, goal_lon = float(goal_lat_text), float(goal_lon_text)
    goal_x, goal_y = ENU(origin_lat,origin_lon).to_local(goal_lat,goal_lon)
    heading_text = LaunchConfiguration('initial_heading_deg').perform(context)
    align_heading = False
    duration_text = LaunchConfiguration('max_duration_s').perform(context)
    max_duration = (0.0 if mode == 'interactive' else 30.0) if duration_text == 'auto' else float(duration_text)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true','1','yes')
    visual=Node(package='gps_nav',executable='visualizer',parameters=[{
        'headless':headless,'output_file':LaunchConfiguration('output_file'),
        'draw_period_s':2.0 if headless else 0.1,
        'save_on_draw':headless,'mode':mode}],
        sigterm_timeout='15')
    nodes=[
        Node(package='gps_nav',executable='waypoint',parameters=[{
            'origin_lat':origin_lat,'origin_lon':origin_lon,
            'goal_lat':goal_lat,'goal_lon':goal_lon}]),
        Node(package='gps_nav',executable='simulator',parameters=[config,{
            'mode':mode,'path':path,
            'goal_x_m':goal_x,'goal_y_m':goal_y,
            'speed_m_s':float(LaunchConfiguration('speed_m_s').perform(context)),
            'turn_rate_rad_s':math.radians(float(LaunchConfiguration('turn_rate_deg_s').perform(context))),
            'max_duration_s':max_duration,
            'cmd_vel_topic':LaunchConfiguration('cmd_vel_topic'),
            'align_initial_heading_to_goal':align_heading,
            'initial_heading_deg':float(heading_text)}]),
        Node(package='gps_nav',executable='preprocess',parameters=[config]),
        Node(package='gps_nav',executable='estimator',parameters=[config]),
        Node(package='gps_nav',executable='navigator'), visual]
    return nodes+[RegisterEventHandler(OnProcessExit(
        target_action=visual,on_exit=[EmitEvent(event=Shutdown(reason='visualizer closed'))]))]


def generate_launch_description():
    config=str(Path(get_package_share_directory('gps_nav'))/'config'/'simulation.yaml')
    args=[DeclareLaunchArgument('mode',default_value='goal'),
          DeclareLaunchArgument('path',default_value='straight'),
          DeclareLaunchArgument('origin_lat',default_value='0.0'),
          DeclareLaunchArgument('origin_lon',default_value='0.0'),
          DeclareLaunchArgument('goal_lat',default_value='auto'),
          DeclareLaunchArgument('goal_lon',default_value='auto'),
          DeclareLaunchArgument('speed_m_s',default_value='0.6'),
          DeclareLaunchArgument('turn_rate_deg_s',default_value='90.0'),
          DeclareLaunchArgument('initial_heading_deg',default_value='0.0'),
          DeclareLaunchArgument('max_duration_s',default_value='auto'),
          DeclareLaunchArgument('headless',default_value='true'),
          DeclareLaunchArgument('cmd_vel_topic',default_value='/simulation/cmd_vel'),
          DeclareLaunchArgument('output_file',default_value='trajectory.png')]
    return LaunchDescription(args+[OpaqueFunction(function=simulation_nodes,args=[config])])
