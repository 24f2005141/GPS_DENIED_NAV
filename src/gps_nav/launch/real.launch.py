from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config=str(Path(get_package_share_directory('gps_nav'))/'config'/'real.yaml')
    args=[DeclareLaunchArgument('serial_port',default_value='/dev/ttyUSB0'),
          DeclareLaunchArgument('byte_order',default_value='little'),
          DeclareLaunchArgument('origin_lat',default_value='0.0'),
          DeclareLaunchArgument('origin_lon',default_value='0.0'),
          DeclareLaunchArgument('goal_lat',default_value='0.0'),
          DeclareLaunchArgument('goal_lon',default_value='0.0')]
    visual=Node(package='gps_nav',executable='visualizer',parameters=[config])
    nodes=[Node(package='gps_nav',executable='waypoint',parameters=[{
        'origin_lat':LaunchConfiguration('origin_lat'),'origin_lon':LaunchConfiguration('origin_lon'),
        'goal_lat':LaunchConfiguration('goal_lat'),'goal_lon':LaunchConfiguration('goal_lon')}]),
        Node(package='gps_nav',executable='hereflow',parameters=[config,{
            'serial_port':LaunchConfiguration('serial_port'),'byte_order':LaunchConfiguration('byte_order')}]),
        Node(package='gps_nav',executable='preprocess',parameters=[config]),
        Node(package='gps_nav',executable='estimator',parameters=[config]),
        Node(package='gps_nav',executable='navigator',parameters=[config]), visual]
    return LaunchDescription(args+nodes+[RegisterEventHandler(
        OnProcessExit(target_action=visual,on_exit=[EmitEvent(event=Shutdown(reason='visualizer closed'))]))])
