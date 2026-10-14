"""Real acquisition plus robot link; both physical movement gates default off."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('gps_nav'))
    names=['serial_port','byte_order','origin_lat','origin_lon','goal_lat','goal_lon']
    defaults=['/dev/ttyUSB0','little','0.0','0.0','0.0','0.0']
    args=[DeclareLaunchArgument(name,default_value=value)
          for name,value in zip(names,defaults)]
    args += [DeclareLaunchArgument('robot_serial_port',default_value='/dev/ttyACM0'),
             DeclareLaunchArgument('allow_physical_control',default_value='false'),
             DeclareLaunchArgument('motor_commands_enabled',default_value='false')]
    included=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch'/'real.launch.py')),
        launch_arguments={name:LaunchConfiguration(name) for name in names}.items())
    config=str(share/'config'/'robot_real.yaml')
    return LaunchDescription(args+[included,
        Node(package='gps_nav',executable='robot_supervisor',parameters=[config,{
            'allow_physical_control':LaunchConfiguration('allow_physical_control')}]),
        Node(package='gps_nav',executable='robot_transport',parameters=[config,{
            'serial_port':LaunchConfiguration('robot_serial_port'),
            'motor_commands_enabled':LaunchConfiguration('motor_commands_enabled')}])])
