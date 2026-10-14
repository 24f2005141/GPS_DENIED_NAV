"""Full navigation-to-command graph with a nonphysical microcontroller endpoint."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('gps_nav'))
    names=['origin_lat','origin_lon','goal_lat','goal_lon',
           'initial_heading_deg','speed_m_s','turn_rate_deg_s','max_duration_s',
           'headless','output_file']
    defaults=['0.0','0.0','auto','auto','0.0','0.6','90.0',
              '0.0','true','robot_simulation.png']
    args=[DeclareLaunchArgument(name,default_value=value)
          for name,value in zip(names,defaults)]
    included=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch'/'simulation.launch.py')),
        launch_arguments={**{name:LaunchConfiguration(name) for name in names},
                          'mode':'interactive','path':'straight',
                          'cmd_vel_topic':'/simulation/robot_cmd_vel'}.items())
    config=str(share/'config'/'robot_simulation.yaml')
    return LaunchDescription(args+[included,
        Node(package='gps_nav',executable='robot_supervisor',parameters=[config]),
        Node(package='gps_nav',executable='robot_sim_endpoint',parameters=[config])])
