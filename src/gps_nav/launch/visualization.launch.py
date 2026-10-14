from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('output_file',default_value='trajectory.png'),
        Node(package='gps_nav',executable='visualizer',parameters=[{
            'headless':LaunchConfiguration('headless'),
            'output_file':LaunchConfiguration('output_file')}])])
