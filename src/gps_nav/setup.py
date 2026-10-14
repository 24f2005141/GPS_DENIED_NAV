from glob import glob
from setuptools import setup

package_name = 'gps_nav'
setup(
    name=package_name, version='0.2.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='GPS Nav', maintainer_email='maintainer@example.invalid',
    description='GPS-denied navigation assistance prototype', license='Apache-2.0',
    entry_points={'console_scripts': [
        'waypoint = gps_nav.waypoint_node:main',
        'hereflow = gps_nav.hereflow_node:main',
        'hereflow_diagnostic = gps_nav.hereflow_diagnostic:main',
        'preprocess = gps_nav.preprocess_node:main',
        'estimator = gps_nav.estimator_node:main',
        'navigator = gps_nav.navigator_node:main',
        'simulator = gps_nav.simulator_node:main',
        'visualizer = gps_nav.visualizer_node:main',
        'terminal_waypoint = gps_nav.terminal_waypoint:main',
        'demo = gps_nav.demo:main',
        'robot_supervisor = gps_nav.robot_supervisor_node:main',
        'robot_transport = gps_nav.robot_transport_node:main',
        'robot_sim_endpoint = gps_nav.robot_sim_endpoint_node:main',
        'robot_serial_diagnostic = gps_nav.robot_serial_diagnostic:main',
    ]},
)
