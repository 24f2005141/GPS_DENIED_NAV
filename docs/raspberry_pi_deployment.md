# Raspberry Pi deployment and commissioning

No Raspberry Pi or mobile robot was available in the development environment. These are instructions, not a deployment result. First record `uname -m`, `cat /etc/os-release`, `python3 --version`, `ros2 doctor --report`, `colcon --version`, and installed ROS distribution **on the Pi**. The [ROS 2 Jazzy Ubuntu package guide](https://repo.test.ros2.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) lists Ubuntu 24.04 for amd64 and arm64; verify the actual Pi OS and architecture before installation. Do not copy the WSL micromamba paths or assume a GUI.

## Raspberry Pi 4 storage decision

The intended Pi has 4 GB RAM and an 8 GB SD card. [Ubuntu's current Raspberry Pi download page](https://ubuntu.com/download/raspberry-pi) specifies **16 GB minimum storage for its listed Desktop image**; it does not establish that an Ubuntu Server 24.04 plus ROS 2 Jazzy native build fits on 8 GB. This project has **not** been shown to fit or build on the 8 GB card. Use at least a 16 GB card, with 32 GB recommended for ROS packages, native build artifacts, updates, and logs. This recommendation is a margin for development, not a measured minimum for this project. Prefer Ubuntu Server and `ros-jazzy-ros-base` over a desktop installation for a headless demonstration. Before and after installing dependencies and building, record `df -h /`, `df -i /`, and `du -sh build install log`; stop if free space is inadequate. Generated WSL `build/`, `install/`, and `log/` directories must never be copied to ARM64. Keep report retention limited on the Pi.

## Install and build on the Pi

Once the repository has been pushed and its URL verified, on a confirmed Ubuntu 24.04 arm64 installation follow the [ROS 2 Jazzy Debian package installation guide](https://repo.test.ros2.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to configure the ROS repository. Install the headless ROS base and build tools, then resolve workspace dependencies with `rosdep`:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools python3-matplotlib python3-pytest
source /opt/ros/jazzy/setup.bash
git clone https://github.com/24f2005141/GPS_DENIED_NAV.git ~/gps_denied_navigation_v2
cd ~/gps_denied_navigation_v2
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
source install/setup.bash
python3 -m pytest -q src/gps_nav/test
df -h /
```

Run `sudo rosdep init` once first if `rosdep` has never been initialized on that Pi; skip it if already initialized. The package manifests declare ROS interfaces and Matplotlib. Keep `src/gps_nav/config/simulation.yaml`, `src/gps_nav/config/real.yaml`, `src/gps_nav/config/robot_simulation.yaml`, and `src/gps_nav/config/robot_real.yaml` separate. Native build results and storage use have **not** been measured on the target Pi.

Run the command-path simulation before connecting motors:

```bash
source install/setup.bash
python3 scripts/run_robot_control_validation.py --output-dir reports/pi_robot_sim_check
ros2 launch gps_nav robot_simulation.launch.py headless:=true
```

The validation script tests startup STOP, motion only after explicit enable, injected link loss, latched STOP, manual reset and reenable, and arrival STOP through the same ROS topics as the real transport. A headless Pi may omit desktop packages and use the visualizer's headless PNG mode; a live graph requires an available display and GUI backend.

## Device discovery and nonmoving diagnostics

With no navigation graph running, identify HereFlow and MCU separately:

```bash
ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM*
udevadm info --query=property --name=/dev/ttyUSB0
id -nG
ros2 run gps_nav hereflow_diagnostic --serial-port /dev/ttyUSB0 --duration 15 \
  --output reports/pi_hereflow_diagnostic.json
ros2 run gps_nav robot_serial_diagnostic --serial-port /dev/ttyACM0 --duration 5 \
  --output reports/pi_robot_serial_diagnostic.json
```

Choose stable `/dev/serial/by-id/...` names where available. If opening a device is denied, inspect its group and add the deployment account to that group according to the Pi OS policy, then start a new login session. Avoid running the application as root. Keep HereFlow and MCU on different configured serial paths; only one process may open each. The HereFlow diagnostic only reports raw packet validity and quality. It does not validate metre scale or physical localization. The robot protocol tests use a pseudo-terminal and a simulated MCU; no motors are involved. There is no identified physical IMU driver yet.

## Real graph and physical enable sequence

The default real launch is deliberately nonmoving:

```bash
ros2 launch gps_nav robot_real.launch.py \
  serial_port:=/dev/serial/by-id/HEREFLOW_ID \
  robot_serial_port:=/dev/serial/by-id/MCU_ID \
  origin_lat:=LAT origin_lon:=LON goal_lat:=LAT goal_lon:=LON
ros2 topic echo /localization_status
ros2 topic echo /robot/control_status
ros2 topic echo /robot/link_status
ros2 service call /robot/request_stop std_srvs/srv/Trigger '{}'
```

Alternatively, while the graph is running, `ros2 run gps_nav terminal_waypoint` prompts for the geographic origin and destination through the existing session service. `ros2 run gps_nav terminal_waypoint --destination-only` changes the destination without redefining the origin. Entering a waypoint does not enable physical control.

Replace uppercase placeholders with **measured** values. The launch keeps `allow_physical_control=false` and `motor_commands_enabled=false`. The real sensor config keeps metric calibration and heading reference invalid, so the supervisor cannot arm. Once hardware, sensor scale, IMU orientation, mounting axes, motor backend, and independent reference are verified, use a reviewed site-specific config and only then change both gates. A positive `/robot/set_enabled` service request is still required each session; startup or reconnect never resumes movement automatically. A fault requires correcting its cause, `/robot/reset_fault`, and a new `/robot/set_enabled` request. `/robot/request_stop` is a software stop request, not a physical emergency stop.

Commission with the drive restrained: first validate power-off output behavior, physical emergency stop, startup STOP, command CRC/ACK, parser rejection, USB unplug, host kill, MCU reset, and watchdog stopping both motors. Log raw sensor data, measured calibration conditions, command/ACK history, and an independent position/heading reference. Verify low-speed direction signs before wheels touch ground. No ground-motion or physical accuracy result is claimed here.

## Headless service and monitoring

For unattended startup, create a site-owned systemd service with `WorkingDirectory` set to the copied workspace and `ExecStart` pointing to a wrapper that sources the verified ROS and workspace setup scripts then launches `robot_real.launch.py`. Set `Restart=on-failure`; never put a movement-enable service call in the startup script. Use `systemctl status`, `journalctl -u <service> -f`, `/robot/control_status`, and `/robot/link_status` for diagnosis. Stop the service and verify the MCU watchdog/physical stop before unplugging sensors. Measure Pi CPU and memory with `ps`, `top`, and `/usr/bin/time -v` during an actual Pi run; this development environment cannot provide Pi resource measurements.

## Hardware details still needed

Exact Pi model/OS/architecture and display availability; HereFlow packet capture and scale versus measured travel and height; IMU model, bus, units, frame and yaw reference; MCU board and USB identity; motor-driver model, supply and pin mapping; independent emergency-stop circuit; independent trajectory reference. Do not implement pin-specific firmware or mark real metric localization valid until these are known and tested.
