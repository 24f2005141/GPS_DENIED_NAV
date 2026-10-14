# GPS-Denied Navigation Assistance Module

A ROS 2 Jazzy prototype that accepts a user supplied geographic origin and destination, converts the destination to local ENU metres, estimates **relative** 2D motion from calibrated optical flow and IMU yaw, publishes direction guidance, and plots the estimated path. A separate, default-disabled command supervisor and USB serial transport can request bounded motion from a microcontroller; the Raspberry Pi never drives motor pins or generates PWM. Simulation uses the same acquisition interface and downstream nodes as the serial mode.

## Status and scope

The simulation pipeline builds and runs in this workspace. Real HereFlow UART decoding is implemented but the device was absent during this validation. Real metric localization is deliberately disabled in `config/real.yaml`: raw flow counts do not establish metres without a measured flow model, ground distance or equivalent calibration, axis checks, and a valid IMU orientation source. The IMU model/interface is unknown, so no physical IMU driver is claimed. This package does not infer an exact global position after the start point is entered.

## Architecture

```mermaid
flowchart LR
  GPS[Origin and destination input] --> W[Waypoint node: WGS84 to ENU]
  W --> G[/navigation_goal]
  SIM[Simulator] --> RAW[/optical_flow/raw]
  UART[HereFlow UART] --> RAW
  SIM --> IMU[/imu/data]
  EXT[Future verified IMU driver] --> IMU
  RAW --> PRE[Flow preprocessing and calibration gate] --> FLOW[/optical_flow]
  FLOW --> EST[Relative estimator]
  IMU --> EST
  EST --> P[/estimated_position and path]
  P --> NAV[Navigator]
  G --> NAV
  NAV --> N[/navigation_status]
  P --> VIZ[Trajectory plot]
  N --> VIZ
```

The acquisition sources are mutually exclusive by launch file. The simulator publishes ground truth only to `/simulation/ground_truth`, which the estimator never subscribes to. The `demo` command uses that topic solely to measure error.

## Hardware and software

Tested environment: Ubuntu 24.04 under WSL 2, ROS 2 Jazzy from RoboStack/micromamba, Python 3.12, `colcon`, `rclpy`, Matplotlib. Standard ROS packages used: `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_srvs`, `rosidl_default_generators`. The UART reader uses Python's `os`, `termios`, and `select`, so `pyserial` is unnecessary. USB device `/dev/ttyUSB0` was absent during this run; an earlier reported Windows COM10 association is not a current detection. A physical IMU producing valid `sensor_msgs/Imu` yaw on `/imu/data` is required for real mode.

## Workspace layout

- `src/gps_nav_interfaces`: timestamped optical flow and status messages; waypoint services.
- `src/gps_nav/gps_nav`: geometry, parser, serial I/O, estimator, simulator, ROS nodes, plotting, and finite demo.
- `src/gps_nav/launch`: simulation, real, and visualization launch files.
- `src/gps_nav/config`: simulated and uncalibrated real configuration.
- `src/gps_nav/test`: pure logic and pseudo-terminal UART tests.
- `docs`: architecture, frames, protocol, calibration, test report, and WSL setup.
- `reports`: captured JSON, console output, and headless plot files from tested demos.
- `firmware/arduino_protocol_only`: board-independent protocol sketch with a no-motor backend; real motor pins and driver code await confirmed hardware.

## Robot command integration

The new control path is `/navigation_status` plus `/localization_status` → `robot_supervisor` → `/robot/command` → either `robot_sim_endpoint` or `robot_transport` → `/robot/ack` and `/robot/link_status` → supervisor. `/robot/control_status` reports enabled, fault, arrival, link, and last acknowledged command; the visualizer displays these separately from navigation. The supervisor demands fresh metric localization, heading, flow, IMU, navigation guidance, and link acknowledgements. Arrival latches STOP. A link or sensor fault latches STOP until `/robot/reset_fault` succeeds and `/robot/set_enabled` is deliberately called again. `GO BACK` requests STOP by default because reversing is disabled. Speed is a bounded per-mille request, not PWM. The microcontroller must independently stop on startup, malformed input, timeout, and reset. See [protocol](docs/robot_protocol.md), [deployment](docs/raspberry_pi_deployment.md), and [firmware](firmware/README.md).

For a headless closed-loop robot simulation with geographic coordinates, where simulated MCU commands actually drive the interactive plant:

```bash
source install/setup.bash
ros2 launch gps_nav robot_simulation.launch.py origin_lat:=13.0827 origin_lon:=80.2707 \
  goal_lat:=13.0827 goal_lon:=80.2707277 headless:=true
# Another sourced terminal in the same ROS_DOMAIN_ID:
ros2 service call /robot/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /robot/control_status
```

The robot starts stopped and requires that service call even in simulation. For an automated closed-loop startup, fault, recovery, and arrival check with no display or USB device:

```bash
python scripts/run_robot_control_validation.py --output-dir reports/robot_control_validation
python -m pytest -q src/gps_nav/test/test_robot_protocol.py \
  src/gps_nav/test/test_robot_supervisor.py src/gps_nav/test/test_robot_serial.py \
  src/gps_nav/test/test_robot_serial_diagnostic.py
```

`robot_real.launch.py` launches the real sensor graph, supervisor, and USB transport with **two default-off movement gates** (`allow_physical_control` and `motor_commands_enabled`). It has no physical IMU driver and the real optical-flow calibration is invalid, so it cannot be armed in its present configuration. The protocol-only firmware has no motor backend and cannot move a robot. Physical commissioning requires verified sensor calibration and hardware-specific motor implementation; do not change those gates before the restrained-wheel STOP/watchdog tests in the deployment guide.

The MCU serial diagnostic sends only STOP and never launches navigation:

```bash
ros2 run gps_nav robot_serial_diagnostic --serial-port /dev/ttyACM0 \
  --duration 5 --output reports/robot_serial_diagnostic.json
```

It reports port access, STOP acknowledgements, invalid ACK frames, and reconnect counts. Exit 0 requires an accepted STOP acknowledgement; exit 2 means none was received. Use the discovered MCU port, not the HereFlow port.

## Build and run

From this directory:

```bash
micromamba activate ros2-jazzy
colcon build
source install/setup.bash
python -m pytest -q src/gps_nav/test
ros2 run gps_nav demo --mode scripted --path straight --output-dir reports/quick
ros2 run gps_nav demo --mode scripted --path turn --output-dir reports/turn_check
ros2 run gps_nav demo --mode scripted --path multi --output-dir reports/multi_check
# Prompt for current GPS coordinate, destination GPS coordinate, and initial heading:
ros2 run gps_nav demo --mode goal --show --output-dir reports/prompted_goal
# Geographic CLI input, approximately 2.987 m north of the start:
ros2 run gps_nav demo --mode goal --start-lat 13.0827 --start-lon 80.2707 \
  --destination-lat 13.082727 --destination-lon 80.2707 \
  --initial-heading-deg 90 --show --output-dir reports/geographic_goal
ros2 run gps_nav demo --mode goal --origin-lat 37.0 --origin-lon -122.0 \
  --goal-x-m 3.0 --goal-y-m 0.0 --speed-m-s 0.6 \
  --show --output-dir reports/goal_live
ros2 run gps_nav demo --mode scripted --path multi --show \
  --output-dir reports/multi_live
ros2 run gps_nav demo --mode scripted --path multi \
  --output-dir reports/multi_headless
ros2 run gps_nav demo --mode interactive --goal-x-m 3 --goal-y-m 0 \
  --domain-id 42 --show --output-dir reports/interactive_live
```

`--mode goal` steers the simulated platform toward the requested waypoint. If geographic coordinates are omitted, the CLI prompts for the current latitude/longitude and destination latitude/longitude, retries invalid entries, and asks for an initial heading; Enter accepts 0° East. `--start-lat/--start-lon` and `--destination-lat/--destination-lon` provide a noninteractive geographic path. The older `--origin-lat/--origin-lon` and `--goal-lat/--goal-lon` names remain aliases. `--goal-x-m/--goal-y-m` specify a local ENU offset and keep the existing noninteractive local-goal workflow; if geographic start coordinates are omitted with a local goal, the reference defaults to (0°, 0°). A destination within 0.01 m of the start is rejected before movement. In an obstacle-free scene goal mode turns in place from the specified heading, then moves incrementally along the straight-line baseline. Set `--speed-m-s`, `--turn-rate-deg-s`, `--max-duration-s`, rates, noise, and dropout intervals as needed. Without an explicit duration, the finite timeout scales with requested distance and speed. The goal simulator stops only after the production navigator confirms arrival or the duration expires. It does not implement obstacle avoidance.

`--mode scripted` retains the straight, 90-degree, and intentionally rectangular multi-turn paths for estimator validation. The scripted multi path is deliberately not a shortest route. `--mode interactive` runs until its graph closes, Ctrl-C is pressed, or a configured `--max-duration-s` expires; it accepts `/simulation/cmd_vel` and destination service updates. Goal is the default mode, but every example specifies its mode. Passing `--path multi` without `--mode scripted` is an error. All three modes use the same estimator, waypoint node, navigator, health checks, and visualizer. The `--show` graph opens **before movement** and updates at 10 Hz during the run; it stays open after finite completion until closed. Matplotlib runs on the main thread while a single ROS executor thread processes callbacks. The final JSON and PNG come from that same run. Use `--capture-live-frames` to save initial and in-progress GUI frames for diagnostics. Headless runs require no display and render only the final PNG.

The demo chooses an isolated ROS domain for each run to prevent another graph from publishing into its topics. A separately launched visualizer on the default ROS domain cannot watch that isolated demo. Use `--domain-id N` to choose a domain explicitly. A live GUI requires a working WSLg/X display; the automated suite remains headless. Interactive ROS graph:

```bash
ros2 launch gps_nav simulation.launch.py mode:=scripted path:=straight headless:=true output_file:=trajectory.png
# For a live multi-turn window and PNG from the same launch graph:
ros2 launch gps_nav simulation.launch.py mode:=scripted path:=multi headless:=false output_file:=multi_live.png
# Goal-directed launch with explicit geographic origin and destination:
ros2 launch gps_nav simulation.launch.py mode:=goal origin_lat:=37.0 origin_lon:=-122.0 \
  goal_lat:=37.0 goal_lon:=-121.99996629655 headless:=false output_file:=goal_live.png
# In another sourced terminal:
ros2 topic echo /navigation_status
ros2 topic echo /localization_status
ros2 topic echo /estimated_position
```

The launch graph stays alive after a finite path ends so status and plot can be inspected; stop with Ctrl-C. Its default geographic goal follows the selected scripted path: straight and multi end 3 m East, while turn ends 1 m East and 1 m North. Goal mode defaults to 3 m East if no destination is supplied. Set both `goal_lat` and `goal_lon` to override either default. Use a separate ROS domain when running multiple launch graphs at once. The launch graph saves a PNG; use the `demo` CLI for same-run JSON evaluation metrics. The effective mode, start, goal, speed, turn rate, and time step are printed at startup and shown in the graph title/status.

The visualizer builds its ordered route from accepted `/estimated_position` messages. `/estimated_path` snapshots can backfill missing earlier poses, but an old snapshot cannot erase newer points. Both the live window and saved PNG use this route and the same drawing function. The finite demo verifies that its recorder and renderer have identical estimated-pose histories before writing a successful result. Ground truth is used only for simulation metrics.

For continuous interactive control, start the demo above with `--mode interactive --domain-id 42 --show`, then use another sourced terminal. Motion commands persist until replaced, and changing the destination updates guidance without resetting the estimated path. `/simulation/cmd_vel` affects only the simulator:

```bash
export ROS_DOMAIN_ID=42
ros2 topic pub --once /simulation/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.6}, angular: {z: 0.0}}'
# Publish zero speed to stop after a chosen interval.
ros2 topic pub --once /simulation/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
ros2 service call /set_destination gps_nav_interfaces/srv/SetDestination \
  '{latitude: 0.000027, longitude: 0.0}'
```

The interactive demo uses an isolated ROS domain only when `--domain-id` is omitted; the explicit domain ID above makes external commands reach that run. For an all-launch interactive graph use `ros2 launch gps_nav simulation.launch.py mode:=interactive headless:=false`; in that case both terminals must use the same `ROS_DOMAIN_ID` (the default is 0). Set `max_duration_s:=N` for an automatic stop after N simulated seconds. Headless interactive `demo` requires `--max-duration-s` to avoid an unbounded CI run.

Real serial acquisition (no simulator is started):

```bash
ros2 launch gps_nav real.launch.py serial_port:=/dev/ttyUSB0 \
  origin_lat:=37.0 origin_lon:=-122.0 goal_lat:=37.00001 goal_lon:=-121.99999
ros2 topic echo /optical_flow/raw
ros2 topic echo /sensor_status/hereflow
```

Inspect UART hardware without navigation first:

```bash
ros2 run gps_nav hereflow_diagnostic --serial-port /dev/ttyUSB0 \
  --duration 15 --capture reports/hardware/hereflow_raw.jsonl \
  --output reports/hardware/hereflow_diagnostic.json
```

The diagnostic reports port and packet health using raw counts only. Exit 0 means at least one frame matched the configured parser, exit 2 means the port never opened, and exit 3 means it opened without a valid frame. Run it separately from `real.launch.py`; see `docs/hardware_setup_wsl.md` for CP2102 attachment and capture steps.

With default real calibration, `/optical_flow` carries raw counts and `metric_valid=false`; the estimator refuses metre-valued updates. Real `heading_reference_valid` also defaults to false. Supply a verified `/imu/data` source, establish its yaw offset to ENU East, and measure flow calibration before enabling those gates in `config/real.yaml`. Rebuild after editing that configuration, then relaunch. Consult `docs/calibration.md` and `docs/hardware_setup_wsl.md` first.

Terminal coordinate entry:

```bash
ros2 run gps_nav terminal_waypoint
ros2 run gps_nav terminal_waypoint --destination-only
```

The first command resets the origin and estimator through `/set_session`; the second updates only the destination through `/set_destination`. A noninteractive service example:

```bash
ros2 service call /set_session gps_nav_interfaces/srv/SetSession \
  '{origin_lat: 37.0, origin_lon: -122.0, goal_lat: 37.00001, goal_lon: -121.99999}'
```

Latitude must lie in [-90,90], longitude in [-180,180]; nonfinite values are rejected. The origin remains fixed until `/set_session` is called.

## Frames, units, and method

`map_enu`: East +X, North +Y, Up +Z. `base_link`: forward +X, left +Y. Yaw 0 faces East; positive yaw turns toward North. Raw UART optical flow remains signed **counts** with an integer quality field. The preprocessing node converts counts to sensor-frame displacement **only when calibration is explicitly marked valid**. The estimator rotates sensor displacement by mounting yaw, then by IMU yaw into ENU, and accumulates it. It rejects low quality, duplicate, uncalibrated, and stale data. Navigation uses `atan2(goal_y-y, goal_x-x)` and wrapped bearing minus heading. `GO BACK` is navigation guidance; the robot supervisor requests STOP by default and only maps it to BACKWARD when reverse motion is explicitly enabled.

The simulation uses a deliberately known 0.01 m/count model; that constant is **not** a real sensor calibration. A physical PMW3901 optical-flow module lacks inherent metric displacement: angular/pixel flow normally needs ground distance and rotation compensation. Simulation can publish ENU velocity on `/estimated_velocity` by dividing accepted displacement by timestamp interval. Real velocity is disabled because the serial packet's integration interval has not been verified. Pose covariance is marked large/unknown; simulation endpoint errors are model tests, not accuracy claims for hardware.

In goal mode, JSON reports estimated path length from accepted poses, the straight-line origin-to-goal distance, and their ratio only for a nonzero reference distance and a valid arrival without unobserved gaps. The ratio can be below 1 because arrival is allowed within 0.25 m of the goal; it is not an optimality score. `path_over_achieved_displacement` compares traveled length with the distance actually reached. Endpoint error compares the estimate with separate simulation truth, while `goal_error_m` is estimated distance to the requested destination. Neither metric proves physical accuracy or collision freedom.

## Configuration

`config/real.yaml` contains serial port, baud, byte order, reconnection and sensor-stale intervals, calibration gate, scale factors, axis inversions, mounting yaw, heading reference gate/offset, yaw variance limit, quality limit, estimator stale interval, and navigation thresholds. `config/simulation.yaml` contains seed, rates, model scale, noise, bias, dropouts, and startup delay. ROS parameters can be inspected with `ros2 param list` and `ros2 param get`. Most parameters are read at startup; restart launch after editing configuration. No filter smoothing or deadband is applied by default (`alpha=1`, deadband=0); nonunity smoothing disables metric output because filtering displacement increments can bias dead reckoning.

## Tests and results

See `docs/testing_and_validation.md` and `reports/`. The user-coordinate build succeeded and **60 automated tests passed**. All nine original scripted scenarios plus the prolonged-IMU fault scenario completed; four goal fault cases and four geographic goal cases completed. The prompted offscreen [live goal report](reports/user_coords_final_live/goal.json) used geographic start `(13.0827,80.2707)` and destination `(13.082727,80.2707)`, drew [13 estimated poses before completion](reports/user_coords_final_live/goal_in_progress.png), and saved the [94-pose final path](reports/user_coords_final_live/goal.png). It converted the destination to `(0,2.987)` m ENU, confirmed arrival after 2.82 m estimated travel, and recorded 0 m endpoint error against separate simulation truth. Errors are simulation-only comparisons. The ROS sandbox can log a UDP transport permission warning; local shared-memory message flow still ran. Hardware calibration, physical GUI display, and physical accuracy remain unverified.

## Troubleshooting

- No serial port: reattach USB with `usbipd`, then inspect `/dev/ttyUSB*` and permissions. The driver logs periodic reconnect attempts and does not crash other nodes.
- Flow packets but no motion: inspect `/optical_flow/raw`, calibration gate, `/imu/data`, and `/localization_status`. This is the expected state before real calibration.
- Wrong signs or turns: verify sensor mounting, byte order, axis inversions, and initial heading before changing gains.
- `POSITION UNRELIABLE`: check timestamps, quality, and freshness. The simulator stops publishing when its finite path ends, so the final status becomes stale after the recorded arrival event.
- Build issue with this micromamba environment: use ordinary `colcon build`; its current setuptools did not accept `--symlink-install` on this project.
- No display: use `headless:=true` and inspect the saved PNG. A live GUI requires a working WSLg or X server and `headless:=false`.

## Future work

Identify the physical IMU, document its orientation convention, validate the actual UART packet order and scale against captured bytes, add a distance sensor or controlled height measurement, measure independent ground truth, and then evaluate calibrated physical performance. An EKF is intentionally deferred until its measurement model and noise data exist.
