# Testing and validation report — 2026-10-13

## Robot-control upgrade — 2026-10-14

This upgrade was developed on Ubuntu 24.04 WSL2 with ROS 2 Jazzy. `/dev/ttyUSB0` and `/dev/ttyACM0` were absent. No Raspberry Pi, microcontroller board, motor driver, HereFlow stream, or physical IMU was connected. The firmware sketch was not compiled for or flashed to a board because no board toolchain or motor wiring was identified. It was compiled with a host Arduino API stub and run as a protocol test. The figures below are simulation, host, or pseudo-terminal results only.

The new V1 protocol tests cover frame CRC, parsing, sequence wrap and replay, STOP priority, ARM/RESET, speed bounds, rejected ACK freshness, startup stop, watchdog expiry, malformed input, fault recovery, and pseudo-terminal disconnect/reconnect. The supervisor tests cover explicit enable, metric and heading gates, stale input, link and ACK faults, arrival STOP, reverse permission, and destination changes. A STOP-only diagnostic was run against the absent `/dev/ttyACM0`; it exited 2 with `exists=false`, `open_errors=1`, zero acknowledgements, and `movement_commands_sent=0` ([captured JSON](../reports/robot_serial_absent.json)). This is a missing-device result, not a physical MCU test.

The host firmware check `g++ -std=c++17 -Wall -Wextra -Werror -I firmware/tests firmware/tests/protocol_host_test.cpp -o /tmp/gps_nav_protocol_host_test && /tmp/gps_nav_protocol_host_test` exited **0**. It compiles the actual `.ino` parser against a minimal test stub and exercises STOP, ARM, no-motor rejection, old STOP priority, RESET, watchdog expiry, and CRC rejection. It does not validate ESP32/Arduino APIs, GPIO, serial timing, or motors.

Existing scripted validation was rerun with `python scripts/run_validation.py --output-dir reports/robot_upgrade_scripted_validation`: **10/10 scenario processes exited 0** ([summary](../reports/robot_upgrade_scripted_validation/summary.json)). Straight, turn, multi-turn, noisy-flow, and mixed-rate paths reached the destination. Flow dropout, brief and prolonged IMU dropout, yaw bias, and low-quality flow did not. The multi-turn endpoint error was `4.0245584642661925e-16` m against separate simulation truth. Goal validation `python scripts/run_goal_validation.py --output-dir reports/robot_upgrade_goal_validation` exited 0 for **4/4** scenarios: clean and noisy reached; flow dropout and low-quality flow did not ([summary](../reports/robot_upgrade_goal_validation/summary.json)). Clean goal-mode estimated path length was about 2.82 m against a 3 m goal, ratio 0.9400000002839392. It stopped within the existing 0.25 m arrival tolerance; this is not a claim of shortest-path optimality. Geographic validation `python scripts/run_geographic_goal_validation.py --output-dir reports/robot_upgrade_geographic_validation` exited 0 for **4/4** start, goal, and heading cases, all arriving with final simulated endpoint error 0 ([summary](../reports/robot_upgrade_geographic_validation/summary.json)).

The integrated robot simulation uses the production sensor, estimator and navigator nodes, the new supervisor, and a simulated MCU that sends bounded `Twist` to the interactive plant on the robot-specific motion topic. A trial run succeeded with startup STOP, a 0.60 m forward move, injected link loss, stationary truth during the fault, explicit RESET and reenable, and arrival STOP. The saved headless PNG initially lagged arrival because the visualizer only saved periodically. The visualizer now saves a final frame on shutdown. Another run concurrent with a validation runner safely latched a stale-localization fault and withheld arrival; this shows the 0.5 s freshness gate can trip under overloaded scheduling. A separate trial failed its pre-enable stationary assertion; its cause was not captured. The robot simulation subsequently moved to a separate `/simulation/robot_cmd_vel` topic to avoid accepting ordinary interactive-demo commands. These trials are not counted as final success.

Final source build: `ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav` exited 0 with **2 packages finished**. The subsequent launch-only shutdown timeout change was rebuilt with `ros2-env colcon build --base-paths src --packages-select gps_nav` (1 package finished). The final test command, run again after that rebuild, `ros2-env colcon test --packages-select gps_nav_interfaces gps_nav --python-testing pytest && ros2-env colcon test-result --verbose` exited 0 with **89 tests, 0 errors, 0 failures, 0 skipped**.

Final command-path run: `python scripts/run_robot_control_validation.py --output-dir reports/robot_control_validation_final` exited 0. The [captured summary](../reports/robot_control_validation_final/summary.json) reports stationary start `(0,0)`, 0.60 m forward motion before link loss, no further motion during the disconnected hold, two ARM commands with a RESET between them, 48 FORWARD commands, 38 STOP commands, 81 accepted ACKs, zero rejected ACKs, and arrival latched with final command STOP. Final truth and estimated position were both approximately `(2.88,0.0)` m; the remaining 0.12 m was inside the unchanged 0.25 m arrival tolerance. The [final PNG](../reports/robot_control_validation_final/trajectory.png) was visually inspected: it shows the complete estimated straight trajectory, goal, current marker, robot link state, STOP acknowledgement, and persistent arrival. The [launch log](../reports/robot_control_validation_final/launch_stdout.txt) contains no traceback, process death, or timeout after the visualizer shutdown grace period was extended. DDS still emitted sandbox UDP permission warnings; local message delivery completed. This is deterministic software simulation, not hardware validation. GUI window presentation was not tested on an actual Raspberry Pi display.

The real graph was smoke tested with both serial paths deliberately set to `/dev/does-not-exist` and default-off movement gates. Eight nodes started (including supervisor and transport), **no simulator**, and a single SIGINT sent to the launcher ended all eight cleanly; launcher exit code was 0 ([captured log](../reports/robot_real_missing_ports_smoke_clean.txt)). This checks missing-port startup and shutdown only. A separate `timeout -s INT` attempt signalled the whole process group and produced multiple interrupt tracebacks; its [log](../reports/robot_real_missing_ports_smoke.txt) is retained but does not indicate an acquisition or motor-control failure. No physical serial communication occurred.

## Environment and scope

The run used Ubuntu 24.04.5 on WSL 2, Python 3.12.3, ROS 2 Jazzy in the local `ros2-jazzy` micromamba environment, and `colcon`. No `/dev/ttyUSB*` or `/dev/ttyACM*` device existed when checked. There was no identified physical IMU or independent physical position reference. All metre-valued error results below compare the estimator with the simulator's separate ground-truth topic. They do **not** measure hardware accuracy.

The ROS sandbox logged a UDP transport permission warning in some runs. Local ROS message delivery worked. GUI display through WSLg was not tested; headless Matplotlib generated PNGs.

## Build and automated tests

Commands run from the workspace root:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
ros2-env colcon test --python-testing pytest
ros2-env colcon test-result --verbose
```

Captured build result: `Summary: 2 packages finished [35.1s]` on the final rebuild. Captured pytest result: `29 passed in 0.84s`; `colcon test-result` reported `29 tests, 0 errors, 0 failures, 0 skipped`. The tests cover ENU signs and longitude wrap, invalid geographic input, parser framing/checksums/signed values/recovery, pseudo-terminal serial EOF and reconnect, estimator rotation and quality gates, guidance, waypoint services, IMU orientation covariance, deterministic paths, and manual simulation stepping. The test command used the explicit pytest runner because this environment's default `colcon test` runner discovered zero tests.

## End-to-end simulation

Command:

```bash
source install/setup.bash
python scripts/run_validation.py > reports/validation_runner_final.txt
```

The script ran nine finite ROS graphs sequentially. Each graph used the simulator's `/optical_flow/raw` and `/imu/data`, the production preprocessing, estimator, waypoint and navigator nodes, and the visualizer. The ground truth topic was consumed only by the demo monitor. Endpoint error is Euclidean final truth-to-estimate distance. Trajectory RMSE pairs truth and estimate messages with identical simulated-sample ROS timestamps; its sample count varies when measurements are rejected. Arrival means the navigator emitted `destination_reached=true` at least once while data was fresh.

| Scenario | Endpoint error (m) | Trajectory RMSE (m) | Paired samples | Arrival observed |
| --- | ---: | ---: | ---: | --- |
| Straight | 0 | 0 | 60 | yes |
| 90-degree turn | <1e-12 | <1e-12 | 60 | yes |
| Multi-turn | <1e-12 | <1e-12 | 140 | yes |
| Straight with 0.005 m flow noise | 0.06708 | 0.04610 | 60 | yes |
| Flow dropout | 0.45000 | 0.40348 | 51 | no |
| IMU dropout | 0.15000 | 0.11754 | 57 | no |
| 0.1 rad/s yaw bias | 0.45634 | 0.20930 | 60 | no |
| Low-quality flow interval | 0.45000 | 0.40348 | 51 | no |
| Flow 10 Hz / IMU 20 Hz | 0.00393 | 0.00323 | 31 | yes |

All nine demo processes exited 0. Full-precision metrics, path names, final position, velocity sample counts, and generated plot paths are in [`reports/validation/summary.json`](../reports/validation/summary.json); stdout and stderr for each scenario are in adjacent scenario directories. Fixed seed 42 was used unless overridden. Noise and bias values are simulation inputs, not estimated physical noise. Dropouts intentionally lose unobserved displacement; the estimator does not fill it with invented motion. Once a finite path stops publishing, health status becomes stale and the latest recommendation may become `SENSOR DEGRADED` even if arrival was correctly emitted earlier.

## Launch and fault checks

`ros2 launch gps_nav simulation.launch.py --show-args` and `ros2 launch gps_nav real.launch.py --show-args` both exited 0 after the launch shutdown event fix. A separate smoke harness started each launch for six seconds and sent one SIGINT to the launch process. The simulation and real launches each started six processes and each had six `finished cleanly` messages, exit code 0, and zero tracebacks. Captures: [`simulation_launch_final.txt`](../reports/simulation_launch_final.txt) and [`real_launch_final.txt`](../reports/real_launch_final.txt).

The real smoke test used `serial_port:=/dev/does-not-exist`. The HereFlow node reported `[Errno 2] No such file or directory`, retried, and did not terminate the navigation graph. The real launch contained no simulator node. The simulation launch did not open the serial port. Unit integration tests additionally exercised session/destination services and a missing serial device.

## Destination regression investigation — 2026-10-14

The reported failed artifact is preserved at [`reports/quick/multi.json`](../reports/quick/multi.json). It recorded `truth_m=[0,0]`, a plotted goal at `(3,0)` m, an estimate near `(0.15,1.45)` m, and `stale/future optical-flow timestamp`. The former scripted multi path ended at `(0,2)` m, so a single finite multi simulator could not produce that truth value. This is strong evidence that another ROS graph shared the absolute simulation topics in the default domain, although the exact second process was no longer present to identify. The launch file also used the straight-path `(3,0)` goal for all paths. A clean isolated baseline run before changes arrived at its then-scripted `(0,2)` goal, supporting the topic-interference diagnosis. No ROS clock mismatch was found within one simulator: truth, IMU, and flow share one ROS timestamp per sample, and preprocessing preserves it. Both finite demo and launch use system ROS time by default.

The finite demo now selects a separate ROS domain by default, accepts `--domain-id` for reproducible isolation, renders its PNG only after callbacks, and verifies that the final truth stamp has a matching estimated pose. The estimator compares ROS stamps in integer nanoseconds without changing `stale_s=0.6` or `future_stamp_tolerance_s=0.05`. Boundary tests cover valid, stale, and genuinely future-dated data; an integration test checks that one simulator sample stamps IMU, flow, and truth identically. The multi script was extended to end at the requested `(3,0)` m destination; the launch default goal is computed from the selected path and geographic origin. Its WGS84 short-offset inverse has round-trip tests. The navigation tolerance remains 0.25 m with three stable updates.

Commands from the workspace root:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
source install/setup.bash
python -m pytest -q src/gps_nav/test
colcon test --python-testing pytest
colcon test-result --verbose
python scripts/run_validation.py --output-dir reports/destination_fix_validation
```

The build completed both packages. Direct pytest reported **45 passed**; `colcon test-result` reported **45 tests, 0 errors, 0 failures, 0 skipped**. The nine-scenario runner exited 0; each demo exited 0, verified its final sample, and wrote JSON and PNG. Captured [summary JSON](../reports/destination_fix_validation/summary.json), [runner output](../reports/destination_fix_validation_runner.txt), and [test output](../reports/destination_fix_test_result.txt) contain exact results. The finite-demo render occurs three seconds after completion, so its final status can say `IMU stale` or `SENSOR DEGRADED` after the correct arrival event; this is a truthful post-run health state.

| Scenario | Endpoint error (m) | Trajectory RMSE (m) | Arrival |
| --- | ---: | ---: | --- |
| Straight | 0 | 0 | yes |
| 90-degree turn | 2.48e-16 | 1.49e-16 | yes |
| Multi-turn to (3,0) | 4.02e-16 | 5.01e-16 | yes |
| Noisy flow | 0.06708 | 0.04610 | yes |
| Flow dropout | 0.45000 | 0.40348 | no |
| IMU dropout | 0.15000 | 0.11754 | no |
| Yaw bias | 0.45634 | 0.20930 | no |
| Low flow quality | 0.45000 | 0.40348 | no |
| Mixed sensor rates | 0.003926 | 0.003231 | yes |

The [multi-turn trajectory plot](../reports/destination_fix_validation/multi/multi.png) was visually checked: the estimated blue path ends on the red `(3,0)` m destination marker. Its [JSON](../reports/destination_fix_validation/multi/multi.json) reports `destination_reached=true`, `final_sample_processed=true`, and final ENU velocity about `(0,-0.997)` m/s. These are deterministic simulation comparisons, not physical accuracy measurements. The four fault cases retain nonarrival; no lost motion was reconstructed. The original failed artifact remains available for comparison.

## Full trajectory visualization regression — 2026-10-14

The finite demo monitor recorded accepted `/estimated_position` poses, while the visualizer had drawn its route by replacing its buffer with every `/estimated_path` snapshot. If an older short snapshot arrived after a newer one, the graph could show only the early segment even though JSON metrics used the full pose history. Also, the finite demo's private ROS domain means an independently launched GUI in the default domain is a different graph and cannot display that run. The fix uses one ordered `TrajectoryHistory` implementation for monitor and visualizer. The visualizer adds accepted pose messages and merges path snapshots without truncating newer data. Session reset clears the history and rejects queued pre-reset stamps. Lines break across long sensor gaps. The renderer uses equal axis scales and explicit limits covering start, destination, all accepted poses, and final pose. Arrival remains latched while later IMU staleness is shown as current health.

Commands run from the workspace root:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
source install/setup.bash
colcon test --python-testing pytest
colcon test-result --verbose
python scripts/run_validation.py --output-dir reports/visualization_fix_validation_final
ros2 launch gps_nav visualization.launch.py --show-args
```

Build result: **2 packages finished**. Test result: **47 tests, 0 errors, 0 failures, 0 skipped** ([captured output](../reports/visualization_fix_test_result.txt)). The new tests feed the multi-turn simulator through the real `Tracker`, then inspect the Matplotlib line: it contains 280 ordered estimated poses, reaches the destination, and is not shortened by a delayed five-pose `/estimated_path` snapshot. A reset test rejects old queued poses. The launch argument check exited 0 and showed configurable `headless` and `output_file` arguments.

The nine-scenario runner exited 0 ([captured output](../reports/visualization_fix_validation_final_runner.txt)); every scenario saved JSON and PNG from its own run, with `plot_history_matches_metrics=true` and matching plotted/paired pose counts. The [summary](../reports/visualization_fix_validation_final/summary.json) has full-precision metrics:

| Scenario | Plotted poses | Endpoint error (m) | Arrival |
| --- | ---: | ---: | --- |
| Straight | 60 | 0 | yes |
| 90-degree turn | 60 | 2.48e-16 | yes |
| Multi-turn | 280 | 4.02e-16 | yes |
| Noisy flow | 60 | 0.06708 | yes |
| Flow dropout | 51 | 0.45000 | no |
| IMU dropout | 57 | 0.15000 | no |
| Yaw bias | 60 | 0.45634 | no |
| Low flow quality | 51 | 0.45000 | no |
| Mixed rates | 31 | 0.003926 | yes |

The [new multi-turn PNG](../reports/visualization_fix_validation_final/multi/multi.png) was visually checked and shows every turn through the final `(3,0)` m destination. Its [JSON](../reports/visualization_fix_validation_final/multi/multi.json) reports `destination_reached=true`, `final_sample_processed=true`, `plot_history_matches_metrics=true`, and `plotted_trajectory_samples=280`. The post-run status is `IMU stale` while the arrival event remains visible.

An additional **QtAgg offscreen GUI rendering smoke test** used the same visualizer `draw()` code with 280 estimator-derived multi-turn poses. It recorded `plotted_samples=280`, final point approximately `(3,0)` m, arrival latched, and a saved [GUI-code-path PNG](../reports/gui_offscreen_multi_final.png) ([result JSON](../reports/gui_offscreen_multi_final.json)). This checks the interactive backend's rendering logic and saved figure without claiming that a physical WSL GUI window was visible in this sandbox. For a visible final window and JSON/PNG from one finite run on a working WSLg/X display, use `ros2 run gps_nav demo --path multi --output-dir reports/multi_gui --show`. For live visualization of a launch graph, use `ros2 launch gps_nav simulation.launch.py path:=multi headless:=false output_file:=multi_live.png`.

The finite `--show` path was also exercised under QtAgg offscreen with `plt.show` replaced by a nonblocking test stub (the sandbox cannot display a real window). That ROS run exited 0 and saved [JSON](../reports/gui_same_run_multi/multi.json) and [PNG](../reports/gui_same_run_multi/multi.png) together: 280 recorded and plotted poses, matching histories, final sample processed, and arrival at `(3,0)` m. The saved PNG was inspected and contains the full route. This validates the same-run recorder and GUI rendering path; actual window presentation on WSLg remains untested here.

## Unverified physical behavior

There was no attached HereFlow or IMU during this run, so UART byte order, actual packet stream, sensor units, optical-flow scale, mount axes, physical timestamp semantics, and ENU heading reference remain unverified. The real configuration has `calibration_valid=false`, zero flow scales, `heading_reference_valid=false`, and velocity publishing disabled. Thus real-mode metre position and destination arrival cannot be claimed. The physical validation and held-out calibration procedure is in [`calibration.md`](calibration.md).

## Hardware-readiness continuation — 2026-10-13

The final source was rebuilt with:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
ros2-env colcon test --python-testing pytest
ros2-env colcon test-result --verbose
```

The final build reported `2 packages finished [33.9s]`. The test result was `39 tests, 0 errors, 0 failures, 0 skipped`, captured in [`hardware_readiness_test_result.txt`](../reports/hardware_readiness_test_result.txt). New tests use a pseudo-terminal and injected port failures to cover valid raw packet reception, permission denial, disconnect/reconnect, partial-frame clearing, checksum failure, quality statistics, stale serial status, missing calibration, nonunity smoothing, invalid IMU covariance/frame, delayed IMU/flow timestamps, and rejection of metric guidance without calibration or heading. The existing geographic and simulation tests remain in the suite.

The standalone command below was run against the current default WSL port:

```bash
source install/setup.bash
ros2 run gps_nav hereflow_diagnostic --serial-port /dev/ttyUSB0 \
  --duration 0.25 --reconnect-interval 0.05 \
  --output reports/hardware/diagnostic_absent.json
```

It exited **2**, as designed for a port that never opened. Its [captured JSON](../reports/hardware/diagnostic_absent.json) reports `exists=false`, `connection_attempts=5`, `open_errors=5`, `bytes_received=0`, `valid_packets=0`, and `data_stale=true`. This is evidence of an unavailable serial device, not a physical parser test. The physical IMU remains unidentified.

A real-launch smoke test used `serial_port:=/dev/does-not-exist` on a separate ROS domain while simulation validation ran. It started six nodes, logged the missing path, started no simulator, and one SIGINT to the launcher ended all six cleanly: launcher exit 0, zero tracebacks. The [captured launch log](../reports/hardware_readiness_real_launch.txt) supports these process checks. This does not validate a physical serial stream.

The nine-scenario runner was repeated after the timestamp guard was installed:

```bash
source install/setup.bash
python scripts/run_validation.py --output-dir reports/hardware_readiness_validation_final \
  > reports/hardware_readiness_validation_final_runner.txt
```

All nine processes exited 0. The actual final-build metrics were:

| Scenario | Endpoint error (m) | Trajectory RMSE (m) | Arrival observed |
| --- | ---: | ---: | --- |
| Straight | 0 | 0 | yes |
| 90-degree turn | <1e-12 | <1e-12 | yes |
| Multi-turn | <1e-12 | <1e-12 | yes |
| 0.005 m simulated flow noise | 0.06708 | 0.04610 | yes |
| Flow dropout | 0.45000 | 0.40348 | no |
| IMU dropout | 0.20000 | 0.15811 | no |
| 0.1 rad/s simulated yaw bias | 0.45659 | 0.20949 | no |
| Low raw quality interval | 0.45000 | 0.40348 | no |
| Flow 10 Hz / IMU 20 Hz | 0.00393 | 0.00323 | yes |

The [final summary JSON](../reports/hardware_readiness_validation_final/summary.json), scenario stdout/stderr and PNG files, and [runner output](../reports/hardware_readiness_validation_final_runner.txt) are the evidence. The noise generator uses a fixed seed, but ROS wall-clock scheduling can alter which queued measurements pass freshness checks, so dropout and yaw-bias endpoint results can vary slightly between runs. These remain simulation-only comparisons. GUI display and physical accuracy were not tested.

After extending the missing-port test to assert `/sensor_status/hereflow` contents, its targeted rerun reported `1 passed in 0.62s`; the source code was unchanged after the 39-test full run.

## Live visualization and goal-directed simulation — 2026-10-14

The earlier finite `--show` implementation drew after simulation completion because it spun ROS in the main thread and called the visualizer only after the finite loop. The current demo draws start and destination before ROS processing starts, spins a single ROS executor on a background thread, and redraws on the Matplotlib main thread every 0.2 s. ROS callbacks only update lock-protected state and ordered pose history. PNG and JSON come from the same final history. Closing the graph stops the executor and ROS nodes cleanly.

Goal mode uses the requested ENU destination and incremental turning/translation. Scripted mode retains the straight, turn, and intentionally rectangular multi-turn paths as validation cases. Both modes use the production preprocessing, estimator, navigator, arrival latch, timestamp guards, and sensor validity gates. The 0.25 m arrival tolerance and three-update stability rule were not changed. `path_length_ratio` is reported only for arrived, calibrated goal-mode runs with a nonzero reference distance and no unobserved gaps. A value below 1 can occur because arrival is accepted inside the tolerance before the exact waypoint; `path_over_achieved_displacement` describes straightness of the observed path.

Commands used:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
source install/setup.bash
colcon test --python-testing pytest
colcon test-result --verbose
python scripts/run_validation.py --output-dir reports/live_arch_validation_final2
python scripts/run_goal_validation.py --output-dir reports/goal_validation_final
python scripts/run_live_gui_smoke.py --mode goal --output-dir reports/live_goal_final2
python scripts/run_live_gui_smoke.py --mode scripted --path multi --output-dir reports/live_multi_final
```

The final rebuild completed **2 packages in 1 min 4 s**. `colcon test-result --verbose` reported **51 tests, 0 errors, 0 failures, 0 skipped**. `ros2 launch gps_nav simulation.launch.py --show-args` exited 0 and listed the new goal-mode and motion parameters. The original nine scripted cases plus a prolonged IMU dropout case completed with exit code 0 and matched visualizer/recorder histories ([summary](../reports/live_arch_validation_final2/summary.json)). Clean straight, 90-degree turn, and multi-turn endpoint errors were 0, 2.48e-16, and 4.02e-16 m respectively; multi had 280 estimated poses and arrived. Noise produced 0.06325 m endpoint error and arrived. Flow dropout and low quality each gave 0.45 m endpoint error and withheld arrival. Brief IMU dropout arrived after recovery with 0.15 m endpoint error, legitimately inside the unchanged arrival tolerance; the prolonged IMU dropout gave 0.85 m error and withheld arrival. Yaw bias withheld arrival at 0.45634 m endpoint error. Mixed rates arrived at 0.00393 m endpoint error.

Four goal-mode cases completed ([summary](../reports/goal_validation_final/summary.json)). Clean goal mode arrived at estimated and true (2.82, 0) m for a (3, 0) m waypoint: endpoint error 0, trajectory RMSE 0, estimated path 2.82 m, reference distance 3 m, ratio 0.94, and path/achieved-displacement ratio 1.0. The noisy case arrived with 0.09220 m endpoint error, 0.06905 m trajectory RMSE, and path/reference ratio 0.95554. Flow dropout and low quality timed out without arrival; each had 0.27 m endpoint error and no path/reference ratio.

QtAgg offscreen smoke runs exercised the actual `--show` event loop. The [goal JSON](../reports/live_goal_final2/goal.json) records the first graph draw before any ground-truth sample, 9 graph draws before completion, an [in-progress frame](../reports/live_goal_final2/goal_in_progress.png) with 13 estimated poses, and a [final frame](../reports/live_goal_final2/goal.png) with 27 poses. The [multi JSON](../reports/live_multi_final/multi.json) records 34 pre-completion draws, an [in-progress frame](../reports/live_multi_final/multi_in_progress.png) with 16 poses, and a [final frame](../reports/live_multi_final/multi.png) with 280 poses ending at the goal. Initial, in-progress, and final PNGs were inspected: the route grows while the run is active, and the final multi frame shows every turn. A scripted close event exited 0 with `graph closed; simulation stopped` ([output](../reports/live_close_smoke_stdout.txt)). These are offscreen GUI event-loop tests; a visible WSLg window could not be observed in this environment. No physical HereFlow or IMU performance is inferred.

## Three-mode selection and interactive motion — 2026-10-14

The older CLI and launch file defaulted to `scripted`; consequently `demo --path multi` ran the rectangular test route. They now default to `goal`. Passing a non-default `--path` without `--mode scripted` exits 2 with `--path selects a validation route; use --mode scripted`; the launch file applies the same guard. The simulator startup log, CLI line, plot title, and plot status show the effective mode. Scripted startup lines identify the validation path rather than printing unused goal-mode speed parameters.

The following commands were run after rebuilding both packages:

```bash
source install/setup.bash
colcon test --python-testing pytest && colcon test-result --verbose
ros2 run gps_nav demo --mode goal --goal-x-m 3 --goal-y-m 0 --output-dir reports/mode_fix_goal
python scripts/run_validation.py --output-dir reports/mode_fix_validation
python scripts/run_goal_validation.py --output-dir reports/mode_fix_goal_validation
python scripts/run_interactive_smoke.py --output-dir reports/mode_fix_interactive
python scripts/run_live_gui_smoke.py --mode goal --goal-x-m 3 --goal-y-m 0 --output-dir reports/mode_fix_live_goal
python scripts/run_live_gui_smoke.py --mode scripted --path multi --output-dir reports/mode_fix_live_multi_repeat
python scripts/run_interactive_close_smoke.py reports/mode_fix_interactive_close
```

The build completed two packages. The full test result was **54 tests, 0 errors, 0 failures, 0 skipped**. The headless [goal run](../reports/mode_fix_goal/goal.json) reported `mode=goal`, destination reached, 94 estimated poses, 2.82 m estimated path length, 3.00 m straight-line reference, ratio 0.94, 0 m endpoint error, and 0 m trajectory RMSE. The 0.18 m remaining distance is inside the unchanged 0.25 m tolerance after three stable updates. Path length divided by the achieved 2.82 m displacement is 1.0. No optimality is claimed from arrival inside the tolerance.

All ten scripted cases exited 0 ([summary](../reports/mode_fix_validation/summary.json)). Straight, turn, and multi endpoint errors were 0, 2.48e-16, and 4.02e-16 m respectively; each arrived. Noise arrived with 0.06708 m endpoint error. Flow dropout, both IMU dropouts, yaw bias, and low quality withheld arrival. Mixed rates arrived with 0.00393 m endpoint error. The four [goal cases](../reports/mode_fix_goal_validation/summary.json) also exited 0: clean and noisy arrived; flow dropout and low quality timed out without false arrival.

The external ROS [interactive smoke test](../reports/mode_fix_interactive/interactive.json) ran `--mode interactive --max-duration-s 8`, published motion commands to `/simulation/cmd_vel`, and called `/set_destination` while the simulator was moving. The estimate advanced to (1.62, 0) m; the local destination changed from approximately (3, 0) to (0, 2.986) m; the run produced 161 paired poses, retained an ordered history equal to the final plot, and did not falsely report arrival. A [GUI close test](../reports/mode_fix_interactive_close_stdout.txt) exited 0 after closing an offscreen interactive window, with `completion_reason=user stopped`, 18 paired poses, and equal recorder/plot histories. For a user-stopped run, the final truth sample may remain unprocessed and is reported as such rather than asserted as complete.

The QtAgg offscreen [live goal JSON](../reports/mode_fix_live_goal/goal.json) records an initial graph before truth, 27 draws before completion, a partially drawn [11-pose frame](../reports/mode_fix_live_goal/goal_in_progress.png), and a [94-pose final plot](../reports/mode_fix_live_goal/goal.png). The inspected in-progress plot title says `goal mode` and shows a straight trajectory. The repeated [live multi JSON](../reports/mode_fix_live_multi_repeat/multi.json) records its separate scripted rectangular path with 280 plotted poses ending at the destination. One earlier offscreen multi run accepted 279 of 280 poses and ended 0.05 m short of truth while still within the arrival tolerance; the repeated run accepted all 280. That scheduling-dependent sample rejection is visible in the artifacts, and freshness checks were not loosened. A visible WSLg window and physical sensor behavior remain untested.

## User-defined geographic start and destination — 2026-10-14

Goal-mode CLI inputs now accept `--start-lat/--start-lon` and `--destination-lat/--destination-lon` (with the old origin/goal names as aliases). Missing geographic fields trigger terminal prompts, including an initial heading prompt with 0° East as the documented default. Inputs are parsed as finite floats, bounded by valid latitude/longitude ranges, and retried if malformed or empty. A destination less than 0.01 m from the fixed ENU origin is rejected before movement. Local `--goal-x-m/--goal-y-m` runs remain noninteractive. An omitted finite-demo timeout is calculated from the converted distance, configured speed and worst-case turning time; the existing navigation arrival tolerance and health gates were unchanged.

Commands run after the final source build:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
source install/setup.bash
colcon test --python-testing pytest && colcon test-result --verbose
python scripts/run_validation.py --output-dir reports/user_coords_final_scripted
python scripts/run_goal_validation.py --output-dir reports/user_coords_final_goal
python scripts/run_geographic_goal_validation.py --output-dir reports/user_coords_final_geographic
printf '13.0827\n80.2707\n13.082727\n80.2707\n90\n' | \
  python scripts/run_live_gui_smoke.py --mode goal --max-duration-s 8 \
    --output-dir reports/user_coords_final_live
```

The final build completed **2 packages** and `colcon test-result --verbose` reported **60 tests, 0 errors, 0 failures, 0 skipped**. New unit tests cover prompt retries for empty, malformed, out-of-range, and nonfinite input; zero-distance retry/rejection; geographic CLI aliases; a measured ENU target; arbitrary initial heading; nonteleporting fixed-step motion; and equal graph extents. Tests run headless.

The [scripted summary](../reports/user_coords_final_scripted/summary.json) records exit 0 for all nine original cases plus prolonged IMU dropout. Straight, turn, and multi reached their goals; flow dropout, both IMU dropouts, yaw bias, and low quality withheld arrival; noise and mixed rates retained their expected arrivals. The [goal fault summary](../reports/user_coords_final_goal/summary.json) records clean/noisy arrival and flow-dropout/low-quality nonarrival. The [geographic summary](../reports/user_coords_final_geographic/summary.json) records four exit-0 arrivals for north aligned, north after an initial East heading, northeast after an initial West heading, and west after an initial North heading. Its clean endpoint errors were 0 m against separate simulation truth.

The prompted QtAgg offscreen demonstration used current geographic position **(13.0827°, 80.2707°)**, destination **(13.082727°, 80.2707°)**, and initial heading **90°**. WGS84 conversion gave local target **(approximately 0, 2.987042169) m** and 2.987042169 m straight-line distance. The [same-run JSON](../reports/user_coords_final_live/goal.json) reports `mode=goal`, arrival, 2.82 m estimated path length, 0.944077733 path/reference ratio, 0 m truth-to-estimate endpoint error, 94 plotted poses, 36 pre-completion GUI draws, and matching visualizer/recorder histories. The [initial](../reports/user_coords_final_live/goal_initial.png), [13-pose in-progress](../reports/user_coords_final_live/goal_in_progress.png), and [final](../reports/user_coords_final_live/goal.png) images were visually inspected. The 0.167 m remaining waypoint distance is inside the unchanged 0.25 m arrival tolerance; later IMU staleness is displayed separately from the latched arrival event. The Qt event loop was exercised offscreen; presentation in a visible WSLg window and physical sensor accuracy remain unverified.

## Repository transfer audit — 2026-10-14

This audit ran on Ubuntu 24.04 under WSL 2 with the existing Jazzy micromamba environment, not on a Raspberry Pi. Commands and observed results:

```bash
ros2-env colcon build --base-paths src --packages-select gps_nav_interfaces gps_nav
ros2-env colcon test --packages-select gps_nav_interfaces gps_nav --python-testing pytest
ros2-env colcon test-result --verbose
# Summary: 2 packages finished; 89 tests, 0 errors, 0 failures, 0 skipped.
source /home/mirtt/micromamba/envs/ros2-jazzy/setup.bash
source install/setup.bash
python scripts/run_robot_control_validation.py --output-dir reports/robot_control_validation_final
python scripts/run_validation.py --output-dir reports/current_scripted_validation
python scripts/run_goal_validation.py --output-dir reports/current_goal_validation
python scripts/run_geographic_goal_validation.py --output-dir reports/current_geographic_validation
g++ -std=c++17 -Wall -Wextra -Werror -I firmware/tests firmware/tests/protocol_host_test.cpp -o /tmp/gps_nav_protocol_host_test
/tmp/gps_nav_protocol_host_test
```

The robot-control script exited 0: startup remained stopped, the simulated link fault latched STOP, rearming required reset and enable, arrival latched, and the final command was STOP. It recorded 140 estimated poses, 81 accepted ACKs, 0 rejected ACKs, and final simulation truth and estimate at approximately (2.88, 0) m ([summary](../reports/robot_control_validation_final/summary.json)). Its ROS parent emitted `getifaddrs`/UDP socket permission diagnostics in this environment while the validation still completed; this should be rerun on the target Pi. The host firmware parser test exited 0 without output; no firmware was flashed.

All 10 scripted scenarios exited 0 and met the runner's arrival/fault checks ([summary](../reports/current_scripted_validation/summary.json)). Straight, 90-degree turn, and multi-turn arrived with endpoint errors 0, 2.48e-16, and 4.02e-16 m. Noise arrived with 0.0671 m endpoint error. Flow dropout, both IMU dropout cases, yaw bias, and low quality withheld arrival; mixed rates arrived with 0.00393 m endpoint error. All 4 goal scenarios exited 0: clean and noise arrived, and flow dropout and low quality withheld arrival ([summary](../reports/current_goal_validation/summary.json)). The clean goal estimated path was 2.82 m toward a 3.00 m goal, ratio 0.94 because the unchanged arrival tolerance accepts a point inside the goal region. All 4 geographic goal scenarios exited 0 and arrived ([summary](../reports/current_geographic_validation/summary.json)). These are simulation results, not physical accuracy measurements.

`ros2 run gps_nav demo --help` and `ros2 launch gps_nav robot_real.launch.py --show-args` confirmed the documented CLI options and default-off real-control launch arguments. No `/dev/ttyUSB*` or `/dev/ttyACM*` device was present, and no Raspberry Pi, physical IMU, robot, or real serial watchdog was tested. The current Pi storage and native ARM64 build remain unmeasured.
