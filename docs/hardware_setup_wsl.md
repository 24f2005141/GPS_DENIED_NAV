# WSL 2 USB and IMU setup

The current run found no `/dev/ttyUSB*` or `/dev/ttyACM*` and `lsusb` was unavailable. COM10 and `/dev/ttyUSB0` are historical observations only. In Windows PowerShell, identify the present CP2102 BUSID with `usbipd list`; use `usbipd bind --busid <BUSID>` from an administrator prompt once, then `usbipd attach --wsl --busid <BUSID>` with WSL running. These commands follow [Microsoft's WSL USB instructions](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) and the [usbipd-win WSL guide](https://github.com/dorssel/usbipd-win/wiki/WSL-support).

In Ubuntu inspect `ls -l /dev/ttyUSB* /dev/serial/by-id/*` and `id`. Confirm the current device path and read permission; avoid assuming ttyUSB0 if several adapters exist. `ros2 launch gps_nav real.launch.py serial_port:=/dev/ttyUSB0` retries when unavailable and publishes connection status. Use `ros2 topic echo /optical_flow/raw` to see accepted packets. Do not set flow calibration from packet presence alone.

## Repeatable CP2102 verification

Keep WSL open. In **Windows PowerShell**, run `usbipd list`, identify the current CP2102 BUSID, then run `usbipd bind --busid <BUSID>` in an administrator PowerShell if it is not already shared. Run `usbipd attach --wsl --busid <BUSID>` and check `usbipd list` again. These commands are from [Microsoft's current WSL USB procedure](https://learn.microsoft.com/en-us/windows/wsl/connect-usb). The historical COM10 label does not identify today's BUSID.

In **Ubuntu WSL**, inspect the device and user permissions:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/*
id -nG
stat -c '%n %a %U:%G' /dev/ttyUSB0
```

Use the device path actually found; `stat` above is an example for ttyUSB0. If there is no device, return to `usbipd list` and USB attachment before starting ROS. If opening it reports permission denied, compare its group with `id -nG`; add the user to that device group using the normal local administration procedure, then start a new login session. A `read_access_hint` in the diagnostic is only a precheck; its actual open result is authoritative. Avoid running another serial monitor at the same time because the current POSIX reader does not guarantee exclusive ownership.

With the workspace built and sourced, run the standalone check **before** starting `real.launch.py`:

```bash
ros2 run gps_nav hereflow_diagnostic --serial-port /dev/ttyUSB0 \
  --baud-rate 19200 --duration 15 --reconnect-interval 2 \
  --capture reports/hardware/hereflow_raw.jsonl \
  --output reports/hardware/hereflow_diagnostic.json
```

The command exits 0 if at least one packet matches the configured frame, 2 if no connection opens, or 3 if a connection opens but no valid packet arrives. Inspect `valid_packets`, `invalid_frames`, `checksum_failures`, `discarded_bytes`, `quality_min/mean/max`, `data_stale`, and `last_error`. Test disconnect/reconnect by safely detaching and reattaching the USB device during a longer diagnostic run; confirm `disconnects` and `reconnects` increase and that no partial frame crosses the reconnect. The raw capture records receive-time hex bytes, not metres or a verified device timestamp. Do controlled stationary, body +X, body +Y, and in-place rotation trials at a measured height; preserve their independent reference distances and surface/lighting notes.

Only after matching captured bytes to the exact board documentation should `byte_order` be selected in the real launch. A high checksum-valid packet count cannot determine byte order or physical units. Keep `calibration_valid: false` and `heading_reference_valid: false` until the separate calibration and IMU checks are complete.

The IMU model, bus, register map, units, and mounting are unknown. Obtain model and firmware, connection type (I2C/SPI/serial/USB), pinout or protocol documentation, output units, quaternion convention if available, data rate, and whether there is a magnetometer. A model-specific driver should convert raw data to SI and publish `sensor_msgs/Imu` on `/imu/data` with frame ID and covariance. The estimator accepts orientation only when a valid quaternion is present and orientation covariance is not marked unavailable (-1). Test axes and timestamps against an independent heading before real navigation.

Future driver integration steps:

1. Record the exact product and firmware, bus address or serial settings, packet/register specification, calibration commands, axes printed on the board, mounting yaw relative to `base_link`, and output rate. Verify whether the device publishes a fused quaternion or only raw gyro and accelerometer. A magnetometer is optional; if used, publish `sensor_msgs/MagneticField` separately and document its calibration.
2. Implement the device-specific acquisition in a new module and launch it only from the real launch. Timestamp samples at acquisition on the same ROS time base as the estimator; samples older than `stale_s` or beyond `future_stamp_tolerance_s` are rejected. Use radians/second for angular velocity and metres/second squared for linear acceleration. Set unavailable covariance fields to -1 per `sensor_msgs/Imu`; never fill an unmeasured orientation with an identity quaternion marked valid.
3. Rotate or configure the IMU axes into the documented `base_link` planar convention (forward +X, left +Y, up +Z). Publish `header.frame_id` matching `expected_imu_frame` (default `imu_link`); the estimator rejects a different or missing frame. Establish the quaternion's yaw zero and sign with controlled 90-degree turns. Configure `heading_offset_deg` for the measured planar offset to ENU East. The current estimator uses yaw only; full roll/pitch compensation and platform tilt require an added validated flow model.
4. If no fused orientation is supplied, implement and independently test a separate orientation estimator before feeding `/imu/data` to this estimator. A gyro-only yaw needs an initial heading reference and drifts; accelerometer data cannot establish yaw. If magnetic yaw is used, measure hard/soft iron calibration and local interference.
5. Run stationary and controlled-turn captures, check timestamps and a finite positive yaw variance within `max_yaw_variance_rad2`, and compare yaw against an independent heading reference. The orientation covariance's first element must not be -1, which means orientation unavailable in `sensor_msgs/Imu`. Only then set `heading_reference_valid: true` in the real configuration. Keep simulation on its separate launch path throughout.
