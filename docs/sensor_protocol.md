# UART optical-flow protocol and unresolved identity

The implemented 9-byte framing is `FE 04 b2 b3 b4 b5 checksum quality AA`, with checksum `(b2+b3+b4+b5)&0xff`. Motion components are signed 16-bit. `FlowParser` survives partial, concatenated, malformed, and garbage-prefixed streams and tracks valid, invalid, and checksum-failed packets. `HereFlowNode` reads a POSIX UART at 19200 baud, timestamps reception, reconnects after errors, and publishes raw counts only.

**Byte-order conflict:** the [Holybro PMW3901 UART product page](https://holybro.com/products/pmw3901-optical-flow-sensor) labels byte 2 as X high and byte 3 as X low. The [PX4 ThoneFlow parser](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/optical_flow/thoneflow/thoneflow_parser.cpp) decodes the four motion bytes low-first. The user's previously tested packet description also says low-first. `byte_order` is configurable; default `little` is provisional, not hardware-verified in this session. The name HereFlow may refer to a different CAN device; verify the physical model and captured stream before attributing this protocol to it.

Quality is published as its raw unsigned 0–255 value. It is not normalized to a physical confidence probability. The parser does not assume that counts are metres, velocity, or angle. [PX4's optical-flow guide](https://github.com/PX4/PX4-user_guide/blob/main/en/sensor/optical_flow.md) describes the need for ground distance in velocity estimation. Verify whether this particular board has a range sensor and whether its UART stream provides one. The implemented packet has no height field.

The current environment had no `/dev/ttyUSB*` device, so neither byte order nor packet frequency nor hardware identity could be confirmed from captured bytes. An observed stream should be saved as hex with times before enabling calibration.

## Assumption and evidence ledger

| Item | Evidence available now | Hardware status / action |
| --- | --- | --- |
| CP2102 at COM10 / `/dev/ttyUSB0` | Previously reported connection path | Historical only. Find the current BUSID and Linux device path again. |
| 19200 baud, `FE 04 ... AA` frame | [Holybro PMW3901 UART page](https://holybro.com/products/pmw3901-optical-flow-sensor) and the user's prior packet description | The parser passes synthetic tests; no physical stream was captured here. Confirm the exact board is this UART model. |
| Four signed 16-bit motion bytes | Prior packet description and [PX4's ThoneFlow parser](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/optical_flow/thoneflow/thoneflow_parser.cpp) | The ThoneFlow driver is related reference code, not proof of this board's firmware. Compare captured positive and negative controlled motion. |
| Little-endian default | Prior packet description and PX4 parser | Holybro's product page labels bytes high first. Configure `--byte-order` only after checking a capture. Checksum success alone cannot distinguish the two orders. |
| Checksum is sum of four motion bytes modulo 256 | Prior packet description and PX4 parser | Synthetic parser tests pass. Compare checksum statistics on a physical stream. |
| Quality is a raw 0–255 byte | Frame layout and message type | Actual threshold and relation to tracking error are unmeasured. The real config's `minimum_quality: 30` is provisional; no probability interpretation is used. |
| Metric scale, motion units, sample interval | Simulation has a known 0.01 m/count model; the real config has no scale | **Unverified for hardware.** No received UART field identifies height or calibrated metres. Determine flow semantics, ground distance, rotation compensation, and packet timing before enabling `metric_valid`. |
| Sensor X/Y signs and mounting yaw | Configurable `invert_x`, `invert_y`, `rotation_offset_deg` | Unknown until body +X/+Y translations and stationary rotations are recorded. |
| IMU alignment and East heading | Standard `/imu/data` contract; simulated orientation works | The physical IMU model and reference remain unknown. Real `heading_reference_valid` stays false. |

The standalone `ros2 run gps_nav hereflow_diagnostic` uses the same serial reader and parser as the ROS source node. It checks `stat` and a read-access hint, then actually attempts to open and read the port. Its JSON includes connection attempts, successful connections, reconnections after the first open, disconnects, open/read errors, bytes received, valid packets, invalid candidate frames, checksum failures, discarded stream bytes, time since the last accepted frame, and raw quality minimum/mean/maximum. Garbage outside a candidate frame contributes to `discarded_bytes`, not `invalid_frames`. A quiet port is reported as stale rather than as a packet failure. Port access cannot prove exclusive ownership by another program; stop other serial readers before diagnosis.

The `--capture` option writes JSONL with a wall-clock **receive time for each read chunk** and its bytes in hex. Neither the capture time nor the ROS receipt timestamp is a sensor measurement time. A single read may contain multiple packets. A successful diagnostic only confirms acceptance by this hypothesized framing; it does not verify the board identity, units, axis directions, or physical accuracy.
