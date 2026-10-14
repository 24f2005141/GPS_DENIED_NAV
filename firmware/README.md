# Protocol-only ESP32/Arduino sketch

`arduino_protocol_only/arduino_protocol_only.ino` implements the V1 line parser, CRC check, sequence rejection, acknowledgement, ARM/RESET/STOP state, and 500 ms watchdog. The `NoMotorBackend` has no pin assignments and rejects all movement with `NO_DRIVER`; uploading this sketch cannot exercise motors. It has **not** been compiled for or flashed to a board in this environment. The actual sketch **was** compiled and exercised with a minimal host Arduino API stub:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -I firmware/tests \
  firmware/tests/protocol_host_test.cpp -o /tmp/gps_nav_protocol_host_test
/tmp/gps_nav_protocol_host_test
```

That test checks STOP, ARM, rejection by the no-motor backend, old STOP priority, RESET, watchdog, and corrupt CRC. It is a host parser test, not evidence of board timing or motor behavior. The executable Python `MicrocontrollerModel` implements the same state machine for protocol tests and ROS simulation; it is not a substitute for a board test.

Before adding a motor backend, record the exact ESP32 or Arduino board, logic voltage, USB interface, motor-driver model, motor supply, wiring diagram, direction/PWM truth table, wheel geometry, encoder availability, and physical emergency-stop behavior. Check that motor outputs default inactive during reset and boot. Implement `MotorBackend.stop()` so it immediately de-energizes both channels and `apply()` so it validates bounded direction/speed before touching pins. Test STOP, malformed input, USB unplug, host crash, watchdog expiry, reset, and brownout with wheels restrained. Only then consider ground movement at low speed. See [protocol](../docs/robot_protocol.md) and [deployment](../docs/raspberry_pi_deployment.md).
