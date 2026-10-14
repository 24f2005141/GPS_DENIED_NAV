// Compile the actual protocol-only sketch against a minimal host Arduino API.
#include "Arduino.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <string>

unsigned long fake_millis = 0;
FakeSerial Serial;
#include "../arduino_protocol_only/arduino_protocol_only.ino"

static std::string command(unsigned long sequence, const char *direction,
                           unsigned speed) {
  char payload[80];
  std::snprintf(payload, sizeof(payload), "V1,%lu,%s,%u", sequence, direction,
                speed);
  char line[96];
  std::snprintf(line, sizeof(line), "%s,%04X\n", payload, crc16(payload));
  return line;
}

static std::string exchange(const std::string &wire) {
  Serial.output.clear(); Serial.feed(wire); loop();
  const size_t end = Serial.output.find("\r\n");
  const size_t comma = Serial.output.rfind(',', end);
  assert(end != std::string::npos && comma != std::string::npos);
  const std::string payload = Serial.output.substr(0, comma);
  const std::string supplied = Serial.output.substr(comma + 1, end - comma - 1);
  assert(supplied.size() == 4);
  assert(std::strtoul(supplied.c_str(), nullptr, 16) == crc16(payload.c_str()));
  return Serial.output;
}

int main() {
  setup();
  assert(!armed && !fault_latched);
  assert(exchange(command(1, "STOP", 0)).find(",OK,STOP,STOPPED,") !=
         std::string::npos);
  assert(exchange(command(2, "ARM", 0)).find(",OK,ARM,ARMED,") !=
         std::string::npos);
  // NoMotorBackend must reject physical movement.
  assert(exchange(command(3, "FORWARD", 50)).find(",ERR,FORWARD,NO_DRIVER,") !=
         std::string::npos);
  assert(!armed && fault_latched);
  assert(exchange(command(2, "STOP", 0)).find(",OK,STOP,STOPPED,") !=
         std::string::npos);  // Old STOP still has priority.
  assert(exchange(command(4, "RESET", 0)).find(",OK,RESET,RESET,") !=
         std::string::npos);
  assert(!fault_latched && !armed);
  assert(exchange(command(5, "ARM", 0)).find(",OK,ARM,ARMED,") !=
         std::string::npos);
  fake_millis = 501; loop();
  assert(!armed && fault_latched);  // Firmware watchdog fired.
  assert(exchange(command(6, "FORWARD", 50)).find(",ERR,FORWARD,FAULT,") !=
         std::string::npos);
  std::string corrupt = command(7, "STOP", 0);
  corrupt[3] = '9';
  assert(exchange(corrupt).find(",ERR,UNKNOWN,BAD_CRC,") !=
         std::string::npos);
  assert(!armed && fault_latched);
  return 0;
}
