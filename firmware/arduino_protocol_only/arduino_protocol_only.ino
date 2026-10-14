// Protocol-only ESP32/Arduino sketch. No motor pins are assigned or driven.
// Replace NoMotorBackend only after board, driver, wiring and E-stop are verified.
#include <Arduino.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const unsigned long WATCHDOG_MS = 500;
static const uint16_t MAX_SPEED_PERMILLE = 100;
static char line_buffer[97];
static size_t line_length = 0;
static bool discarding = false;
static bool armed = false;
static bool fault_latched = false;
static uint32_t last_sequence = 0;
static bool has_sequence = false;
static unsigned long last_motion_ms = 0;

class MotorBackend {
 public:
  virtual void stop() = 0;
  virtual bool apply(const char *direction, uint16_t speed_permille) = 0;
};

class NoMotorBackend : public MotorBackend {
 public:
  void stop() override {}  // No pins exist in the protocol-only build.
  bool apply(const char *, uint16_t) override { return false; }
};

static NoMotorBackend motor;

static void safe_stop() {
  motor.stop();
  armed = false;
}

static uint16_t crc16(const char *text) {
  uint16_t crc = 0xFFFF;
  while (*text) {
    crc ^= static_cast<uint8_t>(*text++) << 8;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

static void acknowledge(uint32_t sequence, bool accepted,
                        const char *command, const char *reason) {
  char payload[80];
  snprintf(payload, sizeof(payload), "A1,%lu,%s,%s,%s",
           static_cast<unsigned long>(sequence), accepted ? "OK" : "ERR",
           command, reason);
  Serial.print(payload);
  Serial.print(',');
  char crc[5];
  snprintf(crc, sizeof(crc), "%04X", crc16(payload));
  Serial.println(crc);
}

static bool parse_uint(const char *text, uint32_t *value) {
  if (!*text || strlen(text) > 10) return false;
  for (const char *p = text; *p; ++p) if (*p < '0' || *p > '9') return false;
  char *end = nullptr;
  errno = 0;
  unsigned long parsed = strtoul(text, &end, 10);
  if (*end || errno == ERANGE || parsed > 0xFFFFFFFFUL) return false;
  *value = static_cast<uint32_t>(parsed);
  return true;
}

static bool newer(uint32_t sequence) {
  if (!has_sequence) return true;
  uint32_t difference = sequence - last_sequence;
  return difference != 0 && difference < 0x80000000UL;
}

static void process_line(char *line) {
  char *last_comma = strrchr(line, ',');
  if (!last_comma || strlen(last_comma + 1) != 4) {
    safe_stop(); fault_latched = true;
    acknowledge(0, false, "UNKNOWN", "BAD_FRAME"); return;
  }
  char *crc_text = last_comma + 1;
  char *end = nullptr;
  unsigned long supplied = strtoul(crc_text, &end, 16);
  if (*end || supplied > 0xFFFF) {
    safe_stop(); fault_latched = true;
    acknowledge(0, false, "UNKNOWN", "BAD_CRC"); return;
  }
  *last_comma = '\0';
  if (crc16(line) != supplied) {
    safe_stop(); fault_latched = true;
    acknowledge(0, false, "UNKNOWN", "BAD_CRC"); return;
  }
  char *save = nullptr;
  char *version = strtok_r(line, ",", &save);
  char *seq_text = strtok_r(nullptr, ",", &save);
  char *direction = strtok_r(nullptr, ",", &save);
  char *speed_text = strtok_r(nullptr, ",", &save);
  if (!version || strcmp(version, "V1") || !seq_text || !direction ||
      !speed_text || strtok_r(nullptr, ",", &save)) {
    safe_stop(); fault_latched = true;
    acknowledge(0, false, "UNKNOWN", "BAD_FRAME"); return;
  }
  uint32_t sequence = 0, speed = 0;
  if (!parse_uint(seq_text, &sequence) || sequence == 0 ||
      !parse_uint(speed_text, &speed) || speed > 1000) {
    safe_stop(); fault_latched = true;
    acknowledge(0, false, "UNKNOWN", "BAD_SPEED"); return;
  }
  bool stop = strcmp(direction, "STOP") == 0 || strcmp(direction, "DISARM") == 0;
  bool arm = strcmp(direction, "ARM") == 0;
  bool reset = strcmp(direction, "RESET") == 0;
  bool move = strcmp(direction, "FORWARD") == 0 ||
              strcmp(direction, "BACKWARD") == 0 ||
              strcmp(direction, "LEFT") == 0 || strcmp(direction, "RIGHT") == 0;
  if ((!stop && !arm && !reset && !move) ||
      ((stop || arm || reset) && speed != 0) || (move && speed == 0)) {
    safe_stop(); fault_latched = true;
    acknowledge(sequence, false, "UNKNOWN", "BAD_COMMAND"); return;
  }
  if (stop) {  // STOP remains effective even if sequence is duplicated or old.
    safe_stop();
    if (newer(sequence)) { last_sequence = sequence; has_sequence = true; }
    acknowledge(sequence, true, direction, "STOPPED"); return;
  }
  if (!newer(sequence)) {
    safe_stop(); fault_latched = true;
    acknowledge(sequence, false, direction, "STALE"); return;
  }
  last_sequence = sequence; has_sequence = true;
  if (reset) {
    safe_stop(); fault_latched = false;
    acknowledge(sequence, true, direction, "RESET"); return;
  }
  if (fault_latched) {
    safe_stop(); acknowledge(sequence, false, direction, "FAULT"); return;
  }
  if (arm) {
    armed = true; last_motion_ms = millis();
    acknowledge(sequence, true, direction, "ARMED"); return;
  }
  if (!armed) {
    safe_stop(); acknowledge(sequence, false, direction, "DISARMED"); return;
  }
  if (speed > MAX_SPEED_PERMILLE) {
    safe_stop(); fault_latched = true;
    acknowledge(sequence, false, direction, "BAD_SPEED"); return;
  }
  if (!motor.apply(direction, static_cast<uint16_t>(speed))) {
    safe_stop(); fault_latched = true;
    acknowledge(sequence, false, direction, "NO_DRIVER"); return;
  }
  last_motion_ms = millis();
  acknowledge(sequence, true, direction, "ACCEPTED");
}

void setup() {
  motor.stop();  // Motors must be stopped before receiving any serial bytes.
  Serial.begin(115200);
}

void loop() {
  if (armed && millis() - last_motion_ms > WATCHDOG_MS) {
    safe_stop(); fault_latched = true;
  }
  while (Serial.available() > 0) {
    int byte_value = Serial.read();
    if (byte_value < 0) break;
    char byte = static_cast<char>(byte_value);
    if (discarding) {
      if (byte == '\n') discarding = false;
      continue;
    }
    if (byte == '\n') {
      line_buffer[line_length] = '\0';
      if (line_length && line_buffer[line_length - 1] == '\r')
        line_buffer[line_length - 1] = '\0';
      process_line(line_buffer);
      line_length = 0;
    } else if (line_length >= 95) {
      line_length = 0; discarding = true;
      safe_stop(); fault_latched = true;
      acknowledge(0, false, "UNKNOWN", "BAD_FRAME");
    } else {
      line_buffer[line_length++] = byte;
    }
  }
}
