#pragma once
#include <deque>
#include <string>

extern unsigned long fake_millis;
inline unsigned long millis() { return fake_millis; }

class FakeSerial {
 public:
  std::deque<char> input;
  std::string output;
  void begin(unsigned long) {}
  int available() const { return static_cast<int>(input.size()); }
  int read() {
    if (input.empty()) return -1;
    char next = input.front(); input.pop_front();
    return static_cast<unsigned char>(next);
  }
  void print(const char *text) { output += text; }
  void print(char value) { output += value; }
  void println(const char *text) { output += text; output += "\r\n"; }
  void feed(const std::string &text) {
    for (char byte : text) input.push_back(byte);
  }
};

extern FakeSerial Serial;
