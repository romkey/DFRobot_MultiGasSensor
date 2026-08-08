#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

typedef uint8_t byte;

inline void delay(unsigned long) {}
inline unsigned long millis() { return 0; }

class String
{
public:
  String() : _value("") {}
  String(const char *value) : _value(value ? value : "") {}
  String(const String &other) : _value(other._value) {}

  const char *c_str() const { return _value.c_str(); }
  bool operator==(const char *other) const { return _value == other; }

private:
  std::string _value;
};

class __FlashStringHelper;

class Print
{
public:
  virtual ~Print() {}
  virtual size_t print(const char *) { return 0; }
  virtual size_t print(int, int = 10) { return 0; }
  virtual size_t println(const char *) { return 0; }
  virtual size_t println(int, int = 10) { return 0; }
  size_t println() { return 0; }
};

class HardwareSerial : public Print
{
public:
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  size_t write(const uint8_t *, size_t len) { return len; }
};

extern HardwareSerial Serial;
