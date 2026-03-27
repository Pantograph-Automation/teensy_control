#pragma once

class SerialInterface
{
public:
  virtual ~SerialInterface() = default;
  virtual void begin(unsigned long baud_rate) = 0;
  virtual int available() = 0;
  virtual char read() = 0;
  virtual void println(const char * message) = 0;
};
