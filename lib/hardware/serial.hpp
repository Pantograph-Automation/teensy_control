#pragma once

#include "serial_interface.hpp"

#if defined(ARDUINO)
#include "Arduino.h"

class HardwareSerialAdapter : public SerialInterface
{
public:
  inline void begin(unsigned long baud_rate) override
  {
    Serial.begin(baud_rate);
  }

  inline int available() override
  {
    return Serial.available();
  }

  inline char read() override
  {
    return static_cast<char>(Serial.read());
  }

  inline void println(const char * message) override
  {
    Serial.println(message);
  }
};
#endif
